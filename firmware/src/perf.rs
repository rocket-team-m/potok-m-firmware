// SPDX-License-Identifier: GPL-3.0-or-later
//! Branch-light fast paths and a deterministic cycle model for the 500 Hz loop.
//!
//! `lib.rs` is owned by the parent task, so this module is included by path in
//! tests and in the benchmark. It depends only on `core` and the protocol
//! crate, and builds unchanged inside the `no_std` firmware library.
//!
//! Scope: the arithmetic and gating *after* a reply has been decoded. CRC,
//! packing and scaling stay in `potok-protocol`; safety state stays in
//! `control.rs`. Nothing here grants actuation authority — callers still run
//! `LinkState::command` gates before applying the result.
//!
//! Two kinds of "cycles" appear:
//! - [`CycleEstimate`] is a deterministic instruction-count model used by the
//!   host tests and by `benches/loop_bench.rs`. It is host-reproducible, which
//!   wall-clock timing is not, and it is what the budget assertion checks.
//! - [`read_cycles`] reads the Cortex-M7 DWT cycle counter on target, so the
//!   model can be checked against hardware once the pins are wired.

use potok_protocol::{flags, Phase};

use crate::{Command, MAX_CORRECTION_RAW};

/// STM32H743 core clock at its rated maximum. The budget is quoted at this
/// rate; a slower clock shortens the available cycles proportionally.
pub const CORE_HZ: u32 = 480_000_000;
/// One tick of the existing control loop (`control.rs`), in microseconds.
pub const TICK_PERIOD_US: u32 = 2_000;
/// Firmware-side allowance for one tick: 50 us at [`CORE_HZ`], 2.5% of a tick.
/// The rest stays free for the codec, DMA, sensor drivers and scheduling.
pub const CYCLE_BUDGET_US: u32 = 50;
/// Hard upper bound on the modelled hot path, in core cycles.
pub const CYCLE_BUDGET: u32 = CYCLE_BUDGET_US * (CORE_HZ / 1_000_000);

// Instruction costs used by the model. They are conservative upper bounds for
// a Cortex-M7 (one table load, a handful of ALU ops per byte), not measurements
// of the host. Change them only together with the pinned numbers in the tests.
const RX_BYTE_CYCLES: u32 = 6;
const CRC_BYTE_CYCLES: u32 = 4;
const QUANT_FIELD_CYCLES: u32 = 12;
const GATE_CYCLES: u32 = 12;
const FIXED_AXIS_CYCLES: u32 = 4;
const FRAME_OVERHEAD_CYCLES: u32 = 200;

/// Precomputed AI gate, indexed directly by the wire phase byte. A 256-entry
/// table costs 256 bytes of flash and removes a bounds branch from the hot
/// path: every value outside `0..=4` fails closed.
const fn build_phase_gate() -> [bool; 256] {
    let mut table = [false; 256];
    let mut phase = Phase::BURN as usize + 1;
    while phase <= Phase::LANDED as usize {
        table[phase] = true;
        phase += 1;
    }
    table
}

/// True only for phases in which active stabilisation is permitted. BURN is
/// false; so is every undefined phase value.
pub const AI_ALLOWED_BY_PHASE: [bool; 256] = build_phase_gate();

/// One table load, three bit ops and one compare: no short-circuit chain in the
/// decision itself. `flags` is the *outgoing* request flag word, so the motor
/// bit comes from the board, never from the AI reply.
#[inline(always)]
pub fn ai_gate(phase: Phase, flags: u8, disabled: bool, deadman_tripped: bool) -> bool {
    let allowed = AI_ALLOWED_BY_PHASE[phase as usize] as u8;
    let motor_clear = ((flags & flags::MOTOR_ARMED) == 0) as u8;
    let enabled = (!disabled) as u8;
    let fresh = (!deadman_tripped) as u8;
    (allowed & motor_clear & enabled & fresh) != 0
}

/// Q15 gain in an `i32`: `Q15::ONE` is 1.0. One widening multiply and a shift
/// cover every `i16` input; no division, float conversion or lookup is needed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Q15(i32);

impl Q15 {
    pub const ZERO: Self = Self(0);
    pub const ONE: Self = Self(1 << 15);
    pub const HALF: Self = Self(1 << 14);

    /// Build a gain from an integer ratio without floating point. A zero
    /// denominator or a gain above 2.0 is a configuration error and is refused
    /// loudly, so the gain stays inside the Q15 range.
    pub fn from_ratio(num: i32, den: i32) -> Self {
        assert!(den > 0, "Q15 denominator must be positive");
        let raw = (num as i64 * (Self::ONE.0 as i64)) / den as i64;
        assert!(
            (-(1 << 16)..=1 << 16).contains(&raw),
            "Q15 gain out of range"
        );
        Self(raw as i32)
    }

    /// Scale one raw value, rounding half away from zero symmetrically.
    #[inline(always)]
    pub fn apply(self, value: i16) -> i16 {
        let product = self.0 as i64 * value as i64;
        let half = 1i64 << 14;
        let rounded = if product >= 0 {
            (product + half) >> 15
        } else {
            -((-product + half) >> 15)
        };
        rounded.clamp(i16::MIN as i64, i16::MAX as i64) as i16
    }
}

/// Saturate to the firmware's raw command bound. Bit-for-bit equal to
/// `Command::clamped` on each axis; use it after the safety gates, not before.
#[inline(always)]
pub fn clamp_raw(value: i32) -> i16 {
    value.clamp(-(MAX_CORRECTION_RAW as i32), MAX_CORRECTION_RAW as i32) as i16
}

/// Fixed-point command scaling: one multiply and one clamp per axis, no
/// division and no float conversion in the tick.
#[inline(always)]
pub fn scale_command(pitch: i16, yaw: i16, gain: Q15) -> Command {
    Command {
        pitch: clamp_raw(gain.apply(pitch) as i32),
        yaw: clamp_raw(gain.apply(yaw) as i32),
    }
}

/// Deterministic per-tick instruction accounting. Each field is a count of
/// operations multiplied by the fixed per-operation cost above.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CycleEstimate {
    pub rx_scan: u32,
    pub crc: u32,
    pub quantize: u32,
    pub gate: u32,
    pub fixed_math: u32,
    pub overhead: u32,
}

impl CycleEstimate {
    /// Cost of one tick with the given byte and field counts.
    pub const fn new(
        rx_bytes: u32,
        crc_bytes: u32,
        quant_fields: u32,
        axes: u32,
        gate: bool,
    ) -> Self {
        Self {
            rx_scan: rx_bytes * RX_BYTE_CYCLES,
            crc: crc_bytes * CRC_BYTE_CYCLES,
            quantize: quant_fields * QUANT_FIELD_CYCLES,
            gate: if gate { GATE_CYCLES } else { 0 },
            fixed_math: axes * FIXED_AXIS_CYCLES,
            overhead: FRAME_OVERHEAD_CYCLES,
        }
    }

    pub const fn cycles(self) -> u32 {
        self.rx_scan + self.crc + self.quantize + self.gate + self.fixed_math + self.overhead
    }

    /// Panics with the offending number when the hot path outgrows its budget.
    pub fn assert_budget(self, label: &str) {
        let cycles = self.cycles();
        assert!(
            cycles <= CYCLE_BUDGET,
            "cycle budget exceeded: {label} = {cycles} > {CYCLE_BUDGET}"
        );
    }
}

/// Enable the DWT cycle counter on target. Call once at startup; reading it
/// before enabling returns zero.
#[cfg(target_os = "none")]
pub fn init_cycle_counter() {
    let mut peripherals = unsafe { cortex_m::Peripherals::steal() };
    peripherals.DCB.enable_trace();
    peripherals.DWT.enable_cycle_counter();
}

/// Raw CYCCNT snapshot. On a 480 MHz core, one tick is 960_000 counts.
#[cfg(target_os = "none")]
#[inline(always)]
pub fn read_cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}
