// SPDX-License-Identifier: GPL-3.0-or-later
//! 500 Hz timing budget from the `perf` cycle model and SIL worst cases.
//!
//! `lib.rs` is owned by the parent task, so this module is included by path in
//! the host tests. It depends only on `core`, [`potok_m_firmware::perf`],
//! [`potok_m_firmware::scheduler`] and the protocol crate, and builds unchanged inside the
//! `no_std` firmware library.
//!
//! The SIL scenarios below are synthetic upper bounds on the work a single
//! control tick can be asked to do by the simulator's frame stream: a full
//! double-buffer RX, every scaled field, an open gate and both command axes.
//! They are *not* wall-clock measurements. Every number comes from
//! [`potok_m_firmware::perf::CycleEstimate`], the same model the benchmark and the
//! hardware cycle counter are meant to check.
//!
//! An overrun is LOUD: [`BudgetReport::evaluate`] panics with the scenario
//! name, the modelled cycles and the 500 Hz budget; the worst-case constant is
//! additionally checked at compile time.

use potok_m_firmware::{
    perf::{CycleEstimate, CORE_HZ, CYCLE_BUDGET, CYCLE_BUDGET_US},
    scheduler::TICK_HZ,
};
use potok_protocol::{REPLY_SIZE, REQUEST_SIZE};

/// Core cycles in one microsecond at [`CORE_HZ`].
pub const CYCLES_PER_US: u32 = CORE_HZ / 1_000_000;
/// Microseconds in one 500 Hz tick.
pub const TICK_US: u32 = 1_000_000 / TICK_HZ;
/// Core cycles available in one tick before any deadline is missed.
pub const TICK_CYCLES: u32 = CORE_HZ / TICK_HZ;

const _: () = assert!(TICK_HZ == 500, "timing budget is defined at 500 Hz");
const _: () = assert!(TICK_US == 2_000);
const _: () = assert!(CYCLES_PER_US * TICK_US == TICK_CYCLES);
/// The firmware allowance may never consume more than 5% of a tick.
const _: () = assert!(CYCLE_BUDGET_US * TICK_HZ <= 50_000);

/// Scaled fields in a compat request: gyro/accel/mag (9), height, pitot.
pub const SIL_MAX_QUANT_FIELDS: u32 = 11;
/// Command axes in one tick: pitch and yaw.
pub const SIL_MAX_AXES: u32 = 2;
/// The RX double buffer can hand a tick two complete reply frames.
pub const SIL_MAX_RX_BYTES: u32 = 2 * REPLY_SIZE as u32;
/// CRC work in the worst tick: one request plus two replies.
pub const SIL_MAX_CRC_BYTES: u32 = REQUEST_SIZE as u32 + SIL_MAX_RX_BYTES;

/// Inputs to one modelled tick. Numbers are byte/field counts, never timings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Scenario {
    pub name: &'static str,
    pub rx_bytes: u32,
    pub crc_bytes: u32,
    pub quant_fields: u32,
    pub axes: u32,
    pub gate: bool,
}

impl Scenario {
    pub const fn new(
        name: &'static str,
        rx_bytes: u32,
        crc_bytes: u32,
        quant_fields: u32,
        axes: u32,
        gate: bool,
    ) -> Self {
        Self {
            name,
            rx_bytes,
            crc_bytes,
            quant_fields,
            axes,
            gate,
        }
    }

    /// The exact [`CycleEstimate`] this scenario contributes to one tick.
    pub const fn estimate(self) -> CycleEstimate {
        CycleEstimate::new(
            self.rx_bytes,
            self.crc_bytes,
            self.quant_fields,
            self.axes,
            self.gate,
        )
    }

    pub const fn cycles(self) -> u32 {
        self.estimate().cycles()
    }
}

/// No complete reply: the request is still encoded and CRC'd.
pub const SIL_IDLE: Scenario = Scenario::new(
    "SIL_IDLE",
    0,
    REQUEST_SIZE as u32,
    SIL_MAX_QUANT_FIELDS,
    0,
    false,
);
/// A full reply that fails validation: scan, CRC, gate, no command math.
pub const SIL_REJECTED: Scenario = Scenario::new(
    "SIL_REJECTED",
    REPLY_SIZE as u32,
    (REQUEST_SIZE + REPLY_SIZE) as u32,
    SIL_MAX_QUANT_FIELDS,
    0,
    true,
);
/// A valid reply under the AI gate: the nominal committed command.
pub const SIL_ACCEPTED: Scenario = Scenario::new(
    "SIL_ACCEPTED",
    REPLY_SIZE as u32,
    (REQUEST_SIZE + REPLY_SIZE) as u32,
    SIL_MAX_QUANT_FIELDS,
    SIL_MAX_AXES,
    true,
);
/// Worst SIL case: RX double buffer full, every field, open gate, both axes.
pub const SIL_FRAME_FLOOD: Scenario = Scenario::new(
    "SIL_FRAME_FLOOD",
    SIL_MAX_RX_BYTES,
    SIL_MAX_CRC_BYTES,
    SIL_MAX_QUANT_FIELDS,
    SIL_MAX_AXES,
    true,
);

/// All scenarios the budget must cover, in increasing cost.
pub const SCENARIOS: [Scenario; 4] = [SIL_IDLE, SIL_REJECTED, SIL_ACCEPTED, SIL_FRAME_FLOOD];

/// Compile-time refusal to ship a model whose worst case already busts 50 us.
pub const WORST_CASE_CYCLES: u32 = SIL_FRAME_FLOOD.cycles();
const _: () = assert!(
    WORST_CASE_CYCLES <= CYCLE_BUDGET,
    "SIL worst case exceeds budget"
);

/// Deterministic accounting for one scenario against the 500 Hz budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BudgetReport {
    pub scenario: Scenario,
    pub cycles: u32,
    pub budget: u32,
    /// Unused cycles at the firmware allowance.
    pub margin_cycles: u32,
    /// Margin in whole microseconds (fractional microseconds truncate).
    pub margin_us: u32,
    pub tick_us: u32,
    pub tick_cycles: u32,
    /// Budget spent as parts per million of the full tick.
    pub tick_utilization_ppm: u32,
}

impl BudgetReport {
    /// Account one scenario, refusing loudly if it does not fit the budget.
    pub fn evaluate(scenario: Scenario) -> Self {
        let cycles = scenario.cycles();
        assert!(
            cycles <= CYCLE_BUDGET,
            "500 Hz timing budget exceeded: {} = {} cycles, budget {} cycles ({} us at {} Hz)",
            scenario.name,
            cycles,
            CYCLE_BUDGET,
            CYCLE_BUDGET_US,
            TICK_HZ
        );
        let margin_cycles = CYCLE_BUDGET - cycles;
        Self {
            scenario,
            cycles,
            budget: CYCLE_BUDGET,
            margin_cycles,
            margin_us: margin_cycles / CYCLES_PER_US,
            tick_us: TICK_US,
            tick_cycles: TICK_CYCLES,
            tick_utilization_ppm: cycles * 1_000_000 / TICK_CYCLES,
        }
    }

    /// The modelled worst case, with the same LOUD refusal as any scenario.
    pub fn worst_case() -> Self {
        Self::evaluate(SIL_FRAME_FLOOD)
    }
}

/// Check every SIL scenario and return the costliest report. Any overrun
/// panics with the offending scenario's name before a tick can be scheduled.
pub fn assert_tick_budget() -> BudgetReport {
    let mut worst = BudgetReport::evaluate(SCENARIOS[0]);
    for &scenario in &SCENARIOS[1..] {
        let report = BudgetReport::evaluate(scenario);
        if report.cycles > worst.cycles {
            worst = report;
        }
    }
    assert_eq!(
        worst.scenario, SIL_FRAME_FLOOD,
        "SIL_FRAME_FLOOD must dominate the declared scenario set"
    );
    worst
}
