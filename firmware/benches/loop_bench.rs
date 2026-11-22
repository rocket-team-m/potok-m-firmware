// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host benchmark for one control tick.
//!
//! Reports modelled core cycles per tick, not wall-clock time: a host timer is
//! noise-dominated at this scale and would not be reproducible across runs or
//! machines. The model's inputs are the real frame sizes and field counts, and
//! the same fixed input stream is executed through the fast paths on every
//! run. The on-target counterpart is `perf::read_cycles`.
//!
//! With Cargo's default libtest harness, `cargo test --test loop_bench --
//! --nocapture` prints the report. Adding `harness = false` for this target in
//! `firmware/Cargo.toml` (parent-owned) makes `cargo bench` print it via
//! `main()` instead; both paths share the same `report()`.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/perf.rs"]
mod perf;

use std::fmt::Write;

use perf::{
    ai_gate, scale_command, CycleEstimate, CORE_HZ, CYCLE_BUDGET, CYCLE_BUDGET_US, Q15,
    TICK_PERIOD_US,
};
use potok_m_firmware::phase::{PhaseEvent, PhaseFsm};
use potok_protocol::{flags, REPLY_SIZE, REQUEST_SIZE};

/// Scaled fields in a compat request: gyro/accel/mag (9), height, pitot.
const QUANT_FIELDS: u32 = 11;
/// Fixed, host-independent iteration count.
const TICKS: u32 = 10_000;

fn scenario(name: &str, est: CycleEstimate) -> String {
    est.assert_budget(name);
    format!(
        "{name:<9} rx={:<3} crc={:<3} fields={:<2} axes={}  {:>6} cycles/tick  headroom {}\n",
        est.rx_scan / 6,
        est.crc / 4,
        est.quantize / 12,
        est.fixed_math / 4,
        est.cycles(),
        CYCLE_BUDGET - est.cycles(),
    )
}

/// Run the real fast paths over a fixed input stream so the compiler cannot
/// drop them; the returned sink is unrelated to the modelled cycle count.
fn exercise(fsm: &PhaseFsm, flags: u8) -> u32 {
    // Rotate through the fixed-point gains so every fast path is hot.
    let gains = [Q15::ZERO, Q15::ONE, Q15::HALF, Q15::from_ratio(1, 3)];
    let mut sink = 0u32;
    for tick in 0..TICKS {
        let allowed = ai_gate(fsm.phase(), std::hint::black_box(flags), false, false);
        let cmd = scale_command(
            std::hint::black_box((tick as i16).wrapping_mul(37)),
            std::hint::black_box((tick as i16).wrapping_mul(-19)),
            gains[(tick % gains.len() as u32) as usize],
        );
        sink = sink
            .wrapping_add(allowed as u32)
            .wrapping_add(cmd.pitch as u32)
            .wrapping_add(cmd.yaw as u32);
    }
    std::hint::black_box(sink)
}

fn report() -> String {
    let mut fsm = PhaseFsm::new();
    fsm.apply(PhaseEvent::Burnout).unwrap();
    let sensor_flags = flags::IMU_OK;
    let sink = exercise(&fsm, sensor_flags);

    let mut out = String::new();
    let _ = writeln!(
        out,
        "Potok-M control loop — deterministic cycle model ({TICKS} ticks, sink={sink})"
    );
    let _ = writeln!(
        out,
        "core={CORE_HZ} Hz  tick={TICK_PERIOD_US} us  budget={CYCLE_BUDGET} cycles ({CYCLE_BUDGET_US} us)"
    );

    let idle = CycleEstimate::new(0, REQUEST_SIZE as u32, QUANT_FIELDS, 0, false);
    let rejected = CycleEstimate::new(
        REPLY_SIZE as u32,
        (REQUEST_SIZE + REPLY_SIZE) as u32,
        QUANT_FIELDS,
        0,
        true,
    );
    let accepted = CycleEstimate::new(
        REPLY_SIZE as u32,
        (REQUEST_SIZE + REPLY_SIZE) as u32,
        QUANT_FIELDS,
        2,
        true,
    );
    let _ = write!(out, "{}", scenario("idle", idle));
    let _ = write!(out, "{}", scenario("rejected", rejected));
    let _ = write!(out, "{}", scenario("accepted", accepted));
    out
}

#[cfg(test)]
#[test]
fn bench_report_is_deterministic() {
    let text = report();
    assert!(text.contains("budget=24000 cycles (50 us)"), "{text}");
    assert!(text.contains("idle "), "{text}");
    assert!(text.contains("rejected"), "{text}");
    assert!(text.contains("accepted "), "{text}");
    assert!(text.contains("716 cycles/tick"), "{text}");
    print!("{text}");
}

fn main() {
    print!("{}", report());
}
