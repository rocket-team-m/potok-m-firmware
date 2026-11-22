// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host benchmark for the software-in-the-loop harness.
//!
//! Like `loop_bench.rs`, this reports a reproducible replay summary rather
//! than wall-clock time: a host timer is noise-dominated and a fixed script is
//! the thing being measured. The script drives the real `step`, phase FSM and
//! interlock through `MockBoard` at the emulated 500 Hz cadence.
//!
//! With Cargo's default libtest harness, `cargo test --test sil_bench --
//! --nocapture` prints the report. Adding `harness = false` for this target in
//! `firmware/Cargo.toml` (parent-owned) makes `cargo bench` print it via
//! `main()` instead; both paths share `report()`.

pub use potok_m_firmware::{board, control, interlocks, phase, protocol, Command};

#[path = "../src/sil.rs"]
mod sil;

use std::fmt::Write;

use potok_m_firmware::{interlocks::Refusal, mock::MockBoard, PhaseEvent, StepResult};
use potok_protocol::{flags, Phase, REPLY_SIZE};
use sil::{simulate, Frame, HarnessBoard, Tick, Trace, TraceEvent, TICK_HZ, TICK_PERIOD_US};

/// A fixed flight profile: boost, burnout, a command ramp, a timeout, a bad
/// frame, apogee/descent/touchdown. No RNG, no clock, no external input.
fn script() -> Vec<Tick> {
    let mut ticks = Vec::with_capacity(28);
    for _ in 0..4 {
        ticks.push(Tick::reply(60, -60));
    }
    ticks.push(Tick::reply(60, -60).event(PhaseEvent::LiftOff));
    ticks.push(Tick::reply(60, -60).event(PhaseEvent::Burnout));
    for i in 0..12 {
        ticks.push(Tick::reply((i * 10) as i16, -(i as i16 * 5)));
    }
    ticks.push(Tick::silent());
    ticks.push(Tick::corrupt(80, 80, 12, 0x02));
    ticks.push(Tick::stale(80, 80));
    ticks.push(
        Tick::reply(80, -80)
            .flags(flags::IMU_OK)
            .phase_code(Phase::COAST as u8),
    );
    ticks.push(Tick::reply(80, -80).incomplete());
    ticks.push(Tick::reply(80, -80).armed(true));
    ticks.push(Tick::unhealthy(80, 80));
    ticks.push(Tick {
        frame: Frame::Raw {
            bytes: [0u8; REPLY_SIZE],
            len: 3,
        },
        ..Tick::reply(0, 0)
    });
    ticks.push(Tick::reply(80, -80).event(PhaseEvent::Apogee));
    ticks.push(Tick::reply(80, -80).event(PhaseEvent::Descent));
    ticks.push(Tick::reply(80, -80).event(PhaseEvent::Touchdown));
    ticks
}

fn report() -> String {
    let ticks = script();
    let mut trace = Trace::<128>::new();
    let mut sim = std::hint::black_box(simulate(MockBoard::new(), &ticks, &mut trace));

    if sim.gate().latched().is_some() {
        sim.reset_gate(&mut trace);
    }
    assert_eq!(trace.len(), ticks.len());

    let records = trace.records();
    let count = |result: StepResult| {
        records
            .iter()
            .filter(|r| core::mem::discriminant(&r.result) == core::mem::discriminant(&result))
            .count()
    };
    let applied_changes = records
        .windows(2)
        .filter(|w| w[0].applied != w[1].applied)
        .count();
    let phase_transitions = trace
        .events()
        .iter()
        .filter(|e| matches!(e, TraceEvent::Phase { .. }))
        .count();
    let interlock_trips = trace
        .events()
        .iter()
        .filter(|e| matches!(e, TraceEvent::InterlockLatched { .. }))
        .count();
    let checksum = records.iter().fold(0xcbf29ce484222325u64, |acc, r| {
        acc.wrapping_mul(0x100000001b3)
            .wrapping_add(r.applied.pitch as u16 as u64)
            .wrapping_add((r.applied.yaw as u16 as u64) << 16)
    });

    assert_eq!(records.len(), ticks.len());
    assert_eq!(
        count(StepResult::Idle)
            + count(StepResult::FrameRejected)
            + count(StepResult::Accepted(Command::ZERO)),
        ticks.len()
    );
    assert!(records.iter().all(|r| r.now_us == r.tick * TICK_PERIOD_US));
    assert_eq!(sim.inner_board().sent_frames().len(), ticks.len());
    assert_eq!(sim.inner_board().last_applied(), Command::ZERO);
    assert!(interlock_trips <= 1, "the fixed script faults at most once");
    assert!(records
        .iter()
        .all(|r| r.interlock_refusal != Some(Refusal::Implausible)));

    let mut out = String::new();
    let _ = writeln!(out, "Potok-M SIL harness — deterministic replay");
    let _ = writeln!(
        out,
        "rate={TICK_HZ} Hz  period={TICK_PERIOD_US} us  ticks={}  window_ms={}",
        ticks.len(),
        ticks.len() as u64 * TICK_PERIOD_US as u64 / 1_000
    );
    let _ = writeln!(
        out,
        "accepted={} idle={} rejected={} applied_changes={} phase_transitions={} interlock_trips={} requests={} pets={}",
        count(StepResult::Accepted(Command::ZERO)),
        count(StepResult::Idle),
        count(StepResult::FrameRejected),
        applied_changes,
        phase_transitions,
        interlock_trips,
        sim.inner_board().sent_frames().len(),
        sim.inner_board().pet_count()
    );
    let _ = writeln!(out, "checksum=0x{checksum:016x}");
    out
}

#[test]
fn bench_report_is_deterministic() {
    let text = report();
    assert_eq!(
        text,
        report(),
        "the replay must not depend on wall-clock state"
    );
    assert!(text.contains("rate=500 Hz"), "{text}");
    assert!(text.contains("period=2000 us"), "{text}");
    assert!(text.contains("phase_transitions=4"), "{text}");
    print!("{text}");
}

fn main() {
    print!("{}", report());
}
