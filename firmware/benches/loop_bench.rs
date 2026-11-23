// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host benchmark for the shipped `control::step` tick.
//!
//! Reports modelled core cycles per tick, not wall-clock time: a host timer is
//! noise-dominated at this scale and would not be reproducible across runs or
//! machines. Unlike the old benchmark, this one runs the **real** production
//! path (`control::step` with its link tick, independent interlock/shield and
//! watchdog policy) over a scripted frame stream. The cycle inputs are call
//! counts observed at the board boundary of those runs; the costs are the
//! source-derived constants in `perf.rs`. No `ai_gate`/`scale_command`/Q15/
//! quantize call is made or charged: production calls none of them.
//!
//! With Cargo's default libtest harness, `cargo test --test loop_bench --
//! --nocapture` prints the report. Adding `harness = false` for this target in
//! `firmware/Cargo.toml` (parent-owned) makes `cargo bench` print it via
//! `main()` instead; both paths share the same `report()`.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

// The shared module also carries the legacy SIL reservation model and the
// gate/fast-path helpers, which this benchmark deliberately does not exercise.
#[allow(dead_code)]
#[path = "../src/perf.rs"]
mod perf;

use std::fmt::Write;

use perf::{
    StepCounts, StepCycles, CORE_HZ, CYCLE_BUDGET, CYCLE_BUDGET_US, REPLY_CRC_BYTES,
    REQUEST_CRC_BYTES, TICK_PERIOD_US,
};
use potok_m_firmware::{
    control::step, interlocks::Interlocks, mock::MockBoard, protocol::REPLY_SIZE,
    watchdog::WatchdogPolicy, LinkState, PhaseEvent, PhaseFsm, ReplyFrame, StepResult,
};
use potok_protocol::flags;

/// Fixed, host-independent iteration count. Each tick starts from fresh state
/// so a rejected frame cannot latch the three-strike counter and change the
/// next tick's path: the model prices one steady-state tick, not a failure run.
const TICKS: u32 = 10_000;

#[derive(Clone, Copy, Debug)]
enum Stream {
    Idle,
    Rejected,
    Accepted,
}

impl Stream {
    const fn name(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Rejected => "rejected",
            Self::Accepted => "accepted",
        }
    }
}

fn reply(seq: u16) -> ReplyFrame {
    ReplyFrame {
        seq,
        g_pitch: 50,
        g_yaw: -50,
        confidence: 200,
        ai_healthy: 1,
        version: 1,
    }
}

/// Run one real production tick and return what the board boundary observed.
/// `COAST` is selected so an accepted reply exercises the full gate/clamp path
/// instead of being vetoed by the initial `BURN` phase.
fn tick(stream: Stream, now_us: u32) -> (StepResult, MockBoard) {
    let mut board = MockBoard::new();
    board.sensor_flags = flags::IMU_OK;
    board.now_us = now_us;
    let mut fsm = PhaseFsm::new();
    fsm.apply(PhaseEvent::Burnout).unwrap();
    let mut link = LinkState::new();
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();

    match stream {
        Stream::Idle => {}
        Stream::Rejected => {
            // Well-formed frame with a stale seq: `on_reply` rejects it, so the
            // shield and watchdog still run but `command`/`clamped` do not.
            board.queue_reply(&reply(1));
        }
        Stream::Accepted => {
            // Fresh LinkState issues seq 0 on its first request.
            board.queue_reply(&reply(0));
        }
    }

    let result = step(
        &mut board,
        &mut fsm,
        &mut link,
        &mut interlock,
        &mut policy,
        true,
    );
    (result, board)
}

/// Count the calls the shipped tick actually made, then turn those observations
/// into the per-tick inputs of [`StepCycles`].
fn measure(stream: Stream) -> (StepCounts, u32) {
    let mut idle = 0u32;
    let mut rejected = 0u32;
    let mut accepted = 0u32;
    let mut frames = 0u32;
    let mut sent = 0u32;
    let mut applied = 0u32;
    let mut pets = 0u32;
    let mut sink = 0u32;

    for tick_index in 0..TICKS {
        let (result, board) = tick(stream, tick_index * TICK_PERIOD_US);
        match std::hint::black_box(result) {
            StepResult::Idle => idle += 1,
            StepResult::FrameRejected => rejected += 1,
            StepResult::Accepted(_) => accepted += 1,
        }
        frames += board.rx().swaps as u32;
        sent += board.sent_frames().len() as u32;
        applied += board.applied.len() as u32;
        pets += board.pets();
        for cmd in &board.applied {
            sink = sink
                .wrapping_add(cmd.pitch as u32)
                .wrapping_add(cmd.yaw as u32);
        }
    }

    // The model inputs must be tied to calls that really happened. If a future
    // edit to `step` adds or drops one, these assertions move with it.
    assert_eq!(idle + rejected + accepted, TICKS);
    assert_eq!(
        match stream {
            Stream::Idle => idle,
            Stream::Rejected => rejected,
            Stream::Accepted => accepted,
        },
        TICKS,
        "{stream:?}: every tick must take the intended path"
    );
    assert_eq!(sent, TICKS, "{:?}: encode_request once per tick", stream);
    assert_eq!(applied, TICKS, "{:?}: apply_command once per tick", stream);
    assert_eq!(pets, TICKS, "{:?}: watchdog fed once per tick", stream);
    assert_eq!(
        frames,
        accepted + rejected,
        "{:?}: one assembler frame per decoded tick",
        stream
    );
    assert_eq!(frames % TICKS, 0);
    assert_eq!(accepted % TICKS, 0);

    let frames = frames / TICKS;
    let accepted = accepted / TICKS;
    let counts = StepCounts {
        rx_bytes: REPLY_SIZE as u32 * frames,
        crc_bytes: REQUEST_CRC_BYTES + REPLY_CRC_BYTES * frames,
        phase_gate_calls: accepted,
        shield_calls: 1,
        watchdog_calls: 1,
        clamped_axes: 2 * accepted,
    };
    (counts, std::hint::black_box(sink))
}

fn scenario(name: &str, counts: StepCounts) -> String {
    let est = StepCycles::new(counts);
    est.assert_budget(name);
    format!(
        "{name:<9} rx={:<3} crc={:<3} gate={} shield={} wd={} clamp={}  {:>6} cycles/tick  headroom {}\n",
        est.rx_scan / 6,
        est.crc / 4,
        est.phase_gate / 12,
        est.shield / perf::SHIELD_CYCLES,
        est.watchdog / perf::WATCHDOG_CYCLES,
        est.command_clamp / 4,
        est.cycles(),
        CYCLE_BUDGET - est.cycles(),
    )
}

fn report() -> (String, u32) {
    let (idle, _) = measure(Stream::Idle);
    let (rejected, _) = measure(Stream::Rejected);
    let (accepted, sink) = measure(Stream::Accepted);

    let mut out = String::new();
    let _ = writeln!(
        out,
        "Potok-M control loop — shipped step() cycle model ({TICKS} ticks/tick, sink={sink})"
    );
    let _ = writeln!(
        out,
        "core={CORE_HZ} Hz  tick={TICK_PERIOD_US} us  budget={CYCLE_BUDGET} cycles ({CYCLE_BUDGET_US} us)"
    );
    let _ = write!(out, "{}", scenario(Stream::Idle.name(), idle));
    let _ = write!(out, "{}", scenario(Stream::Rejected.name(), rejected));
    let _ = write!(out, "{}", scenario(Stream::Accepted.name(), accepted));
    (out, sink)
}

#[cfg(test)]
#[test]
fn bench_report_is_deterministic() {
    let (text, sink) = report();
    assert!(text.contains("budget=24000 cycles (50 us)"), "{text}");
    assert!(text.contains("idle "), "{text}");
    assert!(text.contains("rejected"), "{text}");
    assert!(text.contains("accepted "), "{text}");
    assert!(text.contains("456 cycles/tick"), "{text}");
    assert!(text.contains("588 cycles/tick"), "{text}");
    assert!(text.contains("608 cycles/tick"), "{text}");
    assert_eq!(sink, 0, "symmetric pitch/yaw commands cancel");
    print!("{text}");
}

fn main() {
    print!("{}", report().0);
}
