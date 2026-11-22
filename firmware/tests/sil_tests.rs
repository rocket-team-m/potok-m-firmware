// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the software-in-the-loop harness.
//!
//! `sil.rs` is included by path: it uses `crate::` paths so the same source
//! compiles inside the firmware library and here. These re-exports keep those
//! paths valid in this integration-test crate. Every scenario below is a fixed
//! script of [`Tick`]s driven through `MockBoard` — no pins, no wall clock, no
//! randomness.

pub use potok_m_firmware::{board, control, interlocks, phase, protocol, Command};

#[path = "../src/sil.rs"]
mod sil;

use potok_m_firmware::{interlocks::Refusal, mock::MockBoard, PhaseEvent, StepResult};
use potok_protocol::{decode_request, flags, Phase, REPLY_SIZE};
use sil::{
    simulate, Frame, HarnessBoard, Simulator, Tick, Trace, TraceEvent, TICK_HZ, TICK_PERIOD_US,
};

#[test]
fn script_runs_at_500hz_with_strict_request_sequencing() {
    assert_eq!(TICK_HZ, 500);
    assert_eq!(TICK_PERIOD_US, 2_000);

    let script = [
        Tick::silent(),
        Tick::reply(10, -10),
        Tick::reply(20, -20),
        Tick::silent(),
    ];
    let mut trace = Trace::<16>::new();
    let sim = simulate(MockBoard::new(), &script, &mut trace);

    assert_eq!(trace.len(), 4);
    let records = trace.records();
    assert_eq!(
        records.iter().map(|r| r.now_us).collect::<Vec<_>>(),
        vec![0, 2_000, 4_000, 6_000]
    );
    assert_eq!(
        records.iter().map(|r| r.result).collect::<Vec<_>>(),
        vec![
            StepResult::Idle,
            StepResult::Accepted(Command::ZERO),
            StepResult::Accepted(Command::ZERO),
            StepResult::Idle,
        ]
    );
    // BURN keeps the independent gate closed even though the link accepts.
    assert!(records.iter().all(|r| r.phase == Phase::BURN as u8
        && r.interlock_refusal == Some(Refusal::Boost)
        && !r.interlock_permitted
        && r.applied.is_zero()));

    let frames = sim.inner_board().sent_frames();
    assert_eq!(frames.len(), 4, "one request per control tick");
    for (i, bytes) in frames.iter().enumerate() {
        let request = decode_request(bytes).expect("requests must decode");
        assert_eq!(request.seq, i as u16);
        assert_eq!(request.time_us, i as u32 * TICK_PERIOD_US);
        assert_eq!(request.phase, Phase::BURN);
    }
    assert_eq!(
        sim.inner_board().rx().swaps,
        2,
        "only frames cause DMA swaps"
    );
    assert_eq!(sim.inner_board().pet_count(), 4);
}

#[test]
fn phase_events_are_recorded_as_transitions_and_gate_output() {
    let script = [
        Tick::reply(50, 50),
        Tick::reply(50, 50).event(PhaseEvent::LiftOff), // BURN -> BURN
        Tick::reply(50, 50).event(PhaseEvent::Burnout), // BURN -> COAST
        Tick::reply(50, 50).event(PhaseEvent::Apogee),
        Tick::reply(50, 50).event(PhaseEvent::Descent),
        Tick::reply(50, 50).event(PhaseEvent::Touchdown),
    ];
    let mut trace = Trace::<32>::new();
    let sim = simulate(MockBoard::new(), &script, &mut trace);

    assert_eq!(
        trace.events(),
        &[
            TraceEvent::Phase {
                tick: 2,
                from: 0,
                to: 1
            },
            TraceEvent::Phase {
                tick: 3,
                from: 1,
                to: 2
            },
            TraceEvent::Phase {
                tick: 4,
                from: 2,
                to: 3
            },
            TraceEvent::Phase {
                tick: 5,
                from: 3,
                to: 4
            },
        ]
    );
    assert_eq!(sim.fsm.phase(), Phase::LANDED);
    assert_eq!(
        trace.records().iter().map(|r| r.phase).collect::<Vec<_>>(),
        vec![0, 0, 1, 2, 3, 4]
    );
    // Guidance is zero through both BURN ticks, then slews 10 raw per tick.
    assert_eq!(
        trace
            .records()
            .iter()
            .map(|r| r.applied.pitch)
            .collect::<Vec<_>>(),
        vec![0, 0, 10, 20, 30, 40]
    );
}

#[test]
fn interlock_latch_blocks_a_link_accepted_command_until_reset() {
    let mut sim = Simulator::new(MockBoard::new());
    let mut trace = Trace::<32>::new();

    sim.run(
        &[Tick::reply(90, -90).event(PhaseEvent::Burnout)],
        &mut trace,
    );
    assert_eq!(
        sim.inner_board().last_applied(),
        Command {
            pitch: 10,
            yaw: -10
        }
    );

    sim.tick(&Tick::reply(90, -90).phase_code(0xff), &mut trace);
    let rec = trace.records().last().unwrap();
    assert_eq!(
        rec.result,
        StepResult::Accepted(Command {
            pitch: 90,
            yaw: -90
        })
    );
    assert!(rec.applied.is_zero(), "independent gate vetoes the command");
    assert_eq!(rec.interlock_refusal, Some(Refusal::UnknownPhase(0xff)));
    assert_eq!(sim.gate().latched(), Some(Refusal::UnknownPhase(0xff)));
    assert!(trace.events().contains(&TraceEvent::InterlockLatched {
        tick: 1,
        refusal: Refusal::UnknownPhase(0xff),
    }));

    sim.tick(&Tick::reply(90, -90), &mut trace);
    assert!(trace.records().last().unwrap().applied.is_zero());
    assert_eq!(sim.gate().counters.blocked, 2);

    sim.reset_gate(&mut trace);
    assert!(trace
        .events()
        .contains(&TraceEvent::InterlockReset { tick: 3 }));
    sim.tick(&Tick::reply(90, -90), &mut trace);
    assert_eq!(
        sim.inner_board().last_applied(),
        Command {
            pitch: 10,
            yaw: -10
        },
        "reset clears the slew history, so recovery starts from failsafe"
    );
    assert_eq!(sim.gate().counters.trips, 1);
    assert_eq!(sim.gate().counters.resets, 1);
}

#[test]
fn board_fault_flag_trips_the_interlock_without_touching_the_link() {
    let mut sim = Simulator::new(MockBoard::new());
    let mut trace = Trace::<32>::new();

    sim.run(
        &[Tick::reply(20, 20).event(PhaseEvent::Burnout)],
        &mut trace,
    );
    sim.tick(&Tick::reply(20, 20).flags(flags::FAULT), &mut trace);

    let rec = trace.records().last().unwrap();
    assert_eq!(
        rec.result,
        StepResult::Accepted(Command { pitch: 20, yaw: 20 })
    );
    assert_eq!(rec.interlock_refusal, Some(Refusal::Fault));
    assert!(rec.applied.is_zero());
    assert!(!rec.disabled, "FAULT is not a link latch");
    assert_eq!(sim.gate().latched(), Some(Refusal::Fault));

    sim.reset_gate(&mut trace);
    sim.tick(&Tick::reply(20, 20), &mut trace);
    assert_eq!(
        sim.inner_board().last_applied(),
        Command { pitch: 10, yaw: 10 }
    );
}

#[test]
fn motor_armed_is_a_transient_veto_that_does_not_latch() {
    let mut trace = Trace::<8>::new();
    let sim = simulate(
        MockBoard::new(),
        &[
            Tick::reply(50, 50).event(PhaseEvent::Burnout),
            Tick::reply(50, 50).armed(true),
            Tick::reply(50, 50).armed(false),
        ],
        &mut trace,
    );

    assert_eq!(
        trace
            .records()
            .iter()
            .map(|r| r.applied.pitch)
            .collect::<Vec<_>>(),
        vec![10, 0, 20]
    );
    assert_eq!(
        trace.records()[1].interlock_refusal,
        Some(Refusal::MotorArmed)
    );
    assert!(sim.gate().latched().is_none());
}

#[test]
fn unhealthy_reply_is_refused_by_both_layers_on_the_same_tick() {
    let mut trace = Trace::<16>::new();
    let sim = simulate(
        MockBoard::new(),
        &[
            Tick::unhealthy(70, 70).event(PhaseEvent::Burnout),
            Tick::reply(70, 70),
        ],
        &mut trace,
    );

    let first = &trace.records()[0];
    assert_eq!(first.result, StepResult::FrameRejected);
    assert!(first.disabled);
    assert!(first.applied.is_zero());
    assert_eq!(first.interlock_refusal, Some(Refusal::AiUnhealthy));
    assert!(trace
        .events()
        .contains(&TraceEvent::LinkDisabled { tick: 0 }));
    assert!(trace.events().contains(&TraceEvent::InterlockLatched {
        tick: 0,
        refusal: Refusal::AiUnhealthy,
    }));
    assert_eq!(sim.link.counters.disables, 1);
    assert_eq!(sim.gate().counters.trips, 1);

    // Neither latch clears on a later healthy frame.
    assert!(trace.records()[1].disabled);
    assert!(trace.records()[1].applied.is_zero());
}

#[test]
fn corrupt_stale_and_silent_frames_share_one_three_strike_latch() {
    let script = [
        Tick::corrupt(100, 100, 12, 0x02),
        Tick::stale(100, 100),
        Tick::silent(),
        Tick::reply(60, -60),
    ];
    let mut trace = Trace::<32>::new();
    let mut sim = simulate(MockBoard::new(), &script, &mut trace);

    assert_eq!(
        trace.records().iter().map(|r| r.result).collect::<Vec<_>>(),
        vec![
            StepResult::FrameRejected,
            StepResult::FrameRejected,
            StepResult::Idle,
            StepResult::FrameRejected,
        ]
    );
    assert_eq!(
        trace.records().iter().map(|r| r.misses).collect::<Vec<_>>(),
        vec![1, 2, 3, 0]
    );
    assert!(trace
        .events()
        .contains(&TraceEvent::LinkDisabled { tick: 2 }));
    assert!(trace.records()[2].deadman_tripped);
    assert!(trace.records().iter().all(|r| r.applied.is_zero()));
    assert_eq!(sim.link.counters.rejected, 3);
    assert_eq!(sim.link.counters.disables, 1);
    // Two misses still feed; the third trips the deadman; the valid reply
    // clears it and feeding resumes even though the AI stays latched off.
    assert_eq!(sim.inner_board().pet_count(), 3);

    sim.link.reset();
    let mut next = Trace::<4>::new();
    sim.tick(&Tick::reply(60, -60), &mut next);
    assert_eq!(
        next.records()[0].result,
        StepResult::Accepted(Command::ZERO)
    );
}

#[test]
fn rate_limit_slews_at_most_ten_raw_per_tick() {
    let mut script = vec![Tick::reply(100, 100).event(PhaseEvent::Burnout)];
    for _ in 0..10 {
        script.push(Tick::reply(100, 100));
    }
    let mut trace = Trace::<32>::new();
    let _sim = simulate(MockBoard::new(), &script, &mut trace);

    let expected = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 100];
    let pitches = trace
        .records()
        .iter()
        .map(|r| r.applied.pitch)
        .collect::<Vec<_>>();
    assert_eq!(pitches, expected);
    assert!(trace.records().iter().all(|r| r.interlock_permitted));
    assert_eq!(
        trace
            .records()
            .iter()
            .map(|r| r.interlock_limited)
            .collect::<Vec<_>>(),
        vec![true, true, true, true, true, true, true, true, true, false, false]
    );
}

#[test]
fn identical_scripts_produce_identical_traces() {
    let script = [
        Tick::reply(30, -30).event(PhaseEvent::Burnout),
        Tick::reply(40, -40).event(PhaseEvent::Apogee),
        Tick::silent(),
        Tick::corrupt(50, 50, 12, 0x01),
        Tick::reply(60, -60).flags(flags::FAULT),
        Tick::reply(70, -70),
        Tick::reply(80, -80).event(PhaseEvent::Descent),
        Tick::reply(90, -90),
    ];

    let mut a = Trace::<32>::new();
    let sim_a = simulate(MockBoard::new(), &script, &mut a);
    let mut b = Trace::<32>::new();
    let sim_b = simulate(MockBoard::new(), &script, &mut b);

    assert_eq!(a, b);
    assert_eq!(sim_a.link.counters, sim_b.link.counters);
    assert_eq!(sim_a.gate().counters, sim_b.gate().counters);
    assert_eq!(
        sim_a.inner_board().sent_frames(),
        sim_b.inner_board().sent_frames()
    );
    assert_eq!(sim_a.inner_board().applied, sim_b.inner_board().applied);
    assert_eq!(a.len(), script.len());
}

#[test]
fn an_incomplete_cycle_withholds_the_watchdog_feed() {
    let mut trace = Trace::<8>::new();
    let sim = simulate(
        MockBoard::new(),
        &[
            Tick::reply(30, 30).event(PhaseEvent::Burnout).incomplete(),
            Tick::reply(30, 30),
        ],
        &mut trace,
    );
    assert_eq!(sim.inner_board().pet_count(), 1);
    assert_eq!(trace.records()[0].pets, 0);
    assert_eq!(trace.records()[1].pets, 1);
    assert_eq!(
        trace.records()[0].result,
        StepResult::Accepted(Command { pitch: 30, yaw: 30 })
    );
}

#[test]
fn raw_frame_injection_covers_short_and_badmagic_chunks() {
    let mut bytes = [0u8; REPLY_SIZE];
    potok_m_firmware::protocol::encode_reply(&Frame::template(40, -40), &mut bytes).unwrap();
    let mut bad_magic = bytes;
    bad_magic[0] ^= 0xff;

    // Fresh assemblers: a partial frame left in the buffer would otherwise
    // combine with the next chunk into a complete (CRC-bad) frame.
    let mut short_trace = Trace::<4>::new();
    let _ = simulate(
        MockBoard::new(),
        &[Tick {
            frame: Frame::Raw {
                bytes,
                len: REPLY_SIZE - 1,
            },
            ..Tick::reply(0, 0)
        }],
        &mut short_trace,
    );
    let mut magic_trace = Trace::<4>::new();
    let _ = simulate(
        MockBoard::new(),
        &[Tick {
            frame: Frame::Raw {
                bytes: bad_magic,
                len: REPLY_SIZE,
            },
            ..Tick::reply(0, 0)
        }],
        &mut magic_trace,
    );

    for trace in [&short_trace, &magic_trace] {
        assert_eq!(trace.len(), 1);
        let rec = &trace.records()[0];
        assert_eq!(rec.result, StepResult::Idle, "{rec:?}");
        assert_eq!(rec.misses, 1, "{rec:?}");
        assert!(rec.applied.is_zero());
    }
}

#[test]
#[should_panic(expected = "sil: trace record capacity exceeded")]
fn trace_overflow_fails_loud() {
    let mut trace = Trace::<1>::new();
    simulate(
        MockBoard::new(),
        &[Tick::silent(), Tick::silent()],
        &mut trace,
    );
}
