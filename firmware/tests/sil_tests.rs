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

// ---------------------------------------------------------------------------
// Full shipped path (main.rs loop body minus pins).
//
// Every stage below is the real module driven by a scripted MockBoard. The
// `Guarded` harness above stays as a fault microscope; these gates exercise
// the binary's own `step()` with its interlock, shield and watchdog policy.
// No pin, board, clock or flight behaviour is claimed.
// ---------------------------------------------------------------------------

use potok_m_firmware::{
    control_pid::{Fx, PidConfig},
    deployment::{ArmChannels as DeployArm, DeploymentSample},
    search_beacon::SignalInputs,
    self_test::{
        ContinuityMock, Range, GOLDEN_CONFIG, GOLDEN_CONFIG_CRC, LatchPhase, LatchState,
        SensorLimits, SensorSample, Ticker,
    },
    sensor_voting::ImuSample,
    separation::{ArmChannels as SepArm, PhaseReading, Plan as SepPlan, SensorPair},
    AxisSample, Capability, DeploymentInput, SelfTestInput, SeparationInput, StabilizationSample,
};

/// Capabilities owned by `FlightPath`. The other three (`Actuation`,
/// `PhaseSense`, `MotorArmedSense`) are counted by the `Board` adapter, as in
/// the shipping binary, not by the module ledger.
const MODULE_CAPS: [Capability; 7] = [
    Capability::SensorVoting,
    Capability::Mahony,
    Capability::Pid,
    Capability::Deployment,
    Capability::Separation,
    Capability::SelfTest,
    Capability::Radio,
];

use potok_m_firmware::telemetry_log::Replay;

use sil::full::{
    simulate_full, FlightInputs, FlightSimulator, FlightTick, FlightTrace, LOG_SZ,
    EMIT_AIRDATA, EMIT_APPLIED, EMIT_CAPABILITY, EMIT_REJECT, REACH_AIRDATA, REACH_ALL,
    REACH_APPLY, REACH_DEPLOYMENT, REACH_INTERLOCK, REACH_LINK, REACH_MAHONY, REACH_PHASE,
    REACH_PID, REACH_RADIO, REACH_SELF_TEST, REACH_SEPARATION, REACH_SHIELD, REACH_TELEMETRY,
    REACH_VOTING, REACH_WATCHDOG,
};

/// Every capability present, with a wired pitot. No value here is a flight
/// tuning claim: these are deterministic fixtures for reaching each module.
fn full_inputs() -> FlightInputs {
    let gains = PidConfig {
        kp: Fx::from_raw(2),
        ki_dt: Fx::from_raw(0),
        kd_dt: Fx::from_raw(0),
        out_limit: Fx::from_raw(100),
        i_limit: Fx::from_raw(50),
        deriv_alpha: Fx::from_raw(1),
    };
    FlightInputs {
        imu: Some([Some(ImuSample {
            gyro: [0.05, 0.0, 0.0],
            accel: [0.0, 0.0, 1.0],
        }); 3]),
        height: Some([Some(100.0); 3]),
        stabilization: Some(StabilizationSample {
            pitch: AxisSample {
                setpoint: 20,
                measurement: 0,
            },
            yaw: AxisSample {
                setpoint: -20,
                measurement: 0,
            },
            pitch_config: gains,
            yaw_config: gains,
        }),
        deployment: Some(DeploymentInput {
            arm: DeployArm::BOTH,
            sample: DeploymentSample {
                altitude_m: 100,
                vertical_speed_cm_s: 0,
            },
        }),
        separation: Some(SeparationInput {
            plan: SepPlan {
                trigger: Phase::LANDED,
                delay_us: 0,
                phase_skew_tolerance_us: 1_000,
                sensor_tolerance: 10,
                separation_level: 1,
            },
            arm: SepArm {
                primary: true,
                secondary: true,
            },
            phase: [PhaseReading {
                phase: Phase::COAST,
                at_us: 0,
            }; 2],
            sensors: SensorPair { a: 1, b: 1 },
        }),
        self_test: Some(SelfTestInput {
            sensor: SensorSample {
                accel: 10,
                baro: 100,
                battery_mv: 3_900,
                temp_c: 25,
            },
            limits: SensorLimits {
                accel: Range {
                    min: -100,
                    max: 100,
                },
                baro: Range { min: 0, max: 1_200 },
                battery: Range {
                    min: 3_000,
                    max: 4_200,
                },
                temperature: Range { min: -20, max: 70 },
            },
            config: GOLDEN_CONFIG,
            config_crc: GOLDEN_CONFIG_CRC,
            continuity: ContinuityMock::healthy(),
            latch_state: LatchState {
                phase: LatchPhase::Flight,
                motor_armed: false,
                latched: false,
                ai_allowed: true,
            },
            ticker: Ticker::new(0, 250),
        }),
        signals: Some(SignalInputs {
            post_landing: false,
            melody_requested: false,
            boost_active: false,
            battery_mv: 3_900,
        }),
        airdata: Some((200.0, 100_000.0)),
    }
}

fn nominal_script() -> Vec<FlightTick> {
    let mut script = vec![FlightTick::reply(60, -60)
        .event(PhaseEvent::Burnout)
        .with(full_inputs())];
    for _ in 0..11 {
        script.push(FlightTick::reply(60, -60).with(full_inputs()));
    }
    script
}

#[test]
fn full_path_reaches_every_shipped_module() {
    let script = nominal_script();
    let mut trace = FlightTrace::<32>::new();
    let sim = simulate_full(&script, &mut trace);

    assert_eq!(trace.len(), script.len());
    let reached = sim.reached();
    for (bit, name) in [
        (REACH_LINK, "link"),
        (REACH_PHASE, "phase event"),
        (REACH_VOTING, "sensor voting"),
        (REACH_MAHONY, "Mahony"),
        (REACH_PID, "PID"),
        (REACH_DEPLOYMENT, "deployment"),
        (REACH_SEPARATION, "separation"),
        (REACH_SELF_TEST, "self test"),
        (REACH_RADIO, "radio beacon"),
        (REACH_INTERLOCK, "interlock"),
        (REACH_SHIELD, "shield"),
        (REACH_APPLY, "apply_command"),
        (REACH_WATCHDOG, "watchdog feed"),
        (REACH_TELEMETRY, "telemetry"),
        (REACH_AIRDATA, "airdata"),
    ] {
        assert_ne!(reached & bit, 0, "full path did not reach {name}");
    }
    assert_eq!(reached, REACH_ALL, "all shipped stages must be reached");

    // Reachability is backed by outputs, not by a stage label: the ledger, the
    // applied command, the attitude, the PID output and the feed all moved.
    for &cap in MODULE_CAPS.iter() {
        assert!(
            sim.flight.ledger().used(cap) > 0,
            "{cap:?} never produced an output"
        );
        let unavailable = sim.flight.ledger().unavailable(cap);
        if cap == Capability::Deployment {
            // The first tick is still BURN when `FlightPath::tick` runs, so
            // the deployment gate correctly refuses it once: a real reason,
            // counted, not a silent skip.
            assert_eq!(unavailable, 1, "deployment must refuse only the BURN tick");
        } else {
            assert_eq!(unavailable, 0, "{cap:?} was present but counted unavailable");
        }
    }
    assert!(sim.interlock.counters.accepted > 0);
    assert!(trace.records().iter().any(|r| !r.applied.is_zero()));
    assert!(trace.records().iter().all(|r| r.pets == r.tick + 1));
    assert!(trace.records().iter().any(|r| r.attitude.is_some()));
    assert!(trace.records().iter().any(|r| r.stabilization.is_some()));
    assert_eq!(sim.flight.beacon_tick(), script.len() as u32);
    assert_eq!(sim.sampler.unavailable_counts(), Default::default());
    assert_eq!(sim.board.inner().applied.len(), script.len());
    assert_eq!(
        sim.board.stabilization_applied.len(),
        script.len(),
        "PID wrote one command per tick"
    );
    assert!(!sim.board.radio_applied.is_empty());

    let first = &trace.records()[0];
    assert_eq!(first.result, StepResult::Accepted(Command { pitch: 10, yaw: -10 }));
    assert_eq!(first.phase, Phase::COAST as u8);
    assert!(first.emissions & (EMIT_APPLIED | EMIT_CAPABILITY) != 0);
}

#[test]
fn absent_capabilities_are_counted_unavailable_never_silently_skipped() {
    let script = [
        FlightTick::reply(90, -90).event(PhaseEvent::Burnout),
        FlightTick::reply(90, -90),
    ];
    let mut trace = FlightTrace::<8>::new();
    let sim = simulate_full(&script, &mut trace);

    for &cap in MODULE_CAPS.iter() {
        assert_eq!(
            sim.flight.ledger().used(cap),
            0,
            "{cap:?} had no input and must not claim to have run"
        );
        assert!(
            sim.flight.ledger().unavailable(cap) > 0,
            "{cap:?} was absent but uncounted"
        );
    }
    assert!(
        sim.sampler.unavailable_counts().no_driver >= script.len() as u32,
        "a missing pitot driver must be counted, never turned into a zero sample"
    );
    assert!(trace.records().iter().all(|r| r.applied.is_zero()));
    assert!(trace
        .records()
        .iter()
        .all(|r| r.shield_reasons.pitot_invalid > 0));
    assert_eq!(sim.reached() & REACH_VOTING, 0);
    assert_eq!(sim.reached() & REACH_MAHONY, 0);
    assert_eq!(sim.reached() & REACH_PID, 0);
    assert_ne!(sim.reached() & REACH_SHIELD, 0);
    assert_ne!(sim.reached() & REACH_TELEMETRY, 0);
    assert!(trace.records().iter().any(|r| r.emissions & EMIT_AIRDATA != 0));
}

#[test]
fn stubbed_board_counts_board_level_capabilities_unavailable() {
    let mut sim = FlightSimulator::stubbed();
    let mut trace = FlightTrace::<8>::new();
    sim.run(
        &[
            FlightTick::reply(50, -50).event(PhaseEvent::Burnout).with(full_inputs()),
            FlightTick::reply(50, -50).with(full_inputs()),
        ],
        &mut trace,
    );

    for cap in [
        Capability::Actuation,
        Capability::PhaseSense,
        Capability::MotorArmedSense,
    ] {
        assert_eq!(sim.board.board_caps.used(cap), 0);
        assert_eq!(
            sim.board.board_caps.unavailable(cap),
            2,
            "{cap:?} stub must be counted on every tick"
        );
        assert_eq!(
            sim.board.board_caps.last_reason(cap),
            Some(potok_m_firmware::Unavailable::NoDriver)
        );
    }
    // No phase sense means the FSM stays in BURN: the command that reaches
    // the (discarding) board is zero, the safe direction. The harness still
    // observes it so the stub cannot hide what it dropped.
    assert_eq!(trace.records().len(), 2);
    assert!(sim.board.last_applied_command().is_zero());
}

#[test]
fn incomplete_cycle_withholds_the_feed_without_rewriting_the_gated_command() {
    let mut trace = FlightTrace::<8>::new();
    let sim = simulate_full(
        &[
            FlightTick::reply(50, -50)
                .event(PhaseEvent::Burnout)
                .with(full_inputs()),
            FlightTick::reply(50, -50).incomplete().with(full_inputs()),
        ],
        &mut trace,
    );

    // Shipped `step()` gates the command on the interlock/shield; the cycle
    // flag only decides whether IWDG may be fed. Claiming zero here would
    // describe a different program.
    assert_eq!(trace.records()[1].applied, Command { pitch: 20, yaw: -20 });
    assert_eq!(trace.records()[1].pets, 1);
    assert!(!sim.policy.may_feed());
    assert_eq!(sim.board.inner().pets(), 1);
}

#[test]
fn zero_command_survives_every_fault_injection() {
    let full = full_inputs();
    let mut faults: Vec<(&str, Vec<FlightTick>)> = vec![
        (
            "corrupt CRC",
            vec![FlightTick::corrupt(90, -90, 12, 0x02).with(full)],
        ),
        ("stale seq", vec![FlightTick::stale(90, -90).with(full)]),
        ("silent link", vec![FlightTick::silent().with(full)]),
        (
            "unhealthy AI",
            vec![FlightTick::unhealthy(90, -90).with(full)],
        ),
        (
            "board FAULT bit",
            vec![FlightTick::reply(90, -90).flags(flags::FAULT).with(full)],
        ),
        (
            "motor armed",
            vec![FlightTick::reply(90, -90).armed(true).with(full)],
        ),
    ];

    let mut no_airdata = full;
    no_airdata.airdata = None;
    faults.push(("no pitot driver", vec![FlightTick::reply(90, -90).with(no_airdata)]));

    let mut bad_pitot = full;
    bad_pitot.airdata = Some((0.0, 100_000.0));
    faults.push((
        "non-positive pitot",
        vec![FlightTick::reply(90, -90).with(bad_pitot)],
    ));

    let mut nan_mach = full;
    nan_mach.airdata = Some((f32::NAN, 100_000.0));
    faults.push((
        "non-finite Mach",
        vec![FlightTick::reply(90, -90).with(nan_mach)],
    ));

    let mut fast_mach = full;
    fast_mach.airdata = Some((100_000.0, 100_000.0));
    faults.push((
        "Mach above limit",
        vec![FlightTick::reply(90, -90).with(fast_mach)],
    ));

    for (name, fault) in faults {
        let mut sim = FlightSimulator::new();
        let mut trace = FlightTrace::<16>::new();
        sim.run(
            &[FlightTick::reply(50, -50)
                .event(PhaseEvent::Burnout)
                .with(full_inputs())],
            &mut trace,
        );
        let before = trace.len();
        sim.run(&fault, &mut trace);

        for record in &trace.records()[before..] {
            assert!(record.applied.is_zero(), "{name}: a fault moved the actuator");
        }
        let mut reasons = 0u32;
        for &cap in potok_m_firmware::CAPABILITIES.iter() {
            reasons += sim.flight.ledger().unavailable(cap);
        }
        reasons += trace.records()[before..]
            .iter()
            .map(|r| {
                if r.emissions & (EMIT_REJECT | EMIT_AIRDATA) != 0 {
                    1
                } else {
                    0
                }
            })
            .sum::<u32>();
        reasons += sim.interlock.counters.blocked + sim.interlock.counters.trips;
        assert!(reasons > 0, "{name}: no gate reported the fault");
    }
}

#[test]
fn full_path_replay_is_bit_identical() {
    let script = nominal_script();
    let mut a = FlightTrace::<32>::new();
    let sim_a = simulate_full(&script, &mut a);
    let mut b = FlightTrace::<32>::new();
    let sim_b = simulate_full(&script, &mut b);

    assert_eq!(a, b);
    assert_eq!(sim_a.reached(), sim_b.reached());
    assert_eq!(
        sim_a.flight.ledger().total_unavailable(),
        sim_b.flight.ledger().total_unavailable()
    );
    assert_eq!(sim_a.flight.attitude(), sim_b.flight.attitude());
    assert_eq!(sim_a.flight.stabilization(), sim_b.flight.stabilization());
    assert_eq!(sim_a.board.inner().applied, sim_b.board.inner().applied);
    assert_eq!(
        sim_a.board.stabilization_applied,
        sim_b.board.stabilization_applied
    );
}

#[test]
#[should_panic(expected = "sil: flight trace capacity exceeded")]
fn full_trace_overflow_fails_loud() {
    let mut trace = FlightTrace::<1>::new();
    let _ = simulate_full(&[FlightTick::silent(), FlightTick::silent()], &mut trace);
}

#[test]
fn shipping_loop_emits_bounded_records_and_never_drops_silently() {
    let script = [
        FlightTick::reply(40, -40)
            .event(PhaseEvent::Burnout)
            .with(full_inputs()),
        FlightTick::silent(),
        FlightTick::corrupt(40, -40, 12, 0x01).with(full_inputs()),
        FlightTick::reply(40, -40).with(full_inputs()),
    ];
    let mut trace = FlightTrace::<16>::new();
    let sim = simulate_full(&script, &mut trace);

    let logger = sim.logger();
    assert!(logger.total_pushed() > 0, "the shipping loop must log");
    assert_eq!(
        sim.flushed() + logger.len() as u64,
        logger.total_pushed(),
        "every pushed frame is either flushed or still retained"
    );
    assert_eq!(sim.flush_refusals(), 0);
    assert_eq!(sim.sink().refused(), 0);
    assert_eq!(sim.sink().writes() as u64, sim.flushed());

    let frames = sim.sink().as_bytes();
    assert_eq!(frames.len(), sim.flushed() as usize * LOG_SZ);
    let seqs: Vec<u32> = Replay::<LOG_SZ>::new(frames)
        .map(|r| r.unwrap().seq)
        .collect();
    assert_eq!(
        seqs,
        (0..seqs.len() as u32).collect::<Vec<_>>(),
        "flushed telemetry replays in order with no gaps"
    );

    let mut again = FlightTrace::<16>::new();
    let sim2 = simulate_full(&script, &mut again);
    assert_eq!(sim.logger().total_pushed(), sim2.logger().total_pushed());
    assert_eq!(sim.sink().as_bytes(), sim2.sink().as_bytes());
}

#[test]
fn default_link_state_never_enables_actuation() {
    use potok_m_firmware::{
        control::ShieldSample,
        interlocks::Interlocks,
        protocol::ReplyFrame,
        step,
        watchdog::WatchdogPolicy,
    };

    let mut board = MockBoard::new();
    let mut fsm = PhaseFsm::new();
    fsm.apply(PhaseEvent::Burnout).unwrap();
    let mut link = LinkState::new();
    assert_eq!(link.shield_sample(), ShieldSample::UNAVAILABLE);
    let seq = link.next_request_seq();
    board.queue_reply(&ReplyFrame {
        seq,
        g_pitch: 40,
        g_yaw: -40,
        confidence: 200,
        ai_healthy: 1,
        version: 1,
    });
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    let result = step(
        &mut board,
        &mut fsm,
        &mut link,
        &mut interlock,
        &mut policy,
        true,
    );
    assert!(matches!(result, StepResult::Accepted(cmd) if cmd.is_zero()));
    assert!(board.applied.last().unwrap().is_zero());
    assert!(
        link.shield_reasons.pitot_invalid > 0,
        "an unsupplied pitot must be a counted veto, not a permissive zero"
    );
}

#[test]
fn permissive_legacy_no_sensor_seam_cannot_command() {
    use potok_m_firmware::control::ShieldSample;

    assert_ne!(ShieldSample::LEGACY_NO_SENSOR, ShieldSample::UNAVAILABLE);
    assert_eq!(LinkState::new().shield_sample(), ShieldSample::UNAVAILABLE);
    let main = include_str!("../src/main.rs");
    assert!(main.contains("set_shield_sample"));
    assert!(
        !main.contains("LEGACY_NO_SENSOR"),
        "the permissive seam must never appear in the shipping binary"
    );
}
