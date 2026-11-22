// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host gates for the second-stage separation/ignition sequencer.

#[path = "../src/separation.rs"]
mod separation;

use potok_protocol::Phase;
use separation::{
    ArmChannels, Fault, Output, PhaseReading, Plan, Sample, SensorPair, Sequencer, State,
};

fn plan() -> Plan {
    Plan {
        trigger: Phase::COAST,
        delay_us: 5_000,
        phase_skew_tolerance_us: 1_000,
        sensor_tolerance: 10,
        separation_level: 100,
    }
}

fn sample(now_us: u64, phase: Phase, stamp_a: u64, stamp_b: u64, a: i32, b: i32) -> Sample {
    Sample {
        now_us,
        phase: [
            PhaseReading {
                phase,
                at_us: stamp_a,
            },
            PhaseReading {
                phase,
                at_us: stamp_b,
            },
        ],
        sensors: SensorPair { a, b },
    }
}

fn both_arms() -> Sequencer {
    let mut seq = Sequencer::new(plan());
    seq.arm(ArmChannels {
        primary: true,
        secondary: false,
    })
    .unwrap();
    seq.arm(ArmChannels {
        primary: false,
        secondary: true,
    })
    .unwrap();
    assert_eq!(seq.state(), State::Armed);
    seq
}

#[test]
fn arm_requires_two_independent_channels_latched_separately() {
    let mut seq = Sequencer::new(plan());
    assert_eq!(seq.state(), State::Idle);
    assert_eq!(seq.plan(), plan());
    assert_eq!(seq.arm_channels(), ArmChannels::default());
    assert_eq!(seq.last_fault(), None);
    assert_eq!(seq.last_now_us(), 0);
    assert!(!seq.is_latched());
    assert_eq!(seq.abort_count(), 0);
    assert_eq!(seq.separation_count(), 0);
    assert_eq!(seq.ignition_count(), 0);

    seq.arm(ArmChannels {
        primary: true,
        secondary: false,
    })
    .unwrap();
    assert_eq!(
        seq.arm_channels(),
        ArmChannels {
            primary: true,
            secondary: false
        }
    );
    assert_eq!(seq.state(), State::Idle, "one channel cannot arm");

    seq.arm(ArmChannels {
        primary: false,
        secondary: true,
    })
    .unwrap();
    assert_eq!(seq.state(), State::Armed);
    assert_eq!(
        seq.arm_channels(),
        ArmChannels {
            primary: true,
            secondary: true
        }
    );
    assert_eq!(seq.arm(ArmChannels::default()), Err(Fault::ArmWhenNotIdle));
    assert_eq!(seq.state(), State::Armed);
}

#[test]
fn one_channel_alone_never_fires() {
    let mut seq = Sequencer::new(plan());
    seq.arm(ArmChannels {
        primary: true,
        secondary: false,
    })
    .unwrap();
    let result = seq.tick(sample(10_000, Phase::COAST, 0, 0, 0, 0));
    assert_eq!(result, Ok(Output::None));
    assert_eq!(seq.state(), State::Idle);
    assert_eq!(seq.separation_count(), 0);
}

#[test]
fn separation_waits_for_trigger_phase_and_delay() {
    let mut seq = both_arms();

    // Before the trigger phase: hold, even long after the timer would expire.
    assert_eq!(
        seq.tick(sample(1_000, Phase::BURN, 0, 0, 0, 0)),
        Ok(Output::None)
    );
    assert_eq!(seq.state(), State::Armed);

    // In COAST but one microsecond short of the delay.
    assert_eq!(
        seq.tick(sample(4_999, Phase::COAST, 0, 0, 0, 0)),
        Ok(Output::None)
    );
    assert_eq!(seq.state(), State::Armed);

    // Exactly at the deadline: one separation command, then latched.
    assert_eq!(
        seq.tick(sample(5_000, Phase::COAST, 0, 0, 0, 0)),
        Ok(Output::Separation)
    );
    assert_eq!(seq.state(), State::Separating { at_us: 5_000 });
    assert_eq!(seq.separation_count(), 1);
    assert!(seq.is_latched());
}

#[test]
fn delay_starts_from_the_later_redundant_phase_stamp() {
    let mut seq = both_arms();
    // Both channels agree on COAST; the later stamp is the conservative epoch.
    assert_eq!(
        seq.tick(sample(5_000, Phase::COAST, 0, 800, 0, 0)),
        Ok(Output::None)
    );
    assert_eq!(seq.state(), State::Armed);
    assert_eq!(
        seq.tick(sample(5_799, Phase::COAST, 0, 800, 0, 0)),
        Ok(Output::None)
    );
    assert_eq!(
        seq.tick(sample(5_800, Phase::COAST, 0, 800, 0, 0)),
        Ok(Output::Separation)
    );
    assert_eq!(seq.state(), State::Separating { at_us: 5_800 });
}

#[test]
fn inconsistent_sensors_abort_and_latch_until_explicit_reset() {
    let mut seq = both_arms();
    let fault = Fault::SensorMismatch { a: 0, b: 11 };
    assert_eq!(
        seq.tick(sample(6_000, Phase::COAST, 0, 0, 0, 11)),
        Err(fault)
    );
    assert_eq!(seq.state(), State::Aborted(fault));
    assert_eq!(seq.last_fault(), Some(fault));
    assert_eq!(seq.abort_count(), 1);
    assert_eq!(seq.separation_count(), 0);
    assert_eq!(seq.arm(ArmChannels::default()), Err(Fault::ArmWhenNotIdle));
    assert_eq!(
        seq.tick(sample(7_000, Phase::COAST, 0, 0, 0, 0)),
        Err(Fault::Latched)
    );
    assert_eq!(
        seq.state(),
        State::Aborted(fault),
        "abort state is immutable"
    );

    seq.reset();
    assert_eq!(seq.state(), State::Idle);
    assert!(!seq.is_latched());
    assert_eq!(seq.arm_channels(), ArmChannels::default());
    assert_eq!(seq.last_fault(), None);
    assert_eq!(seq.abort_count(), 1, "diagnostics survive a reset");
}

#[test]
fn disagreeing_phase_channels_abort() {
    let mut seq = both_arms();
    let fault = Fault::PhaseMismatch {
        a: Phase::COAST,
        b: Phase::APOGEE,
    };
    let mut s = sample(6_000, Phase::COAST, 0, 0, 0, 0);
    s.phase[1].phase = Phase::APOGEE;
    assert_eq!(seq.tick(s), Err(fault));
    assert_eq!(seq.state(), State::Aborted(fault));
}

#[test]
fn skewed_phase_stamps_abort() {
    let mut seq = both_arms();
    let fault = Fault::PhaseStampSkew {
        a_us: 0,
        b_us: 2_000,
    };
    assert_eq!(
        seq.tick(sample(6_000, Phase::COAST, 0, 2_000, 0, 0)),
        Err(fault)
    );
    assert_eq!(seq.last_fault(), Some(fault));
}

#[test]
fn future_phase_stamp_aborts() {
    let mut seq = both_arms();
    let fault = Fault::PhaseStampInFuture {
        at_us: 5_000,
        now_us: 1_000,
    };
    assert_eq!(
        seq.tick(sample(1_000, Phase::COAST, 5_000, 5_000, 0, 0)),
        Err(fault)
    );
    assert_eq!(seq.state(), State::Aborted(fault));
}

#[test]
fn missed_phase_window_aborts() {
    let mut seq = both_arms();
    let fault = Fault::PhaseWindowMissed {
        trigger: Phase::COAST,
        observed: Phase::APOGEE,
    };
    assert_eq!(
        seq.tick(sample(6_000, Phase::APOGEE, 0, 0, 0, 0)),
        Err(fault)
    );
    assert_eq!(seq.state(), State::Aborted(fault));
}

#[test]
fn time_regression_aborts() {
    let mut seq = both_arms();
    assert_eq!(
        seq.tick(sample(10_000, Phase::BURN, 0, 0, 0, 0)),
        Ok(Output::None)
    );
    assert_eq!(seq.last_now_us(), 10_000);
    let fault = Fault::TimeWentBackwards {
        now_us: 9_999,
        last_us: 10_000,
    };
    assert_eq!(seq.tick(sample(9_999, Phase::BURN, 0, 0, 0, 0)), Err(fault));
    assert_eq!(seq.state(), State::Aborted(fault));
}

#[test]
fn separation_confirmation_needs_both_sensors_then_ignition_latches() {
    let mut seq = both_arms();
    seq.tick(sample(5_000, Phase::COAST, 0, 0, 0, 0)).unwrap();
    assert_eq!(seq.state(), State::Separating { at_us: 5_000 });

    // One sensor below the level holds the separation command.
    assert_eq!(
        seq.tick(sample(6_000, Phase::COAST, 0, 0, 100, 99)),
        Ok(Output::Separation)
    );
    assert_eq!(seq.state(), State::Separating { at_us: 5_000 });
    assert_eq!(seq.ignition_count(), 0);

    // Both agree: ignition is commanded once and stays latched.
    assert_eq!(
        seq.tick(sample(7_000, Phase::COAST, 0, 0, 100, 100)),
        Ok(Output::Ignition)
    );
    assert_eq!(seq.state(), State::Igniting { at_us: 7_000 });
    assert_eq!(seq.ignition_count(), 1);
    assert_eq!(seq.arm(ArmChannels::default()), Err(Fault::ArmWhenNotIdle));
    assert_eq!(
        seq.tick(sample(8_000, Phase::COAST, 0, 0, 0, 0)),
        Ok(Output::Ignition)
    );
    assert_eq!(seq.ignition_count(), 1, "no repeated ignition command");
    assert_eq!(seq.separation_count(), 1, "no repeated separation command");
}

#[test]
fn sensor_disagreement_after_ignition_is_latched_as_abort() {
    let mut seq = both_arms();
    seq.tick(sample(5_000, Phase::COAST, 0, 0, 0, 0)).unwrap();
    seq.tick(sample(6_000, Phase::COAST, 0, 0, 100, 100))
        .unwrap();
    assert_eq!(seq.state(), State::Igniting { at_us: 6_000 });

    let fault = Fault::SensorMismatch { a: 100, b: 60 };
    assert_eq!(
        seq.tick(sample(7_000, Phase::COAST, 0, 0, 100, 60)),
        Err(fault)
    );
    assert_eq!(seq.state(), State::Aborted(fault));
    assert_eq!(seq.abort_count(), 1);
}

#[test]
fn identical_scripts_produce_identical_traces() {
    let script = [
        sample(0, Phase::BURN, 0, 0, 0, 0),
        sample(2_000, Phase::COAST, 1_000, 1_000, 3, -3),
        sample(5_999, Phase::COAST, 1_000, 1_000, 4, -4),
        sample(6_000, Phase::COAST, 1_000, 1_000, 4, -4),
        sample(6_000, Phase::COAST, 1_000, 1_000, 100, 100),
        sample(7_000, Phase::COAST, 1_000, 1_000, 100, 100),
    ];

    let run = || {
        let mut seq = Sequencer::new(plan());
        seq.arm(ArmChannels {
            primary: true,
            secondary: false,
        })
        .unwrap();
        seq.arm(ArmChannels {
            primary: false,
            secondary: true,
        })
        .unwrap();
        let trace: Vec<_> = script.iter().map(|s| seq.tick(*s)).collect();
        (seq, trace)
    };

    let (a_seq, a_trace) = run();
    let (b_seq, b_trace) = run();
    assert_eq!(a_trace, b_trace);
    assert_eq!(a_seq, b_seq);
    assert_eq!(
        a_trace,
        vec![
            Ok(Output::None),
            Ok(Output::None),
            Ok(Output::None),
            Ok(Output::Separation),
            Ok(Output::Ignition),
            Ok(Output::Ignition),
        ]
    );
    assert_eq!(a_seq.state(), State::Igniting { at_us: 6_000 });
    assert_eq!(a_seq.abort_count(), 0);
    assert_eq!(a_seq.separation_count(), 1);
    assert_eq!(a_seq.ignition_count(), 1);
}

#[test]
fn sensor_pair_spread_handles_the_full_i32_range() {
    assert_eq!(
        SensorPair {
            a: i32::MIN,
            b: i32::MAX
        }
        .spread(),
        u32::MAX
    );
    assert_eq!(SensorPair { a: -5, b: 5 }.spread(), 10);
    assert_eq!(SensorPair { a: 7, b: 7 }.spread(), 0);
}
