// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic host gates for the independent parachute deployment logic.

#[path = "../src/deployment.rs"]
mod deployment;

use deployment::{
    ArmChannels, Decision, DeploymentConfig, DeploymentController, DeploymentSample, Refusal,
    SpringOutputs, Trigger,
};
use potok_protocol::Phase;

fn sample(altitude_m: i32, vertical_speed_cm_s: i32) -> DeploymentSample {
    DeploymentSample {
        altitude_m,
        vertical_speed_cm_s,
    }
}

fn tick(
    ctl: &mut DeploymentController,
    phase: Phase,
    arm: ArmChannels,
    altitude_m: i32,
    vertical_speed_cm_s: i32,
) -> Decision {
    ctl.update(phase, arm, sample(altitude_m, vertical_speed_cm_s))
}

fn armed(ctl: &mut DeploymentController, phase: Phase) {
    for altitude_m in [15, 30, 45, 60] {
        assert_eq!(
            tick(ctl, phase, ArmChannels::BOTH, altitude_m, 300),
            Decision::Hold
        );
    }
}

#[test]
fn default_state_is_safe_and_each_output_is_independent_of_the_phase() {
    let ctl = DeploymentController::default();
    assert_eq!(ctl.outputs(), SpringOutputs::SAFE);
    assert!(!ctl.outputs().is_released());
    assert_eq!(ctl.trigger(), None);
    assert_eq!(ctl.phase(), Phase::BURN);
    assert_eq!(ctl.peak_altitude_m(), 0);
    assert_eq!(ctl.descent_samples(), 0);
    assert_eq!(ctl.refusals(), 0);
    assert_eq!(ctl.config(), &DeploymentConfig::DEFAULT);
    assert_eq!(DeploymentConfig::default(), DeploymentConfig::DEFAULT);
    assert_eq!(DeploymentController::new(DeploymentConfig::DEFAULT), ctl);
}

#[test]
fn config_validation_is_loud() {
    assert!(
        std::panic::catch_unwind(|| DeploymentConfig::new(30, 2, 5, 15, -1500, 3, -1)).is_err()
    );
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(30, 2, 5, 3, -1500, 3, 3)).is_err());
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(10, 2, 5, 15, -1500, 3, 3)).is_err());
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(30, 0, 5, 15, -1500, 3, 3)).is_err());
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(30, 2, 0, 15, -1500, 3, 3)).is_err());
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(30, 2, 5, 15, -1500, 0, 3)).is_err());
    assert!(std::panic::catch_unwind(|| DeploymentConfig::new(30, 2, 5, 15, 0, 3, 3)).is_err());
    let cfg = DeploymentConfig::new(40, 3, 6, 20, -2000, 4, 5);
    assert_eq!(cfg.apogee_min_altitude_m, 40);
    assert_eq!(cfg.low_altitude_m, 20);
    assert_eq!(cfg.fast_descent_cm_s, -2000);
    assert_eq!(cfg.descent_confirm, 4);
}

#[test]
fn wrong_phase_is_refused_and_advances_no_sensor_state() {
    let mut ctl = DeploymentController::default();
    for _ in 0..5 {
        assert_eq!(
            tick(&mut ctl, Phase::BURN, ArmChannels::BOTH, 100, -2_000),
            Decision::Refused(Refusal::WrongPhase { phase: Phase::BURN })
        );
    }
    assert_eq!(ctl.refusals(), 5);
    assert_eq!(ctl.peak_altitude_m(), 0);
    assert_eq!(ctl.descent_samples(), 0);
    assert_eq!(ctl.outputs(), SpringOutputs::SAFE);
    assert_eq!(ctl.phase(), Phase::BURN);

    assert_eq!(
        tick(&mut ctl, Phase::LANDED, ArmChannels::BOTH, 100, -2_000),
        Decision::Refused(Refusal::WrongPhase {
            phase: Phase::LANDED
        })
    );
    assert_eq!(ctl.refusals(), 6);
    assert_eq!(ctl.peak_altitude_m(), 0);

    // A legal phase starts from a clean slate: the refused samples did not
    // pre-load the descent counter.
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 100, -2_000),
        Decision::Hold
    );
    assert_eq!(ctl.descent_samples(), 1);
}

#[test]
fn both_arm_channels_are_required_at_the_decision_tick() {
    assert!(!ArmChannels::UNARMED.is_complete());
    assert!(!ArmChannels { a: true, b: false }.is_complete());
    assert!(!ArmChannels { a: false, b: true }.is_complete());
    assert!(ArmChannels::BOTH.is_complete());

    let mut ctl = DeploymentController::default();
    for (arm, expected) in [
        (
            ArmChannels::UNARMED,
            Refusal::ArmIncomplete { a: false, b: false },
        ),
        (
            ArmChannels { a: true, b: false },
            Refusal::ArmIncomplete { a: true, b: false },
        ),
        (
            ArmChannels { a: false, b: true },
            Refusal::ArmIncomplete { a: false, b: true },
        ),
    ] {
        for altitude_m in [20, 19, 18] {
            assert_eq!(
                tick(&mut ctl, Phase::COAST, arm, altitude_m, -1_600),
                Decision::Refused(expected)
            );
        }
    }
    assert_eq!(ctl.refusals(), 9);
    assert_eq!(ctl.outputs(), SpringOutputs::SAFE);
    assert_eq!(ctl.descent_samples(), 0);

    // Refusals did not pre-load the descent run: three armed samples are needed.
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 18, -1_600),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 17, -1_600),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 16, -1_600),
        Decision::Deploy(Trigger::FastDescent)
    );
    assert_eq!(ctl.outputs(), SpringOutputs::RELEASED);
}

#[test]
fn apogee_descent_fires_only_after_a_verified_climb_and_drop() {
    let mut ctl = DeploymentController::default();
    armed(&mut ctl, Phase::COAST);
    assert_eq!(ctl.peak_altitude_m(), 60);

    assert_eq!(
        tick(&mut ctl, Phase::APOGEE, ArmChannels::BOTH, 60, 0),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::APOGEE, ArmChannels::BOTH, 59, -100),
        Decision::Hold
    );
    assert_eq!(ctl.descent_samples(), 1);
    assert_eq!(
        tick(&mut ctl, Phase::APOGEE, ArmChannels::BOTH, 58, -100),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::DESCENT, ArmChannels::BOTH, 57, -100),
        Decision::Deploy(Trigger::ApogeeDescent)
    );
    assert_eq!(ctl.trigger(), Some(Trigger::ApogeeDescent));
    assert!(ctl.outputs().is_released());
    assert!(ctl.outputs().a);
    assert!(ctl.outputs().b);
    assert_eq!(ctl.refusals(), 0);
}

#[test]
fn fast_descent_and_low_altitude_are_distinct_backup_triggers() {
    // 20 m is below the apogee floor and above the low altitude: only the
    // emergency descent-rate condition can fire here.
    let mut ctl = DeploymentController::default();
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 20, 0),
        Decision::Hold
    );
    for altitude_m in [20, 19, 18] {
        assert_eq!(
            tick(
                &mut ctl,
                Phase::COAST,
                ArmChannels::BOTH,
                altitude_m,
                -1_600
            ),
            if altitude_m == 18 {
                Decision::Deploy(Trigger::FastDescent)
            } else {
                Decision::Hold
            }
        );
    }
    assert_eq!(ctl.trigger(), Some(Trigger::FastDescent));

    // A slow descent near the ground takes the low-altitude path instead.
    let mut backup = DeploymentController::default();
    assert_eq!(
        tick(&mut backup, Phase::DESCENT, ArmChannels::BOTH, 12, -50),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut backup, Phase::DESCENT, ArmChannels::BOTH, 11, -50),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut backup, Phase::DESCENT, ArmChannels::BOTH, 10, -50),
        Decision::Deploy(Trigger::LowAltitude)
    );
    assert_eq!(backup.trigger(), Some(Trigger::LowAltitude));
}

#[test]
fn descent_confirm_suppresses_glitches_and_resets_on_climb() {
    let mut ctl = DeploymentController::default();
    armed(&mut ctl, Phase::COAST);
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 59, -100),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 60, 400),
        Decision::Hold
    );
    assert_eq!(ctl.descent_samples(), 0, "a climb resets the descent run");
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 59, -100),
        Decision::Hold
    );
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 58, -100),
        Decision::Hold
    );
    assert_eq!(ctl.descent_samples(), 2);
    assert_eq!(
        tick(&mut ctl, Phase::COAST, ArmChannels::BOTH, 57, -100),
        Decision::Deploy(Trigger::ApogeeDescent)
    );

    // Also holds when the speed sign is wrong even if the altitude is low.
    let mut low = DeploymentController::default();
    for _ in 0..5 {
        assert_eq!(
            tick(&mut low, Phase::COAST, ArmChannels::BOTH, 5, 10),
            Decision::Hold
        );
    }
    assert_eq!(low.descent_samples(), 0);
    assert_eq!(low.outputs(), SpringOutputs::SAFE);
}

#[test]
fn release_is_one_shot_and_later_samples_cannot_retract_it() {
    let mut ctl = DeploymentController::default();
    let mut fired = false;
    for altitude_m in (1..=20).rev() {
        let decision = tick(&mut ctl, Phase::DESCENT, ArmChannels::BOTH, altitude_m, -50);
        if !fired && decision == Decision::Deploy(Trigger::LowAltitude) {
            fired = true;
        }
    }
    assert!(fired);
    let refusals = ctl.refusals();

    // A release is never undone, even by the most extreme bad input.
    for (phase, arm) in [
        (Phase::BURN, ArmChannels::UNARMED),
        (Phase::LANDED, ArmChannels::UNARMED),
        (Phase::COAST, ArmChannels::BOTH),
    ] {
        assert_eq!(tick(&mut ctl, phase, arm, 0, 5_000), Decision::Hold);
        assert_eq!(ctl.outputs(), SpringOutputs::RELEASED);
    }
    assert_eq!(ctl.refusals(), refusals);
    assert_eq!(ctl.trigger(), Some(Trigger::LowAltitude));

    // Only an explicit ground-side reset returns to the safe state.
    ctl.reset();
    assert_eq!(ctl, DeploymentController::default());
    assert_eq!(ctl.outputs(), SpringOutputs::SAFE);
}

#[test]
fn coast_apogee_and_descent_phases_all_allow_release() {
    for phase in [Phase::COAST, Phase::APOGEE, Phase::DESCENT] {
        let mut ctl = DeploymentController::default();
        for altitude_m in [12, 11, 10] {
            let decision = tick(&mut ctl, phase, ArmChannels::BOTH, altitude_m, -50);
            if altitude_m == 10 {
                assert_eq!(
                    decision,
                    Decision::Deploy(Trigger::LowAltitude),
                    "{phase:?}"
                );
            } else {
                assert_eq!(decision, Decision::Hold, "{phase:?}");
            }
        }
        assert_eq!(ctl.phase(), phase);
    }
}

#[test]
fn identical_input_sequences_produce_identical_decisions() {
    let run = |arm: ArmChannels| {
        let mut ctl = DeploymentController::default();
        let samples = [
            (Phase::BURN, 100, 300),
            (Phase::COAST, 20, 300),
            (Phase::COAST, 40, 100),
            (Phase::APOGEE, 60, 0),
            (Phase::APOGEE, 59, -100),
            (Phase::DESCENT, 58, -100),
            (Phase::DESCENT, 57, -100),
            (Phase::LANDED, 1, -50),
        ];
        let decisions: Vec<Decision> = samples
            .iter()
            .map(|(phase, altitude_m, speed)| tick(&mut ctl, *phase, arm, *altitude_m, *speed))
            .collect();
        (ctl, decisions)
    };

    let (a, da) = run(ArmChannels::BOTH);
    let (b, db) = run(ArmChannels::BOTH);
    assert_eq!(da, db);
    assert_eq!(a, b);
    assert_eq!(a.trigger(), Some(Trigger::ApogeeDescent));
    assert_eq!(a.outputs(), SpringOutputs::RELEASED);
    assert_eq!(a.refusals(), 1);

    let (c, dc) = run(ArmChannels { a: true, b: false });
    assert_ne!(dc, da);
    assert_eq!(c.outputs(), SpringOutputs::SAFE);
    assert_eq!(c.refusals(), 8);
}
