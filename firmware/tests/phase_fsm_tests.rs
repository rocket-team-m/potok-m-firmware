// SPDX-License-Identifier: GPL-3.0-or-later
//! Host-side gates for the strict, forward-only flight phase FSM.

#[path = "../src/phase_fsm.rs"]
mod phase_fsm;

use phase_fsm::{PhaseError, PhaseEvent, PhaseFsm};
use potok_protocol::{flags, Phase};

fn forward_walk() -> [(PhaseEvent, Phase, u64); 4] {
    [
        (PhaseEvent::Burnout, Phase::COAST, 1_500),
        (PhaseEvent::Apogee, Phase::APOGEE, 2_000),
        (PhaseEvent::Descent, Phase::DESCENT, 2_500),
        (PhaseEvent::Touchdown, Phase::LANDED, 3_000),
    ]
}

fn fsm_at(phase: Phase) -> PhaseFsm {
    let mut fsm = PhaseFsm::new_at(100);
    for (event, target, at) in forward_walk() {
        if fsm.phase() == phase {
            break;
        }
        fsm.apply(event, at).unwrap();
        assert_eq!(fsm.phase(), target);
    }
    assert_eq!(fsm.phase(), phase);
    fsm
}

#[test]
fn burn_is_the_safe_initial_phase() {
    let fsm = PhaseFsm::new();
    assert_eq!(fsm.phase(), Phase::BURN);
    assert_eq!(fsm.started_at_us(), 0);
    assert_eq!(fsm.entered_at_us(), 0);
    assert_eq!(fsm.transition_count(), 0);
    assert_eq!(fsm.last_transition(), None);
    assert_eq!(fsm.rejected_count(), 0);
    assert_eq!(PhaseFsm::default().phase(), Phase::BURN);

    let all_sensors = flags::GPS_FIX
        | flags::IMU_OK
        | flags::BARO_OK
        | flags::PITOT_OK
        | flags::AI_READY
        | flags::CALIBRATED;
    assert!(!fsm.ai_allowed(all_sensors), "BURN always gates AI off");
}

#[test]
fn forward_edges_are_committed_with_timestamps() {
    let mut fsm = PhaseFsm::new_at(1_000);
    let mut previous_from = Phase::BURN;

    for (event, expected, at) in forward_walk() {
        let transition = fsm.apply(event, at).unwrap();
        assert_eq!(transition.from, previous_from);
        assert_eq!(transition.to, expected);
        assert_eq!(transition.event, event);
        assert_eq!(transition.at_us, at);
        assert_eq!(fsm.phase(), expected);
        assert_eq!(fsm.entered_at_us(), at);
        assert_eq!(fsm.transition_into(expected), Some(transition));
        assert!(u8::from(expected) > u8::from(previous_from));
        previous_from = expected;
    }

    assert_eq!(fsm.started_at_us(), 1_000);
    assert_eq!(fsm.transition_count(), 4);
    assert_eq!(fsm.transition_into(Phase::BURN), None);
    assert_eq!(fsm.last_transition().unwrap().event, PhaseEvent::Touchdown);
    assert_eq!(fsm.last_transition().unwrap().at_us, 3_000);
    assert_eq!(fsm.rejected_count(), 0);
}

#[test]
fn illegal_events_are_refused_without_moving_the_machine() {
    // All 25 (phase, event) combinations: exactly the five declared edges are legal.
    let phases = [
        Phase::BURN,
        Phase::COAST,
        Phase::APOGEE,
        Phase::DESCENT,
        Phase::LANDED,
    ];
    let events = [
        PhaseEvent::LiftOff,
        PhaseEvent::Burnout,
        PhaseEvent::Apogee,
        PhaseEvent::Descent,
        PhaseEvent::Touchdown,
    ];

    for from in phases {
        for event in events {
            let legal = matches!(
                (from, event),
                (Phase::BURN, PhaseEvent::LiftOff)
                    | (Phase::BURN, PhaseEvent::Burnout)
                    | (Phase::COAST, PhaseEvent::Apogee)
                    | (Phase::APOGEE, PhaseEvent::Descent)
                    | (Phase::DESCENT, PhaseEvent::Touchdown)
            );
            let mut fsm = fsm_at(from);
            let before = fsm;
            let result = fsm.apply(event, 10_000);

            assert_eq!(result.is_ok(), legal, "{event:?} from {from:?}");
            if legal {
                continue;
            }
            assert_eq!(result, Err(PhaseError::IllegalTransition { from, event }));
            assert_eq!(fsm.phase(), before.phase());
            assert_eq!(fsm.entered_at_us(), before.entered_at_us());
            assert_eq!(fsm.transition_count(), before.transition_count());
            assert_eq!(fsm.rejected_count(), before.rejected_count() + 1);
        }
    }
}

#[test]
fn timestamps_must_not_go_backwards() {
    let mut fsm = PhaseFsm::new_at(100);
    assert_eq!(
        fsm.apply(PhaseEvent::Burnout, 99),
        Err(PhaseError::TimestampWentBackwards {
            now_us: 99,
            entered_at_us: 100,
        })
    );
    assert_eq!(fsm.phase(), Phase::BURN);
    assert_eq!(fsm.entered_at_us(), 100);
    assert_eq!(fsm.transition_count(), 0);
    assert_eq!(fsm.rejected_count(), 1);

    // Equal to the current entry is still forward (non-decreasing).
    fsm.apply(PhaseEvent::Burnout, 100).unwrap();
    assert_eq!(fsm.phase(), Phase::COAST);
    assert_eq!(fsm.entered_at_us(), 100);
    assert_eq!(fsm.transition_into(Phase::COAST).unwrap().at_us, 100);
}

#[test]
fn ai_is_gated_by_burn_and_by_the_motor_flag() {
    let healthy = flags::IMU_OK | flags::AI_READY | flags::CALIBRATED;
    let mut fsm = PhaseFsm::new();
    assert!(!fsm.ai_allowed(healthy));
    assert!(!fsm.ai_allowed(healthy | flags::MOTOR_ARMED));

    for (event, phase, at) in forward_walk() {
        fsm.apply(event, at).unwrap();
        if phase == Phase::LANDED {
            continue;
        }
        assert!(fsm.ai_allowed(healthy), "{phase:?} allows AI");
        assert!(
            !fsm.ai_allowed(healthy | flags::MOTOR_ARMED),
            "motor flag independently gates {phase:?}"
        );
    }
}

#[test]
fn liftoff_marks_burn_without_leaving_it() {
    let mut fsm = PhaseFsm::new_at(0);
    let first = fsm.apply(PhaseEvent::LiftOff, 250).unwrap();
    assert_eq!(first.from, Phase::BURN);
    assert_eq!(first.to, Phase::BURN);
    assert_eq!(fsm.phase(), Phase::BURN);
    assert_eq!(fsm.entered_at_us(), 250);
    assert_eq!(fsm.transition_count(), 1);
    assert_eq!(fsm.transition_into(Phase::BURN), Some(first));
    assert!(!fsm.ai_allowed(flags::AI_READY), "still BURN, still gated");

    // A repeated liftoff only refreshes BURN's own slot; it is not a new phase.
    let second = fsm.apply(PhaseEvent::LiftOff, 300).unwrap();
    assert_eq!(second.at_us, 300);
    assert_eq!(fsm.transition_count(), 1);
    assert_eq!(fsm.transition_into(Phase::BURN), Some(second));
    assert_eq!(fsm.phase(), Phase::BURN);
}

#[test]
fn landed_is_terminal() {
    let mut fsm = fsm_at(Phase::LANDED);
    assert_eq!(fsm.phase(), Phase::LANDED);
    assert_eq!(
        fsm.apply(PhaseEvent::Touchdown, 20_000)
            .unwrap_err()
            .to_string(),
        "phase event Touchdown is illegal from LANDED"
    );
    assert_eq!(fsm.phase(), Phase::LANDED);
    assert_eq!(fsm.entered_at_us(), 3_000);
    assert_eq!(fsm.transition_count(), 4);
}
