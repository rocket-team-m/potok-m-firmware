// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the control-loop fast paths and the cycle model.
//!
//! `lib.rs` is owned by another lane; the module is included by path so the
//! exact firmware source is exercised. The re-exports make `crate::Command`
//! and `crate::MAX_CORRECTION_RAW` inside it resolve to the real types.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/perf.rs"]
mod perf;

use perf::{
    ai_gate, clamp_raw, scale_command, CycleEstimate, AI_ALLOWED_BY_PHASE, CORE_HZ, CYCLE_BUDGET,
    CYCLE_BUDGET_US, Q15, TICK_PERIOD_US,
};
use potok_m_firmware::phase::{PhaseEvent, PhaseFsm};
use potok_protocol::{flags, Phase, REPLY_SIZE, REQUEST_SIZE};

/// Number of scaled fields in a compat request: 3 gyro, 3 accel, 3 mag,
/// height, pitot.
const QUANT_FIELDS: u32 = 11;

fn phase_with_events(events: &[PhaseEvent]) -> PhaseFsm {
    let mut fsm = PhaseFsm::new();
    for &event in events {
        fsm.apply(event).unwrap();
    }
    fsm
}

#[test]
fn phase_gate_table_matches_the_fsm_for_every_phase() {
    let stages: [(Phase, &[PhaseEvent]); 5] = [
        (Phase::BURN, &[]),
        (Phase::BURN, &[PhaseEvent::LiftOff]),
        (Phase::COAST, &[PhaseEvent::Burnout]),
        (Phase::APOGEE, &[PhaseEvent::Burnout, PhaseEvent::Apogee]),
        (
            Phase::DESCENT,
            &[PhaseEvent::Burnout, PhaseEvent::Apogee, PhaseEvent::Descent],
        ),
    ];
    let flag_sets = [
        0,
        flags::IMU_OK,
        flags::MOTOR_ARMED,
        flags::IMU_OK | flags::MOTOR_ARMED,
    ];
    for (phase, events) in stages {
        let fsm = phase_with_events(events);
        assert_eq!(fsm.phase(), phase);
        for &sensor_flags in &flag_sets {
            assert_eq!(
                ai_gate(phase, sensor_flags, false, false),
                fsm.ai_allowed(sensor_flags),
                "phase {phase:?} flags {sensor_flags:#04x}"
            );
            assert!(!ai_gate(phase, sensor_flags, true, false));
            assert!(!ai_gate(phase, sensor_flags, false, true));
        }
    }

    // Undefined phase bytes fail closed; no bounds branch in the hot path.
    for code in 0u8..=u8::MAX {
        let expected = (1..=Phase::LANDED as u8).contains(&code);
        assert_eq!(AI_ALLOWED_BY_PHASE[code as usize], expected, "code {code}");
    }
}

#[test]
fn q15_identity_and_half_are_exact_and_symmetric() {
    let inputs = [
        i16::MIN,
        -30_000,
        -20_000,
        -10_000,
        -5_000,
        -1_000,
        -101,
        -1,
        0,
        1,
        101,
        1_000,
        5_000,
        10_000,
        20_000,
        30_000,
        i16::MAX,
    ];
    for &value in &inputs {
        assert_eq!(Q15::ONE.apply(value), value, "identity at {value}");
        assert_eq!(Q15::ZERO.apply(value), 0);
    }
    assert_eq!(Q15::HALF.apply(101), 51);
    assert_eq!(Q15::HALF.apply(-101), -51);
    assert_eq!(Q15::from_ratio(1, 4).apply(100), 25);
    assert_eq!(Q15::from_ratio(-1, 4).apply(100), -25);
}

#[test]
fn fixed_point_scaling_matches_the_code_veto() {
    for &value in &[
        i16::MIN,
        -1000,
        -101,
        -100,
        -1,
        0,
        1,
        100,
        101,
        1000,
        i16::MAX,
    ] {
        let cmd = scale_command(value, value, Q15::ONE);
        let expected = Command {
            pitch: value,
            yaw: value,
        }
        .clamped();
        assert_eq!(cmd, expected, "clamp at {value}");
        assert_eq!(clamp_raw(value as i32), expected.pitch);
    }
    assert_eq!(
        scale_command(50, -50, Q15::HALF),
        Command {
            pitch: 25,
            yaw: -25
        }
    );
    assert_eq!(
        scale_command(1000, -1000, Q15::from_ratio(1, 4)),
        Command {
            pitch: 100,
            yaw: -100
        }
    );
}

#[test]
#[should_panic(expected = "Q15 denominator must be positive")]
fn q15_refuses_zero_denominator() {
    let _ = Q15::from_ratio(1, 0);
}

#[test]
#[should_panic(expected = "Q15 gain out of range")]
fn q15_refuses_oversized_gain() {
    let _ = Q15::from_ratio(4, 1);
}

#[test]
fn cycle_model_is_pinned_and_inside_the_budget() {
    assert_eq!(TICK_PERIOD_US, 2_000);
    assert_eq!(CORE_HZ, 480_000_000);
    assert_eq!(CYCLE_BUDGET_US, 50);
    assert_eq!(CYCLE_BUDGET, 24_000);

    let accepted = CycleEstimate::new(
        REPLY_SIZE as u32,
        (REQUEST_SIZE + REPLY_SIZE) as u32,
        QUANT_FIELDS,
        2,
        true,
    );
    assert_eq!(accepted.rx_scan, 14 * 6);
    assert_eq!(accepted.crc, 70 * 4);
    assert_eq!(accepted.quantize, QUANT_FIELDS * 12);
    assert_eq!(accepted.gate, 12);
    assert_eq!(accepted.fixed_math, 8);
    assert_eq!(accepted.cycles(), 716);
    accepted.assert_budget("accepted");

    let rejected = CycleEstimate::new(
        REPLY_SIZE as u32,
        (REQUEST_SIZE + REPLY_SIZE) as u32,
        QUANT_FIELDS,
        0,
        true,
    );
    assert_eq!(rejected.cycles(), 708);
    assert!(rejected.cycles() < accepted.cycles());

    let idle = CycleEstimate::new(0, REQUEST_SIZE as u32, QUANT_FIELDS, 0, false);
    assert_eq!(idle.cycles(), 556);
    assert!(idle.cycles() < rejected.cycles());
}

#[test]
#[should_panic(expected = "cycle budget exceeded: flood")]
fn cycle_budget_panics_on_overrun() {
    CycleEstimate::new(10_000 * REPLY_SIZE as u32, 0, 0, 0, false).assert_budget("flood");
}
