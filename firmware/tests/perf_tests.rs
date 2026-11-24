// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the control-loop fast paths and both cycle models.
//!
//! `lib.rs` is owned by another lane; the module is included by path so the
//! exact firmware source is exercised. The re-exports make `crate::Command`
//! and `crate::MAX_CORRECTION_RAW` inside it resolve to the real types.
//! The production model is driven by actually calling `control::step` with a
//! mock board; the legacy SIL reservation model is kept pinned because
//! `timing_budget.rs` (outside this task's file scope) consumes it.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/perf.rs"]
mod perf;

use perf::{
    ai_gate, clamp_raw, scale_command, CycleEstimate, StepCounts, StepCycles, AI_ALLOWED_BY_PHASE,
    CORE_HZ, CYCLE_BUDGET, CYCLE_BUDGET_US, Q15, REPLY_CRC_BYTES, REQUEST_CRC_BYTES, SHIELD_CYCLES,
    TICK_PERIOD_US, WATCHDOG_CYCLES,
};
use potok_m_firmware::phase::{PhaseEvent, PhaseFsm};
use potok_m_firmware::{
    control::step, interlocks::Interlocks, mock::MockBoard, watchdog::WatchdogPolicy, LinkState,
    StepResult,
};
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

/// Pins the legacy SIL reservation model, still consumed by `timing_budget.rs`.
/// Its 132-cycle `quantize` term is a phantom: no shipped function quantizes.
/// The authoritative shipped-path model is pinned below.
#[test]
fn cycle_model_is_pinned_and_inside_the_budget() {
    assert_eq!(TICK_PERIOD_US, 2_000);
    assert_eq!(CORE_HZ, 550_000_000);
    assert_eq!(CYCLE_BUDGET_US, 50);
    assert_eq!(CYCLE_BUDGET, 27_500);

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

fn healthy_reply(seq: u16) -> potok_protocol::ReplyFrame {
    potok_protocol::ReplyFrame {
        seq,
        g_pitch: 50,
        g_yaw: -50,
        confidence: 200,
        ai_healthy: 1,
        version: 1,
    }
}

/// One real production tick; `COAST` keeps the accepted path from being vetoed
/// by the initial `BURN` phase, and fresh state keeps counts steady.
fn step_once(frame: Option<potok_protocol::ReplyFrame>) -> (StepResult, MockBoard) {
    let mut board = MockBoard::new();
    board.sensor_flags = flags::IMU_OK;
    let mut fsm = PhaseFsm::new();
    fsm.apply(PhaseEvent::Burnout).unwrap();
    let mut link = LinkState::new();
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    if let Some(reply) = frame {
        board.queue_reply(&reply);
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

#[test]
fn production_step_model_has_no_quantize_item() {
    let counts = StepCounts {
        rx_bytes: REPLY_SIZE as u32,
        crc_bytes: REQUEST_CRC_BYTES + REPLY_CRC_BYTES,
        phase_gate_calls: 1,
        shield_calls: 1,
        watchdog_calls: 1,
        clamped_axes: 2,
    };
    let est = StepCycles::new(counts);
    assert_eq!(est.rx_scan, 14 * 6);
    assert_eq!(est.crc, 66 * 4);
    assert_eq!(est.phase_gate, 12);
    assert_eq!(est.shield, SHIELD_CYCLES);
    assert_eq!(est.watchdog, WATCHDOG_CYCLES);
    assert_eq!(est.command_clamp, 8);
    assert_eq!(est.overhead, 200);
    assert_eq!(est.cycles(), 608);
    est.assert_budget("accepted");
}

#[test]
fn shipped_step_counts_drive_the_production_model() {
    let (result, board) = step_once(None);
    assert_eq!(result, StepResult::Idle);
    assert_eq!(board.rx().swaps, 0);
    assert_eq!(board.sent_frames().len(), 1);
    assert_eq!(board.applied.len(), 1);
    assert_eq!(board.pets(), 1);
    let idle = StepCounts {
        rx_bytes: REPLY_SIZE as u32 * board.rx().swaps as u32,
        crc_bytes: REQUEST_CRC_BYTES,
        phase_gate_calls: 0,
        shield_calls: 1,
        watchdog_calls: 1,
        clamped_axes: 0,
    };
    assert_eq!(StepCycles::new(idle).cycles(), 456);

    let (result, board) = step_once(Some(healthy_reply(1)));
    assert_eq!(result, StepResult::FrameRejected);
    assert_eq!(board.rx().swaps, 1);
    assert_eq!(board.sent_frames().len(), 1);
    assert_eq!(board.applied.len(), 1);
    assert_eq!(board.pets(), 1);
    let rejected = StepCounts {
        rx_bytes: REPLY_SIZE as u32,
        crc_bytes: REQUEST_CRC_BYTES + REPLY_CRC_BYTES,
        phase_gate_calls: 0,
        shield_calls: 1,
        watchdog_calls: 1,
        clamped_axes: 0,
    };
    assert_eq!(StepCycles::new(rejected).cycles(), 588);

    let (result, board) = step_once(Some(healthy_reply(0)));
    assert!(matches!(result, StepResult::Accepted(_)));
    assert_eq!(board.rx().swaps, 1);
    assert_eq!(board.sent_frames().len(), 1);
    assert_eq!(board.applied.len(), 1);
    assert_eq!(board.pets(), 1);
    let accepted = StepCounts {
        rx_bytes: REPLY_SIZE as u32,
        crc_bytes: REQUEST_CRC_BYTES + REPLY_CRC_BYTES,
        phase_gate_calls: 1,
        shield_calls: 1,
        watchdog_calls: 1,
        clamped_axes: 2,
    };
    assert_eq!(StepCycles::new(accepted).cycles(), 608);
}

#[test]
#[should_panic(expected = "cycle budget exceeded: flood")]
fn production_budget_panics_on_overrun() {
    StepCycles::new(StepCounts {
        rx_bytes: 10_000 * REPLY_SIZE as u32,
        ..StepCounts::default()
    })
    .assert_budget("flood");
}
