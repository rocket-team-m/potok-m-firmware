// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the deterministic, hardware-free preflight self-test.
//!
//! The module is included by path because `lib.rs` is owned by the parent task;
//! every observation is supplied here by a fixture, so no board, pin or clock
//! is involved.

#[path = "../src/self_test.rs"]
mod self_test;

use potok_protocol::crc16;
use self_test::*;

static CALIBRATION: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
static GAINS: [u8; 4] = [9, 8, 7, 6];

const CALIBRATION_CRC: u16 = 0x4792;
const GAINS_CRC: u16 = 0x2747;

fn nominal_inputs() -> Inputs<'static> {
    // Descriptor array is static; the bytes it points to are static too.
    static TABLES: [TableSpec<'static>; 2] = [
        TableSpec {
            name: "calibration",
            bytes: &CALIBRATION,
            expected_crc: CALIBRATION_CRC,
        },
        TableSpec {
            name: "gains",
            bytes: &GAINS,
            expected_crc: GAINS_CRC,
        },
    ];
    Inputs {
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
        tables: &TABLES,
        config: GOLDEN_CONFIG,
        config_crc: GOLDEN_CONFIG_CRC,
        continuity: ContinuityMock::healthy(),
        latch_state: PhaseLatch::new().readback(),
        ticker: Ticker::new(1_000, 250),
    }
}

fn run(inputs: Inputs<'_>) -> SelfTestReport {
    SelfTest::new(inputs).run_to_completion(7_000)
}

fn spoil(channel: SensorChannel, sample: &mut SensorSample) {
    match channel {
        SensorChannel::Accel => sample.accel = 500,
        SensorChannel::Baro => sample.baro = -1,
        SensorChannel::Battery => sample.battery_mv = 9_999,
        SensorChannel::Temperature => sample.temp_c = 200,
    }
}

fn reason(report: SelfTestReport, id: CheckId) -> String {
    report
        .results()
        .iter()
        .find(|result| result.id() == id)
        .expect("every check has a result")
        .verdict()
        .to_string()
}

#[test]
fn golden_crc_literals_match_the_protocol_implementation() {
    assert_eq!(crc16(&GOLDEN_CONFIG.to_bytes()), GOLDEN_CONFIG_CRC);
    assert_eq!(crc16(&CALIBRATION), CALIBRATION_CRC);
    assert_eq!(crc16(&GAINS), GAINS_CRC);
    assert_eq!(GOLDEN_CONFIG.to_bytes(), [1, 0, 0x50, 0x4D, 100, 0, 80, 0]);
}

#[test]
fn nominal_preflight_passes_all_checks_in_a_fixed_number_of_ticks() {
    let report = run(nominal_inputs());
    assert!(report.done());
    assert!(report.all_passed(), "{report}");
    assert_eq!(report.passed_count(), CHECK_COUNT);
    assert_eq!(report.first_failure(), None);
    assert_eq!(report.ticks(), 9);
    assert_eq!(report.started_us(), 7_000);
    assert_eq!(report.finished_us(), 7_000);
    for result in report.results().iter() {
        assert!(result.passed(), "{result:?}");
    }
}

#[test]
fn steps_advance_one_bounded_slice_at_a_time_without_blocking() {
    let mut suite = SelfTest::new(nominal_inputs());
    let expected = [
        Progress::Running {
            check: CheckId::TableCrc,
            step: 0,
        },
        Progress::Running {
            check: CheckId::ConfigCrc,
            step: 0,
        },
        Progress::Running {
            check: CheckId::ActuatorContinuity,
            step: 0,
        },
        Progress::Running {
            check: CheckId::ActuatorContinuity,
            step: 1,
        },
        Progress::Running {
            check: CheckId::ActuatorContinuity,
            step: 2,
        },
        Progress::Running {
            check: CheckId::ActuatorContinuity,
            step: 3,
        },
        Progress::Running {
            check: CheckId::PhaseLatch,
            step: 0,
        },
        Progress::Running {
            check: CheckId::StateMachine,
            step: 0,
        },
        Progress::Done,
    ];
    for (index, want) in expected.iter().enumerate() {
        assert_eq!(suite.step(1_000 + index as u32), *want, "step {index}");
        assert_eq!(suite.report().ticks(), index as u32 + 1);
    }
    assert_eq!(suite.step(9_000), Progress::Done);
    assert_eq!(
        suite.report().ticks(),
        9,
        "a finished suite does no more work"
    );
}

#[test]
fn step_cursor_visits_every_check_exactly_once() {
    let mut cursor = StepCursor::new();
    let mut seen = [0u8; CHECK_COUNT];
    while !cursor.done() {
        seen[cursor.index()] += 1;
        cursor.advance();
    }
    assert_eq!(seen, [1; CHECK_COUNT]);
    cursor.advance();
    assert!(cursor.done(), "the cursor never wraps into a check twice");
}

#[test]
fn same_inputs_produce_the_same_report_twice() {
    let a = run(nominal_inputs());
    let mut b = SelfTest::new(nominal_inputs());
    // Different caller timestamps must not change a single verdict.
    for tick in 0..9 {
        b.step(50_000 + tick as u32 * 137);
    }
    let b = b.report();
    assert_eq!(a.results(), b.results());
    assert_eq!(a.ticks(), b.ticks());
    assert_eq!(a.to_string(), b.to_string());
    assert_ne!(a.started_us(), b.started_us());
}

#[test]
fn sensor_range_reason_names_channel_value_and_bounds() {
    let mut inputs = nominal_inputs();
    inputs.sensor.battery_mv = 2_500;
    let report = run(inputs);
    assert!(!report.all_passed());
    assert_eq!(report.first_failure().unwrap().id(), CheckId::SensorRange);
    assert_eq!(
        reason(report, CheckId::SensorRange),
        "battery reads 2500, outside configured range 3000..=4200"
    );
    assert!(report
        .to_string()
        .contains("[FAIL] sensor_range: battery reads"));
}

#[test]
fn every_sensor_channel_is_checked() {
    for channel in SENSOR_CHANNELS {
        let mut inputs = nominal_inputs();
        spoil(channel, &mut inputs.sensor);
        let report = run(inputs);
        let text = reason(report, CheckId::SensorRange);
        assert!(text.starts_with(&format!("{channel} reads")), "{text}");
    }
}

#[test]
fn table_crc_failure_reports_which_table_and_both_values() {
    let mut inputs = nominal_inputs();
    let corrupt = [TableSpec {
        name: "calibration",
        bytes: &CALIBRATION,
        expected_crc: CALIBRATION_CRC ^ 0x0001,
    }];
    inputs.tables = &corrupt;
    let report = run(inputs);
    assert_eq!(report.first_failure().unwrap().id(), CheckId::TableCrc);
    assert_eq!(
        reason(report, CheckId::TableCrc),
        format!(
            "calibration CRC mismatch: expected 0x{:04X}, got 0x{:04X}",
            CALIBRATION_CRC ^ 0x0001,
            CALIBRATION_CRC
        )
    );
    assert_eq!(report.ticks(), 9, "a failed check does not stall the suite");
}

#[test]
fn config_crc_failure_is_reported_separately_from_table_crc() {
    let mut inputs = nominal_inputs();
    inputs.config = ConfigImage {
        pitch_gain: 101,
        ..GOLDEN_CONFIG
    };
    let report = run(inputs);
    assert_eq!(report.first_failure().unwrap().id(), CheckId::ConfigCrc);
    assert!(reason(report, CheckId::ConfigCrc).starts_with("config CRC mismatch: expected"));
    assert!(reason(report, CheckId::TableCrc).starts_with("all calibration"));
}

#[test]
fn open_actuator_path_is_reported_by_role_not_by_pin() {
    let mut inputs = nominal_inputs();
    inputs.continuity = ContinuityMock::open(ActuatorChannel::Yaw);
    let report = run(inputs);
    assert_eq!(
        report.first_failure().unwrap().id(),
        CheckId::ActuatorContinuity
    );
    assert_eq!(
        reason(report, CheckId::ActuatorContinuity),
        "yaw actuator continuity never closed (open path)"
    );
}

#[test]
fn a_single_glitch_is_caught_by_the_debounce_window() {
    let mut inputs = nominal_inputs();
    inputs.continuity = ContinuityMock::glitch(ActuatorChannel::Pitch, 2);
    let report = run(inputs);
    assert_eq!(
        reason(report, CheckId::ActuatorContinuity),
        "pitch continuity unstable: 3 of 4 samples closed"
    );
}

#[test]
fn phase_latch_reasons_distinguish_burn_arm_and_sticky_failures() {
    let mut inputs = nominal_inputs();
    inputs.latch_state = LatchState {
        phase: LatchPhase::Burn,
        motor_armed: true,
        latched: true,
        ai_allowed: true,
    };
    let report = run(inputs);
    assert_eq!(report.first_failure().unwrap().id(), CheckId::PhaseLatch);
    assert_eq!(
        reason(report, CheckId::PhaseLatch),
        "phase latch allows AI during BURN"
    );

    inputs.latch_state = LatchState {
        phase: LatchPhase::Flight,
        motor_armed: true,
        latched: false,
        ai_allowed: true,
    };
    assert_eq!(
        reason(run(inputs), CheckId::PhaseLatch),
        "phase latch allows AI while the motor is armed"
    );

    inputs.latch_state = LatchState {
        phase: LatchPhase::Flight,
        motor_armed: false,
        latched: true,
        ai_allowed: true,
    };
    assert_eq!(
        reason(run(inputs), CheckId::PhaseLatch),
        "phase latch allows AI while still latched"
    );
}

#[test]
fn phase_latch_is_sticky_until_an_explicit_guarded_release() {
    let mut latch = PhaseLatch::new();
    assert!(!latch.readback().ai_allowed);
    assert!(!latch.release(), "release is refused in BURN");
    latch.burnout();
    latch.set_motor_armed(true);
    assert!(!latch.release(), "release is refused while armed");
    latch.set_motor_armed(false);
    assert!(
        !latch.readback().ai_allowed,
        "disarming alone does not clear the latch"
    );

    assert!(latch.release());
    assert!(latch.readback().ai_allowed);
    for _ in 0..LATCH_MISSES {
        latch.note_missed();
    }
    latch.note_valid();
    assert!(
        !latch.readback().ai_allowed,
        "three misses latch again and a valid reply does not clear it"
    );
    assert!(latch.release());
    assert!(latch.readback().ai_allowed);
}

#[test]
fn zero_interval_state_machine_is_a_loud_failure() {
    let mut inputs = nominal_inputs();
    inputs.ticker = Ticker::new(0, 0);
    let report = run(inputs);
    assert_eq!(report.first_failure().unwrap().id(), CheckId::StateMachine);
    assert_eq!(
        reason(report, CheckId::StateMachine),
        "state machine tick interval is zero; a caller would spin"
    );
    assert!(!Ticker::new(0, 0).poll(u32::MAX));
}

#[test]
fn ticker_poll_is_pure_and_survives_wraparound() {
    let ticker = Ticker::new(u32::MAX - 10, 20);
    assert!(!ticker.poll(u32::MAX - 10));
    assert!(!ticker.poll(u32::MAX));
    assert!(ticker.poll(9));
    assert!(
        ticker.poll(9),
        "polling twice does not consume the deadline"
    );
}

#[test]
fn unfinished_report_has_not_run_results_that_are_not_passes() {
    let mut suite = SelfTest::new(nominal_inputs());
    suite.step(0);
    let report = suite.report();
    assert!(!report.done());
    assert!(!report.all_passed());
    assert_eq!(
        report.passed_count(),
        1,
        "the completed sensor check is a pass"
    );
    let failure = report.first_failure().unwrap();
    assert_eq!(failure.id(), CheckId::TableCrc);
    assert_eq!(failure.verdict(), Verdict::NotRun);
    assert_eq!(failure.verdict().to_string(), "not run");
}

#[test]
fn multiple_faults_are_all_reported_in_check_order() {
    let mut inputs = nominal_inputs();
    inputs.sensor.accel = 1_000;
    inputs.config_crc = 0;
    inputs.continuity = ContinuityMock::open(ActuatorChannel::Pitch);
    inputs.ticker = Ticker::new(0, 0);
    let report = run(inputs);
    let failed: Vec<CheckId> = report
        .results()
        .iter()
        .filter(|result| !result.passed())
        .map(|result| result.id())
        .collect();
    assert_eq!(
        failed,
        vec![
            CheckId::SensorRange,
            CheckId::ConfigCrc,
            CheckId::ActuatorContinuity,
            CheckId::StateMachine,
        ]
    );
    assert_eq!(report.to_string().matches("[FAIL]").count(), 4);
}

#[test]
fn malformed_inputs_are_refused_loudly() {
    let mut inputs = nominal_inputs();
    inputs.limits.accel = Range { min: 10, max: -10 };
    // `new` is the boundary: a check must never silently reinterpret it.
    let result = std::panic::catch_unwind(move || SelfTest::new(inputs));
    assert!(result.is_err());
}
