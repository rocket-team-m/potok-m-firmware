// SPDX-License-Identifier: GPL-3.0-or-later
//! Decisive unit gates for redundant-sensor voting.
//!
//! These tests use the library's public `sensor_voting` module — the exact
//! source the target compiles — rather than a second path-included copy.
//! The gates are about dimensional separation, not about confidence values:
//! a gyro fault must be visible when the accelerometer is quiet, an
//! accelerometer fault must be visible when the gyro is quiet, and one shared
//! threshold cannot do both. Source/method: `docs/MATH_VOTING.md`.

use potok_m_firmware::sensor_voting::{Config, ImuSample, Tolerance, Vote, VotingChannel};

const GYRO_QUIET: [f32; 3] = [0.0, 0.0, 0.0];
const ACCEL_QUIET: [f32; 3] = [0.0, 9.81, 0.0];

fn imu(gyro: [f32; 3], accel: [f32; 3]) -> ImuSample {
    ImuSample { gyro, accel }
}

/// HEURISTIC starting values, stated with their units: gyro rad/s, accel
/// m/s². The thresholds themselves are not derived; the *separation* they
/// define is what these gates check.
fn typed() -> Tolerance {
    Tolerance::imu(0.05, 0.02, 0.20, 0.02)
}

fn dual(tol: Tolerance, a: ImuSample, b: ImuSample) -> Vote<ImuSample> {
    VotingChannel::<ImuSample>::new(Config::DEFAULT, tol)
        .vote_dual(Some(a), Some(b))
        .unwrap()
}

#[test]
fn gyro_only_fault_is_detected_when_accel_is_quiet() {
    let good = imu(GYRO_QUIET, ACCEL_QUIET);
    let noisy = imu([0.02, 0.0, 0.0], ACCEL_QUIET);
    // +0.30 rad/s on one gyro axis; all three accel axes are bit-identical.
    let fault = imu([0.30, 0.0, 0.0], ACCEL_QUIET);

    let v = VotingChannel::<ImuSample>::new(Config::DEFAULT, typed())
        .vote_triple(Some(good), Some(noisy), Some(fault))
        .unwrap();

    assert!(v.disagreement, "gyro fault hidden by a quiet accelerometer");
    assert_eq!(v.outlier, Some(2));
    assert_eq!(v.value.accel, ACCEL_QUIET);
    assert_ne!(v.value.gyro[0], fault.gyro[0]);
    assert!((v.value.gyro[0] - 0.02).abs() < 1e-6);
}

#[test]
fn accel_only_fault_is_detected_when_gyro_is_quiet() {
    let good = imu(GYRO_QUIET, ACCEL_QUIET);
    let noisy = imu(GYRO_QUIET, [0.05, 9.81, 0.0]);
    // +0.80 m/s² on one accel axis; all three gyro axes are bit-identical.
    let fault = imu(GYRO_QUIET, [0.80, 9.81, 0.0]);

    let v = VotingChannel::<ImuSample>::new(Config::DEFAULT, typed())
        .vote_triple(Some(good), Some(noisy), Some(fault))
        .unwrap();

    assert!(v.disagreement, "accel fault hidden by a quiet gyro");
    assert_eq!(v.outlier, Some(2));
    assert_eq!(v.value.gyro, GYRO_QUIET);
    assert_ne!(v.value.accel[0], fault.accel[0]);
    assert!((v.value.accel[0] - 0.05).abs() < 1e-6);
}

#[test]
fn single_mixed_threshold_cannot_pass() {
    let quiet = imu(GYRO_QUIET, ACCEL_QUIET);
    // 0.10 rad/s is a gyro fault at the typed thresholds below; 0.25 m/s² is
    // ordinary accel movement (vehicle vibration), not a fault.
    let gyro_fault = imu([0.10, 0.0, 0.0], ACCEL_QUIET);
    let accel_noise = imu(GYRO_QUIET, [0.25, 9.81, 0.0]);
    let accel_fault = imu(GYRO_QUIET, [0.80, 9.81, 0.0]);

    // The retired expression, kept here only as a counterexample:
    // max(raw gyro Δ [rad/s], raw accel Δ [m/s²]) divided by one limit whose
    // relative part is fed by max(|gyro|, |accel|) — gravity contaminates the
    // gyro limit. This is F2, not a path the firmware still takes.
    let legacy = |a: ImuSample, b: ImuSample, tol: Tolerance| -> f32 {
        let mut g = 0.0f32;
        let mut ac = 0.0f32;
        let mut scale = 0.0f32;
        for i in 0..3 {
            g = g.max((a.gyro[i] - b.gyro[i]).abs());
            ac = ac.max((a.accel[i] - b.accel[i]).abs());
            scale = scale
                .max(a.gyro[i].abs())
                .max(b.gyro[i].abs())
                .max(a.accel[i].abs())
                .max(b.accel[i].abs());
        }
        g.max(ac) / (tol.absolute + tol.relative * scale)
    };

    // The pre-fix IMU deployment tolerance (lib.rs:352). 0.02·9.81 adds
    // 0.196 rad/s to the gyro limit, so a 0.10 rad/s fault reads 0.406 < 1.
    let shared = Tolerance::new(0.05, 0.02);
    assert!(
        legacy(quiet, gyro_fault, shared) <= 1.0,
        "the mixed expression should miss the gyro fault"
    );
    assert!(
        dual(typed(), quiet, gyro_fault).disagreement,
        "per-channel thresholds must catch the same gyro fault"
    );

    // With a single absolute threshold and no relative term, catching the
    // gyro fault needs a < 0.10 while ignoring ordinary accel movement needs
    // a > 0.25. No value satisfies both; check the boundary candidates.
    for a in [0.09, 0.20, 0.40] {
        let t = Tolerance::new(a, 0.0);
        let gyro_ok = dual(t, quiet, gyro_fault).disagreement;
        let accel_ok = !dual(t, quiet, accel_noise).disagreement;
        assert!(
            !(gyro_ok && accel_ok),
            "one shared threshold {a} unexpectedly separated gyro and accel"
        );
    }

    // Only two independently chosen, unit-bearing thresholds pass all three.
    let split = Tolerance::imu(0.05, 0.0, 0.40, 0.0);
    assert!(dual(split, quiet, gyro_fault).disagreement);
    assert!(!dual(split, quiet, accel_noise).disagreement);
    assert!(dual(split, quiet, accel_fault).disagreement);
}
