// SPDX-License-Identifier: GPL-3.0-or-later
//! Decisive fixed-point gates for the Mahony explicit complementary filter.
//!
//! Equation numbers refer to Mahony, Hamel & Pflimlin, *Nonlinear
//! Complementary Filters on the Special Orthogonal Group*, IEEE TAC 53(5),
//! 2008, DOI 10.1109/TAC.2008.923738. The module is included by path so the
//! exact source compiled for the target is what these tests exercise.

#[path = "../src/attitude.rs"]
mod attitude;

use attitude::{dt_q20_from_us, FilterError, Gains, Mahony, Quat, Vec3, RATE, UNIT};

const DT_US: u32 = 2000;
const UP: Vec3 = Vec3::new(0, 0, UNIT as i32);
const NORTH: Vec3 = Vec3::new(UNIT as i32, 0, 0);

fn unit(x: f64) -> i32 {
    (x * UNIT as f64).round() as i32
}

fn rate(x: f64) -> i32 {
    (x * RATE as f64).round() as i32
}

fn as_unit(x: i32) -> f64 {
    x as f64 / UNIT as f64
}

fn as_rate(x: i32) -> f64 {
    x as f64 / RATE as f64
}

fn run(filter: &mut Mahony, steps: usize, gyro: Vec3, accel: Option<Vec3>, mag: Option<Vec3>) {
    for _ in 0..steps {
        filter.update(gyro, accel, mag, DT_US).unwrap();
    }
}

fn unit_norm(q: Quat) -> f64 {
    let n2 = q.w as i64 * q.w as i64
        + q.x as i64 * q.x as i64
        + q.y as i64 * q.y as i64
        + q.z as i64 * q.z as i64;
    (n2 as f64).sqrt() / UNIT as f64
}

/// Rotation angle of `q` about the body x axis (the tilt the accel can see).
fn tilt_x(q: Quat) -> f64 {
    2.0 * as_unit(q.x).atan2(as_unit(q.w))
}

#[test]
fn gyro_only_step_matches_quaternion_kinematics_eq45_and_keeps_unit_norm() {
    let mut filter = Mahony::default();
    // 600 steps * 2 ms * 0.5 rad/s = 0.6 rad; q = [cos(0.3), sin(0.3), 0, 0].
    run(&mut filter, 600, Vec3::new(0, 0, rate(0.5)), None, None);
    let q = filter.quaternion();
    assert!((as_unit(q.w) - 0.3_f64.cos()).abs() < 1e-3);
    assert!((as_unit(q.z) - 0.3_f64.sin()).abs() < 1e-3);
    assert_eq!(q.x, 0);
    assert_eq!(q.y, 0);
    assert!((unit_norm(q) - 1.0).abs() < 1e-7, "norm = {}", unit_norm(q));
    assert_eq!(filter.bias(), Vec3::ZERO, "no reference, no bias integral");
}

#[test]
fn known_gravity_tilt_reduces_attitude_error_eq32c_eq48b() {
    let alpha = 0.4_f64;
    let tilted = Vec3::new(0, unit(alpha.sin()), unit(alpha.cos()));
    let mut filter = Mahony::default(); // kp = 2, ki = 0.5 (positive paper gains)
    let mut errors = vec![tilt_x(filter.quaternion()) - alpha];

    for extra in [50, 150, 300, 500, 11000] {
        run(&mut filter, extra, Vec3::ZERO, Some(tilted), None);
        errors.push(tilt_x(filter.quaternion()) - alpha);
    }

    // The proportional term acts first: the first three checkpoints must
    // shrink monotonically. The integral term can overshoot once before the
    // pair settles, so only the settled error is compared at the tail.
    for pair in errors[..3].windows(2) {
        assert!(
            pair[1].abs() < pair[0].abs(),
            "attitude error did not shrink: {:?}",
            errors
        );
    }
    let q = filter.quaternion();
    assert!(
        (tilt_x(q) - alpha).abs() < 1e-2,
        "final tilt = {}, expected {}",
        tilt_x(q),
        alpha
    );
    assert!((unit_norm(q) - 1.0).abs() < 1e-7);
}

#[test]
fn constant_gyro_bias_static_gravity_converges_and_does_not_diverge() {
    // The bias is orthogonal to gravity; its z component is unobservable
    // from gravity alone and must stay at zero (Eq. 32c cross product).
    let true_bias = (0.08_f64, -0.04_f64);
    let gyro = Vec3::new(rate(true_bias.0), rate(true_bias.1), 0);
    let mut filter = Mahony::default();

    let mut snapshots = [(0.0_f64, 0.0_f64); 4];
    for (i, extra) in [1000, 3000, 8000, 8000].into_iter().enumerate() {
        run(&mut filter, extra, gyro, Some(UP), None);
        let b = filter.bias();
        snapshots[i] = (
            (as_rate(b.x) - true_bias.0).abs(),
            (as_rate(b.y) - true_bias.1).abs(),
        );
    }

    // Eq. 32b with a negative integral and Eq. 32a with a subtracted estimate
    // make b̂ → b. A flipped sign diverges; a frozen integral stays at zero.
    for pair in snapshots.windows(2) {
        assert!(
            pair[1].0 < pair[0].0 && pair[1].1 < pair[0].1,
            "bias error did not shrink: {:?}",
            snapshots
        );
        assert!(
            pair[1].0 <= 2.0 && pair[1].1 <= 2.0,
            "bias estimate diverged: {:?}",
            snapshots
        );
    }
    let b = filter.bias();
    assert!(
        (as_rate(b.x) - true_bias.0).abs() < 5e-3,
        "bx = {}, expected {}",
        as_rate(b.x),
        true_bias.0
    );
    assert!(
        (as_rate(b.y) - true_bias.1).abs() < 5e-3,
        "by = {}, expected {}",
        as_rate(b.y),
        true_bias.1
    );
    assert_eq!(b.z, 0, "yaw bias is not observable from gravity alone");
    let q = filter.quaternion();
    assert!(as_unit(q.x).abs() < 1e-2 && as_unit(q.y).abs() < 1e-2);
    assert!((unit_norm(q) - 1.0).abs() < 1e-7);
}

#[test]
fn constant_gyro_bias_with_gravity_and_north_converges_on_all_axes() {
    let truth = (0.06_f64, -0.03_f64, 0.04_f64);
    let gyro = Vec3::new(rate(truth.0), rate(truth.1), rate(truth.2));
    let mut filter = Mahony::default();

    run(&mut filter, 12000, gyro, Some(UP), Some(NORTH));
    let b = filter.bias();
    assert!(
        (as_rate(b.x) - truth.0).abs() < 5e-3,
        "bx = {}",
        as_rate(b.x)
    );
    assert!(
        (as_rate(b.y) - truth.1).abs() < 5e-3,
        "by = {}",
        as_rate(b.y)
    );
    assert!(
        (as_rate(b.z) - truth.2).abs() < 5e-3,
        "bz = {}",
        as_rate(b.z)
    );

    let q = filter.quaternion();
    assert!(as_unit(q.x).abs() < 1e-2);
    assert!(as_unit(q.y).abs() < 1e-2);
    assert!(as_unit(q.z).abs() < 1e-2);
    assert!((unit_norm(q) - 1.0).abs() < 1e-7);
}

#[test]
fn custom_gains_survive_reset_without_changing_sign_conventions() {
    let mut filter = Mahony::default();
    run(&mut filter, 300, Vec3::new(rate(0.4), 0, 0), Some(UP), None);

    let custom = Gains::new(rate(3.0), rate(1.0), rate(1.0), 0);
    filter.set_gains(custom);
    assert_eq!(filter.gains(), custom);

    filter.reset();
    assert_eq!(filter.quaternion(), Quat::IDENTITY);
    assert_eq!(filter.bias(), Vec3::ZERO);
    assert_eq!(filter.gains(), custom);

    run(
        &mut filter,
        12000,
        Vec3::new(rate(0.05), 0, 0),
        Some(UP),
        None,
    );
    // Same sign convention with custom positive gains: b̂ → +b.
    assert!((as_rate(filter.bias().x) - 0.05).abs() < 5e-3);
}

#[test]
fn zero_dt_and_degenerate_references_are_loud_without_mutating_state() {
    let mut filter = Mahony::new(Gains::default());
    // Reach a non-trivial state first: a refused step must not reset it.
    run(
        &mut filter,
        400,
        Vec3::new(rate(0.3), 0, 0),
        Some(UP),
        Some(NORTH),
    );
    let q0 = filter.quaternion();
    let b0 = filter.bias();
    assert_ne!(q0, Quat::IDENTITY);

    assert_eq!(
        filter.update(Vec3::ZERO, Some(UP), Some(NORTH), 0),
        Err(FilterError::ZeroDt)
    );
    assert_eq!(
        filter.update(Vec3::ZERO, Some(Vec3::ZERO), Some(NORTH), DT_US),
        Err(FilterError::DegenerateAccel)
    );
    assert_eq!(
        filter.update(Vec3::ZERO, Some(UP), Some(Vec3::ZERO), DT_US),
        Err(FilterError::DegenerateMag)
    );
    // A disabled reference must not let its degenerate twin through.
    assert_eq!(
        filter.update(Vec3::ZERO, None, Some(Vec3::ZERO), DT_US),
        Err(FilterError::DegenerateMag)
    );
    assert_eq!(dt_q20_from_us(0), 0);

    assert_eq!(filter.quaternion(), q0);
    assert_eq!(filter.bias(), b0);
}
