// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the fixed-point Mahony filter.
//!
//! `lib.rs` is out of scope for this task, so the module under test is included
//! by path: the same source that would be wired into the crate is compiled and
//! exercised here without touching any other file.

#[path = "../src/attitude.rs"]
mod attitude;

use attitude::{dt_q20_from_us, FilterError, Gains, Mahony, Quat, Vec3, DT, GAIN, RATE, UNIT};

const DT_US: u32 = 2000;
const UP: Vec3 = Vec3::new(0, 0, UNIT as i32);
const NORTH: Vec3 = Vec3::new(UNIT as i32, 0, 0);

fn unit(x: f64) -> i32 {
    (x * UNIT as f64).round() as i32
}

fn rate(x: f64) -> i32 {
    (x * RATE as f64).round() as i32
}

fn gain(x: f64) -> i32 {
    (x * GAIN as f64).round() as i32
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

#[test]
fn normalization_rejects_zero_and_unitizes_inputs() {
    assert_eq!(Quat::new(0, 0, 0, 0).normalized(), None);
    assert_eq!(Vec3::ZERO.normalized(), None);

    let q = Quat::new(unit(0.3), unit(-0.4), unit(0.5), unit(0.6))
        .normalized()
        .unwrap();
    assert!((q.norm_squared() - UNIT * UNIT).abs() < 1i64 << 42);
    assert_eq!(Quat::IDENTITY.norm_squared(), UNIT * UNIT);

    let v = Vec3::new(3, 4, 0).normalized().unwrap();
    assert!((as_unit(v.x) - 0.6).abs() < 1e-6);
    assert!((as_unit(v.y) - 0.8).abs() < 1e-6);
    assert_eq!(as_unit(Vec3::new(0, 0, 5).normalized().unwrap().z), 1.0);
}

#[test]
fn fixed_scales_and_microsecond_conversion_are_consistent() {
    assert_eq!(UNIT, 1 << 30);
    assert_eq!(RATE, 1 << 16);
    assert_eq!(DT, 1 << 20);
    assert_eq!(GAIN, 1 << 8);
    assert_eq!(dt_q20_from_us(DT_US), 2097);
    assert_eq!(dt_q20_from_us(1_000_000), DT as i32);
    assert_eq!(dt_q20_from_us(1), 1);
}

#[test]
fn zero_dt_and_degenerate_references_are_loud_errors() {
    let mut filter = Mahony::default();
    let before = (filter.quaternion(), filter.bias());

    assert_eq!(
        filter.update(Vec3::ZERO, Some(UP), Some(NORTH), 0),
        Err(FilterError::ZeroDt)
    );
    assert_eq!(
        filter.update(Vec3::ZERO, Some(Vec3::ZERO), None, DT_US),
        Err(FilterError::DegenerateAccel)
    );
    assert_eq!(
        filter.update(Vec3::ZERO, Some(UP), Some(Vec3::ZERO), DT_US),
        Err(FilterError::DegenerateMag)
    );
    assert_eq!((filter.quaternion(), filter.bias()), before);
}

#[test]
fn aligned_references_and_no_motion_keep_identity_exactly() {
    let mut filter = Mahony::default();
    run(&mut filter, 500, Vec3::ZERO, Some(UP), Some(NORTH));
    assert_eq!(filter.quaternion(), Quat::IDENTITY);
    assert_eq!(filter.bias(), Vec3::ZERO);
}

#[test]
fn gyro_only_integration_rotates_and_keeps_unit_norm() {
    let mut filter = Mahony::default();
    // 500 * 2 ms = 1 s at 1 rad/s about z: quaternion angle 1 rad -> w,z at 0.5 rad.
    run(&mut filter, 500, Vec3::new(0, 0, rate(1.0)), None, None);
    let q = filter.quaternion();

    assert!((as_unit(q.w) - 0.5_f64.cos()).abs() < 2e-3);
    assert!((as_unit(q.z) - 0.5_f64.sin()).abs() < 2e-3);
    assert!(q.x.abs() < 8 && q.y.abs() < 8);
    assert!((q.norm_squared() - UNIT * UNIT).abs() < 1i64 << 42);
    assert_eq!(filter.bias(), Vec3::ZERO);
}

#[test]
fn gravity_reference_levels_a_tilted_attitude() {
    let mut filter = Mahony::new(Gains::new(gain(4.0), gain(0.5), 0, 0));
    // Build a ~0.4 rad roll with gyro only, then switch the gravity reference on.
    run(&mut filter, 500, Vec3::new(rate(0.4), 0, 0), None, None);
    assert!(as_unit(filter.quaternion().x) > 0.15);

    run(&mut filter, 2000, Vec3::ZERO, Some(UP), None);
    let q = filter.quaternion();
    assert!(as_unit(q.x).abs() < 0.02, "residual roll: {}", as_unit(q.x));
    assert!((as_unit(q.w) - 1.0).abs() < 0.01);
    assert!((q.norm_squared() - UNIT * UNIT).abs() < 1i64 << 42);
}

#[test]
fn bias_estimate_cancels_a_constant_gyro_bias_on_all_axes() {
    // Mahony Eq. 11b/32b: b̂ tracks the bias *added* to the gyro, and the
    // filter subtracts it (Eq. 32a). The earlier assertion expected −b.
    let truth = (0.04_f64, -0.02_f64, 0.03_f64);
    let gyro = Vec3::new(rate(truth.0), rate(truth.1), rate(truth.2));
    let mut filter = Mahony::new(Gains::new(gain(4.0), gain(2.0), gain(4.0), gain(2.0)));

    run(&mut filter, 4000, gyro, Some(UP), Some(NORTH));
    let b = filter.bias();
    assert!(
        (as_rate(b.x) - truth.0).abs() < 0.01,
        "bx = {}",
        as_rate(b.x)
    );
    assert!(
        (as_rate(b.y) - truth.1).abs() < 0.01,
        "by = {}",
        as_rate(b.y)
    );
    assert!(
        (as_rate(b.z) - truth.2).abs() < 0.01,
        "bz = {}",
        as_rate(b.z)
    );

    // With the bias cancelled the attitude stays near level/north.
    let q = filter.quaternion();
    assert!(as_unit(q.x).abs() < 0.05);
    assert!(as_unit(q.y).abs() < 0.05);
    assert!(as_unit(q.z).abs() < 0.05);
}

#[test]
fn higher_proportional_gain_corrects_faster() {
    fn residual_roll(kp: f64) -> f64 {
        let mut filter = Mahony::new(Gains::new(gain(kp), 0, 0, 0));
        run(&mut filter, 500, Vec3::new(rate(0.4), 0, 0), None, None);
        run(&mut filter, 60, Vec3::ZERO, Some(UP), None);
        as_unit(filter.quaternion().x).abs()
    }

    let slow = residual_roll(1.0);
    let fast = residual_roll(8.0);
    assert!(fast < slow * 0.6, "slow = {slow}, fast = {fast}");
}

#[test]
fn reset_clears_state_but_keeps_gains() {
    let mut filter = Mahony::default();
    run(&mut filter, 500, Vec3::new(rate(0.5), 0, 0), None, None);
    assert_ne!(filter.quaternion(), Quat::IDENTITY);

    let custom = Gains::new(1, 2, 3, 4);
    filter.set_gains(custom);
    assert_eq!(filter.gains(), custom);

    filter.reset();
    assert_eq!(filter.quaternion(), Quat::IDENTITY);
    assert_eq!(filter.bias(), Vec3::ZERO);
    assert_eq!(filter.gains(), custom);
}
