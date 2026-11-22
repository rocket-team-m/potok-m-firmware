// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the PID control law. The module is included by path because
//! `lib.rs` is owned by another owner; re-exports below make `crate::Command`
//! inside it resolve to the firmware's real type, unchanged.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/control_pid.rs"]
mod control_pid;

use control_pid::{
    command_from_raw, Fx, Pid, PidConfig, PidNum, DEFAULT_OUT_LIMIT_RAW, TICK_500HZ_S,
};

fn cfg(kp: f32, ki_dt: f32, kd_dt: f32) -> PidConfig<f32> {
    PidConfig {
        kp,
        ki_dt,
        kd_dt,
        out_limit: 100.0,
        i_limit: 10.0,
        deriv_alpha: 1.0,
    }
}

/// Integer-valued gains are exact in both Q16.16 and f32, so the two backends
/// can be compared for exact equality rather than a fuzzy tolerance.
fn as_fx(c: &PidConfig<f32>) -> PidConfig<Fx> {
    PidConfig {
        kp: Fx::from_raw(c.kp as i32),
        ki_dt: Fx::from_raw(c.ki_dt as i32),
        kd_dt: Fx::from_raw(c.kd_dt as i32),
        out_limit: Fx::from_raw(c.out_limit as i32),
        i_limit: Fx::from_raw(c.i_limit as i32),
        deriv_alpha: Fx::from_raw(c.deriv_alpha as i32),
    }
}

#[test]
fn proportional_term_tracks_error_and_output_saturates() {
    let mut p = Pid::new(cfg(2.0, 0.0, 0.0));
    assert_eq!(p.update(10.0, 10.0), 0.0); // first tick primes at the setpoint
    assert_eq!(p.update(10.0, 0.0), 20.0); // error from the measurement side
    assert!(!p.saturated());
    assert_eq!(p.update(10.0, -90.0), 100.0); // 200 -> code veto
    assert!(p.saturated());
    assert_eq!(p.update(10.0, 10.0), 0.0); // error gone, unwinds at once
    assert!(!p.saturated());
}

#[test]
fn derivative_is_on_measurement_not_setpoint() {
    let mut p = Pid::new(cfg(0.0, 0.0, 3.0));
    assert_eq!(p.update(0.0, 0.0), 0.0);
    assert_eq!(p.update(50.0, 0.0), 0.0); // setpoint step, no kick
    assert_eq!(p.update(50.0, 1.0), -3.0); // measurement rises -> opposes it
    assert_eq!(p.update(50.0, 0.0), 3.0); // and falls back
}

#[test]
fn integral_accumulates_then_clamps_at_i_limit() {
    let mut p = Pid::new(cfg(0.0, 1.0, 0.0));
    assert_eq!(p.update(5.0, 0.0), 5.0);
    assert_eq!(p.update(5.0, 0.0), 10.0);
    assert_eq!(p.update(5.0, 0.0), 10.0);
    assert_eq!(p.integral(), 10.0);
}

#[test]
fn anti_windup_freezes_while_saturated_and_unwinds_immediately() {
    let c = PidConfig {
        kp: 0.0,
        ki_dt: 1.0,
        kd_dt: 0.0,
        out_limit: 10.0,
        i_limit: 100.0, // wider than the output: clamp alone cannot stop windup
        deriv_alpha: 1.0,
    };
    let mut p = Pid::new(c);
    for _ in 0..50 {
        p.update(10.0, 0.0);
    }
    assert_eq!(
        p.integral(),
        10.0,
        "integrator froze at the saturation edge"
    );
    // Error sign reverses without a setpoint step (so bumpless preload does not mask it).
    assert_eq!(p.update(10.0, 20.0), 0.0, "reversal unwinds in one tick");
    assert_eq!(p.integral(), 0.0);
}

#[test]
fn setpoint_change_is_bumpless() {
    let mut p = Pid::new(cfg(2.0, 1.0, 0.0));
    p.update(10.0, 0.0); // integral 10, output 30
    p.update(10.0, 0.0); // integral stays clamped at 10
    let before = p.last_output();
    assert_eq!(before, 30.0);
    let after = p.update(20.0, 0.0); // p=40, integral preloaded to -10
    assert_eq!(after, before, "output must not jump on a setpoint step");
    assert_eq!(p.integral(), -10.0);
    assert_eq!(p.update(20.0, 0.0), 50.0); // integration resumes afterwards
}

#[test]
fn derivative_filter_suppresses_a_single_spike() {
    let mut raw = Pid::new(cfg(0.0, 0.0, 2.0)); // alpha 1
    let mut smooth = Pid::new(PidConfig {
        deriv_alpha: 0.5,
        ..cfg(0.0, 0.0, 2.0)
    });
    raw.update(0.0, 0.0);
    smooth.update(0.0, 0.0);
    assert_eq!(raw.update(0.0, 10.0), -20.0);
    assert_eq!(smooth.update(0.0, 10.0), -10.0);
}

#[test]
fn fixed_point_and_f32_paths_agree_exactly() {
    let c = cfg(2.0, 1.0, 3.0);
    let mut f = Pid::new(c);
    let mut q = Pid::new(as_fx(&c));
    let sequence = [
        (10.0, 0.0),
        (10.0, 1.0),
        (20.0, 1.0),
        (0.0, 5.0),
        (-20.0, 5.0),
        (-20.0, -3.0),
        (100.0, -3.0),
        (5.0, 0.0),
    ];
    for (sp, meas) in sequence {
        let a = f.update(sp, meas);
        let b = q.update(Fx::from_raw(sp as i32), Fx::from_raw(meas as i32));
        assert_eq!(PidNum::to_raw(a), PidNum::to_raw(b), "sp={sp} meas={meas}");
        assert_eq!(f.saturated(), q.saturated());
    }
}

#[test]
fn reset_matches_a_fresh_controller() {
    let c = cfg(1.0, 1.0, 0.0);
    let mut p = Pid::new(c);
    p.update(5.0, 0.0);
    p.update(5.0, 0.0);
    p.reset();
    let mut fresh = Pid::new(c);
    assert_eq!(p.update(3.0, 0.0), fresh.update(3.0, 0.0));
    assert_eq!(p.integral(), fresh.integral());
}

#[test]
fn output_bridges_to_command_with_the_existing_code_veto() {
    assert_eq!(DEFAULT_OUT_LIMIT_RAW, MAX_CORRECTION_RAW as i32);
    assert_eq!(
        command_from_raw(100, -100),
        Command {
            pitch: 100,
            yaw: -100
        }
    );
    assert_eq!(
        command_from_raw(10_000, -10_000),
        Command {
            pitch: MAX_CORRECTION_RAW,
            yaw: -MAX_CORRECTION_RAW
        }
    );
    assert!(command_from_raw(0, 0).is_zero());
}

#[test]
fn si_helper_scales_per_second_gains_to_per_step() {
    let c = PidConfig::from_si(2.0, 2.0, 4.0, 0.25, 100.0, 10.0, 1.0);
    assert_eq!(c.kp, 2.0);
    assert_eq!(c.ki_dt, 0.5);
    assert_eq!(c.kd_dt, 16.0);
    assert_eq!(TICK_500HZ_S, 0.002);
}

#[test]
#[should_panic(expected = "out_limit")]
fn invalid_configuration_is_loud() {
    let mut c = cfg(1.0, 0.0, 0.0);
    c.out_limit = 0.0;
    let _ = Pid::new(c);
}
