// SPDX-License-Identifier: GPL-3.0-or-later
//! Closed-form gates for the source-traced PID law in `control_pid.rs`.
//!
//! Every expected value below is computed from the discrete equations in
//! Åström & Hägglund (AH95 *PID Controllers*, 2nd ed. 1995; AH06 *Advanced PID
//! Control* 2006) rather than from a captured run. Gains are chosen so the
//! arithmetic is exact; no gain is adjusted to make a formula pass. The module
//! is included by path, matching `control_pid_tests.rs`, so these gates do not
//! depend on `lib.rs` wiring.

pub use potok_m_firmware::{Command, MAX_CORRECTION_RAW};

#[path = "../src/control_pid.rs"]
mod control_pid;

use control_pid::{Fx, Pid, PidConfig, PidNum, DEFAULT_OUT_LIMIT_RAW, TICK_500HZ_S};

fn cfg(
    kp: f32,
    ki_dt: f32,
    kd_dt: f32,
    out_limit: f32,
    i_limit: f32,
    alpha: f32,
) -> PidConfig<f32> {
    PidConfig {
        kp,
        ki_dt,
        kd_dt,
        out_limit,
        i_limit,
        deriv_alpha: alpha,
    }
}

/// Integer-valued gains are exact in Q16.16 and f32, so both backends can be
/// compared for exact equality rather than a tolerance.
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

fn close(a: f32, b: f32) {
    let tol = 1e-6 * b.abs().max(1.0);
    assert!((a - b).abs() <= tol, "expected {b}, got {a} (tol {tol})");
}

// ---------------------------------------------------------------------------
// P / I / D step responses
// ---------------------------------------------------------------------------

/// AH06 Eq. (13.5), b = 1: `P_k = K*(r_k - y_k)`, no discretisation error.
/// The error is driven by the measurement: a setpoint step would engage the
/// bumpless preload and hold the output continuous (tested separately below).
#[test]
fn proportional_step_response_is_instant_linear_and_unsigned_free() {
    let mut p = Pid::new(cfg(3.0, 0.0, 0.0, 1000.0, 1000.0, 1.0));
    assert_eq!(p.update(0.0, 0.0), 0.0, "e = 0");
    assert_eq!(p.update(0.0, -4.0), 12.0, "K*e = 3*4");
    assert_eq!(p.update(0.0, -4.0), 12.0, "constant error -> constant P");
    assert_eq!(p.update(0.0, 2.0), -6.0, "negative error -> negative P");
    assert_eq!(p.update(0.0, 3.0), -9.0, "measurement enters with a minus");
    assert_eq!(p.integral(), 0.0);
    assert!(!p.saturated());
}

/// A setpoint step is a bumpless event, not a P kick: with Ki = Kd = 0 the
/// preload absorbs the whole P change and the output stays where it was.
#[test]
fn pure_p_controller_holds_output_continuous_across_a_setpoint_step() {
    let mut p = Pid::new(cfg(3.0, 0.0, 0.0, 1000.0, 1000.0, 1.0));
    assert_eq!(p.update(0.0, 0.0), 0.0);
    assert_eq!(p.update(10.0, 0.0), 0.0, "I = 0 - K*(10-0) = -30");
    assert_eq!(p.integral(), -30.0);
    assert_eq!(p.update(10.0, 0.0), 0.0, "Ki = 0, so I stays -30");
}

/// AH06 Eq. (13.7), forward Euler: `I_{k+1} = I_k + (K*h/Ti)*e_k`, then the
/// firmware's `i_limit` clamp. Closed form for constant e: min(k*ki_dt*e, Ilim).
#[test]
fn integral_step_response_is_forward_euler_then_clamped() {
    let mut p = Pid::new(cfg(0.0, 0.25, 0.0, 1000.0, 3.0, 1.0));
    assert_eq!(p.update(4.0, 0.0), 1.0); // 0 + 0.25*4
    assert_eq!(p.update(4.0, 0.0), 2.0); // 1 + 1
    assert_eq!(p.update(4.0, 0.0), 3.0); // 2 + 1 = i_limit
    assert_eq!(p.update(4.0, 0.0), 3.0); // 4 clamped to 3
    assert_eq!(p.update(4.0, 0.0), 3.0);
    assert_eq!(p.integral(), 3.0);
    // Reverse through the measurement, not the setpoint: no bumpless preload.
    assert_eq!(p.update(4.0, 8.0), 2.0, "3 + 0.25*(-4)");
    assert_eq!(p.update(4.0, 8.0), 1.0);
    assert_eq!(p.update(4.0, 8.0), 0.0);
    assert_eq!(p.update(4.0, 8.0), -1.0);
    assert_eq!(p.update(4.0, 8.0), -2.0);
    assert_eq!(p.update(4.0, 8.0), -3.0);
    assert_eq!(p.update(4.0, 8.0), -3.0, "negative i_limit clamp");
    assert_eq!(p.integral(), -3.0);
}

/// AH06 Eq. (13.13) with alpha = 1: `D_k = -kd_dt*(y_k - y_{k-1})`.
#[test]
fn derivative_step_response_is_on_measurement_with_negative_sign() {
    let mut p = Pid::new(cfg(0.0, 0.0, 2.0, 1000.0, 1000.0, 1.0));
    assert_eq!(p.update(0.0, 0.0), 0.0, "first tick primes y_prev");
    assert_eq!(p.update(0.0, 1.0), -2.0, "dy=+1 -> -kd_dt");
    assert_eq!(p.update(0.0, 1.0), 0.0, "dy=0 -> no D");
    assert_eq!(p.update(0.0, 3.0), -4.0, "dy=+2 -> -2*kd_dt");
    assert_eq!(p.update(0.0, 0.0), 6.0, "dy=-3 -> +3*kd_dt");
}

/// AH06 Eq. (13.13): `D_k = alpha*D_{k-1} - kd_dt*(1-alpha)*Δy_k`. Filtering
/// the increment before scaling is the same recursion because the map is
/// linear. alpha = 0.5 makes every intermediate value an exact binary f32.
#[test]
fn derivative_filter_is_the_geometric_recursion_of_ah13_13() {
    let mut p = Pid::new(cfg(0.0, 0.0, 2.0, 1000.0, 1000.0, 0.5));
    assert_eq!(p.update(0.0, 0.0), 0.0);
    assert_eq!(p.update(0.0, 2.0), -2.0, "f = 0.5*2 = 1, D = -2");
    assert_eq!(p.update(0.0, 2.0), -1.0, "f = 0.5*1 = 0.5, D = -1");
    assert_eq!(p.update(0.0, 2.0), -0.5, "f = 0.25, D = -0.5");
    assert_eq!(p.update(0.0, 6.0), -4.25, "f = 0.25+0.5*(4-0.25) = 2.125");
}

/// AH06 Eq. (3.19) with c = 0: the setpoint never reaches the derivative.
/// Two controllers with different setpoint steps but the same measurement
/// history must produce bit-identical D output.
#[test]
fn setpoint_step_causes_no_derivative_kick() {
    let c = cfg(0.0, 0.0, 3.0, 1000.0, 1000.0, 1.0);
    let mut stepped = Pid::new(c);
    let mut flat = Pid::new(c);
    assert_eq!(stepped.update(0.0, 0.0), flat.update(0.0, 0.0));
    assert_eq!(stepped.update(100.0, 0.0), 0.0, "setpoint step: no kick");
    assert_eq!(flat.update(0.0, 0.0), 0.0);
    assert_eq!(stepped.update(100.0, 1.0), -3.0);
    assert_eq!(flat.update(0.0, 1.0), -3.0, "identical to no setpoint step");
}

// ---------------------------------------------------------------------------
// Anti-windup, integral limits, saturation
// ---------------------------------------------------------------------------

/// AH95 §3.5 / AH06 §3.5 conditional integration: freeze only while saturated
/// and pushing further out. Here `ki_dt*e == out_limit`, so the first tick
/// lands exactly on the limit (not saturated) and the second freezes.
#[test]
fn anti_windup_freezes_at_the_edge_and_unwinds_in_one_tick() {
    let mut p = Pid::new(cfg(0.0, 1.0, 0.0, 10.0, 100.0, 1.0));
    assert_eq!(p.update(10.0, 0.0), 10.0, "tick 1 reaches the limit");
    assert!(!p.saturated(), "unsat == limit is not saturation");
    for _ in 0..20 {
        assert_eq!(p.update(10.0, 0.0), 10.0);
        assert!(p.saturated());
        assert_eq!(p.integral(), 10.0, "I frozen at out_limit, not wound up");
    }
    // Reverse via measurement so the bumpless preload cannot mask the unwind.
    assert_eq!(p.update(10.0, 20.0), 0.0, "I = 10 + 1*(-10)");
    assert_eq!(p.integral(), 0.0);
    assert!(!p.saturated());
}

/// Mirror of the above at the lower limit.
#[test]
fn anti_windup_freezes_at_the_lower_limit_too() {
    let mut p = Pid::new(cfg(0.0, 1.0, 0.0, 10.0, 100.0, 1.0));
    assert_eq!(p.update(-10.0, 0.0), -10.0);
    for _ in 0..20 {
        assert_eq!(p.update(-10.0, 0.0), -10.0);
        assert_eq!(p.integral(), -10.0);
    }
    assert_eq!(p.update(-10.0, -20.0), 0.0, "error reverses -> +10 step");
    assert_eq!(p.integral(), 0.0);
}

/// `i_limit` is a firmware clamp in output units, independent of `out_limit`.
#[test]
fn integral_limit_is_symmetric_and_binds_before_output_saturation() {
    let mut p = Pid::new(cfg(0.0, 3.0, 0.0, 1000.0, 7.0, 1.0));
    assert_eq!(p.update(10.0, 0.0), 7.0, "min(3*10, 7)");
    assert_eq!(p.update(10.0, 0.0), 7.0);
    assert_eq!(p.integral(), 7.0);
    assert!(!p.saturated(), "output never reached out_limit");
    assert_eq!(p.update(10.0, 30.0), -7.0, "min(7 - 3*20, -7)");
    assert_eq!(p.update(10.0, 30.0), -7.0);
    assert_eq!(p.integral(), -7.0);
}

/// AH95 computer code `u = sat(v, ulow, uhigh)`; symmetric and counted.
#[test]
fn output_saturation_is_symmetric_and_counted() {
    let mut p = Pid::new(cfg(10.0, 0.0, 0.0, 25.0, 1000.0, 1.0));
    assert_eq!(p.update(10.0, 0.0), 25.0, "K*e = 100 -> +limit");
    assert!(p.saturated());
    assert_eq!(
        p.update(10.0, 20.0),
        -25.0,
        "measurement reverses the error"
    );
    assert!(p.saturated());
    assert_eq!(p.update(10.0, 10.0), 0.0, "error zero -> no P, no windup");
    assert!(!p.saturated());
    assert_eq!(DEFAULT_OUT_LIMIT_RAW, MAX_CORRECTION_RAW as i32);
}

// ---------------------------------------------------------------------------
// Bumpless transfer
// ---------------------------------------------------------------------------

/// AH06 Eq. (13.25) / IFAC PID'24 §3.6: `I_new = u_prev - P_new - D_new`
/// holds `P+I+D` continuous across a setpoint step, then integration resumes.
#[test]
fn setpoint_change_is_bumpless_in_closed_form() {
    let mut p = Pid::new(cfg(2.0, 1.0, 0.0, 1000.0, 1000.0, 1.0));
    assert_eq!(p.update(4.0, 0.0), 12.0, "P=8, I=4");
    assert_eq!(p.update(4.0, 0.0), 16.0, "P=8, I=8");
    let before = p.last_output();
    assert_eq!(before, 16.0);
    assert_eq!(p.update(10.0, 0.0), 16.0, "P=20, so I=16-20=-4");
    assert_eq!(p.integral(), -4.0);
    assert_eq!(p.update(10.0, 0.0), 26.0, "integration resumes: I=-4+10=6");
    assert_eq!(p.integral(), 6.0);
}

/// The formula holds even when the previous output was saturated: the preload
/// uses `last_output`, not the unsaturated sum.
#[test]
fn bumpless_preload_uses_the_saturated_previous_output() {
    let mut p = Pid::new(cfg(2.0, 1.0, 0.0, 100.0, 1000.0, 1.0));
    assert_eq!(p.update(10.0, -100.0), 100.0, "saturated high");
    let before = p.last_output();
    assert_eq!(p.update(20.0, -100.0), before, "still continuous");
    assert_eq!(
        p.integral(),
        100.0 - 2.0 * (20.0 + 100.0),
        "I = u_prev - K*(sp - y)"
    );
}

// ---------------------------------------------------------------------------
// Per-step scaling and backend equivalence
// ---------------------------------------------------------------------------

/// The AH95 program precomputes `bi = K*h/Ti` and `bd = 2*K*N*Td/(2*Td+N*h)`
/// (Tustin); AH06 Eq. (13.13) uses the backward-Euler pair
/// `ad = Td/(Td+N*h)`, `bd = K*Td*N/(Td+N*h)`. This firmware uses the latter:
/// `ki_dt = bi`, `kd_dt = K*Td/h` and `kd_dt*(1-ad) = bd`.
#[test]
fn si_gains_match_the_book_discrete_coefficients() {
    let k = 2.0_f32;
    let ti = 0.5_f32;
    let td = 0.25_f32;
    let h = 0.01_f32;
    let n = 5.0_f32;
    let alpha = td / (td + n * h);
    let c = PidConfig::from_si(k, k / ti, k * td, h, 100.0, 100.0, alpha);
    close(c.ki_dt, k * h / ti);
    close(c.kd_dt, k * td / h);
    close(c.kd_dt * (1.0 - alpha), k * td * n / (td + n * h));
    assert!(alpha > 0.0 && alpha <= 1.0);
    assert_eq!(TICK_500HZ_S, 0.002);
}

/// Same integer gain set on both backends through setpoint steps, measurement
/// ramps, saturation and anti-windup freeze: outputs, integral and flag match
/// exactly after conversion to raw units.
#[test]
fn fixed_point_and_f32_agree_through_the_full_nonlinear_path() {
    let c = cfg(2.0, 1.0, 3.0, 100.0, 10.0, 1.0);
    let mut f = Pid::new(c);
    let mut q = Pid::new(as_fx(&c));
    let sequence = [
        (10.0, 0.0),
        (10.0, 1.0),
        (20.0, 1.0),
        (100.0, 5.0),
        (-20.0, 5.0),
        (-20.0, -3.0),
        (5.0, 0.0),
        (5.0, 50.0),
        (5.0, -50.0),
    ];
    for (sp, meas) in sequence {
        let out_f = f.update(sp, meas);
        let out_q = q.update(Fx::from_raw(sp as i32), Fx::from_raw(meas as i32));
        assert_eq!(
            PidNum::to_raw(out_f),
            PidNum::to_raw(out_q),
            "sp={sp} y={meas}"
        );
        assert_eq!(PidNum::to_raw(f.integral()), q.integral().to_raw());
        assert_eq!(f.saturated(), q.saturated(), "sp={sp} y={meas}");
    }
}

/// Reset must clear the derivative history too, not only the integral.
#[test]
fn reset_clears_measurement_history_for_the_derivative() {
    let c = cfg(0.0, 0.0, 2.0, 1000.0, 1000.0, 1.0);
    let mut p = Pid::new(c);
    p.update(0.0, 10.0);
    p.update(0.0, 12.0);
    p.reset();
    let mut fresh = Pid::new(c);
    assert_eq!(p.update(0.0, 5.0), fresh.update(0.0, 5.0));
}

#[test]
#[should_panic(expected = "deriv_alpha")]
fn zero_derivative_alpha_is_rejected_loudly() {
    let _ = Pid::new(cfg(1.0, 0.0, 0.0, 100.0, 10.0, 0.0));
}

#[test]
#[should_panic(expected = "i_limit")]
fn negative_integral_limit_is_rejected_loudly() {
    let _ = Pid::new(cfg(1.0, 0.0, 0.0, 100.0, -1.0, 1.0));
}

// Keep the re-export shim honest: the module only bridges through Command.
#[test]
fn command_bridge_is_unchanged() {
    let cmd = control_pid::command_from_raw(MAX_CORRECTION_RAW as i32 * 2, 0);
    assert_eq!(cmd.pitch, MAX_CORRECTION_RAW);
    assert_eq!(cmd.yaw, 0);
    let _ = Command::ZERO;
}
