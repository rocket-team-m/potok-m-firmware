// SPDX-License-Identifier: GPL-3.0-or-later
//! Host contract of the pitot/static airdata seam: deterministic Mach,
//! finite/positive validation, counted UNAVAILABLE fallback, and the sample
//! reaching `evaluate_shield`.

#[path = "../src/shield_sample.rs"]
mod shield_sample;

use potok_m_firmware::{
    control::{ReasonCounts, ShieldSample},
    interlocks::{Input, Interlocks, Refusal},
    protocol::Phase,
    Command,
};
use shield_sample::{Airdata, NoAirdata, ShieldSampler, Unavailable, UnavailableCounts};

struct Fixed {
    dp: Option<f32>,
    p: Option<f32>,
}

impl Airdata for Fixed {
    fn pitot_dp_pa(&self) -> Option<f32> {
        self.dp
    }
    fn static_pa(&self) -> Option<f32> {
        self.p
    }
}

fn coast_command() -> Input {
    Input {
        phase_code: Phase::COAST as u8,
        flags: 0,
        ai_healthy: true,
        requested: Command { pitch: 8, yaw: -8 },
        now_us: 0,
    }
}

#[test]
fn mach_is_derived_deterministically_from_pitot_and_static() {
    // Isentropic q_c/p at M = 0.6, sea level p = 101325 Pa: dp ~= 25534 Pa.
    let src = Fixed {
        dp: Some(25_534.0),
        p: Some(101_325.0),
    };
    let mut sampler = ShieldSampler::new();
    let a = sampler.sample(&src);
    let b = sampler.sample(&src);

    assert_eq!(a, b, "same pressures must give the same bits every tick");
    assert_eq!(a.pitot_dp, 25_534);
    assert!((a.mach_est - 0.575_842).abs() < 1e-3, "got {}", a.mach_est);
    assert!(a.mach_est.is_finite() && a.mach_est > 0.0);
    assert_eq!(sampler.unavailable_counts(), UnavailableCounts::default());
}

#[test]
fn mach_responds_to_both_pressures_in_the_right_direction() {
    let mut sampler = ShieldSampler::new();
    let base = sampler.sample(&Fixed {
        dp: Some(1_000.0),
        p: Some(50_000.0),
    });
    assert!(
        (base.mach_est - 0.168_433).abs() < 1e-4,
        "got {}",
        base.mach_est
    );

    let faster = sampler.sample(&Fixed {
        dp: Some(4_000.0),
        p: Some(50_000.0),
    });
    let thinner = sampler.sample(&Fixed {
        dp: Some(1_000.0),
        p: Some(25_000.0),
    });
    assert!(
        faster.mach_est > base.mach_est,
        "more dynamic pressure -> faster"
    );
    assert!(
        thinner.mach_est > base.mach_est,
        "less static pressure -> faster"
    );
    assert_eq!(sampler.unavailable_counts(), UnavailableCounts::default());
}

#[test]
fn non_finite_and_non_positive_readings_are_unavailable_and_counted() {
    let cases = [
        (Some(f32::NAN), Some(101_325.0), Unavailable::NonFinite),
        (Some(f32::INFINITY), Some(101_325.0), Unavailable::NonFinite),
        (
            Some(1_000.0),
            Some(f32::NEG_INFINITY),
            Unavailable::NonFinite,
        ),
        (Some(0.0), Some(101_325.0), Unavailable::NonPositive),
        (Some(-1.0), Some(101_325.0), Unavailable::NonPositive),
        (Some(1_000.0), Some(0.0), Unavailable::NonPositive),
        (Some(1_000.0), Some(-5.0), Unavailable::NonPositive),
    ];
    for (dp, p, reason) in cases {
        let mut sampler = ShieldSampler::new();
        let sample = sampler.sample(&Fixed { dp, p });
        assert_eq!(sample, ShieldSample::UNAVAILABLE, "dp={dp:?} p={p:?}");
        let counts = sampler.unavailable_counts();
        let (expected, total) = match reason {
            Unavailable::NoDriver => (counts.no_driver, counts.no_driver),
            Unavailable::NonFinite => (counts.non_finite, counts.non_finite),
            Unavailable::NonPositive => (counts.non_positive, counts.non_positive),
        };
        assert_eq!(
            expected, 1,
            "reason {reason:?} counted for dp={dp:?} p={p:?}"
        );
        assert_eq!(total, 1, "exactly one fallback counted");
    }
}

#[test]
fn missing_driver_is_unavailable_with_a_counted_reason_never_a_healthy_zero() {
    let mut sampler = ShieldSampler::new();

    assert_eq!(sampler.sample(&NoAirdata), ShieldSample::UNAVAILABLE);
    assert_eq!(
        sampler.sample(&Fixed {
            dp: Some(1_000.0),
            p: None,
        }),
        ShieldSample::UNAVAILABLE
    );
    assert_eq!(
        sampler.unavailable_counts(),
        UnavailableCounts {
            no_driver: 2,
            ..UnavailableCounts::default()
        }
    );
    assert_eq!(ShieldSample::UNAVAILABLE.pitot_dp, 0);
}

#[test]
fn out_of_range_pitot_dp_saturates_the_wire_field_and_is_counted() {
    let mut sampler = ShieldSampler::new();
    let fast = Fixed {
        dp: Some(40_000.0),
        p: Some(101_325.0),
    };
    let sample = sampler.sample(&fast);

    assert_eq!(
        sample.pitot_dp,
        i16::MAX,
        "the wire field must saturate, never wrap negative"
    );
    assert_eq!(
        sample.mach_est.to_bits(),
        shield_sample::mach_from_pitot(40_000.0, 101_325.0)
            .unwrap()
            .to_bits(),
        "the Mach gate must still see the unclamped pressure"
    );
    assert_eq!(
        sampler.unavailable_counts(),
        UnavailableCounts {
            saturated: 1,
            ..UnavailableCounts::default()
        }
    );

    // Counting is cumulative; an in-range tick neither clears nor re-counts.
    let _ = sampler.sample(&fast);
    let _ = sampler.sample(&Fixed {
        dp: Some(25_534.0),
        p: Some(101_325.0),
    });
    assert_eq!(sampler.unavailable_counts().saturated, 2);
    assert_eq!(sampler.unavailable_counts().non_positive, 0);
}

#[test]
fn pitot_dp_at_the_i16_boundary_is_not_saturated() {
    let mut sampler = ShieldSampler::new();
    let sample = sampler.sample(&Fixed {
        dp: Some(i16::MAX as f32),
        p: Some(101_325.0),
    });
    assert_eq!(sample.pitot_dp, i16::MAX);
    assert_eq!(sampler.unavailable_counts(), UnavailableCounts::default());
}

#[test]
fn unavailable_feeds_the_shield_pitot_veto_with_a_counted_reason() {
    let mut sampler = ShieldSampler::new();
    let sample = sampler.sample(&NoAirdata);

    let mut gate = Interlocks::default();
    let decision = gate.evaluate_shield(coast_command(), sample.pitot_dp, sample.mach_est);
    assert_eq!(decision.command, Command::ZERO);
    assert_eq!(decision.refusal, Some(Refusal::PitotInvalid));
    assert_eq!(
        gate.latched(),
        None,
        "a missing driver is transient, not a fault"
    );

    let mut reasons = ReasonCounts::EMPTY;
    reasons.note(decision.refusal);
    assert_eq!(reasons.pitot_invalid, 1);
    assert_eq!(reasons.accepted, 0);
}

/// F7: the seed + four Newton steps must agree with the correctly-rounded
/// FPU sqrt over the whole airdata span, not just two operating points.
/// Sweep every whole pascal 1..=35 000 at sea level and convert dp -> x with
/// the isentropic relation in f64, sampling the x span the bisection can hand
/// the helper (representative, not every f32 bit pattern). For positive f32,
/// bit patterns are monotone in value, so their integer difference is the ULP
/// distance.
#[test]
fn sqrt_approximation_ulp_error_over_airdata_range_is_measured() {
    const P: f32 = 101_325.0;
    let mut worst_ulp = 0u32;
    let mut worst_dp = 0.0f32;
    let mut worst_x = 0.0f32;
    for dp_pa in 1..=35_000u32 {
        let dp = dp_pa as f32;
        let ratio = f64::from(dp) / f64::from(P);
        let u = (1.0 + ratio).powf(2.0 / 7.0);
        let x = ((u - 1.0) / 0.2) as f32;
        let approx = shield_sample::sqrt_approx(x);
        let reference = x.sqrt();
        let ulp = approx.to_bits().abs_diff(reference.to_bits());
        if ulp > worst_ulp {
            worst_ulp = ulp;
            worst_dp = dp;
            worst_x = x;
        }
        assert!(approx.is_finite() && approx > 0.0, "dp={dp} x={x}");
    }
    println!(
        "sqrt_approx worst = {worst_ulp} ULP at dp={worst_dp} Pa, x={worst_x}, ref={}",
        worst_x.sqrt()
    );
    assert!(
        worst_ulp <= 1,
        "measured {worst_ulp} ULP over dp 1..=35000 Pa (worst dp={worst_dp}, x={worst_x})"
    );
}

#[test]
fn a_live_sample_feeds_the_mach_ceiling_gate() {
    let mut sampler = ShieldSampler::new();
    let sample = sampler.sample(&Fixed {
        dp: Some(50_000.0),
        p: Some(101_325.0),
    });
    assert!(sample.mach_est > 0.6, "got {}", sample.mach_est);

    let mut gate = Interlocks::default();
    let decision = gate.evaluate_shield(coast_command(), sample.pitot_dp, sample.mach_est);
    assert_eq!(decision.command, Command::ZERO);
    assert_eq!(decision.refusal, Some(Refusal::MachLimit));
    assert_eq!(gate.counters.mach_blocked, 1);

    let below = sampler.sample(&Fixed {
        dp: Some(25_000.0),
        p: Some(101_325.0),
    });
    assert!(below.mach_est < 0.6);
    let mut gate = Interlocks::default();
    let decision = gate.evaluate_shield(coast_command(), below.pitot_dp, below.mach_est);
    assert_eq!(decision.refusal, None);
}
