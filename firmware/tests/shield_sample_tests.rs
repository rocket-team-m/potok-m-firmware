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
    // q = gamma*p*M^2/2 at M = 0.6, sea level p = 101325 Pa.
    let src = Fixed {
        dp: Some(25_534.0),
        p: Some(101_325.0),
    };
    let mut sampler = ShieldSampler::new();
    let a = sampler.sample(&src);
    let b = sampler.sample(&src);

    assert_eq!(a, b, "same pressures must give the same bits every tick");
    assert_eq!(a.pitot_dp, 25_534);
    assert!((a.mach_est - 0.6).abs() < 1e-3, "got {}", a.mach_est);
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
        (base.mach_est - 0.169_030_5).abs() < 1e-4,
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
