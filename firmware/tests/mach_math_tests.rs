// SPDX-License-Identifier: GPL-3.0-or-later
//! Math contract of the air-data Mach seam: the isentropic Pitot relation,
//! its deterministic libm-free solver, and the loud refusal above M = 1.
//! Host-only: no pin, no sensor, no flight claim is made here.

#[path = "../src/shield_sample.rs"]
mod shield_sample;

use potok_m_firmware::control::ShieldSample;
use shield_sample::{
    mach_from_pitot, Airdata, MachError, NoAirdata, ShieldSampler, UnavailableCounts,
};

const SEA_LEVEL_P: f32 = 101_325.0;

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

/// Trusted values from the isentropic Pitot relation at p = 101325 Pa,
/// `q_c/p = (1 + 0.2 M^2)^3.5 - 1`, evaluated in f64 and rounded to whole Pa
/// before pinning. Impact pressure `q_c` is the pitot-minus-static reading dp.
const TABLE: [(f32, f32); 4] = [
    (711.0, 0.1),
    (6_528.0, 0.3),
    (27_915.0, 0.6),
    (70_046.0, 0.9),
];

#[test]
fn matches_trusted_isentropic_table() {
    for (dp, expected_mach) in TABLE {
        let mach =
            mach_from_pitot(dp, SEA_LEVEL_P).unwrap_or_else(|e| panic!("dp={dp} refused: {e:?}"));
        assert!(
            (mach - expected_mach).abs() < 2e-4,
            "dp={dp}: isentropic Mach {mach}, table {expected_mach}"
        );
        assert!(mach.is_finite() && mach > 0.0 && mach < 1.0);
    }
}

#[test]
fn incompressible_estimate_over_reads_where_it_matters() {
    // The retired formula M^2 = 2 q_c / (gamma p) reads 0.6273 for the dp that
    // the isentropic relation says is M = 0.6: a 0.0274 over-read, 4.6% high.
    let dp = 27_915.0;
    let old = (2.0 * dp / (1.4 * SEA_LEVEL_P)).sqrt();
    let new = mach_from_pitot(dp, SEA_LEVEL_P).unwrap();
    assert!(
        old - 0.6 > 0.02,
        "expected the old formula to over-read: {old}"
    );
    assert!((new - 0.6).abs() < 2e-4, "isentropic Mach: {new}");
    assert!(
        new < old,
        "isentropic Mach must be lower than incompressible"
    );
}

#[test]
fn mach_is_monotone_in_impact_pressure_and_inverse_in_static() {
    let mut last = 0.0;
    for (dp, _) in TABLE {
        let mach = mach_from_pitot(dp, SEA_LEVEL_P).unwrap();
        assert!(mach > last, "more impact pressure must mean more Mach");
        last = mach;
    }

    let base = mach_from_pitot(27_915.0, SEA_LEVEL_P).unwrap();
    let thinner = mach_from_pitot(27_915.0, 90_000.0).unwrap();
    let denser = mach_from_pitot(27_915.0, 110_000.0).unwrap();
    assert!(thinner > base, "less static pressure must mean more Mach");
    assert!(denser < base, "more static pressure must mean less Mach");
}

#[test]
fn zero_negative_and_non_finite_readings_are_refused() {
    let cases = [
        (0.0, SEA_LEVEL_P, MachError::NonPositive),
        (-1.0, SEA_LEVEL_P, MachError::NonPositive),
        (1_000.0, 0.0, MachError::NonPositive),
        (1_000.0, -5.0, MachError::NonPositive),
        (f32::NAN, SEA_LEVEL_P, MachError::NonFinite),
        (f32::INFINITY, SEA_LEVEL_P, MachError::NonFinite),
        (1_000.0, f32::NEG_INFINITY, MachError::NonFinite),
    ];
    for (dp, p, expected) in cases {
        assert_eq!(mach_from_pitot(dp, p), Err(expected), "dp={dp} p={p}");
    }

    for (dp, p, reason) in cases {
        let mut sampler = ShieldSampler::new();
        assert_eq!(
            sampler.sample(&Fixed {
                dp: Some(dp),
                p: Some(p)
            }),
            ShieldSample::UNAVAILABLE
        );
        let counts = sampler.unavailable_counts();
        let got = match reason {
            MachError::NonFinite => counts.non_finite,
            MachError::NonPositive => counts.non_positive,
            MachError::AboveSubsonicLimit => counts.above_subsonic,
        };
        assert_eq!(got, 1, "reason {reason:?} counted once for dp={dp} p={p}");
        assert_eq!(counts.above_subsonic, 0, "not an out-of-domain ratio");
    }

    let mut sampler = ShieldSampler::new();
    assert_eq!(sampler.sample(&NoAirdata), ShieldSample::UNAVAILABLE);
    assert_eq!(sampler.unavailable_counts().no_driver, 1);
}

#[test]
fn supersonic_ratio_is_refused_loudly_never_clamped() {
    // M = 1 gives q_c/p = 1.2^3.5 - 1 ~= 0.892929. Just below it the solver
    // still returns a subsonic Mach; at and above it there is no solution.
    let below = mach_from_pitot(90_000.0, SEA_LEVEL_P).unwrap();
    assert!(below < 1.0, "below the limit must stay subsonic: {below}");

    for dp in [90_477.0, 100_000.0, 500_000.0] {
        assert_eq!(
            mach_from_pitot(dp, SEA_LEVEL_P),
            Err(MachError::AboveSubsonicLimit),
            "dp={dp} is outside the subsonic domain"
        );

        let mut sampler = ShieldSampler::new();
        let sample = sampler.sample(&Fixed {
            dp: Some(dp),
            p: Some(SEA_LEVEL_P),
        });
        assert_eq!(sample, ShieldSample::UNAVAILABLE);
        assert_eq!(sample.mach_est, 0.0, "no clamped M = 1 sample may escape");
        assert_eq!(
            sampler.unavailable_counts(),
            UnavailableCounts {
                above_subsonic: 1,
                ..UnavailableCounts::default()
            }
        );
    }
}

#[test]
fn valid_sample_reaches_the_shield_without_a_fallback() {
    let mut sampler = ShieldSampler::new();
    let source = Fixed {
        dp: Some(27_915.0),
        p: Some(SEA_LEVEL_P),
    };
    let first = sampler.sample(&source);
    assert!((first.mach_est - 0.6).abs() < 2e-4);
    assert_eq!(first.pitot_dp, 27_915);
    assert_eq!(sampler.unavailable_counts(), UnavailableCounts::default());

    for _ in 0..8 {
        assert_eq!(
            sampler.sample(&source).mach_est.to_bits(),
            first.mach_est.to_bits(),
            "the same pressures must give the same bits every tick"
        );
    }
    for _ in 0..8 {
        assert_eq!(
            mach_from_pitot(27_915.0, SEA_LEVEL_P).unwrap().to_bits(),
            first.mach_est.to_bits()
        );
    }
    assert_eq!(sampler.unavailable_counts(), UnavailableCounts::default());
}
