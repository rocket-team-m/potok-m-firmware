// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for redundant-sensor voting. The module is included by path
//! because `lib.rs` is owned by another owner; the module has no crate
//! dependencies, so nothing needs re-exporting.

#[path = "../src/sensor_voting.rs"]
mod sensor_voting;

use sensor_voting::{Config, ImuSample, Sample, Tolerance, Vote, VotingChannel};

fn tol() -> Tolerance {
    Tolerance::new(0.5, 0.01)
}

fn channel() -> VotingChannel<f32> {
    VotingChannel::new(Config::DEFAULT, tol())
}

fn close(got: f32, want: f32) {
    assert!(
        (got - want).abs() < 1e-6,
        "expected {want}, got {got} (delta {})",
        got - want
    );
}

#[test]
fn triple_median_rejects_one_wild_reading() {
    let mut ch = channel();
    let v = ch.vote_triple(Some(1.0), Some(2.0), Some(100.0)).unwrap();
    assert_eq!(v.value, 2.0);
    assert_eq!(v.outlier, Some(2));
    assert!(v.disagreement);
    assert_eq!(v.used_mask, 0b111);
    assert_eq!(v.stuck_mask, 0);
    assert_eq!(v.used_count(), 3);
    // Total disagreement leaves only the 20 % confidence floor of a triple.
    close(v.confidence, 0.2);
    assert_eq!(v.confidence_u8(), 51);
}

#[test]
fn dual_midpoint_agrees_and_disagrees() {
    let mut ch = channel();
    let good = ch.vote_dual(Some(10.0), Some(10.4)).unwrap();
    close(good.value, 10.2);
    assert!(!good.disagreement);
    assert_eq!(good.outlier, None);
    assert_eq!(good.used_mask, 0b011);
    assert_eq!(good.confidence, 0.75);
    assert_eq!(good.confidence_u8(), 191);

    // Two sensors cannot arbitrate which one is wrong: no outlier is named.
    let bad = ch.vote_dual(Some(10.0), Some(100.0)).unwrap();
    close(bad.value, 55.0);
    assert!(bad.disagreement);
    assert_eq!(bad.outlier, None);
    assert_eq!(bad.confidence_u8(), 38);
}

#[test]
fn single_sensor_is_usable_but_low_confidence() {
    let mut ch = channel();
    let v = ch.vote_dual(Some(5.0), None).unwrap();
    assert_eq!(v.value, 5.0);
    assert!(!v.disagreement);
    assert_eq!(v.used_mask, 0b001);
    assert_eq!(v.used_count(), 1);
    assert_eq!(v.confidence_u8(), 89);
}

#[test]
fn no_usable_reading_returns_none_and_recovers() {
    let mut ch = channel();
    assert!(ch.vote_triple(None, None, None).is_none());
    assert!(ch
        .vote_triple(Some(f32::NAN), Some(f32::INFINITY), Some(f32::NEG_INFINITY))
        .is_none());
    assert!(ch.vote_dual(Some(f32::NAN), None).is_none());

    let v = ch.vote_dual(Some(3.0), None).unwrap();
    assert_eq!(v.value, 3.0);
    assert_eq!(v.stuck_mask, 0);
}

#[test]
fn frozen_sensor_is_quarantined_only_when_peers_move() {
    let mut ch = VotingChannel::new(Config::new(3), tol());
    assert_eq!(
        ch.vote_triple(Some(10.0), Some(10.0), Some(10.0))
            .unwrap()
            .value,
        10.0
    );
    for (tick, peer) in [12.0, 14.0].iter().enumerate() {
        let v = ch
            .vote_triple(Some(10.0), Some(*peer), Some(*peer))
            .unwrap();
        assert_eq!(v.stuck_mask, 0, "not stuck at frozen tick {}", tick + 1);
        assert_eq!(v.used_mask, 0b111);
    }

    let v = ch.vote_triple(Some(10.0), Some(16.0), Some(16.0)).unwrap();
    assert_eq!(v.stuck_mask, 0b001);
    assert_eq!(v.used_mask, 0b110);
    assert_eq!(v.value, 16.0);
    assert!(!v.disagreement, "the two live peers agree");
    assert_eq!(v.confidence_u8(), 191);

    // A fresh value unfreezes and reinstates the sensor on the next tick.
    let v = ch.vote_triple(Some(25.0), Some(18.0), Some(18.0)).unwrap();
    assert_eq!(v.stuck_mask, 0);
    assert_eq!(v.used_mask, 0b111);
    assert_eq!(v.value, 18.0);
    assert_eq!(v.outlier, Some(0));
}

#[test]
fn stationary_vehicle_does_not_quarantine_any_sensor() {
    let mut ch = VotingChannel::new(Config::new(3), tol());
    for _ in 0..20 {
        let v = ch
            .vote_triple(Some(1013.25), Some(1013.25), Some(1013.25))
            .unwrap();
        assert_eq!(v.value, 1013.25);
        assert_eq!(v.stuck_mask, 0);
        assert_eq!(v.used_mask, 0b111);
        assert!(!v.disagreement);
        assert_eq!(v.confidence_u8(), 255);
    }
}

#[test]
fn dual_with_one_stuck_slot_degrades_to_single_confidence() {
    let mut ch = VotingChannel::new(Config::new(3), tol());
    ch.vote_dual(Some(10.0), Some(10.0));
    ch.vote_dual(Some(10.0), Some(12.0));
    ch.vote_dual(Some(10.0), Some(14.0));

    let v = ch.vote_dual(Some(10.0), Some(16.0)).unwrap();
    assert_eq!(v.stuck_mask, 0b001);
    assert_eq!(v.used_mask, 0b010);
    assert_eq!(v.value, 16.0);
    assert_eq!(v.confidence_u8(), 89);
}

#[test]
fn reset_clears_stuck_and_history() {
    let mut ch = VotingChannel::new(Config::new(3), tol());
    ch.vote_triple(Some(10.0), Some(10.0), Some(10.0));
    ch.vote_triple(Some(10.0), Some(12.0), Some(12.0));
    ch.vote_triple(Some(10.0), Some(14.0), Some(14.0));
    let v = ch.vote_triple(Some(10.0), Some(16.0), Some(16.0)).unwrap();
    assert_eq!(v.stuck_mask, 0b001);
    assert_eq!(ch.stuck_mask(), 0b001);

    ch.reset();
    assert_eq!(ch.stuck_mask(), 0);
    let v = ch.vote_triple(Some(10.0), Some(16.0), Some(16.0)).unwrap();
    assert_eq!(v.stuck_mask, 0);
    assert_eq!(v.used_mask, 0b111);
    assert!(v.disagreement);
}

#[test]
fn relative_tolerance_scales_with_magnitude() {
    let mut ch = VotingChannel::new(Config::DEFAULT, Tolerance::new(0.1, 0.01));
    let high = ch.vote_dual(Some(1000.0), Some(1000.5)).unwrap();
    assert!(!high.disagreement);

    let low = ch.vote_dual(Some(1.0), Some(2.0)).unwrap();
    assert!(low.disagreement);
    assert_eq!(low.value, 1.5);
}

#[test]
fn imu_vectors_and_barometer_scalar_vote_independently() {
    let v = VotingChannel::<[f32; 3]>::new(Config::DEFAULT, tol())
        .vote_dual(Some([1.0, 2.0, 3.0]), Some([1.0, 2.4, 3.0]))
        .unwrap();
    close(v.value[0], 1.0);
    close(v.value[1], 2.2);
    close(v.value[2], 3.0);
    assert!(!v.disagreement);

    let a = ImuSample {
        gyro: [0.0; 3],
        accel: [0.0, 9.81, 0.0],
    };
    let b = ImuSample {
        gyro: [0.1, 0.0, 0.0],
        accel: [0.0, 9.81, 0.0],
    };
    let wild = ImuSample {
        gyro: [50.0, 0.0, 0.0],
        accel: [0.0, 9.81, 0.0],
    };
    let mut imu = VotingChannel::<ImuSample>::new(Config::DEFAULT, tol());
    let v = imu.vote_triple(Some(a), Some(b), Some(wild)).unwrap();
    assert_eq!(v.value, b);
    assert_eq!(v.outlier, Some(2));
    assert!(v.disagreement);
    assert!(a.is_finite() && b.is_finite() && wild.is_finite());

    let mut baro = channel();
    assert_eq!(
        baro.vote_dual(Some(900.0), Some(901.0)).unwrap().value,
        900.5
    );
}

#[test]
fn confidence_byte_is_bounded() {
    let vote = Vote {
        value: 1.0f32,
        confidence: 42.0,
        disagreement: false,
        stuck_mask: 0,
        used_mask: 0b001,
        outlier: None,
    };
    assert_eq!(vote.confidence_u8(), 255);
    let vote = Vote {
        confidence: -1.0,
        ..vote
    };
    assert_eq!(vote.confidence_u8(), 0);
}

#[test]
fn identical_input_sequences_produce_identical_votes() {
    let mut a = VotingChannel::new(Config::new(3), tol());
    let mut b = VotingChannel::new(Config::new(3), tol());
    let sequence = [
        [Some(10.0), Some(10.0), Some(10.0)],
        [Some(10.0), Some(12.0), Some(12.0)],
        [Some(10.0), Some(14.0), Some(14.0)],
        [Some(10.0), Some(16.0), Some(16.0)],
        [Some(40.0), Some(18.0), Some(18.0)],
        [None, Some(18.0), Some(18.0)],
    ];
    for tick in sequence {
        assert_eq!(a.vote(tick), b.vote(tick));
    }
    assert_eq!(a.stuck_mask(), b.stuck_mask());
}
