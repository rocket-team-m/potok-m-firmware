// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the typed runtime profiles. `lib.rs` is owned by the parent
//! task, so the module is included by path; the re-exports below make its
//! `crate::` paths resolve to the firmware's real types, unchanged.

pub use potok_m_firmware::{control_pid, interlocks, watchdog, MAX_CORRECTION_RAW};

#[path = "../src/profile.rs"]
mod profile;

use control_pid::Pid;
use interlocks::Interlocks;
use profile::{
    manifest_hash, profile_hash, Activate, ActivationError, ProfileId, ProfileSet, RuntimeProfile,
    BALANCED,
};
use watchdog::WATCHDOG_TIMEOUT_US;

fn balanced() -> RuntimeProfile {
    BALANCED
}

#[test]
fn default_set_is_balanced_and_carries_the_existing_envelopes() {
    let set = ProfileSet::new();
    assert_eq!(ProfileId::DEFAULT, ProfileId::BALANCED);
    assert_eq!(set.active(), ProfileId::BALANCED);
    assert!(!set.is_locked());
    assert_eq!(set, ProfileSet::default());

    let cfg = set.config();
    assert_eq!(cfg.id, ProfileId::BALANCED);
    assert_eq!(cfg.shield, interlocks::Limits::DEFAULT);
    assert_eq!(cfg.shield.mach_limit, 0.6);
    assert_eq!(cfg.watchdog_deadline_us, WATCHDOG_TIMEOUT_US);
    assert_eq!(cfg.ai_authority_cap, MAX_CORRECTION_RAW);
    assert_eq!(cfg.control.out_limit, MAX_CORRECTION_RAW as f32);
    assert_eq!(cfg.log_every_ticks, 1);
}

#[test]
fn every_shipped_profile_is_valid_and_directly_consumable() {
    for id in ProfileId::ALL {
        let cfg = id.config();
        assert_eq!(cfg.id, id);
        cfg.validate();
        // The stored values feed the real builders without conversion: this is
        // the test that the profile table cannot drift away from its consumers.
        let _ = Pid::<f32>::new(cfg.control);
        let _ = Interlocks::new(cfg.shield);
        assert_eq!(id.config().hash(), profile_hash(id));
    }
}

#[test]
fn profiles_form_a_monotone_safety_ladder() {
    let safe = ProfileId::SAFE.config();
    let balanced = ProfileId::BALANCED.config();
    let perf = ProfileId::PERFORMANCE.config();

    assert!(safe.ai_authority_cap < balanced.ai_authority_cap);
    assert_eq!(balanced.ai_authority_cap, perf.ai_authority_cap);
    assert_eq!(perf.ai_authority_cap, MAX_CORRECTION_RAW);

    // The M <= 0.6 flight ceiling is not a knob: no profile may raise it.
    assert!(safe.shield.mach_limit < balanced.shield.mach_limit);
    assert_eq!(balanced.shield.mach_limit, perf.shield.mach_limit);
    assert_eq!(perf.shield.mach_limit, 0.6);

    assert!(safe.watchdog_deadline_us < balanced.watchdog_deadline_us);
    assert_eq!(balanced.watchdog_deadline_us, perf.watchdog_deadline_us);
    assert!(safe.log_every_ticks <= balanced.log_every_ticks);
    assert!(balanced.log_every_ticks < perf.log_every_ticks);

    assert!(safe.control.out_limit < balanced.control.out_limit);
    assert_eq!(balanced.control.out_limit, perf.control.out_limit);
}

#[test]
fn activation_requires_the_exact_hash() {
    let mut set = ProfileSet::new();
    let hash = profile_hash(ProfileId::SAFE);
    let wrong = hash ^ 1;

    let err = set
        .activate(ProfileId::SAFE, wrong, Activate::explicit())
        .unwrap_err();
    assert_eq!(
        err,
        ActivationError::HashMismatch {
            expected: hash,
            got: wrong,
        }
    );
    assert_eq!(set.active(), ProfileId::BALANCED);
    assert!(!set.is_locked());

    assert_eq!(
        set.activate(ProfileId::SAFE, hash, Activate::explicit()),
        Ok(ProfileId::BALANCED)
    );
    assert_eq!(set.active(), ProfileId::SAFE);
    assert_eq!(set.config().ai_authority_cap, 40);
}

#[test]
fn boot_selection_is_also_hashed_and_explicit() {
    let hash = profile_hash(ProfileId::PERFORMANCE);
    let set = ProfileSet::boot(ProfileId::PERFORMANCE, hash, Activate::explicit()).unwrap();
    assert_eq!(set.active(), ProfileId::PERFORMANCE);
    assert_eq!(set.config().log_every_ticks, 10);

    assert!(ProfileSet::boot(ProfileId::PERFORMANCE, hash ^ 1, Activate::explicit()).is_err());
}

#[test]
fn locked_set_refuses_every_switch_even_with_the_right_hash() {
    let mut set = ProfileSet::new();
    set.lock();
    assert!(set.is_locked());

    let correct = profile_hash(ProfileId::PERFORMANCE);
    assert_eq!(
        set.activate(ProfileId::PERFORMANCE, correct, Activate::explicit()),
        Err(ActivationError::Locked)
    );
    // Lock is checked before the hash gate, and re-selecting the same profile
    // is a switch like any other once the firmware is in flight.
    assert_eq!(
        set.activate(ProfileId::BALANCED, 0, Activate::explicit()),
        Err(ActivationError::Locked)
    );
    assert_eq!(set.active(), ProfileId::BALANCED);
}

#[test]
fn hashes_are_stable_distinct_and_sensitive_to_a_field_change() {
    for id in ProfileId::ALL {
        assert_eq!(profile_hash(id), id.config().hash());
        assert_eq!(profile_hash(id), profile_hash(id));
    }
    assert_ne!(
        profile_hash(ProfileId::SAFE),
        profile_hash(ProfileId::BALANCED)
    );
    assert_ne!(
        profile_hash(ProfileId::BALANCED),
        profile_hash(ProfileId::PERFORMANCE)
    );
    assert_ne!(
        profile_hash(ProfileId::SAFE),
        profile_hash(ProfileId::PERFORMANCE)
    );
    assert_eq!(manifest_hash(), manifest_hash());

    let mut changed = balanced();
    changed.ai_authority_cap -= 1;
    assert_ne!(changed.hash(), balanced().hash());
}

#[test]
fn activation_token_is_a_zero_sized_explicit_proof() {
    assert_eq!(core::mem::size_of::<Activate>(), 0);
    assert_eq!(Activate::explicit(), Activate::explicit());
}

#[test]
#[should_panic(expected = "pid: kp must be non-negative")]
fn negative_gain_is_loud() {
    let mut cfg = balanced();
    cfg.control.kp = -1.0;
    cfg.validate();
}

#[test]
#[should_panic(expected = "pid: out_limit must be positive")]
fn zero_output_limit_is_loud() {
    let mut cfg = balanced();
    cfg.control.out_limit = 0.0;
    cfg.validate();
}

#[test]
#[should_panic(expected = "pid: deriv_alpha must be in (0, 1]")]
fn deriv_alpha_above_one_is_loud() {
    let mut cfg = balanced();
    cfg.control.deriv_alpha = 1.5;
    cfg.validate();
}

#[test]
#[should_panic(expected = "max_command must be positive")]
fn zero_shield_command_bound_is_loud() {
    let mut cfg = balanced();
    cfg.shield.max_command = 0;
    cfg.validate();
}

#[test]
#[should_panic(expected = "mach_limit must be finite and positive")]
fn non_finite_mach_ceiling_is_loud() {
    let mut cfg = balanced();
    cfg.shield.mach_limit = f32::NAN;
    cfg.validate();
}

#[test]
#[should_panic(expected = "watchdog_deadline_us")]
fn zero_watchdog_deadline_is_loud() {
    let mut cfg = balanced();
    cfg.watchdog_deadline_us = 0;
    cfg.validate();
}

#[test]
#[should_panic(expected = "watchdog_deadline_us")]
fn deadline_beyond_the_hardware_window_is_loud() {
    let mut cfg = balanced();
    cfg.watchdog_deadline_us = WATCHDOG_TIMEOUT_US + 1;
    cfg.validate();
}

#[test]
#[should_panic(expected = "log_every_ticks must be positive")]
fn zero_logging_cadence_is_loud() {
    let mut cfg = balanced();
    cfg.log_every_ticks = 0;
    cfg.validate();
}

#[test]
#[should_panic(expected = "ai_authority_cap")]
fn authority_cap_above_the_code_veto_is_loud() {
    let mut cfg = balanced();
    cfg.ai_authority_cap = MAX_CORRECTION_RAW + 1;
    cfg.validate();
}

#[test]
#[should_panic(expected = "control.out_limit")]
fn output_limit_above_the_code_veto_is_loud() {
    let mut cfg = balanced();
    cfg.control.out_limit = MAX_CORRECTION_RAW as f32 + 1.0;
    cfg.validate();
}
