// SPDX-License-Identifier: GPL-3.0-or-later
//! Typed MCU runtime profiles and the gate that changes the active one.
//!
//! A [`RuntimeProfile`] bundles the five knobs that currently live as loose
//! constants across the firmware: control gains (`control_pid::PidConfig`),
//! shield thresholds (`interlocks::Limits`), the watchdog deadline
//! (`watchdog::WATCHDOG_TIMEOUT_US`), the telemetry logging cadence and the AI
//! authority cap (`MAX_CORRECTION_RAW`). The three shipped profiles are ordered
//! from most conservative to most capable; [`ProfileId::BALANCED`] reproduces
//! the existing envelopes exactly, so adopting this table changes no default.
//!
//! ## Changing the active profile is gated twice
//!
//! 1. **`hash`** — the caller must supply the exact [`RuntimeProfile::hash`] of
//!    the profile it means to activate. A stale or hand-edited config cannot
//!    select a profile: the mismatch is a typed [`ActivationError`], never a
//!    fallback.
//! 2. **`token`** — an [`Activate`] value, constructible only through
//!    [`Activate::explicit`], so a profile cannot be selected by accident
//!    through a struct literal at a call site.
//!
//! [`ProfileSet::lock`] is the in-flight boundary. It is one-way and has no
//! counterpart: after the firmware locks before its control loop, every
//! `activate` returns [`ActivationError::Locked`]. There is no setter and no
//! path from a UART reply to the active profile.
//!
//! ## Invalid config is LOUD
//!
//! [`RuntimeProfile::validate`] reuses the builders that own each contract
//! (`Pid::new`, `Interlocks::new`) so the messages cannot drift from the code
//! that will consume the values, and asserts the three fields those builders
//! do not own. `ProfileSet::activate` validates before it mutates anything.
//!
//! ## Integration
//!
//! This file declares no `mod` entries elsewhere: `lib.rs` and `main.rs` are
//! owned by the parent task. It uses `crate::` paths so it can be declared in
//! the library unchanged; host tests provide a re-export shim, the same way
//! `control_pid_tests.rs` and `perf_tests.rs` include their modules by path.

use crate::{
    control_pid::{Pid, PidConfig},
    interlocks::Limits,
    watchdog::WATCHDOG_TIMEOUT_US,
    MAX_CORRECTION_RAW,
};

/// The three shipped profiles, ordered from most to least conservative.
///
/// `#[repr(u8)]` makes the discriminant the canonical hash input. Adding a
/// variant is a config change: it changes [`manifest_hash`] and must be
/// re-hashed by every caller that pins a hash.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProfileId {
    Safe = 0,
    Balanced = 1,
    Performance = 2,
}

impl ProfileId {
    pub const SAFE: Self = Self::Safe;
    pub const BALANCED: Self = Self::Balanced;
    pub const PERFORMANCE: Self = Self::Performance;
    /// The unchanged flight recipe. `ProfileSet::new` starts here.
    pub const DEFAULT: Self = Self::BALANCED;
    pub const ALL: [Self; 3] = [Self::SAFE, Self::BALANCED, Self::PERFORMANCE];

    /// The immutable configuration for this profile.
    pub const fn config(self) -> RuntimeProfile {
        match self {
            Self::Safe => SAFE,
            Self::Balanced => BALANCED,
            Self::Performance => PERFORMANCE,
        }
    }
}

/// One complete runtime profile. All fields are plain data; no state, no I/O.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RuntimeProfile {
    pub id: ProfileId,
    /// Per-step PID gains for each stabilisation axis.
    pub control: PidConfig<f32>,
    /// Independent shield gates, including the Mach ceiling.
    pub shield: Limits,
    /// Software watchdog deadline in microseconds. Must not outlast the
    /// hardware IWDG window: the hardware would reset first.
    pub watchdog_deadline_us: u32,
    /// Emit one telemetry record every N control ticks. `1` logs every tick.
    pub log_every_ticks: u32,
    /// Hard ceiling on an AI correction, in raw units, on top of the code veto.
    /// A profile may lower this; it can never raise it above `MAX_CORRECTION_RAW`.
    pub ai_authority_cap: i16,
}

/// Most conservative: half the code veto, an earlier Mach ceiling, a short
/// deadline and a record on every tick.
pub const SAFE: RuntimeProfile = RuntimeProfile {
    id: ProfileId::SAFE,
    control: PidConfig {
        kp: 0.5,
        ki_dt: 0.0,
        kd_dt: 0.5,
        out_limit: 40.0,
        i_limit: 5.0,
        deriv_alpha: 0.5,
    },
    shield: Limits {
        max_command: 40,
        max_delta: 4,
        min_interval_us: 2_000,
        mach_limit: 0.5,
    },
    watchdog_deadline_us: 50_000,
    log_every_ticks: 1,
    ai_authority_cap: 40,
};

/// Existing flight recipe. `Limits::DEFAULT`, `WATCHDOG_TIMEOUT_US` and
/// `MAX_CORRECTION_RAW` are carried through unchanged; no default is retuned.
pub const BALANCED: RuntimeProfile = RuntimeProfile {
    id: ProfileId::BALANCED,
    control: PidConfig {
        kp: 1.0,
        ki_dt: 0.0,
        kd_dt: 0.0,
        out_limit: MAX_CORRECTION_RAW as f32,
        i_limit: 10.0,
        deriv_alpha: 1.0,
    },
    shield: Limits::DEFAULT,
    watchdog_deadline_us: WATCHDOG_TIMEOUT_US,
    log_every_ticks: 1,
    ai_authority_cap: MAX_CORRECTION_RAW,
};

/// Most capable, still inside the flight envelope: Mach stays at 0.6 and the
/// authority cap stays at the code veto, but slewing is less restricted and
/// telemetry is sampled every tenth tick to free loop time.
pub const PERFORMANCE: RuntimeProfile = RuntimeProfile {
    id: ProfileId::PERFORMANCE,
    control: PidConfig {
        kp: 1.0,
        ki_dt: 0.01,
        kd_dt: 0.25,
        out_limit: MAX_CORRECTION_RAW as f32,
        i_limit: 10.0,
        deriv_alpha: 0.75,
    },
    shield: Limits {
        max_command: MAX_CORRECTION_RAW,
        max_delta: 16,
        min_interval_us: 500,
        mach_limit: 0.6,
    },
    watchdog_deadline_us: WATCHDOG_TIMEOUT_US,
    log_every_ticks: 10,
    ai_authority_cap: MAX_CORRECTION_RAW,
};

impl RuntimeProfile {
    /// Loud rejection of an unusable table entry. The two builders below own
    /// the gain and shield contracts; restating their messages here would let
    /// this check drift from the code that consumes the values.
    pub fn validate(self) {
        let _ = Pid::<f32>::new(self.control);
        let _ = crate::interlocks::Interlocks::new(self.shield);
        assert!(
            self.control.out_limit <= MAX_CORRECTION_RAW as f32,
            "profile: control.out_limit must not exceed MAX_CORRECTION_RAW"
        );
        assert!(
            self.watchdog_deadline_us > 0 && self.watchdog_deadline_us <= WATCHDOG_TIMEOUT_US,
            "profile: watchdog_deadline_us must be in 1..=WATCHDOG_TIMEOUT_US"
        );
        assert!(
            self.log_every_ticks > 0,
            "profile: log_every_ticks must be positive"
        );
        assert!(
            self.ai_authority_cap > 0 && self.ai_authority_cap <= MAX_CORRECTION_RAW,
            "profile: ai_authority_cap must be in 1..=MAX_CORRECTION_RAW"
        );
    }

    /// FNV-1a over the canonical little-endian field layout. Floats are hashed
    /// by bits, so the value is identical on host and MCU and NaN cannot be
    /// normalized into a different "valid" hash.
    pub fn hash(self) -> u64 {
        let mut h = CanonicalHash::new();
        h.u8(self.id as u8);
        h.f32(self.control.kp);
        h.f32(self.control.ki_dt);
        h.f32(self.control.kd_dt);
        h.f32(self.control.out_limit);
        h.f32(self.control.i_limit);
        h.f32(self.control.deriv_alpha);
        h.i16(self.shield.max_command);
        h.i16(self.shield.max_delta);
        h.u32(self.shield.min_interval_us);
        h.f32(self.shield.mach_limit);
        h.u32(self.watchdog_deadline_us);
        h.u32(self.log_every_ticks);
        h.i16(self.ai_authority_cap);
        h.finish()
    }
}

/// Canonical hash of the whole table. A change to any profile changes it.
pub fn manifest_hash() -> u64 {
    let mut h = CanonicalHash::new();
    for id in ProfileId::ALL {
        h.u64(id.config().hash());
    }
    h.finish()
}

/// Convenience for the caller that must supply a profile's hash.
pub fn profile_hash(id: ProfileId) -> u64 {
    id.config().hash()
}

/// Proof of an explicit activation intent. The field is private, so the only
/// way to obtain one is [`Activate::explicit`]. Zero-sized: no runtime cost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Activate(());

impl Activate {
    pub const fn explicit() -> Self {
        Self(())
    }
}

/// Why an activation was refused. No variant is a warning: the active profile
/// is never changed on a failed activation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActivationError {
    /// The supplied hash is not the hash of the target profile.
    HashMismatch { expected: u64, got: u64 },
    /// The set is locked; the firmware is in flight and cannot switch.
    Locked,
}

/// The active profile and its one-way in-flight lock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProfileSet {
    active: ProfileId,
    locked: bool,
}

impl ProfileSet {
    /// Boot state: the unchanged default profile, unlocked and ready for one
    /// explicit pre-flight selection.
    pub const fn new() -> Self {
        Self {
            active: ProfileId::DEFAULT,
            locked: false,
        }
    }

    /// Boot directly into a chosen profile with hash + explicit intent.
    pub fn boot(id: ProfileId, hash: u64, token: Activate) -> Result<Self, ActivationError> {
        let mut set = Self::new();
        set.activate(id, hash, token)?;
        Ok(set)
    }

    /// Select a profile. Both gates are required and checked before any state
    /// changes; a locked set refuses even a perfectly hashed request.
    ///
    /// Returns the previously active profile on success.
    pub fn activate(
        &mut self,
        id: ProfileId,
        hash: u64,
        token: Activate,
    ) -> Result<ProfileId, ActivationError> {
        let _ = token;
        if self.locked {
            return Err(ActivationError::Locked);
        }
        let config = id.config();
        config.validate();
        let expected = config.hash();
        if hash != expected {
            return Err(ActivationError::HashMismatch {
                expected,
                got: hash,
            });
        }
        let previous = self.active;
        self.active = id;
        Ok(previous)
    }

    /// One-way transition to in flight. Call before the 500 Hz loop starts;
    /// there is deliberately no `unlock`.
    pub const fn lock(&mut self) {
        self.locked = true;
    }

    pub const fn active(&self) -> ProfileId {
        self.active
    }

    pub const fn config(&self) -> RuntimeProfile {
        self.active.config()
    }

    pub const fn is_locked(&self) -> bool {
        self.locked
    }
}

impl Default for ProfileSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Minimal incremental FNV-1a; `no_std`, no allocation, no trait objects.
#[derive(Clone, Copy)]
struct CanonicalHash(u64);

impl CanonicalHash {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    const fn new() -> Self {
        Self(Self::OFFSET)
    }

    fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= byte as u64;
            self.0 = self.0.wrapping_mul(Self::PRIME);
        }
    }

    fn u8(&mut self, value: u8) {
        self.bytes(&value.to_le_bytes());
    }

    fn i16(&mut self, value: i16) {
        self.bytes(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        self.bytes(&value.to_bits().to_le_bytes());
    }

    const fn finish(self) -> u64 {
        self.0
    }
}
