// SPDX-License-Identifier: GPL-3.0-or-later
//! Independent parachute deployment decision layer.
//!
//! This module decides *whether* to release the parachute; it never touches
//! actuators, pins, timers or the AI link. It has no I/O and no allocator, so
//! the same source runs on the controller and in deterministic host tests.
//! The caller feeds one sensor sample per tick and maps the two logical spring
//! outputs onto whatever the board adapter provides (not in scope here).
//!
//! Safety rules, applied in this order on every tick:
//! - a release is one-shot and irreversible: once both outputs are asserted
//!   they stay asserted, and no later sample can change or retract them;
//! - only the flight phases between burnout and touchdown may release — BURN
//!   (motor running) and LANDED refuse, and a refusal changes no sensor state;
//! - both arming channels must be active at the moment of release. A partial
//!   or missing arm refuses rather than falling back to "good enough";
//! - three independent triggers may cause a release: descent after a verified
//!   climb (apogee), an excessive descent rate, or a low altitude while
//!   descending. Every one needs `descent_confirm` consecutive descending
//!   samples, so a single glitch cannot release.
//!
//! All quantities are integers: altitude in whole metres above the launch
//! site, vertical speed in cm/s (positive = climbing, negative = descending).
//! No clock is read: the only time notion is the count of consecutive samples.

use potok_protocol::Phase;

/// Mission thresholds, derived as REQUIREMENTs in `docs/DEPLOYMENT_REQUIREMENTS.md`;
/// none is a measured or standard value. Bad values are rejected loudly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeploymentConfig {
    /// Descent after a climb is only credible this high up.
    pub apogee_min_altitude_m: i32,
    /// Drop from the observed peak before apogee descent counts.
    pub apogee_drop_m: i32,
    /// Peak needed before a climb is considered verified.
    pub climb_confirm_m: i32,
    /// Low-altitude backup trigger, above `min_deploy_altitude_m`.
    pub low_altitude_m: i32,
    /// Descent rate at or below this (cm/s) is an emergency trigger.
    pub fast_descent_cm_s: i32,
    /// Consecutive descending samples needed for any trigger.
    pub descent_confirm: u8,
    /// Floor below which the fast-descent trigger is suppressed.
    pub min_deploy_altitude_m: i32,
}

impl DeploymentConfig {
    /// Nominal values: apogee 30 m / 2 m drop, 5 m climb verification,
    /// 15 m backup altitude, -15 m/s emergency rate, 3 samples, 3 m floor.
    pub const DEFAULT: Self = Self {
        apogee_min_altitude_m: 30,
        apogee_drop_m: 2,
        climb_confirm_m: 5,
        low_altitude_m: 15,
        fast_descent_cm_s: -1_500,
        descent_confirm: 3,
        min_deploy_altitude_m: 3,
    };

    pub fn new(
        apogee_min_altitude_m: i32,
        apogee_drop_m: i32,
        climb_confirm_m: i32,
        low_altitude_m: i32,
        fast_descent_cm_s: i32,
        descent_confirm: u8,
        min_deploy_altitude_m: i32,
    ) -> Self {
        assert!(min_deploy_altitude_m >= 0, "deployment: negative floor");
        assert!(
            low_altitude_m > min_deploy_altitude_m,
            "deployment: low altitude must be above the floor"
        );
        assert!(
            apogee_min_altitude_m >= low_altitude_m,
            "deployment: apogee minimum must be at least the low altitude"
        );
        assert!(
            apogee_drop_m > 0,
            "deployment: apogee drop must be positive"
        );
        assert!(
            climb_confirm_m > 0,
            "deployment: climb confirmation must be positive"
        );
        assert!(
            descent_confirm > 0,
            "deployment: descent confirm must be positive"
        );
        assert!(
            fast_descent_cm_s < 0,
            "deployment: fast descent threshold must be negative"
        );
        Self {
            apogee_min_altitude_m,
            apogee_drop_m,
            climb_confirm_m,
            low_altitude_m,
            fast_descent_cm_s,
            descent_confirm,
            min_deploy_altitude_m,
        }
    }
}

impl Default for DeploymentConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// The two independent arm channels. Both must be active to release.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArmChannels {
    pub a: bool,
    pub b: bool,
}

impl ArmChannels {
    pub const UNARMED: Self = Self { a: false, b: false };
    pub const BOTH: Self = Self { a: true, b: true };

    pub const fn is_complete(self) -> bool {
        self.a && self.b
    }
}

/// One sensor sample. No timestamp: ordering is the caller's tick order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeploymentSample {
    /// Altitude above the launch site, whole metres.
    pub altitude_m: i32,
    /// Vertical speed in cm/s; positive climbs, negative descends.
    pub vertical_speed_cm_s: i32,
}

/// The two logical spring-release outputs. Both are asserted together and
/// stay asserted once a release is committed. No pin mapping lives here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpringOutputs {
    pub a: bool,
    pub b: bool,
}

impl SpringOutputs {
    pub const SAFE: Self = Self { a: false, b: false };
    pub const RELEASED: Self = Self { a: true, b: true };

    pub const fn is_released(self) -> bool {
        self.a && self.b
    }
}

/// Which sensor condition committed the release.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// Descent confirmed after a verified climb, above the apogee floor.
    ApogeeDescent,
    /// Descent rate at or beyond the emergency threshold.
    FastDescent,
    /// Descending at the low-altitude backup threshold.
    LowAltitude,
}

/// Why a tick did not commit a release. Every variant is a real refusal, not
/// an implicit "hold": callers can count and report it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The reported phase is BURN or LANDED (or any phase outside the window).
    WrongPhase { phase: Phase },
    /// One or both arm channels were missing at the decision tick.
    ArmIncomplete { a: bool, b: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// No trigger yet, or the release was already committed earlier.
    Hold,
    /// Both spring outputs were just asserted.
    Deploy(Trigger),
    /// The tick was refused; no sensor state was advanced.
    Refused(Refusal),
}

/// Pure, one-shot deployment controller. Copyable so tests can snapshot it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeploymentController {
    cfg: DeploymentConfig,
    phase: Phase,
    peak_altitude_m: i32,
    descent_samples: u8,
    outputs: SpringOutputs,
    trigger: Option<Trigger>,
    refusals: u32,
}

impl DeploymentController {
    pub const fn new(cfg: DeploymentConfig) -> Self {
        Self {
            cfg,
            phase: Phase::BURN,
            peak_altitude_m: 0,
            descent_samples: 0,
            outputs: SpringOutputs::SAFE,
            trigger: None,
            refusals: 0,
        }
    }

    /// Ground-test reset only. A flight controller never re-arms itself.
    pub fn reset(&mut self) {
        *self = Self::new(self.cfg);
    }

    /// Advance one tick. A refusal leaves peak/descent state untouched.
    pub fn update(&mut self, phase: Phase, arm: ArmChannels, sample: DeploymentSample) -> Decision {
        self.phase = phase;

        if self.outputs.is_released() {
            return Decision::Hold;
        }
        if !phase_allows_release(phase) {
            self.refusals = self.refusals.wrapping_add(1);
            return Decision::Refused(Refusal::WrongPhase { phase });
        }
        if !arm.is_complete() {
            self.refusals = self.refusals.wrapping_add(1);
            return Decision::Refused(Refusal::ArmIncomplete { a: arm.a, b: arm.b });
        }

        if sample.altitude_m > self.peak_altitude_m {
            self.peak_altitude_m = sample.altitude_m;
        }
        self.descent_samples = if sample.vertical_speed_cm_s < 0 {
            self.descent_samples
                .saturating_add(1)
                .min(self.cfg.descent_confirm)
        } else {
            0
        };

        let confirmed = self.descent_samples >= self.cfg.descent_confirm;
        let climbed = self.peak_altitude_m >= self.cfg.climb_confirm_m;
        let trigger = if confirmed
            && climbed
            && sample.altitude_m >= self.cfg.apogee_min_altitude_m
            && sample.altitude_m <= self.peak_altitude_m.saturating_sub(self.cfg.apogee_drop_m)
        {
            Some(Trigger::ApogeeDescent)
        } else if confirmed
            && sample.vertical_speed_cm_s <= self.cfg.fast_descent_cm_s
            && sample.altitude_m >= self.cfg.min_deploy_altitude_m
        {
            Some(Trigger::FastDescent)
        } else if confirmed && sample.altitude_m <= self.cfg.low_altitude_m {
            Some(Trigger::LowAltitude)
        } else {
            None
        };

        match trigger {
            Some(trigger) => {
                self.outputs = SpringOutputs::RELEASED;
                self.trigger = Some(trigger);
                Decision::Deploy(trigger)
            }
            None => Decision::Hold,
        }
    }

    pub const fn outputs(&self) -> SpringOutputs {
        self.outputs
    }

    pub const fn trigger(&self) -> Option<Trigger> {
        self.trigger
    }

    pub const fn phase(&self) -> Phase {
        self.phase
    }

    pub const fn peak_altitude_m(&self) -> i32 {
        self.peak_altitude_m
    }

    pub const fn descent_samples(&self) -> u8 {
        self.descent_samples
    }

    pub const fn refusals(&self) -> u32 {
        self.refusals
    }

    pub const fn config(&self) -> &DeploymentConfig {
        &self.cfg
    }
}

/// BURN and LANDED are outside the release window; COAST, APOGEE and DESCENT
/// are inside it. The comparison is on the wire phase code, which is ordered.
const fn phase_allows_release(phase: Phase) -> bool {
    let code = phase as u8;
    code >= Phase::COAST as u8 && code <= Phase::DESCENT as u8
}

impl Default for DeploymentController {
    fn default() -> Self {
        Self::new(DeploymentConfig::DEFAULT)
    }
}
