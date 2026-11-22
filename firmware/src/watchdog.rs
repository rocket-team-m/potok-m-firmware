// SPDX-License-Identifier: GPL-3.0-or-later
//! Independent watchdog: abstract feed, hardware setup, feed policy.
//!
//! The software policy below is hardware-free: it is compiled and tested on
//! the host (`tests/watchdog_tests.rs`). The hardware section is compiled only
//! for the firmware target and is never called by the policy.

/// Feed policy: the control task may feed only after a complete cycle with a
/// valid STM32-owned phase, and only while the deadman has not tripped.
pub const fn may_feed(cycle_complete: bool, deadman_tripped: bool) -> bool {
    cycle_complete && !deadman_tripped
}

pub trait Watchdog {
    fn pet(&mut self);
}

pub const WATCHDOG_TIMEOUT_US: u32 = 100_000;

#[cfg(target_os = "none")]
pub type Iwdg<'d> = embassy_stm32::wdg::IndependentWatchdog<'d, embassy_stm32::peripherals::IWDG1>;

/// Configures and starts IWDG. Once unleashed, only a reset stops it.
#[cfg(target_os = "none")]
pub fn setup_iwdg(
    instance: embassy_stm32::Peri<'static, embassy_stm32::peripherals::IWDG1>,
) -> Iwdg<'static> {
    let mut wdg = embassy_stm32::wdg::IndependentWatchdog::new(instance, WATCHDOG_TIMEOUT_US);
    wdg.unleash();
    wdg
}

#[cfg(target_os = "none")]
impl Watchdog for Iwdg<'_> {
    fn pet(&mut self) {
        embassy_stm32::wdg::IndependentWatchdog::pet(self);
    }
}

/// The control loop runs at 500 Hz (2 ms tick in the Embassy adapter).
pub const CONTROL_PERIOD_US: u32 = 2_000;
/// A cycle arriving later than one period plus this slack has missed its
/// deadline. Jitter below the slack is not a fault.
pub const DEADLINE_SLACK_US: u32 = 250;
/// Three consecutive missed deadlines trip the software deadman. At 500 Hz
/// that is well inside the 100 ms hardware IWDG window.
pub const DEADLINE_LIMIT: u8 = 3;

/// Escalation of a missed control deadline.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    Healthy,
    Degraded,
    Tripped,
}

/// Software deadman: watches the control loop's own cadence, not the AI link.
///
/// A late cycle counts as a miss; an on-time cycle clears the escalation.
/// Once tripped, watchdog feeding is withheld so the hardware IWDG resets the
/// board. This is independent of the link's AI latch: a latched AI must not
/// stop the control loop from feeding, or recovery would be impossible.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftwareDeadman {
    missed: u8,
    last_cycle_us: Option<u32>,
    tripped: bool,
}

impl SoftwareDeadman {
    pub const fn new() -> Self {
        Self {
            missed: 0,
            last_cycle_us: None,
            tripped: false,
        }
    }

    /// Record one control-cycle notification. Returns whether it met its
    /// deadline. The first notification establishes the cadence, it cannot
    /// be late. Microsecond wraparound is handled with wrapping subtraction.
    pub fn note_cycle(&mut self, now_us: u32) -> bool {
        let on_time = match self.last_cycle_us {
            Some(last) => now_us.wrapping_sub(last) <= CONTROL_PERIOD_US + DEADLINE_SLACK_US,
            None => true,
        };
        self.last_cycle_us = Some(now_us);
        if on_time {
            self.missed = 0;
            self.tripped = false;
        } else {
            if self.missed < DEADLINE_LIMIT {
                self.missed += 1;
            }
            if self.missed >= DEADLINE_LIMIT {
                self.tripped = true;
            }
        }
        on_time
    }

    pub const fn missed(&self) -> u8 {
        self.missed
    }

    pub const fn tripped(&self) -> bool {
        self.tripped
    }

    pub const fn severity(&self) -> Severity {
        if self.tripped {
            Severity::Tripped
        } else if self.missed != 0 {
            Severity::Degraded
        } else {
            Severity::Healthy
        }
    }

    /// Explicit firmware reset only. Late-but-valid cycles can de-escalate on
    /// their own; this clears the state when the loop was stopped outright.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for SoftwareDeadman {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of the watchdog feeding policy for one control cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedDecision {
    Feed,
    Withhold,
}

/// Hardware-free watchdog feeding policy.
///
/// Only `on_control_cycle` can arm a feed; there is no path from the UART
/// callback to a feed. The AI latch is tracked here solely to force zero
/// output and to give it a reset independent of the deadline monitor:
/// clearing the AI latch never disarms the watchdog, and resetting the
/// watchdog never clears the AI latch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogPolicy {
    deadman: SoftwareDeadman,
    cycle_armed: bool,
    ai_latch: bool,
}

impl WatchdogPolicy {
    pub const fn new() -> Self {
        Self {
            deadman: SoftwareDeadman::new(),
            cycle_armed: false,
            ai_latch: false,
        }
    }

    /// Called at the end of a full control cycle with a valid STM32-owned
    /// phase. This is the only feed-arming path. `ai_latched` is deliberately
    /// not a feed gate: the loop must keep feeding while AI is latched off so
    /// the latch can be reset explicitly.
    pub fn on_control_cycle(
        &mut self,
        now_us: u32,
        cycle_complete: bool,
        phase_valid: bool,
    ) -> FeedDecision {
        self.deadman.note_cycle(now_us);
        self.cycle_armed = cycle_complete && phase_valid;
        self.feed_decision()
    }

    pub const fn feed_decision(&self) -> FeedDecision {
        if self.cycle_armed && !self.deadman.tripped() {
            FeedDecision::Feed
        } else {
            FeedDecision::Withhold
        }
    }

    pub const fn may_feed(&self) -> bool {
        self.cycle_armed && !self.deadman.tripped()
    }

    /// True while the latch is set or the deadline monitor has tripped: the
    /// controller must apply zero even though feeding may still be allowed.
    pub const fn must_zero_output(&self) -> bool {
        self.ai_latch || self.deadman.tripped()
    }

    /// Declared by the safety logic when AI is latched off (three bad frames
    /// or an unhealthy reply). Does not touch the deadline monitor.
    pub fn latch_ai(&mut self) {
        self.ai_latch = true;
    }

    /// Explicit firmware reset of the AI latch only. Never disarms the
    /// watchdog deadman.
    pub fn reset_ai_latch(&mut self) {
        self.ai_latch = false;
    }

    /// Explicit firmware reset of the deadline monitor only. Never clears the
    /// AI latch. Also disarms feeding until the next complete control cycle.
    pub fn reset_deadman(&mut self) {
        self.deadman.reset();
        self.cycle_armed = false;
    }

    pub const fn ai_latched(&self) -> bool {
        self.ai_latch
    }

    pub const fn deadman(&self) -> &SoftwareDeadman {
        &self.deadman
    }

    pub const fn severity(&self) -> Severity {
        self.deadman.severity()
    }
}

impl Default for WatchdogPolicy {
    fn default() -> Self {
        Self::new()
    }
}
