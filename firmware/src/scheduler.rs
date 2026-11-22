// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic 500 Hz control tick scheduler.
//!
//! Deadlines live on a fixed grid `origin + n * period`; they are never
//! re-phased from the arrival time, so the tick phase cannot drift. Time enters
//! only through [`Clock::now_us`] (firmware) or the `now_us` argument of
//! [`Scheduler::tick`] (host tests). Nothing here reads a wall clock, uses
//! randomness, or allocates: every field is a fixed-width integer and the
//! whole scheduler is `Copy`.
//!
//! The caller waits until [`Scheduler::next_deadline_us`] and then releases a
//! tick with [`Scheduler::tick`] / [`Scheduler::tick_now`]. An early call is a
//! no-op ([`Scheduler::tick`] returns `None`); a late call records the lateness
//! as jitter and counts how many whole deadlines were crossed as missed ticks,
//! then advances to the next point on the same grid.

/// Control rate.
pub const TICK_HZ: u32 = 500;
/// Exact tick period in microseconds: 500 Hz -> 2000 us.
pub const TICK_PERIOD_US: u32 = 1_000_000 / TICK_HZ;

/// Monotonic microsecond clock. The host supplies a scripted mock; the
/// firmware supplies an Embassy-backed adapter. `u32` matching `Board::now_us`.
pub trait Clock {
    fn now_us(&self) -> u32;
}

/// Bounds and running totals of the observed arrival error. No storage grows
/// with the number of samples. `min_us`/`max_us` are meaningful when
/// `samples > 0`; before the first sample both read `0`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JitterStats {
    pub samples: u32,
    /// Smallest observed arrival error (us). Always >= 0: early calls are not
    /// released as ticks.
    pub min_us: i32,
    /// Largest observed arrival error (us).
    pub max_us: i32,
    /// Most recent arrival error (us).
    pub last_us: i32,
    /// Sum of absolute errors (us, widened to u64 so long runs cannot wrap).
    pub sum_abs_us: u64,
}

impl JitterStats {
    /// Rounded mean absolute arrival error (us); 0 before the first sample.
    pub fn mean_abs_us(&self) -> u32 {
        if self.samples == 0 {
            0
        } else {
            (self.sum_abs_us / u64::from(self.samples)) as u32
        }
    }

    fn record(&mut self, error_us: i32) {
        if self.samples == 0 {
            self.min_us = error_us;
            self.max_us = error_us;
        } else {
            if error_us < self.min_us {
                self.min_us = error_us;
            }
            if error_us > self.max_us {
                self.max_us = error_us;
            }
        }
        self.samples = self.samples.wrapping_add(1);
        self.last_us = error_us;
        let abs = if error_us < 0 {
            -(i64::from(error_us))
        } else {
            i64::from(error_us)
        };
        self.sum_abs_us = self.sum_abs_us.wrapping_add(abs as u64);
    }
}

/// A released control tick. All times are absolute microseconds on the
/// scheduler's monotonic timeline (wrapping at `u32::MAX` like Embassy time).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickReport {
    /// Grid index of the tick that fired.
    pub index: u32,
    /// The deadline this tick was due at (before any catch-up).
    pub deadline_us: u32,
    /// The clock reading at which the tick was released.
    pub arrival_us: u32,
    /// `arrival_us - deadline_us` in us (>= 0; positive means late).
    pub error_us: i32,
    /// Whole intermediate deadlines crossed to catch up (0 = on time).
    pub missed: u32,
}

/// Fixed-layout, allocation-free 500 Hz ticker. `origin_us` is the deadline of
/// tick index 0; every later deadline is computed by multiplication, never by
/// accumulating observations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Scheduler {
    origin_us: u32,
    period_us: u32,
    next_index: u32,
    next_deadline_us: u32,
    ticks: u32,
    missed: u32,
    jitter: JitterStats,
}

impl Scheduler {
    /// A 500 Hz ticker whose tick 0 is due at `origin_us`.
    pub const fn new(origin_us: u32) -> Self {
        Self::with_period(origin_us, TICK_PERIOD_US)
    }

    /// Custom period (mainly for deterministic host tests). A zero period is
    /// rejected at compile time when used in a const context.
    pub const fn with_period(origin_us: u32, period_us: u32) -> Self {
        assert!(period_us != 0, "tick period must be non-zero");
        // Signed lateness math in `tick` needs the period to fit in i32.
        assert!(period_us <= i32::MAX as u32, "tick period too large");
        Self {
            origin_us,
            period_us,
            next_index: 0,
            next_deadline_us: origin_us,
            ticks: 0,
            missed: 0,
            jitter: JitterStats {
                samples: 0,
                min_us: 0,
                max_us: 0,
                last_us: 0,
                sum_abs_us: 0,
            },
        }
    }

    pub const fn period_us(&self) -> u32 {
        self.period_us
    }

    pub const fn origin_us(&self) -> u32 {
        self.origin_us
    }

    pub const fn next_index(&self) -> u32 {
        self.next_index
    }

    pub const fn next_deadline_us(&self) -> u32 {
        self.next_deadline_us
    }

    /// Total ticks released so far (excludes missed deadlines).
    pub const fn ticks(&self) -> u32 {
        self.ticks
    }

    /// Total whole deadlines crossed and skipped so far.
    pub const fn missed_ticks(&self) -> u32 {
        self.missed
    }

    pub const fn jitter(&self) -> JitterStats {
        self.jitter
    }

    /// Deadline of an arbitrary grid index, computed from the origin.
    pub const fn deadline_for(&self, index: u32) -> u32 {
        self.origin_us
            .wrapping_add(index.wrapping_mul(self.period_us))
    }

    /// Microseconds from `now_us` to the next deadline, or 0 if it is due.
    pub fn us_until_next(&self, now_us: u32) -> u32 {
        let delta = (now_us.wrapping_sub(self.next_deadline_us)) as i32;
        if delta < 0 {
            (-(i64::from(delta))) as u32
        } else {
            0
        }
    }

    /// Release the tick due at the current deadline when `now_us` has reached
    /// it. `None` means "too early, nothing consumed".
    pub fn tick(&mut self, now_us: u32) -> Option<TickReport> {
        // Wrapping subtraction keeps the comparison correct across the u32
        // microsecond rollover (Embassy time wraps the same way).
        let delta = (now_us.wrapping_sub(self.next_deadline_us)) as i32;
        if delta < 0 {
            return None;
        }

        let missed = (delta / self.period_us as i32) as u32;
        let steps = missed.wrapping_add(1);
        let index = self.next_index;
        let deadline_us = self.next_deadline_us;

        // Rejoin the fixed grid: advance by whole periods, never from `now`.
        self.next_deadline_us = self
            .next_deadline_us
            .wrapping_add(steps.wrapping_mul(self.period_us));
        self.next_index = self.next_index.wrapping_add(steps);
        self.ticks = self.ticks.wrapping_add(1);
        self.missed = self.missed.wrapping_add(missed);
        self.jitter.record(delta);

        Some(TickReport {
            index,
            deadline_us,
            arrival_us: now_us,
            error_us: delta,
            missed,
        })
    }

    /// Same as [`Scheduler::tick`], taking the time from a [`Clock`]. This is
    /// the host-testable seam; the firmware passes its `Board`-backed clock.
    pub fn tick_now<C: Clock>(&mut self, clock: &C) -> Option<TickReport> {
        self.tick(clock.now_us())
    }

    /// Restart on the grid at `origin_us`, clearing all counters and stats.
    pub fn reset(&mut self, origin_us: u32) {
        *self = Self::with_period(origin_us, self.period_us);
    }
}
