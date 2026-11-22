// SPDX-License-Identifier: GPL-3.0-or-later

//! Second-stage separation/ignition sequencer: pure, deterministic logic.
//!
//! The clock is the flight phase, not a wall clock: arming must come from two
//! independent channels, the trigger delay is measured from the later of two
//! redundant phase-entry stamps, and the separation confirmation must agree
//! across two redundant sensors before ignition is commanded. A disagreement
//! between any redundant pair latches an abort. Once separation or ignition is
//! commanded the state is latched; only an explicit [`Sequencer::reset`] clears
//! it. Nothing here allocates, reads a real clock, or names a pin: the board
//! adapter maps [`Output`] to hardware from the schematic.
//!
//! Time is supplied by the caller as monotonic microseconds, so the same
//! script of [`Sample`]s always produces the same trace on host and target.

use potok_protocol::Phase;

/// Logical actuation command. The board owns the physical mapping; no pin
/// numbers or alternate functions exist in this layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Output {
    None,
    Separation,
    Ignition,
}

/// Two independent arm assertions, latched separately. Both must have been
/// seen before the sequencer leaves [`State::Idle`]; a single stuck channel can
/// never arm it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArmChannels {
    pub primary: bool,
    pub secondary: bool,
}

/// One phase observation from an independent sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseReading {
    pub phase: Phase,
    /// Microseconds at which this sensor entered `phase`.
    pub at_us: u64,
}

/// Two readings of the same physical quantity from independent sensors, in
/// the same units. Same units and same sign are a caller precondition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorPair {
    pub a: i32,
    pub b: i32,
}

impl SensorPair {
    /// Absolute difference, widened so `i32::MIN`/`i32::MAX` cannot overflow.
    pub const fn spread(self) -> u32 {
        let a = self.a as i64;
        let b = self.b as i64;
        let d = if a > b { a - b } else { b - a };
        d as u32
    }
}

/// Fixed plan, supplied once. All values are caller-owned constants; there is
/// no default that could hide an unset timing budget.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Flight phase whose entry starts the ignition timer.
    pub trigger: Phase,
    /// Minimum delay after entry into `trigger` before separation is commanded.
    pub delay_us: u64,
    /// Maximum allowed difference between the two phase-entry stamps.
    pub phase_skew_tolerance_us: u64,
    /// Maximum allowed `SensorPair::spread` on every active tick.
    pub sensor_tolerance: u32,
    /// Both separation sensors at or above this level confirm separation.
    pub separation_level: i32,
}

/// One evaluation step. Every field is explicit; there is no implicit time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub now_us: u64,
    /// Two phase channels, indexed by channel identity (not by hardware pin).
    pub phase: [PhaseReading; 2],
    pub sensors: SensorPair,
}

/// Why a tick was refused or an abort was latched.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    ArmWhenNotIdle,
    SensorMismatch { a: i32, b: i32 },
    PhaseMismatch { a: Phase, b: Phase },
    PhaseStampSkew { a_us: u64, b_us: u64 },
    PhaseStampInFuture { at_us: u64, now_us: u64 },
    TimeWentBackwards { now_us: u64, last_us: u64 },
    PhaseWindowMissed { trigger: Phase, observed: Phase },
    Latched,
}

/// Sequencer state. `Separating`, `Igniting` and `Aborted` are one-way: only
/// [`Sequencer::reset`] returns to `Idle`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Idle,
    Armed,
    Separating { at_us: u64 },
    Igniting { at_us: u64 },
    Aborted(Fault),
}

/// Allocation-free sequencer. Every method is a pure function of its inputs
/// and the current value, so two instances fed the same script stay identical.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sequencer {
    plan: Plan,
    state: State,
    arm_primary: bool,
    arm_secondary: bool,
    last_now_us: u64,
    aborts: u32,
    separations: u32,
    ignitions: u32,
}

impl Sequencer {
    pub const fn new(plan: Plan) -> Self {
        Self {
            plan,
            state: State::Idle,
            arm_primary: false,
            arm_secondary: false,
            last_now_us: 0,
            aborts: 0,
            separations: 0,
            ignitions: 0,
        }
    }

    pub const fn plan(&self) -> Plan {
        self.plan
    }

    pub const fn state(&self) -> State {
        self.state
    }

    /// What has been latched on each independent arm channel.
    pub const fn arm_channels(&self) -> ArmChannels {
        ArmChannels {
            primary: self.arm_primary,
            secondary: self.arm_secondary,
        }
    }

    pub const fn last_now_us(&self) -> u64 {
        self.last_now_us
    }

    pub const fn last_fault(&self) -> Option<Fault> {
        match self.state {
            State::Aborted(fault) => Some(fault),
            _ => None,
        }
    }

    pub const fn abort_count(&self) -> u32 {
        self.aborts
    }

    pub const fn separation_count(&self) -> u32 {
        self.separations
    }

    pub const fn ignition_count(&self) -> u32 {
        self.ignitions
    }

    pub const fn is_latched(&self) -> bool {
        matches!(
            self.state,
            State::Separating { .. } | State::Igniting { .. } | State::Aborted(_)
        )
    }

    /// Assert one or both arm channels. Each `true` latches independently, so
    /// the two checks may arrive on different ticks. A single channel cannot
    /// arm the sequencer. Once armed or latched, further arm attempts are
    /// refused rather than silently ignored.
    pub fn arm(&mut self, channels: ArmChannels) -> Result<(), Fault> {
        if !matches!(self.state, State::Idle) {
            return Err(Fault::ArmWhenNotIdle);
        }
        self.arm_primary |= channels.primary;
        self.arm_secondary |= channels.secondary;
        if self.arm_primary && self.arm_secondary {
            self.state = State::Armed;
        }
        Ok(())
    }

    /// Evaluate one tick. On a fault the abort is latched and returned as
    /// `Err`; a subsequent tick on a latched sequencer returns
    /// [`Fault::Latched`] without changing anything.
    pub fn tick(&mut self, sample: Sample) -> Result<Output, Fault> {
        if let State::Aborted(_) = self.state {
            return Err(Fault::Latched);
        }
        if sample.now_us < self.last_now_us {
            return self.abort(Fault::TimeWentBackwards {
                now_us: sample.now_us,
                last_us: self.last_now_us,
            });
        }
        self.last_now_us = sample.now_us;

        // Redundant sensors are checked on every active tick, including while
        // waiting for separation confirmation and after ignition.
        if !matches!(self.state, State::Idle)
            && sample.sensors.spread() > self.plan.sensor_tolerance
        {
            return self.abort(Fault::SensorMismatch {
                a: sample.sensors.a,
                b: sample.sensors.b,
            });
        }

        match self.state {
            State::Idle => Ok(Output::None),
            State::Armed => self.tick_armed(sample),
            State::Separating { .. } => {
                if sample.sensors.a >= self.plan.separation_level
                    && sample.sensors.b >= self.plan.separation_level
                {
                    self.state = State::Igniting {
                        at_us: sample.now_us,
                    };
                    self.ignitions = self.ignitions.wrapping_add(1);
                    Ok(Output::Ignition)
                } else {
                    Ok(Output::Separation)
                }
            }
            State::Igniting { .. } => Ok(Output::Ignition),
            State::Aborted(_) => Err(Fault::Latched),
        }
    }

    fn tick_armed(&mut self, sample: Sample) -> Result<Output, Fault> {
        let a = sample.phase[0];
        let b = sample.phase[1];
        if a.phase != b.phase {
            return self.abort(Fault::PhaseMismatch {
                a: a.phase,
                b: b.phase,
            });
        }
        let (lo, hi) = if a.at_us <= b.at_us {
            (a.at_us, b.at_us)
        } else {
            (b.at_us, a.at_us)
        };
        if hi - lo > self.plan.phase_skew_tolerance_us {
            return self.abort(Fault::PhaseStampSkew {
                a_us: a.at_us,
                b_us: b.at_us,
            });
        }
        // Conservative epoch: never fire earlier than either sensor says.
        let entered_at_us = hi;
        if entered_at_us > sample.now_us {
            return self.abort(Fault::PhaseStampInFuture {
                at_us: entered_at_us,
                now_us: sample.now_us,
            });
        }
        let observed = a.phase as u8;
        let trigger = self.plan.trigger as u8;
        if observed < trigger {
            return Ok(Output::None);
        }
        if observed > trigger {
            return self.abort(Fault::PhaseWindowMissed {
                trigger: self.plan.trigger,
                observed: a.phase,
            });
        }
        if sample.now_us - entered_at_us < self.plan.delay_us {
            return Ok(Output::None);
        }
        self.state = State::Separating {
            at_us: sample.now_us,
        };
        self.separations = self.separations.wrapping_add(1);
        Ok(Output::Separation)
    }

    fn abort<T>(&mut self, fault: Fault) -> Result<T, Fault> {
        self.state = State::Aborted(fault);
        self.aborts = self.aborts.wrapping_add(1);
        Err(fault)
    }

    /// Explicit, one-way clearance of every latch. Counters survive so a
    /// ground reader can still see how many aborts/commands this unit issued.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.arm_primary = false;
        self.arm_secondary = false;
        self.last_now_us = 0;
    }
}
