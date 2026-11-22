// SPDX-License-Identifier: GPL-3.0-or-later
//! Post-landing / search signal scheduler.
//!
//! Pure scheduler that arbitrates the locator signals after landing: a
//! 144 MHz radio duty-cycle hint and a priority buzzer pattern. It owns no
//! timer, no board-specific identifiers and no radio driver — a board adapter
//! applies [`SignalFrame`] to the hardware.
//!
//! Time enters only through [`SearchBeacon::tick`] with [`SignalInputs`].
//! Every decision is a function of a wrapping tick counter and those inputs:
//! no wall clock, no randomness, no floating point, no allocation and no host
//! types. The same input stream always produces the same frame, on the host
//! and on the MCU.
//!
//! Priority: boost or a critically low battery silences everything; otherwise
//! the search beacon preempts the idle melody. Battery voltage sets the radio
//! duty cap and the buzzer drive, never the pattern timing, so the cadence a
//! rescuer listens for stays the same as the pack drains.

/// Scheduler tick period in milliseconds. Note durations are whole ticks.
pub const TICK_MS: u16 = 100;

/// Locator radio band in MHz. A hint only: no driver, channel or wire here.
pub const RADIO_BAND_MHZ: u16 = 144;
/// Radio slot grid in scheduler ticks. TX slots per grid = duty% * grid / 100.
pub const RADIO_PERIOD_TICKS: u16 = 20;

/// Pack voltage below which no signal is allowed (mV).
pub const VBAT_CRITICAL_MV: u16 = 3_300;
/// Pack voltage below which signals run at the reduced limit (mV).
pub const VBAT_LOW_MV: u16 = 3_600;
/// Pack voltage at and above which signals run at full limit (mV).
pub const VBAT_NOMINAL_MV: u16 = 3_900;

pub const DUTY_OFF_RADIO: u8 = 0;
pub const DUTY_LOW_RADIO: u8 = 5;
pub const DUTY_NOMINAL_RADIO: u8 = 10;
pub const DUTY_FULL_RADIO: u8 = 20;
pub const DUTY_LOW_BUZZER: u8 = 25;
pub const DUTY_FULL_BUZZER: u8 = 50;

/// Search pattern: three short calls, a higher closing call, then a long gap.
/// One cycle is 24 ticks (2.4 s at [`TICK_MS`]).
pub const SEARCH_CALL_HZ: u16 = 880;
pub const SEARCH_PEAK_HZ: u16 = 1047;

/// One element of a buzzer pattern: a frequency in Hz (0 = rest) and a
/// duration in scheduler ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ToneStep {
    pub freq_hz: u16,
    pub ticks: u8,
}

impl ToneStep {
    pub const fn tone(freq_hz: u16, ticks: u8) -> Self {
        Self { freq_hz, ticks }
    }

    pub const fn rest(ticks: u8) -> Self {
        Self { freq_hz: 0, ticks }
    }
}

pub const SEARCH_PATTERN: &[ToneStep] = &[
    ToneStep::tone(SEARCH_CALL_HZ, 2),
    ToneStep::rest(1),
    ToneStep::tone(SEARCH_CALL_HZ, 2),
    ToneStep::rest(1),
    ToneStep::tone(SEARCH_CALL_HZ, 2),
    ToneStep::rest(4),
    ToneStep::tone(SEARCH_PEAK_HZ, 4),
    ToneStep::rest(8),
];

/// One buzzer output. `freq_hz == 0` or `duty == 0` means silence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tone {
    pub freq_hz: u16,
    pub duty: u8,
}

pub const SILENT: Tone = Tone {
    freq_hz: 0,
    duty: 0,
};

impl Tone {
    pub const fn is_silent(self) -> bool {
        self.freq_hz == 0 || self.duty == 0
    }
}

/// Radio state the board adapter may apply. Nothing here selects a radio
/// channel, configures a transmitter or touches hardware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioHint {
    pub band_mhz: u16,
    /// Battery-clamped duty cap in percent.
    pub duty_percent: u8,
    /// True only inside a transmit slot while the search beacon owns the air.
    pub transmit: bool,
}

/// Which pattern owns the buzzer this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BuzzerPattern {
    /// Boost or critical battery: nothing may sound.
    Silence,
    /// Search not armed: the beacon releases the buzzer to the idle melody.
    /// The frame's tone is [`SILENT`] in this state.
    Melody,
    /// Post-landing search beacon: preempts the melody.
    Search,
}

/// Available signal power, derived from pack voltage (and overridden by boost).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerBudget {
    Off,
    Low,
    Nominal,
    Full,
}

impl PowerBudget {
    pub const fn radio_duty_percent(self) -> u8 {
        match self {
            Self::Off => DUTY_OFF_RADIO,
            Self::Low => DUTY_LOW_RADIO,
            Self::Nominal => DUTY_NOMINAL_RADIO,
            Self::Full => DUTY_FULL_RADIO,
        }
    }

    pub const fn buzzer_duty_percent(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::Low => DUTY_LOW_BUZZER,
            Self::Nominal | Self::Full => DUTY_FULL_BUZZER,
        }
    }

    pub const fn allows_signals(self) -> bool {
        match self {
            Self::Off => false,
            Self::Low | Self::Nominal | Self::Full => true,
        }
    }
}

/// Battery band for a voltage in millivolts. Boundaries are inclusive at the
/// low end: 3300 mV is the first usable band, 3900 mV the first full band.
pub const fn budget_for(battery_mv: u16) -> PowerBudget {
    if battery_mv < VBAT_CRITICAL_MV {
        PowerBudget::Off
    } else if battery_mv < VBAT_LOW_MV {
        PowerBudget::Low
    } else if battery_mv < VBAT_NOMINAL_MV {
        PowerBudget::Nominal
    } else {
        PowerBudget::Full
    }
}

/// Inputs sampled by one scheduler tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignalInputs {
    /// Landing detected: the search beacon should own the signals.
    pub post_landing: bool,
    /// The idle melody would like the buzzer (ignored during search).
    pub melody_requested: bool,
    /// High-current boost in progress: everything goes quiet.
    pub boost_active: bool,
    /// Pack voltage in millivolts.
    pub battery_mv: u16,
}

/// Boost overrides the battery band: it is a hard silence, not a duty cut.
pub const fn effective_budget(inputs: SignalInputs) -> PowerBudget {
    if inputs.boost_active {
        PowerBudget::Off
    } else {
        budget_for(inputs.battery_mv)
    }
}

/// Pattern arbitration. Search outranks melody; boost and a dead pack outrank
/// both. A lone `melody_requested` hands the buzzer back without forcing a
/// tone from this module.
pub const fn pattern_for(inputs: SignalInputs) -> BuzzerPattern {
    if !effective_budget(inputs).allows_signals() {
        BuzzerPattern::Silence
    } else if inputs.post_landing {
        BuzzerPattern::Search
    } else if inputs.melody_requested {
        BuzzerPattern::Melody
    } else {
        BuzzerPattern::Silence
    }
}

/// Complete output of one scheduler tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignalFrame {
    pub tick: u32,
    pub budget: PowerBudget,
    pub pattern: BuzzerPattern,
    pub radio: RadioHint,
    pub buzzer: Tone,
}

impl SignalFrame {
    /// True when the search beacon is the active owner of both signals.
    pub const fn search_active(self) -> bool {
        match self.pattern {
            BuzzerPattern::Search => true,
            BuzzerPattern::Silence | BuzzerPattern::Melody => false,
        }
    }
}

/// Fixed-layout search signal scheduler. `Copy` so an ISR-owned cell can
/// replace it without allocation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SearchBeacon {
    tick: u32,
    pattern: BuzzerPattern,
    step: u8,
    elapsed: u8,
}

impl SearchBeacon {
    pub const fn new() -> Self {
        Self::at_tick(0)
    }

    /// Start on a chosen point of the wrapping tick timeline. Mainly for
    /// deterministic tests of the rollover; firmware uses [`Self::new`].
    pub const fn at_tick(tick: u32) -> Self {
        Self {
            tick,
            pattern: BuzzerPattern::Silence,
            step: 0,
            elapsed: 0,
        }
    }

    pub const fn tick_count(&self) -> u32 {
        self.tick
    }

    pub const fn pattern(&self) -> BuzzerPattern {
        self.pattern
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Advance one tick and return the signals to apply now.
    pub fn tick(&mut self, inputs: SignalInputs) -> SignalFrame {
        self.tick = self.tick.wrapping_add(1);

        let budget = effective_budget(inputs);
        let pattern = pattern_for(inputs);
        if pattern != self.pattern {
            self.pattern = pattern;
            self.step = 0;
            self.elapsed = 0;
        }

        let buzzer = self.buzzer_now(budget, pattern);
        self.advance(pattern);

        let duty = budget.radio_duty_percent();
        let slots = u32::from(duty) * u32::from(RADIO_PERIOD_TICKS) / 100;
        let in_slot = self.tick % u32::from(RADIO_PERIOD_TICKS) < slots;

        SignalFrame {
            tick: self.tick,
            budget,
            pattern,
            radio: RadioHint {
                band_mhz: RADIO_BAND_MHZ,
                duty_percent: duty,
                transmit: pattern == BuzzerPattern::Search && in_slot,
            },
            buzzer,
        }
    }

    fn buzzer_now(&self, budget: PowerBudget, pattern: BuzzerPattern) -> Tone {
        if pattern != BuzzerPattern::Search {
            return SILENT;
        }
        let step = SEARCH_PATTERN[self.step as usize];
        if step.freq_hz == 0 {
            SILENT
        } else {
            Tone {
                freq_hz: step.freq_hz,
                duty: budget.buzzer_duty_percent(),
            }
        }
    }

    fn advance(&mut self, pattern: BuzzerPattern) {
        if pattern != BuzzerPattern::Search {
            return;
        }
        self.elapsed += 1;
        if self.elapsed >= SEARCH_PATTERN[self.step as usize].ticks {
            self.elapsed = 0;
            let next = self.step + 1;
            self.step = if next as usize == SEARCH_PATTERN.len() {
                0
            } else {
                next
            };
        }
    }
}

impl Default for SearchBeacon {
    fn default() -> Self {
        Self::new()
    }
}
