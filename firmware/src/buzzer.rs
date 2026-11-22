// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic, integer-only PWM buzzer scheduler.
//!
//! This module is a pure scheduler: it maps a flight phase and a monotonic
//! tick count to a PWM frequency/duty pair. It owns no timer, no GPIO and no
//! pin numbers — a board adapter outside this module owns the PWM channel.
//! [`Buzzer::tick`] is the ISR entry point; it uses only integer arithmetic,
//! no allocation and no wall clock, so the same phase/tick stream always
//! produces the same output.

use potok_protocol::Phase;

/// ISR tick period in milliseconds. Call [`Buzzer::tick`] at this cadence;
/// every note duration below is a whole number of ticks.
pub const TICK_MS: u16 = 10;

/// Buzzer duty cycle for an audible note, in percent.
const DUTY: u8 = 50;

/// Equal-temperament notes (A4 = 440 Hz), rounded to whole hertz. Integers
/// only: the ISR path never computes a frequency from a floating-point value.
pub const NOTE_C4: u16 = 262;
pub const NOTE_D4: u16 = 294;
pub const NOTE_E4: u16 = 330;
pub const NOTE_F4: u16 = 349;
pub const NOTE_G4: u16 = 392;
pub const NOTE_A4: u16 = 440;
pub const NOTE_B4: u16 = 494;
pub const NOTE_C5: u16 = 523;
pub const NOTE_D5: u16 = 587;
pub const NOTE_E5: u16 = 659;
pub const NOTE_F5: u16 = 698;
pub const NOTE_G5: u16 = 784;
pub const NOTE_A5: u16 = 880;
pub const NOTE_B5: u16 = 988;
pub const NOTE_C6: u16 = 1047;

/// Silent PWM output: no tone, zero duty.
pub const SILENT: Output = Output {
    freq_hz: 0,
    duty: 0,
};

/// One PWM output value. `freq_hz == 0` or `duty == 0` means silence.
/// There is deliberately no pin field: the caller decides which PWM channel
/// receives this value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    pub freq_hz: u16,
    pub duty: u8,
}

/// One note: an integer frequency in Hz and a duration in scheduler ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Note {
    pub freq_hz: u16,
    pub ticks: u16,
}

impl Note {
    pub const fn tone(freq_hz: u16, ticks: u16) -> Self {
        Self { freq_hz, ticks }
    }

    pub const fn rest(ticks: u16) -> Self {
        Self { freq_hz: 0, ticks }
    }
}

/// Priority locator beacon: three piercing beeps then a long gap, repeating.
/// Once the parachute is deployed this owns the buzzer and preempts the tune.
const BEACON: &[Note] = &[
    Note::tone(NOTE_C6, 4),
    Note::rest(2),
    Note::tone(NOTE_C6, 4),
    Note::rest(2),
    Note::tone(NOTE_C6, 4),
    Note::rest(16),
];

/// Idle melody: an ascending/descending arpeggio cycle ending on a rest.
const MELODY: &[Note] = &[
    Note::tone(NOTE_C4, 2),
    Note::tone(NOTE_E4, 2),
    Note::tone(NOTE_G4, 2),
    Note::tone(NOTE_C5, 4),
    Note::tone(NOTE_B4, 2),
    Note::tone(NOTE_G4, 2),
    Note::tone(NOTE_E4, 2),
    Note::tone(NOTE_C4, 4),
    Note::tone(NOTE_D4, 2),
    Note::tone(NOTE_F4, 2),
    Note::tone(NOTE_A4, 2),
    Note::tone(NOTE_D5, 4),
    Note::tone(NOTE_A5, 2),
    Note::tone(NOTE_F5, 2),
    Note::tone(NOTE_D5, 2),
    Note::tone(NOTE_A4, 4),
    Note::tone(NOTE_B5, 2),
    Note::tone(NOTE_G5, 2),
    Note::tone(NOTE_E5, 2),
    Note::tone(NOTE_C6, 6),
    Note::rest(8),
];

/// Which pattern currently owns the buzzer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BuzzerMode {
    /// Motor burning (or no pattern selected): no sound at all.
    Silence,
    /// Parachute deployed: priority locator beacon.
    Beacon,
    /// Motor out, not deployed yet: idle melody.
    Melody,
}

/// Priority mapping from flight phase to buzzer pattern.
///
/// `BURN` is always silent. After deployment (`DESCENT`, `LANDED`) the
/// beacon has priority; `COAST` and `APOGEE` are idle and play the melody.
pub fn mode_for_phase(phase: Phase) -> BuzzerMode {
    match phase {
        Phase::BURN => BuzzerMode::Silence,
        Phase::COAST | Phase::APOGEE => BuzzerMode::Melody,
        Phase::DESCENT | Phase::LANDED => BuzzerMode::Beacon,
    }
}

/// Integer-only buzzer scheduler state. Copy-friendly for an ISR-owned cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Buzzer {
    mode: BuzzerMode,
    note_index: u16,
    elapsed: u16,
}

impl Buzzer {
    pub const fn new() -> Self {
        Self {
            mode: BuzzerMode::Silence,
            note_index: 0,
            elapsed: 0,
        }
    }

    /// ISR entry point. Advance one tick for `phase` and return the PWM value
    /// to apply now. No floats, no allocation, no hardware access.
    pub fn tick(&mut self, phase: Phase) -> Output {
        self.select(mode_for_phase(phase));
        self.advance()
    }

    /// Restart a pattern only when the mode actually changes, so a repeated
    /// phase report does not glitch the tune.
    fn select(&mut self, mode: BuzzerMode) {
        if self.mode != mode {
            self.mode = mode;
            self.note_index = 0;
            self.elapsed = 0;
        }
    }

    fn advance(&mut self) -> Output {
        let notes = match self.mode {
            BuzzerMode::Silence => return SILENT,
            BuzzerMode::Beacon => BEACON,
            BuzzerMode::Melody => MELODY,
        };
        let note = notes[self.note_index as usize];
        let out = if note.freq_hz == 0 {
            SILENT
        } else {
            Output {
                freq_hz: note.freq_hz,
                duty: DUTY,
            }
        };
        self.elapsed += 1;
        if self.elapsed >= note.ticks {
            self.elapsed = 0;
            self.note_index = if self.note_index + 1 == notes.len() as u16 {
                0
            } else {
                self.note_index + 1
            };
        }
        out
    }
}

impl Default for Buzzer {
    fn default() -> Self {
        Self::new()
    }
}
