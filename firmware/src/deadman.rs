//! Deadman counter: stale guidance forces zero output and withholds the
//! watchdog refresh, so a stalled control loop ends in an IWDG reset.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deadman {
    missed: u8,
}

impl Deadman {
    pub const LIMIT: u8 = crate::MAX_CONSECUTIVE_MISSES;

    pub const fn new() -> Self {
        Self { missed: 0 }
    }

    pub fn note_valid(&mut self) {
        self.missed = 0;
    }

    pub fn note_missed(&mut self) {
        if self.missed < Self::LIMIT {
            self.missed += 1;
        }
    }

    pub const fn tripped(&self) -> bool {
        self.missed >= Self::LIMIT
    }

    pub const fn missed(&self) -> u8 {
        self.missed
    }

    /// Explicit firmware reset only: a late valid reply does not clear it.
    pub fn reset(&mut self) {
        self.missed = 0;
    }
}

impl Default for Deadman {
    fn default() -> Self {
        Self::new()
    }
}
