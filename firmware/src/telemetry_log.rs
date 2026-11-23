// SPDX-License-Identifier: GPL-3.0-or-later
//! Bounded, allocation-free flight telemetry ring logger.
//!
//! Records are serialized once into fixed-size byte frames and stored
//! verbatim in a static ring. When the ring is full the oldest frame is
//! overwritten; every frame carries a monotonic sequence number so a reader
//! of the flushed stream can detect loss. CRC and framing come from
//! `potok-protocol`; this module adds the ring, the overwrite policy and a
//! deterministic replay of the stored byte stream.
//!
//! Wiring into the crate root is owned by the parent task; this file declares
//! no `mod` entries elsewhere.

use potok_protocol::crc16;

/// Framing word at the head of every record ("LO").
pub const LOG_MAGIC: u16 = 0x4C4F;

/// `magic(2) + seq(4) + time_us(4) + tag(1) + flags(1) + payload_len(1)`.
pub const HEADER_LEN: usize = 13;
/// CRC-16/CCITT-FALSE trailing the frame.
pub const CRC_LEN: usize = 2;
/// Smallest legal serialized record.
pub const MIN_RECORD_SIZE: usize = HEADER_LEN + CRC_LEN;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The chosen frame size cannot hold even the header and CRC.
    FrameTooSmall { got: usize, need: usize },
    /// Payload does not fit the frame; no record was emitted.
    PayloadTooLong { got: usize, max: usize },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplayError {
    FrameTooSmall {
        got: usize,
        need: usize,
    },
    BadMagic {
        got: u16,
    },
    BadPayloadLen {
        got: usize,
        max: usize,
    },
    BadCrc {
        expected: u16,
        got: u16,
    },
    /// A sequence number jumped or went backwards (wrapping is expected).
    SequenceGap {
        expected: u32,
        got: u32,
        at: usize,
    },
    /// Trailing bytes shorter than one full record.
    Truncated {
        need: usize,
        got: usize,
        at: usize,
    },
}

/// Borrowed view over one decoded record. The payload slice points into the
/// replayed byte stream and is valid for the lifetime of that stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecordView<'a> {
    pub seq: u32,
    pub time_us: u32,
    pub tag: u8,
    pub flags: u8,
    pub payload: &'a [u8],
}

/// Encode one record into `out`. Padding is zero-filled, so identical inputs
/// always produce byte-identical frames.
pub fn encode_record<const SZ: usize>(
    seq: u32,
    time_us: u32,
    tag: u8,
    flags: u8,
    payload: &[u8],
    out: &mut [u8; SZ],
) -> Result<(), EncodeError> {
    if SZ < MIN_RECORD_SIZE {
        return Err(EncodeError::FrameTooSmall {
            got: SZ,
            need: MIN_RECORD_SIZE,
        });
    }
    // The length is one byte on the wire, so replay can recover the exact
    // payload instead of the zero padding.
    let max = (SZ - MIN_RECORD_SIZE).min(u8::MAX as usize);
    if payload.len() > max {
        return Err(EncodeError::PayloadTooLong {
            got: payload.len(),
            max,
        });
    }

    out[0..2].copy_from_slice(&LOG_MAGIC.to_le_bytes());
    out[2..6].copy_from_slice(&seq.to_le_bytes());
    out[6..10].copy_from_slice(&time_us.to_le_bytes());
    out[10] = tag;
    out[11] = flags;
    out[12] = payload.len() as u8;
    let payload_end = HEADER_LEN + payload.len();
    out[HEADER_LEN..payload_end].copy_from_slice(payload);
    let crc_off = SZ - CRC_LEN;
    out[payload_end..crc_off].fill(0);
    let crc = crc16(&out[..crc_off]);
    out[crc_off..].copy_from_slice(&crc.to_le_bytes());
    Ok(())
}

/// Decode and validate one frame. `bytes` must be exactly one record; a
/// shorter slice is an error rather than a partially decoded record.
pub fn decode_record(bytes: &[u8]) -> Result<RecordView<'_>, ReplayError> {
    if bytes.len() < MIN_RECORD_SIZE {
        return Err(ReplayError::FrameTooSmall {
            got: bytes.len(),
            need: MIN_RECORD_SIZE,
        });
    }
    let magic = u16::from_le_bytes([bytes[0], bytes[1]]);
    if magic != LOG_MAGIC {
        return Err(ReplayError::BadMagic { got: magic });
    }
    let crc_off = bytes.len() - CRC_LEN;
    let expected = crc16(&bytes[..crc_off]);
    let got = u16::from_le_bytes([bytes[crc_off], bytes[crc_off + 1]]);
    if got != expected {
        return Err(ReplayError::BadCrc { expected, got });
    }
    let len = usize::from(bytes[12]);
    if HEADER_LEN + len > crc_off {
        return Err(ReplayError::BadPayloadLen {
            got: len,
            max: crc_off - HEADER_LEN,
        });
    }
    Ok(RecordView {
        seq: u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
        time_us: u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
        tag: bytes[10],
        flags: bytes[11],
        payload: &bytes[HEADER_LEN..HEADER_LEN + len],
    })
}

/// Sink for flushed records. `write_records` must be atomic from the caller's
/// point of view: on `Err`, none of the passed frames are considered stored,
/// and the logger keeps them for a retry.
pub trait Storage {
    type Error;

    /// `records` are consecutive full frames in replay order.
    fn write_records<const SZ: usize>(&mut self, records: &[[u8; SZ]]) -> Result<(), Self::Error>;
}

/// A bounded write could not be accepted. Nothing was stored; the caller
/// keeps the records and may retry after draining the sink.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageError {
    Full,
}

/// Heap-free `Storage` sink over a fixed byte buffer. A write that does not
/// fit is refused and counted — never partially stored, never silently
/// dropped — so [`FlightLogger::flush`] keeps the frames for a later retry.
#[derive(Clone, Copy, Debug)]
pub struct BoundedSink<const CAP: usize> {
    bytes: [u8; CAP],
    len: usize,
    writes: u32,
    refused: u32,
}

impl<const CAP: usize> BoundedSink<CAP> {
    pub const fn new() -> Self {
        assert!(CAP > 0, "telemetry sink needs at least one byte");
        Self {
            bytes: [0; CAP],
            len: 0,
            writes: 0,
            refused: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    pub const fn writes(&self) -> u32 {
        self.writes
    }

    /// Writes refused because the sink lacked room. The records stayed in the
    /// logger's ring; this count is the loud signal that storage is full.
    pub const fn refused(&self) -> u32 {
        self.refused
    }
}

impl<const CAP: usize> Default for BoundedSink<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> Storage for BoundedSink<CAP> {
    type Error = StorageError;

    fn write_records<const SZ: usize>(&mut self, records: &[[u8; SZ]]) -> Result<(), Self::Error> {
        let needed = records.len().checked_mul(SZ).ok_or(StorageError::Full)?;
        let end = self.len.checked_add(needed).ok_or(StorageError::Full)?;
        if end > CAP {
            self.refused += 1;
            return Err(StorageError::Full);
        }
        for record in records {
            self.bytes[self.len..self.len + SZ].copy_from_slice(record);
            self.len += SZ;
        }
        self.writes += 1;
        Ok(())
    }
}

/// Static-capacity ring of serialized flight records. `N` is the number of
/// retained records, `SZ` the byte size of each one (header + payload + CRC).
#[derive(Clone, Copy, Debug)]
pub struct FlightLogger<const N: usize, const SZ: usize> {
    slots: [[u8; SZ]; N],
    head: usize,
    len: usize,
    write: usize,
    next_seq: u32,
    pushed: u64,
    overwritten: u64,
}

impl<const N: usize, const SZ: usize> FlightLogger<N, SZ> {
    pub const fn new() -> Self {
        assert!(N > 0, "telemetry ring needs at least one slot");
        assert!(
            SZ >= MIN_RECORD_SIZE,
            "telemetry record smaller than header + CRC"
        );
        Self {
            slots: [[0; SZ]; N],
            head: 0,
            len: 0,
            write: 0,
            next_seq: 0,
            pushed: 0,
            overwritten: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Total records accepted since construction, including overwritten ones.
    pub const fn total_pushed(&self) -> u64 {
        self.pushed
    }

    /// Records evicted by the overwrite policy.
    pub const fn total_overwritten(&self) -> u64 {
        self.overwritten
    }

    /// Sequence number the next accepted record will carry.
    pub const fn next_seq(&self) -> u32 {
        self.next_seq
    }

    /// Append a record and return its sequence number. An encoding failure
    /// consumes neither a sequence number nor a ring slot.
    pub fn push(
        &mut self,
        time_us: u32,
        tag: u8,
        flags: u8,
        payload: &[u8],
    ) -> Result<u32, EncodeError> {
        let seq = self.next_seq;
        let mut frame = [0u8; SZ];
        encode_record(seq, time_us, tag, flags, payload, &mut frame)?;

        self.slots[self.write] = frame;
        self.write = (self.write + 1) % N;
        if self.len < N {
            self.len += 1;
        } else {
            self.head = self.write;
            self.overwritten += 1;
        }
        self.next_seq = seq.wrapping_add(1);
        self.pushed += 1;
        Ok(seq)
    }

    /// Retained frames as raw records, oldest first. No decoding or copying.
    pub fn iter(&self) -> RingIter<'_, N, SZ> {
        RingIter {
            slots: &self.slots,
            head: self.head,
            len: self.len,
            idx: 0,
        }
    }

    /// Write up to `BATCH` retained frames to `storage` in one call and remove
    /// them from the ring only on success. Returns the number written.
    pub fn flush<const BATCH: usize, S: Storage>(
        &mut self,
        storage: &mut S,
    ) -> Result<usize, S::Error> {
        assert!(BATCH > 0, "flush batch must contain at least one record");
        let count = self.len.min(BATCH);
        if count == 0 {
            return Ok(0);
        }
        let mut batch = [[0u8; SZ]; BATCH];
        for (dst, frame) in batch.iter_mut().zip(self.iter().take(count)) {
            *dst = *frame;
        }
        storage.write_records(&batch[..count])?;
        self.head = (self.head + count) % N;
        self.len -= count;
        Ok(count)
    }
}

impl<const N: usize, const SZ: usize> Default for FlightLogger<N, SZ> {
    fn default() -> Self {
        Self::new()
    }
}

/// Oldest-first iterator over the raw frames still held by the ring.
#[derive(Clone, Copy, Debug)]
pub struct RingIter<'a, const N: usize, const SZ: usize> {
    slots: &'a [[u8; SZ]; N],
    head: usize,
    len: usize,
    idx: usize,
}

impl<'a, const N: usize, const SZ: usize> Iterator for RingIter<'a, N, SZ> {
    type Item = &'a [u8; SZ];

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.len {
            return None;
        }
        let slot = (self.head + self.idx) % N;
        self.idx += 1;
        Some(&self.slots[slot])
    }
}

/// Deterministic replay over a concatenated stream of `SZ`-byte records.
/// Stops at the first fault: once framing or sequence continuity is broken,
/// later bytes cannot be trusted to describe the same flight.
#[derive(Clone, Debug)]
pub struct Replay<'a, const SZ: usize> {
    bytes: &'a [u8],
    off: usize,
    last_seq: Option<u32>,
    ended: bool,
}

impl<'a, const SZ: usize> Replay<'a, SZ> {
    pub fn new(bytes: &'a [u8]) -> Self {
        assert!(
            SZ >= MIN_RECORD_SIZE,
            "telemetry record smaller than header + CRC"
        );
        Self {
            bytes,
            off: 0,
            last_seq: None,
            ended: false,
        }
    }
}

impl<'a, const SZ: usize> Iterator for Replay<'a, SZ> {
    type Item = Result<RecordView<'a>, ReplayError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ended {
            return None;
        }
        let remaining = self.bytes.len() - self.off;
        if remaining == 0 {
            self.ended = true;
            return None;
        }
        if remaining < SZ {
            self.ended = true;
            return Some(Err(ReplayError::Truncated {
                need: SZ,
                got: remaining,
                at: self.off,
            }));
        }
        let at = self.off;
        let frame = &self.bytes[at..at + SZ];
        self.off += SZ;
        match decode_record(frame) {
            Ok(view) => {
                if let Some(last) = self.last_seq {
                    let expected = last.wrapping_add(1);
                    if view.seq != expected {
                        self.ended = true;
                        return Some(Err(ReplayError::SequenceGap {
                            expected,
                            got: view.seq,
                            at,
                        }));
                    }
                }
                self.last_seq = Some(view.seq);
                Some(Ok(view))
            }
            Err(err) => {
                self.ended = true;
                Some(Err(err))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Typed log cadence policy
// ---------------------------------------------------------------------------
//
// The shipping loop runs at 500 Hz. An unconditional `defmt::info!` there has
// no term in the `perf` cycle model and dominates the unmodelled cost. This
// policy makes the emission rate a declared, host-testable decision: a gate
// emits on a typed state change, on a new safety reason, or at a declared slow
// cadence, and counts every observation it suppresses. Every-tick logging is
// reachable only through the explicitly named debug cadence.

/// Why an observation was allowed to reach the logging sink.
///
/// Returned by [`LogGate::observe`] so the caller can keep `defmt!` out of the
/// hot path while the decision itself stays host-testable without a logger.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Emission {
    /// A new safety-relevant reason appeared: emit immediately, never deferred
    /// to a cadence window.
    Safety,
    /// The typed state key changed: emit immediately.
    Change,
    /// The declared slow cadence elapsed: emit a periodic summary.
    Cadence,
    /// Explicit debug-every-tick mode: emit (a production gate never selects
    /// this cadence).
    Debug,
    /// Nothing eligible: the observation was counted and suppressed.
    Suppressed,
}

impl Emission {
    /// True unless this observation was suppressed.
    pub const fn should_emit(self) -> bool {
        !matches!(self, Self::Suppressed)
    }
}

/// Declared emission rate for non-safety records.
///
/// A period of one (every tick) is deliberately not constructible through
/// [`Cadence::periodic`]; the only route to it is [`Cadence::debug_every_tick`],
/// so 500 Hz logging is always a visible, typed choice in review.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cadence {
    /// Ticks between periodic emissions: 0 = state-change only, 1 = debug
    /// every tick, >= 2 = periodic summary.
    period_ticks: u32,
}

impl Cadence {
    /// Emit only when the observed state key changes (or a safety reason is
    /// observed). No periodic emission.
    pub const fn on_change() -> Self {
        Self { period_ticks: 0 }
    }

    /// Emit a periodic summary at most once every `period_ticks` observations.
    /// A period of one would be every-tick logging and is refused loudly.
    pub const fn periodic(period_ticks: u32) -> Self {
        assert!(
            period_ticks >= 2,
            "periodic cadence must be >= 2 ticks; use debug_every_tick() for 1"
        );
        Self { period_ticks }
    }

    /// Explicit debug opt-in: emit on every observation. The shipping loop
    /// never selects this unless its debug flag is turned on by a human.
    pub const fn debug_every_tick() -> Self {
        Self { period_ticks: 1 }
    }

    /// True only for the explicit every-tick debug cadence.
    pub const fn is_every_tick(self) -> bool {
        self.period_ticks == 1
    }
}

/// Stateful cadence gate over a typed, `Copy + PartialEq` state key.
///
/// `Key` must be the *qualitative* state — which reasons are active, which
/// latch is set — never a monotonic counter. A growing tally changes on every
/// tick and would turn every tick into a state change, reproducing the 500 Hz
/// log this gate exists to remove. Cumulative counts belong in the emitted
/// payload; the active-reason bits belong in the key.
#[derive(Clone, Copy, Debug)]
pub struct LogGate<Key: Copy + PartialEq> {
    cadence: Cadence,
    last: Option<Key>,
    since_emit: u32,
    suppressed: u64,
    emitted: u64,
}

impl<Key: Copy + PartialEq> LogGate<Key> {
    pub const fn new(cadence: Cadence) -> Self {
        Self {
            cadence,
            last: None,
            since_emit: 0,
            suppressed: 0,
            emitted: 0,
        }
    }

    /// Decide whether this observation may be logged. Do not reach the
    /// `defmt!` sink unless [`Emission::should_emit`] is true.
    pub fn observe(&mut self, key: Key) -> Emission {
        self.since_emit = self.since_emit.saturating_add(1);

        let emission = if self.last != Some(key) {
            Emission::Change
        } else if self.cadence.is_every_tick() {
            Emission::Debug
        } else if self.cadence.period_ticks >= 2 && self.since_emit >= self.cadence.period_ticks {
            Emission::Cadence
        } else {
            Emission::Suppressed
        };

        if emission.should_emit() {
            self.last = Some(key);
            self.since_emit = 0;
            self.emitted += 1;
        } else {
            self.suppressed += 1;
        }
        emission
    }

    /// Observe a safety-relevant reason. A *new* reason emits immediately as
    /// [`Emission::Safety`] (the state-change path, never deferred to a
    /// cadence window); repeats dedupe and count exactly as [`Self::observe`].
    pub fn observe_error(&mut self, key: Key) -> Emission {
        match self.observe(key) {
            Emission::Change => Emission::Safety,
            other => other,
        }
    }

    /// Observations not emitted, counted over the gate's lifetime.
    pub const fn suppressed(&self) -> u64 {
        self.suppressed
    }

    /// Observations the gate approved for emission over its lifetime.
    pub const fn emitted(&self) -> u64 {
        self.emitted
    }

    /// The cadence this gate was built with.
    pub const fn cadence(&self) -> Cadence {
        self.cadence
    }
}
