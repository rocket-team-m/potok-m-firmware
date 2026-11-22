//! Framing and sequencing on top of the generated `potok-protocol` codec.
//!
//! Magic, CRC, version and phase validation, packing and scaling are not
//! reimplemented here. This module adds strict request/reply sequence matching
//! and a resynchronising reply assembler for the UART byte stream.

use potok_protocol as pp;

pub use pp::{flags, Phase, ReplyFrame, RequestFrame, REPLY_MAGIC, REPLY_SIZE, REQUEST_SIZE};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    TooShort,
    BufferTooSmall,
    BadMagic,
    BadCrc,
    BadPhase,
    BadVersion,
    /// Reply seq must equal the last transmitted request seq exactly.
    BadSeq {
        expected: u16,
        got: u16,
    },
}

impl From<pp::Error> for FrameError {
    fn from(err: pp::Error) -> Self {
        match err {
            pp::Error::TooShort { .. } => Self::TooShort,
            pp::Error::BufferTooSmall { .. } => Self::BufferTooSmall,
            pp::Error::BadMagic { .. } => Self::BadMagic,
            pp::Error::BadCrc { .. } => Self::BadCrc,
            pp::Error::BadPhase(_) => Self::BadPhase,
            pp::Error::VersionMismatch { .. } => Self::BadVersion,
        }
    }
}

#[inline]
pub fn encode_request(frame: &RequestFrame, out: &mut [u8]) -> Result<(), FrameError> {
    pp::encode_request(frame, out).map_err(Into::into)
}

#[inline]
pub fn encode_reply(frame: &ReplyFrame, out: &mut [u8]) -> Result<(), FrameError> {
    pp::encode_reply(frame, out).map_err(Into::into)
}

/// Validates magic, CRC-16/CCITT-FALSE and version using the protocol crate.
#[inline]
pub fn decode_reply(bytes: &[u8]) -> Result<ReplyFrame, FrameError> {
    pp::decode_reply(bytes).map_err(Into::into)
}

/// Wrap-safe strict sequencing. One outstanding request at a time: a reply is
/// accepted only if it echoes the most recent request seq.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SeqGate {
    last_tx: Option<u16>,
}

impl SeqGate {
    pub const fn new() -> Self {
        Self { last_tx: None }
    }

    /// Issue the next request seq, wrapping at `u16::MAX`.
    pub fn issue(&mut self) -> u16 {
        let seq = self.last_tx.map_or(0, |last| last.wrapping_add(1));
        self.last_tx = Some(seq);
        seq
    }

    pub const fn last(&self) -> Option<u16> {
        self.last_tx
    }

    pub fn check(&self, got: u16) -> Result<(), FrameError> {
        match self.last_tx {
            Some(expected) if expected == got => Ok(()),
            expected => Err(FrameError::BadSeq {
                expected: expected.unwrap_or(0),
                got,
            }),
        }
    }

    pub fn reset(&mut self) {
        self.last_tx = None;
    }
}

impl Default for SeqGate {
    fn default() -> Self {
        Self::new()
    }
}

/// Reassembles fixed-size reply frames from arbitrary UART chunks. Bytes
/// before the magic are noise and are discarded without counting a miss.
#[derive(Clone, Copy, Debug)]
pub struct ReplyAssembler {
    buf: [u8; REPLY_SIZE],
    len: usize,
}

impl ReplyAssembler {
    pub const fn new() -> Self {
        Self {
            buf: [0; REPLY_SIZE],
            len: 0,
        }
    }

    /// Call repeatedly for each DMA chunk. Returns at most one frame per call;
    /// keep feeding until it returns `None` to drain a chunk with two frames.
    pub fn push(&mut self, bytes: &[u8]) -> Option<[u8; REPLY_SIZE]> {
        for &byte in bytes {
            if self.len == REPLY_SIZE {
                self.len = 0;
            }
            self.buf[self.len] = byte;
            self.len += 1;
            if self.len == 1 {
                continue;
            }
            let magic = u16::from_le_bytes([self.buf[0], self.buf[1]]);
            if magic != REPLY_MAGIC {
                // The newest byte may start a new magic; everything before it
                // is noise and must not be counted as a rejected frame.
                self.buf[0] = byte;
                self.len = 1;
                continue;
            }
            if self.len == REPLY_SIZE {
                let frame = self.buf;
                self.len = 0;
                return Some(frame);
            }
        }
        None
    }

    /// Called after a rejected frame so the rest of a corrupted frame cannot
    // be mistaken for the start of the next one.
    pub fn reset(&mut self) {
        self.len = 0;
    }
}

impl Default for ReplyAssembler {
    fn default() -> Self {
        Self::new()
    }
}
