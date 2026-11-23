// SPDX-License-Identifier: GPL-3.0-or-later
//! Protocol reference wired to the Embassy [`Board`] boundary.
//!
//! Wire types, offsets, magic, CRC and version come from `potok-protocol`;
//! nothing here reimplements them. Hardware enters through a `Board` and a
//! DMA-backed [`DoubleBufferRx`]. Pin numbers, UART instances and DMA streams
//! are deliberately absent: they belong to the schematic-bound binary, not to
//! link logic. No allocation, no host types, `no_std` by construction.

use potok_protocol as pp;

use crate::{
    board::Board,
    control::{step_link, Counters, LinkState, StepResult},
    phase::PhaseFsm,
    protocol::{encode_request, FrameError, RequestFrame},
    uart::{DmaBuffers, DoubleBufferRx, UartError, ZeroCopyRx},
};

/// Static DMA receive slots. The peripheral fills [`back`](Self::back) while
/// the control task validates the front slot; [`submit_back`](Self::submit_back)
/// publishes the completed length and [`take`](Self::take) swaps the slots.
#[derive(Clone, Copy, Debug)]
pub struct DmaDoubleBufferRx<const N: usize> {
    bufs: DmaBuffers<N>,
    pub submissions: u32,
    pub takes: u32,
    pub overruns: u32,
    pub swaps: usize,
}

impl<const N: usize> DmaDoubleBufferRx<N> {
    pub const fn new() -> Self {
        Self {
            bufs: DmaBuffers::new(),
            submissions: 0,
            takes: 0,
            overruns: 0,
            swaps: 0,
        }
    }

    /// Slot the DMA engine writes into. Its contents are not visible to the
    /// control task until `submit_back` publishes a length.
    pub fn back(&mut self) -> &mut [u8; N] {
        let back = self.bufs.back_index();
        &mut self.bufs.slots[back]
    }

    /// Publish `len` bytes just written into `back()`. Submitting again before
    /// the front slot is consumed is an overrun: the byte count is returned as
    /// an error and the pending frame is left untouched.
    pub fn submit_back(&mut self, len: usize) -> Result<(), UartError> {
        assert!(len <= N, "protocol_link: DMA length exceeds slot size");
        let back = self.bufs.back_index();
        if self.bufs.lens[back].is_some() {
            self.overruns += 1;
            return Err(UartError::Overrun);
        }
        self.bufs.mark_ready(back, len);
        self.submissions += 1;
        Ok(())
    }

    /// Length published in the slot the control task takes next.
    pub fn pending(&self) -> Option<usize> {
        self.bufs.lens[self.bufs.back_index()]
    }
}

impl<const N: usize> Default for DmaDoubleBufferRx<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DoubleBufferRx for DmaDoubleBufferRx<N> {
    fn take(&mut self, out: &mut [u8]) -> Result<Option<usize>, UartError> {
        let slot = self.bufs.back_index();
        let Some(len) = self.bufs.take_ready(slot) else {
            return Ok(None);
        };
        assert!(
            out.len() >= len,
            "protocol_link: sink smaller than DMA chunk"
        );
        out[..len].copy_from_slice(&self.bufs.slots[slot][..len]);
        self.bufs.swap();
        self.takes += 1;
        self.swaps += 1;
        Ok(Some(len))
    }
}

impl<const N: usize> ZeroCopyRx for DmaDoubleBufferRx<N> {
    /// Publish and swap exactly as `take` does, but return a borrow of the DMA
    /// slot instead of a second copy. All mutable bookkeeping completes before
    /// the slot is borrowed, so the returned slice is the front slot and stays
    /// valid until the next take. One data move is removed per RX chunk.
    fn take_borrowed(&mut self) -> Result<Option<&[u8]>, UartError> {
        let slot = self.bufs.back_index();
        let Some(len) = self.bufs.take_ready(slot) else {
            return Ok(None);
        };
        self.bufs.swap();
        self.takes += 1;
        self.swaps += 1;
        Ok(Some(&self.bufs.slots[slot][..len]))
    }
}

/// Sensor payload for an outbound request. The board owns the sensors; this is
/// only the wire-shaped snapshot the caller fills before sending.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorSample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub mag: [f32; 3],
    pub height: f32,
    pub pitot_dp: i16,
}

/// Reusable request frame buffer. The 42-byte sensor tail (offsets 12..54) is
/// written once in `new`; each tick patches only the 8 varying header bytes
/// (seq, time_us, phase, flags) plus the CRC. `bytes()` is the TX slice, so
/// no constant tail is rewritten per tick and there is no TX copy either.
#[derive(Clone, Copy, Debug)]
pub struct RequestTemplate {
    buf: [u8; pp::REQUEST_SIZE],
}

impl RequestTemplate {
    /// Build the template once through the protocol reference so the wire
    /// format stays owned by `potok-protocol`. A failed encode is a config
    /// error, not a fallback: the reference only fails on a too-small buffer.
    pub fn new(sensors: SensorSample) -> Self {
        let frame = RequestFrame {
            seq: 0,
            time_us: 0,
            phase: pp::Phase::BURN,
            version: pp::REQUEST_VERSION,
            flags: 0,
            reserved: 0,
            gyro: sensors.gyro,
            accel: sensors.accel,
            mag: sensors.mag,
            height: sensors.height,
            pitot_dp: sensors.pitot_dp,
        };
        let mut buf = [0u8; pp::REQUEST_SIZE];
        encode_request(&frame, &mut buf).expect("request template: reference encode failed");
        Self { buf }
    }

    /// Patch the time-varying header and recompute the CRC in place. The sensor
    /// tail is never touched again after `new`.
    #[inline]
    pub fn encode(&mut self, seq: u16, time_us: u32, phase: pp::Phase, flags: u8) {
        self.buf[pp::REQUEST_OFF_SEQ..pp::REQUEST_OFF_SEQ + 2].copy_from_slice(&seq.to_le_bytes());
        self.buf[pp::REQUEST_OFF_TIME_US..pp::REQUEST_OFF_TIME_US + 4]
            .copy_from_slice(&time_us.to_le_bytes());
        self.buf[pp::REQUEST_OFF_PHASE] = u8::from(phase);
        self.buf[pp::REQUEST_OFF_FLAGS] = flags;
        let crc = pp::crc16(&self.buf[..pp::REQUEST_CRC_OFF]);
        self.buf[pp::REQUEST_CRC_OFF..].copy_from_slice(&crc.to_le_bytes());
    }

    /// The TX slice. Borrowed, not copied.
    #[inline]
    pub const fn bytes(&self) -> &[u8; pp::REQUEST_SIZE] {
        &self.buf
    }
}

/// Reply assembler with a byte-wise resync scan and a locked-length fast path.
///
/// While unsynchronised every byte follows the same rules as
/// [`crate::protocol::ReplyAssembler`]: noise before the magic is discarded and
/// the newest byte may start a new magic. Once the first two bytes match
/// `REPLY_MAGIC` those bytes can never change, so the remaining frame is copied
/// with one slice copy instead of re-reading the magic on every byte.
#[derive(Clone, Copy, Debug)]
pub struct LockedReplyAssembler {
    buf: [u8; pp::REPLY_SIZE],
    len: usize,
    locked: bool,
}

impl LockedReplyAssembler {
    pub const fn new() -> Self {
        Self {
            buf: [0; pp::REPLY_SIZE],
            len: 0,
            locked: false,
        }
    }

    /// Consume one DMA chunk. Returns at most one frame; keep feeding to drain
    /// a chunk holding two frames.
    pub fn push(&mut self, bytes: &[u8]) -> Option<[u8; pp::REPLY_SIZE]> {
        let mut i = 0;
        while i < bytes.len() {
            if !self.locked {
                // Byte-wise resync: preserve the reference assembler's rules.
                if self.len == pp::REPLY_SIZE {
                    self.len = 0;
                }
                let byte = bytes[i];
                i += 1;
                self.buf[self.len] = byte;
                self.len += 1;
                if self.len == 1 {
                    continue;
                }
                let magic = u16::from_le_bytes([self.buf[0], self.buf[1]]);
                if magic != pp::REPLY_MAGIC {
                    self.buf[0] = byte;
                    self.len = 1;
                    continue;
                }
                self.locked = true;
            } else {
                let count = (pp::REPLY_SIZE - self.len).min(bytes.len() - i);
                self.buf[self.len..self.len + count].copy_from_slice(&bytes[i..i + count]);
                self.len += count;
                i += count;
                if self.len == pp::REPLY_SIZE {
                    let frame = self.buf;
                    self.len = 0;
                    self.locked = false;
                    return Some(frame);
                }
            }
        }
        None
    }

    /// Discard a partial frame and return to byte-wise resync.
    pub const fn reset(&mut self) {
        self.len = 0;
        self.locked = false;
    }
}

impl Default for LockedReplyAssembler {
    fn default() -> Self {
        Self::new()
    }
}

/// Board-bound link endpoint. Owns only protocol state: the `Board` owns the
/// peripheral, the clock and the actuator. Every safety decision stays in
/// [`LinkState`] and [`PhaseFsm`], reached through [`step_link`]. The shipping
/// loop adds the independent gates in [`crate::control::step`].
#[derive(Clone, Copy, Debug)]
pub struct ProtocolLink {
    fsm: PhaseFsm,
    link: LinkState,
}

impl ProtocolLink {
    pub const fn new() -> Self {
        Self {
            fsm: PhaseFsm::new(),
            link: LinkState::new(),
        }
    }

    /// One 500 Hz tick: send a request, consume at most one reply frame, gate
    /// the command and decide the watchdog feed.
    pub fn tick<B: Board>(&mut self, board: &mut B, cycle_complete: bool) -> StepResult {
        step_link(board, &mut self.fsm, &mut self.link, cycle_complete)
    }

    pub const fn phase(&self) -> pp::Phase {
        self.fsm.phase()
    }

    pub const fn counters(&self) -> Counters {
        self.link.counters
    }

    pub const fn is_disabled(&self) -> bool {
        self.link.is_disabled()
    }

    pub const fn last_request_seq(&self) -> Option<u16> {
        self.link.last_request_seq()
    }

    pub fn link_mut(&mut self) -> &mut LinkState {
        &mut self.link
    }

    /// Clear the AI latch only. The phase machine stays forward-only: resetting
    /// the link must never move the flight back into BURN.
    pub fn reset(&mut self) {
        self.link.reset();
    }

    /// Build a request from board-owned state plus a sensor snapshot, using the
    /// protocol reference for version and flags. `MOTOR_ARMED` is a second,
    /// independent gate on AI output; `AI_READY` is cleared once latched off.
    pub fn request_frame<B: Board>(
        &self,
        board: &B,
        seq: u16,
        sensors: SensorSample,
    ) -> RequestFrame {
        let mut flags = board.sensor_flags();
        if board.motor_armed() {
            pp::flags::set(&mut flags, pp::flags::MOTOR_ARMED);
        }
        if !self.link.is_disabled() {
            pp::flags::set(&mut flags, pp::flags::AI_READY);
        }
        RequestFrame {
            seq,
            time_us: board.now_us(),
            phase: self.fsm.phase(),
            version: pp::REQUEST_VERSION,
            flags,
            reserved: 0,
            gyro: sensors.gyro,
            accel: sensors.accel,
            mag: sensors.mag,
            height: sensors.height,
            pitot_dp: sensors.pitot_dp,
        }
    }

    /// Encode that request with the protocol reference into a caller-owned
    /// buffer. No copy of the wire format lives here.
    pub fn encode_request<B: Board>(
        &self,
        board: &B,
        seq: u16,
        sensors: SensorSample,
        out: &mut [u8],
    ) -> Result<(), FrameError> {
        encode_request(&self.request_frame(board, seq, sensors), out)
    }
}

impl Default for ProtocolLink {
    fn default() -> Self {
        Self::new()
    }
}
