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
    uart::{DmaBuffers, DoubleBufferRx, UartError},
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
