//! Host-only mock HAL. No hardware calls, no `unsafe`; all frame bytes are
//! scripted by the tests.

use std::collections::VecDeque;
use std::vec::Vec;

use crate::{
    board::Board,
    phase::PhaseEvent,
    protocol::{encode_reply, ReplyFrame, REPLY_SIZE},
    uart::{DmaBuffers, DoubleBufferRx, FrameTx, UartError},
    watchdog::Watchdog,
    Command,
};

/// RX that delivers scripted DMA chunks, alternating the two static slots.
#[derive(Debug)]
pub struct MockRx {
    chunks: VecDeque<Vec<u8>>,
    bufs: DmaBuffers<REPLY_SIZE>,
    pub swaps: usize,
}

impl MockRx {
    pub fn new() -> Self {
        Self {
            chunks: VecDeque::new(),
            bufs: DmaBuffers::new(),
            swaps: 0,
        }
    }

    pub fn queue(&mut self, bytes: &[u8]) {
        self.chunks.push_back(bytes.to_vec());
    }
}

impl Default for MockRx {
    fn default() -> Self {
        Self::new()
    }
}

impl DoubleBufferRx for MockRx {
    fn take(&mut self, out: &mut [u8]) -> Result<Option<usize>, UartError> {
        let Some(bytes) = self.chunks.pop_front() else {
            return Ok(None);
        };
        let slot = self.bufs.back_index();
        let len = bytes.len().min(REPLY_SIZE).min(out.len());
        self.bufs.slots[slot][..len].copy_from_slice(&bytes[..len]);
        self.bufs.mark_ready(slot, len);
        let ready = self.bufs.take_ready(slot).unwrap_or(0);
        out[..ready].copy_from_slice(&self.bufs.slots[slot][..ready]);
        self.bufs.swap();
        self.swaps += 1;
        Ok(Some(ready))
    }
}

#[derive(Debug, Default)]
pub struct MockTx {
    pub frames: Vec<Vec<u8>>,
}

impl FrameTx for MockTx {
    fn send(&mut self, frame: &[u8]) -> Result<(), UartError> {
        self.frames.push(frame.to_vec());
        Ok(())
    }
}

#[derive(Debug, Default)]
pub struct MockWatchdog {
    pub pets: u32,
}

impl Watchdog for MockWatchdog {
    fn pet(&mut self) {
        self.pets += 1;
    }
}

#[derive(Debug)]
pub struct MockBoard {
    rx: MockRx,
    tx: MockTx,
    wd: MockWatchdog,
    pub now_us: u32,
    pub sensor_flags: u8,
    pub motor_armed: bool,
    pub phase_event: Option<PhaseEvent>,
    pub cycle_complete: bool,
    pub applied: Vec<Command>,
}

impl MockBoard {
    pub fn new() -> Self {
        Self {
            rx: MockRx::new(),
            tx: MockTx::default(),
            wd: MockWatchdog::default(),
            now_us: 0,
            sensor_flags: 0,
            motor_armed: false,
            phase_event: None,
            cycle_complete: true,
            applied: Vec::new(),
        }
    }

    pub fn queue_reply(&mut self, reply: &ReplyFrame) {
        let mut buf = [0u8; REPLY_SIZE];
        encode_reply(reply, &mut buf).unwrap();
        self.rx.queue(&buf);
    }

    /// Raw byte injection for CRC/magic/seq fault tests.
    pub fn queue_bytes(&mut self, bytes: &[u8]) {
        self.rx.queue(bytes);
    }

    pub fn rx(&self) -> &MockRx {
        &self.rx
    }

    pub fn rx_mut(&mut self) -> &mut MockRx {
        &mut self.rx
    }

    pub fn sent_frames(&self) -> &[Vec<u8>] {
        &self.tx.frames
    }

    pub fn pets(&self) -> u32 {
        self.wd.pets
    }
}

impl Default for MockBoard {
    fn default() -> Self {
        Self::new()
    }
}

impl Board for MockBoard {
    type Rx = MockRx;
    type Tx = MockTx;
    type Wd = MockWatchdog;

    fn rx(&mut self) -> &mut Self::Rx {
        &mut self.rx
    }
    fn tx(&mut self) -> &mut Self::Tx {
        &mut self.tx
    }
    fn wd(&mut self) -> &mut Self::Wd {
        &mut self.wd
    }
    fn now_us(&self) -> u32 {
        self.now_us
    }
    fn sensor_flags(&self) -> u8 {
        self.sensor_flags
    }
    fn motor_armed(&self) -> bool {
        self.motor_armed
    }
    fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
        self.phase_event.take()
    }
    fn apply_command(&mut self, cmd: Command) {
        self.applied.push(cmd);
    }
}
