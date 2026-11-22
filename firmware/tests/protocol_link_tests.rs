// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the Board-bound protocol driver and its DMA RX adapter.
//!
//! `lib.rs` is out of scope for this task, so the module under test is included
//! by path: the same source that would be wired into the crate is compiled and
//! exercised here without touching any other file.

pub use potok_m_firmware::{board, control, phase, protocol, uart, Command};

#[path = "../src/protocol_link.rs"]
mod protocol_link;

use crate::{
    phase::PhaseEvent,
    protocol::{encode_reply, flags, Phase, ReplyFrame, REPLY_SIZE, REQUEST_SIZE},
    uart::{DoubleBufferRx, UartError},
};
use potok_m_firmware::mock::{MockTx, MockWatchdog};
use potok_protocol::{decode_request, REPLY_VERSION};

use protocol_link::{DmaDoubleBufferRx, ProtocolLink, SensorSample};

/// Host board whose receive path is the real DMA adapter. No pin, UART or DMA
/// stream is named: only the `Board` trait and static buffers.
struct TestBoard {
    rx: DmaDoubleBufferRx<REPLY_SIZE>,
    tx: MockTx,
    wd: MockWatchdog,
    now_us: u32,
    flags: u8,
    armed: bool,
    event: Option<PhaseEvent>,
    applied: Vec<Command>,
}

impl TestBoard {
    fn new() -> Self {
        Self {
            rx: DmaDoubleBufferRx::new(),
            tx: MockTx::default(),
            wd: MockWatchdog::default(),
            now_us: 0,
            flags: 0,
            armed: false,
            event: None,
            applied: Vec::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        assert!(bytes.len() <= REPLY_SIZE);
        self.rx.back()[..bytes.len()].copy_from_slice(bytes);
        self.rx.submit_back(bytes.len()).unwrap();
    }

    fn queue_reply(&mut self, reply: &ReplyFrame) {
        let mut buf = [0u8; REPLY_SIZE];
        encode_reply(reply, &mut buf).unwrap();
        self.feed(&buf);
    }
}

impl board::Board for TestBoard {
    type Rx = DmaDoubleBufferRx<REPLY_SIZE>;
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
        self.flags
    }
    fn motor_armed(&self) -> bool {
        self.armed
    }
    fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
        self.event.take()
    }
    fn apply_command(&mut self, cmd: Command) {
        self.applied.push(cmd);
    }
}

fn reply(seq: u16, pitch: i16, yaw: i16) -> ReplyFrame {
    ReplyFrame {
        seq,
        g_pitch: pitch,
        g_yaw: yaw,
        confidence: 200,
        ai_healthy: 1,
        version: REPLY_VERSION,
    }
}

#[test]
fn dma_double_buffer_delivers_chunks_and_swaps_slots() {
    let mut rx = DmaDoubleBufferRx::<REPLY_SIZE>::new();
    let mut out = [0u8; REPLY_SIZE * 2];
    assert_eq!(rx.take(&mut out).unwrap(), None);
    assert_eq!(rx.pending(), None);

    rx.back()[..3].copy_from_slice(&[1, 2, 3]);
    rx.submit_back(3).unwrap();
    assert_eq!(rx.pending(), Some(3));
    assert_eq!(rx.take(&mut out).unwrap(), Some(3));
    assert_eq!(&out[..3], &[1, 2, 3]);
    assert_eq!(rx.pending(), None);

    rx.back()[..2].copy_from_slice(&[4, 5]);
    rx.submit_back(2).unwrap();
    assert_eq!(rx.take(&mut out).unwrap(), Some(2));
    assert_eq!(&out[..2], &[4, 5]);

    assert_eq!(rx.submissions, 2);
    assert_eq!(rx.takes, 2);
    assert_eq!(rx.swaps, 2);
    assert_eq!(rx.overruns, 0);
}

#[test]
fn dma_double_buffer_resubmission_is_loud_and_counted() {
    let mut rx = DmaDoubleBufferRx::<4>::new();
    let mut out = [0u8; 8];

    rx.back()[..2].copy_from_slice(&[1, 2]);
    rx.submit_back(2).unwrap();
    assert_eq!(rx.submit_back(2), Err(UartError::Overrun));
    assert_eq!(rx.overruns, 1);
    assert_eq!(
        rx.submissions, 1,
        "an overrun must not publish a new length"
    );
    assert_eq!(rx.pending(), Some(2));

    assert_eq!(rx.take(&mut out).unwrap(), Some(2));
    assert_eq!(&out[..2], &[1, 2]);
    assert_eq!(rx.take(&mut out).unwrap(), None);
}

#[test]
fn link_drives_board_through_protocol_frames() {
    let mut board = TestBoard::new();
    board.now_us = 12_345;
    board.flags = flags::IMU_OK;
    board.event = Some(PhaseEvent::Burnout);
    let mut link = ProtocolLink::new();
    assert_eq!(link.phase(), Phase::BURN);

    // First tick commits Burnout and sends on BURN-owned state with no reply.
    assert_eq!(link.tick(&mut board, true), control::StepResult::Idle);
    assert_eq!(link.phase(), Phase::COAST);
    let sent = decode_request(&board.tx.frames[0]).unwrap();
    assert_eq!(sent.seq, 0);
    assert_eq!(sent.phase, Phase::COAST);
    assert_eq!(sent.time_us, 12_345);
    assert!(flags::has(sent.flags, flags::IMU_OK));
    assert!(flags::has(sent.flags, flags::AI_READY));

    // The reply echoes the seq issued by this tick; acceptance gates the command.
    board.queue_reply(&reply(1, 25, -30));
    assert_eq!(
        link.tick(&mut board, true),
        control::StepResult::Accepted(Command {
            pitch: 25,
            yaw: -30
        })
    );
    assert_eq!(
        board.applied.last(),
        Some(&Command {
            pitch: 25,
            yaw: -30
        })
    );
    assert_eq!(link.counters().accepted, 1);
    assert_eq!(link.last_request_seq(), Some(1));
    assert_eq!(board.wd.pets, 2);
    assert_eq!(board.rx.swaps, 1, "only the tick with a DMA chunk swaps");
}

#[test]
fn request_frame_carries_board_state_and_sensor_snapshot() {
    let mut board = TestBoard::new();
    board.now_us = 7;
    board.flags = flags::BARO_OK;
    let link = ProtocolLink::new();
    let sensors = SensorSample {
        gyro: [1.0, 2.0, 3.0],
        accel: [4.0, 5.0, 6.0],
        mag: [7.0, 8.0, 9.0],
        height: 10.5,
        pitot_dp: -11,
    };

    let frame = link.request_frame(&board, 9, sensors);
    assert_eq!(frame.seq, 9);
    assert_eq!(frame.time_us, 7);
    assert_eq!(frame.phase, Phase::BURN);
    assert_eq!(frame.version, potok_protocol::REQUEST_VERSION);
    assert_eq!(frame.gyro, sensors.gyro);
    assert_eq!(frame.accel, sensors.accel);
    assert_eq!(frame.mag, sensors.mag);
    assert_eq!(frame.height, 10.5);
    assert_eq!(frame.pitot_dp, -11);
    assert!(flags::has(frame.flags, flags::BARO_OK));
    assert!(flags::has(frame.flags, flags::AI_READY));

    // Board-owned motor sense is an independent AI gate; the latch is another.
    board.armed = true;
    let armed = link.request_frame(&board, 10, sensors);
    assert!(flags::has(armed.flags, flags::MOTOR_ARMED));
    assert!(flags::has(armed.flags, flags::AI_READY));

    let mut latched = link;
    latched.link_mut().disable_now();
    let latched_frame = latched.request_frame(&board, 11, sensors);
    assert!(!flags::has(latched_frame.flags, flags::AI_READY));

    // Encoding goes through the protocol reference, not a local copy.
    let mut buf = [0u8; REQUEST_SIZE];
    link.encode_request(&board, 12, sensors, &mut buf).unwrap();
    assert_eq!(decode_request(&buf).unwrap().seq, 12);
}

#[test]
fn split_dma_chunks_assemble_one_reply() {
    let mut board = TestBoard::new();
    board.event = Some(PhaseEvent::Burnout);
    let mut link = ProtocolLink::new();
    link.tick(&mut board, true); // BURN -> COAST, seq 0, timeout

    let mut bytes = [0u8; REPLY_SIZE];
    encode_reply(&reply(2, 7, -7), &mut bytes).unwrap();
    board.feed(&bytes[..4]);
    assert_eq!(link.tick(&mut board, true), control::StepResult::Idle);
    board.feed(&bytes[4..]);
    assert_eq!(
        link.tick(&mut board, true),
        control::StepResult::Accepted(Command { pitch: 7, yaw: -7 })
    );
}

#[test]
fn bad_frames_latch_and_reset_clears_only_the_latch() {
    let mut board = TestBoard::new();
    board.event = Some(PhaseEvent::Burnout);
    let mut link = ProtocolLink::new();
    link.tick(&mut board, true);

    for _ in 0..3 {
        let seq = link.last_request_seq().unwrap_or(0).wrapping_add(1);
        let mut bytes = [0u8; REPLY_SIZE];
        encode_reply(&reply(seq, 100, 100), &mut bytes).unwrap();
        bytes[12] ^= 0x02; // CRC
        board.feed(&bytes);
        assert_eq!(
            link.tick(&mut board, true),
            control::StepResult::FrameRejected
        );
    }
    assert!(link.is_disabled());
    assert_eq!(link.counters().rejected, 4); // one timeout + three bad frames

    let seq = link.last_request_seq().unwrap().wrapping_add(1);
    board.queue_reply(&reply(seq, 50, 50));
    assert_eq!(
        link.tick(&mut board, true),
        control::StepResult::FrameRejected
    );
    assert_eq!(board.applied.last(), Some(&Command::ZERO));
    assert!(
        link.is_disabled(),
        "a valid late reply must not clear the latch"
    );

    link.reset();
    assert!(!link.is_disabled());
    assert_eq!(
        link.phase(),
        Phase::COAST,
        "reset must not rewind the phase"
    );
}
