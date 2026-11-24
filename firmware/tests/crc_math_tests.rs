// SPDX-License-Identifier: GPL-3.0-or-later
//! Independent gates for the CRC-16/CCITT-FALSE seam (CRC-1 in
//! `docs/MATH_PROVENANCE.md`).
//!
//! The protocol crate's `crc16` is external and table-based. These tests never
//! use it to produce an expected value: `ccitt_false_ref` is a bit-at-a-time
//! transcription of the Rocksoft/Williams model — poly 0x1021, init 0xFFFF,
//! no reflection in or out, no final XOR — and the expected values are
//! published vectors. `crc16_ibm_ref` is the CRC-16/IBM (ARC) foil: reflected
//! poly 0x8005, init 0x0000, no final XOR.
//!
//! Vectors opened 2026-11-24 in the RevEng CRC catalogue
//! <https://reveng.sourceforge.io/crc-catalogue/16.htm>:
//! - `CRC-16/IBM-3740`, alias `CRC-16/CCITT-FALSE`: `poly=0x1021
//!   init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1`, plus
//!   seven codewords attributed there to AUTOSAR Classic Platform R22-11,
//!   "Specification of CRC Routines", §7.2.2.1 p.26 (the AUTOSAR PDF itself
//!   was not opened; the codewords are as transcribed by RevEng).
//! - `CRC-16/ARC`, alias `CRC-IBM`: `check=0xbb3d` for the same message.

use potok_m_firmware::protocol::{
    decode_reply, encode_reply, encode_request, FrameError, Phase, ReplyFrame, RequestFrame,
};
use potok_m_firmware::telemetry_log::{decode_record, encode_record, ReplayError, CRC_LEN};
use potok_protocol::{crc16, decode_request, Error as PpError, REPLY_CRC_OFF, REQUEST_CRC_OFF};

/// Bit-at-a-time reference, independent of the external table implementation.
fn ccitt_false_ref(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Foil algorithm: CRC-16/IBM (RevEng's CRC-16/ARC). Reflected 0x8005 is
/// 0xA001; init 0x0000, no reflected output, no final XOR.
fn crc16_ibm_ref(data: &[u8]) -> u16 {
    let mut crc = 0x0000u16;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// Check a frame's trailing CRC against the independent reference and require
/// the IBM foil to differ, so a matching wire field cannot be a coincidence.
fn discriminating_crc(covered: &[u8], wire_crc: &[u8]) -> (u16, u16) {
    let ccitt = ccitt_false_ref(covered);
    let ibm = crc16_ibm_ref(covered);
    assert_ne!(ccitt, ibm, "the foil must differ for this frame");
    assert_eq!(wire_crc, ccitt.to_le_bytes(), "wire CRC is not CCITT-FALSE");
    (ccitt, ibm)
}

#[test]
fn published_vectors_pin_poly_init_and_no_final_xor() {
    // The universal check value and the empty message. Empty reads back the
    // init word only because xorout is zero.
    assert_eq!(ccitt_false_ref(b""), 0xFFFF);
    assert_eq!(ccitt_false_ref(b"123456789"), 0x29B1);
    assert_eq!(crc16(b""), 0xFFFF);
    assert_eq!(crc16(b"123456789"), 0x29B1);

    // AUTOSAR R22-11 §7.2.2.1 codewords as transcribed by RevEng: each pair is
    // (message bytes, CRC bytes in transmission order, high byte first).
    let autosar: [(&[u8], u16); 5] = [
        (&[0x0F, 0xAA, 0x00, 0x55], 0x2023),
        (&[0x00, 0xFF, 0x55, 0x11], 0xB8F9),
        (
            &[0x33, 0x22, 0x55, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
            0xF53F,
        ),
        (&[0x92, 0x6B, 0x55], 0x0745),
        (&[0xFF, 0xFF, 0xFF, 0xFF], 0x1D0F),
    ];
    for (msg, expected) in autosar {
        assert_eq!(ccitt_false_ref(msg), expected, "reference on {msg:02X?}");
        assert_eq!(crc16(msg), expected, "protocol crc16 on {msg:02X?}");
    }

    // Guard the foil: CRC-16/IBM reads 0xBB3D for the canonical message.
    assert_eq!(crc16_ibm_ref(b"123456789"), 0xBB3D);
    assert_ne!(ccitt_false_ref(b"123456789"), crc16_ibm_ref(b"123456789"));
}

#[test]
fn reply_frame_uses_ccitt_false_and_decoder_rejects_ibm_crc() {
    let frame = ReplyFrame {
        seq: 0x1234,
        g_pitch: -1500,
        g_yaw: 2048,
        confidence: 200,
        ai_healthy: 0xA5,
        version: potok_protocol::REPLY_VERSION,
    };
    let mut bytes = [0u8; potok_protocol::REPLY_SIZE];
    encode_reply(&frame, &mut bytes).expect("reply encode");
    let (_, ibm) = discriminating_crc(&bytes[..REPLY_CRC_OFF], &bytes[REPLY_CRC_OFF..]);
    assert_eq!(decode_reply(&bytes), Ok(frame));

    bytes[REPLY_CRC_OFF..].copy_from_slice(&ibm.to_le_bytes());
    // FrameError erases the protocol's expected/got fields; the wire field
    // itself is checked against the independent reference above.
    assert_eq!(
        decode_reply(&bytes),
        Err(FrameError::BadCrc),
        "the decoder must refuse a CRC-16/IBM field"
    );
}

#[test]
fn request_frame_uses_ccitt_false_and_decoder_rejects_ibm_crc() {
    let frame = RequestFrame {
        seq: 0x00FF,
        time_us: 0x0102_0304,
        phase: Phase::DESCENT,
        version: potok_protocol::REQUEST_VERSION,
        flags: 0x5A,
        reserved: 0x11,
        gyro: [1.0, -2.0, 3.0],
        accel: [0.5, 9.81, -0.25],
        mag: [-1.0, 2.0, 3.0],
        height: 123.5,
        pitot_dp: -777,
    };
    let mut bytes = [0u8; potok_protocol::REQUEST_SIZE];
    encode_request(&frame, &mut bytes).expect("request encode");
    let (ccitt, ibm) = discriminating_crc(&bytes[..REQUEST_CRC_OFF], &bytes[REQUEST_CRC_OFF..]);
    assert_eq!(decode_request(&bytes), Ok(frame));

    bytes[REQUEST_CRC_OFF..].copy_from_slice(&ibm.to_le_bytes());
    assert!(matches!(
        decode_request(&bytes),
        Err(PpError::BadCrc { expected, got }) if expected == ccitt && got == ibm
    ));
}

#[test]
fn telemetry_record_uses_ccitt_false_and_decoder_rejects_ibm_crc() {
    const SZ: usize = 32;
    let payload = [0xDE, 0xAD, 0xBE, 0xEF];
    let mut bytes = [0u8; SZ];
    encode_record::<SZ>(0x0102_0304, 0xDEAD_BEEF, 7, 0xA5, &payload, &mut bytes)
        .expect("telemetry encode");

    let crc_off = SZ - CRC_LEN;
    let (ccitt, ibm) = discriminating_crc(&bytes[..crc_off], &bytes[crc_off..]);
    let view = decode_record(&bytes).expect("telemetry decode");
    assert_eq!(view.seq, 0x0102_0304);
    assert_eq!(view.time_us, 0xDEAD_BEEF);
    assert_eq!(view.payload, payload);

    bytes[crc_off..].copy_from_slice(&ibm.to_le_bytes());
    assert_eq!(
        decode_record(&bytes),
        Err(ReplayError::BadCrc {
            expected: ccitt,
            got: ibm
        }),
        "the telemetry decoder must refuse a CRC-16/IBM field"
    );
}
