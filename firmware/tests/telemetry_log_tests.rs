// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the bounded telemetry ring: framing, overwrite, batch
//! flush to a storage trait, and deterministic replay of the stored stream.

#[path = "../src/telemetry_log.rs"]
mod telemetry_log;

use telemetry_log::{
    decode_record, encode_record, EncodeError, FlightLogger, Replay, ReplayError, HEADER_LEN,
    LOG_MAGIC, MIN_RECORD_SIZE,
};

const SZ: usize = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct StorageFail;

#[derive(Default)]
struct RecordingStorage {
    bytes: Vec<u8>,
    calls: usize,
    batch_lens: Vec<usize>,
    fail_on_call: Option<usize>,
}

impl telemetry_log::Storage for RecordingStorage {
    type Error = StorageFail;

    fn write_records<const N: usize>(&mut self, records: &[[u8; N]]) -> Result<(), Self::Error> {
        self.calls += 1;
        if self.fail_on_call == Some(self.calls) {
            return Err(StorageFail);
        }
        self.batch_lens.push(records.len());
        for record in records {
            self.bytes.extend_from_slice(record);
        }
        Ok(())
    }
}

fn frame(seq: u32, time_us: u32, tag: u8, payload: &[u8]) -> [u8; SZ] {
    let mut out = [0u8; SZ];
    encode_record::<SZ>(seq, time_us, tag, 0, payload, &mut out).unwrap();
    out
}

fn seqs(storage: &RecordingStorage) -> Vec<u32> {
    Replay::<SZ>::new(&storage.bytes)
        .map(|record| record.unwrap().seq)
        .collect()
}

#[test]
fn record_roundtrip_crc_and_magic_are_validated() {
    let record = frame(7, 1_234, 3, b"abc");
    let view = decode_record(&record).unwrap();
    assert_eq!(view.seq, 7);
    assert_eq!(view.time_us, 1_234);
    assert_eq!(view.tag, 3);
    assert_eq!(view.flags, 0);
    assert_eq!(view.payload, b"abc");
    assert_eq!(u16::from_le_bytes([record[0], record[1]]), LOG_MAGIC);

    let mut corrupt = record;
    corrupt[HEADER_LEN] ^= 0x01;
    assert!(matches!(
        decode_record(&corrupt),
        Err(ReplayError::BadCrc { .. })
    ));

    corrupt = record;
    corrupt[0] = 0;
    assert!(matches!(
        decode_record(&corrupt),
        Err(ReplayError::BadMagic { got: 0x4C00 })
    ));

    assert_eq!(
        decode_record(&[0u8; 4]),
        Err(ReplayError::FrameTooSmall {
            got: 4,
            need: MIN_RECORD_SIZE
        })
    );
}

#[test]
fn identical_inputs_produce_byte_identical_frames() {
    let payload = [1u8, 2, 3];
    let mut a = [0u8; SZ];
    let mut b = [0u8; SZ];
    encode_record::<SZ>(9, 55, 4, 0xF0, &payload, &mut a).unwrap();
    b.copy_from_slice(&[0xAAu8; SZ]);
    encode_record::<SZ>(9, 55, 4, 0xF0, &payload, &mut b).unwrap();
    assert_eq!(a, b);

    // Padding is zero-filled rather than left as stale ring contents.
    let crc_off = SZ - 2;
    assert!(a[HEADER_LEN + payload.len()..crc_off]
        .iter()
        .all(|&byte| byte == 0));
}

#[test]
fn encode_failures_consume_neither_sequence_number_nor_slot() {
    // SZ 16 -> one payload byte.
    let mut log = FlightLogger::<2, 16>::new();
    assert_eq!(
        log.push(0, 0, 0, &[1, 2]),
        Err(EncodeError::PayloadTooLong { got: 2, max: 1 })
    );
    assert_eq!(log.len(), 0);
    assert_eq!(log.next_seq(), 0);
    assert_eq!(log.push(0, 0, 0, &[1]).unwrap(), 0);
    assert_eq!(log.next_seq(), 1);

    assert_eq!(
        encode_record::<13>(0, 0, 0, 0, &[], &mut [0u8; 13]),
        Err(EncodeError::FrameTooSmall {
            got: 13,
            need: MIN_RECORD_SIZE
        })
    );
}

#[test]
fn ring_overwrite_drops_oldest_and_counts_evictions() {
    let mut log = FlightLogger::<4, SZ>::new();
    assert_eq!(log.capacity(), 4);
    assert!(log.is_empty());
    for i in 0..7u32 {
        assert_eq!(log.push(i * 10, 1, 0, &[i as u8]).unwrap(), i);
    }
    assert_eq!(log.len(), 4);
    assert_eq!(log.total_pushed(), 7);
    assert_eq!(log.total_overwritten(), 3);
    assert_eq!(log.next_seq(), 7);

    let retained: Vec<u32> = log
        .iter()
        .map(|record| decode_record(record).unwrap().seq)
        .collect();
    assert_eq!(retained, vec![3, 4, 5, 6]);

    let mut one = FlightLogger::<1, SZ>::new();
    for i in 0..3u32 {
        one.push(i, 0, 0, &[]).unwrap();
    }
    assert_eq!(one.len(), 1);
    assert_eq!(one.total_overwritten(), 2);
    assert_eq!(decode_record(one.iter().next().unwrap()).unwrap().seq, 2);
}

#[test]
fn batch_flush_writes_bounded_groups_and_replays_in_order() {
    // 12 accepted records into an 8-slot ring: the four oldest were evicted,
    // so the flushed stream must still be gap-free.
    let mut log = FlightLogger::<8, SZ>::new();
    for i in 0..12u32 {
        log.push(i, 0, 0, &[i as u8]).unwrap();
    }
    let mut storage = RecordingStorage::default();
    assert_eq!(log.flush::<3, _>(&mut storage).unwrap(), 3);
    assert_eq!(log.flush::<3, _>(&mut storage).unwrap(), 3);
    assert_eq!(log.flush::<3, _>(&mut storage).unwrap(), 2);
    assert_eq!(log.flush::<3, _>(&mut storage).unwrap(), 0);
    assert_eq!(storage.batch_lens, vec![3, 3, 2]);
    assert_eq!(storage.bytes.len(), 8 * SZ);
    assert!(log.is_empty());
    assert_eq!(seqs(&storage), (4u32..12).collect::<Vec<_>>());

    let views: Vec<_> = Replay::<SZ>::new(&storage.bytes)
        .map(|record| record.unwrap())
        .collect();
    assert_eq!(views[0].payload, &[4]);
    assert_eq!(views[7].payload, &[11]);
}

#[test]
fn failed_flush_keeps_records_for_retry() {
    let mut log = FlightLogger::<4, SZ>::new();
    for i in 0..4u32 {
        log.push(i, 0, 0, &[i as u8]).unwrap();
    }
    let mut storage = RecordingStorage {
        fail_on_call: Some(1),
        ..RecordingStorage::default()
    };
    assert_eq!(log.flush::<4, _>(&mut storage), Err(StorageFail));
    assert_eq!(log.len(), 4);
    assert!(storage.bytes.is_empty());

    storage.fail_on_call = None;
    assert_eq!(log.flush::<4, _>(&mut storage).unwrap(), 4);
    assert_eq!(log.len(), 0);
    assert_eq!(seqs(&storage), (0u32..4).collect::<Vec<_>>());
}

#[test]
fn replay_detects_gaps_truncation_and_bad_frames() {
    let mut bytes = Vec::new();
    for seq in [0u32, 1, 3, 4] {
        bytes.extend_from_slice(&frame(seq, 0, 0, &[]));
    }
    let results: Vec<_> = Replay::<SZ>::new(&bytes).collect();
    assert_eq!(results.len(), 3); // two good records, then the gap
    assert!(results[0].is_ok() && results[1].is_ok());
    assert!(matches!(
        results[2],
        Err(ReplayError::SequenceGap {
            expected: 2,
            got: 3,
            at: 40
        })
    ));

    let mut truncated = Vec::new();
    for seq in 0..4u32 {
        truncated.extend_from_slice(&frame(seq, 0, 0, &[]));
    }
    truncated.push(0);
    assert!(matches!(
        Replay::<SZ>::new(&truncated).last(),
        Some(Err(ReplayError::Truncated {
            need: 20,
            got: 1,
            at: 80
        }))
    ));

    let mut bad_magic = Vec::new();
    bad_magic.extend_from_slice(&frame(0, 0, 0, &[]));
    let mut second = frame(1, 0, 0, &[]);
    second[0] = 0;
    bad_magic.extend_from_slice(&second);
    let results: Vec<_> = Replay::<SZ>::new(&bad_magic).collect();
    assert_eq!(results.len(), 2);
    assert!(matches!(results[1], Err(ReplayError::BadMagic { .. })));
}

#[test]
fn replay_accepts_u32_sequence_wrap() {
    let mut bytes = Vec::new();
    for seq in [u32::MAX - 1, u32::MAX, 0, 1] {
        bytes.extend_from_slice(&frame(seq, 0, 0, &[]));
    }
    let seqs: Vec<u32> = Replay::<SZ>::new(&bytes)
        .map(|record| record.unwrap().seq)
        .collect();
    assert_eq!(seqs, vec![u32::MAX - 1, u32::MAX, 0, 1]);
}
