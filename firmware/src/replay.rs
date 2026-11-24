// SPDX-License-Identifier: GPL-3.0-or-later
//! Deterministic software-in-the-loop replay of `potok-sim` side artifacts.
//!
//! The simulator writes two binary artifacts next to each other:
//! `validation.bin` (50 Hz sensor features and reference targets) and an
//! optional `validation.trajectory` sidecar (per-tick flown truth). This module
//! reads those byte layouts without depending on the simulator, maps one 50 Hz
//! source frame onto ten 500 Hz firmware ticks, and emits everything the host
//! `MockBoard` needs: a reply frame, sensor values, phase, flags and time.
//!
//! Nothing here allocates, reads the clock or touches hardware. If no artifact
//! is available, [`Replay::synthetic`] produces an equally deterministic stream
//! from [`synthetic_tick`], and [`synthetic_validation_bytes`] writes a small
//! real-shaped fixture into a caller-owned buffer for parser tests.
//!
//! Honest seam: neither artifact carries a **phase column**. `validation.bin`
//! v3 has eight features and six targets (angles, confidence, health, accel);
//! the trajectory sidecar adds true height and climb rate only. `phase_code()`
//! reconstructs the simulator's phase from those quantities: Pad, then Boost
//! (powered flight precedes a measured-height ground test), then ground
//! contact, descent, apogee and coast. Horizontal speed is approximated by
//! vertical speed at apogee, and `burn_end_s` is a caller input (default 9.95 s
//! from `sim/config/default.toml`). This is a deterministic replay of the
//! flight, not a bit-exact reconstruction of the simulator's internal phase enum.
//!
//! A non-finite pitot pressure is a typed [`PitotDp::Unavailable`], never a zero
//! a consumer could read as a valid 0 Pa sample.

use potok_protocol as pp;
use pp::{flags, Phase, ReplyFrame};

/// Simulator validation frame rate (the `frame_rate_hz` header field).
pub const SIM_FRAME_RATE_HZ: f32 = 50.0;
/// Firmware control tick rate (`Ticker::every(2 ms)` in `main.rs`).
pub const FIRMWARE_TICK_HZ: u32 = 500;
/// Source frames are held for exactly this many firmware ticks.
pub const TICKS_PER_SIM_FRAME: u8 = (FIRMWARE_TICK_HZ / SIM_FRAME_RATE_HZ as u32) as u8;
/// Duration of one firmware tick in microseconds.
pub const TICK_US: u32 = 1_000_000 / FIRMWARE_TICK_HZ;

/// `POTOKSIM` header length (see sim/README.md).
pub const VALIDATION_HEADER_SIZE: usize = 80;
pub const VALIDATION_MAGIC: &[u8; 8] = b"POTOKSIM";
/// `POTOKTRJ` header length (see sim/README.md).
pub const TRAJECTORY_HEADER_SIZE: usize = 64;
pub const TRAJECTORY_MAGIC: &[u8; 8] = b"POTOKTRJ";

const VERSION_V1: u32 = 1;
const VERSION_V2: u32 = 2;
const VERSION_V3: u32 = 3;
const META_V1: usize = 9;
const META_V2: usize = 17;
const META_V2_TRUTH: usize = 39;
const TARGETS_V3: usize = 6;
const TARGETS_LEGACY: usize = 4;
const RAW_FEATURES: usize = 8;
const DEFAULT_BURN_END_S: f32 = 9.95;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplayError {
    ShortHeader,
    BadMagic,
    UnsupportedVersion(u32),
    UnsupportedMetaCount(u32),
    BadTargetCount(u32),
    BadStride(u32),
    TooFewFeatures(u32),
    Truncated,
    ShortBuffer,
}

/// Simulator phase code (`Phase::code()` in `dynamics.rs`), before the
/// protocol's Pad/Boost collapse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SimPhaseCode {
    Pad = 0,
    Boost = 1,
    Coast = 2,
    Apogee = 3,
    Descent = 4,
    Landed = 5,
}

/// Forward-only phase event for the board FSM. Mirrors `phase::PhaseEvent`
/// without importing it, so this file stays self-contained for host tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplayEvent {
    LiftOff,
    Burnout,
    Apogee,
    Descent,
    Touchdown,
}

/// One 50 Hz source record, with simulator-side units preserved.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimTick {
    pub episode: u64,
    pub index: u32,
    pub time_s: f32,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub height: f32,
    pub pitot_pa: f32,
    pub pitot_dp: PitotDp,
    pub g_theta: [f32; 2],
    pub confidence: f32,
    pub ai_healthy: bool,
    pub g_accel: [f32; 2],
    /// True height / climb rate from the trajectory sidecar, when present.
    pub truth_height: Option<f32>,
    pub truth_climb_rate: Option<f32>,
}

/// One 500 Hz firmware tick, ready to be applied to `MockBoard`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReplayTick {
    pub time_us: u32,
    pub seq: u16,
    pub reply: ReplyFrame,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub height: f32,
    pub pitot_dp: PitotDp,
    pub phase: Phase,
    pub phase_code: SimPhaseCode,
    pub event: Option<ReplayEvent>,
    pub sensor_flags: u8,
    pub motor_armed: bool,
}

fn u32_at(b: &[u8], off: usize) -> Result<u32, ReplayError> {
    let a = b.get(off..off + 4).ok_or(ReplayError::Truncated)?;
    Ok(u32::from_le_bytes([a[0], a[1], a[2], a[3]]))
}

fn u64_at(b: &[u8], off: usize) -> Result<u64, ReplayError> {
    let a = b.get(off..off + 8).ok_or(ReplayError::Truncated)?;
    let mut v = 0u64;
    for (i, &byte) in a.iter().enumerate() {
        v |= (byte as u64) << (8 * i);
    }
    Ok(v)
}

fn f32_at(b: &[u8], off: usize) -> Result<f32, ReplayError> {
    Ok(f32::from_bits(u32_at(b, off)?))
}

fn take_f32_at(bytes: &[u8], cursor: &mut usize) -> Result<f32, ReplayError> {
    let v = f32_at(bytes, *cursor)?;
    *cursor += 4;
    Ok(v)
}

fn put_u32(out: &mut [u8], off: usize, v: u32) {
    out[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

fn put_f32(out: &mut [u8], off: usize, v: f32) {
    put_u32(out, off, v.to_bits());
}

/// Pitot wire value with a distinct unavailable state. A non-finite pressure
/// must not become `0`: that is a valid 0 Pa sample to every downstream reader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PitotDp {
    /// Finite pressure, clamped to the `i16` wire field.
    Available(i16),
    /// Non-finite pressure: no usable reading exists.
    Unavailable,
}

/// Checked conversion from a simulator pitot pressure to the wire field.
pub fn pitot_i16(pa: f32) -> PitotDp {
    if pa.is_finite() {
        PitotDp::Available(pa.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
    } else {
        PitotDp::Unavailable
    }
}

/// Map a simulator phase code onto the wire phase. Pad and Boost both become
/// `BURN`: the AI is gated off on the pad and during powered flight.
pub fn map_phase(code: SimPhaseCode) -> Phase {
    match code {
        SimPhaseCode::Pad | SimPhaseCode::Boost => Phase::BURN,
        SimPhaseCode::Coast => Phase::COAST,
        SimPhaseCode::Apogee => Phase::APOGEE,
        SimPhaseCode::Descent => Phase::DESCENT,
        SimPhaseCode::Landed => Phase::LANDED,
    }
}

/// Forward-only events; a regressing or repeated code produces no event.
pub fn phase_event(previous: SimPhaseCode, current: SimPhaseCode) -> Option<ReplayEvent> {
    match (previous, current) {
        (SimPhaseCode::Pad, SimPhaseCode::Boost) => Some(ReplayEvent::LiftOff),
        (SimPhaseCode::Boost, SimPhaseCode::Coast) => Some(ReplayEvent::Burnout),
        (SimPhaseCode::Coast, SimPhaseCode::Apogee) => Some(ReplayEvent::Apogee),
        (SimPhaseCode::Apogee, SimPhaseCode::Descent) => Some(ReplayEvent::Descent),
        (SimPhaseCode::Descent, SimPhaseCode::Landed) => Some(ReplayEvent::Touchdown),
        _ => None,
    }
}

/// Deterministic phase reconstruction from time, height and climb rate. Neither
/// artifact carries a phase column, so the gate orders Pad → Boost → ground →
/// descent/apogee/coast: powered flight precedes a measured-height ground test,
/// which otherwise labels a stationary pad `Landed` at `t > 0.5 s`. The
/// simulator's `phase_at` also looks at horizontal speed; vertical speed is the
/// available proxy at apogee, which is all the replay needs to drive the FSM.
pub fn phase_code(time_s: f32, height: f32, climb_rate: f32, burn_end_s: f32) -> SimPhaseCode {
    if time_s <= 0.0 {
        SimPhaseCode::Pad
    } else if time_s <= burn_end_s {
        // Powered flight precedes the ground test. The simulator checks
        // true position first; without a sidecar the replay has only a baro
        // height, and a pad/noisy reading at or below zero must not pre-empt
        // boost and emit a spurious Touchdown.
        SimPhaseCode::Boost
    } else if height <= 0.0 && time_s > 0.5 {
        SimPhaseCode::Landed
    } else if climb_rate < 0.0 {
        if height < 5.0 {
            SimPhaseCode::Landed
        } else {
            SimPhaseCode::Descent
        }
    } else if climb_rate * climb_rate < 4.0 && height > 10.0 {
        SimPhaseCode::Apogee
    } else {
        SimPhaseCode::Coast
    }
}

/// Zero-copy reader for `validation.bin` (v1, v2 and v3).
#[derive(Debug)]
pub struct ValidationReader<'a> {
    bytes: &'a [u8],
    pos: usize,
    pub version: u32,
    pub episodes: u32,
    pub frames_per_episode: u32,
    pub feature_count: u32,
    pub target_count: u32,
    pub frame_rate_hz: f32,
    episode: u32,
    frame: u32,
    remaining: u32,
    meta_floats: usize,
    current_episode: u64,
    truth_floats: usize,
    /// The writer emits every feature row first, then every target row.
    features_pos: usize,
    targets_pos: usize,
}

impl<'a> ValidationReader<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Self, ReplayError> {
        let header = bytes
            .get(..VALIDATION_HEADER_SIZE)
            .ok_or(ReplayError::ShortHeader)?;
        if &header[0..8] != VALIDATION_MAGIC {
            return Err(ReplayError::BadMagic);
        }
        let version = u32_at(header, 8)?;
        let meta_floats = match version {
            VERSION_V1 => META_V1,
            VERSION_V2 | VERSION_V3 => {
                let n = u32_at(header, 48)? as usize;
                if n != META_V2 && n != META_V2_TRUTH {
                    return Err(ReplayError::UnsupportedMetaCount(n as u32));
                }
                n
            }
            other => return Err(ReplayError::UnsupportedVersion(other)),
        };
        let targets = match version {
            VERSION_V1 | VERSION_V2 => TARGETS_LEGACY,
            VERSION_V3 => TARGETS_V3,
            _ => unreachable!("version checked above"),
        };
        let target_count = u32_at(header, 32)?;
        if target_count as usize != targets {
            return Err(ReplayError::BadTargetCount(target_count));
        }
        let feature_count = u32_at(header, 28)?;
        if feature_count < RAW_FEATURES as u32 {
            return Err(ReplayError::TooFewFeatures(feature_count));
        }
        Ok(Self {
            bytes,
            pos: VALIDATION_HEADER_SIZE,
            version,
            episodes: u32_at(header, 20)?,
            frames_per_episode: u32_at(header, 24)?,
            feature_count,
            target_count,
            frame_rate_hz: f32_at(header, 36)?,
            episode: 0,
            frame: 0,
            remaining: 0,
            meta_floats,
            current_episode: 0,
            truth_floats: meta_floats.saturating_sub(META_V2),
            features_pos: VALIDATION_HEADER_SIZE,
            targets_pos: VALIDATION_HEADER_SIZE,
        })
    }

    fn begin_episode(&mut self) -> Result<bool, ReplayError> {
        if self.episode >= self.episodes {
            return Ok(false);
        }
        self.current_episode = u64_at(self.bytes, self.pos)?;
        let data = self.pos + 8 + self.meta_floats * 4;
        self.features_pos = data;
        self.targets_pos =
            data + self.frames_per_episode as usize * self.feature_count as usize * 4;
        self.pos =
            self.targets_pos + self.frames_per_episode as usize * self.target_count as usize * 4;
        self.frame = 0;
        self.remaining = self.frames_per_episode;
        self.episode += 1;
        Ok(true)
    }

    pub fn next_tick(&mut self) -> Result<Option<SimTick>, ReplayError> {
        if self.remaining == 0 && !self.begin_episode()? {
            return Ok(None);
        }
        let mut features = [0f32; RAW_FEATURES];
        for slot in features.iter_mut() {
            *slot = take_f32_at(self.bytes, &mut self.features_pos)?;
        }
        self.features_pos += (self.feature_count as usize - RAW_FEATURES) * 4;
        let mut targets = [0f32; TARGETS_V3];
        for slot in targets.iter_mut().take(self.target_count as usize) {
            *slot = take_f32_at(self.bytes, &mut self.targets_pos)?;
        }
        let index = self.frame;
        let time_s = if self.frame_rate_hz > 0.0 {
            index as f32 / self.frame_rate_hz
        } else {
            0.0
        };
        self.frame += 1;
        self.remaining -= 1;
        Ok(Some(SimTick {
            episode: self.current_episode,
            index,
            time_s,
            gyro: [features[0], features[1], features[2]],
            accel: [features[3], features[4], features[5]],
            height: features[6],
            pitot_pa: features[7],
            pitot_dp: pitot_i16(features[7]),
            g_theta: [targets[0], targets[1]],
            confidence: targets[2],
            ai_healthy: targets[3] >= 0.5,
            g_accel: [targets[4], targets[5]],
            truth_height: None,
            truth_climb_rate: None,
        }))
    }

    /// The 22 terminal-truth floats of a truth-extended episode are part of
    /// the per-episode header; expose the count so callers can audit the layout.
    pub fn terminal_truth_floats(&self) -> usize {
        self.truth_floats
    }
}

/// Zero-copy reader for the per-tick `POTOKTRJ` sidecar (v1, v2 and v3).
#[derive(Debug)]
pub struct TrajectoryReader<'a> {
    bytes: &'a [u8],
    pos: usize,
    pub version: u32,
    pub episodes: u32,
    pub frames_per_episode: u32,
    pub stride: usize,
    episode: u32,
    remaining: u32,
}

impl<'a> TrajectoryReader<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Self, ReplayError> {
        let header = bytes
            .get(..TRAJECTORY_HEADER_SIZE)
            .ok_or(ReplayError::ShortHeader)?;
        if &header[0..8] != TRAJECTORY_MAGIC {
            return Err(ReplayError::BadMagic);
        }
        let version = u32_at(header, 8)?;
        let stride = match version {
            VERSION_V1 => 6usize,
            VERSION_V2 => 24usize,
            VERSION_V3 => 26usize,
            other => return Err(ReplayError::UnsupportedVersion(other)),
        };
        if u32_at(header, 20)? as usize != stride {
            return Err(ReplayError::BadStride(u32_at(header, 20)?));
        }
        Ok(Self {
            bytes,
            pos: TRAJECTORY_HEADER_SIZE,
            version,
            episodes: u32_at(header, 12)?,
            frames_per_episode: u32_at(header, 16)?,
            stride,
            episode: 0,
            remaining: 0,
        })
    }

    pub fn next_tick(&mut self) -> Result<Option<(u32, f32, f32)>, ReplayError> {
        if self.remaining == 0 {
            if self.episode >= self.episodes {
                return Ok(None);
            }
            u64_at(self.bytes, self.pos)?;
            self.pos += 8;
            self.remaining = self.frames_per_episode;
            self.episode += 1;
        }
        let height = f32_at(self.bytes, self.pos + 2 * 4)?;
        let climb_rate = f32_at(self.bytes, self.pos + 5 * 4)?;
        self.pos += self.stride * 4;
        self.remaining -= 1;
        Ok(Some((self.episode - 1, height, climb_rate)))
    }
}

#[derive(Debug)]
enum Source<'a> {
    Files {
        validation: ValidationReader<'a>,
        trajectory: Option<TrajectoryReader<'a>>,
    },
    Synthetic {
        frames: u32,
        next: u32,
    },
}

/// Stateful 50 Hz -> 500 Hz replay adapter.
#[derive(Debug)]
pub struct Replay<'a> {
    source: Source<'a>,
    frame_rate_hz: f32,
    burn_end_s: f32,
    seq_offset: u16,
    pending: Option<SimTick>,
    sub: u8,
    tick_index: u32,
    prev_code: SimPhaseCode,
    prev_height: f32,
    prev_episode: u64,
}

impl<'a> Replay<'a> {
    pub fn new(validation: &'a [u8], trajectory: Option<&'a [u8]>) -> Result<Self, ReplayError> {
        let reader = ValidationReader::new(validation)?;
        let frame_rate_hz = reader.frame_rate_hz;
        Ok(Self::from_readers(
            Source::Files {
                validation: reader,
                trajectory: match trajectory {
                    Some(bytes) => Some(TrajectoryReader::new(bytes)?),
                    None => None,
                },
            },
            frame_rate_hz,
        ))
    }

    /// No data on disk: replay a deterministic synthetic flight of `frames`
    /// source frames (ten firmware ticks each).
    pub fn synthetic(frames: u32) -> Self {
        Self::from_readers(Source::Synthetic { frames, next: 0 }, SIM_FRAME_RATE_HZ)
    }

    fn from_readers(source: Source<'a>, frame_rate_hz: f32) -> Self {
        Self {
            source,
            frame_rate_hz,
            burn_end_s: DEFAULT_BURN_END_S,
            seq_offset: 0,
            pending: None,
            sub: 0,
            tick_index: 0,
            prev_code: SimPhaseCode::Pad,
            prev_height: 0.0,
            prev_episode: u64::MAX,
        }
    }

    /// Last powered tick in `sim/config/default.toml`; callers with a custom
    /// thrust table should set this to that table's last non-zero thrust time.
    pub fn with_burn_end_s(mut self, seconds: f32) -> Self {
        self.burn_end_s = seconds;
        self
    }

    /// First reply seq. A fresh firmware `LinkState` issues seq 0 on its first
    /// tick, so the default matches one request per tick from link creation.
    pub fn with_seq_offset(mut self, seq: u16) -> Self {
        self.seq_offset = seq;
        self
    }

    fn pull_sim_tick(&mut self) -> Result<Option<SimTick>, ReplayError> {
        let mut tick = match &mut self.source {
            Source::Files {
                validation,
                trajectory,
            } => {
                let Some(tick) = validation.next_tick()? else {
                    return Ok(None);
                };
                if let Some(traj) = trajectory {
                    if let Some((_episode, height, climb_rate)) = traj.next_tick()? {
                        let mut tick = tick;
                        tick.truth_height = Some(height);
                        tick.truth_climb_rate = Some(climb_rate);
                        return Ok(Some(tick));
                    }
                }
                tick
            }
            Source::Synthetic { frames, next } => {
                if *next >= *frames {
                    return Ok(None);
                }
                let tick = synthetic_tick(*next);
                *next += 1;
                tick
            }
        };
        if tick.episode != self.prev_episode {
            self.prev_height = tick.height;
            self.prev_episode = tick.episode;
        }
        let climb_rate = (tick.height - self.prev_height) * self.frame_rate_hz;
        tick.truth_climb_rate = Some(climb_rate);
        tick.truth_height = Some(tick.height);
        self.prev_height = tick.height;
        Ok(Some(tick))
    }

    pub fn next_tick(&mut self) -> Result<Option<ReplayTick>, ReplayError> {
        if self.sub == 0 {
            match self.pull_sim_tick()? {
                Some(tick) => self.pending = Some(tick),
                None => return Ok(None),
            }
        }
        // The borrow ends before mutating `sub` below.
        let sim = *self.pending.as_ref().ok_or(ReplayError::Truncated)?;
        let height = sim.truth_height.unwrap_or(sim.height);
        let climb_rate = sim.truth_climb_rate.unwrap_or(0.0);
        let code = phase_code(sim.time_s, height, climb_rate, self.burn_end_s);
        let event = phase_event(self.prev_code, code);
        self.prev_code = code;

        let sensor_ok = sim.gyro.iter().all(|v| v.is_finite())
            && sim.accel.iter().all(|v| v.is_finite())
            && sim.height.is_finite()
            && sim.pitot_dp != PitotDp::Unavailable;
        let mut sensor_flags = flags::IMU_OK | flags::BARO_OK | flags::PITOT_OK;
        if !sensor_ok {
            sensor_flags |= flags::FAULT;
        }
        let phase = map_phase(code);
        let motor_armed = phase == Phase::BURN;
        if motor_armed {
            flags::set(&mut sensor_flags, flags::MOTOR_ARMED);
        }
        let confident = if sim.confidence <= 0.0 {
            0u8
        } else if sim.confidence >= 1.0 {
            255u8
        } else {
            (sim.confidence * 255.0 + 0.5) as u8
        };
        let tick = ReplayTick {
            time_us: self.tick_index * TICK_US,
            seq: self.seq_offset.wrapping_add(self.tick_index as u16),
            reply: ReplyFrame {
                seq: self.seq_offset.wrapping_add(self.tick_index as u16),
                // Sim targets are [roll, pitch]; the wire reply is [pitch, yaw].
                g_pitch: pp::reply_g_pitch_from_rad(f64::from(sim.g_theta[1])),
                g_yaw: pp::reply_g_yaw_from_rad(f64::from(sim.g_theta[0])),
                confidence: confident,
                ai_healthy: if sim.ai_healthy { 1 } else { 0 },
                version: pp::REPLY_VERSION,
            },
            gyro: sim.gyro,
            accel: sim.accel,
            height: sim.height,
            pitot_dp: sim.pitot_dp,
            phase,
            phase_code: code,
            event,
            sensor_flags,
            motor_armed,
        };
        self.tick_index += 1;
        self.sub += 1;
        if self.sub == TICKS_PER_SIM_FRAME {
            self.sub = 0;
            self.pending = None;
        }
        Ok(Some(tick))
    }
}

/// Deterministic source frame used when no simulator artifact is present.
/// Integer arithmetic only: no RNG, no wall clock, no libm dependency.
pub fn synthetic_tick(index: u32) -> SimTick {
    let i = index as i32;
    let height = index as f32 * 0.5;
    let pitot_pa = (index % 13) as f32 * 10.0;
    SimTick {
        episode: 0,
        index,
        time_s: index as f32 / SIM_FRAME_RATE_HZ,
        gyro: [
            ((i % 7) - 3) as f32 * 0.001,
            ((i % 5) - 2) as f32 * -0.002,
            ((i % 3) - 1) as f32 * 0.003,
        ],
        accel: [
            ((i % 9) - 4) as f32 * 0.01,
            ((i % 11) - 5) as f32 * -0.02,
            9.8 + (i % 3) as f32 * 0.1,
        ],
        height,
        pitot_pa,
        pitot_dp: pitot_i16(pitot_pa),
        g_theta: [((i % 5) - 2) as f32 * 0.01, ((i % 3) - 1) as f32 * 0.02],
        confidence: 0.5 + (index % 5) as f32 * 0.1,
        ai_healthy: true,
        g_accel: [0.0, 0.0],
        truth_height: None,
        truth_climb_rate: None,
    }
}

/// Write a complete one-episode v3 `validation.bin` fixture into `out`.
/// Returns the number of bytes written; a short buffer is a loud error.
pub fn synthetic_validation_bytes(frames: u32, out: &mut [u8]) -> Result<usize, ReplayError> {
    let need = VALIDATION_HEADER_SIZE
        + 8
        + META_V2 * 4
        + frames as usize * (RAW_FEATURES + TARGETS_V3) * 4;
    if out.len() < need {
        return Err(ReplayError::ShortBuffer);
    }
    for byte in out[..need].iter_mut() {
        *byte = 0;
    }
    let h = &mut out[..VALIDATION_HEADER_SIZE];
    h[0..8].copy_from_slice(VALIDATION_MAGIC);
    put_u32(h, 8, VERSION_V3);
    put_u32(h, 20, 1);
    put_u32(h, 24, frames);
    put_u32(h, 28, RAW_FEATURES as u32);
    put_u32(h, 32, TARGETS_V3 as u32);
    put_f32(h, 36, SIM_FRAME_RATE_HZ);
    put_f32(h, 40, 1.0 / SIM_FRAME_RATE_HZ);
    put_f32(h, 44, frames as f32 / SIM_FRAME_RATE_HZ);
    put_u32(h, 48, META_V2 as u32);
    let mut pos = VALIDATION_HEADER_SIZE + 8 + META_V2 * 4;
    for frame in 0..frames {
        let s = synthetic_tick(frame);
        for v in [
            s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2], s.height,
            s.pitot_pa,
        ] {
            put_f32(out, pos, v);
            pos += 4;
        }
    }
    for frame in 0..frames {
        let s = synthetic_tick(frame);
        for v in [
            s.g_theta[0],
            s.g_theta[1],
            s.confidence,
            1.0,
            s.g_accel[0],
            s.g_accel[1],
        ] {
            put_f32(out, pos, v);
            pos += 4;
        }
    }
    debug_assert_eq!(pos, need);
    Ok(need)
}
