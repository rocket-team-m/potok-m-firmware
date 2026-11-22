<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# SIL — software-in-the-loop verification

SIL runs the firmware's decision logic on the host, with the hardware boundary
replaced by `MockBoard`. It is a fast regression filter for the safety gates,
the wire protocol and the timing model. **SIL is not flight qualification.**
Real-board/HIL verification remains the final verification (see
[What SIL does not cover](#what-sil-does-not-cover)).

The same boundary that makes SIL possible is described in
[ARCHITECTURE.md](ARCHITECTURE.md): `lib.rs` is `#![no_std]`, `extern crate std`
is host-only, and the Embassy dependencies live behind `cfg(target_os = "none")`.
The host tests therefore exercise the firmware sources, not a reimplementation.

## The `sil` CI job

The dedicated workflow [`.github/workflows/sil.yml`](../.github/workflows/sil.yml)
runs on every push and pull request. It is complementary to `ci.yml` (format,
clippy and the ordinary workspace test/check gates), not a replacement for it.

| Step | Command | What it proves |
|---|---|---|
| Host SIL suite | `cargo test --workspace` | Every host integration gate passes: safety, protocol link, phase FSM, interlocks, PID, attitude, deployment, separation, sensor voting, self-test, telemetry, scheduler, watchdog, buzzer, calibration and the cycle model. Catches gates added later that the focused steps below do not name. |
| Fault-injection gates | `cargo test --test safety --test watchguard_tests --test protocol_link_tests` | Corrupt CRC/magic/version/seq, stale and wrapped seq, noise without magic, split DMA frames, repeated DMA submission, mixed bad frames plus timeout, silent link and `ai_healthy = 0` all drive the three-strike latch and a `Command::ZERO` output. Re-runs the wire-fault subset so a red step names the failing class. |
| Timing-budget gates | `cargo test --test perf_tests --test scheduler_tests --test watchdog_tests` | The cycle model is pinned (716 accepted / 708 rejected / 556 idle cycles against a 24 000-cycle, 50 µs budget at 480 MHz), the 500 Hz tick grid and jitter accounting hold, and the watchdog deadline monitor (250 µs slack, 3 missed deadlines) and feed policy behave. Focused re-run for attribution; these are arithmetic/model checks, not measurements on silicon. |
| thumb `no_std` check | `cargo check --target thumbv7em-none-eabihf` | The real Embassy adapter, IWDG setup and linker configuration still compile for Cortex-M7, and no host-only code leaks into the target build. It does **not** flash or run anything. |

Run the same four commands locally before a board session:

```sh
cargo test --workspace
cargo test --test safety --test watchguard_tests --test protocol_link_tests
cargo test --test perf_tests --test scheduler_tests --test watchdog_tests
cargo check --target thumbv7em-none-eabihf
```

## What SIL covers

- **Wire protocol** — encode/decode round trips with `potok-protocol` as the
  single source of the frame format, CRC and version validation, strict `seq`
  with `u16` wrap, `ReplyAssembler` noise handling and split-magic reassembly.
- **Safety gates** — the BURN gate and independent `MOTOR_ARMED` bit, the code
  veto (`MAX_CORRECTION_RAW = 100`, i.e. 0.1 rad), the three-consecutive-miss
  latch with explicit-reset-only clearing, and the watchdog/deadman feed
  policy. `apply_command` receives `ZERO` on rejected frames and timeouts.
- **Deterministic logic** — forward-only phase FSM, double-buffer RX swapping,
  fixed-point/Q15 fast paths, redundant-sensor voting, deployment and
  separation sequencers, preflight self-test, flight-log ring/replay, buzzer
  and search-beacon patterns. Repeated identical inputs produce identical
  outputs; clock wrap and degenerate inputs are handled without silent
  fallbacks.
- **Timing model** — a pinned per-cycle cost model and tick/deadline budgets,
  so a change that quietly inflates the hot path turns a test red.

Every gate above runs without a board, without Embassy and without an
allocator.

## What SIL does not cover

SIL cannot prove anything about pins, electrical signals, real time or an
actual flight. These checks require the real board and Raspberry Pi and are
listed in the hardware-test section of
[`FIRMWARE_RUST_PLAN.md`](../FIRMWARE_RUST_PLAN.md):

- Logic-analyser captures at 2 Mbit/s: frame lengths (56/14 and 42/16) and
  measured wire time against the 0.25 ms budget. The SIL cycle model is an
  arithmetic estimate, not a measurement.
- Back-to-back frames at full rate for at least an hour with no byte loss,
  including idle-line gaps and Pi scheduling jitter.
- Physical fault injection on the wire (corrupt magic, CRC, version, seq)
  against the real UART/DMA path and its rejection counters.
- BURN gate with servos connected; unplugging the UART and killing the Pi,
  then confirming zero output, stale detection, watchdog reset and its logged
  reset reason.
- Cross-language interop against the Pi client with the golden frames on both
  profiles; cold flash, brownout, and RAM/flash headroom.
- Pin numbers, alternate functions and DMA streams, the chip variant from the
  BOM and the `memory.x` linker layout. `hw::build_board()` is still
  `todo!(schematic)`, so no board adapter is exercised by SIL.

A green `sil` job is necessary before a board session and is **not sufficient**
to call the firmware flight-qualified. HIL on the real board is the final
verification.

## Scope of this document

This page describes verification scope only. It does not change source,
workflows other than [`sil.yml`](../.github/workflows/sil.yml), or the safety
invariants in the [README](../README.md) and
[ARCHITECTURE.md](ARCHITECTURE.md). See also
[`ci.yml`](../.github/workflows/ci.yml) for the format/clippy gates.
