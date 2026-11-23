<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# SIL — full shipped path (host only)

This page documents the host harness added in `firmware/src/sil.rs` (`mod full`)
and its gates in `firmware/tests/sil_tests.rs`. It runs the **`main.rs` loop
body with the pins removed**: board link, phase event, airdata sampling,
`FlightPath::tick` (voting → Mahony → PID → deployment → separation → radio),
`control::step` (independent interlock + Mach shield + watchdog policy),
`apply_command`, watchdog feed, and the typed telemetry cadences.

It is a deterministic regression filter on the host. **It makes no hardware
claim**: no pin, UART, DMA, clock, flash, servo, radio or flight behaviour is
exercised or implied. A green run here is not flight qualification; see
[SIL.md](SIL.md) and [HIL_CHECKLIST.md](HIL_CHECKLIST.md) for what only the
board can prove.

## Why this exists

The older SIL harness drove `step_link()` through a `Guarded` board decorator
that placed an interlock in the actuation path. That is a useful fault
microscope, but it is not the binary: the binary calls `step()`, owns its own
`Interlocks` and `WatchdogPolicy`, samples airdata before the step, and runs
the flight-path modules before it. The full-path harness drives exactly that
sequence; the older tests remain unchanged as the link-level microscope.

## What runs, and in what order

Each scripted 500 Hz tick (`FlightTick`) does this, in the binary's order:

1. **Board link** — scripted `ReplyFrame`/raw bytes are queued into `MockBoard`'s
   RX; one request is sent with a strict `seq`. No wall clock: tick *n* reads
   `n × 2000 µs`.
2. **Phase event** — an optional `PhaseEvent` is placed on the board and
   consumed by `link_tick` inside `step()`, exactly as the hardware adapter
   would publish one.
3. **Airdata** — `ShieldSampler::sample` uses a scripted `Airdata` source.
   A missing driver returns `None` and is tallied as `Unavailable::NoDriver`;
   the sample is `ShieldSample::UNAVAILABLE`, which the shield vetoes. A
   fabricated zero is never substituted for a missing reading.
4. **Flight-path modules** — `FlightPath::tick` calls sensor voting, Mahony,
   PID, deployment, separation, self-test and the search beacon. Each input
   lives in `FlightInputs`; `None` means the board does not own that capability
   and `FlightPath` counts it `Unavailable` with a reason. `apply_stabilization`,
   `apply_springs`, `apply_separation` and `apply_radio` are separate,
   capability-scoped sinks; they never touch `apply_command`.
5. **Shipped safety step** — `step()` runs the link gates, the independent
   interlock (`evaluate_shield`: phase/health/fault/latch *and* pitot/Mach),
   the watchdog policy, and the single `board.apply_command` call. A veto
   collapses the output to `Command::ZERO`.
6. **Watchdog feed** — only when the cycle is complete, the deadline monitor
   has not tripped and the link deadman allows it. An incomplete cycle
   withholds the feed; the shipped `step()` still applies the already-vetted
   command (the cycle flag is a feed gate, not a command gate). The test suite
   asserts this explicitly so nobody "fixes" it into a different program.
7. **Telemetry** — the same seven `LogGate`s as the binary (reject, applied,
   airdata, AI-latch, interlock-latch, deadman, capability summary), with the
   same cadences, produce a per-tick emission bitmask in the trace.

Every stage records an output, and the reachability mask is set only from an
observation made that tick. A module whose output never moved cannot mark
itself reached.

## Capabilities: absent means counted

Two ledgers describe debt, never silence:

- `FlightPath::ledger()` counts the seven module capabilities
  (`SensorVoting`, `Mahony`, `Pid`, `Deployment`, `Separation`, `SelfTest`,
  `Radio`) as `used` or `unavailable` with `NoDriver`/`NoSample`/`Refused`/
  `OutputFailed`.
- `FlightMockBoard::board_caps` counts the three `Board` capabilities the
  hardware adapter stubs (`Actuation`, `PhaseSense`, `MotorArmedSense`).
  `FlightSimulator::stubbed()` mirrors the unimplemented `HwBoard`: each is
  counted `Unavailable::NoDriver` on every tick, and because no phase event
  arrives the FSM stays in BURN and the applied command stays zero — the safe
  direction, with the reason recorded.

`ShieldSampler` has its own `UnavailableCounts` (`no_driver`, `non_finite`,
`non_positive`). The telemetry gate's key uses the *active reason bits*, never
the growing tallies, so a steady fault is summarised once and counted in
between instead of streaming 500 Hz.

## Gates in `sil_tests.rs`

| Gate | What it proves |
|---|---|
| `full_path_reaches_every_shipped_module` | All 15 stage bits set; all seven module capabilities produced an output; interlock accepted, command applied, watchdog fed, telemetry emitted, airdata read, attitude and PID output moved. The one deployment refusal on the BURN tick is asserted, not hidden. |
| `absent_capabilities_are_counted_unavailable_never_silently_skipped` | With no inputs and no pitot, no module claims to have run, every module capability has an `unavailable` tally, the sampler counts a missing driver, and every applied command is zero with `PitotInvalid` on the shield. |
| `stubbed_board_counts_board_level_capabilities_unavailable` | The three `Board` stubs are counted every tick with `NoDriver`; no phase sense keeps the applied command zero. |
| `zero_command_survives_every_fault_injection` | Corrupt CRC, stale seq, silent link, unhealthy AI, board FAULT, motor armed, missing pitot, non-positive pitot, non-finite Mach and above-limit Mach all leave `applied` zero, and at least one gate reports the reason. |
| `incomplete_cycle_withholds_the_feed_without_rewriting_the_gated_command` | Pins the honest `step()` behaviour described above. |
| `full_path_replay_is_bit_identical` | Two runs of one script give identical traces, counters, attitude, stabilization output, applied commands and capability totals. |
| `full_trace_overflow_fails_loud` | A too-small trace panics rather than dropping a tick. |

The original 12 link-level SIL gates are unchanged and still run.

## Scope and limits

- The harness reproduces `main.rs`'s loop body, not the Embassy adapter.
  `build_board`, UART RX/TX, IWDG setup, pin maps and DMA streams are still
  `todo!(schematic)` and are not exercised.
- `FlightPath`'s stabilization output goes to a capability-scoped mock sink,
  not into `apply_command`. This harness proves the module runs and its output
  is observable; it does **not** claim an onboard PID closes the actuator loop
  or replaces the AI reply.
- Faults that cannot enter through the shipped `step()` signature are not
  fabricated here. An unknown interlock phase code, for example, is covered by
  the link-level `Guarded` harness (`interlock_latch_blocks_a_link_accepted_
  command_until_reset`), not by mutating the FSM's phase byte.
- Timing is synthetic (2000 µs per tick); no DWT counter or wall clock is read.
  The cycle model in `perf.rs` remains an arithmetic model, not a measurement.
- No claim is made about pins, electrical signals, real time, an actual flight,
  or flight qualification. HIL on the real board remains the final verification.
