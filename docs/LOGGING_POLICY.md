<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Logging policy — bounded-rate runtime telemetry

**Date:** 2026-11-24 · **Repo:** `/home/sehaxe/potok-m-firmware` · **Finding:** MCUF01 §4.2.

## 1. The defect this policy fixes

The shipping loop (`firmware/src/main.rs`) ticks at 500 Hz. Before this change
it logged on every tick: with no pitot driver wired (`main.rs`, `Airdata` impl)
every accepted reply yields a zero command and emitted a six-field
`defmt::info!` of shield reasons and airdata-unavailable tallies.

Two things made that expensive and invisible:

1. The deterministic cycle model (`firmware/src/perf.rs`, `CycleEstimate`) has
   **no term for `defmt`/RTT**. The model's 716-cycle accepted tick does not
   describe the program that actually ran.
2. There is **no DWT measurement** of the loop anywhere in the repo
   (`perf::read_cycles()` has no call site). The cost of the 500 Hz log was
   therefore unmeasurable, not merely unbudgeted.

The fix makes the log rate a declared, host-testable decision. It does not
claim a cycle number it cannot measure.

## 2. Typed policy

`firmware/src/telemetry_log.rs` owns three types:

| Type | Role |
|---|---|
| `Emission` | Why an observation was allowed out: `Safety`, `Change`, `Cadence`, `Debug`, or `Suppressed`. Only a non-`Suppressed` result may reach `defmt!`. |
| `Cadence` | Declared rate: `on_change()` (no periodic), `periodic(n)` with `n >= 2`, or `debug_every_tick()` (the explicit `n == 1` opt-in). `periodic(1)` panics loudly. |
| `LogGate<Key>` | Stateful gate over a typed key: dedupes on key change, emits at the declared cadence, and counts every suppressed observation (`suppressed()`) and every approved one (`emitted()`). |

Rules, enforced in code and in `tests/telemetry_log_tests.rs`:

- **State change** → emit now (`Emission::Change`), not on a timer.
- **New error / safety reason** → `observe_error()` emits now
  (`Emission::Safety`); it is never deferred to a cadence window.
- **Slow cadence** → periodic summary at most once per declared period.
- **Suppression is counted**, never silent. Every emitted line carries the
  gate's running `suppressed` count, so an RTT reader sees what was withheld.
- **Every tick only via explicit debug**: `Cadence::periodic()` refuses `1`;
  the sole route is `Cadence::debug_every_tick()`, selected by the named
  `DEBUG_EVERY_TICK` constant in `main.rs` (currently `false`).

### The key must be qualitative

`Key` must be the *active-reason state*, never a cumulative tally. With no
pitot driver, all six tallies grow every tick; keying on them would make every
tick a "state change" and reproduce the defect. The gate key is a bitmask of
which reasons are active (`count > 0`); the growing tallies stay in the
emitted payload only. This is pinned by
`monotonic_tallies_must_not_be_used_as_the_gate_key`.

## 3. Declared cadences in the shipping loop

One gate per message; each owns its dedupe, cadence and counters.
`TICK_HZ` is 500, so `Cadence::periodic(TICK_HZ)` is one summary per second.

| Message | Key | Cadence | Class |
|---|---|---|---|
| six-field zero-command airdata line | `AirdataState` (6 active-reason bits) | `periodic(TICK_HZ)` — 1 Hz | info |
| `reply frame rejected` | `()` | `periodic(TICK_HZ)` — 1 Hz | warn |
| `AI command applied` | `()` | `periodic(TICK_HZ)` — 1 Hz | info |
| `AI latched off` | `bool` latch | `on_change`, via `observe_error` | safety |
| `independent interlock latched` | `bool` latch | `on_change`, via `observe_error` | safety |
| `control deadline monitor tripped` | `bool` latch | `on_change`, via `observe_error` | safety |

With no pitot driver the airdata line emits **once on the first tick**, then at
1 Hz; the other ~499 ticks per second are counted as suppressed instead of
formatted into RTT.

## 4. Safety messages stay immediate

A latch warning is emitted on **entry** into the latched state (`false → true`
key change), without waiting for a cadence window. While the latch holds,
repeats are suppressed and counted rather than streamed at 500 Hz. A clear and
re-latch emits again immediately. This is the reading of "immediate" used
here: immediate on the safety-relevant event, never deferred — not "repeated
every tick", which is the defect being removed. `latch_entered()` in `main.rs`
keeps the condition test (`latched`) and the gate decision in one place.

The `panic` handler's `defmt::error!` is unchanged and unconditional: a reset
is not a cadenced telemetry event.

## 5. Measurement integrity

- No claim in this document is a cycle measurement. The cycle model still has
  no defmt term (out of scope here); what changed is that the log **rate is
  now bounded by declaration**, so a future model term or DWT bracket has a
  fixed upper bound to price: at most one multi-field record per gate per
  second in production, plus first-entry emissions.
- To measure on silicon later: enable `perf::init_cycle_counter()` once, bracket
  the logging decision with `read_cycles()` on a board with UART/RTT wired, and
  compare a `DEBUG_EVERY_TICK = false` run against a `true` run at the same
  tick count. Do this before trusting any absolute number.

## 6. Invariants

1. No production gate selects `Cadence::debug_every_tick()`.
2. No gate key is a monotonic counter.
3. Every `defmt!` in the 500 Hz loop is behind an `Emission::should_emit()`
   check; the panic handler is the only unconditional emitter.
4. Suppressed observations are counted on every gate and surfaced on its
   emitted lines; nothing is silently dropped.
5. `periodic(1)` panics; every-tick logging requires the named debug flag.
