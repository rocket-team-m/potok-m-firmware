# Deployment thresholds: derivation and evidence status

**Date:** 2026-11-24 · **Repo:** `/home/sehaxe/potok-m-firmware` · **Closes:** DEP-1 and DEP-4 in `MATH_PROVENANCE.md`.
**Scope:** the seven fields of `DeploymentConfig::DEFAULT` in `firmware/src/deployment.rs:51-59`.
No threshold value was changed. This document replaces the word "Verified" (now removed from the
struct header) with an explicit derivation; it does not promote a design value to a flight-qualified
one.

## Authority rule

Each field is labelled **REQUIREMENT**: a mission/design limit imposed from outside the mathematics,
not a formula and not a measurement. Where the number depends on an external fact, the fact is cited
by exact path and line if it exists in-tree; where it does not, the section says **NO SOURCE** and
names the measurement that would close it. An in-tree placeholder is never presented as a measured
value, and an in-tree document is never presented as an external standard.

**There is no flight-log or drop-test source in these trees.**

- `flights/README.md:5-8` says raw logs and reports live in Releases under tags `flight-NN`; this
  checkout has no such tag and no log file. There is no baro altitude trace, no apogee time and no
  measured descent rate to fit or validate a threshold against.
- `sim/config/parachute.toml:8-13` states "NO DROP TEST EXISTS IN THIS TREE" and marks both canopy
  speeds `placeholder`; `sim/docs/PARACHUTE.md:41-44` repeats that they are not Potok-M canopy
  measurements. The promotion gate is `sim/core/src/parachute.rs:145-174`
  (`provenance_signature` / `SignedProvenance::sign`), which requires a signed record bound to the
  exact value. No such record exists here.
- `docs/HIL_CHECKLIST.md:228` (D06, "Threshold authority") already records the design position:
  the `DEFAULT` values are **logic constants, not flight thresholds**, and owner-signed values must
  be recorded in its (currently empty) Result/Evidence column. D05 and D07 (`:227`, `:229`) gate the
  decision logic only.

Consequently every section below ends with NO SOURCE for the exact numeric value. The derivations
show that the shipped values are *consistent* with an explicit requirement and stated assumptions;
they do not show that the assumptions are true on hardware.

## Fixed in-tree inputs used by the derivations

| Input | Value | Citation | Status |
|---|---|---|---|
| Tick cadence | 500 Hz, `TICK_PERIOD_US = 2000 µs` | `firmware/src/scheduler.rs:18-20`; shipping tick `main.rs:202,220` | IN-TREE REQUIREMENT |
| Altitude quantum | whole metres AGL, 1 m per count | `firmware/src/deployment.rs:22-23`, `:131-133` | IN-TREE CHOICE |
| Speed quantum/sign | cm/s, `>0` climbs, `<0` descends, 1 cm/s per count | `firmware/src/deployment.rs:23-24`, `:134-135` | IN-TREE CHOICE |
| Consistency bounds | `0 ≤ floor < low ≤ apogee_min`; `drop>0`, `climb>0`, `confirm>0`, `rate<0` | `firmware/src/deployment.rs:70-94` | IN-TREE CHOICE (DEP-2) |
| Mission apogee | minimum 1 km, target corridor 2–2.5 km; user-stated TTX | `/home/sehaxe/potok-m/vehicle/DESIGN_INPUT.md:42` | IN-TREE TTX, not a measurement |
| Canopy model (placeholder) | drogue 25 m/s, main 6.5 m/s, main trigger 1500 m, main inflation 2.0 s | `/home/sehaxe/potok-m/sim/config/parachute.toml:26`, `:34-40` | PLACEHOLDER, no drop test |
| Baro noise in sim | 0.35 (unit as configured), labelled an assumption | `/home/sehaxe/potok-m/sim/config/default.toml:73` | PLACEHOLDER, not sensor data |

The canopy model is a *different module* from this decision layer: the simulator deploys its main at
1500 m (`parachute.toml:34`), while `deployment.rs` is a single irreversible spring release whose
lowest backup is 15 m. The canopy numbers are used below only as the sole in-tree numbers for
"controlled descent rate" and "inflation time". They are not evidence that the firmware thresholds
match the mission recovery system.

---

## 1. Sample counts — `descent_confirm = 3`

**Requirement.** An irreversible release must not be committed from a single bad speed sample. A
descending run must be observed on consecutive ticks, and a glitch shorter than the run must be
rejected.

**Assumptions.** 500 Hz ⇒ `Δt = 2 ms/tick` (`scheduler.rs:18-20`). The counter increments only while
`v < 0` and resets to zero on `v >= 0` (`deployment.rs:235-241`); the release reads `count >= 3`
(`:243`). A run of length *N* therefore rejects any descending burst of at most *N−1* consecutive
samples.

**Arithmetic.** With *N* = 3: rejected burst ≤ 2 samples; first-to-third span `(3−1)·2 ms = 4 ms`;
decision latency from the first descending sample `3·2 ms = 6 ms`. At the fastest in-tree descent
number (placeholder drogue 25 m/s) the altitude lost in that latency is
`25 m/s × 0.006 s = 0.15 m`, below the 1 m altitude quantum, so the confirmation itself does not
add a whole altitude count of uncertainty. The bound `N·Δt·v < 1 m` gives `N < 20`, so *N* = 3 is
feasible but not *determined* by that bound; *N* = 2 would also satisfy it while rejecting only
1-sample bursts.

**Status.** REQUIREMENT. **NO SOURCE** for the choice of 3 rather than 2 or 4: no glitch statistics
exist in-tree. **Missing measurement:** a flight-log or HIL capture of the baro-derived speed sign
sequence near apogee and at low altitude — the distribution of consecutive same-sign runs and the
rate of single/double-tick sign flips. Until then, "3 rejects a 2-sample glitch" is a statement about
the code, not about the sensor.

## 2. Apogee detection — `apogee_min_altitude_m = 30`, `apogee_drop_m = 2`, `climb_confirm_m = 5`

The apogee trigger requires all of: `peak >= 5 m`, current altitude `>= 30 m`, current altitude
`<= peak − 2 m`, plus the 3-sample descent confirmation (`deployment.rs:243-250`).

### 2a. Floor `apogee_min_altitude_m = 30 m`

**Requirement.** Do not credit an apogee descent near the pad: the release must stay below the
arming floor while the vehicle is still in boost or subject to launch-site pressure transients. The
floor must also stay at or above the low-altitude backup band so the two triggers cannot both own
the same descent (`deployment.rs:76-78` enforces `apogee_min >= low`).

**Assumptions.** Mission apogee ≥ 1 km (`DESIGN_INPUT.md:42`) ⇒ by the time descent begins the
vehicle is far above 30 m; the floor is an anti-premature-fire guard, not an apogee predictor.
Altitude LSB = 1 m.

**Arithmetic.** `30 m = 30 LSB = 2 × 15 m` (low backup) `= 6 × 5 m` (climb confirm). From 30 m down
to the 15 m backup band takes `(30−15)/25 = 0.6 s` at the placeholder drogue rate and
`(30−15)/6.5 = 2.3 s` at the placeholder main rate; both exceed the 6 ms confirmation window, so
the apogee decision completes before the backup band is entered.

**Status.** REQUIREMENT. **NO SOURCE** for 30 m: no flight-log altitude trace exists.
**Missing measurement:** baro altitude error and the flight-log altitude at which post-apogee
descent becomes unambiguous; also the altitude at which a launch-site pressure transient decays.

### 2b. Drop `apogee_drop_m = 2 m`

**Requirement.** Require more than one altitude count of fall below the tracked peak before calling
apogee, so a single quantisation step cannot be mistaken for the turn-over.

**Assumptions.** Altitude LSB = 1 m (`deployment.rs:22-23`); peak is the max over armed samples
(`:232-234`); `drop > 0` is enforced (`:79-82`). The slowest descending sample in the gates is
−1 m/s (`deployment_tests.rs:165-175`).

**Arithmetic.** `2 m = 2 LSB`, i.e. 1 count of margin over the quantum (100 % of one LSB). At
1 m/s the 2 m drop takes `2 s`; at the placeholder drogue 25 m/s it takes `2/25 = 80 ms`. Both are
≫ 6 ms confirmation, so under either rate a genuine 2 m drop is a sustained descent, not a glitch.

**Status.** REQUIREMENT. **NO SOURCE** for 2 m: the baro error near apogee is not measured (the sim
uses an unmeasured 0.35 noise field, `default.toml:73`).
**Missing measurement:** flight-log baro altitude sigma at apogee; the drop must exceed that sigma
by a stated multiple, which cannot be computed without it.

### 2c. Climb verification `climb_confirm_m = 5 m`

**Requirement.** Before a descent is credited as post-apogee, the vehicle must have been observed at
least 5 m above the launch site, so handling on the pad or a pressure transient cannot arm the
trigger.

**Assumptions.** Altitude LSB = 1 m; gate fixtures climb at 3 m/s (`DeploymentSample` of
`300 cm/s` in `deployment_tests.rs:31-36`).

**Arithmetic.** `5 m = 5 LSB`, reached in `5/3 = 1.67 s` at the fixture ascent rate — inside the
~5–6 s first-stage burn window noted in `sim/config/default.toml:132-140`. At the configured
values this field is **redundant**: any sample with `altitude >= 30 m` forces
`peak >= 30 m >= 5 m`, so `climbed` (`deployment.rs:244`) is automatically true whenever the apogee
floor is met. Any `climb_confirm` in `1..=30` would produce identical decisions at these settings.

**Status.** REQUIREMENT. **NO SOURCE** for 5 m. **Missing measurement:** the magnitude of pad-side
altitude transients (HIL baro capture before lift-off). The derivation cannot choose 5 over any
other redundant value without that measurement; the field is kept because removing it would be a
logic change, which is outside this task's scope.

## 3. Fast-descent backup — `fast_descent_cm_s = -1500`

**Requirement.** An emergency trigger that fires only on a descent faster than the controlled
recovery descent, so it indicates a missed or failed recovery event rather than nominal behaviour.
It is suppressed below the hard floor (`deployment.rs:251-254`).

**Assumptions.** The only in-tree descent-rate numbers are the simulator's **placeholder** settled
rates: drogue 25 m/s and main 6.5 m/s (`sim/config/parachute.toml:26`, `:35`, `:38-40`), explicitly
not measured (`:8-13`; `sim/docs/PARACHUTE.md:41-44`). Speed LSB = 1 cm/s.

**Arithmetic.** `−1500 cm/s = −15 m/s`, exactly representable in the integer field. Ratio to
settled main: `15 / 6.5 = 2.31×`; ratio to settled drogue: `15 / 25 = 0.60×`. The threshold therefore
sits between the two placeholder rates: above nominal main descent, below nominal drogue descent.
At 15 m/s one altitude count (1 m) is crossed in `1/15 = 67 ms`, ≫ the 6 ms confirmation, so the
confirmation window cannot straddle more than about 0.09 m of descent.

**Status.** REQUIREMENT, conditional on placeholder canopy rates. **NO SOURCE** for −15 m/s:
no measured descent rate and no signed drop-test record exist.
**Missing measurement:** the signed drop-test values for settled main and drogue descent, promoted
through `sim/core/src/parachute.rs:145-174`, plus the baro-derived vertical-speed error. Without
them the 2.31×/0.60× margins are arithmetic on assumptions, not a safety margin.

## 4. Low-altitude main backup — `low_altitude_m = 15`

**Requirement.** The last backup must release while there is still altitude left for the canopy to
inflate before touchdown, and must sit strictly above the hard floor so the two paths are disjoint
(`deployment.rs:72-74` enforces `low > floor`).

**Assumptions.** Placeholder main inflation time constant `τ = 2.0 s` and settled main descent
6.5 m/s (`sim/config/parachute.toml:35-36`). The simulator's inflation is a first-order relaxation
(`sim/docs/PARACHUTE.md:26-29`), so `τ` reaches ~63 % of the target rate, not 100 %.

**Arithmetic.** Altitude consumed during one inflation time constant at settled main rate:
`6.5 m/s × 2.0 s = 13.0 m`. Adding one altitude quantum (1 m) plus the peak-drop guard (2 m) gives
`13 + 1 + 2 = 16 m`, one count above the shipped 15 m; using the bare time-constant figure plus the
2 m guard gives exactly `13 + 2 = 15 m`. So the shipped value is consistent with "one inflation time
constant plus a 2 m guard" and is `15/13 = 1.15×` the bare inflation distance. Also `15 m = 15 LSB
= 5 × climb_confirm`, and `15 < 30` keeps it below the apogee floor.

**Status.** REQUIREMENT, conditional on the placeholder canopy parameters. **NO SOURCE** for 15 m:
there is no measured inflation time, no measured main descent rate and no landing-speed limit.
**Missing measurement:** a signed drop test giving inflation time and settled main descent, plus the
maximum acceptable touchdown speed. With a first-order model, one `τ` is not fully inflated: a
3τ criterion would need `3 × 13 = 39 m`, so the shipped 15 m has a much thinner margin than the
arithmetic above suggests. This is a gap to close, not a hidden justification.

## 5. Altitude floor — `min_deploy_altitude_m = 3`

**Requirement.** Suppress the fast-descent trigger below a hard floor: a release that cannot inflate
before ground is not worth commanding, and the last metres must not be owned by the emergency path.
The slow-descent low-altitude backup remains active down to the ground.

**Assumptions.** `0 <= floor < low` is enforced (`deployment.rs:70-74`); floor is in whole metres.

**Arithmetic.**
`3 m / 6.5 m/s = 0.46 s` of flight time to ground at the placeholder settled main rate — far below
the 2.0 s inflation time constant, so a fast-descent release below 3 m cannot produce an inflated
canopy before touchdown. `3 m = 3 LSB` and `3 < 15`, so the low-altitude backup still covers the
final metres. At the slowest gated descent (−0.5 m/s, `deployment_tests.rs:214-223`) 3 m gives 6 s,
but that case is handled by `low_altitude_m` anyway.

**Status.** REQUIREMENT. **NO SOURCE** for 3 m. **Missing measurement:** the minimum altitude at
which the real canopy inflates within the required landing-speed limit (same signed drop-test
record as §3–§4). The value is a 3-count clearance chosen to be below the 15 m backup; no hardware
fact fixes it as exactly 3.

---

## Summary: what is closed and what is not

| Field | Value | Requirement | Arithmetic basis | Evidence |
|---|---|---|---|---|
| `descent_confirm` | 3 | reject multi-sample glitches | 6 ms at 500 Hz; 0.15 m at 25 m/s | NO SOURCE — glitch statistics |
| `apogee_min_altitude_m` | 30 | no release near pad | 30 LSB = 2×15 m; ≥0.6 s to backup band | NO SOURCE — flight alt trace |
| `apogee_drop_m` | 2 | exceed 1 altitude count | 2 LSB; 2 s at 1 m/s, 80 ms at 25 m/s | NO SOURCE — baro sigma at apogee |
| `climb_confirm_m` | 5 | verified climb | 5 LSB; redundant below the 30 m floor | NO SOURCE — pad transient data |
| `fast_descent_cm_s` | −1500 | faster than controlled descent | 2.31× main, 0.60× drogue (placeholders) | NO SOURCE — signed drop test |
| `low_altitude_m` | 15 | altitude for inflation | 13 m (τ×v) + 2 m guard | NO SOURCE — drop test, landing limit |
| `min_deploy_altitude_m` | 3 | suppress too-late release | 0.46 s to ground ≪ 2 s τ | NO SOURCE — drop test |

**Closed:** DEP-1 and DEP-4 no longer say "no derivation". Each value now has an explicit
requirement, assumptions and arithmetic, and each names the missing measurement.

**Not closed:** none of the seven values is validated against hardware. The ledger status moves
from `NO SOURCE` to `IN-TREE (REQUIREMENT)` because the authority is this in-tree derivation;
`NO SOURCE` still applies to every numeric value because no external standard, flight log or
signed drop test exists. To close that part, record the owner-signed flight thresholds in the empty
`docs/HIL_CHECKLIST.md:228` (D06) evidence column and attach the signed canopy drop-test values
defined at `sim/core/src/parachute.rs:145-174`.
