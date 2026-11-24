# Redundant-sensor voting — method, units and evidence

## Source and scope

**Fault-detection reference.** R. Isermann, *Fault-Diagnosis Systems: An
Introduction from Fault Detection to Fault Tolerance*, Springer (2006),
**DOI 10.1007/3-540-30368-5** (confirmed via the DOI landing page). The method
taken from this reference is the general analytic-redundancy scheme: redundant
measurements of one quantity are compared, a **residual** is formed, the
residual is checked against a **threshold** to give a fault indication, and a
third redundant channel is used for **isolation** by consensus. No chapter,
page or equation number is claimed: this is a method attribution, not a
transcription of a specific formula, and no hardware or flight result is
claimed here. Everything beyond that scheme — the medoid selection, the stuck
counter, the confidence shape and every numeric threshold — is this
repository's own engineering and is labelled as such below.

Implementation: `firmware/src/sensor_voting.rs`. Gates:
`firmware/tests/sensor_voting_tests.rs` (existing behaviour) and
`firmware/tests/voting_math_tests.rs` (dimensional gates added with this
document). Host tests only; the voting module has no drivers.

## Residual and decision rule

For two readings `a`, `b` of the same physical quantity and a channel
threshold `T = (absolute, relative)`:

```
r(a,b) = |a - b| / (absolute + relative · max(|a|, |b|))
fault indication  <=>  r > 1
```

`r` is dimensionless. This is the residual-threshold step of the Isermann
scheme. With three channels, the pair with the smallest `r` is the working
consensus, the remaining channel is the candidate outlier (isolated only when
it also exceeds its threshold against the selected value), and the medoid
minimizing `Σ r` is the delivered value. With two channels the midpoint is
delivered and **no** outlier is named: two readings cannot arbitrate which
one is wrong. A slot stuck bit-identically for `stuck_samples` ticks is
quarantined only if a peer has moved beyond threshold in that same tick, so a
stationary vehicle does not quarantine its sensors.

## Units — the F2 fix

The retired code formed
`distance = max(gyro Δ [rad/s], accel Δ [m/s²])` and divided it by one limit
`absolute + relative · max(|gyro|, |accel|)`. That has two dimensional
defects:

1. rad/s and m/s² were compared by magnitude, which is not a physical
   comparison;
2. the relative part of the limit was fed by the *accelerometer's* static
   gravity (~9.81 m/s²) even for a gyro comparison. With the deployed IMU
   tolerance `Tolerance::new(0.05, 0.02)` that inflates the gyro limit by
   `0.02 · 9.81 = 0.196 rad/s`; a 0.10 rad/s gyro fault reads
   `0.10 / 0.246 = 0.406 < 1` and is missed.

The module now applies the residual per physical channel, each with its own
unit-bearing limit:

| channel | quantity | limit | constructor field |
|---|---|---|---|
| barometer / scalar | height, m | `absolute + relative·max(|a|,|b|)` m | `Tolerance::new` |
| `[f32; N]` vector | component unit | same per component | `Tolerance::new` |
| gyro (3 axes) | angular rate, rad/s | `gyro_absolute_rad_s + gyro_relative·max(|g_a|,|g_b|)` rad/s | `Tolerance::imu` |
| accel (3 axes) | specific force, m/s² | `accel_absolute_m_s2 + accel_relative·max(|a_a|,|a_b|)` m/s² | `Tolerance::imu` |

`Tolerance::imu(gyro_abs [rad/s], gyro_rel [–], accel_abs [m/s²], accel_rel
[–])` is the per-channel constructor. `Tolerance::new(a, r)` is retained for
scalar/vector channels and backwards compatibility: for an `ImuSample` it
reuses `a`/`r` in **each channel's own unit** (gyro rad/s, accel m/s²), so no
cross-unit comparison remains, but the *values* are then a single shared
number and remain HEURISTIC. Use `Tolerance::imu` when two noise floors are
known.

`firmware/src/lib.rs:352` still constructs the production IMU channel with
`Tolerance::new(0.05, 0.02)`; that file is owned by another owner and was not
edited in this task. Its semantics are now dimensionally correct (gyro
0.05 rad/s and accel 0.05 m/s², each with its own relative term), but the
numbers are **not re-derived**: replace them with `Tolerance::imu(...)` from
measured per-channel noise floors before a flight claim.

## Confidence weights — F3, HEURISTIC

| constant | value | status |
|---|---|---|
| `HEURISTIC_CONF_SINGLE` | 0.35 | HEURISTIC — no paper, calibration or flight datum |
| `HEURISTIC_CONF_DUAL` | 0.75 | HEURISTIC — same |
| `HEURISTIC_CONF_TRIPLE` | 1.0 | HEURISTIC — same |
| `HEURISTIC_DISAGREEMENT_PENALTY` | 0.8 | HEURISTIC — same |

`confidence = base · (1 − 0.8·clamp01(worst_r − 1))`; `confidence_u8()` is
only the protocol's byte rounding (`round(clamp01(c)·255)`), which is not a
source for the weights. A defensible alternative rule is "fraction of healthy
sensors" (`n/3`), but it is **not** what is implemented and is not claimed to
be equivalent; adopting it would change values pinned by
`sensor_voting_tests.rs` and needs a separate decision. These constants are
named `HEURISTIC_*` in code so the status cannot be lost. They are not a
safety argument.

## Deployment thresholds — F4, out of scope

- `Config::DEFAULT.stuck_samples = 8`: HEURISTIC, no noise-floor or flight
  datum fixes the count.
- IMU/barometer tolerances in `lib.rs:352-353` (`0.05/0.02`, `0.5/0.02`):
  HEURISTIC deployment assumptions, unchanged numerically here.
- `deployment.rs` parachute thresholds (`30/2/5/15/−1500/3/3`, header calls
  them "Verified"): unsupported by any named source and **not touched** by
  this task — `deployment.rs` is outside the assigned files. They stay
  HEURISTIC until a flight datum or standard is named.

## Gates added

`voting_math_tests.rs`:

- `gyro_only_fault_is_detected_when_accel_is_quiet` — a +0.30 rad/s gyro
  fault in a triple is flagged and isolated while all accel axes are
  bit-identical.
- `accel_only_fault_is_detected_when_gyro_is_quiet` — a +0.80 m/s² accel
  fault in a triple is flagged and isolated while all gyro axes are
  bit-identical.
- `single_mixed_threshold_cannot_pass` — reproduces the retired mixed
  expression and shows it misses the gyro fault at the deployed tolerance;
  shows that no absolute-only shared threshold simultaneously catches
  0.10 rad/s and ignores 0.25 m/s²; and shows only two independent
  unit-bearing thresholds pass all three cases.

These are host-math gates on the exact source compiled for the target. They
do not validate sensor noise floors, threshold choices or hardware behaviour.
