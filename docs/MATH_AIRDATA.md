# Air-data Mach: isentropic Pitot relation

## Scope

This document describes the arithmetic in `firmware/src/shield_sample.rs`. It
is a host-verifiable math contract only: no sensor, pin, calibration, range,
accuracy, or flight behavior is claimed here. The airdata seam reports
`Unavailable` when a driver is absent, so this math never stands in for a
measurement.

## Relation

For subsonic, isentropic flow the pitot measures stagnation (total) pressure
`p_t`, the barometer measures static pressure `p`, and their difference is the
impact pressure `q_c`:

```
q_c = p_t - p = dp
q_c / p = (1 + (gamma - 1)/2 * M^2)^(gamma / (gamma - 1)) - 1
```

with `gamma = 1.4` for air. This is the compressible Pitot formula in Anderson,
*Fundamentals of Aerodynamics* (isentropic impact-pressure relation).

The retired seam used the incompressible result `q = rho V^2 / 2` and
`a^2 = gamma p / rho`, giving `M^2 = 2 dp / (gamma p)`. It over-reads: at the
dp that is truly M = 0.6 (27 915 Pa at sea level) the old formula returns
0.6273, 4.6% high. The error grows with Mach (0.1001 vs 0.1000 at M = 0.1;
0.9938 vs 0.9000 at M = 0.9).

## Solver (no libm, deterministic)

The exponent `gamma/(gamma-1) = 3.5` is fractional. Squaring the relation
removes it and gives a monotone equation in `x = M^2`:

```
(1 + 0.2 x)^7 = (1 + dp/p)^2        for x in [0, 1]  (M < 1)
```

`mach_from_pitot(dp, p)`:

1. Refuses non-finite and non-positive inputs.
2. Computes `target = (1 + dp/p)^2` and refuses it when it is non-finite or
   `>= 1.2^7`, the target at M = 1. There is no subsonic solution beyond this
   point; the function returns `MachError::AboveSubsonicLimit` rather than
   clamping to M = 1.
3. Bisects `[0, 1]` on `x` with a **fixed 32 steps**, comparing `u^7` computed
   by three multiplications against `target`. Positive `u` keeps the map
   monotone, so rounding cannot make the search oscillate; a fixed count makes
   the result bit-reproducible.
4. Takes one bit-trick seed (`f32::from_bits((bits >> 1) + 0x1fc0_0000)`)
   refined by four Newton steps to get `M = sqrt(x)`.

No `powf`, `exp`, `log`, `sqrtf` or other `libm` call is involved. All
intermediate values stay near 1–4, so there is no overflow in the airdata
range.

A ratio at or above the M = 1 limit is a **counted, loud refusal**: the sampler
returns `ShieldSample::UNAVAILABLE` (so the shield vetoes with `PitotInvalid`)
and increments `UnavailableCounts::above_subsonic`. It never emits a clamped
M = 1 sample that would look like a healthy fast-air reading.

## Trusted table

At `p = 101325 Pa`, dp in Pa rounded to whole values from the f64 relation;
tolerances in `firmware/tests/mach_math_tests.rs` allow for the rounding.

| M | dp (Pa) | q_c/p | retired incompressible M |
|---|---------|-------|--------------------------|
| 0.1 | 711 | 0.007018 | 0.1001 |
| 0.3 | 6 528 | 0.064426 | 0.3034 |
| 0.6 | 27 915 | 0.275500 | 0.6274 |
| 0.9 | 70 046 | 0.691300 | 0.9938 |

M = 1 boundary: `q_c/p = 1.2^3.5 - 1 = 0.892929` (dp = 90 476 Pa at sea
level). Above this the subsonic solver refuses; transonic/supersonic impact
pressure obeys a different (normal-shock) relation that is not modeled here.

## Verification

`firmware/tests/mach_math_tests.rs` pins the table, monotonicity in both
pressures, zero/negative/non-finite refusal, bit-for-bit determinism, and the
loud above-M = 1 refusal with its own counter. The same arithmetic is checked
under `thumbv7em-none-eabihf` with `cargo check`; those are compile checks, not
hardware measurements.
