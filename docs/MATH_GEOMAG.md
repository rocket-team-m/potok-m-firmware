# Geomagnetic north: sourced WMM2025 model

**Date:** 2026-11-24 · **Repo:** `/home/sehaxe/potok-m-firmware` · **Code:** `firmware/src/geomag.rs`, declared at `firmware/src/lib.rs:20` · **Gates:** `firmware/tests/geomag_tests.rs`

**Decision:** WMM2025 is feasible on this target. It is implemented rather than replaced with a ground-station declination input. The full degree/order-12 model is 360 published numbers (~2.9 KB in `.rodata`) and ~4 KB of fixed-size stack arrays; there is no allocation, no `libm` dependency and no dynamic state.

## Authority

| Item | Exact identifier | How it was verified |
|---|---|---|
| Model values | NOAA NCEI Geomagnetic Modeling Team; British Geological Survey, *World Magnetic Model 2025*, **DOI 10.25921/aqfd-sd83** | Citation and DOI text opened on the NCEI WMM product page `https://www.ncei.noaa.gov/products/world-magnetic-model` (2026-11-24) |
| Technical report | Chulliat, Brown, Nair, Gomez Perez, Young, Watson, Boneh, Beggan, Meyer, Paniccia (2025), *The US/UK World Magnetic Model for 2025–2030: Technical Report*, **DOI 10.25923/prbc-s316** | Citation and DOI opened on the same product page; that page points to report §1.2 for the Gauss coefficients. The PDF itself was **not** opened here, so no equation number is claimed from it |
| Coefficient file | `WMM2025.COF` inside `https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip`; header `2025.0  WMM-2025  11/13/2024` | Archive downloaded; `sha256(WMM2025.COF) = dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f`. 90 coefficient rows, two `9999…` terminators |
| Test values | `WMM2025_TestValues.txt` in the same archive | `sha256 = e6975b093dddeb6153e0b23cc418425c438167e7c5b1dd795da379cb654f5819`. 100 rows, columns: year, HAE km, geodetic lat, geodetic lon, D, I, H, X, Y, Z, F and six annual changes |
| Reference algorithm | NOAA public-domain `GeomagnetismLibrary.c` (`WMM_Summation`, geoid/rotation helpers) | The synthesis here mirrors that algorithm as transcribed in two independently published ports (`rubenCodeforges/ardudeck` `utils/wmm.ts`, `jonnymaserati/welleng` `geomag.py`), both of which check against the NOAA test table. Correctness here is gated by the NOAA table, not by those ports |

The US Government places WMM software and coefficients in the public domain (17 U.S.C. 403); no license fee or permission is needed, but the source must be named. **No coefficient in `geomag.rs` was invented, fitted, rounded from memory or taken from a third-party copy**: all four arrays were generated mechanically from the downloaded `WMM2025.COF` rows `n m g h gdot hdot`, indexed at `n*(n+1)/2 + m` with index 0 unused.

## Formula

Let `a = 6378.137 km`, `b = 6356.7523142 km`, `e² = 1 − (b/a)²`, reference radius `R = 6371.2 km`, epoch `t₀ = 2025.0`.

1. **Time adjustment.** `g_n^m(t) = g_n^m(t₀) + (t − t₀) ġ_n^m`, likewise `h_n^m`.
2. **Geodetic → geocentric** (HAE height `H` above WGS84):
   `r_c = a / √(1 − e² sin²φ)`, `x_p = (r_c + H) cosφ`, `z_p = (r_c(1 − e²) + H) sinφ`,
   `r = √(x_p² + z_p²)`, `sin φ_c = z_p/r`. The code keeps `sin φ_c` and `cos φ_c = √(1 − sin²φ_c)` directly; no `asin` is needed.
3. **Schmidt semi-normalised Legendre functions** at `u = sin φ_c`, `s = cos φ_c` via the standard NOAA recursion (`P_n^n = s·P_{n−1}^{n−1}`, `P_n^0 = u·P_{n−1}^0 − k·P_{n−2}^0`, `k = ((n−1)² − m²)/((2n−1)(2n−3))`), scaled by `S_n^0 = S_{n−1}^0(2n−1)/n`, `S_n^m = S_n^{m−1}√((n−m+1)·(m=1 ? 2 : 1)/(n+m))`.
4. **Synthesis**, `q = (R/r)²` and `q_n = q^(n/2)·q`:
   `Z = −Σ_{n=1}^{12} q_n (n+1) (g cos mλ + h sin mλ) P_n^m`,
   `Y = Σ q_n m (−g sin mλ + h cos mλ) P_n^m / cos φ_c`,
   `X = Σ q_n (g cos mλ + h sin mλ) ∂P_n^m/∂θ`.
   At `cos φ_c → 0` the east component is 0/0; the NOAA dedicated pole series is used instead of dividing by zero.
5. **Frame rotation.** `ψ = φ_c − φ`; `(X_g, Z_g)` are rotated by `ψ` into geodetic north/down. Then `H = √(X²+Y²)`, `F = √(H²+Z²)`, `D = atan2(Y, X)` (east positive), `I = atan2(Z, H)` (down positive).

`Field::magnetic_north_unit()` returns `[cos D, sin D, 0]` in the z-up world frame of `attitude.rs` (e1 north, e2 east, e3 up) — the sourced quantity that replaces a hard-coded `+e1`. Near a magnetic pole `H → 0` and declination is undefined; check `horizontal_nt` before treating it as a heading.

## Why the local polynomials

Rust `core` provides none of `sin`, `cos`, `atan`, `atan2`, `sqrt` and this crate has no math dependency. `sqrt` is a bit-seed plus six Newton steps; `sin_cos` range-reduces to `[−π/4, π/4]` then uses a degree-13/14 Taylor polynomial; `atan` uses four half-angle reductions followed by the odd series to `t¹⁵`. These are fixed-order and deterministic, the same approach already used by `shield_sample.rs`'s hand-rolled `sqrt`. Bounds are below f64 epsilon on the reduced intervals; the empirical gate is the NOAA table below.

## Verification

- All **100** published NOAA rows were evaluated with a host harness during development. Worst absolute component error: **0.00072 nT** (X), 0.00070 nT (H), 0.00043 nT (F), all below NOAA's 0.1 nT numerical-agreement specification. D/I agree within 0.005°, which is the rounding of the published two-decimal angles.
- Six official rows are pinned as a regression gate in `geomag_tests.rs`, covering both hemispheres, the equator, latitude 87°, dates 2025.0/2026.5/2027.0/2029.0 and altitudes 18–65 km.
- `cargo test -p potok-m-firmware --test geomag_tests`: **5 passed, 0 failed**.
- `cargo check -p potok-m-firmware --lib --target thumbv7em-none-eabihf`: clean. The module contains no `Vec`, `Box`, `String`, floating-point environment access or host syscall.

## Remaining limitations

1. **Not wired into the flying filter.** `attitude.rs:19-25` and `:284-288` still take world north as `+e1`; this change only adds the sourced model and its unit-vector accessor. Wiring belongs with a field-calibration decision (item 2) and a new A/B gate; until then the MAH-6 row of `MATH_PROVENANCE.md` correctly still describes the flight path as unsourced.
2. **No magnetometer calibration.** WMM gives the *main* field only (~95% of the surface field). Hard-iron/soft-iron offsets, local ferrous distortion, crustal anomalies and space-weather disturbance are outside the model. A field-calibration table is still required before the corrected vector can be trusted as a heading.
3. **Validity window.** NOAA accepts 2025.0–2030.0 only; `evaluate` refuses anything else loudly (`Error::OutOfWindow`). The model must be replaced at the next five-year release.
4. **Cost is not measured on hardware.** The evaluator is intended for a ground-station/startup computation, not the 500 Hz tick. No cycle count or stack watermark has been taken on the STM32H723.

## Provenance rows (new family; not yet folded into the consolidated counts)

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| GEO-1 | 90×4 Gauss + secular-variation coefficients, epoch 2025.0, degree/order 12 | `geomag.rs:40-78` | NOAA NCEI WMM2025 `WMM2025.COF`, DOI 10.25921/aqfd-sd83 | SOURCE (FORMULA) | `geomag_tests.rs::coefficients_are_the_official_cof_endpoints` pins first/last rows | PDF §1.2 not opened; no equation number claimed |
| GEO-2 | WGS84 geodetic→geocentric, Schmidt-normalised SH synthesis, geodetic rotation, D/I/H/X/Y/Z/F | `geomag.rs:135-266` | WMM technical report DOI 10.25923/prbc-s316; NOAA `GeomagnetismLibrary.c` algorithm | SOURCE (FORMULA) | 6 NOAA rows in `wmm2025_matches_published_noaa_test_values_at_six_points`; all 100 checked in development | Poles and distortion fields outside the model |
| GEO-3 | Local `sqrt`/`sin_cos`/`atan`/`atan2` polynomials | `geomag.rs:268-365` | — | IN-TREE (CHOICE) | indirect via GEO-2 against NOAA values | No formal ULP bound; fixed-order series |
| GEO-4 | Magnetic-north unit `[cos D, sin D, 0]` in the z-up world frame | `geomag.rs:100-110` | Angle convention from WMM2025 (D east of true north); frame from `attitude.rs` docs | SOURCE (FORMULA) | `evaluation_is_pure_and_north_unit_has_no_vertical_component` | Undefined where H→0; not yet consumed by `attitude.rs` |
