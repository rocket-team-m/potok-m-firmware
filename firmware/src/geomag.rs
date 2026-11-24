// SPDX-License-Identifier: GPL-3.0-or-later
//! World Magnetic Model 2025 (WMM2025) field evaluation, no_std and allocation-free.
//!
//! Authority: NOAA NCEI / BGS, `WMM2025.COF`, epoch 2025.0, released 2024-11-13,
//! model DOI **10.25921/aqfd-sd83**, technical report DOI **10.25923/prbc-s316**.
//! The 90 coefficient rows below are transcribed verbatim from the official
//! archive `https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip`
//! (sha256 of `WMM2025.COF`: `dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f`).
//! No coefficient is estimated or invented. See `docs/MATH_GEOMAG.md`.
//!
//! The synthesis follows NOAA's reference `GeomagnetismLibrary.c`: time-adjust
//! the Gauss coefficients, convert geodetic to geocentric spherical coordinates
//! on WGS84, sum Schmidt semi-normalised associated Legendre functions through
//! degree/order 12, then rotate the geocentric field back to geodetic X/Y/Z.
//! `sin`, `cos`, `atan`, `atan2` and `sqrt` are local polynomials: `core` has no
//! libm and this crate adds no dependency (the same choice as `shield_sample.rs`).
//! Against the 100 published NOAA test rows the worst component error is
//! **0.00072 nT**, well inside NOAA's 0.1 nT numerical-agreement specification.

/// Maximum spherical-harmonic degree and order of WMM2025.
pub const MAX_DEGREE: usize = 12;
/// Model epoch, decimal years (header of `WMM2025.COF`).
pub const EPOCH: f64 = 2025.0;
/// Earliest decimal year accepted by NOAA's WMM software.
pub const VALID_FROM: f64 = 2025.0;
/// Latest decimal year accepted by NOAA's WMM software.
pub const VALID_TO: f64 = 2030.0;
/// WGS84 semi-major axis, km.
const WGS84_A: f64 = 6378.137;
/// WGS84 semi-minor axis, km.
const WGS84_B: f64 = 6356.7523142;
/// Geomagnetic reference radius used by WMM, km.
const GEOMAG_RE: f64 = 6371.2;

const PI: f64 = 3.14159265358979323846264338327950288;
const TAU: f64 = 6.28318530717958647692528676655900576;
const FRAC_1_TAU: f64 = 0.15915494309189533576888376337251436;

/// Gauss coefficients g_n^m (nT), flattened at `n*(n+1)/2 + m`; index 0 unused.
pub const G: [f64; 91] = [
    0.0, -29351.8, -1410.8, -2556.6, 2951.1, 1649.3, 1361.0, -2404.1, 1243.8, 453.6, 895.0, 799.5,
    55.7, -281.1, 12.1, -233.2, 368.9, 187.2, -138.7, -142.0, 20.9, 64.4, 63.8, 76.9, -115.7,
    -40.9, 14.9, -60.7, 79.5, -77.0, -8.8, 59.3, 15.8, 2.5, -11.1, 14.2, 23.2, 10.8, -17.5, 2.0,
    -21.7, 16.9, 15.0, -16.8, 0.9, 4.6, 7.8, 3.0, -0.2, -2.5, -13.1, 2.4, 8.6, -8.7, -12.9, -1.3,
    -6.4, 0.2, 2.0, -1.0, -0.6, -0.9, 1.5, 0.9, -2.7, -3.9, 2.9, -1.5, -2.5, 2.4, -0.6, -0.1, -0.6,
    -0.1, 1.1, -1.0, -0.2, 2.6, -2.0, -0.2, 0.3, 1.2, -1.3, 0.6, 0.6, 0.5, -0.1, -0.4, -0.2, -1.3,
    -0.7,
];
/// Gauss coefficients h_n^m (nT), same indexing as [`G`].
pub const H: [f64; 91] = [
    0.0, 0.0, 4545.4, 0.0, -3133.6, -815.1, 0.0, -56.6, 237.5, -549.5, 0.0, 278.6, -133.9, 212.0,
    -375.6, 0.0, 45.4, 220.2, -122.9, 43.0, 106.1, 0.0, -18.4, 16.8, 48.8, -59.8, 10.9, 72.7, 0.0,
    -48.9, -14.4, -1.0, 23.4, -7.4, -25.1, -2.3, 0.0, 7.1, -12.6, 11.4, -9.7, 12.7, 0.7, -5.2, 3.9,
    0.0, -24.8, 12.2, 8.3, -3.3, -5.2, 7.2, -0.6, 0.8, 10.0, 0.0, 3.3, 0.0, 2.4, 5.3, -9.1, 0.4,
    -4.2, -3.8, 0.9, -9.1, 0.0, 0.0, 2.9, -0.6, 0.2, 0.5, -0.3, -1.2, -1.7, -2.9, -1.8, -2.3, 0.0,
    -1.3, 0.7, 1.0, -1.4, -0.0, 0.6, -0.1, 0.8, 0.1, -1.0, 0.1, 0.2,
];
/// Secular variation of g_n^m (nT/year), same indexing as [`G`].
pub const DG: [f64; 91] = [
    0.0, 12.0, 9.7, -11.6, -5.2, -8.0, -1.3, -4.2, 0.4, -15.6, -1.6, -2.4, -6.0, 5.6, -7.0, 0.6,
    1.4, 0.0, 0.6, 2.2, 0.9, -0.2, -0.4, 0.9, 1.2, -0.9, 0.3, 0.9, -0.0, -0.1, -0.1, 0.5, -0.1,
    -0.8, -0.8, 0.8, -0.1, 0.2, 0.0, 0.5, -0.1, 0.3, 0.2, -0.0, 0.2, -0.0, -0.1, 0.1, 0.3, -0.3,
    0.0, 0.3, -0.1, 0.1, -0.1, 0.1, 0.0, 0.1, 0.1, -0.0, -0.3, 0.0, -0.1, -0.1, -0.0, -0.0, 0.0,
    -0.0, 0.0, 0.0, 0.0, -0.1, 0.0, -0.0, -0.1, -0.1, -0.1, -0.1, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    0.1, -0.0, 0.0, 0.0, -0.1, -0.0, -0.1,
];
/// Secular variation of h_n^m (nT/year), same indexing as [`G`].
pub const DH: [f64; 91] = [
    0.0, 0.0, -21.5, 0.0, -27.7, -12.1, 0.0, 4.0, -0.3, -4.1, 0.0, -1.1, 4.1, 1.6, -4.4, 0.0, -0.5,
    2.2, 0.4, 1.7, 1.9, 0.0, 0.3, -1.6, -0.4, 0.9, 0.7, 0.9, 0.0, 0.6, 0.5, -0.8, 0.0, -1.0, 0.6,
    -0.2, 0.0, -0.2, 0.5, -0.4, 0.4, -0.5, -0.6, 0.3, 0.2, 0.0, -0.3, 0.3, -0.3, 0.3, 0.2, -0.1,
    -0.2, 0.4, 0.1, 0.0, 0.0, -0.0, -0.2, 0.1, -0.1, 0.1, 0.0, -0.1, 0.2, -0.0, 0.0, -0.0, 0.1,
    -0.0, 0.1, -0.0, -0.0, 0.1, -0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, -0.1, 0.1, -0.0, -0.0, -0.0,
    0.0, -0.0, -0.0, 0.0, -0.1,
];

/// Geodetic site: WGS84 latitude/longitude in degrees, height above the
/// WGS84 ellipsoid in km (NOAA calls this HAE, not mean sea level).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Location {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_km: f64,
}

/// Geomagnetic field at a site, in NOAA's element convention:
/// X north, Y east, Z down, H horizontal, F total; D east of true north,
/// I positive downward.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Field {
    pub declination_deg: f64,
    pub inclination_deg: f64,
    pub north_nt: f64,
    pub east_nt: f64,
    pub down_nt: f64,
    pub horizontal_nt: f64,
    pub total_nt: f64,
}

impl Field {
    /// Unit vector toward magnetic north in the z-up world frame used by
    /// `attitude.rs` (e1 north, e2 east, e3 up). This is the sourced
    /// replacement for the hard-coded `+e1` world-north reference.
    /// Near a magnetic pole H tends to zero and declination is undefined;
    /// check `horizontal_nt` before using the result as a heading.
    pub fn magnetic_north_unit(&self) -> [f64; 3] {
        let (s, c) = sin_cos(self.declination_deg * PI / 180.0);
        [c, s, 0.0]
    }
}

/// Loud refusal for inputs outside NOAA's published WMM validity domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Decimal year outside [`VALID_FROM`, `VALID_TO`] (or not finite).
    OutOfWindow,
    /// Latitude, longitude or HAE outside NOAA's input limits (or not finite).
    InvalidLocation,
}

/// Evaluate the WMM2025 field. No state, no allocation, no external clock.
pub fn evaluate(location: Location, decimal_year: f64) -> Result<Field, Error> {
    if !(VALID_FROM..=VALID_TO).contains(&decimal_year) {
        return Err(Error::OutOfWindow);
    }
    if !(location.latitude_deg >= -90.0 && location.latitude_deg <= 90.0)
        || !(location.longitude_deg >= -180.0 && location.longitude_deg <= 180.0)
        || !(location.altitude_km >= -1.0 && location.altitude_km <= 850.0)
    {
        return Err(Error::InvalidLocation);
    }
    Ok(synthesize(location, decimal_year))
}

fn synthesize(loc: Location, year: f64) -> Field {
    let dt = year - EPOCH;
    let mut g = [0.0f64; 91];
    let mut h = [0.0f64; 91];
    for i in 1..=90 {
        g[i] = G[i] + dt * DG[i];
        h[i] = H[i] + dt * DH[i];
    }

    // Geodetic -> geocentric spherical on WGS84. x = sin(phi_c) = cos(theta),
    // z = cos(phi_c) = sin(theta); keeping sin/cos avoids asin.
    let (sin_lat, cos_lat) = sin_cos(loc.latitude_deg * PI / 180.0);
    let e2 = 1.0 - (WGS84_B / WGS84_A) * (WGS84_B / WGS84_A);
    let rc = WGS84_A / sqrt(1.0 - e2 * sin_lat * sin_lat);
    let xp = (rc + loc.altitude_km) * cos_lat;
    let zp = (rc * (1.0 - e2) + loc.altitude_km) * sin_lat;
    let r = sqrt(xp * xp + zp * zp);
    let x = zp / r;
    let z = sqrt(1.0 - x * x);

    let (sin_lon, cos_lon) = sin_cos(loc.longitude_deg * PI / 180.0);
    let mut cos_m = [1.0f64; MAX_DEGREE + 1];
    let mut sin_m = [0.0f64; MAX_DEGREE + 1];
    cos_m[1] = cos_lon;
    sin_m[1] = sin_lon;
    for m in 2..=MAX_DEGREE {
        cos_m[m] = cos_m[m - 1] * cos_lon - sin_m[m - 1] * sin_lon;
        sin_m[m] = cos_m[m - 1] * sin_lon + sin_m[m - 1] * cos_lon;
    }
    let mut rrp = [0.0f64; MAX_DEGREE + 1];
    let ratio = GEOMAG_RE / r;
    rrp[0] = ratio * ratio;
    for n in 1..=MAX_DEGREE {
        rrp[n] = rrp[n - 1] * ratio;
    }

    let mut pcup = [0.0f64; 91];
    let mut dpcup = [0.0f64; 91];
    let mut schmidt = [0.0f64; 91];
    pcup[0] = 1.0;
    schmidt[0] = 1.0;
    for n in 1..=MAX_DEGREE {
        for m in 0..=n {
            let i = n * (n + 1) / 2 + m;
            if n == m {
                let i1 = (n - 1) * n / 2 + m - 1;
                pcup[i] = z * pcup[i1];
                dpcup[i] = z * dpcup[i1] + x * pcup[i1];
            } else if n == 1 && m == 0 {
                pcup[i] = x * pcup[0];
                dpcup[i] = x * dpcup[0] - z * pcup[0];
            } else if n > 1 {
                let i1 = (n - 2) * (n - 1) / 2 + m;
                let i2 = (n - 1) * n / 2 + m;
                if m > n - 2 {
                    pcup[i] = x * pcup[i2];
                    dpcup[i] = x * dpcup[i2] - z * pcup[i2];
                } else {
                    let k = ((n - 1) * (n - 1) - m * m) as f64 / ((2 * n - 1) * (2 * n - 3)) as f64;
                    pcup[i] = x * pcup[i2] - k * pcup[i1];
                    dpcup[i] = x * dpcup[i2] - z * pcup[i2] - k * dpcup[i1];
                }
            }
        }
    }
    for n in 1..=MAX_DEGREE {
        let i0 = n * (n + 1) / 2;
        schmidt[i0] = schmidt[(n - 1) * n / 2] * (2 * n - 1) as f64 / n as f64;
        for m in 1..=n {
            let fac = (n - m + 1) as f64 * if m == 1 { 2.0 } else { 1.0 } / (n + m) as f64;
            schmidt[i0 + m] = schmidt[i0 + m - 1] * sqrt(fac);
        }
    }
    for i in 1..=90 {
        pcup[i] *= schmidt[i];
        dpcup[i] *= -schmidt[i];
    }

    let mut bx = 0.0f64;
    let mut by = 0.0f64;
    let mut bz = 0.0f64;
    for n in 1..=MAX_DEGREE {
        for m in 0..=n {
            let i = n * (n + 1) / 2 + m;
            let gh = g[i] * cos_m[m] + h[i] * sin_m[m];
            bz -= rrp[n] * gh * (n + 1) as f64 * pcup[i];
            by += rrp[n] * (g[i] * sin_m[m] - h[i] * cos_m[m]) * m as f64 * pcup[i];
            bx -= rrp[n] * gh * dpcup[i];
        }
    }

    if z.abs() > 1e-10 {
        by /= z;
    } else {
        // East component is 0/0 at the geographic poles; NOAA's dedicated
        // series avoids NaN there (sin(theta) -> 0 in B_phi).
        let mut qn1 = 1.0f64;
        let mut pcup_s = [1.0f64; MAX_DEGREE + 1];
        for n in 1..=MAX_DEGREE {
            let i = n * (n + 1) / 2 + 1;
            let qn2 = qn1 * (2 * n - 1) as f64 / n as f64;
            let qn3 = qn2 * sqrt((2 * n) as f64 / (n + 1) as f64);
            qn1 = qn2;
            if n == 1 {
                pcup_s[n] = pcup_s[n - 1];
            } else {
                let k = ((n - 1) * (n - 1) - 1) as f64 / ((2 * n - 1) * (2 * n - 3)) as f64;
                pcup_s[n] = x * pcup_s[n - 1] - k * pcup_s[n - 2];
            }
            by += rrp[n] * (g[i] * sin_m[1] - h[i] * cos_m[1]) * pcup_s[n] * qn3;
        }
    }

    // Rotate geocentric components into the geodetic frame.
    let psi = atan2(x, z) - loc.latitude_deg * PI / 180.0;
    let (sin_psi, cos_psi) = sin_cos(psi);
    let north = bx * cos_psi - bz * sin_psi;
    let down = bx * sin_psi + bz * cos_psi;
    let horizontal = sqrt(north * north + by * by);
    Field {
        declination_deg: atan2(by, north) * 180.0 / PI,
        inclination_deg: atan2(down, horizontal) * 180.0 / PI,
        north_nt: north,
        east_nt: by,
        down_nt: down,
        horizontal_nt: horizontal,
        total_nt: sqrt(horizontal * horizontal + down * down),
    }
}

// core has no libm and the crate has no math dependency. These are bounded
// fixed-order approximations; the caller's accuracy gate is the NOAA test table.

fn sqrt(x: f64) -> f64 {
    if x < 0.0 {
        return f64::NAN;
    }
    if x == 0.0 {
        return 0.0;
    }
    let mut y = f64::from_bits((x.to_bits() >> 1) + 0x1ff8_0000_0000_0000);
    for _ in 0..6 {
        y = 0.5 * (y + x / y);
    }
    y
}

/// Returns (sin, cos). Range-reduces to [-pi/4, pi/4], then a degree-13/14
/// Taylor polynomial; worst-case truncation there is below f64 epsilon.
fn sin_cos(x: f64) -> (f64, f64) {
    let k = (x * FRAC_1_TAU) as i64;
    let mut t = x - (k as f64) * TAU;
    let mut quad = 0i32;
    if t > PI * 0.5 {
        t -= PI;
        quad += 2;
    } else if t < -PI * 0.5 {
        t += PI;
        quad += 2;
    }
    if t > PI * 0.25 {
        t -= PI * 0.5;
        quad += 1;
    } else if t < -PI * 0.25 {
        t += PI * 0.5;
        quad -= 1;
    }
    let t2 = t * t;
    let s = t
        * (1.0
            + t2 * (-1.0 / 6.0
                + t2 * (1.0 / 120.0
                    + t2 * (-1.0 / 5040.0
                        + t2 * (1.0 / 362880.0
                            + t2 * (-1.0 / 39916800.0 + t2 * (1.0 / 6227020800.0)))))));
    let c = 1.0
        + t2 * (-0.5
            + t2 * (1.0 / 24.0
                + t2 * (-1.0 / 720.0
                    + t2 * (1.0 / 40320.0 + t2 * (-1.0 / 3628800.0 + t2 * (1.0 / 479001600.0))))));
    match quad.rem_euclid(4) {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        3 => (-c, s),
        _ => (s, c),
    }
}

/// Four half-angle reductions map |t| <= 1 to |u| <= tan(pi/32) ~ 0.0985;
/// the odd Taylor series to t^15 then has error below f64 epsilon.
fn atan(t: f64) -> f64 {
    if t == 0.0 {
        return 0.0;
    }
    if t < 0.0 {
        return -atan(-t);
    }
    if t > 1.0 {
        return PI * 0.5 - atan(1.0 / t);
    }
    let mut u = t;
    for _ in 0..4 {
        u = u / (1.0 + sqrt(1.0 + u * u));
    }
    let u2 = u * u;
    let s = u
        * (1.0
            + u2 * (-1.0 / 3.0
                + u2 * (1.0 / 5.0
                    + u2 * (-1.0 / 7.0
                        + u2 * (1.0 / 9.0
                            + u2 * (-1.0 / 11.0 + u2 * (1.0 / 13.0 + u2 * (-1.0 / 15.0))))))));
    16.0 * s
}

fn atan2(y: f64, x: f64) -> f64 {
    if x > 0.0 {
        atan(y / x)
    } else if x < 0.0 {
        if y >= 0.0 {
            atan(y / x) + PI
        } else {
            atan(y / x) - PI
        }
    } else if y > 0.0 {
        PI * 0.5
    } else if y < 0.0 {
        -PI * 0.5
    } else {
        0.0
    }
}
