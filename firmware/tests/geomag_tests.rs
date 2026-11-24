// SPDX-License-Identifier: GPL-3.0-or-later
//! Gates for `geomag.rs`: published NOAA WMM2025 test values, coefficient
//! provenance, and loud refusals at the model's validity boundary.
//!
//! The vectors below are transcribed from `WMM2025_TestValues.txt` in the
//! official NOAA archive `WMM2025COF.zip`
//! (https://www.ncei.noaa.gov/sites/default/files/2024-12/WMM2025COF.zip).
//! Column order there is: year, HAE km, geodetic lat, geodetic lon, D, I,
//! H, X, Y, Z, F, then secular variations (not evaluated here).

use potok_m_firmware::geomag::{
    evaluate, Error, Field, Location, DG, DH, G, H, MAX_DEGREE, VALID_FROM, VALID_TO,
};

struct Noaa {
    year: f64,
    alt_km: f64,
    lat: f64,
    lon: f64,
    d: f64,
    i: f64,
    h: f64,
    x: f64,
    y: f64,
    z: f64,
    f: f64,
}

// Six official rows: both hemispheres, equator to 87 deg, three dates,
// altitudes 18..65 km, declination from -73.48 to +24.31 deg.
const NOAA_VALUES: [Noaa; 6] = [
    Noaa {
        year: 2025.0,
        alt_km: 65.0,
        lat: 43.0,
        lon: 93.0,
        d: 0.50,
        i: 64.10,
        h: 24300.764692,
        x: 24299.852822,
        y: 210.517066,
        z: 50037.923998,
        f: 55626.621348,
    },
    Noaa {
        year: 2025.0,
        alt_km: 51.0,
        lat: -33.0,
        lon: 109.0,
        d: -5.49,
        i: -67.50,
        h: 21838.046477,
        x: 21737.778822,
        y: -2090.274098,
        z: -52710.003920,
        f: 57054.752538,
    },
    Noaa {
        year: 2026.5,
        alt_km: 43.0,
        lat: 17.0,
        lon: 52.0,
        d: 1.19,
        i: 23.95,
        h: 36002.676553,
        x: 35994.907968,
        y: 747.876551,
        z: 15990.895993,
        f: 39394.180708,
    },
    Noaa {
        year: 2027.0,
        alt_km: 44.0,
        lat: -43.0,
        lon: -111.0,
        d: 24.31,
        i: -52.57,
        h: 22462.470699,
        x: 20471.522067,
        y: 9245.505620,
        z: -29347.556906,
        f: 36957.295441,
    },
    Noaa {
        year: 2029.0,
        alt_km: 50.0,
        lat: 87.0,
        lon: -154.0,
        d: -73.48,
        i: 89.07,
        h: 906.920459,
        x: 257.877420,
        y: -869.484879,
        z: 55992.308183,
        f: 55999.652503,
    },
    Noaa {
        year: 2025.0,
        alt_km: 18.0,
        lat: 0.0,
        lon: 21.0,
        d: 1.29,
        i: -26.06,
        h: 29282.246275,
        x: 29274.811882,
        y: 659.800118,
        z: -14316.722540,
        f: 32594.761714,
    },
];

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn check(v: &Noaa, got: Field) {
    // NOAA's own numerical-agreement spec is 0.1 nT; the implementation is
    // tighter, but D/I in the published table are rounded to 0.01 deg.
    assert!(
        close(got.north_nt, v.x, 0.01),
        "X {} vs {}",
        got.north_nt,
        v.x
    );
    assert!(
        close(got.east_nt, v.y, 0.01),
        "Y {} vs {}",
        got.east_nt,
        v.y
    );
    assert!(
        close(got.down_nt, v.z, 0.01),
        "Z {} vs {}",
        got.down_nt,
        v.z
    );
    assert!(
        close(got.horizontal_nt, v.h, 0.01),
        "H {} vs {}",
        got.horizontal_nt,
        v.h
    );
    assert!(
        close(got.total_nt, v.f, 0.01),
        "F {} vs {}",
        got.total_nt,
        v.f
    );
    assert!(
        close(got.declination_deg, v.d, 0.006),
        "D {} vs {}",
        got.declination_deg,
        v.d
    );
    assert!(
        close(got.inclination_deg, v.i, 0.006),
        "I {} vs {}",
        got.inclination_deg,
        v.i
    );
}

#[test]
fn wmm2025_matches_published_noaa_test_values_at_six_points() {
    for v in &NOAA_VALUES {
        let got = evaluate(
            Location {
                latitude_deg: v.lat,
                longitude_deg: v.lon,
                altitude_km: v.alt_km,
            },
            v.year,
        )
        .expect("official test point must be inside the validity window");
        check(v, got);
    }
}

#[test]
fn coefficients_are_the_official_cof_endpoints() {
    assert_eq!(MAX_DEGREE, 12);
    assert_eq!(VALID_FROM, 2025.0);
    assert_eq!(VALID_TO, 2030.0);
    assert_eq!(G.len(), 91);
    assert_eq!(G[0], 0.0);
    assert_eq!(G[1], -29351.8); // line " 1  0  -29351.8  0.0  12.0  0.0"
    assert_eq!(H[2], 4545.4); // line " 1  1   -1410.8  4545.4  9.7  -21.5"
    assert_eq!(DG[1], 12.0);
    assert_eq!(DH[2], -21.5);
    assert_eq!(G[90], -0.7); // last coefficient row, "12 12 -0.7 0.2 -0.1 -0.1"
    assert_eq!(H[90], 0.2);
    assert_eq!(DG[90], -0.1);
    assert_eq!(DH[90], -0.1);
}

#[test]
fn secular_variation_moves_the_field_between_years() {
    let loc = Location {
        latitude_deg: 43.0,
        longitude_deg: 93.0,
        altitude_km: 65.0,
    };
    let a = evaluate(loc, 2025.0).unwrap();
    let b = evaluate(loc, 2026.0).unwrap();
    assert!(!close(a.north_nt, b.north_nt, 1.0));
    assert!(!close(a.declination_deg, b.declination_deg, 0.001));
}

#[test]
fn out_of_window_and_bad_locations_are_refused_loudly() {
    let loc = Location {
        latitude_deg: 43.0,
        longitude_deg: 93.0,
        altitude_km: 65.0,
    };
    assert_eq!(evaluate(loc, 2024.999), Err(Error::OutOfWindow));
    assert_eq!(evaluate(loc, 2030.001), Err(Error::OutOfWindow));
    assert_eq!(evaluate(loc, f64::NAN), Err(Error::OutOfWindow));
    for bad in [
        Location {
            latitude_deg: 90.1,
            ..loc
        },
        Location {
            latitude_deg: -90.1,
            ..loc
        },
        Location {
            longitude_deg: 180.1,
            ..loc
        },
        Location {
            altitude_km: -1.1,
            ..loc
        },
        Location {
            altitude_km: 850.1,
            ..loc
        },
        Location {
            latitude_deg: f64::NAN,
            ..loc
        },
    ] {
        assert_eq!(evaluate(bad, 2025.0), Err(Error::InvalidLocation));
    }
}

#[test]
fn evaluation_is_pure_and_north_unit_has_no_vertical_component() {
    let loc = Location {
        latitude_deg: 17.0,
        longitude_deg: 52.0,
        altitude_km: 43.0,
    };
    let a = evaluate(loc, 2026.5).unwrap();
    let b = evaluate(loc, 2026.5).unwrap();
    assert_eq!(a.total_nt.to_bits(), b.total_nt.to_bits());
    assert_eq!(a.declination_deg.to_bits(), b.declination_deg.to_bits());
    let n = a.magnetic_north_unit();
    assert_eq!(n[2], 0.0);
    assert!(close(n[0] * n[0] + n[1] * n[1], 1.0, 1e-12));
    assert!(close(
        n[1].atan2(n[0]) * 180.0 / std::f64::consts::PI,
        a.declination_deg,
        1e-9,
    ));
}
