// SPDX-License-Identifier: GPL-3.0-or-later
//! Host gates for the fixed-size calibration table: exact correction at a
//! point, deterministic interpolation, endpoint clamping, LOUD rejection of
//! invalid tables and readings, and compile-time validation.

#[path = "../src/calibration_table.rs"]
mod calibration_table;

use calibration_table::{
    AxisCal, CalibrationError, CalibrationTable, TempPoint, AXES, BIAS_ONE, MAX_RAW_ABS, MAX_SCALE,
    MAX_TEMP_POINTS, SCALE_ONE, TEMP_ONE,
};

/// Degrees Celsius to the table's Q8 temperature unit.
const fn c(degrees: i32) -> i32 {
    degrees * TEMP_ONE
}

/// Raw counts to a Q12 bias.
const fn bias(counts: i32) -> i32 {
    counts * BIAS_ONE
}

/// Multiplier to a Q12 scale.
const fn scale(multiplier: i32) -> i32 {
    multiplier * SCALE_ONE
}

const fn point(temp_c: i32, biases: [i32; AXES], scales: [i32; AXES]) -> TempPoint {
    TempPoint {
        temp_q8: c(temp_c),
        axes: [
            AxisCal {
                bias_q12: bias(biases[0]),
                scale_q12: scale(scales[0]),
            },
            AxisCal {
                bias_q12: bias(biases[1]),
                scale_q12: scale(scales[1]),
            },
            AxisCal {
                bias_q12: bias(biases[2]),
                scale_q12: scale(scales[2]),
            },
        ],
    }
}

#[test]
fn single_point_applies_bias_and_scale_exactly() {
    let table = CalibrationTable::new(&[TempPoint {
        temp_q8: c(20),
        axes: [
            AxisCal {
                bias_q12: bias(1),
                scale_q12: scale(2),
            },
            AxisCal {
                bias_q12: bias(2),
                scale_q12: SCALE_ONE,
            },
            AxisCal {
                bias_q12: bias(3),
                scale_q12: SCALE_ONE / 2,
            },
        ],
    }])
    .unwrap();

    assert_eq!(table.point_count(), 1);
    assert_eq!(table.point(0).temp_q8, c(20));
    // (raw - bias) * scale: (11-1)*2, (12-2)*1, (13-3)*0.5.
    assert_eq!(table.apply([11, 12, 13], c(20)), Ok([20, 10, 5]));
}

#[test]
fn interpolation_is_linear_and_exact_at_known_fractions() {
    let table = CalibrationTable::new(&[
        point(0, [0, 0, 0], [1, 1, 1]),
        point(100, [10, 20, 30], [2, 2, 2]),
    ])
    .unwrap();

    // 25% of the way: bias 2.5/5/7.5, scale 1.25.
    assert_eq!(table.apply([10, 10, 10], c(25)), Ok([9, 6, 3]));
    // 50% of the way: bias 5/10/15, scale 1.5; halves round away from zero.
    assert_eq!(table.apply([10, 10, 10], c(50)), Ok([8, 0, -8]));
    // Exactly on a knot selects that point's parameters.
    assert_eq!(table.apply([10, 11, 12], c(0)), Ok([10, 11, 12]));
    assert_eq!(table.apply([10, 20, 30], c(100)), Ok([0, 0, 0]));
}

#[test]
fn temperatures_outside_table_clamp_to_the_nearest_endpoint() {
    let table = CalibrationTable::new(&[
        point(0, [0, 0, 0], [1, 1, 1]),
        point(100, [10, 20, 30], [2, 2, 2]),
    ])
    .unwrap();

    assert_eq!(
        table.apply([10, 11, 12], c(-50)),
        table.apply([10, 11, 12], c(0))
    );
    assert_eq!(
        table.apply([10, 11, 12], c(150)),
        table.apply([10, 11, 12], c(100))
    );
}

#[test]
fn repeated_applications_are_bit_identical() {
    let table = CalibrationTable::new(&[
        point(-40, [1, -2, 3], [1, 1, 1]),
        point(25, [2, -1, 4], [2, 1, 1]),
        point(85, [3, 0, 5], [3, 2, 1]),
    ])
    .unwrap();
    let raw = [123, -456, 789];
    let temp = c(37);
    let first = table.apply(raw, temp).unwrap();
    for _ in 0..100 {
        assert_eq!(table.apply(raw, temp), Ok(first));
    }
}

#[test]
fn invalid_tables_are_loud() {
    let good = point(0, [0, 0, 0], [1, 1, 1]);

    assert_eq!(CalibrationTable::new(&[]), Err(CalibrationError::NoPoints));

    let too_many = [good; MAX_TEMP_POINTS + 1];
    assert_eq!(
        CalibrationTable::new(&too_many),
        Err(CalibrationError::TooManyPoints)
    );

    let mut decreasing = good;
    decreasing.temp_q8 = -1;
    assert_eq!(
        CalibrationTable::new(&[good, decreasing]),
        Err(CalibrationError::TemperaturesNotIncreasing)
    );
    assert_eq!(
        CalibrationTable::new(&[good, good]),
        Err(CalibrationError::TemperaturesNotIncreasing)
    );

    for bad_scale in [0, -1, MAX_SCALE + 1] {
        let mut bad = good;
        bad.axes[1].scale_q12 = bad_scale;
        assert_eq!(
            CalibrationTable::new(&[bad]),
            Err(CalibrationError::ScaleOutOfRange)
        );
    }
}

#[test]
fn out_of_range_readings_are_loud() {
    let table = CalibrationTable::new(&[point(0, [0, 0, 0], [1, 1, 1])]).unwrap();
    for axis in 0..AXES {
        let mut raw = [0; AXES];
        raw[axis] = MAX_RAW_ABS + 1;
        assert_eq!(
            table.apply(raw, c(20)),
            Err(CalibrationError::RawOutOfRange)
        );
        raw[axis] = -(MAX_RAW_ABS + 1);
        assert_eq!(
            table.apply(raw, c(20)),
            Err(CalibrationError::RawOutOfRange)
        );
    }
    assert!(table.apply([MAX_RAW_ABS; AXES], c(20)).is_ok());
}

#[test]
fn construction_and_correction_are_const() {
    // Compiling this item proves `new` is a const fn and validates at build time.
    const TABLE: Result<CalibrationTable, CalibrationError> =
        CalibrationTable::new(&[point(20, [0, 0, 0], [1, 1, 1])]);
    assert!(TABLE.is_ok());
    const OUT: [i32; AXES] = match TABLE {
        Ok(t) => match t.apply([7, 8, 9], c(20)) {
            Ok(v) => v,
            Err(_) => [0; AXES],
        },
        Err(_) => [0; AXES],
    };
    assert_eq!(OUT, [7, 8, 9]);
}
