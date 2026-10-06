#![no_main]

use hyperreal::Real;
use hypersolve::{
    CurveIntersectionResultantConfig, CurveIntersectionResultantStatus, CurveResultantParameter,
    PolynomialParametricCurve2, resultant_parametric_curve_intersection,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fuzz_target!(|data: [i16; 2]| {
    let height = i64::from((data[0] % 32).abs()) + 1;
    let x_shift = i64::from(data[1] % 16);
    let parabola = PolynomialParametricCurve2::new(
        vec![real(x_shift), real(1)],
        vec![real(0), real(0), real(1)],
    );
    let horizontal = PolynomialParametricCurve2::new(
        vec![real(x_shift), real(1)],
        vec![real(height)],
    );

    let report = resultant_parametric_curve_intersection(
        &parabola,
        &horizontal,
        CurveResultantParameter::First,
        CurveIntersectionResultantConfig::default(),
    );
    assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
    assert_eq!(
        report.resultant_coefficients,
        vec![real(-height), real(0), real(1)]
    );
});
