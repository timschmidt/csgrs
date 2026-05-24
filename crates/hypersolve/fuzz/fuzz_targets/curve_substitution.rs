#![no_main]

use hyperreal::Real;
use hypersolve::{
    BezierPowerBasisSubstitutionConfig, BezierPowerBasisSubstitutionStatus,
    CurveIntersectionResultantConfig, CurveIntersectionResultantStatus, CurveResultantParameter,
    PolynomialCurvePoint2, PolynomialParametricCurve2, RationalBezierPowerBasisSubstitutionStatus,
    RationalCurveControlPoint2, RationalParametricCurve2,
    resultant_rational_parametric_curve_intersection, substitute_bezier_power_basis,
    substitute_rational_bezier_power_basis,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fn point(x: i64, y: i64) -> PolynomialCurvePoint2 {
    PolynomialCurvePoint2::new(real(x), real(y))
}

fn weighted(x: i64, y: i64, weight: i64) -> RationalCurveControlPoint2 {
    RationalCurveControlPoint2::new(real(x), real(y), real(weight))
}

fuzz_target!(|data: [i16; 4]| {
    let x0 = i64::from(data[0] % 32);
    let y0 = i64::from(data[1] % 32);
    let x1 = i64::from(data[2] % 32);
    let y1 = i64::from(data[3] % 32);
    let substitution = substitute_bezier_power_basis(
        &[point(x0, y0), point(x1, y1)],
        BezierPowerBasisSubstitutionConfig::default(),
    );
    assert_eq!(
        substitution.status,
        BezierPowerBasisSubstitutionStatus::Constructed
    );
    let curve = substitution.power_basis.unwrap();
    assert_eq!(curve.x_coefficients, vec![real(x0), real(x1 - x0)]);
    assert_eq!(curve.y_coefficients, vec![real(y0), real(y1 - y0)]);

    let w0 = i64::from((data[0] % 8).abs()) + 1;
    let w1 = i64::from((data[1] % 8).abs()) + 1;
    let rational_substitution = substitute_rational_bezier_power_basis(
        &[weighted(x0, y0, w0), weighted(x1, y1, w1)],
        BezierPowerBasisSubstitutionConfig::default(),
    );
    assert_eq!(
        rational_substitution.status,
        RationalBezierPowerBasisSubstitutionStatus::Constructed
    );
    let rational_curve = rational_substitution.homogeneous_power_basis.unwrap();
    assert_eq!(
        rational_curve.x_numerator,
        vec![real(x0 * w0), real(x1 * w1 - x0 * w0)]
    );
    assert_eq!(
        rational_curve.y_numerator,
        vec![real(y0 * w0), real(y1 * w1 - y0 * w0)]
    );
    assert_eq!(rational_curve.weight, vec![real(w0), real(w1 - w0)]);

    let height = i64::from((data[0] % 16).abs()) + 1;
    let weighted_parabola = RationalParametricCurve2::new(
        vec![real(0), real(1)],
        vec![real(0), real(0), real(1)],
        vec![real(1), real(1)],
    );
    let horizontal = RationalParametricCurve2::from_polynomial(&PolynomialParametricCurve2::new(
        vec![real(0), real(1)],
        vec![real(height)],
    ));
    let report = resultant_rational_parametric_curve_intersection(
        &weighted_parabola,
        &horizontal,
        CurveResultantParameter::First,
        CurveIntersectionResultantConfig::default(),
    );
    assert_eq!(report.status, CurveIntersectionResultantStatus::Constructed);
    assert_eq!(
        report.resultant_coefficients,
        vec![real(-height), real(-height), real(1)]
    );
});
