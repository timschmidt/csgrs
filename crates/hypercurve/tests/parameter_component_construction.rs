use hypercurve::{
    Curve2, CurveLocationComponent2, CurveParameter2, CurveParameterRange2, CurveParameterSet2,
    Point2, RationalBezier2,
};
use hyperreal::Real;
#[test]
fn component_metadata_preserves_selected_charts_without_claiming_incidence() {
    let alpha: CurveParameter2 = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-1, 1, 0, 0, 0, 1].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into();
    assert!(alpha.scalar().is_none());
    let component = CurveLocationComponent2::from_span_parameters(
        0,
        3.into(),
        (-2).into(),
        CurveParameterSet2::Single(alpha.clone()),
        [true, true],
    )
    .unwrap();
    let global = alpha.affine_image(&(-5).into(), &3.into()).unwrap();
    assert!(component.contains_parameter(&global).unwrap());
    let unit = Curve2::from(
        RationalBezier2::try_new(vec![Point2::from_values(7, 9); 2], vec![Real::one(); 2]).unwrap(),
    );
    assert!(unit.point_at_component(&component, &global).is_err());
    let unit_component = CurveLocationComponent2::from_span_parameters(
        0,
        0.into(),
        1.into(),
        CurveParameterSet2::Single(alpha.clone()),
        [true, true],
    )
    .unwrap();
    assert!(
        unit.point_at_component(&unit_component, &alpha)
            .unwrap()
            .coincides_with(&Point2::from_values(7, 9).into())
            .unwrap()
    );
}
#[test]
fn invalid_or_empty_component_metadata_never_publishes() {
    let range = || CurveParameterSet2::Range(CurveParameterRange2::unit());
    assert!(
        CurveLocationComponent2::from_span_parameters(0, 1.into(), 1.into(), range(), [true, true])
            .is_err()
    );
    for value in [-1, 2] {
        assert!(
            CurveLocationComponent2::from_span_parameters(
                0,
                0.into(),
                1.into(),
                CurveParameterSet2::Single(Real::from(value).into()),
                [true, true]
            )
            .is_err()
        );
    }
    for inclusion in [[false, true], [true, false], [false, false]] {
        assert!(
            CurveLocationComponent2::from_span_parameters(
                0,
                0.into(),
                1.into(),
                CurveParameterSet2::Single(Real::zero().into()),
                inclusion
            )
            .is_err()
        );
    }
    let r = CurveParameterRange2::try_new(Real::from(-1).into(), Real::one().into()).unwrap();
    assert!(
        CurveLocationComponent2::from_span_parameters(
            0,
            0.into(),
            1.into(),
            CurveParameterSet2::Range(r),
            [true, true]
        )
        .is_err()
    );
}
