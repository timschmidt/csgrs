use hypercurve::{
    Curve2, CurveParameterRange2, CurveParameterSet2, CurvePointLocations2, LineSeg2, NurbsCurve2,
    Point2, RationalMap2,
};
use hyperreal::Real;
use hypersolve::BivariatePolynomial;
use std::cmp::Ordering;
fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(rows: Vec<Vec<i32>>) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.into_iter()
            .map(|r| r.into_iter().map(Real::from).collect())
            .collect(),
    )
}

#[test]
fn mixed_components_transport_original_charts_and_exact_restrictions() {
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(0, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 2.into(), 3.into(), 3.into()],
        )
        .unwrap(),
    );
    let CurvePointLocations2::Components(components) = source
        .point_locations(&Point2::from_values(0, 0).into())
        .unwrap()
    else {
        panic!("mixed source locus")
    };
    assert_eq!(components.len(), 2);
    let mapped = components[0]
        .affine_image_restricted(&q(-1, 3), &Real::one(), &CurveParameterRange2::unit())
        .unwrap()
        .unwrap();
    assert_eq!(mapped.span_index(), components[0].span_index());
    let CurveParameterSet2::Range(range) = mapped.parameters().unwrap() else {
        panic!("interval")
    };
    assert_eq!(
        range.start().compare(&q(2, 3).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Real::one().into()).unwrap(),
        Ordering::Equal
    );
    assert!(mapped.contains_parameter(&q(5, 6).into()).unwrap());
    assert!(!mapped.contains_parameter(&q(1, 2).into()).unwrap());
    let clip = CurveParameterRange2::try_new(q(3, 4).into(), q(5, 6).into()).unwrap();
    let clipped = components[0]
        .affine_image_restricted(&q(-1, 3), &Real::one(), &clip)
        .unwrap()
        .unwrap();
    let CurveParameterSet2::Range(range) = clipped.parameters().unwrap() else {
        panic!("interval")
    };
    assert_eq!(
        range.start().compare(&q(3, 4).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&q(5, 6).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(clipped.endpoint_inclusion(), [true, true]);
    assert!(
        components[0]
            .affine_image_restricted(&Real::zero(), &Real::one(), &clip)
            .is_err()
    );
}

#[test]
fn original_guard_punctures_survive_chart_reversal_and_boundary_clipping() {
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let map = RationalMap2::new(
        [p(vec![vec![-2], vec![4]]), p(vec![vec![-3], vec![6]])],
        p(vec![vec![-1], vec![2]]),
    );
    let CurvePointLocations2::Components(components) = source
        .rational_image_point_locations(&map, &Point2::from_values(2, 3).into())
        .unwrap()
    else {
        panic!("punctured native constant inverse")
    };
    assert_eq!(components.len(), 2);
    let domain = CurveParameterRange2::try_new(Real::from(1).into(), Real::from(3).into()).unwrap();
    let mapped = components[0]
        .affine_image_restricted(&(-2).into(), &2.into(), &domain)
        .unwrap()
        .unwrap();
    assert_eq!(mapped.endpoint_inclusion(), [false, true]);
    assert!(!mapped.contains_parameter(&Real::from(1).into()).unwrap());
    assert!(mapped.contains_parameter(&q(3, 2).into()).unwrap());
    assert!(mapped.contains_parameter(&Real::from(2).into()).unwrap());
    assert!(
        components[1]
            .affine_image_restricted(&(-2).into(), &2.into(), &domain)
            .unwrap()
            .is_none()
    );
    let outside =
        CurveParameterRange2::try_new(Real::from(3).into(), Real::from(4).into()).unwrap();
    assert!(
        components[0]
            .affine_image_restricted(&Real::one(), &Real::zero(), &outside)
            .unwrap()
            .is_none()
    );
}

#[test]
fn selected_clipping_endpoint_retains_its_original_root() {
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(0, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 2.into(), 3.into(), 3.into()],
        )
        .unwrap(),
    );
    let CurvePointLocations2::Components(components) = source
        .point_locations(&Point2::from_values(0, 0).into())
        .unwrap()
    else {
        panic!("mixed source")
    };
    let root: hypercurve::CurveParameter2 =
        hypercurve::BezierParameterPolynomial::try_new_power_basis(
            [-1, 1, 0, 0, 0, 1].map(Real::from).to_vec(),
        )
        .unwrap()
        .isolate_unit_interval_roots()
        .unwrap()[0]
            .clone()
            .into();
    let domain = CurveParameterRange2::try_new(root.clone(), Real::from(2).into()).unwrap();
    let clipped = components[0]
        .affine_image_restricted(&Real::one(), &Real::zero(), &domain)
        .unwrap()
        .unwrap();
    let CurveParameterSet2::Range(range) = clipped.parameters().unwrap() else {
        panic!("selected interval")
    };
    assert_eq!(range.start().compare(&root).unwrap(), Ordering::Equal);
    assert_eq!(
        range.end().compare(&Real::one().into()).unwrap(),
        Ordering::Equal
    );
    assert!(clipped.contains_parameter(&root).unwrap());
    assert!(!clipped.contains_parameter(&q(1, 2).into()).unwrap());
}

#[test]
fn source_components_select_distinct_one_sided_values_and_reject_changed_charts() {
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(1, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 1.into(), 2.into(), 2.into()],
        )
        .unwrap(),
    );
    let mut values = Vec::new();
    for x in [0, 1] {
        let CurvePointLocations2::Components(components) = source
            .point_locations(&Point2::from_values(x, 0).into())
            .unwrap()
        else {
            panic!("one-sided interval")
        };
        assert_eq!(components.len(), 1);
        let point = source
            .point_at_component(&components[0], &Real::one().into())
            .unwrap();
        assert!(
            point
                .coincides_with(&Point2::from_values(x, 0).into())
                .unwrap()
        );
        values.push(point);
        let changed = components[0]
            .affine_image_restricted(
                &Real::from(2),
                &Real::zero(),
                &CurveParameterRange2::try_new(Real::zero().into(), Real::from(4).into()).unwrap(),
            )
            .unwrap()
            .unwrap();
        assert!(
            source
                .point_at_component(&changed, &Real::from(2).into())
                .is_err()
        );
    }
    assert!(!values[0].coincides_with(&values[1]).unwrap());
    assert!(
        matches!(source.point_at(&Real::one().into()),Err(hypercurve::ExactCurveError::Blocked(cause)) if cause.reason()==hypercurve::UncertaintyReason::Boundary)
    );
}
