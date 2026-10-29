//! Display chords consume certified enclosures without replacing selected curves.

use std::cmp::Ordering;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurveParameterRange2, CurvePath2, FiniteProjectionOptions,
    PolynomialCurveSet2, Real, Similarity2,
};
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn curves(rows: &[&[i32]], first: [i32; 2], second: [i32; 2]) -> PolynomialCurveSet2 {
    let equation = BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    );
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: first.map(Real::from),
            second: second.map(Real::from),
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn leq(first: &Real, second: &Real) {
    assert!(matches!(
        compare_reals(first, second, PredicatePolicy::STRICT).value(),
        Some(Ordering::Less | Ordering::Equal)
    ));
}

#[test]
fn polynomial_projection_preserves_the_exact_source() {
    let source = curves(&[&[1, 2], &[], &[-2]], [-1, 1], [-1, 1]);
    assert_eq!(source.curves().len(), 1);
    let curve = source.curves()[0].clone();
    let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();
    let projection = path.project_to_finite_polyline(&options).unwrap();
    assert!(!projection.is_closed());
    assert_eq!(projection.points().first(), Some(&[-1.0, 0.5]));
    assert_eq!(projection.points().last(), Some(&[1.0, 0.5]));
    let error = Real::try_from(options.chord_error()).unwrap();
    let points: Vec<_> = projection
        .points()
        .iter()
        .map(|point| point.map(|coordinate| Real::try_from(coordinate).unwrap()))
        .collect();
    for [x, y] in &points {
        // All expected vertices in this dyadic quadratic have an exact oracle.
        assert_eq!(
            compare_reals(y, &(x * x - q(1, 2)), PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal)
        );
    }
    for pair in points.windows(2) {
        let dx = &pair[1][0] - &pair[0][0];
        assert_eq!(
            compare_reals(&dx, &Real::zero(), PredicatePolicy::STRICT).value(),
            Some(Ordering::Greater)
        );
        // y minus its exact secant is (x-a)(x-b), whose absolute maximum
        // on [a,b] is (b-a)^2/4. This bounds the entire arc independently.
        leq(&((&dx * &dx / Real::from(4)).unwrap()), &error);
    }
    assert!(
        path.curves()[0]
            .polynomial_branch()
            .unwrap()
            .shares_source(curve.polynomial_branch().unwrap())
    );
    let (left, right) = path.curves()[0].split_at(Real::zero().into()).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    assert!(left.start().coincides_with(&curve.start()).unwrap());
    assert!(right.end().coincides_with(&curve.end()).unwrap());
}

#[test]
fn polynomial_projection_handles_selected_trims_and_singular_endpoints() {
    let source = curves(&[&[0, 0, 1], &[], &[], &[-1]], [0, 1], [-1, 1]);
    assert_eq!(source.curves().len(), 2);
    let end = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![q(-1, 2), 0.into(), 1.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), 1.into()).unwrap(),
        )
        .unwrap(),
    );
    let curve = source.curves()[1]
        .restrict_source(CurveParameterRange2::try_new(Real::zero().into(), end.into()).unwrap())
        .unwrap();
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        1.into(),
        3.into(),
    )
    .unwrap();
    for curve in [
        curve.clone(),
        curve.reversed().unwrap(),
        curve.transform_similarity(&transform).unwrap(),
    ] {
        let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
        let projected = path
            .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.02).unwrap())
            .unwrap();
        assert!(projected.points().len() >= 2);
        assert!(
            projected
                .points()
                .iter()
                .flatten()
                .all(|value| value.is_finite())
        );
        assert!(
            path.curves()[0]
                .start()
                .coincides_with(&curve.start())
                .unwrap()
        );
        assert!(path.curves()[0].end().coincides_with(&curve.end()).unwrap());
        assert!(path.curves()[0].polynomial_branch().is_some());
    }
}

#[test]
fn polynomial_projection_bounds_waves_between_equal_samples() {
    // y=16x^3-24x^2+8x vanishes at both endpoints and their midpoint.
    let source = curves(&[&[0, 1], &[-8], &[24], &[-16]], [0, 1], [-2, 2]);
    assert_eq!(source.curves().len(), 1);
    let path = CurvePath2::try_new(source.curves().to_vec()).unwrap();
    let projected = path
        .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.02).unwrap())
        .unwrap();
    let error = Real::try_from(projected.chord_error()).unwrap();
    let points: Vec<_> = projected
        .points()
        .iter()
        .map(|point| point.map(|coordinate| Real::try_from(coordinate).unwrap()))
        .collect();
    assert!(projected.points().iter().any(|point| point[1] > 0.7));
    assert!(projected.points().iter().any(|point| point[1] < -0.7));
    for [x, y] in &points {
        let expected = Real::from(16) * x * x * x - Real::from(24) * x * x + Real::from(8) * x;
        assert_eq!(
            compare_reals(y, &expected, PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal)
        );
    }
    for pair in points.windows(2) {
        let dx = &pair[1][0] - &pair[0][0];
        // |y''| <= 48 on [0,1], so secant interpolation error <= 6*(b-a)^2.
        leq(&(Real::from(6) * &dx * &dx), &error);
    }
    eprintln!(
        "certified wavy projection: {} points",
        projected.points().len()
    );
}

#[test]
fn polynomial_projection_exhausts_and_retries_without_partial_output() {
    use hypercurve::{ExactCurveError, UncertaintyReason};
    use hypersolve::work_budget::{WorkBudget, with_budget};
    let source = curves(&[&[1, 2], &[], &[-2]], [-1, 1], [-1, 1]);
    let path = CurvePath2::try_new(source.curves().to_vec()).unwrap();
    let options = FiniteProjectionOptions::try_new(0.01).unwrap();
    for limited in [
        options.with_polynomial_subdivision_limits(0, 65_536),
        options.with_polynomial_subdivision_limits(128, 0),
        options.with_polynomial_subdivision_limits(128, 1),
    ] {
        assert!(matches!(path.project_to_finite_polyline(&limited),
            Err(ExactCurveError::Blocked(blocker)) if blocker.reason()==UncertaintyReason::BudgetExhausted));
    }
    let denied = with_budget(WorkBudget::ZERO, || {
        path.project_to_finite_polyline(&options)
    });
    assert!(denied.result.is_err());
    let completed = with_budget(
        WorkBudget {
            refinements: 1_000_000,
            promotions: 10_000,
        },
        || path.project_to_finite_polyline(&options),
    );
    let projected = completed.result.unwrap().unwrap();
    assert!(completed.usage.refinements > 0);
    assert_eq!(
        projected,
        path.project_to_finite_polyline(&options).unwrap()
    );
    let denied_again = with_budget(WorkBudget::ZERO, || {
        path.project_to_finite_polyline(&options)
    });
    assert!(denied_again.result.is_err());
    assert!(
        path.curves()[0]
            .polynomial_branch()
            .unwrap()
            .shares_source(source.curves()[0].polynomial_branch().unwrap())
    );
    eprintln!(
        "retry projection: {} points, {:?}",
        projected.points().len(),
        completed.usage
    );
}

#[test]
fn polynomial_projection_transports_original_rational_maps() {
    let source = curves(&[&[1, 2], &[], &[-2]], [-1, 1], [-1, 1]);
    let map = hypercurve::RationalMap2::new(
        [
            BivariatePolynomial::new(vec![vec![], vec![1.into()]]),
            BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]),
        ],
        BivariatePolynomial::new(vec![vec![2.into(), 1.into()]]),
    );
    let curve = source.curves()[0].rational_image(&map).unwrap();
    let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
    let projected = path
        .project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.02).unwrap())
        .unwrap();
    assert_eq!(projected.points().first(), Some(&[-0.4, 0.2]));
    assert_eq!(projected.points().last(), Some(&[0.4, 0.2]));
    assert_eq!(
        path.curves()[0]
            .polynomial_branch()
            .unwrap()
            .rational_maps(),
        &[map]
    );
    let (left, right) = curve.split_at(Real::zero().into()).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    assert!(
        left.polynomial_branch()
            .unwrap()
            .shares_source(source.curves()[0].polynomial_branch().unwrap())
    );
}

#[test]
fn polynomial_projection_rejects_an_unrepresentable_error_budget() {
    use hypercurve::{ExactCurveError, UncertaintyReason};
    let source = curves(&[&[1, 2], &[], &[-2]], [-1, 1], [-1, 1]);
    let map = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        Real::from(1_u64 << 60),
        0.into(),
    )
    .unwrap();
    let curve = source.curves()[0].transform_similarity(&map).unwrap();
    let path = CurvePath2::try_new(vec![curve]).unwrap();
    assert!(
        matches!(path.project_to_finite_polyline(&FiniteProjectionOptions::try_new(0.01).unwrap()),
        Err(ExactCurveError::Blocked(blocker)) if blocker.reason()==UncertaintyReason::Predicate)
    );
}

#[cfg(feature = "svg")]
#[test]
fn polynomial_svg_replay_preserves_maps_reversal_and_further_restriction() {
    use hypercurve::{
        CurveRegion2, RationalMap2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    let source = curves(&[&[1, 2], &[], &[-2]], [-1, 1], [-1, 1]);
    let map = RationalMap2::new(
        [
            BivariatePolynomial::new(vec![vec![], vec![1.into()]]),
            BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]),
        ],
        BivariatePolynomial::new(vec![vec![2.into(), 1.into()]]),
    );
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        1.into(),
        3.into(),
    )
    .unwrap();
    let curve = source.curves()[0]
        .rational_image(&map)
        .unwrap()
        .transform_similarity(&transform)
        .unwrap()
        .reversed()
        .unwrap();
    let original = CurvePath2::try_new(vec![curve.clone()]).unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![original],
    ))
    .unwrap();
    let replay = import_svg_document(&text).unwrap();
    let restored = &replay.paths()[0].curves()[0];
    let branch = restored.polynomial_branch().unwrap();
    assert!(branch.is_reversed());
    assert_eq!(branch.rational_maps(), &[map]);
    assert_eq!(branch.image_transform(), Some(&transform));
    for parameter in [q(-1, 2), Real::zero(), q(1, 2)] {
        assert!(
            restored
                .point_at(&parameter.clone().into())
                .unwrap()
                .coincides_with(&curve.point_at(&parameter.into()).unwrap())
                .unwrap()
        );
    }
    let (left, right) = restored.split_at(Real::zero().into()).unwrap();
    assert!(left.end().coincides_with(&right.start()).unwrap());
    assert!(left.start().coincides_with(&restored.start()).unwrap());
    assert!(right.end().coincides_with(&restored.end()).unwrap());
    assert!(
        left.polynomial_branch()
            .unwrap()
            .shares_source(right.polynomial_branch().unwrap())
    );
    assert_eq!(export_svg_document(&replay).unwrap(), text);
}

#[cfg(feature = "svg")]
#[test]
fn polynomial_filled_svg_replay_reenters_the_shared_arrangement() {
    use hypercurve::{
        BooleanOp, Curve2, CurveRegion2, FillRule, LineSeg2, Point2, RegionPointLocation,
        SvgGeometry2, export_svg_document, import_svg_document,
    };
    let source = curves(&[&[0, 0, 1], &[-1]], [-1, 1], [-2, 2]);
    assert_eq!(source.curves().len(), 2);
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    let path = CurvePath2::try_new(vec![
        source.curves()[0].restrict_source(range.clone()).unwrap(),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(q(1, 4), q(-1, 2)),
                Point2::new(q(1, 4), q(1, 2)),
            )
            .unwrap(),
        ),
        source.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    let text = export_svg_document(&SvgGeometry2::new(region.clone(), vec![], vec![])).unwrap();
    let replay = import_svg_document(&text).unwrap();
    assert_eq!(replay.region().boundary_loops().len(), 1);
    assert_eq!(
        replay.region().boundary_loops()[0]
            .curves()
            .iter()
            .filter(|curve| curve.polynomial_branch().is_some())
            .count(),
        2
    );
    // Independent set contract: 0 <= x <= 1/4 and y^2 <= x.
    for (x, y, expected) in [
        (q(1, 8), Real::zero(), RegionPointLocation::Inside),
        (q(1, 8), q(1, 2), RegionPointLocation::Outside),
        (q(1, 4), Real::zero(), RegionPointLocation::Boundary),
        (Real::zero(), Real::zero(), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            replay
                .region()
                .classify_point(&Point2::new(x, y).into())
                .unwrap(),
            expected
        );
    }
    assert!(
        replay
            .region()
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let empty = replay
        .region()
        .boolean_region(&region, BooleanOp::Difference)
        .unwrap();
    assert!(empty.is_empty());
    assert!(
        empty
            .boolean_region(replay.region(), BooleanOp::Union)
            .unwrap()
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert_eq!(export_svg_document(&replay).unwrap(), text);
    // A native rectangular hole and a translated disconnected cap exercise
    // several retained source loops through the same compound graph.
    let hole_points = [
        Point2::new(q(1, 8), q(-1, 8)),
        Point2::new(q(3, 16), q(-1, 8)),
        Point2::new(q(3, 16), q(1, 8)),
        Point2::new(q(1, 8), q(1, 8)),
    ];
    let hole_path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(hole_points[i].clone(), hole_points[(i + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let hole = CurveRegion2::try_from_boundary_paths(&[hole_path], FillRule::NonZero).unwrap();
    let translation = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
    )
    .unwrap();
    let translated_path = CurvePath2::try_new(
        region.boundary_loops()[0]
            .curves()
            .iter()
            .map(|curve| curve.transform_similarity(&translation).unwrap())
            .collect(),
    )
    .unwrap();
    let translated =
        CurveRegion2::try_from_boundary_paths(&[translated_path], FillRule::NonZero).unwrap();
    let compound = region
        .boolean_region(&hole, BooleanOp::Difference)
        .unwrap()
        .boolean_region(&translated, BooleanOp::Union)
        .unwrap();
    let text = export_svg_document(&SvgGeometry2::new(compound.clone(), vec![], vec![])).unwrap();
    let replay = import_svg_document(&text).unwrap();
    assert_eq!(replay.region().boundary_loops().len(), 3);
    for (x, expected) in [
        (q(5, 32), RegionPointLocation::Outside),
        (q(1, 16), RegionPointLocation::Inside),
        (q(17, 16), RegionPointLocation::Inside),
        (q(1, 2), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            replay
                .region()
                .classify_point(&Point2::new(x, Real::zero()).into())
                .unwrap(),
            expected
        );
    }
    assert!(
        replay
            .region()
            .boolean_region(&compound, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    assert_eq!(export_svg_document(&replay).unwrap(), text);
}

#[test]
fn polynomial_bounds_keep_selected_abscissas_before_images() {
    let source = curves(&[&[0, 0, 1], &[-1]], [-1, 1], [-2, 2]);
    let selected = source.curves()[0]
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap(),
        )
        .unwrap();
    let bounds = selected.bounds().unwrap();
    assert_eq!(bounds.min().x(), &Real::zero());
    assert_eq!(bounds.max().x(), &q(1, 4));
    let image = selected
        .transform_similarity(
            &Similarity2::try_from_real_affine(
                1.into(),
                0.into(),
                0.into(),
                1.into(),
                1.into(),
                0.into(),
            )
            .unwrap(),
        )
        .unwrap();
    let bounds = image.bounds().unwrap();
    assert_eq!(bounds.min().x(), &Real::one());
    assert_eq!(bounds.max().x(), &q(5, 4));
    assert!(
        selected
            .polynomial_branch()
            .unwrap()
            .shares_source(image.polynomial_branch().unwrap())
    );
}
