//! Complete preimages through retained rational maps.
use hypercurve::{
    Curve2, CurveParameter2, CurvePoint2, CurvePointLocations2, LineSeg2, NurbsCurve2, Point2,
    PolynomialCurveSet2, RationalBezier2, RationalMap2, Real,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
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
fn branches(equation: BivariatePolynomial, first: [Real; 2], second: [Real; 2]) -> Vec<Curve2> {
    let t = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain { first, second },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(t, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
        .curves()
        .to_vec()
}
fn branch() -> Curve2 {
    branches(
        p(vec![vec![0, 1]]),
        [(-1).into(), 1.into()],
        [(-1).into(), 1.into()],
    )
    .remove(0)
}
fn alpha() -> CurveParameter2 {
    hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-1, 1, 0, 0, 0, 1].map(Real::from).to_vec(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into()
}
fn monomials(degree: usize, x: usize, y: usize) -> Curve2 {
    fn choose(n: usize, k: usize) -> i64 {
        if k > n {
            return 0;
        }
        (0..k).fold(1, |v, i| v * (n - i) as i64 / (i + 1) as i64)
    }
    RationalBezier2::try_new(
        (0..=degree)
            .map(|i| {
                Point2::new(
                    q(choose(i, x), choose(degree, x)),
                    q(choose(i, y), choose(degree, y)),
                )
            })
            .collect(),
        vec![Real::one(); degree + 1],
    )
    .unwrap()
    .into()
}
fn locations(source: &Curve2, map: &RationalMap2, query: &CurvePoint2) -> Vec<CurveParameter2> {
    let CurvePointLocations2::Locations(found) =
        source.rational_image_point_locations(map, query).unwrap()
    else {
        panic!("finite preimages")
    };
    let parameters: Vec<_> = found.iter().map(|l| l.parameter().unwrap()).collect();
    for t in &parameters {
        assert!(
            source
                .point_at(t)
                .unwrap()
                .rational_image(map)
                .unwrap()
                .coincides_with(query)
                .unwrap()
        );
    }
    for pair in parameters.windows(2) {
        assert_eq!(pair[0].compare(&pair[1]).unwrap(), Ordering::Less);
    }
    parameters
}
#[test]
fn folded_selected_branch_reports_every_exact_visit() {
    let source = branch();
    let a = alpha();
    let map = RationalMap2::new(
        [p(vec![vec![], vec![], vec![1]]), p(vec![])],
        p(vec![vec![1]]),
    );
    let mapped = source.rational_image(&map).unwrap();
    let query = source.point_at(&a).unwrap().rational_image(&map).unwrap();
    assert!(query.coordinates().is_none());
    let CurvePointLocations2::Locations(found) = mapped.point_locations(&query).unwrap() else {
        panic!("finite visits")
    };
    assert_eq!(found.len(), 2);
    let expected = source
        .point_at(&a)
        .unwrap()
        .affine_form((-1).into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert_eq!(
        found[0].parameter().unwrap().compare(&expected).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        found[1].parameter().unwrap().compare(&a).unwrap(),
        Ordering::Equal
    );
    assert_eq!(locations(&source, &map, &query).len(), 2);
    let restricted = source
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(Real::zero().into(), Real::one().into())
                .unwrap(),
        )
        .unwrap();
    assert_eq!(locations(&restricted, &map, &query).len(), 1);
    assert_eq!(
        locations(&source.reversed().unwrap(), &map, &query).len(),
        2
    );
}
#[test]
fn native_map_queries_keep_independent_fields_knots_and_original_poles() {
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let a = alpha();
    let independent = monomials(6, 4, 6).point_at(&a).unwrap();
    let target = RationalMap2::new(
        [
            p(vec![vec![], vec![], vec![1]]),
            p(vec![vec![], vec![], vec![], vec![1]]),
        ],
        p(vec![vec![1], vec![1]]),
    );
    let query = independent
        .rational_image(&RationalMap2::new(
            [
                p(vec![vec![], vec![], vec![1]]),
                p(vec![vec![], vec![0, 1]]),
            ],
            p(vec![vec![0, 1], vec![1]]),
        ))
        .unwrap();
    let found = locations(&source, &target, &query);
    assert_eq!(found.len(), 1);
    let expected = monomials(2, 2, 2)
        .point_at(&a)
        .unwrap()
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert_eq!(found[0].compare(&expected).unwrap(), Ordering::Equal);
    let pole = RationalMap2::new(
        [p(vec![vec![], vec![-1], vec![2]]), p(vec![])],
        p(vec![vec![-1], vec![2]]),
    );
    let at_a = source.point_at(&a).unwrap();
    assert_eq!(locations(&source, &pole, &at_a).len(), 1);
    // (a^5+a)/2=1/2, while the original map excludes that parameter.
    let half = RationalBezier2::try_new(
        vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 10), 0.into()),
            Point2::new(q(1, 5), 0.into()),
            Point2::new(q(3, 10), 0.into()),
            Point2::new(q(2, 5), 0.into()),
            Point2::from_values(1, 0),
        ],
        vec![Real::one(); 6],
    )
    .unwrap();
    let excluded = Curve2::from(half).point_at(&a).unwrap();
    assert!(locations(&source, &pole, &excluded).is_empty());
    let spline = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(-1, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
            ],
            vec![Real::one(); 3],
            vec![(-2).into(), (-2).into(), 1.into(), 4.into(), 4.into()],
        )
        .unwrap(),
    );
    let square = RationalMap2::new(
        [p(vec![vec![], vec![], vec![1]]), p(vec![])],
        p(vec![vec![1]]),
    );
    let query = monomials(2, 2, 2)
        .point_at(&a)
        .unwrap()
        .rational_image(&RationalMap2::new(
            [p(vec![vec![], vec![1]]), p(vec![])],
            p(vec![vec![1]]),
        ))
        .unwrap();
    let found = locations(&spline, &square, &query);
    assert_eq!(found.len(), 2);
    let point = source.point_at(&a).unwrap();
    for (t, scale) in found.iter().zip([-3, 3]) {
        let expected = point
            .affine_form(scale.into(), 0.into(), 1.into())
            .parameter()
            .unwrap();
        assert_eq!(t.compare(&expected).unwrap(), Ordering::Equal);
    }
}
#[test]
fn selected_nonrational_branches_keep_rank_and_constant_images() {
    let curves = branches(
        p(vec![vec![0, 0, 1], vec![-1]]),
        [q(1, 4), 1.into()],
        [(-2).into(), 2.into()],
    );
    assert_eq!(curves.len(), 2);
    let a = alpha();
    let map = RationalMap2::new(
        [p(vec![vec![], vec![], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    );
    let identity = RationalMap2::new(
        [p(vec![vec![], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    );
    let query = curves[0]
        .point_at(&a)
        .unwrap()
        .rational_image(&map)
        .unwrap()
        .rational_image(&identity)
        .unwrap();
    assert!(query.coordinates().is_none());
    assert_eq!(locations(&curves[0], &map, &query).len(), 1);
    assert!(locations(&curves[1], &map, &query).is_empty());
    // y^5+y=1 selects one constant ordinate without a Real coordinate view.
    let source = branches(
        p(vec![vec![-1, 1, 0, 0, 0, 1]]),
        [0.into(), 1.into()],
        [0.into(), 1.into()],
    )
    .remove(0);
    let map = RationalMap2::new([p(vec![vec![0, 1]]), p(vec![])], p(vec![vec![1]]));
    let query = source
        .point_at(&q(1, 2).into())
        .unwrap()
        .rational_image(&map)
        .unwrap();
    assert!(query.coordinates().is_none());
    assert!(matches!(
        source.rational_image_point_locations(&map, &query).unwrap(),
        CurvePointLocations2::EntireCurve
    ));
    let wrong = query
        .rational_image(&RationalMap2::new(
            [p(vec![vec![], vec![1]]), p(vec![vec![1]])],
            p(vec![vec![1]]),
        ))
        .unwrap();
    assert!(locations(&source, &map, &wrong).is_empty());
}
#[test]
fn constant_and_punctured_images_and_budget_retry_stay_explicit() {
    use hypercurve::{CurveCertainty, PredicatePolicy, WorkBudget, evaluate_with_budget};
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let constant = RationalMap2::new([p(vec![vec![2]]), p(vec![vec![3]])], p(vec![vec![1]]));
    let query = source
        .point_at(&alpha())
        .unwrap()
        .rational_image(&constant)
        .unwrap();
    assert!(matches!(
        source
            .rational_image_point_locations(&constant, &query)
            .unwrap(),
        CurvePointLocations2::EntireCurve
    ));
    let punctured = RationalMap2::new(
        [p(vec![vec![-2], vec![4]]), p(vec![vec![-3], vec![6]])],
        p(vec![vec![-1], vec![2]]),
    );
    let CurvePointLocations2::Components(parts) = source
        .rational_image_point_locations(&punctured, &query)
        .unwrap()
    else {
        panic!("punctured constant domain")
    };
    assert_eq!(parts.len(), 2);
    for (part, (start, end, inclusion)) in parts.iter().zip([
        (Real::zero(), q(1, 2), [true, false]),
        (q(1, 2), Real::one(), [false, true]),
    ]) {
        let hypercurve::CurveParameterSet2::Range(range) = part.parameters().unwrap() else {
            panic!("interval")
        };
        assert_eq!(
            range.start().compare(&start.into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(range.end().compare(&end.into()).unwrap(), Ordering::Equal);
        assert_eq!(part.endpoint_inclusion(), inclusion);
    }
    let map = RationalMap2::new(
        [
            p(vec![vec![], vec![], vec![1]]),
            p(vec![vec![], vec![], vec![], vec![1]]),
        ],
        p(vec![vec![1]]),
    );
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let fresh = Curve2::from(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
        );
        let query = monomials(3, 2, 3).point_at(&alpha()).unwrap();
        let refused = evaluate_with_budget(policy, WorkBudget::ZERO, || {
            fresh.rational_image_point_locations(&map, &query)
        });
        let exhausted = refused.result.unwrap_err();
        assert_eq!(exhausted.limit, 0);
        assert_eq!(exhausted.used, 0);
        assert!(exhausted.requested > 0);
        let retry = evaluate_with_budget(
            policy,
            WorkBudget {
                refinements: 100_000,
                promotions: 10_000,
            },
            || fresh.rational_image_point_locations(&map, &query),
        );
        let result = retry.result.unwrap();
        assert_eq!(result.certainty(), CurveCertainty::Certified);
        let CurvePointLocations2::Locations(found) = result.certified().unwrap().unwrap() else {
            panic!("finite")
        };
        assert_eq!(found.len(), 1);
    }
}
#[test]
fn polynomial_map_queries_keep_removable_poles_on_the_original_branch() {
    let source = branches(
        p(vec![vec![0, 0, 1], vec![-1]]),
        [q(1, 4), 1.into()],
        [0.into(), 2.into()],
    )
    .remove(0);
    // Identity away from x=1/2. Cancellation must not admit the excluded fiber.
    let map = RationalMap2::new(
        [
            p(vec![vec![], vec![-1], vec![2]]),
            p(vec![vec![0, -1], vec![0, 2]]),
        ],
        p(vec![vec![-1], vec![2]]),
    );
    let query = source.point_at(&alpha()).unwrap();
    assert_eq!(locations(&source, &map, &query).len(), 1);
    let pole = source.point_at(&q(1, 2).into()).unwrap();
    assert!(locations(&source, &map, &pole).is_empty());
}
#[test]
fn zero_map_denominators_are_invalid_even_when_coordinates_have_no_roots() {
    let source = branch();
    let map = RationalMap2::new([p(vec![vec![1]]), p(vec![vec![1]])], p(vec![]));
    assert!(matches!(
        source.rational_image_point_locations(&map, &Point2::from_values(0, 0).into()),
        Err(hypercurve::ExactCurveError::Invalid {
            cause: hypercurve::CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
}
#[test]
fn native_circular_charts_keep_multiple_visits_and_shared_seams() {
    use hypercurve::CircularArc2;
    let quarter = Curve2::from(
        CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, 1),
            Point2::from_values(0, 0),
            false,
        )
        .unwrap(),
    );
    let major = Curve2::from(
        CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, 1),
            Point2::from_values(0, 0),
            true,
        )
        .unwrap(),
    );
    let map = RationalMap2::new(
        [p(vec![vec![], vec![], vec![1]]), p(vec![vec![0, 0, 1]])],
        p(vec![vec![1]]),
    );
    let query = quarter
        .point_at(&alpha())
        .unwrap()
        .rational_image(&map)
        .unwrap();
    assert!(query.coordinates().is_none());
    assert_eq!(locations(&major, &map, &query).len(), 3);
    assert_eq!(
        locations(&major, &map, &Point2::from_values(1, 0).into()).len(),
        2
    );
}
#[test]
fn discontinuous_spline_map_contacts_keep_both_source_charts_at_one_knot() {
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(-1, 0),
                Point2::from_values(0, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 1.into(), 2.into(), 2.into()],
        )
        .unwrap(),
    );
    let map = RationalMap2::new(
        [p(vec![vec![], vec![], vec![1]]), p(vec![])],
        p(vec![vec![1]]),
    );
    let query = Point2::from_values(1, 0).into();
    let CurvePointLocations2::Locations(found) =
        source.rational_image_point_locations(&map, &query).unwrap()
    else {
        panic!("two finite chart locations")
    };
    assert_eq!(found.len(), 2);
    for (index, (location, side)) in found
        .iter()
        .zip([
            hypercurve::CurveParameterSide2::Left,
            hypercurve::CurveParameterSide2::Right,
        ])
        .enumerate()
    {
        assert_eq!(location.span_index(), index);
        assert_eq!(
            location
                .parameter()
                .unwrap()
                .compare(&Real::one().into())
                .unwrap(),
            Ordering::Equal
        );
        let uv = source.point_at_side(&Real::one().into(), side).unwrap();
        assert!(
            uv.rational_image(&map)
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
}

#[test]
fn mixed_constant_intervals_and_isolated_visits_are_complete() {
    let zero = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    )
    .point_at(&alpha())
    .unwrap()
    .rational_image(&RationalMap2::new([p(vec![]), p(vec![])], p(vec![vec![1]])))
    .unwrap();
    assert!(zero.coordinates().is_none());
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(-1, 0),
            ],
            vec![Real::one(); 4],
            vec![0.into(), 0.into(), 1.into(), 2.into(), 3.into(), 3.into()],
        )
        .unwrap(),
    );
    let identity = RationalMap2::new(
        [p(vec![vec![], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    );
    // The controls and knots give C(t)=0 on [0,1], (t-1,0) on
    // [1,2], and (5-2t,0) on [2,3]. The exact locus is [0,1] U {5/2}.
    let result = source.rational_image_point_locations(&identity, &zero);
    let CurvePointLocations2::Components(parts) = result.unwrap() else {
        panic!("complete mixed locus")
    };
    assert_eq!(parts.len(), 2);
    let hypercurve::CurveParameterSet2::Range(range) = parts[0].parameters().unwrap() else {
        panic!("constant interval")
    };
    assert_eq!(
        range.start().compare(&Real::zero().into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Real::one().into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(parts[0].endpoint_inclusion(), [true, true]);
    let hypercurve::CurveParameterSet2::Single(root) = parts[1].parameters().unwrap() else {
        panic!("isolated visit")
    };
    assert_eq!(root.compare(&q(5, 2).into()).unwrap(), Ordering::Equal);
    for query in [&zero, &CurvePoint2::from(Point2::from_values(0, 0))] {
        let CurvePointLocations2::Components(parts) = source.point_locations(query).unwrap() else {
            panic!("ordinary complete mixed locus")
        };
        assert_eq!(parts.len(), 2);
        let hypercurve::CurveParameterSet2::Range(range) = parts[0].parameters().unwrap() else {
            panic!("constant interval")
        };
        assert_eq!(
            range.start().compare(&Real::zero().into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            range.end().compare(&Real::one().into()).unwrap(),
            Ordering::Equal
        );
        let hypercurve::CurveParameterSet2::Single(root) = parts[1].parameters().unwrap() else {
            panic!("isolated visit")
        };
        assert_eq!(root.compare(&q(5, 2).into()).unwrap(), Ordering::Equal);
    }
}

fn assert_components(
    result: CurvePointLocations2,
    expected: &[(CurveParameter2, CurveParameter2, [bool; 2])],
) {
    let CurvePointLocations2::Components(parts) = result else {
        panic!("continuous components")
    };
    assert_eq!(parts.len(), expected.len());
    for (part, (start, end, inclusion)) in parts.iter().zip(expected) {
        let hypercurve::CurveParameterSet2::Range(range) = part.parameters().unwrap() else {
            panic!("interval")
        };
        assert_eq!(range.start().compare(start).unwrap(), Ordering::Equal);
        assert_eq!(range.end().compare(end).unwrap(), Ordering::Equal);
        assert_eq!(&part.endpoint_inclusion(), inclusion);
    }
}
fn identity_map() -> RationalMap2 {
    RationalMap2::new(
        [p(vec![vec![], vec![1]]), p(vec![vec![0, 1]])],
        p(vec![vec![1]]),
    )
}
fn selected_constant_query() -> CurvePoint2 {
    monomials(1, 1, 1)
        .point_at(&alpha())
        .unwrap()
        .rational_image(&RationalMap2::new(
            [p(vec![vec![2]]), p(vec![vec![3]])],
            p(vec![vec![1]]),
        ))
        .unwrap()
}
#[test]
fn original_source_and_map_poles_split_a_constant_locus_without_filling_holes() {
    // X=t, W=2t-1; the source pole is 1/2. D=x-2 has its
    // pullback zero at 2/3. P=2D, Q=3D do not remove either guard.
    let source = Curve2::from(
        RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 0)],
            vec![(-1).into(), Real::one()],
        )
        .unwrap(),
    );
    let map = RationalMap2::new(
        [p(vec![vec![-4], vec![2]]), p(vec![vec![-6], vec![3]])],
        p(vec![vec![-2], vec![1]]),
    );
    let query = selected_constant_query();
    assert_components(
        source.rational_image_point_locations(&map, &query).unwrap(),
        &[
            (Real::zero().into(), q(1, 2).into(), [true, false]),
            (q(1, 2).into(), q(2, 3).into(), [false, false]),
            (q(2, 3).into(), Real::one().into(), [false, true]),
        ],
    );
    // A nonzero bivariate denominator can vanish on the entire source.
    // That gives an empty defined query domain, not an EntireCurve proof.
    let line = monomials(1, 1, 0);
    let nowhere = RationalMap2::new(
        [p(vec![vec![-2, 2]]), p(vec![vec![-3, 3]])],
        p(vec![vec![-1, 1]]),
    );
    assert!(
        matches!(line.rational_image_point_locations(&nowhere,&query).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
    );
}
#[test]
fn algebraic_punctures_keep_selected_endpoints_and_exact_restrictions() {
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    // All three coordinates share h=x^5+x-1. Its unique unit root
    // is alpha; cancellation leaves the original exclusion authoritative.
    let h = [-1, 1, 0, 0, 0, 1];
    let map = RationalMap2::new(
        [
            p(h.iter().map(|x| vec![2 * x]).collect()),
            p(h.iter().map(|x| vec![3 * x]).collect()),
        ],
        p(h.into_iter().map(|x| vec![x]).collect()),
    );
    let query = selected_constant_query();
    let a = alpha();
    let result = source.rational_image_point_locations(&map, &query).unwrap();
    assert_components(
        result.clone(),
        &[
            (Real::zero().into(), a.clone(), [true, false]),
            (a.clone(), Real::one().into(), [false, true]),
        ],
    );
    let at_hole = source
        .point_locations(&source.point_at(&a).unwrap())
        .unwrap();
    assert!(
        matches!(source.intersect_point_locations(&result,&at_hole).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
    );
    let selected = source
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(a.clone(), Real::one().into()).unwrap(),
        )
        .unwrap();
    assert_components(
        selected
            .rational_image_point_locations(&map, &query)
            .unwrap(),
        &[(a, Real::one().into(), [false, true])],
    );
}
#[test]
fn endpoint_exclusions_survive_intersection_and_reversal() {
    let source = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let query = selected_constant_query();
    // Constant (2,3) for x != 0, with the authored endpoint still absent.
    let map = RationalMap2::new(
        [p(vec![vec![], vec![2]]), p(vec![vec![], vec![3]])],
        p(vec![vec![], vec![1]]),
    );
    let result = source.rational_image_point_locations(&map, &query).unwrap();
    assert_components(
        result.clone(),
        &[(Real::zero().into(), Real::one().into(), [false, true])],
    );
    let start = source
        .point_locations(&Point2::from_values(0, 0).into())
        .unwrap();
    assert!(
        matches!(source.intersect_point_locations(&result,&start).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
    );
    let end = source
        .point_locations(&Point2::from_values(1, 0).into())
        .unwrap();
    let CurvePointLocations2::Locations(points) =
        source.intersect_point_locations(&result, &end).unwrap()
    else {
        panic!("one endpoint")
    };
    assert_eq!(points.len(), 1);
    assert_eq!(
        points[0]
            .parameter()
            .unwrap()
            .compare(&Real::one().into())
            .unwrap(),
        Ordering::Equal
    );
    assert_components(
        source
            .reversed()
            .unwrap()
            .rational_image_point_locations(&map, &query)
            .unwrap(),
        &[(Real::zero().into(), Real::one().into(), [true, false])],
    );
    // Intersecting a locus with itself preserves each original puncture.
    assert_components(
        source.intersect_point_locations(&result, &result).unwrap(),
        &[(Real::zero().into(), Real::one().into(), [false, true])],
    );
}
#[test]
fn parameter_intersection_distinguishes_discontinuous_source_germs() {
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 0),
                Point2::from_values(-1, 0),
                Point2::from_values(0, 0),
            ],
            vec![Real::one(); 4],
            vec![
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::one(),
                2.into(),
                2.into(),
            ],
        )
        .unwrap(),
    );
    let positive = source
        .rational_image_point_locations(&identity_map(), &Point2::from_values(1, 0).into())
        .unwrap();
    let negative = source
        .rational_image_point_locations(&identity_map(), &Point2::from_values(-1, 0).into())
        .unwrap();
    assert!(
        matches!(source.intersect_point_locations(&positive,&negative).unwrap(),CurvePointLocations2::Locations(points) if points.is_empty())
    );
    let folded = source
        .rational_image_point_locations(
            &RationalMap2::new(
                [p(vec![vec![], vec![], vec![1]]), p(vec![])],
                p(vec![vec![1]]),
            ),
            &Point2::from_values(1, 0).into(),
        )
        .unwrap();
    for (expected, locus) in [(0, positive), (1, negative)] {
        let CurvePointLocations2::Locations(points) =
            source.intersect_point_locations(&folded, &locus).unwrap()
        else {
            panic!("one source germ")
        };
        assert_eq!(points.len(), 1);
        assert_eq!(points[0].span_index(), expected);
    }
}

#[test]
fn ordinary_constant_source_queries_keep_original_punctures() {
    let source = Curve2::from(
        RationalBezier2::try_new(
            vec![Point2::from_values(2, 3); 2],
            vec![(-1).into(), 1.into()],
        )
        .unwrap(),
    );
    let selected = selected_constant_query();
    for query in [&selected, &CurvePoint2::from(Point2::from_values(2, 3))] {
        assert_components(
            source.point_locations(query).unwrap(),
            &[
                (Real::zero().into(), q(1, 2).into(), [true, false]),
                (q(1, 2).into(), Real::one().into(), [false, true]),
            ],
        );
    }
}
