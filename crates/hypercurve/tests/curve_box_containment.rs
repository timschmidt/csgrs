//! Exact curve-in-box containment against independently derived extrema.
//!
//! The motivating fixture is the cubic with controls `(8, 0), (9, 3),
//! (11, 3), (12, 0)`, which traces `y = 9t(1 - t)`. Its control hull reaches
//! `y = 3` and its image only `y = 9/4`, at `t = 1/2`.

use std::cmp::Ordering;

use hypercurve::{
    Aabb2, Axis2, BezierFlatteningOptions, BoxBoundary2, CircularArc2, Classification,
    CubicBezier2, Curve2, CurveBoxContainment2, CurveBoxEscape2, CurveCertainty, CurveGeometry2,
    CurveParameter2, CurvePath2, CurvePoint2, ExactCurveError, LineSeg2, Point2, PredicatePolicy,
    RationalBezier2, Real, UncertaintyReason, evaluate_under,
};

const POLICIES: [PredicatePolicy; 2] = [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512];

fn ratio(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn tiny(exponent: i64) -> Real {
    Real::from(2).powi_i64(-exponent).unwrap()
}

fn compare(left: &Real, right: &Real) -> Ordering {
    hyperlimit::compare_reals(left, right, PredicatePolicy::STRICT)
        .value()
        .expect("test oracle comparisons are decidable")
}

fn same_point(left: &Point2, right: &Point2) -> bool {
    compare(left.x(), right.x()) == Ordering::Equal
        && compare(left.y(), right.y()) == Ordering::Equal
}

fn witness_coordinates(escape: &CurveBoxEscape2) -> &Point2 {
    escape
        .point()
        .coordinates()
        .expect("witnesses on native curves have stored coordinates")
}

/// The closed box `[min_x, max_x] x [min_y, max_y]`.
fn closed_box(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Aabb2 {
    Aabb2::new_unchecked(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
}

/// A box bounded only by `y <= max_y` within the fixtures' extent.
fn below(max_y: Real) -> Aabb2 {
    closed_box(Real::from(-100), Real::from(-100), Real::from(100), max_y)
}

fn fixture_cubic() -> CubicBezier2 {
    CubicBezier2::new(point(8, 0), point(9, 3), point(11, 3), point(12, 0))
}

/// Evaluates `query` under `policy` and requires a certified result.
fn certified<T>(policy: PredicatePolicy, query: impl FnOnce() -> T) -> T {
    let provisional = evaluate_under(policy, query);
    assert_eq!(provisional.certainty(), CurveCertainty::Certified);
    provisional.into_unverified()
}

fn path_containment(path: &CurvePath2, envelope: &Aabb2) -> Vec<CurveBoxContainment2> {
    POLICIES
        .into_iter()
        .map(|policy| certified(policy, || path.box_containment(envelope)).unwrap())
        .collect()
}

fn assert_contained(path: &CurvePath2, envelope: &Aabb2) {
    for relation in path_containment(path, envelope) {
        assert_eq!(relation, CurveBoxContainment2::Contained);
    }
}

/// Requires an escape beyond `boundary` on `axis` at exactly `witness`.
fn assert_escapes_at(
    path: &CurvePath2,
    envelope: &Aabb2,
    axis: Axis2,
    boundary: BoxBoundary2,
    witness: &Point2,
) -> usize {
    let mut index = None;
    for relation in path_containment(path, envelope) {
        let CurveBoxContainment2::Escapes(escape) = relation else {
            panic!("the path leaves the box");
        };
        assert_eq!(escape.axis(), axis);
        assert_eq!(escape.boundary(), boundary);
        assert!(same_point(witness_coordinates(&escape), witness));
        assert_eq!(
            *index.get_or_insert(escape.curve_index()),
            escape.curve_index()
        );
    }
    index.unwrap()
}

#[test]
fn cubic_containment_decides_the_image_not_the_control_hull() {
    let curve = Curve2::from(fixture_cubic());
    let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
    // The cached enclosure is the control hull: it rejects `y <= 5/2`.
    assert_eq!(
        compare(path.bounds().unwrap().max_y(), &Real::from(3)),
        Ordering::Equal
    );
    assert_eq!(
        compare(path.bounds().unwrap().max_y(), &ratio(5, 2)),
        Ordering::Greater
    );

    let peak = Point2::new(Real::from(10), ratio(9, 4));
    assert!(same_point(&fixture_cubic().point_at(ratio(1, 2)), &peak));
    assert_contained(&path, &below(ratio(5, 2)));
    assert_contained(&path, &below(ratio(9, 4)));
    assert_contained(&path, &below(ratio(9, 4) + tiny(100)));
    for bound in [Real::from(2), ratio(9, 4) - tiny(100)] {
        assert_escapes_at(&path, &below(bound), Axis2::Y, BoxBoundary2::Maximum, &peak);
    }
    // A single curve answers the same query.
    assert_eq!(
        curve.box_containment(&below(ratio(5, 2))).unwrap(),
        CurveBoxContainment2::Contained
    );
    // Endpoint escapes are found without extremum candidates.
    let left = closed_box(Real::from(9), Real::from(-1), Real::from(20), Real::from(5));
    assert_escapes_at(&path, &left, Axis2::X, BoxBoundary2::Minimum, &point(8, 0));
}

#[test]
fn containment_is_independent_of_subdivision_and_traversal() {
    let curve = Curve2::from(fixture_cubic());
    let cut = |curve: &Curve2, parameter: Real| curve.split_at(CurveParameter2::from(parameter));
    let (left_half, right_half) = cut(&curve, ratio(1, 2)).unwrap();
    let (left_third, right_third) = cut(&curve, ratio(1, 3)).unwrap();
    let (middle, tail) = cut(&right_third, ratio(5, 8)).unwrap();
    let spline = Curve2::try_polynomial_bspline(
        3,
        fixture_cubic()
            .control_points()
            .into_iter()
            .cloned()
            .collect(),
        [0, 0, 0, 0, 1, 1, 1, 1].map(Real::from).to_vec(),
    )
    .unwrap();
    let Some(CurveGeometry2::PolynomialBSpline(spline)) = spline.geometry() else {
        panic!("a clamped cubic spline keeps its spline carrier");
    };
    let knotted = |knots: &[Real]| {
        let mut spline = spline.clone();
        for knot in knots {
            spline = spline.insert_knot(knot.clone()).unwrap();
        }
        Curve2::from(spline)
    };
    let paths = [
        vec![curve.clone()],
        vec![left_half.clone(), right_half.clone()],
        vec![left_third.clone(), right_third.clone()],
        vec![left_third, middle, tail],
        vec![knotted(&[ratio(1, 2)])],
        vec![knotted(&[ratio(1, 3), ratio(3, 4)])],
    ]
    .map(|curves| CurvePath2::try_new(curves).unwrap());
    let reversed = paths.clone().map(|path| path.reversed().unwrap());

    // The control-hull decision depends on the representation.
    let hull_admits = |path: &CurvePath2| {
        compare(path.bounds().unwrap().max_y(), &ratio(5, 2)) != Ordering::Greater
    };
    assert!(!hull_admits(&paths[0]));
    assert!(hull_admits(&paths[1]));
    assert!(!hull_admits(&paths[2]));
    let spans = paths[5].curves()[0].native_bezier_fragments().unwrap();
    assert_eq!(spans.len(), 3);
    let CurveGeometry2::CubicBezier(middle) = spans[1].curve() else {
        panic!("a cubic spline span is a cubic Bezier");
    };
    let Classification::Decided(middle_hull) = middle.control_hull_box() else {
        panic!("rational controls are ordered");
    };
    assert_eq!(
        compare(middle_hull.max_y(), &ratio(29, 12)),
        Ordering::Equal
    );

    let peak = Point2::new(Real::from(10), ratio(9, 4));
    for path in paths.iter().chain(&reversed) {
        // 7/3 lies below the hull of the middle span of the second spline,
        // [1/3, 3/4], whose controls reach 29/12.
        for bound in [ratio(5, 2), ratio(7, 3), ratio(9, 4)] {
            assert_contained(path, &below(bound));
        }
        for bound in [Real::from(2), ratio(9, 4) - tiny(100)] {
            assert_escapes_at(path, &below(bound), Axis2::Y, BoxBoundary2::Maximum, &peak);
        }
    }
}

#[test]
fn minimum_sides_and_the_x_axis_use_the_same_extrema() {
    let mirrored = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        point(8, 0),
        point(9, -3),
        point(11, -3),
        point(12, 0),
    ))])
    .unwrap();
    let above = |min_y: Real| closed_box(Real::from(-100), min_y, Real::from(100), Real::from(100));
    assert_contained(&mirrored, &above(ratio(-5, 2)));
    assert_contained(&mirrored, &above(ratio(-9, 4)));
    assert_escapes_at(
        &mirrored,
        &above(Real::from(-2)),
        Axis2::Y,
        BoxBoundary2::Minimum,
        &Point2::new(Real::from(10), ratio(-9, 4)),
    );

    let transposed = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        point(0, 8),
        point(3, 9),
        point(3, 11),
        point(0, 12),
    ))])
    .unwrap();
    let left_of =
        |max_x: Real| closed_box(Real::from(-100), Real::from(-100), max_x, Real::from(100));
    assert_contained(&transposed, &left_of(ratio(5, 2)));
    assert_escapes_at(
        &transposed,
        &left_of(Real::from(2)),
        Axis2::X,
        BoxBoundary2::Maximum,
        &Point2::new(ratio(9, 4), Real::from(10)),
    );
}

#[test]
fn an_irrational_extremum_is_decided_without_subdivision() {
    // y(t) = 9t - 15t^2 + 6t^3 peaks at t = (5 - sqrt 7)/6 with
    // y = 5/9 + 7 sqrt(7)/18, derived by hand from the power basis.
    let path = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        point(0, 0),
        point(1, 3),
        point(2, 1),
        point(3, 0),
    ))])
    .unwrap();
    let sqrt7 = Real::from(7).sqrt().unwrap();
    let peak_y = ratio(5, 9) + ratio(7, 18) * sqrt7.clone();
    let peak_x = ratio(5, 2) - (sqrt7 / Real::from(2)).unwrap();
    assert_eq!(
        compare(path.bounds().unwrap().max_y(), &Real::from(3)),
        Ordering::Equal
    );

    assert_contained(&path, &below(peak_y.clone() + tiny(200)));
    let relations = path_containment(&path, &below(peak_y.clone() - tiny(200)));
    for relation in relations {
        let CurveBoxContainment2::Escapes(escape) = relation else {
            panic!("the peak lies above the bound");
        };
        assert_eq!(escape.boundary(), BoxBoundary2::Maximum);
        assert!(same_point(
            witness_coordinates(&escape),
            &Point2::new(peak_x.clone(), peak_y.clone())
        ));
    }
    // The peak itself touches a closed box; refinement cannot prove this.
    assert_contained(&path, &below(peak_y));
}

#[test]
fn circular_arcs_use_cardinal_extrema() {
    let arc = |start: Point2, end: Point2, clockwise: bool| {
        CurvePath2::try_new(vec![Curve2::from(
            CircularArc2::try_from_center(start, end, point(0, 0), clockwise).unwrap(),
        )])
        .unwrap()
    };
    let upper = arc(point(1, 0), point(-1, 0), false);
    assert_contained(&upper, &below(Real::one()));
    assert_escapes_at(
        &upper,
        &below(ratio(1, 2)),
        Axis2::Y,
        BoxBoundary2::Maximum,
        &point(0, 1),
    );
    // The clockwise arc between the same points is the lower half.
    assert_contained(&arc(point(1, 0), point(-1, 0), true), &below(Real::zero()));

    let root_two = Real::from(2).sqrt().unwrap();
    let wide = arc(point(1, 1), point(-1, 1), false);
    assert_contained(&wide, &below(ratio(3, 2)));
    assert_escapes_at(
        &wide,
        &below(ratio(7, 5)),
        Axis2::Y,
        BoxBoundary2::Maximum,
        &Point2::new(Real::zero(), root_two),
    );
    // A minor arc has no interior cardinal point: its endpoints decide.
    let minor = arc(point(3, 4), point(4, 3), true);
    let unit = closed_box(Real::from(3), Real::from(3), Real::from(4), Real::from(4));
    assert_contained(&minor, &unit);
}

#[test]
fn rational_extrema_complete_and_preserve_first_escape_order() {
    // y(t) = 18t(1 - t) / D(t) with weights 1, 2, 2, 1 peaks at 18/7 < 3.
    let rational = Curve2::from(
        RationalBezier2::try_new(
            vec![point(0, 0), point(1, 3), point(2, 3), point(3, 0)],
            [1, 2, 2, 1].map(Real::from).to_vec(),
        )
        .unwrap(),
    );
    let alone = CurvePath2::try_new(vec![rational.clone()]).unwrap();
    assert_contained(&alone, &below(Real::from(3)));
    assert_contained(&alone, &below(ratio(14, 5)));
    let peak = Point2::new(ratio(3, 2), ratio(18, 7));
    assert_escapes_at(
        &alone,
        &below(Real::from(2)),
        Axis2::Y,
        BoxBoundary2::Maximum,
        &peak,
    );

    let path = CurvePath2::try_new(vec![
        rational,
        Curve2::from(LineSeg2::try_new(point(3, 0), point(4, 0)).unwrap()),
        Curve2::from(CubicBezier2::new(
            point(4, 0),
            point(5, 3),
            point(7, 3),
            point(8, 0),
        )),
    ])
    .unwrap();
    let index = assert_escapes_at(
        &path,
        &below(Real::from(2)),
        Axis2::Y,
        BoxBoundary2::Maximum,
        &peak,
    );
    assert_eq!(index, 0);
    assert_contained(&path, &below(ratio(14, 5)));
}

#[test]
fn unsupported_extrema_stay_typed_and_a_definite_escape_wins() {
    // This procedural zero-distance parallel has the same cubic image but
    // remains an analytic carrier. Its interior extrema are a separate route.
    let source = CubicBezier2::new(point(0, 0), point(1, 3), point(2, 3), point(3, 0));
    let curve = Curve2::try_analytic_parallel(
        source.parallel_left(Real::zero()).unwrap(),
        hypercurve::BezierParameterRange2::try_new(
            hypercurve::BezierParameter2::Exact(Real::zero()),
            hypercurve::BezierParameter2::Exact(Real::one()),
        )
        .unwrap(),
    )
    .unwrap();
    for policy in POLICIES {
        let error = certified(policy, || curve.box_containment(&below(Real::from(2)))).unwrap_err();
        let ExactCurveError::Blocked(blocker) = error else {
            panic!("unsupported remains a blocker");
        };
        assert_eq!(blocker.reason(), UncertaintyReason::Unsupported);
    }
    let path = CurvePath2::try_new(vec![
        curve.clone(),
        LineSeg2::try_new(point(3, 0), point(4, 0)).unwrap().into(),
        CubicBezier2::new(point(4, 0), point(5, 3), point(7, 3), point(8, 0)).into(),
    ])
    .unwrap();
    assert_eq!(
        assert_escapes_at(
            &path,
            &below(Real::from(2)),
            Axis2::Y,
            BoxBoundary2::Maximum,
            &Point2::new(6.into(), ratio(9, 4))
        ),
        2
    );
    assert_contained(&path, &below(ratio(14, 5)));
}

#[test]
fn selected_endpoints_of_implicit_branches_are_witnesses() {
    use hypersolve::bivariate_topology::{
        BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
    };
    use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
    // x^3 - y = 0 for 1 <= x <= 2 and 0 <= y <= 2 runs from (1, 1) to the
    // selected point (2^(1/3), 2), which has no stored coordinates.
    let topology = decompose_bivariate_curve(
        &BivariatePolynomial::new(vec![
            [0, 0, 0, 1].map(Real::from).to_vec(),
            vec![Real::from(-1)],
        ]),
        &BivariateCurveDomain {
            first: [Real::from(1), Real::from(2)],
            second: [Real::from(0), Real::from(2)],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set = hypercurve::PolynomialCurveSet2::from_topology(
        topology,
        AlgebraicFiberRootIsolationConfig::default(),
    )
    .unwrap();
    let [curve] = set.curves() else {
        panic!("the cubic has one branch in the domain");
    };
    let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
    let corner = curve.end();
    let far = if corner.coordinates().is_none() {
        corner
    } else {
        curve.start()
    };
    assert!(far.coordinates().is_none());
    // 5/4 < 2^(1/3) < 13/10.
    let narrow = closed_box(Real::zero(), Real::zero(), ratio(5, 4), Real::from(3));
    for relation in path_containment(&path, &narrow) {
        let CurveBoxContainment2::Escapes(escape) = relation else {
            panic!("the selected endpoint lies beyond x = 5/4");
        };
        assert_eq!(
            (escape.axis(), escape.boundary()),
            (Axis2::X, BoxBoundary2::Maximum)
        );
        assert!(escape.point().coincides_with(&far).unwrap());
    }
    assert_contained(
        &path,
        &closed_box(Real::zero(), Real::zero(), Real::from(3), Real::from(3)),
    );
}

#[test]
fn flattening_bounds_set_distance_not_same_parameter_distance() {
    // C(t) = (t^3, 0) lies on its chord, so one chord certifies any error
    // bound. The chord's own parameterization L(t) = (t, 0) is 3/8 away at
    // t = 1/2: the flattening certificate is not a same-parameter bound.
    let cubic = CubicBezier2::new(point(0, 0), point(0, 0), point(0, 0), point(1, 0));
    let options = BezierFlatteningOptions::try_new(ratio(1, 16), 12).unwrap();
    let polyline = Curve2::from(cubic.clone())
        .segment_certified(&options)
        .unwrap();
    assert_eq!(polyline.points().len(), 2);
    assert!(same_point(&polyline.points()[0], &point(0, 0)));
    assert!(same_point(&polyline.points()[1], &point(1, 0)));
    let half = ratio(1, 2);
    let on_curve = cubic.point_at(half.clone());
    let same_parameter = half - on_curve.x().clone();
    assert_eq!(compare(&same_parameter, &ratio(3, 8)), Ordering::Equal);
    assert_eq!(
        compare(&same_parameter, polyline.certificate().max_error()),
        Ordering::Greater
    );
}

#[test]
fn rational_cubic_containment_matches_its_exact_peak() {
    let source = Curve2::from(
        RationalBezier2::try_new(
            [(0, 0), (1, 3), (2, 3), (3, 0)]
                .map(|(x, y)| Point2::new(x.into(), y.into()))
                .to_vec(),
            [1, 2, 2, 1].map(Real::from).to_vec(),
        )
        .unwrap(),
    );
    let peak = Point2::new(ratio(3, 2), ratio(18, 7));
    assert!(
        source
            .point_at(&CurveParameter2::from(ratio(1, 2)))
            .unwrap()
            .coincides_with(&CurvePoint2::from(peak.clone()))
            .unwrap()
    );
    let (left_half, right_half) = source.split_at(ratio(1, 2).into()).unwrap();
    let (left_third, right_third) = source.split_at(ratio(1, 3).into()).unwrap();
    let original = CurvePath2::try_new(vec![source.clone()]).unwrap();
    let reversed = original.reversed().unwrap();
    let paths = [
        ("native", original),
        ("reversed", reversed),
        (
            "halves",
            CurvePath2::try_new(vec![left_half, right_half]).unwrap(),
        ),
        (
            "thirds",
            CurvePath2::try_new(vec![left_third, right_third]).unwrap(),
        ),
    ];
    drop(source);
    let tiny = Real::from(2).powi_i64(-200).unwrap();
    let bounds = [
        ("loose", ratio(14, 5), true),
        ("tangent", ratio(18, 7), true),
        ("above", ratio(18, 7) + &tiny, true),
        ("below", ratio(18, 7) - &tiny, false),
        ("outside", Real::from(2), false),
    ];
    let mut correct = 0;
    let mut total = 0;
    for (path_name, path) in paths {
        for (bound_name, max_y, contained) in &bounds {
            let envelope = Aabb2::new_unchecked(
                Point2::new((-10).into(), (-10).into()),
                Point2::new(10.into(), max_y.clone()),
            );
            for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
                total += 1;
                let outcome = evaluate_under(policy, || path.box_containment(&envelope));
                assert_eq!(outcome.certainty(), CurveCertainty::Certified);
                match outcome.into_unverified() {
                    Ok(CurveBoxContainment2::Contained) => {
                        assert!(*contained, "false containment at {path_name}/{bound_name}");
                        correct += 1;
                        println!("R23 {path_name} {bound_name} {policy:?} Contained");
                    }
                    Ok(CurveBoxContainment2::Escapes(witness)) => {
                        assert!(!*contained, "false escape at {path_name}/{bound_name}");
                        assert_eq!(
                            (witness.axis(), witness.boundary()),
                            (Axis2::Y, BoxBoundary2::Maximum)
                        );
                        // A split endpoint may be the first escaping member.
                        // The contract does not require the global maximum.
                        let independent_candidates =
                            [peak.clone(), Point2::new(ratio(17, 15), ratio(12, 5))];
                        assert!(independent_candidates.iter().any(|expected| {
                            witness
                                .point()
                                .coincides_with(&CurvePoint2::from(expected.clone()))
                                .unwrap()
                        }));
                        assert_eq!(
                            witness
                                .point()
                                .compare_coordinate(
                                    &CurvePoint2::from(Point2::new(Real::zero(), max_y.clone())),
                                    Axis2::Y,
                                )
                                .unwrap(),
                            std::cmp::Ordering::Greater
                        );
                        correct += 1;
                        println!("R23 {path_name} {bound_name} {policy:?} Escapes exact member");
                    }
                    Err(error) => println!("R23 {path_name} {bound_name} {policy:?} {error:?}"),
                }
            }
        }
    }
    assert_eq!(
        correct, total,
        "every form must decide its exact image; correct={correct}, total={total}"
    );
}
