//! Complete native-span inverses over retained selected point fields.
use hypercurve::{
    BezierParameterPolynomial, Curve2, CurveParameter2, CurvePoint2, CurvePointLocations2, Point2,
    RationalBezier2, Real,
};
use hypersolve::BivariatePolynomial;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn alpha() -> CurveParameter2 {
    BezierParameterPolynomial::try_new_power_basis(
        [-1, 1, 0, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()[0]
        .clone()
        .into()
}
fn choose(n: usize, k: usize) -> i64 {
    if k > n {
        return 0;
    }
    (0..k).fold(1, |v, i| v * (n - i) as i64 / (i + 1) as i64)
}
fn monomials(degree: usize, x: usize, y: usize) -> Curve2 {
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
fn parameters(curve: &Curve2, point: &CurvePoint2) -> Vec<CurveParameter2> {
    match curve.point_locations(point).unwrap() {
        CurvePointLocations2::Locations(locations) => {
            locations.iter().map(|l| l.parameter().unwrap()).collect()
        }
        CurvePointLocations2::EntireCurve | CurvePointLocations2::Components(_) => {
            panic!("expected finitely many preimages")
        }
    }
}
fn agrees(curve: &Curve2, point: &CurvePoint2, count: usize) -> Vec<CurveParameter2> {
    let locations = parameters(curve, point);
    assert_eq!(locations.len(), count);
    for t in &locations {
        assert!(curve.point_at(t).unwrap().coincides_with(point).unwrap());
        assert!(t.scalar().is_none());
        let replay = curve
            .point_at(t)
            .unwrap()
            .definition()
            .unwrap()
            .into_point()
            .unwrap();
        assert!(replay.coincides_with(point).unwrap());
    }
    locations
}

#[test]
fn independent_selected_queries_invert_polynomial_and_rational_spans() {
    let a = alpha();
    let query = monomials(6, 4, 6).point_at(&a).unwrap();
    assert!(query.coordinates().is_none());
    let expected = monomials(2, 2, 2)
        .point_at(&a)
        .unwrap()
        .affine_form(Real::one(), Real::zero(), Real::zero())
        .parameter()
        .unwrap();
    let polynomial = monomials(3, 2, 3);
    let t = agrees(&polynomial, &query, 1).remove(0);
    let restricted = polynomial
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(expected.clone(), Real::one().into())
                .unwrap(),
        )
        .unwrap();
    agrees(&restricted, &query, 1);
    let half_parameter = monomials(2, 2, 2)
        .point_at(&a)
        .unwrap()
        .affine_form(q(1, 2), Real::zero(), Real::zero())
        .parameter()
        .unwrap();
    let below = polynomial.point_at(&half_parameter).unwrap();
    assert!(parameters(&restricted, &below).is_empty());
    assert_eq!(t.compare(&expected).unwrap(), Ordering::Equal);
    let rational = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 5), 0.into()),
                Point2::new(q(1, 2), q(1, 2)),
            ],
            vec![1.into(), q(4, 3), q(5, 3), 2.into()],
        )
        .unwrap(),
    );
    // (X,Y)=(a^4,a^6) -> (X^2/(X+Y),XY/(X+Y)). The original
    // denominator remains guarded; no independent a coordinate is recovered.
    let p = |rows: Vec<Vec<Real>>| BivariatePolynomial::new(rows);
    let mapped = query
        .rational_image(&hypercurve::RationalMap2::new(
            [
                p(vec![vec![], vec![], vec![1.into()]]),
                p(vec![vec![], vec![0.into(), 1.into()]]),
            ],
            p(vec![vec![0.into(), 1.into()], vec![1.into()]]),
        ))
        .unwrap();
    assert_eq!(
        agrees(&rational, &mapped, 1)[0].compare(&expected).unwrap(),
        Ordering::Equal
    );
    let delta = q(1, 1_i64 << 60);
    let shifted = mapped
        .rational_image(&hypercurve::RationalMap2::new(
            [
                p(vec![vec![], vec![1.into()]]),
                p(vec![vec![delta, 1.into()]]),
            ],
            p(vec![vec![1.into()]]),
        ))
        .unwrap();
    assert!(parameters(&rational, &shifted).is_empty());
    for (curve, point) in [
        (polynomial.reversed().unwrap(), &query),
        (rational.reversed().unwrap(), &mapped),
    ] {
        // Check by the defining image, independently of traversal order.
        agrees(&curve, point, 1);
    }
}

#[test]
fn selected_inverse_keeps_all_visits_constant_coordinates_and_trim_domains() {
    let query = Curve2::from(
        hypercurve::LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
            .unwrap(),
    )
    .point_at(&alpha())
    .unwrap();
    let retraced = Curve2::from(hypercurve::QuadraticBezier2::new(
        Point2::from_values(1, 0),
        Point2::from_values(-1, 0),
        Point2::from_values(1, 0),
    ));
    let visits = agrees(&retraced, &query, 2);
    assert!(
        visits
            .iter()
            .any(|t| t.compare(&q(1, 2).into()).unwrap() == Ordering::Less)
    );
    assert!(
        visits
            .iter()
            .any(|t| t.compare(&q(1, 2).into()).unwrap() == Ordering::Greater)
    );
    let restricted = retraced
        .restrict_source(
            hypercurve::CurveParameterRange2::try_new(Real::zero().into(), q(1, 2).into()).unwrap(),
        )
        .unwrap();
    agrees(&restricted, &query, 1);
    let vertical = Curve2::from(hypercurve::QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(0, 0),
        Point2::from_values(0, 1),
    ));
    let swapped = query
        .rational_image(&hypercurve::RationalMap2::new(
            [
                BivariatePolynomial::new(vec![]),
                BivariatePolynomial::new(vec![vec![], vec![1.into()]]),
            ],
            BivariatePolynomial::new(vec![vec![1.into()]]),
        ))
        .unwrap();
    agrees(&vertical, &swapped, 1);
    let constant = Curve2::from(
        RationalBezier2::try_new(vec![Point2::from_values(0, 0); 3], vec![Real::one(); 3]).unwrap(),
    );
    assert!(parameters(&constant, &query).is_empty());
    let zero = swapped
        .affine_form(Real::one(), Real::zero(), Real::zero())
        .compare_to_real(&Real::zero());
    assert!(matches!(zero, Ok(Ordering::Equal)));
}

#[test]
fn retained_native_points_keep_all_visits_on_folded_and_multispan_curves() {
    // (x,y)=((t-1/2)^2,(t-1/2)^4). The selected source image has two
    // independent visits given by the two roots of 8t^2-8t+1.
    let folded = Curve2::from(
        RationalBezier2::try_new(
            [q(1, 4), Real::zero(), q(-1, 12), Real::zero(), q(1, 4)]
                .into_iter()
                .enumerate()
                .map(|(i, x)| Point2::new(x, q(if i % 2 == 0 { 1 } else { -1 }, 16)))
                .collect(),
            vec![Real::one(); 5],
        )
        .unwrap(),
    );
    let roots: Vec<CurveParameter2> = BezierParameterPolynomial::try_new_power_basis(
        [1, -8, 8].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap()
    .into_iter()
    .map(Into::into)
    .collect();
    assert_eq!(roots.len(), 2);
    let query = zero_distance_source_point(&folded, &roots[0]);
    assert!(query.coordinates().is_none());
    let visits = agrees(&folded, &query, 2);
    for (actual, expected) in visits.iter().zip(&roots) {
        assert_eq!(actual.compare(expected).unwrap(), Ordering::Equal);
    }

    // The second span traces the first in reverse. Recognizing the selected
    // point on the first chart must still enumerate the second span.
    let p = Point2::from_values;
    let retraced = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(1, 2), p(2, 0), p(1, 2), p(0, 0)],
        vec![Real::one(); 5],
        [0, 0, 0, 1, 1, 2, 2, 2]
            .into_iter()
            .map(Real::from)
            .collect(),
    )
    .unwrap();
    let parameter = alpha();
    let query = zero_distance_source_point(&retraced, &parameter);
    assert!(query.coordinates().is_none());
    let visits = agrees(&retraced, &query, 2);
    assert_eq!(visits[0].compare(&parameter).unwrap(), Ordering::Equal);
    assert_eq!(
        visits[1].compare(&Real::one().into()).unwrap(),
        Ordering::Greater
    );
}

#[test]
fn retained_native_point_does_not_exclude_an_exterior_preimage() {
    // The source (t²,t⁴) is injective on [0,1], but evaluating it at -t
    // in an explicitly admitted exterior chart gives the same image.
    let source = monomials(4, 2, 4);
    let query = zero_distance_source_point(&source, &alpha());
    assert!(query.coordinates().is_none());
    let exterior = Curve2::try_from_bezier_range(
        source.geometry().unwrap().clone(),
        hypercurve::CurveParameterRange2::try_new(Real::from(-1).into(), Real::zero().into())
            .unwrap(),
    )
    .unwrap();
    let visits = agrees(&exterior, &query, 1);
    assert_eq!(
        visits[0].compare(&Real::zero().into()).unwrap(),
        Ordering::Less
    );
}

fn zero_distance_source_point(curve: &Curve2, parameter: &CurveParameter2) -> CurvePoint2 {
    let span = &curve.native_bezier_fragments().unwrap()[0];
    let source = match span.curve() {
        hypercurve::CurveGeometry2::RationalBezier(source) => source,
        hypercurve::CurveGeometry2::QuadraticBezier(source) => RationalBezier2::try_new(
            source.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        )
        .unwrap(),
        hypercurve::CurveGeometry2::RationalQuadraticBezier(source) => source.into(),
        other => panic!("unexpected source span: {:?}", other.family()),
    };
    let source = source.parallel_left(Real::zero()).unwrap();
    hypercurve::CurvePointDefinition2::Parallel {
        source,
        parameter: parameter.clone(),
    }
    .into_point()
    .unwrap()
}

#[test]
fn retained_native_inverse_respects_trimmed_and_reversed_domains() {
    use hypercurve::{CurveParameterRange2, PredicatePolicy, WorkBudget, evaluate_with_budget};

    let source = monomials(4, 2, 4);
    let parameter = alpha();
    let query = zero_distance_source_point(&source, &parameter);
    let retained = source
        .restrict_source(
            CurveParameterRange2::try_new(parameter.clone(), Real::one().into()).unwrap(),
        )
        .unwrap();
    let omitted = source
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap(),
        )
        .unwrap();

    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (curve, count) in [
            (&retained, 1),
            (&retained.reversed().unwrap(), 1),
            (&omitted, 0),
        ] {
            let result = evaluate_with_budget(
                policy,
                WorkBudget {
                    refinements: 64,
                    promotions: 0,
                },
                || curve.point_locations(&query),
            );
            let CurvePointLocations2::Locations(locations) =
                result.result.unwrap().certified().unwrap().unwrap()
            else {
                panic!("expected finite retained preimages");
            };
            assert_eq!(locations.len(), count);
            for location in locations {
                let t = location.parameter().unwrap();
                assert_eq!(t.compare(&parameter).unwrap(), Ordering::Equal);
                assert!(curve.point_at(&t).unwrap().coincides_with(&query).unwrap());
            }
        }
    }
}

#[test]
fn selected_queries_keep_original_poles_and_entire_span_outcomes() {
    let a = alpha();
    let point = Curve2::from(
        hypercurve::LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
            .unwrap(),
    )
    .point_at(&a)
    .unwrap();
    let zero = point
        .rational_image(&hypercurve::RationalMap2::new(
            [
                BivariatePolynomial::new(vec![]),
                BivariatePolynomial::new(vec![]),
            ],
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        ))
        .unwrap();
    assert!(zero.coordinates().is_none());
    let constant = Curve2::from(
        RationalBezier2::try_new(vec![Point2::from_values(0, 0); 3], vec![Real::one(); 3]).unwrap(),
    );
    assert!(matches!(
        constant.point_locations(&zero).unwrap(),
        CurvePointLocations2::EntireCurve
    ));
    // Homogeneous (X,W)=(t(2t-1)^2,(2t-1)^2). The authored
    // denominator excludes t=1/2 even though the supporting image has a limit.
    let punctured = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(-1, 0),
                Point2::from_values(2, 0),
                Point2::from_values(1, 0),
            ],
            vec![1.into(), q(-1, 3), q(-1, 3), 1.into()],
        )
        .unwrap(),
    );
    agrees(&punctured, &point, 1);
    let half = Curve2::from(
        RationalBezier2::try_new(
            (0..=5)
                .map(|i| {
                    Point2::new(
                        q(i, 10) + if i == 5 { q(1, 2) } else { Real::zero() },
                        Real::zero(),
                    )
                })
                .collect(),
            vec![Real::one(); 6],
        )
        .unwrap(),
    )
    .point_at(&a)
    .unwrap();
    assert!(half.coordinates().is_none());
    assert_eq!(
        half.affine_form(1.into(), 0.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
    assert!(parameters(&punctured, &half).is_empty());
}

#[test]
fn selected_spline_locations_keep_authored_knots_and_shared_seams() {
    let curve = Curve2::try_nurbs(
        2,
        vec![
            Point2::from_values(0, 0),
            Point2::new(q(1, 2), 0.into()),
            Point2::from_values(1, 0),
            Point2::new(q(3, 2), 0.into()),
            Point2::from_values(2, 0),
        ],
        vec![Real::one(); 5],
        [-2, -2, -2, 1, 1, 4, 4, 4]
            .into_iter()
            .map(Real::from)
            .collect(),
    )
    .unwrap();
    let a = alpha();
    let query = Curve2::from(
        hypercurve::LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
            .unwrap(),
    )
    .point_at(&a)
    .unwrap();
    let expected = Curve2::from(
        hypercurve::LineSeg2::try_new(Point2::from_values(-2, 0), Point2::from_values(1, 0))
            .unwrap(),
    )
    .point_at(&a)
    .unwrap()
    .affine_form(1.into(), 0.into(), 0.into())
    .parameter()
    .unwrap();
    assert_eq!(
        agrees(&curve, &query, 1)[0].compare(&expected).unwrap(),
        Ordering::Equal
    );
    let seam = query
        .rational_image(&hypercurve::RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![1.into()]]),
                BivariatePolynomial::new(vec![]),
            ],
            BivariatePolynomial::new(vec![vec![1.into()]]),
        ))
        .unwrap();
    assert!(seam.coordinates().is_none());
    let visits = parameters(&curve, &seam);
    assert_eq!(visits.len(), 1);
    assert_eq!(
        visits[0].compare(&Real::one().into()).unwrap(),
        Ordering::Equal
    );
}

#[test]
fn selected_inverse_budget_refusal_does_not_poison_a_larger_retry() {
    use hypercurve::{CurveCertainty, PredicatePolicy, WorkBudget, evaluate_with_budget};
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let query = monomials(6, 4, 6).point_at(&alpha()).unwrap();
        let target = monomials(3, 2, 3);
        let denied =
            evaluate_with_budget(policy, WorkBudget::ZERO, || target.point_locations(&query));
        let exhausted = denied.result.unwrap_err();
        assert_eq!(exhausted.limit, 0);
        assert_eq!(exhausted.used, 0);
        assert!(exhausted.requested > 0);
        let retry = evaluate_with_budget(
            policy,
            WorkBudget {
                refinements: 100_000,
                promotions: 10_000,
            },
            || target.point_locations(&query),
        );
        let completed = retry.result.unwrap();
        assert_eq!(completed.certainty(), CurveCertainty::Certified);
        let CurvePointLocations2::Locations(locations) = completed.certified().unwrap().unwrap()
        else {
            panic!("finite preimages");
        };
        assert_eq!(locations.len(), 1);
        assert!(
            target
                .point_at(&locations[0].parameter().unwrap())
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
}

#[test]
fn near_selected_preimages_remain_distinct_and_partial_constant_spans_stay_explicit() {
    let source = monomials(2, 2, 2).point_at(&alpha()).unwrap();
    let epsilon = q(1, 1_i64 << 60);
    let point = source
        .rational_image(&hypercurve::RationalMap2::new(
            [
                BivariatePolynomial::new(vec![vec![], vec![&epsilon * &epsilon]]),
                BivariatePolynomial::new(vec![]),
            ],
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        ))
        .unwrap();
    let target = Curve2::from(hypercurve::QuadraticBezier2::new(
        Point2::from_values(1, 0),
        Point2::from_values(-2, 0),
        Point2::from_values(4, 0),
    ));
    let mut visits = agrees(&target, &point, 2);
    if visits[0].compare(&visits[1]).unwrap() == Ordering::Greater {
        visits.swap(0, 1);
    }
    let center = CurveParameter2::from(q(1, 3));
    assert_eq!(visits[0].compare(&center).unwrap(), Ordering::Less);
    assert_eq!(visits[1].compare(&center).unwrap(), Ordering::Greater);
    assert_eq!(visits[0].compare(&visits[1]).unwrap(), Ordering::Less);
    let partial = Curve2::try_nurbs(
        1,
        vec![
            Point2::from_values(0, 0),
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
        ],
        vec![Real::one(); 3],
        [0, 0, 1, 2, 2].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let zero = point
        .rational_image(&hypercurve::RationalMap2::new(
            [
                BivariatePolynomial::new(vec![]),
                BivariatePolynomial::new(vec![]),
            ],
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        ))
        .unwrap();
    // The original controls and knots give C(t)=0 on [0,1] and
    // C(t)=(t-1,0) on [1,2]. Its entire preimage is the closed [0,1].
    assert!(zero.coordinates().is_none());
    let CurvePointLocations2::Components(parts) = partial.point_locations(&zero).unwrap() else {
        panic!("one proper constant interval")
    };
    assert_eq!(parts.len(), 1);
    assert_eq!(parts[0].span_index(), 0);
    assert_eq!(parts[0].endpoint_inclusion(), [true, true]);
    let hypercurve::CurveParameterSet2::Range(range) = parts[0].parameters().unwrap() else {
        panic!("interval closure")
    };
    assert_eq!(
        range.start().compare(&Real::zero().into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        range.end().compare(&Real::one().into()).unwrap(),
        Ordering::Equal
    );
}
