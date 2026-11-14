#![cfg(feature = "interval-ranges")]

use hyperbrep::{
    GeometryError, Vector3,
    interval_ranges::{SurfaceRangeEnclosure, SurfaceRangeError},
};
use hyperbrep::{Point3, Surface, interval_ranges::prepare_rational_bezier_surface_ranges};
use hyperinterval::IntervalError;
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice::BoxN;
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, ordered_aabb3s_intersect};
use hyperreal::{Rational, Real};
use hypersolve::expression_program::ExpressionBudget;
use hypersolve::{expression_program::ExpressionError, work_budget::CancellationToken};

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn patch() -> Surface {
    Surface::rational_bezier(
        vec![
            vec![
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(1.into(), 0.into(), 0.into()),
            ],
            vec![
                Point3::new(0.into(), 2.into(), 0.into()),
                Point3::new(1.into(), 2.into(), 1.into()),
            ],
        ],
        vec![vec![1.into(); 2]; 2],
    )
    .unwrap()
}
fn enclose(surface: &Surface, domain: &BoxN<DyadicInterval, 2>) -> SurfaceRangeEnclosure {
    prepare_rational_bezier_surface_ranges(surface, &mut graph())
        .unwrap()
        .enclose(domain, -128, &mut graph(), &mut arithmetic())
        .unwrap()
}
fn contains(range: &DyadicInterval, lower: Rational, upper: Rational) {
    let r = range.to_rational(&mut arithmetic()).unwrap();
    assert!(r.contains_point(&lower), "{r:?} lacks {lower}");
    assert!(r.contains_point(&upper), "{r:?} lacks {upper}");
}
fn encloses_rational_point(range: &DyadicInterval, expected: Rational) {
    contains(range, expected.clone(), expected);
    let bounds = range.to_rational(&mut arithmetic()).unwrap();
    let (lo, hi) = bounds.bounds().unwrap();
    let tolerance = d(1, 1, -100).to_rational(&mut arithmetic()).unwrap();
    assert!(
        hi - lo <= *tolerance.bounds().unwrap().0,
        "excessive outward width: {range:?}"
    );
}
fn exact_values(
    prepared: &hyperbrep::interval_ranges::PreparedRationalBezierSurfaceRanges,
    arguments: [Real; 2],
    coordinates: [Real; 3],
    partials: [[Real; 3]; 2],
) {
    let definition = prepared.program().program().definition();
    // Independent exact oracle: normalize each arithmetic step through the
    // existing scalar quadratic field before comparing against the closed form.
    // Comparing the entire expanded native expression can exceed the scalar
    // recognizer's bounded search; that is not a failed interval certificate.
    use hypersolve::expression_program::{BinaryOperation, ExpressionNode, UnaryOperation};
    let mut nodes: Vec<Real> = Vec::new();
    for node in &definition.nodes {
        let value = match node {
            ExpressionNode::Constant(value) => value.clone(),
            ExpressionNode::Variable(symbol) => arguments[definition
                .variables
                .iter()
                .position(|id| *id == symbol.id)
                .unwrap()]
            .clone(),
            ExpressionNode::Binary {
                operation,
                left,
                right,
            } => match operation {
                BinaryOperation::Add => &nodes[*left] + &nodes[*right],
                BinaryOperation::Sub => &nodes[*left] - &nodes[*right],
                BinaryOperation::Mul => &nodes[*left] * &nodes[*right],
                BinaryOperation::Div => (&nodes[*left] / &nodes[*right]).unwrap(),
            },
            ExpressionNode::Unary {
                operation: UnaryOperation::Neg,
                input,
            } => -nodes[*input].clone(),
            ExpressionNode::Unary {
                operation: UnaryOperation::PowI(exponent),
                input,
            } => nodes[*input].clone().powi((*exponent).into()).unwrap(),
            _ => panic!("unexpected operation in polynomial quotient oracle: {node:?}"),
        };
        let (a, b, d) = value
            .exact_quadratic_normal_form()
            .expect("quadratic fixture node");
        nodes.push(Real::new(a) + Real::new(b) * Real::new(d).sqrt().unwrap());
    }
    let values: Vec<_> = definition
        .outputs
        .iter()
        .map(|&id| nodes[id].clone())
        .collect();
    for (coordinate, expected) in coordinates.iter().enumerate() {
        assert_eq!(
            hyperlimit::compare_reals(&values[coordinate], expected, PredicatePolicy::STRICT)
                .value(),
            Some(std::cmp::Ordering::Equal)
        );
        for (axis, partial) in partials.iter().enumerate() {
            let actual = &values[prepared.program().partial_output(coordinate, axis).unwrap()];
            assert_eq!(
                hyperlimit::compare_reals(actual, &partial[coordinate], PredicatePolicy::STRICT)
                    .value(),
                Some(std::cmp::Ordering::Equal),
                "coordinate {coordinate} axis {axis}"
            );
        }
    }
}

fn d(lower: i64, upper: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lower.into(), upper.into(), exponent).unwrap()
}

#[test]
fn nonuniform_weights_keep_independent_rational_values_and_partials() {
    // S=(u/(1+u),v,0), with weights [1,2] in each row.
    let source = Surface::rational_bezier(
        vec![
            vec![
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(q(1, 2), 0.into(), 0.into()),
            ],
            vec![
                Point3::new(0.into(), 1.into(), 0.into()),
                Point3::new(q(1, 2), 1.into(), 0.into()),
            ],
        ],
        vec![vec![1.into(), 2.into()]; 2],
    )
    .unwrap();
    let full = enclose(&source, &BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap());
    contains(
        full.coordinates()[0],
        0.into(),
        Rational::from(1) / Rational::from(2),
    );
    contains(full.coordinates()[1], 0.into(), 1.into());
    contains(
        full.partials()[0][0],
        Rational::from(1) / Rational::from(4),
        1.into(),
    );
    let point = enclose(&source, &BoxN::new([d(1, 1, -1), d(1, 1, -2)]).unwrap());
    contains(
        point.coordinates()[0],
        Rational::from(1) / Rational::from(3),
        Rational::from(1) / Rational::from(3),
    );
    assert_eq!(point.coordinates()[1], &d(1, 1, -2));
    contains(
        point.partials()[0][0],
        Rational::from(4) / Rational::from(9),
        Rational::from(4) / Rational::from(9),
    );
    for (range, expected) in point.partials()[1].into_iter().zip([0, 1, 0]) {
        encloses_rational_point(range, expected.into());
    }
    exact_values(
        &prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap(),
        [q(1, 2), q(1, 4)],
        [q(1, 3), q(1, 4), 0.into()],
        [
            [q(4, 9), 0.into(), 0.into()],
            [0.into(), 1.into(), 0.into()],
        ],
    );
}

#[test]
fn positive_native_weights_do_not_turn_denominator_overestimation_into_a_pole() {
    // W=1-3u/2+3u²/2 >=5/8. Its whole Horner enclosure crosses zero.
    let source = Surface::rational_bezier(
        (0..2)
            .map(|v| {
                (0..3)
                    .map(|u| Point3::new(q(u, 2), v.into(), 0.into()))
                    .collect()
            })
            .collect(),
        vec![vec![1.into(), q(1, 4), 1.into()]; 2],
    )
    .unwrap();
    let prepared = prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap();
    let error = prepared
        .enclose(
            &BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap(),
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap_err();
    assert!(matches!(
        error,
        SurfaceRangeError::Expression(ExpressionError::Enclosure {
            source: IntervalError::ContainsZero,
            ..
        })
    ));
    assert!(std::error::Error::source(&error).is_some());
    let narrow = prepared
        .enclose(
            &BoxN::new([d(0, 1, -4), d(0, 1, 0)]).unwrap(),
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    // X(1/16)=(u/4+3u²/4)/W=19/934; X is increasing here.
    contains(
        narrow.coordinates()[0],
        0.into(),
        Rational::from(19) / Rational::from(934),
    );
    let retry = prepared
        .enclose(
            &BoxN::new([d(0, 1, -4), d(0, 1, 0)]).unwrap(),
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert_eq!(narrow.evidence().values(), retry.evidence().values());
}

#[test]
fn asymmetric_arbitrary_degree_tensor_has_exact_independent_midpoint_derivatives() {
    for (n, m) in [(2, 3), (7, 2), (33, 2)] {
        // Bernstein controls of (u,v,u^n v^m); no degree reduction is assumed.
        let controls = (0..=m)
            .map(|j| {
                (0..=n)
                    .map(|i| Point3::new(q(i, n), q(j, m), i32::from(i == n && j == m).into()))
                    .collect()
            })
            .collect();
        let source = Surface::rational_bezier(
            controls,
            vec![vec![1.into(); n as usize + 1]; m as usize + 1],
        )
        .unwrap();
        let domain = BoxN::new([d(1, 1, -1), d(1, 1, -1)]).unwrap();
        let prepared = prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap();
        let report = prepared
            .enclose(&domain, -128, &mut graph(), &mut arithmetic())
            .unwrap();
        assert_eq!(
            report.coordinates(),
            [&d(1, 1, -1), &d(1, 1, -1), &d(1, 1, -n - m)]
        );
        assert_eq!(
            report.partials()[0],
            [&d(1, 1, 0), &d(0, 0, 0), &d(n.into(), n.into(), 1 - n - m)]
        );
        assert_eq!(
            report.partials()[1],
            [&d(0, 0, 0), &d(1, 1, 0), &d(m.into(), m.into(), 1 - n - m)]
        );
        let coarse = prepared
            .enclose(&domain, -16, &mut graph(), &mut arithmetic())
            .unwrap();
        for (fine, coarse) in report
            .evidence()
            .values()
            .iter()
            .zip(coarse.evidence().values())
        {
            assert!(fine.subset_of(coarse, &mut arithmetic()).unwrap());
        }
        assert_eq!(
            report.evidence().values(),
            prepared
                .enclose(&domain, -128, &mut graph(), &mut arithmetic())
                .unwrap()
                .evidence()
                .values()
        );
    }
}

#[test]
fn algebraic_subdivisions_feed_ranges_with_both_chain_rule_factors() {
    let alpha = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    let (u_left, _) = patch().split_u_at(&alpha).unwrap();
    let (both_left, _) = u_left.split_v_at(&alpha).unwrap();
    // S(alpha*u,alpha*v)=(alpha*u,2alpha*v,uv/2).
    let report = enclose(&both_left, &BoxN::new([d(1, 1, 0), d(1, 1, 0)]).unwrap());
    for (range, expected) in [
        (
            report.coordinates()[0],
            Rational::from(1) / Rational::from(2),
        ),
        (report.coordinates()[1], Rational::from(2)),
        (
            report.partials()[0][0],
            Rational::from(1) / Rational::from(2),
        ),
        (report.partials()[1][1], Rational::from(2)),
    ] {
        let bounds = range.to_rational(&mut arithmetic()).unwrap();
        let (lo, hi) = bounds.bounds().unwrap();
        assert!(!lo.is_negative());
        assert!(lo * lo <= expected && hi * hi >= expected);
    }
    for range in [
        report.coordinates()[2],
        report.partials()[0][2],
        report.partials()[1][2],
    ] {
        contains(
            range,
            Rational::from(1) / Rational::from(2),
            Rational::from(1) / Rational::from(2),
        );
    }
    encloses_rational_point(report.partials()[0][1], 0.into());
    encloses_rational_point(report.partials()[1][0], 0.into());
    exact_values(
        &prepare_rational_bezier_surface_ranges(&both_left, &mut graph()).unwrap(),
        [1.into(), 1.into()],
        [alpha.clone(), Real::from(2) * &alpha, q(1, 2)],
        [
            [alpha.clone(), 0.into(), q(1, 2)],
            [0.into(), Real::from(2) * &alpha, q(1, 2)],
        ],
    );
}

#[test]
fn constant_and_rank_one_patches_have_c1_ranges_without_an_immersion_claim() {
    for rank_one in [false, true] {
        let source = Surface::rational_bezier(
            vec![
                vec![
                    Point3::new(0.into(), 2.into(), 3.into()),
                    Point3::new(i32::from(rank_one).into(), 2.into(), 3.into()),
                ];
                2
            ],
            vec![vec![1.into(); 2]; 2],
        )
        .unwrap();
        let report = enclose(&source, &BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap());
        assert_eq!(
            report.coordinates(),
            [&d(0, i64::from(rank_one), 0), &d(2, 2, 0), &d(3, 3, 0)]
        );
        assert_eq!(
            report.partials()[0],
            [
                &d(i64::from(rank_one), i64::from(rank_one), 0),
                &d(0, 0, 0),
                &d(0, 0, 0)
            ]
        );
        assert_eq!(report.partials()[1], [&d(0, 0, 0); 3]);
        assert!(matches!(
            source.normal_at(&hypercurve::Point2::new(q(1, 2), q(1, 2)).into()),
            Err(GeometryError::SingularSurfaceParameter)
        ));
    }
}

#[test]
fn empty_outside_and_unsupported_inputs_remain_distinct() {
    let source = patch();
    let prepared = prepare_rational_bezier_surface_ranges(&source, &mut graph()).unwrap();
    // Cartesian emptiness is canonical even when the other lane is outside UV.
    let empty = BoxN::new([DyadicInterval::empty(), d(9, 10, 0)]).unwrap();
    let report = prepared
        .enclose(&empty, -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert!(report.parameters().is_empty());
    assert!(
        report
            .evidence()
            .values()
            .iter()
            .all(DyadicInterval::is_empty)
    );
    assert!(
        report
            .native_bounds(&mut graph(), &mut arithmetic())
            .unwrap()
            .bounds()
            .is_none()
    );
    for domain in [
        BoxN::new([d(-1, 0, 0), d(0, 1, 0)]).unwrap(),
        BoxN::new([d(0, 1, 0), d(1, 2, 0)]).unwrap(),
    ] {
        assert!(matches!(
            prepared.enclose(&domain, -64, &mut graph(), &mut arithmetic()),
            Err(SurfaceRangeError::Geometry(
                GeometryError::SurfaceParameterOutsideDomain
            ))
        ));
    }
    let plane = Surface::plane(
        Point3::new(0.into(), 0.into(), 0.into()),
        Vector3::new([1.into(), 0.into(), 0.into()]),
        Vector3::new([0.into(), 1.into(), 0.into()]),
    )
    .unwrap();
    assert!(matches!(
        prepare_rational_bezier_surface_ranges(&plane, &mut graph()),
        Err(SurfaceRangeError::UnsupportedSurface(_))
    ));
}

#[test]
fn resource_failures_and_cancellation_are_typed_and_retryable() {
    let source = patch();
    let mut g = graph();
    let prepared = prepare_rational_bezier_surface_ranges(&source, &mut g).unwrap();
    let work = 100_000_000 - g.remaining_work();
    for limit in [0, 1, work / 2, work - 1] {
        assert!(matches!(
            prepare_rational_bezier_surface_ranges(
                &source,
                &mut ExpressionBudget::new(1_000_000, limit)
            ),
            Err(SurfaceRangeError::Expression(
                ExpressionError::WorkExhausted
            ))
        ));
    }
    assert!(
        prepare_rational_bezier_surface_ranges(
            &source,
            &mut ExpressionBudget::new(1_000_000, work)
        )
        .is_ok()
    );
    assert!(matches!(
        prepare_rational_bezier_surface_ranges(&source, &mut ExpressionBudget::new(1, 100_000_000)),
        Err(SurfaceRangeError::Expression(
            ExpressionError::SizeExhausted
        ))
    ));
    let domain = BoxN::new([d(1, 2, -2), d(2, 3, -2)]).unwrap();
    let mut g = graph();
    let mut a = arithmetic();
    let reference = prepared.enclose(&domain, -64, &mut g, &mut a).unwrap();
    let work = 100_000_000 - g.remaining_work();
    let arithmetic_work = 1_000_000_000_000 - a.remaining_work();
    for limit in 0..work {
        assert!(
            matches!(
                prepared.enclose(
                    &domain,
                    -64,
                    &mut ExpressionBudget::new(1_000_000, limit),
                    &mut arithmetic()
                ),
                Err(SurfaceRangeError::Expression(
                    ExpressionError::WorkExhausted
                ))
            ),
            "work cutoff {limit}"
        );
    }
    let exact = prepared
        .enclose(
            &domain,
            -64,
            &mut ExpressionBudget::new(1_000_000, work),
            &mut DyadicBudget::new(16384, arithmetic_work),
        )
        .unwrap();
    assert_eq!(reference.evidence().values(), exact.evidence().values());
    let failure = prepared
        .enclose(
            &domain,
            -64,
            &mut graph(),
            &mut DyadicBudget::new(16384, arithmetic_work - 1),
        )
        .unwrap_err();
    assert!(matches!(
        failure,
        SurfaceRangeError::Interval(IntervalError::WorkExhausted)
            | SurfaceRangeError::Expression(ExpressionError::Enclosure {
                source: IntervalError::WorkExhausted,
                ..
            })
            | SurfaceRangeError::Expression(ExpressionError::ArgumentEnclosure {
                source: IntervalError::WorkExhausted,
                ..
            })
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        prepare_rational_bezier_surface_ranges(&source, &mut cancelled),
        Err(SurfaceRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert!(matches!(
        prepared.enclose(&domain, -64, &mut cancelled, &mut arithmetic()),
        Err(SurfaceRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert!(matches!(
        reference.native_bounds(&mut cancelled, &mut arithmetic()),
        Err(SurfaceRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert_eq!(
        reference.evidence().values(),
        enclose(&source, &domain).evidence().values()
    );
}

#[test]
fn spatial_exclusion_preserves_touching_candidates_and_tiny_distinct_gaps() {
    let report = enclose(&patch(), &BoxN::new([d(0, 1, 0), d(0, 1, 0)]).unwrap());
    let projected = report
        .native_bounds(&mut graph(), &mut arithmetic())
        .unwrap();
    let bounds = projected.bounds().unwrap();
    let tiny = Real::from(2).powi((-80).into()).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (minimum_x, expected) in [(Real::one(), true), (Real::one() + &tiny, false)] {
            let outcome = ordered_aabb3s_intersect(
                &bounds.mins,
                &bounds.maxs,
                &Point3::new(minimum_x, 0.into(), 0.into()),
                &Point3::new(2.into(), 2.into(), 1.into()),
                policy,
            );
            assert!(
                matches!(outcome,PredicateOutcome::Decided{value,certainty:Certainty::Exact,..} if value==expected)
            );
        }
    }
}

#[test]
fn immutable_program_sharing_keeps_worker_budgets_and_domain_evidence_separate() {
    let prepared = prepare_rational_bezier_surface_ranges(&patch(), &mut graph()).unwrap();
    let domains = [
        BoxN::new([d(0, 1, -1), d(1, 2, -1)]).unwrap(),
        BoxN::new([d(1, 2, -1), d(0, 1, -1)]).unwrap(),
    ];
    let serial = domains
        .iter()
        .map(|domain| {
            prepared
                .enclose(domain, -64, &mut graph(), &mut arithmetic())
                .unwrap()
        })
        .collect::<Vec<_>>();
    std::thread::scope(|scope| {
        let workers = domains
            .iter()
            .map(|domain| {
                let prepared = &prepared;
                scope.spawn(move || {
                    // Each worker owns an explicit budget; no implicit shared allowance.
                    let mut exhausted = ExpressionBudget::new(1_000_000, 0);
                    assert!(matches!(
                        prepared.enclose(domain, -64, &mut exhausted, &mut arithmetic()),
                        Err(SurfaceRangeError::Expression(
                            ExpressionError::WorkExhausted
                        ))
                    ));
                    prepared
                        .enclose(domain, -64, &mut graph(), &mut arithmetic())
                        .unwrap()
                })
            })
            .collect::<Vec<_>>();
        for ((worker, expected), domain) in workers.into_iter().zip(&serial).zip(&domains) {
            let actual = worker.join().unwrap();
            assert_eq!(actual.parameters(), domain);
            assert_eq!(actual.evidence().values(), expected.evidence().values());
        }
    });
    let report = prepared
        .enclose(&domains[0], -64, &mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(report.evidence().values(), serial[0].evidence().values());
}

#[test]
fn native_asymmetric_patch_ranges_feed_certified_spatial_exclusion() {
    // Rows increase v, columns increase u: S(u,v) = (u, 2v, uv).
    let surface = Surface::rational_bezier(
        vec![
            vec![
                Point3::new(0.into(), 0.into(), 0.into()),
                Point3::new(1.into(), 0.into(), 0.into()),
            ],
            vec![
                Point3::new(0.into(), 2.into(), 0.into()),
                Point3::new(1.into(), 2.into(), 1.into()),
            ],
        ],
        vec![vec![1.into(), 1.into()]; 2],
    )
    .unwrap();
    let mut graph = ExpressionBudget::new(1_000_000, 100_000_000);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000_000);
    let prepared = prepare_rational_bezier_surface_ranges(&surface, &mut graph).unwrap();
    let domain = BoxN::new([d(1, 2, -2), d(2, 3, -2)]).unwrap();
    let report = prepared
        .enclose(&domain, -64, &mut graph, &mut arithmetic)
        .unwrap();
    assert_eq!(report.parameters(), &domain);
    assert_eq!(
        report.coordinates(),
        [&d(1, 2, -2), &d(2, 3, -1), &d(1, 3, -3)]
    );
    assert_eq!(
        report.partials()[0],
        [&d(1, 1, 0), &d(0, 0, 0), &d(2, 3, -2)]
    );
    assert_eq!(
        report.partials()[1],
        [&d(0, 0, 0), &d(2, 2, 0), &d(1, 2, -2)]
    );
    let projected = report.native_bounds(&mut graph, &mut arithmetic).unwrap();
    assert!(std::ptr::eq(projected.evidence(), &report));
    let bounds = projected.bounds().unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert!(matches!(
            ordered_aabb3s_intersect(
                &bounds.mins,
                &bounds.maxs,
                &Point3::new(2.into(), 2.into(), 2.into()),
                &Point3::new(3.into(), 3.into(), 3.into()),
                policy,
            ),
            PredicateOutcome::Decided {
                value: false,
                certainty: Certainty::Exact,
                ..
            }
        ));
    }
}
