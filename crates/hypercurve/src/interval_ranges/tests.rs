use super::*;
use crate::{HomogeneousControl2, Point2};
use hyperreal::Rational;

fn graph() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn arithmetic() -> DyadicBudget {
    DyadicBudget::new(16384, 1_000_000_000_000)
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn parabola() -> RationalBezier2 {
    RationalBezier2::try_new(vec![pt!(0, 0), pt!(1, 0), pt!(2, 1)], vec![r!(1); 3]).unwrap()
}
fn enclose(source: &RationalBezier2, domain: &DyadicInterval) -> RationalBezierRangeEnclosure {
    prepare_rational_bezier_ranges(source, &mut graph())
        .unwrap()
        .enclose(domain, -64, &mut graph(), &mut arithmetic())
        .unwrap()
}
fn rational_contains(interval: &DyadicInterval, lower: Rational, upper: Rational) {
    let q = interval.to_rational(&mut arithmetic()).unwrap();
    assert!(q.contains_point(&lower), "{q:?} lacks {lower}");
    assert!(q.contains_point(&upper), "{q:?} lacks {upper}");
}
#[test]
fn polynomial_coordinate_and_derivative_ranges_are_exact_and_reusable() {
    let source = parabola();
    let prepared = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
    for domain in [d(1, 2, -2), d(2, 3, -2)] {
        let report = prepared
            .enclose(&domain, -32, &mut graph(), &mut arithmetic())
            .unwrap();
        let expected_x = domain.mul(&d(2, 2, 0), &mut arithmetic()).unwrap();
        let expected_y = domain.square(&mut arithmetic()).unwrap();
        assert_eq!(report.coordinates(), [&expected_x, &expected_y]);
        assert_eq!(report.tangents(), [&d(2, 2, 0), &expected_x]);
        assert_eq!(report.parameters(), &domain);
        assert!(report.source().shares_retained_data(&source));
        assert!(std::ptr::eq(
            report.evidence().program().source().definition(),
            prepared.program().source().definition()
        ));
        let rebuilt = enclose(&source, &domain);
        assert_eq!(report.evidence().values(), rebuilt.evidence().values());
        assert_eq!(
            report.evidence().program().program().definition(),
            rebuilt.evidence().program().program().definition()
        );
    }
    drop(source);
    assert!(
        prepared
            .enclose(&d(0, 1, 0), -32, &mut graph(), &mut arithmetic())
            .is_ok()
    );
}
#[test]
fn rational_coordinates_and_tangents_contain_independent_analytic_ranges() {
    // C=(t/(1+t),1/(1+t)), C'=(1,-1)/(1+t)^2 on [0,1].
    let source = RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(r!(0), r!(1), r!(1)),
        HomogeneousControl2::new(r!(1), r!(1), r!(2)),
    ])
    .unwrap();
    let report = enclose(&source, &d(0, 1, 0));
    let half = Rational::from(1) / Rational::from(2);
    let quarter = Rational::from(1) / Rational::from(4);
    rational_contains(report.coordinates()[0], Rational::from(0), half.clone());
    rational_contains(report.coordinates()[1], half, Rational::from(1));
    rational_contains(report.tangents()[0], quarter.clone(), Rational::from(1));
    rational_contains(report.tangents()[1], Rational::from(-1), -quarter);
    let singleton = enclose(&source, &d(1, 1, -1));
    for (range, expected) in singleton.coordinates().into_iter().zip([
        Rational::from(1) / Rational::from(3),
        Rational::from(2) / Rational::from(3),
    ]) {
        rational_contains(range, expected.clone(), expected);
    }
}
#[test]
fn arbitrary_degree_power_family_has_independent_exact_midpoint_expectations() {
    for n in [1, 2, 5, 12, 65] {
        // Bernstein controls of (t,t^n): x_i=i/n and y_i=0 except y_n=1.
        let controls = (0..=n)
            .map(|i| {
                Point2::new(
                    (Real::from(i) / Real::from(n)).unwrap(),
                    Real::from(i32::from(i == n)),
                )
            })
            .collect();
        let source = RationalBezier2::try_new(controls, vec![r!(1); n as usize + 1]).unwrap();
        let prepared = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
        let report = prepared
            .enclose(&d(1, 1, -1), -128, &mut graph(), &mut arithmetic())
            .unwrap();
        assert_eq!(report.coordinates(), [&d(1, 1, -1), &d(1, 1, -n)]);
        assert_eq!(
            report.tangents(),
            [&d(1, 1, 0), &d(n.into(), n.into(), 1 - n)]
        );
        if n == 65 {
            let coarse = prepared
                .enclose(&d(1, 1, -1), -64, &mut graph(), &mut arithmetic())
                .unwrap();
            assert_eq!(coarse.coordinates()[1], &d(0, 1, -64));
            assert!(
                d(1, 1, -65)
                    .subset_of(coarse.coordinates()[1], &mut arithmetic())
                    .unwrap()
            );
            let fine_again = prepared
                .enclose(&d(1, 1, -1), -128, &mut graph(), &mut arithmetic())
                .unwrap();
            assert_eq!(fine_again.evidence().values(), report.evidence().values());
        }
    }
}
#[test]
fn algebraic_subdivision_reversal_and_reconstruction_preserve_source_and_chain_rule() {
    let original = parabola();
    let alpha = (r!(2).sqrt().unwrap() / r!(2)).unwrap();
    let (left, _) = original.split_at_exact(&alpha).unwrap();
    let reversed = left.reversed();
    let rebuilt =
        RationalBezier2::from_homogeneous_controls(reversed.homogeneous_controls().to_vec())
            .unwrap();
    for source in [&reversed, &rebuilt] {
        let report = enclose(source, &d(0, 0, 0));
        // Reversed left piece: (sqrt(2)*(1-t), (1-t)^2/2).
        let x = report.coordinates()[0]
            .to_rational(&mut arithmetic())
            .unwrap();
        let (lower, upper) = x.bounds().unwrap();
        assert!(!lower.is_negative());
        assert!(lower * lower <= Rational::from(2));
        assert!(upper * upper >= Rational::from(2));
        rational_contains(
            report.coordinates()[1],
            Rational::from(1) / Rational::from(2),
            Rational::from(1) / Rational::from(2),
        );
        rational_contains(report.tangents()[1], Rational::from(-1), Rational::from(-1));
        let dx = report.tangents()[0].to_rational(&mut arithmetic()).unwrap();
        let (lo, hi) = dx.bounds().unwrap();
        assert!(hi.is_negative());
        assert!(hi * hi <= Rational::from(2) && lo * lo >= Rational::from(2));
        assert!(report.source().shares_retained_data(source));
    }
}
#[test]
fn denominator_overestimation_is_a_typed_noncertificate_and_subdivision_retries() {
    // W=(t-1/2)^2+1/16 > 0. The whole Horner interval still crosses zero.
    let source = RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(r!(0), r!(1), r!(5 / 16)),
        HomogeneousControl2::new(r!(1 / 2), r!(1), r!(-3 / 16)),
        HomogeneousControl2::new(r!(1), r!(1), r!(5 / 16)),
    ])
    .unwrap();
    let prepared = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
    assert!(matches!(
        prepared.enclose(&d(0, 1, 0), -64, &mut graph(), &mut arithmetic()),
        Err(CurveRangeError::Expression(ExpressionError::Enclosure {
            source: IntervalError::ContainsZero,
            ..
        }))
    ));
    let report = prepared
        .enclose(&d(0, 1, -4), -64, &mut graph(), &mut arithmetic())
        .unwrap();
    rational_contains(
        report.coordinates()[0],
        Rational::from(0),
        Rational::from(16) / Rational::from(65),
    );
    // t=1/16 has W=65/256, hence x=16/65; native source was not changed.
    assert!(report.source().shares_retained_data(&source));
}
#[test]
fn empty_invalid_domains_and_resource_failures_stay_distinct_and_retryable() {
    let source = parabola();
    let mut g = graph();
    let prepared = prepare_rational_bezier_ranges(&source, &mut g).unwrap();
    let prepare_work = 100_000_000 - g.remaining_work();
    for limit in 0..prepare_work {
        assert!(
            prepare_rational_bezier_ranges(&source, &mut ExpressionBudget::new(1_000_000, limit))
                .is_err(),
            "preparation cutoff {limit}"
        );
    }
    assert!(
        prepare_rational_bezier_ranges(
            &source,
            &mut ExpressionBudget::new(1_000_000, prepare_work)
        )
        .is_ok()
    );
    let empty = prepared
        .enclose(
            &DyadicInterval::empty(),
            -64,
            &mut graph(),
            &mut arithmetic(),
        )
        .unwrap();
    assert!(
        empty
            .coordinates()
            .into_iter()
            .chain(empty.tangents())
            .all(DyadicInterval::is_empty)
    );
    for domain in [d(-1, 0, 0), d(1, 2, 0)] {
        assert!(matches!(
            prepared.enclose(&domain, -64, &mut graph(), &mut arithmetic()),
            Err(CurveRangeError::OutsideUnitDomain)
        ));
    }
    let mut g = graph();
    let mut a = arithmetic();
    let full = prepared.enclose(&d(0, 1, -1), -32, &mut g, &mut a).unwrap();
    let work = 100_000_000 - g.remaining_work();
    let scalar = 1_000_000_000_000 - a.remaining_work();
    for limit in 0..work {
        assert!(
            prepared
                .enclose(
                    &d(0, 1, -1),
                    -32,
                    &mut ExpressionBudget::new(1_000_000, limit),
                    &mut arithmetic()
                )
                .is_err(),
            "query cutoff {limit}"
        );
    }
    let exact = prepared
        .enclose(
            &d(0, 1, -1),
            -32,
            &mut ExpressionBudget::new(1_000_000, work),
            &mut DyadicBudget::new(16384, scalar),
        )
        .unwrap();
    assert_eq!(exact.evidence().values(), full.evidence().values());
    assert!(
        prepared
            .enclose(
                &d(0, 1, -1),
                -32,
                &mut graph(),
                &mut DyadicBudget::new(16384, scalar - 1)
            )
            .is_err()
    );
    assert!(
        prepared
            .enclose(
                &d(0, 1, -1),
                -32,
                &mut graph(),
                &mut DyadicBudget::new(1, u64::MAX)
            )
            .is_err()
    );
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut cancelled = graph();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        prepare_rational_bezier_ranges(&source, &mut cancelled),
        Err(CurveRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert!(matches!(
        prepared.enclose(&d(0, 1, -1), -32, &mut cancelled, &mut arithmetic()),
        Err(CurveRangeError::Expression(ExpressionError::Cancelled))
    ));
    let retry = enclose(&source, &d(0, 1, -1));
    assert_eq!(retry.evidence().values(), full.evidence().values());
}
#[test]
fn fresh_process_reconstructs_native_controls_and_replays_range_proof() {
    const KEY: &str = "HYPERCURVE_RANGE_REPLAY_CONTROLS";
    if let Ok(encoded) = std::env::var(KEY) {
        let data: Vec<i32> = encoded.split(',').map(|x| x.parse().unwrap()).collect();
        assert_eq!(data.len() % 3, 0);
        let source = RationalBezier2::from_homogeneous_controls(
            data.as_chunks::<3>()
                .0
                .iter()
                .map(|c| {
                    HomogeneousControl2::new(Real::from(c[0]), Real::from(c[1]), Real::from(c[2]))
                })
                .collect(),
        )
        .unwrap();
        let report = enclose(&source, &d(1, 3, -2));
        println!("PROOF={:?}", report.evidence());
        return;
    }
    let mut previous = None;
    for _ in 0..3 {
        let output=std::process::Command::new(std::env::current_exe().unwrap()).args(["--exact","interval_ranges::tests::fresh_process_reconstructs_native_controls_and_replays_range_proof","--nocapture"]).env(KEY,"0,0,1,1,0,1,2,1,1").output().unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let text = String::from_utf8(output.stdout).unwrap();
        let proof = text
            .lines()
            .find(|l| l.starts_with("PROOF="))
            .unwrap()
            .to_owned();
        if let Some(old) = previous.as_ref() {
            assert_eq!(&proof, old);
        }
        previous = Some(proof);
    }
}

#[test]
fn native_box_projection_keeps_evidence_empty_semantics_and_touching_candidates() {
    let source = parabola();
    let report = enclose(&source, &d(1, 2, -2));
    let mut g = graph();
    let mut a = arithmetic();
    let projected = report.native_bounds(&mut g, &mut a).unwrap();
    assert!(std::ptr::eq(projected.evidence(), &report));
    let bounds = projected.bounds().unwrap();
    assert_eq!(bounds.min_x(), &r!(1 / 2));
    assert_eq!(bounds.max_x(), &r!(1));
    assert_eq!(bounds.min_y(), &r!(1 / 16));
    assert_eq!(bounds.max_y(), &r!(1 / 4));
    assert!(
        bounds
            .overlaps(&Aabb2::from_point(Point2::new(r!(1), r!(1 / 4))))
            .unwrap()
    );
    assert!(!bounds.overlaps(&Aabb2::from_point(pt!(3, 3))).unwrap());
    let work = 100_000_000 - g.remaining_work();
    let native = 1_000_000_000_000 - a.remaining_work();
    for limit in 0..work {
        assert!(matches!(
            report.native_bounds(
                &mut ExpressionBudget::new(1_000_000, limit),
                &mut arithmetic()
            ),
            Err(CurveRangeError::Expression(ExpressionError::WorkExhausted))
        ));
    }
    assert_eq!(
        report
            .native_bounds(
                &mut ExpressionBudget::new(1_000_000, work),
                &mut DyadicBudget::new(16384, native)
            )
            .unwrap()
            .bounds(),
        Some(bounds)
    );
    assert!(matches!(
        report.native_bounds(&mut graph(), &mut DyadicBudget::new(16384, native - 1)),
        Err(CurveRangeError::Interval(IntervalError::WorkExhausted))
    ));
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut g = graph();
    g.cancellation = Some(token);
    assert!(matches!(
        report.native_bounds(&mut g, &mut arithmetic()),
        Err(CurveRangeError::Expression(ExpressionError::Cancelled))
    ));
    assert_eq!(
        report
            .native_bounds(&mut graph(), &mut arithmetic())
            .unwrap()
            .bounds(),
        Some(bounds)
    );
    let empty = enclose(&source, &DyadicInterval::empty());
    assert!(
        empty
            .native_bounds(&mut graph(), &mut arithmetic())
            .unwrap()
            .bounds()
            .is_none()
    );
}

#[test]
fn zero_weight_control_remains_homogeneous_and_has_exact_midpoint_tangent() {
    let source = RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(r!(0), r!(0), r!(1)),
        HomogeneousControl2::new(r!(1), r!(1), r!(0)),
        HomogeneousControl2::new(r!(2), r!(0), r!(1)),
    ])
    .unwrap();
    // X=2t, Y=2t(1-t), W=(1-t)^2+t^2. No affine middle control exists.
    let report = enclose(&source, &d(1, 1, -1));
    assert_eq!(report.coordinates(), [&d(2, 2, 0), &d(1, 1, 0)]);
    assert_eq!(report.tangents(), [&d(4, 4, 0), &d(0, 0, 0)]);
    assert_eq!(report.source().homogeneous_controls()[1].weight(), &r!(0));
    assert!(source.cached_homogeneous_power_basis().is_none());
}

#[test]
fn constant_curve_is_c1_without_claiming_nonzero_tangents_or_injectivity() {
    let source = RationalBezier2::try_new(vec![pt!(2, 3); 3], vec![r!(1); 3]).unwrap();
    let report = enclose(&source, &d(0, 1, 0));
    assert_eq!(report.coordinates(), [&d(2, 2, 0), &d(3, 3, 0)]);
    assert_eq!(report.tangents(), [&d(0, 0, 0), &d(0, 0, 0)]);
    let projected = report
        .native_bounds(&mut graph(), &mut arithmetic())
        .unwrap();
    assert_eq!(projected.bounds(), Some(&Aabb2::from_point(pt!(2, 3))));
}

#[test]
fn preparation_and_failed_attempts_leave_native_basis_caches_unchanged() {
    let source = parabola();
    assert!(source.cached_homogeneous_power_basis().is_none());
    // The conversion reservation is 6*n*n+6*n=72. Fail after conversion.
    assert!(matches!(
        prepare_rational_bezier_ranges(&source, &mut ExpressionBudget::new(1_000_000, 73)),
        Err(CurveRangeError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(source.cached_homogeneous_power_basis().is_none());
    let independent = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
    assert!(source.cached_homogeneous_power_basis().is_none());
    assert_eq!(
        source.point_at(&r!(1 / 2)).unwrap(),
        Point2::new(r!(1), r!(1 / 4))
    );
    let native = source.cached_homogeneous_power_basis().unwrap() as *const _;
    let reused = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
    assert_eq!(
        source.cached_homogeneous_power_basis().unwrap() as *const _,
        native
    );
    assert_eq!(
        independent.program().program().definition(),
        reused.program().program().definition()
    );
    assert_eq!(
        independent
            .enclose(&d(0, 1, 0), -64, &mut graph(), &mut arithmetic())
            .unwrap()
            .evidence()
            .values(),
        reused
            .enclose(&d(0, 1, 0), -64, &mut graph(), &mut arithmetic())
            .unwrap()
            .evidence()
            .values()
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn high_degree_native_point_dispatch_is_unchanged_by_range_preparation() {
    use hyperreal::dispatch_trace;
    // Fixed dispatch counts require a fresh scalar-cache lifecycle. A reused
    // libtest worker can retain scalar facts from unrelated earlier fixtures.
    const CHILD: &str = "HYPERCURVE_INTERVAL_RANGE_DISPATCH_CHILD";
    if std::env::var_os(CHILD).is_none() {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "interval_ranges::tests::high_degree_native_point_dispatch_is_unchanged_by_range_preparation",
                "--nocapture",
            ])
            .env(CHILD, "1")
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        return;
    }
    for policy in [
        crate::PredicatePolicy::STRICT,
        crate::PredicatePolicy::APPROXIMATE_512,
    ] {
        let n = 257;
        let source = RationalBezier2::try_new(
            (0..=n)
                .map(|i| {
                    Point2::new(
                        (Real::from(i) / Real::from(n)).unwrap(),
                        Real::from(i32::from(i == n)),
                    )
                })
                .collect(),
            vec![r!(1); n as usize + 1],
        )
        .unwrap();
        // Match the external dispatch probe: construct the query before recording.
        let parameter = r!(1 / 2);
        let evaluate = || {
            let outcome = crate::evaluate_under(policy, || source.point_at(&parameter));
            assert_eq!(outcome.certainty(), crate::CurveCertainty::Certified);
            outcome.certified().unwrap().unwrap()
        };
        let warm = evaluate();
        dispatch_trace::reset();
        let expected = dispatch_trace::with_recording(evaluate);
        let baseline = dispatch_trace::take_trace();
        assert_eq!(expected, warm);
        assert_eq!(baseline.rational.temporary_rationals, 2567);
        assert_eq!(baseline.rational.gcds, 751);
        let _prepared = prepare_rational_bezier_ranges(&source, &mut graph()).unwrap();
        assert!(source.cached_homogeneous_power_basis().is_none());
        dispatch_trace::reset();
        let actual = dispatch_trace::with_recording(evaluate);
        assert_eq!(actual, expected);
        assert_eq!(dispatch_trace::take_trace(), baseline);
    }
}
