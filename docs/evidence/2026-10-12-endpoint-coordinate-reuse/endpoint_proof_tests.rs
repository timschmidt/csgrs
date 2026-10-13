use super::{CircularArc2, CurveContext, Point2, points_equal};
use crate::{CurveCertainty, CurvePreviewOptions, WorkBudget, provisional};
use hyperreal::{Real, ZeroKnowledge};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

// Preserve the old sequence as an independently invoked differential control.
fn original(arc: &CircularArc2, point: &Point2, policy: &CurveContext) -> Option<bool> {
    if point == arc.start() || point == arc.end() {
        return Some(true);
    }
    let at_start = points_equal(point, arc.start(), policy);
    if at_start == Some(true) {
        return Some(true);
    }
    match points_equal(point, arc.end(), policy) {
        Some(true) => Some(true),
        Some(false) if at_start == Some(false) => Some(false),
        Some(false) | None => None,
    }
}

fn in_context<T>(context: usize, evaluate: impl FnOnce(&CurveContext) -> T) -> T {
    match context {
        0 => evaluate(&CurveContext::STRICT),
        1 => CurveContext::STRICT.bounded_exact_predicate_pass(|| evaluate(&CurveContext::STRICT)),
        2 => evaluate(&CurveContext::APPROXIMATE_512),
        3 => CurveContext::APPROXIMATE_512.bounded_exact_predicate_pass(|| evaluate(&CurveContext::APPROXIMATE_512)),
        4 => CurvePreviewOptions::try_strict(1e-8, 1e-8).unwrap().evaluate(|| evaluate(&crate::policy::principal_context())),
        5 => CurvePreviewOptions::try_approximate_512(1e-8, 1e-8).unwrap().evaluate(|| evaluate(&crate::policy::principal_context())),
        6 => CurveContext::APPROXIMATE_512.strict_predicate_pass(|| evaluate(&CurveContext::APPROXIMATE_512)),
        _ => unreachable!(),
    }
}

fn fixture(representation: usize, query: usize) -> (CircularArc2, Point2) {
    let (center, scale) = match representation {
        0 => (Point2::from_values(0, 0), Real::one()),
        1 => (Point2::new(Real::pi(), Real::e()), Real::pi()),
        2 => (Point2::new(Real::e().sin(), Real::e().cos()), Real::from(-3)),
        3 => (Point2::from_values(0, 0), "1e-400".parse::<Real>().unwrap()),
        4 => (Point2::from_values(0, 0), "1e400".parse::<Real>().unwrap()),
        _ => unreachable!(),
    };
    let transform = |x: Real, y: Real| Point2::new(center.x() + &scale * x, center.y() + &scale * y);
    let start = transform(Real::one(), Real::zero());
    let end = transform(Real::zero(), Real::one());
    let point = match query {
        0 => start.clone(),
        1 => end.clone(),
        2 => transform(Real::one(), Real::zero()),
        3 => transform(Real::zero(), Real::one()),
        4 => transform(Real::from(-1), Real::zero()),
        5 => center.clone(),
        6 => transform((Real::from(3) / Real::from(5)).unwrap(), (Real::from(4) / Real::from(5)).unwrap()),
        7 => transform(Real::one() + "1e-40".parse::<Real>().unwrap(), Real::zero()),
        _ => unreachable!(),
    };
    let arc = CircularArc2::try_from_center(start, end, center, false).unwrap();
    (arc, point)
}

#[test]
fn coordinate_facts_preserve_unknown_and_lazy_y_queries() {
    let origin = Point2::from_values(0, 0);
    assert_eq!(origin.coordinate_equality(&origin.clone()), Some(true));
    assert_eq!(origin.coordinate_equality(&Point2::from_values(0, 0)), Some(true));
    assert_eq!(origin.coordinate_equality(&Point2::from_values(1, 0)), Some(false));
    assert_eq!(origin.coordinate_equality(&Point2::from_values(0, 1)), Some(false));
    let zero = unresolved_zero();
    assert_eq!(zero.zero_status(), ZeroKnowledge::Unknown);
    let untouched_y = Real::from(19).sin();
    assert!(untouched_y.cached_rational_interval(-256).is_none());
    let unknown = Point2::new(zero, untouched_y);
    assert_eq!(origin.coordinate_equality(&unknown), None);
    assert!(unknown.y().cached_rational_interval(-256).is_none());
    assert_eq!(origin.coordinate_equality(&Point2::new(Real::zero(), unresolved_zero())), None);
    for precision in [-8, -128, -16] {
        unknown.x().certified_rational_interval(precision).unwrap();
        assert_eq!(origin.coordinate_equality(&unknown.clone()), None);
    }
}

#[test]
fn endpoint_receipts_match_independent_controls_across_contexts_and_warming() {
    for representation in 0..5 {
        for query in 0..8 {
            for context in 0..7 {
                let (arc, point) = fixture(representation, query);
                let (control, control_point) = fixture(representation, query);
                for stage in 0..3 {
                    if stage == 1 {
                        for p in [&point, &control_point] {
                            for coordinate in [p.x(), p.y()] {
                                coordinate.certified_rational_interval(-128).unwrap();
                            }
                        }
                    }
                    let actual = provisional(|| in_context(context, |policy| arc.clone().contains_endpoint(&point.clone(), policy)));
                    let expected = provisional(|| in_context(context, |policy| original(&control.clone(), &control_point.clone(), policy)));
                    assert_eq!(actual, expected, "representation {representation}, query {query}, context {context}, stage {stage}");
                    assert_eq!(*actual.as_unverified(), Some(query < 4));
                    assert_eq!(actual.certainty(), CurveCertainty::Certified);
                }
            }
        }
    }
}

#[test]
fn unknown_endpoint_retries_preserve_approximation_receipts() {
    for context in [2, 5] {
        let (arc, _) = fixture(0, 0);
        let (control, _) = fixture(0, 0);
        let point = Point2::new(Real::one() + unresolved_zero(), Real::zero());
        let control_point = Point2::new(Real::one() + unresolved_zero(), Real::zero());
        assert_eq!(point.coordinate_equality(arc.start()), None);
        for _ in 0..2 {
            let actual = provisional(|| in_context(context, |policy| arc.contains_endpoint(&point, policy)));
            let expected = provisional(|| in_context(context, |policy| original(&control, &control_point, policy)));
            assert_eq!(actual, expected);
            assert_eq!(*actual.as_unverified(), Some(true));
            if context == 2 {
                assert_eq!(actual.certainty(), CurveCertainty::Approximate512Consumed);
            }
        }
    }
}

#[test]
fn final_coordinate_abort_preserves_shared_equality_and_unknown_facts() {
    let signal = Arc::new(AtomicBool::new(true));
    let mut coordinate = Real::one() + unresolved_zero();
    coordinate.abort(signal.clone());
    let point = Point2::new(coordinate, Real::zero());
    let (arc, _) = fixture(0, 0);
    for active in [true, false, true, false] {
        signal.store(active, Ordering::Relaxed);
        assert_eq!(point.x().certified_rational_interval(-128).is_none(), active);
        assert_eq!(point.coordinate_equality(&point.clone()), Some(true));
        assert_eq!(point.coordinate_equality(arc.start()), None);
        if active {
            let actual = provisional(|| arc.contains_endpoint(&point, &CurveContext::STRICT));
            let expected = provisional(|| original(&arc, &point, &CurveContext::STRICT));
            assert_eq!(actual, expected);
            assert_eq!(*actual.as_unverified(), None);
            assert_eq!(actual.certainty(), CurveCertainty::Certified);
            assert!(point.x().certified_rational_interval(-256).is_none());
        }
    }
}

#[test]
fn certified_negative_endpoints_reuse_proofs_inside_shared_budgets() {
    use hypersolve::work_budget::{WorkKind, consume, with_budget};
    let (arc, point) = fixture(1, 6);
    let negative = || arc.contains_endpoint(&point, &CurveContext::STRICT);
    for _ in 0..3 {
        let result = with_budget(WorkBudget::ZERO, negative);
        assert_eq!(result.result, Ok(Some(false)));
        assert_eq!(result.usage.refinements, 0);
        assert_eq!(result.usage.promotions, 0);
    }
    let outer = with_budget(WorkBudget { refinements: 1, promotions: 0 }, || {
        consume(WorkKind::RootRefinement, 1).unwrap();
        assert_eq!(with_budget(WorkBudget::ZERO, negative).result, Ok(Some(false)));
        assert!(consume(WorkKind::RootRefinement, 1).is_err());
        negative()
    });
    assert!(outer.result.is_err(), "a cached proof cannot hide outer budget exhaustion");
    assert_eq!(outer.usage.refinements, 1);
    assert_eq!(outer.usage.promotions, 0);
}
