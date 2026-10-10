//! Calibrate aborts on the final query coordinates before comparing public calls.
use hypercurve::{
    CircularArc2, CurvePreviewOptions, ExactCurveResult, Point2, PredicatePolicy, Provisional,
    Real, WorkBudget, evaluate_under, evaluate_with_budget, provisional,
};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

type Queries = (ExactCurveResult<bool>, (bool, bool), ExactCurveResult<bool>);

struct Fixture {
    arc: CircularArc2,
    point: Point2,
    signal: Arc<AtomicBool>,
    non_rational: usize,
    expected: bool,
}

fn fraction(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn fixture(family: usize) -> Fixture {
    let (center, scale) = match family {
        3 | 5 => (Point2::new(Real::pi(), Real::e()), Real::pi()),
        4 => (Point2::new(Real::e().sin(), Real::e().cos()), Real::one()),
        _ => (Point2::from_values(0, 0), Real::one()),
    };
    let transform =
        |x: Real, y: Real| Point2::new(center.x() + &scale * x, center.y() + &scale * y);
    let arc = CircularArc2::try_from_center(
        transform(Real::one(), Real::zero()),
        transform(Real::zero(), Real::one()),
        center.clone(),
        false,
    )
    .unwrap();
    let original = match family {
        0 => {
            let sine = Real::e().sin();
            let cosine = Real::e().cos();
            let zero = &sine * &sine + &cosine * &cosine - Real::one();
            transform(Real::one() + zero, Real::zero())
        },
        1 => {
            let root = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
            transform(root.clone(), root)
        },
        2 => transform(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap()),
        5 => arc.start().clone(),
        _ => transform(fraction(3, 5), fraction(4, 5)),
    };
    let signal = Arc::new(AtomicBool::new(true));
    let mut coordinates = [original.x().clone(), original.y().clone()];
    let mut non_rational = 0;
    for coordinate in &mut coordinates {
        if coordinate.exact_rational_ref().is_none() {
            coordinate.abort(signal.clone());
            assert!(coordinate.certified_rational_interval(-128).is_none());
            non_rational += 1;
        }
    }
    assert!(non_rational > 0);
    let [x, y] = coordinates;
    Fixture {
        arc,
        point: Point2::new(x, y),
        signal,
        non_rational,
        expected: family != 2,
    }
}

fn query(fixture: &Fixture) -> Queries {
    (
        fixture.arc.contains_point(&fixture.point),
        (
            fixture.point == *fixture.arc.start(),
            fixture.point == *fixture.arc.end(),
        ),
        fixture.arc.contains_point(&fixture.point),
    )
}

fn check(value: &Provisional<Queries>, fixture: &Fixture) {
    if value.is_certified() {
        for result in [&value.as_unverified().0, &value.as_unverified().2] {
            if let Ok(member) = result {
                assert_eq!(*member, fixture.expected);
            }
        }
    }
}

fn receipt(fixture: &Fixture, context: usize) -> String {
    match context {
        0 | 1 | 6 => {
            let evaluate = || {
                evaluate_under(
                    if context == 1 {
                        PredicatePolicy::APPROXIMATE_512
                    } else {
                        PredicatePolicy::STRICT
                    },
                    || query(fixture),
                )
            };
            let result = if context == 6 {
                provisional(evaluate).into_unverified()
            } else {
                evaluate()
            };
            check(&result, fixture);
            format!("{result:?}")
        },
        2 | 3 => {
            let preview = if context == 2 {
                CurvePreviewOptions::try_strict(1e-8, 1e-8)
            } else {
                CurvePreviewOptions::try_approximate_512(1e-8, 1e-8)
            }
            .unwrap();
            // Preview is a separately labelled lossy context; no topology oracle claim.
            format!("{:?}", provisional(|| preview.evaluate(|| query(fixture))))
        },
        4 | 5 => {
            let result = evaluate_with_budget(
                if context == 4 {
                    PredicatePolicy::STRICT
                } else {
                    PredicatePolicy::APPROXIMATE_512
                },
                WorkBudget::ZERO,
                || query(fixture),
            );
            if let Ok(value) = &result.result {
                check(value, fixture);
            }
            format!("{result:?}")
        },
        _ => unreachable!(),
    }
}

fn availability(fixture: &Fixture, precision: i32, active: bool) -> usize {
    let mut available = 0;
    for coordinate in [fixture.point.x(), fixture.point.y()] {
        let interval = coordinate.certified_rational_interval(precision);
        let must_be_available = !active || coordinate.exact_rational_ref().is_some();
        assert_eq!(interval.is_some(), must_be_available);
        available += usize::from(interval.is_some());
    }
    assert_eq!(available, if active { 2 - fixture.non_rational } else { 2 });
    available
}

fn main() {
    let mut count = 0;
    for family in 0..6 {
        for context in 0..7 {
            let fixture = fixture(family);
            for (stage, active) in [
                ("active_fresh", true),
                ("active_repeated", true),
                ("cleared_prepared", false),
                ("reenabled_after_warm", true),
                ("cleared_final", false),
            ] {
                fixture.signal.store(active, Ordering::Relaxed);
                let before = availability(&fixture, -128, active);
                let value = receipt(&fixture, context);
                let after = availability(&fixture, -256, active);
                println!(
                    "{{\"family\":{family},\"context\":{context},\"stage\":{stage:?},\"abort_active\":{active},\"non_rational_query_coordinates\":{},\"available_before\":{before},\"available_after\":{after},\"receipt\":{value:?}}}",
                    fixture.non_rational
                );
                count += 1;
            }
        }
    }
    assert_eq!(count, 210);
}
