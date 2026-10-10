use super::{CircularArc2, CurveContext, Point2, is_zero};
use crate::{Classification, CurveCertainty, CurvePreviewOptions};
use hyperreal::{Real, ZeroKnowledge};
use std::fmt::Debug;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

struct Case {
    id: usize,
    end: usize,
    clockwise: bool,
    alias: usize,
    x: &'static str,
    y: &'static str,
    member: bool,
    endpoint: bool,
    on_circle: bool,
}

include!("arc_endpoint_cases.rs");

struct Fixture {
    arc: CircularArc2,
    point: Point2,
}

fn fraction(text: &str) -> Real {
    let (numerator, denominator) = text.split_once('/').unwrap_or((text, "1"));
    (numerator.parse::<Real>().unwrap() / denominator.parse::<Real>().unwrap()).unwrap()
}

fn fixture(case: &Case, representation: usize, reversed: bool) -> Fixture {
    let (center, scale, cosine, sine) = match representation {
        0 => (
            Point2::from_values(0, 0),
            Real::one(),
            Real::one(),
            Real::zero(),
        ),
        1 => (
            Point2::from_values(137, -91),
            Real::from(-3),
            fraction("3/5"),
            fraction("4/5"),
        ),
        2 => (
            Point2::new(fraction("1e400"), fraction("-1e400")),
            fraction("1e390"),
            fraction("3/5"),
            fraction("4/5"),
        ),
        3 => (
            Point2::from_values(0, 0),
            fraction("1e-400"),
            fraction("3/5"),
            fraction("4/5"),
        ),
        4 => {
            let half_root = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
            (
                Point2::new(Real::from(2).sqrt().unwrap(), Real::from(3).sqrt().unwrap()),
                Real::from(5).sqrt().unwrap(),
                half_root.clone(),
                half_root,
            )
        }
        5 => (
            Point2::new(Real::pi(), Real::e()),
            Real::pi(),
            fraction("3/5"),
            fraction("4/5"),
        ),
        6 => (
            Point2::new(Real::e().sin(), Real::e().cos()),
            Real::one(),
            fraction("3/5"),
            fraction("4/5"),
        ),
        _ => unreachable!(),
    };
    let transform = |x: Real, y: Real| {
        Point2::new(
            center.x() + &scale * (&cosine * &x - &sine * &y),
            center.y() + &scale * (&sine * x + &cosine * y),
        )
    };
    let start = transform(Real::one(), Real::zero());
    let (ex, ey) = [(0, 1), (-1, 0), (0, -1), (1, 0)][case.end];
    let end = transform(Real::from(ex), Real::from(ey));
    // The Fraction oracle and similarity identities prove these endpoint radii.
    let mut arc = CircularArc2::new_with_certified_radius(
        start,
        end,
        center.clone(),
        &scale * &scale,
        case.clockwise,
        None,
    );
    if reversed {
        arc = arc.reversed();
    }
    let point = match case.alias {
        0 => transform(fraction(case.x), fraction(case.y)),
        1 => arc.start().clone(),
        2 => arc.end().clone(),
        _ => unreachable!(),
    };
    Fixture { arc, point }
}

fn in_context<T>(context: usize, evaluate: impl FnOnce(&CurveContext) -> T) -> T {
    match context {
        0 => evaluate(&CurveContext::STRICT),
        1 => CurveContext::STRICT.bounded_exact_predicate_pass(|| evaluate(&CurveContext::STRICT)),
        2 => evaluate(&CurveContext::APPROXIMATE_512),
        3 => CurveContext::APPROXIMATE_512
            .bounded_exact_predicate_pass(|| evaluate(&CurveContext::APPROXIMATE_512)),
        4 => CurvePreviewOptions::try_strict(1e-8, 1e-8)
            .unwrap()
            .evaluate(|| evaluate(&crate::policy::principal_context())),
        5 => CurvePreviewOptions::try_approximate_512(1e-8, 1e-8)
            .unwrap()
            .evaluate(|| evaluate(&crate::policy::principal_context())),
        6 => CurveContext::APPROXIMATE_512
            .strict_predicate_pass(|| evaluate(&CurveContext::APPROXIMATE_512)),
        _ => unreachable!(),
    }
}

fn observed<T: Debug>(
    rows: &mut Vec<String>,
    name: &str,
    evaluate: impl FnOnce() -> T,
) -> (T, CurveCertainty) {
    let (value, certainty) = crate::policy::arc_endpoint_probe_observe(evaluate);
    rows.push(format!(
        "[{name:?},{:?},{:?}]",
        format!("{value:?}"),
        format!("{certainty:?}")
    ));
    (value, certainty)
}

fn check_member(
    value: &Classification<bool>,
    certainty: CurveCertainty,
    case: Option<&Case>,
    policy: &CurveContext,
) {
    if !policy.is_edge_preview()
        && certainty == CurveCertainty::Certified
        && let (Some(case), Classification::Decided(member)) = (case, value)
    {
        assert_eq!(
            *member, case.member,
            "independent membership oracle case {}",
            case.id
        );
    }
}

fn receipt(fixture: &Fixture, case: Option<&Case>, context: usize) -> String {
    in_context(context, |policy| {
        let mut rows = Vec::new();
        let (member, certainty) = observed(&mut rows, "membership", || {
            fixture
                .arc
                .contains_point_with_policy(&fixture.point, policy)
        });
        check_member(&member, certainty, case, policy);
        observed(&mut rows, "sweep_after", || {
            fixture
                .arc
                .contains_sweep_point_with_policy(&fixture.point, policy)
        });
        let (endpoint, certainty) = observed(&mut rows, "endpoint_after", || {
            fixture.arc.contains_endpoint(&fixture.point, policy)
        });
        if !policy.is_edge_preview()
            && certainty == CurveCertainty::Certified
            && let (Some(case), Some(endpoint)) = (case, endpoint)
        {
            assert_eq!(endpoint, case.endpoint, "endpoint oracle case {}", case.id);
        }
        observed(&mut rows, "location_after", || {
            fixture
                .arc
                .strict_sweep_point_location(&fixture.point, policy)
        });
        observed(&mut rows, "point_equality_after", || {
            (
                fixture.point == *fixture.arc.start(),
                fixture.point == *fixture.arc.end(),
            )
        });
        let (circle, certainty) = observed(&mut rows, "circle_after", || {
            is_zero(
                &(fixture.point.distance_squared(fixture.arc.center())
                    - fixture.arc.radius_squared()),
                policy,
            )
        });
        if !policy.is_edge_preview()
            && certainty == CurveCertainty::Certified
            && let (Some(case), Some(circle)) = (case, circle)
        {
            assert_eq!(circle, case.on_circle, "circle oracle case {}", case.id);
        }
        let (member, certainty) = observed(&mut rows, "membership_after", || {
            fixture
                .arc
                .contains_point_with_policy(&fixture.point, policy)
        });
        check_member(&member, certainty, case, policy);
        format!("[{}]", rows.join(","))
    })
}

fn warm(fixture: &Fixture, precision: i32) -> usize {
    [
        fixture.arc.start(),
        fixture.arc.end(),
        fixture.arc.center(),
        &fixture.point,
    ]
    .into_iter()
    .flat_map(|point| [point.x(), point.y()])
    .filter(|coordinate| coordinate.certified_rational_interval(precision).is_some())
    .count()
}

#[test]
fn arc_endpoint_frontier_matrix() {
    let mut count = 0;
    for representation in 0..7 {
        for case in CASES {
            for reversed in [false, true] {
                for context in 0..7 {
                    let original = fixture(case, representation, reversed);
                    for stage in ["fresh", "repeated", "cloned", "prepared"] {
                        let current = match stage {
                            "cloned" => Fixture {
                                arc: original.arc.clone(),
                                point: original.point.clone(),
                            },
                            "prepared" => fixture(case, representation, reversed),
                            _ => Fixture {
                                arc: original.arc.clone(),
                                point: original.point.clone(),
                            },
                        };
                        let prepared = if stage == "prepared" {
                            warm(&current, -128)
                        } else {
                            0
                        };
                        let value = receipt(&current, Some(case), context);
                        println!(
                            "ARC_ENDPOINT_FRONTIER {{\"case\":{},\"representation\":{representation},\"reversed\":{reversed},\"context\":{context},\"stage\":{stage:?},\"prepared_coordinates\":{prepared},\"queries\":{value}}}",
                            case.id
                        );
                        count += 1;
                    }
                }
            }
        }
    }
    assert_eq!(count, CASES.len() * 7 * 2 * 7 * 4);
    println!("ARC_ENDPOINT_FRONTIER_SUMMARY {{\"matrix_rows\":{count}}}");
}

fn unresolved(aborted: bool) -> (Fixture, Arc<AtomicBool>) {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let mut zero = &sine * &sine + &cosine * &cosine - Real::one();
    assert_eq!(zero.zero_status(), ZeroKnowledge::Unknown);
    let signal = Arc::new(AtomicBool::new(aborted));
    if aborted {
        zero.abort(signal.clone());
    }
    let arc = CircularArc2::new_with_certified_radius(
        Point2::from_values(1, 0),
        Point2::from_values(0, 1),
        Point2::from_values(0, 0),
        Real::one(),
        false,
        None,
    );
    (
        Fixture {
            arc,
            point: Point2::new(Real::one() + zero, Real::zero()),
        },
        signal,
    )
}

#[test]
fn arc_endpoint_frontier_unresolved_and_aborted() {
    let mut count = 0;
    for aborted in [false, true] {
        // An unproved exact zero is queried under bounded exact passes; the
        // active abort signal also bounds the ordinary strict/approximate paths.
        let contexts: &[usize] = if aborted {
            &[0, 1, 2, 3, 4, 5, 6]
        } else {
            &[1, 3]
        };
        for &context in contexts {
            let (fixture, signal) = unresolved(aborted);
            for stage in ["fresh", "prepared", "repeated"] {
                let prepared = if stage == "prepared" {
                    warm(&fixture, -128)
                } else {
                    0
                };
                let value = receipt(&fixture, None, context);
                println!(
                    "ARC_ENDPOINT_SPECIAL {{\"aborted\":{aborted},\"context\":{context},\"stage\":{stage:?},\"prepared_coordinates\":{prepared},\"queries\":{value}}}"
                );
                count += 1;
            }
            signal.store(false, Ordering::Relaxed);
            let value = receipt(&fixture, None, 1);
            println!(
                "ARC_ENDPOINT_SPECIAL {{\"aborted\":{aborted},\"context\":{context},\"stage\":\"abort_cleared_bounded\",\"queries\":{value}}}"
            );
            count += 1;
        }
    }
    println!("ARC_ENDPOINT_SPECIAL_SUMMARY {{\"rows\":{count}}}");
}

#[test]
fn arc_endpoint_frontier_shared_queries() {
    let mut count = 0;
    for representation in 0..7 {
        for case in CASES
            .iter()
            .filter(|case| [0, 2, 12, 14, 17, 18].contains(&(case.id % 19)))
        {
            let fixture = fixture(case, representation, false);
            let rows = std::thread::scope(|scope| {
                let handles: Vec<_> = (0..4).map(|slot| {
                    let fixture = &fixture;
                    scope.spawn(move || {
                        [-16, -128, -32].into_iter().map(|precision| {
                            let prepared = warm(fixture, precision);
                            let value = receipt(fixture, Some(case), slot % 2);
                            format!("ARC_ENDPOINT_CONCURRENT {{\"case\":{},\"representation\":{representation},\"slot\":{slot},\"precision\":{precision},\"prepared_coordinates\":{prepared},\"queries\":{value}}}", case.id)
                        }).collect::<Vec<_>>()
                    })
                }).collect();
                handles
                    .into_iter()
                    .flat_map(|handle| handle.join().unwrap())
                    .collect::<Vec<_>>()
            });
            count += rows.len();
            for row in rows {
                println!("{row}");
            }
        }
    }
    println!("ARC_ENDPOINT_CONCURRENT_SUMMARY {{\"rows\":{count}}}");
}
