//! Independent orientation controls for admitting interval filter stages.

use hyperlimit::{Certainty, Escalation, Point2, PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::Real;

const CASES: [(&str, Sign, Escalation, &str); 9] = [
    (
        "integer",
        Sign::Positive,
        Escalation::Exact,
        "certified-real-det2-filter",
    ),
    (
        "thirds",
        Sign::Positive,
        Escalation::Exact,
        "exact-word-rational-det2-filter",
    ),
    (
        "rational-near",
        Sign::Positive,
        Escalation::Exact,
        "exact-word-rational-det2-filter",
    ),
    (
        "algebraic-axis",
        Sign::Positive,
        Escalation::Structural,
        "real-determinant",
    ),
    (
        "algebraic-cancellation",
        Sign::Negative,
        Escalation::Structural,
        "real-determinant",
    ),
    (
        "transcendental-separated",
        Sign::Negative,
        Escalation::Refined,
        "real-determinant",
    ),
    (
        "transcendental-translated",
        Sign::Negative,
        Escalation::Structural,
        "real-determinant",
    ),
    (
        "transcendental-near",
        Sign::Positive,
        Escalation::Refined,
        "real-determinant",
    ),
    (
        "transcendental-collinear",
        Sign::Zero,
        Escalation::Structural,
        "real-determinant",
    ),
];

fn points(index: usize) -> [Point2; 3] {
    let origin = Point2::new(0.into(), 0.into());
    match index {
        0 => [
            origin,
            Point2::new(1.into(), 0.into()),
            Point2::new(0.into(), 1.into()),
        ],
        1 => {
            let third = (Real::one() / Real::from(3)).unwrap();
            [
                origin,
                Point2::new(third.clone(), 0.into()),
                Point2::new(0.into(), third),
            ]
        }
        2 => [
            origin,
            Point2::new(1.into(), 1.into()),
            Point2::new(
                1.into(),
                Real::one() + Real::from(2).powi_i64(-100).unwrap(),
            ),
        ],
        3 => [
            origin,
            Point2::new(Real::from(2).sqrt().unwrap(), 0.into()),
            Point2::new(0.into(), Real::from(3).sqrt().unwrap()),
        ],
        4 => {
            let u = Real::from(2).sqrt().unwrap();
            let v = Real::from(3).sqrt().unwrap();
            [origin, Point2::new(u.clone(), v.clone()), Point2::new(v, u)]
        }
        // e < 3 < pi independently proves e²-pi² < 0.
        5 => [
            origin,
            Point2::new(Real::e(), Real::pi()),
            Point2::new(Real::pi(), Real::e()),
        ],
        // Translation cancels, leaving the integer determinant 1*4-2*3=-2.
        6 => {
            let u = Real::one().sin();
            let v = Real::from(2).cos();
            [
                Point2::new(u.clone(), v.clone()),
                Point2::new(&u + Real::one(), &v + Real::from(2)),
                Point2::new(&u + Real::from(3), &v + Real::from(4)),
            ]
        }
        // Determinant e*2^-100 is positive; the limiting collinear case is zero.
        7 | 8 => {
            let delta = if index == 7 {
                Real::from(2).powi_i64(-100).unwrap()
            } else {
                Real::zero()
            };
            [
                origin,
                Point2::new(Real::e(), Real::pi()),
                Point2::new(
                    Real::e() * Real::from(2),
                    Real::pi() * Real::from(2) + delta,
                ),
            ]
        }
        _ => unreachable!(),
    }
}

fn check_frontier() {
    for (i, (name, sign, stage, _)) in CASES.iter().enumerate() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let [a, b, c] = points(i);
            assert_eq!(
                hyperlimit::orient2(&a, &b, &c, policy),
                PredicateOutcome::decided(*sign, Certainty::Exact, *stage),
                "{name}: {policy:?}"
            );
        }
    }
}

#[test]
fn interval_filter_frontier_preserves_exact_results_and_receipts() {
    check_frontier();
}

#[test]
fn generated_translations_and_reversal_preserve_orientation() {
    for (i, (name, sign, _, _)) in CASES.iter().enumerate() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let [a, b, c] =
                points(i).map(|p| Point2::new(p.x + Real::from(7), p.y - Real::from(5)));
            for (b, c, want) in [(&b, &c, *sign), (&c, &b, sign.reversed())] {
                assert!(
                    matches!(
                        hyperlimit::orient2(&a, b, c, policy),
                        PredicateOutcome::Decided { value, certainty: Certainty::Exact, .. } if value == want
                    ),
                    "{name}: {policy:?}"
                );
            }
        }
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn admitted_stage_counts_preserve_structural_and_rational_shortcuts() {
    use hyperreal::dispatch_trace as trace;
    for (i, (name, sign, stage, path)) in CASES.iter().enumerate() {
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let [a, b, c] = points(i);
            trace::reset();
            let outcome = trace::with_recording(|| hyperlimit::orient2(&a, &b, &c, policy));
            assert_eq!(
                outcome,
                PredicateOutcome::decided(*sign, Certainty::Exact, *stage)
            );
            let snapshot = trace::take_trace();
            assert_eq!(
                snapshot.path_count("hyperlimit", "orient2d", path),
                1,
                "{name}"
            );
            assert_eq!(
                snapshot.path_count("hyperlimit", "resolve_real_sign", "structural-real-facts"),
                u64::from(*stage == Escalation::Structural),
                "{name}"
            );
            assert_eq!(
                snapshot.path_count("hyperlimit", "resolve_real_sign", "real-refinement"),
                u64::from(*stage == Escalation::Refined),
                "{name}"
            );
            assert_eq!(
                snapshot.path_count(
                    "hyperlimit",
                    "resolve_real_sign",
                    "policy-final-approximation"
                ),
                0,
                "{name}"
            );
        }
    }
}

#[test]
fn frontier_replays_in_a_fresh_process() {
    const KEY: &str = "HYPERLIMIT_INTERVAL_FRONTIER_REPLAY";
    if std::env::var_os(KEY).is_some() {
        check_frontier();
        return;
    }
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "frontier_replays_in_a_fresh_process",
            "--nocapture",
        ])
        .env(KEY, "1")
        .status()
        .unwrap();
    assert!(status.success());
}
