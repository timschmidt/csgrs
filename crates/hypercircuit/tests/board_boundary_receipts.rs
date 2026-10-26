#![cfg(feature = "layout")]

use hypercircuit::{BoardBoundaryGeometry, BoardContour, BoardOutline, Real};
use hypercurve::{Classification, CurveCertainty, RegionPointLocation, evaluate_under};
use hyperlattice::Point2;
use hyperlimit::PredicatePolicy;
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn board() -> BoardBoundaryGeometry {
    // The authored outline drops before any query uses the retained geometry.
    BoardOutline::rectangle(10.into(), 10.into())
        .unwrap()
        .boundary_geometry(PredicatePolicy::STRICT)
        .unwrap()
}

fn symbolic_zero() -> Real {
    // Independently zero by the sine addition formula, but undecided by the
    // current strict predicate kernel. This is a real terminal-consumption gate.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
}

#[test]
fn all_five_queries_retain_terminal_receipts_and_strict_retries() {
    let board = board();
    let z = symbolic_zero();
    let boundary = Point2::new(z.clone(), 5.into());
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        // Mapping only interprets the returned values; it must preserve the
        // original receipt, including the nested classification in each query.
        let outcomes = [
            board
                .classify_point_outcome(&boundary, policy)
                .map(|result| {
                    result.map(|value| {
                        value.map(|location| location == RegionPointLocation::Boundary)
                    })
                }),
            board
                .contains_disc_outcome(&boundary, 0.into(), policy)
                .map(|result| result.map(|value| value.map(|inside| !inside))),
            board
                .contains_segment_outcome(&boundary, &p(5, 5), 0.into(), policy)
                .map(|result| result.map(|value| value.map(|inside| !inside))),
            board
                .contains_axis_aligned_box_outcome(
                    &Point2::new(z.clone(), 3.into()),
                    &p(7, 7),
                    policy,
                )
                .map(|result| result.map(|value| value.map(|inside| !inside))),
            board.inset_outcome(z.clone(), policy).map(|result| {
                result
                    .map(|value| value.map(|inset| inset.exterior_bounds() == (p(0, 0), p(10, 10))))
            }),
        ];
        for outcome in outcomes {
            if policy == PredicatePolicy::STRICT {
                assert_eq!(outcome.certainty(), CurveCertainty::Certified);
                assert!(matches!(
                    outcome.certified().unwrap().unwrap(),
                    Classification::Uncertain(_)
                ));
            } else {
                assert_eq!(outcome.certainty(), CurveCertainty::Approximate512Consumed);
                assert!(outcome.clone().certified().is_none());
                assert_eq!(
                    outcome.into_unverified().unwrap(),
                    Classification::Decided(true)
                );
            }
        }
    }
}

#[test]
fn choosing_approximate_policy_keeps_exact_queries_certified() {
    let board = board();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let point = board.classify_point_outcome(&p(5, 5), policy);
        assert_eq!(
            point.certified().unwrap().unwrap(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        for outcome in [
            board.contains_disc_outcome(&p(5, 5), 1.into(), policy),
            board.contains_segment_outcome(&p(4, 5), &p(6, 5), 1.into(), policy),
            board.contains_axis_aligned_box_outcome(&p(3, 3), &p(7, 7), policy),
        ] {
            assert_eq!(
                outcome.certified().unwrap().unwrap(),
                Classification::Decided(true)
            );
        }
        let inset = board
            .inset_outcome(1.into(), policy)
            .certified()
            .unwrap()
            .unwrap();
        let Classification::Decided(inset) = inset else {
            panic!("exact inset must complete")
        };
        assert_eq!(inset.exterior_bounds(), (p(1, 1), p(9, 9)));
        assert_eq!(
            inset
                .classify_point_outcome(&p(5, 5), PredicatePolicy::STRICT)
                .certified()
                .unwrap()
                .unwrap(),
            Classification::Decided(RegionPointLocation::Inside),
        );
    }
}

#[test]
fn nested_policy_scopes_report_consumption_and_restore_the_caller() {
    let board = board();
    let boundary = Point2::new(symbolic_zero(), 5.into());
    let outer = evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
        let strict = board.classify_point_outcome(&boundary, PredicatePolicy::STRICT);
        assert!(matches!(
            strict.certified().unwrap().unwrap(),
            Classification::Uncertain(_)
        ));
        board.contains_disc_outcome(&boundary, 0.into(), PredicatePolicy::APPROXIMATE_512)
    });
    assert_eq!(outer.certainty(), CurveCertainty::Approximate512Consumed);
    let inner = outer.into_unverified();
    assert_eq!(inner.certainty(), CurveCertainty::Approximate512Consumed);
    assert_eq!(
        inner.into_unverified().unwrap(),
        Classification::Decided(false)
    );
    assert!(matches!(
        board
            .classify_point_outcome(&boundary, PredicatePolicy::STRICT)
            .certified()
            .unwrap()
            .unwrap(),
        Classification::Uncertain(_),
    ));
}

#[test]
fn malformed_query_is_an_error_and_does_not_contaminate_later_receipts() {
    let board = board();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let outcome = board.contains_segment_outcome(&p(5, 5), &p(5, 5), 0.into(), policy);
        assert!(outcome.certified().unwrap().is_err());
        assert_eq!(
            board
                .contains_disc_outcome(&p(5, 5), 1.into(), policy)
                .certified()
                .unwrap()
                .unwrap(),
            Classification::Decided(true),
        );
    }
}

#[test]
fn mixed_boundary_queries_preserve_holes_and_exact_receipts() {
    let policy = PredicatePolicy::STRICT;
    let line = |a, b| LinePathSegment::new(a, b, policy).unwrap().into();
    let board = BoardOutline {
        // A 10 by 10 rectangle capped by the upper radius-five semicircle.
        exterior: BoardContour::from_segments(vec![
            line(p(0, 0), p(10, 0)),
            line(p(10, 0), p(10, 10)),
            ExplicitCircularArc::new(
                p(5, 10),
                5.into(),
                p(10, 10),
                p(0, 10),
                ArcDirection::Ccw,
                policy,
            )
            .unwrap()
            .into(),
            line(p(0, 10), p(0, 0)),
        ]),
        cutouts: vec![BoardContour::polygon(vec![p(2, 2), p(4, 2), p(4, 4), p(2, 4)]).unwrap()],
    }
    .boundary_geometry(policy)
    .unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        assert_eq!(
            board
                .classify_point_outcome(&p(3, 3), policy)
                .certified()
                .unwrap()
                .unwrap(),
            Classification::Decided(RegionPointLocation::Outside),
        );
        for (outcome, expected) in [
            (
                board.contains_disc_outcome(&p(5, 14), 2.into(), policy),
                false,
            ),
            (
                board.contains_disc_outcome(&p(6, 6), 1.into(), policy),
                true,
            ),
            (
                board.contains_segment_outcome(&p(1, 3), &p(5, 3), 0.into(), policy),
                false,
            ),
            (
                board.contains_axis_aligned_box_outcome(&p(1, 1), &p(5, 5), policy),
                false,
            ),
            (
                board.contains_axis_aligned_box_outcome(&p(5, 5), &p(7, 7), policy),
                true,
            ),
        ] {
            assert_eq!(
                outcome.certified().unwrap().unwrap(),
                Classification::Decided(expected)
            );
        }
    }
}

#[test]
fn exact_near_boundary_queries_are_not_merged_by_approximate_policy() {
    let board = board();
    let epsilon = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (x, expected) in [
            (epsilon.clone(), RegionPointLocation::Inside),
            (-epsilon.clone(), RegionPointLocation::Outside),
        ] {
            assert_eq!(
                board
                    .classify_point_outcome(&Point2::new(x, 5.into()), policy)
                    .certified()
                    .unwrap()
                    .unwrap(),
                Classification::Decided(expected),
            );
        }
    }
}
