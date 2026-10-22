#![cfg(feature = "curve")]
//! Exact literals, and line and circle relations (families 1, 7 and 8).
//!
//! Downstream CAD workloads migrated into acceptance gates: each test
//! restates one workload family from the pinned SynapsCAD (`a63c4c95fa`)
//! and exactcore-hyper-comparison (`1e3b1f7a81`) sources through the current
//! `csgrs` and `hypercurve` public APIs. Those snapshots use older APIs, so
//! the semantics are adapted, not the code. Each case states its expected
//! outcome (success, empty or a typed error) and checks it against an
//! independently derived exact oracle where one is feasible. A binary64
//! tolerance or a sampled-profile bound is a smoke check, labelled as such,
//! never an exactness certificate.

#[allow(dead_code)]
#[path = "support/downstream_workloads.rs"]
mod support;

use csgrs::solid::SolidExt;
use csgrs::{GeometryCertainty, GeometryContext, Real, solid};
use hypercurve::{
    CircleCircleRelation, CircularArc2, IntersectionKind, LineCircleRelation,
    LineLineIntersection, LineSeg2, LineSide, Point2, Segment2, SegmentIntersection,
};
use hyperlimit::Sign;
use hyperreal::Rational;
use std::cmp::Ordering;
use support::*;

// ---------------------------------------------------------------------------
// Family 1: exact literals reach native bounds.
//
// SynapsCAD: `cube([1/3 + 1/6, 2/5, 3/7])` and `cube([0.1, 2.5e-1, 3E-1])`.
// OpenSCAD numeric source enters `Real` without a binary64 parse (P34), so
// the bounds are the exact rationals and the volumes their products.

#[test]
fn exact_and_decimal_literals_reach_native_bounds() {
    let width = q(1, 3) + q(1, 6);
    assert_eq!(width.exact_rational(), Some(rational(1, 2)));
    let fractions = solid::cuboid(width, q(2, 5), q(3, 7)).expect("cuboid");
    assert_bounds(
        &fractions,
        [[&r(0), &r(0), &r(0)], [&q(1, 2), &q(2, 5), &q(3, 7)]],
        "fraction literals",
    );
    assert_certified_closed_manifold(&fractions, "fraction literals");
    // Volume 1/2 * 2/5 * 3/7 = 3/35.
    assert_eq!(
        six_volume(&fractions).exact_rational(),
        Some(rational(18, 35))
    );

    let decimals = ["0.1", "2.5e-1", "3E-1"].map(literal);
    for (value, expected) in
        decimals
            .iter()
            .zip([rational(1, 10), rational(1, 4), rational(3, 10)])
    {
        assert_eq!(value.exact_rational(), Some(expected));
    }
    // The decimal spelling is not the nearest binary64 value.
    let binary64 = Real::try_from(0.1_f64).expect("finite binary64");
    assert_ne!(strict_order(&decimals[0], &binary64), Ordering::Equal);
    let [x, y, z] = decimals;
    let decimal = solid::cuboid(x, y, z).expect("cuboid");
    assert_bounds(
        &decimal,
        [[&r(0), &r(0), &r(0)], [&q(1, 10), &q(1, 4), &q(3, 10)]],
        "decimal literals",
    );
    // Volume 1/10 * 1/4 * 3/10 = 3/400.
    assert_eq!(six_volume(&decimal).exact_rational(), Some(rational(18, 400)));

    // `center = true` centres the exact bounds.
    let centered = solid::try_center(&fractions).expect("center");
    assert_bounds(
        &centered,
        [
            [&q(-1, 4), &q(-1, 5), &q(-3, 14)],
            [&q(1, 4), &q(1, 5), &q(3, 14)],
        ],
        "centered",
    );

    // A literal below binary64 resolution survives translation (P24).
    let tiny = literal("1e-30");
    assert_eq!(
        tiny.exact_rational(),
        Some(
            literal("1/1000000000000000000000000000000")
                .exact_rational()
                .unwrap()
        )
    );
    let shifted = solid::cube(r(1))
        .expect("cube")
        .translated(tiny.clone(), r(0), r(0))
        .expect("translation");
    assert_bounds(
        &shifted,
        [
            [&tiny, &r(0), &r(0)],
            [&(r(1) + tiny.clone()), &r(1), &r(1)],
        ],
        "1e-30 translation",
    );
}

#[test]
fn literal_cuboids_compose_through_a_strict_difference() {
    // [0, 1/2] x [0, 2/5] x [0, 3/7] minus [0, 1/10] x [0, 1/4] x [0, 3/10].
    // The second box lies inside the first, so inclusion-exclusion gives
    // 3/35 - 3/400 = 219/2800.
    let body = solid::cuboid(q(1, 3) + q(1, 6), q(2, 5), q(3, 7)).expect("cuboid");
    let tool =
        solid::cuboid(literal("0.1"), literal("2.5e-1"), literal("3E-1")).expect("cuboid");
    let outcome = body
        .try_difference_with_context(&tool, &GeometryContext::STRICT)
        .expect("strict difference");
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let difference = outcome.into_value();
    assert_certified_closed_manifold(&difference, "difference");
    assert_eq!(
        six_volume(&difference).exact_rational(),
        Some(rational(6 * 219, 2800))
    );
    assert_bounds(
        &difference,
        [[&r(0), &r(0), &r(0)], [&q(1, 2), &q(2, 5), &q(3, 7)]],
        "difference",
    );
}

// ---------------------------------------------------------------------------
// Family 7: line relations with exact parameter replay.
//
// exactcore-hyper-comparison `tests/curves.rs`: crossing, endpoint, overlap
// and disjoint segment pairs, and direct versus `Segment2` enum dispatch. The
// oracle solves `p + t r = q + u s` by Cramer's rule in rationals, and every
// returned parameter is replayed on its own segment.

/// Exact rational coordinates of a point.
fn rational_point(point: &Point2) -> [Rational; 2] {
    [point.x(), point.y()].map(|value| value.exact_rational().expect("rational coordinate"))
}

/// Independent line relation of two integer segments.
#[derive(Debug)]
enum LineOracle {
    None,
    /// The crossing parameters on the first and second segment.
    Point(Rational, Rational),
    /// The overlap parameter ranges on the first and second segment.
    Overlap([Rational; 2], [Rational; 2]),
}

fn line_oracle(first: [i64; 4], second: [i64; 4]) -> LineOracle {
    let [px, py, rx, ry] = [first[0], first[1], first[2] - first[0], first[3] - first[1]];
    let [qx, qy, sx, sy] = [
        second[0],
        second[1],
        second[2] - second[0],
        second[3] - second[1],
    ];
    let cross = |ax: i64, ay: i64, bx: i64, by: i64| ax * by - ay * bx;
    let denominator = cross(rx, ry, sx, sy);
    let (dx, dy) = (qx - px, qy - py);
    let in_unit = |value: &Rational| *value >= Rational::zero() && *value <= Rational::new(1);
    if denominator != 0 {
        let fraction = |numerator: i64| Rational::new(numerator) / Rational::new(denominator);
        let t = fraction(cross(dx, dy, sx, sy));
        let u = fraction(cross(dx, dy, rx, ry));
        return if in_unit(&t) && in_unit(&u) {
            LineOracle::Point(t, u)
        } else {
            LineOracle::None
        };
    }
    if cross(dx, dy, rx, ry) != 0 {
        return LineOracle::None;
    }
    // Collinear: project the second segment onto the first's parameter.
    let length = Rational::new(rx * rx + ry * ry);
    let project =
        |x: i64, y: i64| Rational::new((x - px) * rx + (y - py) * ry) / length.clone();
    let (mut start, mut end) = (project(second[0], second[1]), project(second[2], second[3]));
    if start > end {
        std::mem::swap(&mut start, &mut end);
    }
    let low = if start > Rational::zero() {
        start
    } else {
        Rational::zero()
    };
    let high = if end < Rational::new(1) {
        end
    } else {
        Rational::new(1)
    };
    if low > high {
        return LineOracle::None;
    }
    // The second segment's parameter of a point at first-parameter `t`.
    let second_length = Rational::new(sx * sx + sy * sy);
    let back = |t: &Rational| {
        let x = Rational::new(px) + t.clone() * Rational::new(rx) - Rational::new(qx);
        let y = Rational::new(py) + t.clone() * Rational::new(ry) - Rational::new(qy);
        (x * Rational::new(sx) + y * Rational::new(sy)) / second_length.clone()
    };
    if low == high {
        // Collinear segments that share only an endpoint meet in a point.
        let second_parameter = back(&low);
        return LineOracle::Point(low, second_parameter);
    }
    LineOracle::Overlap([low.clone(), high.clone()], [back(&low), back(&high)])
}

fn segment(coordinates: [i64; 4]) -> LineSeg2 {
    LineSeg2::try_new(
        p(coordinates[0], coordinates[1]),
        p(coordinates[2], coordinates[3]),
    )
    .expect("nondegenerate segment")
}

fn as_unordered(range: [Rational; 2]) -> [Rational; 2] {
    let [a, b] = range;
    if a <= b { [a, b] } else { [b, a] }
}

/// Checks one direct line relation against the oracle and replays it.
fn check_line_relation(
    first: [i64; 4],
    second: [i64; 4],
    label: &str,
) -> LineLineIntersection {
    let (a, b) = (segment(first), segment(second));
    let relation = a
        .intersect_line(&b)
        .unwrap_or_else(|error| panic!("{label}: {error:?}"));
    match (&relation, line_oracle(first, second)) {
        (LineLineIntersection::None, LineOracle::None) => {},
        (
            LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            },
            LineOracle::Point(t, u),
        ) => {
            assert_eq!(
                a_param.exact_rational(),
                Some(t.clone()),
                "{label}: first parameter"
            );
            assert_eq!(
                b_param.exact_rational(),
                Some(u.clone()),
                "{label}: second parameter"
            );
            // Replay both parameters exactly on their own segments.
            assert!(points_equal(&a.point_at(a_param.clone()), point), "{label}");
            assert!(points_equal(&b.point_at(b_param.clone()), point), "{label}");
            let interior =
                |value: &Rational| *value > Rational::zero() && *value < Rational::new(1);
            let expected = if interior(&t) && interior(&u) {
                IntersectionKind::Crossing
            } else {
                IntersectionKind::Endpoint
            };
            assert_eq!(*kind, expected, "{label}");
        },
        (
            LineLineIntersection::Overlap {
                segment,
                a_range,
                b_range,
            },
            LineOracle::Overlap(first_range, second_range),
        ) => {
            let exact =
                |range: &hypercurve::ParamRange| {
                    as_unordered([range.start(), range.end()].map(|value| {
                        value.exact_rational().expect("rational overlap parameter")
                    }))
                };
            assert_eq!(exact(a_range), as_unordered(first_range), "{label}");
            assert_eq!(exact(b_range), as_unordered(second_range), "{label}");
            // The overlap geometry is the first segment restricted to its range.
            for (parameter, endpoint) in [
                (a_range.start(), segment.start()),
                (a_range.end(), segment.end()),
            ] {
                assert!(
                    points_equal(&a.point_at(parameter.clone()), endpoint),
                    "{label}"
                );
            }
        },
        (relation, oracle) => panic!("{label}: {relation:?} but the oracle gives {oracle:?}"),
    }
    relation
}

#[test]
fn line_relations_replay_exact_parameters() {
    for (first, second, label) in [
        ([0, 0, 6, 6], [0, 6, 6, 0], "crossing"),
        ([0, 0, 6, 0], [6, 0, 8, 2], "shared endpoint"),
        ([0, 0, 6, 0], [3, 0, 3, 5], "T junction"),
        ([0, 0, 6, 0], [2, 0, 8, 0], "collinear overlap"),
        ([0, 0, 6, 0], [8, 0, 1, 0], "reversed containing overlap"),
        ([0, 0, 2, 0], [3, 0, 7, 0], "collinear disjoint"),
        ([0, 0, 2, 0], [2, 0, 7, 0], "collinear touching"),
        ([0, 0, 6, 0], [0, 1, 6, 1], "parallel"),
        ([0, 0, 6, 0], [7, -1, 7, 1], "supporting lines cross outside"),
        ([-2, 3, 6, -3], [1, -7, 3, 9], "non-dyadic crossing"),
    ] {
        check_line_relation(first, second, label);
        // Operand swap exchanges the parameters; reversal maps t to 1 - t.
        check_line_relation(second, first, label);
        check_line_relation([first[2], first[3], first[0], first[1]], second, label);
    }
}

#[test]
fn line_relations_keep_sub_binary64_separation() {
    // A gap of 2^-80 after an endpoint is disjoint, not an endpoint contact.
    let gap = (Real::one() / Real::from(2).powi(80_u32.into()).expect("power"))
        .expect("nonzero power");
    let a = segment([0, 0, 6, 0]);
    let b =
        LineSeg2::try_new(Point2::new(r(6) + gap.clone(), r(0)), p(8, 2)).expect("segment");
    assert_eq!(
        a.intersect_line(&b).expect("relation"),
        LineLineIntersection::None
    );
    let touching =
        LineSeg2::try_new(Point2::new(r(6) - gap.clone(), r(0)), p(8, 2)).expect("segment");
    let LineLineIntersection::Point { a_param, kind, .. } =
        a.intersect_line(&touching).expect("relation")
    else {
        panic!("a segment starting 2^-80 inside the first must meet it");
    };
    assert_eq!(kind, IntersectionKind::Endpoint);
    assert!(strictly_equal(
        &a_param,
        &(Real::one() - (gap / r(6)).expect("division"))
    ));
}

#[test]
fn segment_enum_dispatch_matches_direct_line_relation() {
    for (first, second) in [
        ([0, 0, 4, 4], [0, 4, 4, 0]),
        ([0, 0, 6, 0], [2, 0, 8, 0]),
        ([0, 0, 2, 0], [3, 0, 7, 0]),
    ] {
        let (a, b) = (segment(first), segment(second));
        let direct = a.intersect_line(&b).expect("direct relation");
        let dispatched = Segment2::Line(a)
            .intersect_segment(&Segment2::Line(b))
            .expect("dispatched relation");
        assert_eq!(dispatched, SegmentIntersection::LineLine(direct));
    }
}

#[test]
fn line_side_and_containment_match_the_determinant() {
    let a = segment([0, 0, 6, 0]);
    for (query, side, contained) in [
        (p(2, 3), LineSide::Left, false),
        (p(2, -3), LineSide::Right, false),
        (p(2, 0), LineSide::On, true),
        (p(6, 0), LineSide::On, true),
        (p(9, 0), LineSide::On, false),
    ] {
        assert_eq!(a.classify_point(&query).expect("side"), side);
        assert_eq!(a.contains_point(&query).expect("containment"), contained);
    }
}

// ---------------------------------------------------------------------------
// Family 8: circle relations.
//
// exactcore-hyper-comparison classified circle pairs with exactCore's clamped
// separation `max(0, d - r1 - r2)`, which is zero for coincident, secant and
// tangent pairs alike, so it cannot serve as the relation oracle. The oracle
// here compares the squared centre distance with the squared radius sum and
// difference, and every returned contact point must satisfy both circle
// equations exactly.

/// A circle of the given centre and radius, as an arc of its full support.
fn circle(center: Point2, radius: &Real) -> CircularArc2 {
    let start = Point2::new(center.x().clone() + radius.clone(), center.y().clone());
    let end = Point2::new(center.x().clone() - radius.clone(), center.y().clone());
    CircularArc2::try_from_center(start, end, center, false).expect("nonzero radius")
}

#[derive(Debug, PartialEq, Eq)]
enum CircleOracle {
    Coincident,
    Disjoint,
    Tangent,
    Secant,
}

fn circle_oracle(distance_squared: &Real, first: &Real, second: &Real) -> CircleOracle {
    let sum = (first.clone() + second.clone()) * (first.clone() + second.clone());
    let difference = (first.clone() - second.clone()) * (first.clone() - second.clone());
    match (
        strict_order(distance_squared, &sum),
        strict_order(distance_squared, &difference),
    ) {
        (Ordering::Greater, _) => CircleOracle::Disjoint,
        (Ordering::Equal, _) => CircleOracle::Tangent,
        (Ordering::Less, Ordering::Greater) => CircleOracle::Secant,
        (Ordering::Less, Ordering::Equal) if strict_sign(distance_squared) == Sign::Zero => {
            CircleOracle::Coincident
        },
        (Ordering::Less, Ordering::Equal) => CircleOracle::Tangent,
        (Ordering::Less, Ordering::Less) => CircleOracle::Disjoint,
    }
}

fn on_circle(point: &Point2, center: &Point2, radius: &Real) -> bool {
    strictly_equal(
        &point.distance_squared(center),
        &(radius.clone() * radius.clone()),
    )
}

fn check_circle_pair(center: Point2, radius: Real, label: &str) -> CircleCircleRelation {
    let origin = p(0, 0);
    let five = r(5);
    let relation = circle(origin.clone(), &five)
        .circle_relation(&circle(center.clone(), &radius))
        .unwrap_or_else(|error| panic!("{label}: {error:?}"));
    let expected = circle_oracle(&center.distance_squared(&origin), &five, &radius);
    let actual = match &relation {
        CircleCircleRelation::Coincident => CircleOracle::Coincident,
        CircleCircleRelation::Disjoint => CircleOracle::Disjoint,
        CircleCircleRelation::Tangent { point } => {
            assert!(on_circle(point, &origin, &five), "{label}");
            assert!(on_circle(point, &center, &radius), "{label}");
            CircleOracle::Tangent
        },
        CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => {
            for point in [first_point, second_point] {
                assert!(on_circle(point, &origin, &five), "{label}");
                assert!(on_circle(point, &center, &radius), "{label}");
            }
            assert!(!points_equal(first_point, second_point), "{label}");
            CircleOracle::Secant
        },
        CircleCircleRelation::Uncertain { reason } => panic!("{label}: uncertain {reason:?}"),
    };
    assert_eq!(actual, expected, "{label}");
    relation
}

#[test]
fn circle_relations_follow_the_exact_discriminant() {
    let tiny = q(1, 1_000_000_000_000);
    for (center, radius, label) in [
        (p(0, 0), r(5), "coincident"),
        (p(8, 0), r(5), "secant"),
        (p(10, 0), r(5), "external tangent"),
        (p(12, 0), r(5), "disjoint"),
        (p(3, 0), r(2), "internal tangent"),
        (p(1, 0), r(2), "nested"),
        (p(0, 0), r(2), "concentric"),
        (p(3, 4), r(10), "internal tangent containing"),
        (
            Point2::new(r(10) + tiny.clone(), r(0)),
            r(5),
            "just outside tangency",
        ),
        (
            Point2::new(r(10) - tiny.clone(), r(0)),
            r(5),
            "just inside tangency",
        ),
        (p(6, 8), r(5), "tangent off axis"),
    ] {
        check_circle_pair(center, radius, label);
    }
    // The secant witnesses of the exactCore fixture are the rational (4, +-3).
    let CircleCircleRelation::Secant {
        first_point,
        second_point,
    } = check_circle_pair(p(8, 0), r(5), "secant")
    else {
        unreachable!()
    };
    let mut witnesses = [rational_point(&first_point), rational_point(&second_point)];
    witnesses.sort_by(|left, right| left[1].partial_cmp(&right[1]).unwrap());
    assert_eq!(
        witnesses,
        [
            [Rational::new(4), Rational::new(-3)],
            [Rational::new(4), Rational::new(3)]
        ]
    );
    let CircleCircleRelation::Tangent { point } = check_circle_pair(p(10, 0), r(5), "tangent")
    else {
        unreachable!()
    };
    assert_eq!(rational_point(&point), [Rational::new(5), Rational::zero()]);
}

#[test]
fn supporting_line_circle_relations_follow_the_exact_discriminant() {
    let five = r(5);
    let disk = circle(p(0, 0), &five);
    for (height, expected) in [
        (7, "disjoint"),
        (5, "tangent"),
        (0, "secant"),
        (-5, "tangent"),
    ] {
        let line = segment([-10, height, 10, height]);
        let relation = line.supporting_line_circle_relation(&disk).expect("relation");
        let actual = match &relation {
            LineCircleRelation::Disjoint => "disjoint",
            LineCircleRelation::Tangent { point, line_param } => {
                assert!(on_circle(point, &p(0, 0), &five));
                assert!(points_equal(&line.point_at(line_param.clone()), point));
                "tangent"
            },
            LineCircleRelation::Secant {
                first_point,
                first_param,
                second_point,
                second_param,
            } => {
                for (point, parameter) in
                    [(first_point, first_param), (second_point, second_param)]
                {
                    assert!(on_circle(point, &p(0, 0), &five));
                    assert!(points_equal(&line.point_at(parameter.clone()), point));
                }
                assert_eq!(strict_order(first_param, second_param), Ordering::Less);
                "secant"
            },
            LineCircleRelation::Uncertain { reason } => panic!("uncertain {reason:?}"),
        };
        assert_eq!(actual, expected, "y = {height}");
    }
}
