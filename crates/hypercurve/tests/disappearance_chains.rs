//! Features that disappear under regularized offsets, followed by further
//! operations on the result. Each expected set is authored independently and
//! compared through an exact empty symmetric difference.
mod support;

use hypercurve::{
    BooleanOp, CircularArc2, Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2,
    OffsetCornerStyle2, Point2, PredicatePolicy, Real,
};

const STRICT: PredicatePolicy = PredicatePolicy::STRICT;

fn fraction(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn line(start: &Point2, end: &Point2) -> Curve2 {
    Curve2::from(LineSeg2::try_new(start.clone(), end.clone()).unwrap())
}

fn arc(start: &Point2, end: &Point2, center: &Point2, clockwise: bool) -> Curve2 {
    Curve2::from(
        CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), clockwise)
            .unwrap(),
    )
}

/// A full circle as two half arcs, counterclockwise unless `clockwise`.
fn circle(cx: i64, cy: i64, radius: i64, clockwise: bool) -> CurvePath2 {
    let (east, west, center) = (p(cx + radius, cy), p(cx - radius, cy), p(cx, cy));
    CurvePath2::try_new(vec![
        arc(&east, &west, &center, clockwise),
        arc(&west, &east, &center, clockwise),
    ])
    .unwrap()
}

/// A closed polygon through integer vertices.
fn polygon(points: &[(i64, i64)]) -> CurvePath2 {
    CurvePath2::try_new(
        (0..points.len())
            .map(|i| {
                let (a, b) = (points[i], points[(i + 1) % points.len()]);
                line(&p(a.0, a.1), &p(b.0, b.1))
            })
            .collect(),
    )
    .unwrap()
}

/// The counterclockwise square `[-half, half]^2`.
fn square(half: i64) -> CurvePath2 {
    polygon(&[(-half, -half), (half, -half), (half, half), (-half, half)])
}

fn reversed(path: &CurvePath2) -> CurvePath2 {
    support::under(&STRICT, || path.reversed())
        .unwrap()
        .into_value()
}

/// The square `[-half, half]^2` dilated by `distance` with round corners.
fn rounded_square(half: &Real, distance: &Real) -> CurvePath2 {
    let (near, far) = (half.clone(), half + distance);
    let (nn, nf) = (-near.clone(), -far.clone());
    let q = |x: &Real, y: &Real| Point2::new(x.clone(), y.clone());
    CurvePath2::try_new(vec![
        line(&q(&nn, &nf), &q(&near, &nf)),
        arc(&q(&near, &nf), &q(&far, &nn), &q(&near, &nn), false),
        line(&q(&far, &nn), &q(&far, &near)),
        arc(&q(&far, &near), &q(&near, &far), &q(&near, &near), false),
        line(&q(&near, &far), &q(&nn, &far)),
        arc(&q(&nn, &far), &q(&nf, &near), &q(&nn, &near), false),
        line(&q(&nf, &near), &q(&nf, &nn)),
        arc(&q(&nf, &nn), &q(&nn, &nf), &q(&nn, &nn), false),
    ])
    .unwrap()
}

fn region(paths: &[CurvePath2]) -> CurveRegion2 {
    support::under(&STRICT, || {
        CurveRegion2::try_from_boundary_paths(paths, FillRule::NonZero)
    })
    .unwrap()
    .into_value()
}

fn offset_with(region: &CurveRegion2, distance: Real, style: &OffsetCornerStyle2) -> CurveRegion2 {
    support::under(&STRICT, || region.offset(distance, style))
        .unwrap()
        .into_value()
}

fn offset(region: &CurveRegion2, distance: Real) -> CurveRegion2 {
    offset_with(region, distance, &OffsetCornerStyle2::Round)
}

fn boolean(left: &CurveRegion2, right: &CurveRegion2, op: BooleanOp) -> CurveRegion2 {
    support::under(&STRICT, || left.boolean_region(right, op))
        .unwrap()
        .into_value()
}

fn assert_same_set(label: &str, actual: &CurveRegion2, expected: &CurveRegion2) {
    assert!(
        boolean(actual, expected, BooleanOp::Xor).is_empty(),
        "{label}: the regions differ"
    );
}

/// Eroding a disk past its radius, or exactly to its center, leaves the
/// canonical empty region, which then behaves as the identity or absorbing
/// element of every further operation.
#[test]
fn eroded_disk_disappears_and_its_empty_result_composes() {
    let disk = region(&[circle(0, 0, 2, false)]);
    let block = region(&[polygon(&[(5, -1), (7, -1), (7, 1), (5, 1)])]);
    for distance in [fraction(-5, 2), Real::from(-2)] {
        let gone = offset(&disk, distance);
        assert!(gone.is_empty());
        assert_same_set("union", &boolean(&gone, &block, BooleanOp::Union), &block);
        assert_same_set("union", &boolean(&block, &gone, BooleanOp::Union), &block);
        assert!(boolean(&gone, &block, BooleanOp::Intersection).is_empty());
        assert!(boolean(&block, &gone, BooleanOp::Intersection).is_empty());
        assert_same_set(
            "difference",
            &boolean(&block, &gone, BooleanOp::Difference),
            &block,
        );
        assert!(boolean(&gone, &block, BooleanOp::Difference).is_empty());
        assert_same_set("xor", &boolean(&gone, &block, BooleanOp::Xor), &block);
        assert!(offset(&gone, Real::one()).is_empty());
        assert!(offset(&gone, -Real::one()).is_empty());
        let paths = support::under(&STRICT, || gone.boundary_paths())
            .unwrap()
            .into_value();
        assert!(paths.is_empty());
        assert!(region(&paths).is_empty());
    }
}

/// Dilating an annulus by at least its hole radius fills the hole, including
/// at exactly the hole radius, where the hole's offset is one point. Eroding
/// the result by the same distance gives a disk, not the original annulus.
#[test]
fn dilation_fills_a_hole_that_erosion_does_not_restore() {
    let annulus = region(&[circle(0, 0, 3, false), circle(0, 0, 1, true)]);
    for (distance, radius) in [(1, 4), (2, 5)] {
        let filled = offset(&annulus, Real::from(distance));
        assert_same_set("filled", &filled, &region(&[circle(0, 0, radius, false)]));
        let eroded = offset(&filled, Real::from(-distance));
        assert_same_set("eroded", &eroded, &region(&[circle(0, 0, 3, false)]));
        assert!(!boolean(&eroded, &annulus, BooleanOp::Xor).is_empty());
    }
    // A smaller dilation keeps a smaller concentric hole.
    let narrowed = offset(&annulus, fraction(1, 2));
    let area = support::under(&STRICT, || narrowed.signed_area())
        .unwrap()
        .into_value()
        .unwrap();
    assert_eq!(area, Real::from(12) * Real::pi());
}

/// One component of a disconnected region vanishes under erosion while the
/// other shrinks; dilating back restores only the surviving component.
#[test]
fn erosion_removes_one_disconnected_component() {
    let pair = region(&[circle(0, 0, 1, false), circle(10, 0, 3, false)]);
    let eroded = offset(&pair, Real::from(-2));
    assert_same_set("eroded", &eroded, &region(&[circle(10, 0, 1, false)]));
    let restored = offset(&eroded, Real::from(2));
    assert_same_set("restored", &restored, &region(&[circle(10, 0, 3, false)]));
}

/// Polygonal holes close once the dilation reaches their half-width. Past
/// that distance their raw parallels cross and keep the hole's orientation;
/// the result must still have no hole.
#[test]
fn polygonal_holes_close_at_and_past_their_half_width() {
    let frame = square(8);
    let square_hole = reversed(&square(1));
    let slot_hole = reversed(&polygon(&[(-3, -1), (3, -1), (3, 1), (-3, 1)]));
    for hole in [square_hole, slot_hole] {
        let framed = region(&[frame.clone(), hole]);
        for distance in [1, 2] {
            let distance = Real::from(distance);
            let dilated = offset(&framed, distance.clone());
            let expected = region(&[rounded_square(&Real::from(8), &distance)]);
            assert_same_set("closed hole", &dilated, &expected);
        }
        let partial = offset(&framed, fraction(1, 2));
        assert_eq!(partial.boundary_loops().len(), 2);
    }
}

/// A frame whose hole survives keeps each corner convention: the hole's
/// corners stay sharp under every style, while the outer corners are round,
/// mitred or bevelled.
#[test]
fn surviving_hole_keeps_each_corner_style() {
    let framed = region(&[square(5), reversed(&square(2))]);
    let hole = reversed(&square(1));
    let round = offset_with(&framed, Real::one(), &OffsetCornerStyle2::Round);
    assert_same_set(
        "round",
        &round,
        &region(&[rounded_square(&Real::from(5), &Real::one()), hole.clone()]),
    );
    let miter = offset_with(
        &framed,
        Real::one(),
        &OffsetCornerStyle2::Miter {
            limit: Real::from(4),
        },
    );
    assert_same_set("miter", &miter, &region(&[square(6), hole.clone()]));
    let bevel = offset_with(&framed, Real::one(), &OffsetCornerStyle2::Bevel);
    let octagon = polygon(&[
        (-5, -6),
        (5, -6),
        (6, -5),
        (6, 5),
        (5, 6),
        (-5, 6),
        (-6, 5),
        (-6, -5),
    ]);
    assert_same_set("bevel", &bevel, &region(&[octagon, hole]));
}

/// A dumbbell-shaped hole: two 4x4 lobes joined by a channel of width 2.
/// Dilating by 3/2 closes the channel and shrinks each lobe to the points at
/// distance at least 3/2 from the hole boundary: `|y| <= 1/2`, bounded at the
/// channel mouth by circles of radius 3/2 about the mouth corners `(+-2, +-1)`.
/// Those circles meet `y = +-1/2` at `|x| = 2 + sqrt 2` and the axis at
/// `|x| = 2 + sqrt(5)/2`.
#[test]
fn dumbbell_hole_channel_closes_while_its_lobes_survive() {
    let dumbbell = reversed(&polygon(&[
        (-6, -2),
        (-2, -2),
        (-2, -1),
        (2, -1),
        (2, -2),
        (6, -2),
        (6, 2),
        (2, 2),
        (2, 1),
        (-2, 1),
        (-2, 2),
        (-6, 2),
    ]));
    let framed = region(&[square(10), dumbbell]);
    let dilated = offset(&framed, fraction(3, 2));

    let half = fraction(1, 2);
    let mouth = Real::from(2) + Real::from(2).sqrt().unwrap();
    let waist = Real::from(2) + (Real::from(5).sqrt().unwrap() / Real::from(2)).unwrap();
    let wall = fraction(9, 2);
    // The mirrored right lobe is traversed clockwise, as a hole; the left
    // lobe is built counterclockwise and then reversed.
    let lobe = |sign: i64| {
        let s = Real::from(sign);
        let q = |x: &Real, y: &Real| Point2::new(&s * x, y.clone());
        let (a, b, c, d, e) = (
            q(&wall, &-half.clone()),
            q(&mouth, &-half.clone()),
            q(&waist, &Real::zero()),
            q(&mouth, &half),
            q(&wall, &half),
        );
        let clockwise = sign < 0;
        let path = CurvePath2::try_new(vec![
            line(&a, &b),
            arc(&b, &c, &q(&Real::from(2), &Real::from(-1)), clockwise),
            arc(&c, &d, &q(&Real::from(2), &Real::one()), clockwise),
            line(&d, &e),
            line(&e, &a),
        ])
        .unwrap();
        if sign < 0 { reversed(&path) } else { path }
    };
    let expected = region(&[
        rounded_square(&Real::from(10), &fraction(3, 2)),
        lobe(-1),
        lobe(1),
    ]);
    assert_same_set("dumbbell", &dilated, &expected);
    assert_eq!(dilated.boundary_loops().len(), 3);
    // At distance 2 each lobe collapses exactly to its center and vanishes.
    let closed = offset(&framed, Real::from(2));
    assert_same_set(
        "closed dumbbell",
        &closed,
        &region(&[rounded_square(&Real::from(10), &Real::from(2))]),
    );
}

/// A convex hole whose short edge vanishes before the hole closes: the square
/// `[-4, 4]^2` with its corner cut by the edge from `(4, 3)` to `(3, 4)`. At
/// distance 1 the cut survives on the line `x + y = 7 - sqrt 2`; at distance 2
/// that line no longer reaches the shifted corner and the hole is the square
/// `[-2, 2]^2`.
#[test]
fn convex_hole_edge_vanishes_before_the_hole_closes() {
    let hole = reversed(&polygon(&[(-4, -4), (4, -4), (4, 3), (3, 4), (-4, 4)]));
    let framed = region(&[square(8), hole]);
    let cut = Real::from(4) - Real::from(2).sqrt().unwrap();
    let q = |x: &Real, y: &Real| Point2::new(x.clone(), y.clone());
    let (three, minus_three) = (Real::from(3), Real::from(-3));
    let shrunk = reversed(
        &CurvePath2::try_new(vec![
            line(&q(&minus_three, &minus_three), &q(&three, &minus_three)),
            line(&q(&three, &minus_three), &q(&three, &cut)),
            line(&q(&three, &cut), &q(&cut, &three)),
            line(&q(&cut, &three), &q(&minus_three, &three)),
            line(&q(&minus_three, &three), &q(&minus_three, &minus_three)),
        ])
        .unwrap(),
    );
    assert_same_set(
        "cut survives",
        &offset(&framed, Real::one()),
        &region(&[rounded_square(&Real::from(8), &Real::one()), shrunk]),
    );
    assert_same_set(
        "cut vanishes",
        &offset(&framed, Real::from(2)),
        &region(&[
            rounded_square(&Real::from(8), &Real::from(2)),
            reversed(&square(2)),
        ]),
    );
}

/// An elliptical hole, the disk of radius 1 scaled by 2 along x, in a
/// square frame. Its inward parallels are nonrational. Dilating the region by
/// at least the minor semi-axis closes the hole: at distance 1 the eroded
/// ellipse is its center point. Smaller dilations keep a hole, which must be
/// exactly the ellipse eroded by the same distance: the two are disjoint and
/// together fill the dilated frame. At distance 1/2, the smallest radius of
/// curvature, the inner parallel has cusps exactly at the half-arc ends
/// `(+-2, 0)`; distance 3/4 exceeds it, so the raw parallels have swallowtails.
#[test]
fn elliptical_hole_closes_at_its_minor_semi_axis() {
    let unit_disk = region(&[circle(0, 0, 1, false)]);
    let ellipse = support::under(&STRICT, || {
        unit_disk.transform_affine(
            &Real::from(2),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
        )
    })
    .unwrap()
    .into_value();
    let framed = boolean(&region(&[square(8)]), &ellipse, BooleanOp::Difference);
    for distance in [Real::one(), fraction(3, 2)] {
        assert_same_set(
            "closed ellipse",
            &offset(&framed, distance.clone()),
            &region(&[rounded_square(&Real::from(8), &distance)]),
        );
    }
    for distance in [fraction(1, 4), fraction(1, 2), fraction(3, 4)] {
        let dilated = offset(&framed, distance.clone());
        let hole = offset(&ellipse, -distance.clone());
        assert!(!hole.is_empty());
        assert_eq!(dilated.boundary_loops().len(), 2);
        assert!(boolean(&dilated, &hole, BooleanOp::Intersection).is_empty());
        assert_same_set(
            "surviving ellipse",
            &boolean(&dilated, &hole, BooleanOp::Union),
            &region(&[rounded_square(&Real::from(8), &distance)]),
        );
    }
}

/// Invalid offset options are reported for the canonical empty region exactly
/// as for a nonempty one, including at zero distance: an empty operand never
/// turns an invalid request into a successful empty result.
#[test]
fn invalid_offset_options_are_reported_for_empty_regions() {
    let invalid = OffsetCornerStyle2::Miter {
        limit: Real::from(-1),
    };
    let empty = CurveRegion2::empty();
    let block = region(&[square(1)]);
    for (operand, distance) in [
        (&empty, Real::one()),
        (&empty, -Real::one()),
        (&block, Real::one()),
        (&block, Real::zero()),
    ] {
        assert!(matches!(
            support::under(&STRICT, || operand.offset(distance.clone(), &invalid)),
            Err(hypercurve::ExactCurveError::Invalid { .. })
        ));
    }
    assert!(offset_with(&empty, Real::one(), &OffsetCornerStyle2::Bevel).is_empty());
}

/// Repeated erosion through a topology change. A dumbbell of two 4x4 lobes
/// joined by a width-2 neck splits when eroded by 1: each lobe becomes
/// `[-5, -3] x [-1, 1]` (mirrored) extended toward the mouth by a lens bounded
/// by unit arcs about the mouth corners `(+-2, +-1)`, which meet at a cusp on
/// the axis; the remaining neck is the segment `y = 0`, which regularization
/// removes. Erosions compose, so two steps equal one, before and after the
/// split.
#[test]
fn repeated_erosion_splits_a_dumbbell_exactly() {
    let dumbbell = region(&[polygon(&[
        (-6, -2),
        (-2, -2),
        (-2, -1),
        (2, -1),
        (2, -2),
        (6, -2),
        (6, 2),
        (2, 2),
        (2, 1),
        (-2, 1),
        (-2, 2),
        (-6, 2),
    ])]);
    let lobe = |sign: i64| {
        let (wall, mouth, tip) = (p(5 * sign, 1), p(3 * sign, 1), p(2 * sign, 0));
        let (wall_low, mouth_low) = (p(5 * sign, -1), p(3 * sign, -1));
        let (center_low, center_high) = (p(2 * sign, -1), p(2 * sign, 1));
        let path = CurvePath2::try_new(vec![
            line(&wall_low, &mouth_low),
            arc(&mouth_low, &tip, &center_low, sign < 0),
            arc(&tip, &mouth, &center_high, sign < 0),
            line(&mouth, &wall),
            line(&wall, &wall_low),
        ])
        .unwrap();
        // The left lobe is counterclockwise as written; the mirrored right
        // lobe is clockwise and is reversed into material.
        if sign < 0 { path } else { reversed(&path) }
    };
    let split = region(&[lobe(-1), lobe(1)]);
    let half = offset(&dumbbell, -fraction(1, 2));
    assert_eq!(half.boundary_loops().len(), 1);
    let twice = offset(&half, -fraction(1, 2));
    assert_eq!(twice.boundary_loops().len(), 2);
    assert_same_set("split", &twice, &split);
    assert_same_set("split", &offset(&dumbbell, -Real::one()), &split);
    let three_quarters = offset(&dumbbell, -fraction(3, 4));
    assert_same_set(
        "composed erosion",
        &offset(&three_quarters, -fraction(3, 4)),
        &offset(&dumbbell, -fraction(3, 2)),
    );
}

/// An island inside a hole. Dilating the frame shrinks the hole and grows the
/// island until they meet: at distance 1 an annular gap remains; at distance 2
/// the island's straight edges coincide with the shrunken hole's edges, leaving
/// four corner holes, each the square corner minus the island's quarter disk;
/// at distance 3 everything fills.
#[test]
fn island_meets_its_shrinking_hole() {
    let framed = region(&[square(10), reversed(&square(6)), square(2)]);
    assert_same_set(
        "gap",
        &offset(&framed, Real::one()),
        &region(&[
            rounded_square(&Real::from(10), &Real::one()),
            reversed(&square(5)),
            rounded_square(&Real::from(2), &Real::one()),
        ]),
    );
    let corner = |sx: i64, sy: i64| {
        let (a, b, c) = (p(4 * sx, 2 * sy), p(4 * sx, 4 * sy), p(2 * sx, 4 * sy));
        let even = sx * sy > 0;
        let path = CurvePath2::try_new(vec![
            line(&a, &b),
            line(&b, &c),
            arc(&c, &a, &p(2 * sx, 2 * sy), even),
        ])
        .unwrap();
        // Counterclockwise for an even number of reflections; holes are clockwise.
        if even { reversed(&path) } else { path }
    };
    let touching = offset(&framed, Real::from(2));
    assert_eq!(touching.boundary_loops().len(), 5);
    assert_same_set(
        "corner holes",
        &touching,
        &region(&[
            rounded_square(&Real::from(10), &Real::from(2)),
            corner(1, 1),
            corner(-1, 1),
            corner(-1, -1),
            corner(1, -1),
        ]),
    );
    assert_same_set(
        "filled",
        &offset(&framed, Real::from(3)),
        &region(&[rounded_square(&Real::from(10), &Real::from(3))]),
    );
}
