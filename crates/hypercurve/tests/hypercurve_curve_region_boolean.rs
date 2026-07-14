use hypercurve::{
    BooleanOp, Classification, Curve2, CurveBoundaryInteriorSide2, CurvePath2, CurvePolicy,
    CurveRegion2, LineSeg2, Point2, QuadraticBezier2, Real, RegionPointLocation,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn square_path(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> CurvePath2 {
    let points = [
        point(min_x, min_y),
        point(max_x, min_y),
        point(max_x, max_y),
        point(min_x, max_y),
    ];
    let curves = (0..points.len())
        .map(|index| {
            Curve2::from(
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .unwrap(),
            )
        })
        .collect();
    CurvePath2::try_new(curves).unwrap()
}

fn square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths(&[square_path(min_x, min_y, max_x, max_y)]).unwrap()
}

fn assert_location(region: &CurveRegion2, point: Point2, expected: RegionPointLocation) {
    assert_eq!(
        region
            .classify_point(&point, &CurvePolicy::certified())
            .unwrap(),
        Classification::Decided(expected)
    );
}

#[test]
fn curved_regions_boolean_and_reuse_prepared_pair() {
    let first = square(0, 0, 4, 4);
    let second = square(2, 0, 6, 4);
    let prepared = first
        .try_prepare_boolean(&second, &CurvePolicy::certified())
        .unwrap();
    assert_eq!(prepared.authored_carrier_pair_count(), 16);
    assert!(prepared.carrier_pair_count() < prepared.authored_carrier_pair_count());
    assert!(!prepared.is_boolean_region_cached(BooleanOp::Union));

    let union = prepared.boolean_region(BooleanOp::Union).unwrap();
    assert!(prepared.is_boolean_region_cached(BooleanOp::Union));
    assert_location(&union, point(1, 2), RegionPointLocation::Inside);
    assert_location(&union, point(3, 2), RegionPointLocation::Inside);
    assert_location(&union, point(5, 2), RegionPointLocation::Inside);

    let intersection = prepared.boolean_region(BooleanOp::Intersection).unwrap();
    assert_location(&intersection, point(1, 2), RegionPointLocation::Outside);
    assert_location(&intersection, point(3, 2), RegionPointLocation::Inside);

    let difference = prepared.boolean_region(BooleanOp::Difference).unwrap();
    assert_location(&difference, point(1, 2), RegionPointLocation::Inside);
    assert_location(&difference, point(3, 2), RegionPointLocation::Outside);

    let xor = prepared.boolean_region(BooleanOp::Xor).unwrap();
    assert_location(&xor, point(1, 2), RegionPointLocation::Inside);
    assert_location(&xor, point(3, 2), RegionPointLocation::Outside);
    assert_location(&xor, point(5, 2), RegionPointLocation::Inside);
}

#[test]
fn curved_region_boolean_output_can_feed_another_boolean() {
    let first = square(0, 0, 4, 4);
    let second = square(2, 0, 6, 4);
    let third = square(4, 0, 8, 4);
    let policy = CurvePolicy::certified();

    let first_union = first
        .boolean_region(&second, BooleanOp::Union, &policy)
        .unwrap();
    let chained = first_union
        .boolean_region(&third, BooleanOp::Union, &policy)
        .unwrap();

    for x in [1, 3, 5, 7] {
        assert_location(&chained, point(x, 2), RegionPointLocation::Inside);
    }
    assert_location(&chained, point(9, 2), RegionPointLocation::Outside);
}

#[test]
fn curved_region_boolean_respects_nested_hole_roles() {
    let ring = CurveRegion2::try_from_boundary_paths(&[
        square_path(0, 0, 10, 10),
        square_path(2, 2, 8, 8),
    ])
    .unwrap();
    let island = square(4, 4, 6, 6);
    let policy = CurvePolicy::certified();

    let union = ring
        .boolean_region(&island, BooleanOp::Union, &policy)
        .unwrap();
    assert_location(&union, point(1, 1), RegionPointLocation::Inside);
    assert_location(&union, point(3, 3), RegionPointLocation::Outside);
    assert_location(&union, point(5, 5), RegionPointLocation::Inside);

    let intersection = ring
        .boolean_region(&island, BooleanOp::Intersection, &policy)
        .unwrap();
    assert!(intersection.is_empty());
}

#[test]
#[cfg(feature = "predicates")]
fn algebraic_curved_region_output_can_feed_another_boolean() {
    let curved = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            point(-2, 4),
            point(0, -4),
            point(2, 4),
        )),
        Curve2::from(LineSeg2::try_new(point(2, 4), point(-2, 4)).unwrap()),
    ])
    .unwrap();
    let cutter_path = square_path(-3, 2, 3, 5);
    let policy = CurvePolicy::certified();
    let algebraic = curved
        .boolean_region(
            &cutter_path,
            BooleanOp::Difference,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap();
    assert!(algebraic.has_algebraic_fragments());

    let disjoint = square(10, 0, 12, 2);
    let chained = algebraic
        .boolean_region(&disjoint, BooleanOp::Union, &policy)
        .unwrap();
    assert!(chained.has_algebraic_fragments());
    assert_location(&chained, point(0, 1), RegionPointLocation::Inside);
    assert_location(&chained, point(11, 1), RegionPointLocation::Inside);

    let crossing = square(-2, -1, 2, 1);
    let crossed = algebraic
        .boolean_region(&crossing, BooleanOp::Union, &policy)
        .unwrap();
    assert!(crossed.has_algebraic_fragments());
    assert_location(&crossed, point(0, 0), RegionPointLocation::Inside);
    assert_location(&crossed, point(0, 1), RegionPointLocation::Inside);

    assert_eq!(
        algebraic
            .boolean_region(&algebraic, BooleanOp::Union, &policy)
            .unwrap(),
        algebraic
    );
    assert!(
        algebraic
            .boolean_region(&algebraic, BooleanOp::Xor, &policy)
            .unwrap()
            .is_empty()
    );
}

#[test]
#[cfg(feature = "predicates")]
fn retained_regions_clip_shared_source_components_to_carrier_ranges() {
    let curved = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            point(-2, 4),
            point(0, -4),
            point(2, 4),
        )),
        Curve2::from(LineSeg2::try_new(point(2, 4), point(-2, 4)).unwrap()),
    ])
    .unwrap();
    let policy = CurvePolicy::certified();
    let narrow = curved
        .boolean_region(
            &square_path(-3, -1, 3, 2),
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap();
    let wide = curved
        .boolean_region(
            &square_path(-3, -1, 3, 3),
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap();
    assert!(narrow.has_algebraic_fragments());
    assert!(wide.has_algebraic_fragments());
    let prepared = narrow.try_prepare_boolean(&wide, &policy).unwrap();
    let union = prepared.boolean_region(BooleanOp::Union).unwrap();
    assert_location(&union, point(0, 1), RegionPointLocation::Inside);
    assert_location(&union, point(0, 3), RegionPointLocation::Boundary);
    assert_location(&union, point(0, 4), RegionPointLocation::Outside);

    let intersection = prepared.boolean_region(BooleanOp::Intersection).unwrap();
    assert_location(&intersection, point(0, 1), RegionPointLocation::Inside);
    assert_location(&intersection, point(0, 3), RegionPointLocation::Outside);

    assert!(
        prepared
            .boolean_region(BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );

    let xor = prepared.boolean_region(BooleanOp::Xor).unwrap();
    let between_tops = Point2::new(Real::zero(), (Real::from(5_i8) / Real::from(2_i8)).unwrap());
    assert_location(&xor, point(0, 1), RegionPointLocation::Outside);
    assert_location(&xor, between_tops, RegionPointLocation::Inside);
}
