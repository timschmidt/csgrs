use hypercurve::{
    BooleanOp, Classification, Curve2, CurveCertainty, CurveContext, CurvePath2, CurveRegion2,
    LineSeg2, Point2, Real, RegionPointLocation,
};
#[cfg(feature = "predicates")]
use hypercurve::{CurveBoundaryInteriorSide2, QuadraticBezier2};

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
    CurveRegion2::try_from_boundary_paths(
        &[square_path(min_x, min_y, max_x, max_y)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

#[cfg(feature = "predicates")]
fn symbolic_rectangle(width: Real) -> CurveRegion2 {
    let points = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(width.clone(), Real::zero()),
        Point2::new(width, Real::one()),
        Point2::new(Real::zero(), Real::one()),
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
    CurveRegion2::try_from_boundary_paths(
        &[CurvePath2::try_new(curves).unwrap()],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

fn assert_location(region: &CurveRegion2, point: Point2, expected: RegionPointLocation) {
    assert_eq!(
        region
            .classify_point(&point, &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(expected)
    );
}

#[test]
fn curved_regions_immediate_batch_reuses_pair_topology() {
    let first = square(0, 0, 4, 4);
    let second = square(2, 0, 6, 4);
    let policy = CurveContext::STRICT;
    let contacts = first.intersect_region(&second, &policy).unwrap();
    assert_eq!(contacts.certainty, CurveCertainty::Certified);
    let contacts = contacts.value;
    assert!(contacts.is_complete());
    assert!(!contacts.is_disjoint());
    assert!(!contacts.contacts().is_empty());
    assert!(!contacts.overlaps().is_empty());
    assert!(contacts.blockers().is_empty());
    assert!(contacts.contacts().iter().all(|contact| {
        contact.first().operand() == hypercurve::CurvePathBooleanOperand2::First
            && contact.second().operand() == hypercurve::CurvePathBooleanOperand2::Second
            && contact.first().loop_index() == 0
            && contact.second().loop_index() == 0
    }));

    let results = first.boolean_regions(&second, &policy).unwrap();
    assert_eq!(results.certainty, CurveCertainty::Certified);
    let results = results.value;
    assert_eq!(results.authored_carrier_pair_count(), 16);
    assert!(results.candidate_carrier_pair_count() < results.authored_carrier_pair_count());
    assert_eq!(
        contacts.authored_carrier_pair_count(),
        results.authored_carrier_pair_count()
    );
    assert_eq!(results.candidate_carrier_pair_count(), 0);
    assert_eq!(results.topology_fragment_count(), 0);
    assert_eq!(results.topology_point_classification_count(), 0);
    let union = results.union();
    assert_location(union, point(1, 2), RegionPointLocation::Inside);
    assert_location(union, point(3, 2), RegionPointLocation::Inside);
    assert_location(union, point(5, 2), RegionPointLocation::Inside);

    let intersection = results.intersection();
    assert_location(intersection, point(1, 2), RegionPointLocation::Outside);
    assert_location(intersection, point(3, 2), RegionPointLocation::Inside);

    let difference = results.difference();
    assert_location(difference, point(1, 2), RegionPointLocation::Inside);
    assert_location(difference, point(3, 2), RegionPointLocation::Outside);

    let xor = results.xor();
    assert_location(xor, point(1, 2), RegionPointLocation::Inside);
    assert_location(xor, point(3, 2), RegionPointLocation::Outside);
    assert_location(xor, point(5, 2), RegionPointLocation::Inside);
}

#[test]
fn curved_region_boolean_output_can_feed_another_boolean() {
    let first = square(0, 0, 4, 4);
    let second = square(2, 0, 6, 4);
    let third = square(4, 0, 8, 4);
    let policy = CurveContext::STRICT;

    let first_union = first
        .boolean_region(&second, BooleanOp::Union, &policy)
        .unwrap()
        .value;
    let chained = first_union
        .boolean_region(&third, BooleanOp::Union, &policy)
        .unwrap()
        .value;

    for x in [1, 3, 5, 7] {
        assert_location(&chained, point(x, 2), RegionPointLocation::Inside);
    }
    assert_location(&chained, point(9, 2), RegionPointLocation::Outside);
}

#[test]
#[cfg(feature = "predicates")]
fn approximate_policy_reports_a_consumed_terminal_instead_of_relabeling_it_exact() {
    let first = symbolic_rectangle(Real::pi() + Real::e());
    let second = symbolic_rectangle(Real::e() + Real::pi());

    assert!(matches!(
        first.boolean_region(&second, BooleanOp::Union, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let outcome = first
        .boolean_region(&second, BooleanOp::Union, &CurveContext::APPROXIMATE_512)
        .expect("the authorized 512-bit terminal should complete equal symbolic boundaries");
    assert_eq!(outcome.certainty, CurveCertainty::Approximate512Consumed);
    assert_location(
        &outcome.value,
        Point2::new(Real::one(), (Real::one() / Real::from(2_u8)).unwrap()),
        RegionPointLocation::Inside,
    );
}

#[test]
#[cfg(feature = "predicates")]
fn approximate_offset_reports_a_consumed_terminal_for_symbolic_zero_distance() {
    let source = square(0, 0, 4, 4);
    let distance = (Real::pi() + Real::e()) - (Real::e() + Real::pi());

    assert!(matches!(
        source.offset(distance.clone(), &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let outcome = source
        .offset(distance, &CurveContext::APPROXIMATE_512)
        .expect("the authorized 512-bit terminal should decide symbolic zero offset");
    assert_eq!(outcome.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(outcome.value, source);
}

#[test]
fn curved_region_boolean_respects_nested_hole_roles() {
    let ring = CurveRegion2::try_from_boundary_paths(
        &[square_path(0, 0, 10, 10), square_path(2, 2, 8, 8)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let island = square(4, 4, 6, 6);
    let policy = CurveContext::STRICT;

    let union = ring
        .boolean_region(&island, BooleanOp::Union, &policy)
        .unwrap()
        .value;
    assert_location(&union, point(1, 1), RegionPointLocation::Inside);
    assert_location(&union, point(3, 3), RegionPointLocation::Outside);
    assert_location(&union, point(5, 5), RegionPointLocation::Inside);

    let intersection = ring
        .boolean_region(&island, BooleanOp::Intersection, &policy)
        .unwrap()
        .value;
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
    let policy = CurveContext::STRICT;
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
        .unwrap()
        .value;
    assert!(chained.has_algebraic_fragments());
    assert_location(&chained, point(0, 1), RegionPointLocation::Inside);
    assert_location(&chained, point(11, 1), RegionPointLocation::Inside);

    let crossing = square(-2, -1, 2, 1);
    let results = algebraic.boolean_regions(&crossing, &policy).unwrap().value;
    let crossed = results.union();
    assert!(results.topology_fragment_count() > 0);
    assert!(
        results.topology_point_classification_count() < results.topology_fragment_count(),
        "the immediate batch should share classified topology across operations"
    );
    assert!(crossed.has_algebraic_fragments());
    assert_location(crossed, point(0, 0), RegionPointLocation::Inside);
    assert_location(crossed, point(0, 1), RegionPointLocation::Inside);

    assert_eq!(
        algebraic
            .boolean_region(&algebraic, BooleanOp::Union, &policy)
            .unwrap()
            .value,
        algebraic
    );
    assert!(
        algebraic
            .boolean_region(&algebraic, BooleanOp::Xor, &policy)
            .unwrap()
            .value
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
    let policy = CurveContext::STRICT;
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
    let results = narrow.boolean_regions(&wide, &policy).unwrap().value;
    let union = results.union();
    assert_location(union, point(0, 1), RegionPointLocation::Inside);
    assert_location(union, point(0, 3), RegionPointLocation::Boundary);
    assert_location(union, point(0, 4), RegionPointLocation::Outside);

    let intersection = results.intersection();
    assert_location(intersection, point(0, 1), RegionPointLocation::Inside);
    assert_location(intersection, point(0, 3), RegionPointLocation::Outside);

    assert!(results.difference().is_empty());

    let xor = results.xor();
    let between_tops = Point2::new(Real::zero(), (Real::from(5_i8) / Real::from(2_i8)).unwrap());
    assert_location(xor, point(0, 1), RegionPointLocation::Outside);
    assert_location(xor, between_tops, RegionPointLocation::Inside);
}
