use hypercurve::{
    BooleanOp, Classification, Curve2, CurvePath2, CurvePolicy, CurveRegion2, LineSeg2, Point2,
    Real, RegionPointLocation,
};
#[cfg(feature = "predicates")]
use hypercurve::{
    CircularArc2, CubicBezier2, CurveBoundaryInteriorSide2, QuadraticBezier2,
    RationalQuadraticBezier2,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[cfg(feature = "predicates")]
fn exact_f64_point(x: f64, y: f64) -> Point2 {
    Point2::new(
        Real::try_from(x).expect("finite binary rational x"),
        Real::try_from(y).expect("finite binary rational y"),
    )
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
        .retain_boolean(&second, &CurvePolicy::certified())
        .unwrap();
    assert_eq!(prepared.authored_carrier_pair_count(), 16);
    assert!(prepared.carrier_pair_count() < prepared.authored_carrier_pair_count());
    assert!(!prepared.is_boolean_region_cached(BooleanOp::Union));
    assert!(!prepared.is_intersection_result_cached());

    let contacts = prepared.intersection_result().unwrap();
    assert!(prepared.is_intersection_result_cached());
    assert!(contacts.is_complete());
    assert!(!contacts.is_disjoint());
    assert_eq!(
        contacts.authored_carrier_pair_count(),
        prepared.authored_carrier_pair_count()
    );
    assert_eq!(
        contacts.candidate_carrier_pair_count(),
        prepared.carrier_pair_count()
    );
    assert!(!contacts.contacts().is_empty());
    assert!(!contacts.overlaps().is_empty());
    assert!(contacts.blockers().is_empty());
    assert!(contacts.contacts().iter().all(|contact| {
        contact.first().operand() == hypercurve::CurvePathBooleanOperand2::First
            && contact.second().operand() == hypercurve::CurvePathBooleanOperand2::Second
            && contact.first().loop_index() == 0
            && contact.second().loop_index() == 0
    }));

    let direct_contacts = first
        .intersect_region(&second, &CurvePolicy::certified())
        .unwrap();
    assert_eq!(direct_contacts.contacts().len(), contacts.contacts().len());
    assert_eq!(direct_contacts.overlaps().len(), contacts.overlaps().len());

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
    let prepared = algebraic.retain_boolean(&crossing, &policy).unwrap();
    assert!(!prepared.is_boolean_topology_cached());
    let crossed = prepared.boolean_region(BooleanOp::Union).unwrap();
    assert!(prepared.is_boolean_topology_cached());
    prepared.boolean_region(BooleanOp::Difference).unwrap();
    assert!(prepared.is_boolean_topology_cached());
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
    let prepared = narrow.retain_boolean(&wide, &policy).unwrap();
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

#[test]
#[cfg(feature = "predicates")]
fn shared_demo_algebraic_polyline_blocker_resolves_all_boolean_modes() {
    let p0 = exact_f64_point(-18.6, -6.2);
    let p1 = exact_f64_point(-14.20650382593274, -19.07368476688862);
    let p2 = exact_f64_point(-6.4565038147568705, -21.17386977761984);
    let p3 = exact_f64_point(4.03, -4.03);
    let p4 = exact_f64_point(14.600491094738247, -20.78282692939043);
    let p5 = exact_f64_point(20.150000000000002, 6.820000000000001);
    let p6 = exact_f64_point(7.4399999999999995, 10.85);
    let p7 = exact_f64_point(-16.43, 7.13);
    let first_path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p0.clone(), p1.clone()).unwrap()),
        Curve2::from(
            CircularArc2::from_bulge(
                p1,
                p2.clone(),
                Real::try_from(0.46).expect("finite binary rational bulge"),
            )
            .unwrap(),
        ),
        Curve2::from(QuadraticBezier2::new(
            p2,
            exact_f64_point(-0.31000000000000005, 7.13),
            p3.clone(),
        )),
        Curve2::from(CubicBezier2::new(
            p3,
            exact_f64_point(7.184295191466809, -46.13835323035717),
            exact_f64_point(10.85, 5.89),
            p4.clone(),
        )),
        Curve2::from(
            RationalQuadraticBezier2::try_new(
                p4,
                exact_f64_point(18.91, 6.2),
                p5.clone(),
                Real::one(),
                Real::try_from(0.36).expect("finite binary rational weight"),
                Real::one(),
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(p5, p6.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(p6, p7.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(p7, p0).unwrap()),
    ])
    .unwrap();

    let q0 = exact_f64_point(-24.8, -18.6);
    let q1 = exact_f64_point(24.8, -18.6);
    let q2 = exact_f64_point(24.8, 8.06);
    let q3 = exact_f64_point(-24.8, 8.06);
    let second_path = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            q0.clone(),
            exact_f64_point(-12.4, -20.77),
            exact_f64_point(12.4, -20.77),
            q1.clone(),
        )),
        Curve2::from(LineSeg2::try_new(q1, q2.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(q2, q3.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(q3, q0).unwrap()),
    ])
    .unwrap();

    let first = CurveRegion2::try_from_boundary_paths(&[first_path]).unwrap();
    let second = CurveRegion2::try_from_boundary_paths(&[second_path]).unwrap();
    let policy = CurvePolicy::certified();
    let prepared = first.retain_boolean(&second, &policy).unwrap();
    let intersections = prepared.intersection_result().unwrap();
    assert!(
        intersections.blockers().is_empty(),
        "{:#?}",
        intersections.blockers()
    );

    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let result = prepared
            .boolean_region(operation)
            .unwrap_or_else(|error| panic!("{operation:?} remained blocked: {error}"));
        assert!(matches!(
            result.project_to_finite_curve_paths(&policy).unwrap(),
            Classification::Decided(_)
        ));
    }
}
