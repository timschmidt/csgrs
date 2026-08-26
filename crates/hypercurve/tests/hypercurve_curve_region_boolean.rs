mod support;

use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, Curve2, CurveCertainty, CurveContext,
    CurvePath2, CurveRegion2, LineSeg2, Point2, Real, RegionPointLocation, Segment2,
};
use hypercurve::{
    CircularArc2, CubicBezier2, CurveBoundaryInteriorSide2, CurveRegionLoopRole, FillRule,
    OffsetCornerStyle2, QuadraticBezier2, RationalBezier2, RationalBezierIntersectionContacts2,
    UncertaintyReason,
};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn sharp_offset() -> OffsetCornerStyle2 {
    OffsetCornerStyle2::Miter {
        limit: Real::from(1_000),
    }
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

fn path_region(
    path: &CurvePath2,
    interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        std::slice::from_ref(path),
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &[interior_side],
        policy,
    )
    .unwrap()
    .into_value()
}

fn boolean_paths(
    first: &CurvePath2,
    second: &CurvePath2,
    operation: BooleanOp,
    first_interior_side: CurveBoundaryInteriorSide2,
    second_interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    path_region(first, first_interior_side, policy)
        .boolean_region(
            &path_region(second, second_interior_side, policy),
            operation,
            policy,
        )
        .unwrap()
        .into_value()
}

fn square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths(
        &[square_path(min_x, min_y, max_x, max_y)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value()
}

fn circle(center_x: Real) -> CurveRegion2 {
    circle_with_policy(center_x, &CurveContext::STRICT)
}

fn integer_circle(center_x: i64, radius: i64) -> CurveRegion2 {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(center_x - radius, 0), Real::one()),
        BulgeVertex2::new(point(center_x + radius, 0), Real::one()),
    ])
    .unwrap();
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurveContext::STRICT)
        .unwrap()
        .into_value()
}

fn circle_with_policy(center_x: Real, policy: &CurveContext) -> CurveRegion2 {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(
            Point2::new(&center_x - Real::from(2_i8), Real::zero()),
            Real::one(),
        ),
        BulgeVertex2::new(
            Point2::new(center_x + Real::from(2_i8), Real::zero()),
            Real::one(),
        ),
    ])
    .unwrap();
    CurveRegion2::try_from_native_material_contours(vec![contour], policy)
        .unwrap()
        .into_value()
}

fn capsule(center_x: i64) -> CurveRegion2 {
    capsule_at(center_x, 0)
}

fn capsule_at(center_x: i64, center_y: i64) -> CurveRegion2 {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(center_x - 3, center_y - 2), Real::zero()),
        BulgeVertex2::new(point(center_x + 3, center_y - 2), Real::one()),
        BulgeVertex2::new(point(center_x + 3, center_y + 2), Real::zero()),
        BulgeVertex2::new(point(center_x - 3, center_y + 2), Real::one()),
    ])
    .unwrap();
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurveContext::STRICT)
        .unwrap()
        .into_value()
}

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

fn symbolic_quadratic_cap(control_y: Real, policy: &CurveContext) -> CurveRegion2 {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            point(-2, 4),
            Point2::new(Real::zero(), control_y),
            point(2, 4),
        )),
        Curve2::from(LineSeg2::try_new(point(2, 4), point(-2, 4)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
        policy,
    )
    .unwrap()
    .into_value()
}

fn symbolic_general_line_region(control_y: Real, policy: &CurveContext) -> CurveRegion2 {
    let bottom = RationalBezier2::try_new(
        vec![
            point(0, 0),
            Point2::new((Real::from(4_i8) / Real::from(3_i8)).unwrap(), control_y),
            Point2::new((Real::from(8_i8) / Real::from(3_i8)).unwrap(), Real::zero()),
            point(4, 0),
        ],
        vec![Real::one(); 4],
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(bottom),
        Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 4), point(0, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
        policy,
    )
    .unwrap()
    .into_value()
}

fn symbolic_elevated_circle(center_x: Real, policy: &CurveContext) -> CurveRegion2 {
    let left = Point2::new(&center_x - Real::from(2_i8), Real::zero());
    let right = Point2::new(center_x + Real::from(2_i8), Real::zero());
    let arcs = [
        CircularArc2::from_bulge(left.clone(), right.clone(), Real::one()).unwrap(),
        CircularArc2::from_bulge(right, left, Real::one()).unwrap(),
    ];
    let mut curves = Vec::with_capacity(4);
    for arc in &arcs {
        let decomposition = arc
            .rational_bezier_decomposition(policy)
            .unwrap()
            .into_value();
        for span in decomposition.spans() {
            let general = RationalBezier2::from(span.curve().clone())
                .elevated_to_degree(3)
                .unwrap();
            curves.push(Curve2::from(general));
        }
    }
    CurveRegion2::try_from_boundary_paths_with_loop_topology(
        &[CurvePath2::try_new_with_policy(curves, policy)
            .unwrap()
            .into_value()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &[CurveBoundaryInteriorSide2::Left],
        policy,
    )
    .unwrap()
    .into_value()
}

fn assert_location(region: &CurveRegion2, point: Point2, expected: RegionPointLocation) {
    assert_eq!(
        region
            .classify_point(&point, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        Classification::Decided(expected)
    );
}

fn native_segment_counts(region: &CurveRegion2, policy: &CurveContext) -> (usize, usize) {
    let native = region
        .native_contours_fast_path(policy)
        .expect("native publication must not fail")
        .into_value();
    let Classification::Decided(native) = native else {
        panic!("certified line/circular Boolean output must publish native contours");
    };
    native
        .material_contours()
        .iter()
        .chain(native.hole_contours())
        .flat_map(Contour2::segments)
        .fold((0, 0), |(lines, arcs), segment| match segment {
            Segment2::Line(_) => (lines + 1, arcs),
            Segment2::Arc(_) => (lines, arcs + 1),
        })
}

#[test]
fn boolean_batch_short_circuits_empty_and_identical_operands() {
    let empty = CurveRegion2::empty();
    let region = square(0, 0, 4, 4);
    let policy = CurveContext::STRICT;

    let empty_first = empty
        .boolean_regions(&region, &policy)
        .unwrap()
        .into_value();
    assert_eq!(empty_first.union(), &region);
    assert!(empty_first.intersection().is_empty());
    assert!(empty_first.difference().is_empty());
    assert_eq!(empty_first.xor(), &region);
    assert_eq!(empty_first.candidate_carrier_pair_count(), 0);
    assert_eq!(empty_first.topology_fragment_count(), 0);

    let identical = region
        .boolean_regions(&region, &policy)
        .unwrap()
        .into_value();
    assert_eq!(identical.union(), &region);
    assert_eq!(identical.intersection(), &region);
    assert!(identical.difference().is_empty());
    assert!(identical.xor().is_empty());
    assert_eq!(identical.candidate_carrier_pair_count(), 0);
    assert_eq!(identical.topology_fragment_count(), 0);
}

#[test]
fn affine_line_batch_reuses_the_authoritative_arrangement_topology() {
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
        contact.first().operand() == hypercurve::CurveRegionBooleanOperand2::First
            && contact.second().operand() == hypercurve::CurveRegionBooleanOperand2::Second
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
    assert!(results.candidate_carrier_pair_count() > 0);
    assert!(results.topology_fragment_count() > 0);
    assert!(
        results.topology_point_classification_count() < results.topology_fragment_count(),
        "all four operations must share propagated fragment classifications"
    );
    let union = results.union();
    assert_eq!(union.boundary_loops()[0].len(), 4);
    assert_location(union, point(1, 2), RegionPointLocation::Inside);
    assert_location(union, point(3, 2), RegionPointLocation::Inside);
    assert_location(union, point(5, 2), RegionPointLocation::Inside);

    let intersection = results.intersection();
    assert_eq!(intersection.boundary_loops()[0].len(), 4);
    assert_location(intersection, point(1, 2), RegionPointLocation::Outside);
    assert_location(intersection, point(3, 2), RegionPointLocation::Inside);

    let difference = results.difference();
    assert_eq!(difference.boundary_loops()[0].len(), 4);
    assert_location(difference, point(1, 2), RegionPointLocation::Inside);
    assert_location(difference, point(3, 2), RegionPointLocation::Outside);

    let xor = results.xor();
    assert_eq!(
        xor.boundary_loops()
            .iter()
            .map(|loop_| loop_.len())
            .sum::<usize>(),
        8
    );
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
fn affine_line_batch_preserves_material_and_hole_roles() {
    let policy = CurveContext::STRICT;
    let frame = square(0, 0, 10, 10)
        .boolean_region(&square(3, 3, 7, 7), BooleanOp::Difference, &policy)
        .unwrap()
        .into_value();
    let inset = square(1, 1, 9, 9);
    let results = frame.boolean_regions(&inset, &policy).unwrap().into_value();
    let intersection = results.intersection();

    assert_eq!(intersection.boundary_loops().len(), 2);
    assert_location(intersection, point(2, 2), RegionPointLocation::Inside);
    assert_location(intersection, point(5, 5), RegionPointLocation::Outside);
    assert_location(intersection, point(0, 0), RegionPointLocation::Outside);
}

#[test]
fn regularized_affine_contacts_discard_lower_dimensional_intersections() {
    let point_touching = (square(0, 0, 2, 2), square(2, 2, 4, 4));
    let edge_touching = (square(0, 0, 2, 2), square(2, 0, 4, 2));

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (label, (first, second), second_interior, expected_union_loops) in [
            ("point", &point_touching, point(3, 3), 2_usize),
            ("edge", &edge_touching, point(3, 1), 1_usize),
        ] {
            let evidence = first.intersect_region(second, &policy).unwrap();
            assert_eq!(evidence.certainty, CurveCertainty::Certified, "{label}");
            assert!(evidence.value.is_complete(), "{label}");
            assert!(!evidence.value.contacts().is_empty(), "{label}");
            if label == "point" {
                assert!(evidence.value.overlaps().is_empty(), "{label}");
            } else {
                assert!(!evidence.value.overlaps().is_empty(), "{label}");
            }

            let results = first.boolean_regions(second, &policy).unwrap();
            assert_eq!(results.certainty, CurveCertainty::Certified, "{label}");
            let results = results.into_value();
            assert!(results.intersection().is_empty(), "{label}");
            assert_eq!(
                results.union().boundary_loops().len(),
                expected_union_loops,
                "{label} union"
            );
            assert_eq!(
                results.difference().boundary_loops().len(),
                1,
                "{label} difference"
            );
            assert_location(
                results.difference(),
                point(1, 1),
                RegionPointLocation::Inside,
            );
            assert_location(
                results.difference(),
                second_interior.clone(),
                RegionPointLocation::Outside,
            );
            assert_eq!(
                results.xor().boundary_loops().len(),
                expected_union_loops,
                "{label} xor"
            );
            for (operation, result) in [("union", results.union()), ("xor", results.xor())] {
                for sample in [point(1, 1), second_interior.clone()] {
                    assert_eq!(
                        result
                            .classify_point(&sample, &policy)
                            .unwrap()
                            .into_value(),
                        Classification::Decided(RegionPointLocation::Inside),
                        "{label} {operation} result at {sample:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn regularized_conic_tangencies_preserve_regions_but_not_point_intersections() {
    let external = (integer_circle(0, 2), integer_circle(4, 2));
    let internal = (integer_circle(0, 3), integer_circle(2, 1));

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let external_evidence = external.0.intersect_region(&external.1, &policy).unwrap();
        assert_eq!(external_evidence.certainty, CurveCertainty::Certified);
        assert!(external_evidence.value.is_complete());
        assert!(!external_evidence.value.contacts().is_empty());
        assert!(external_evidence.value.overlaps().is_empty());
        let external_results = external.0.boolean_regions(&external.1, &policy).unwrap();
        assert_eq!(external_results.certainty, CurveCertainty::Certified);
        let external_results = external_results.into_value();
        assert!(external_results.intersection().is_empty());
        assert_eq!(external_results.union().boundary_loops().len(), 2);
        assert_eq!(external_results.difference().boundary_loops().len(), 1);
        assert_eq!(external_results.xor().boundary_loops().len(), 2);
        for result in [external_results.union(), external_results.xor()] {
            assert_location(result, point(0, 0), RegionPointLocation::Inside);
            assert_location(result, point(4, 0), RegionPointLocation::Inside);
        }

        let internal_evidence = internal.0.intersect_region(&internal.1, &policy).unwrap();
        assert_eq!(internal_evidence.certainty, CurveCertainty::Certified);
        assert!(internal_evidence.value.is_complete());
        assert!(!internal_evidence.value.contacts().is_empty());
        assert!(internal_evidence.value.overlaps().is_empty());
        let internal_results = internal.0.boolean_regions(&internal.1, &policy).unwrap();
        assert_eq!(internal_results.certainty, CurveCertainty::Certified);
        let internal_results = internal_results.into_value();
        assert_eq!(internal_results.union().boundary_loops().len(), 1);
        assert_eq!(internal_results.intersection().boundary_loops().len(), 1);
        assert_eq!(internal_results.difference().boundary_loops().len(), 2);
        assert_eq!(internal_results.xor().boundary_loops().len(), 2);
        assert_location(
            internal_results.union(),
            point(0, 0),
            RegionPointLocation::Inside,
        );
        assert_location(
            internal_results.intersection(),
            point(2, 0),
            RegionPointLocation::Inside,
        );
        for result in [internal_results.difference(), internal_results.xor()] {
            assert_location(result, point(-2, 0), RegionPointLocation::Inside);
            assert_location(result, point(2, 0), RegionPointLocation::Outside);
        }
    }
}

#[test]
fn regularized_partial_shared_edges_resolve_exact_side_ownership() {
    let attached = (square(0, 0, 4, 4), square(4, 1, 6, 3));
    let boundary_contained = (square(0, 0, 4, 4), square(1, 0, 3, 2));

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let attached_results = attached.0.boolean_regions(&attached.1, &policy).unwrap();
        assert_eq!(attached_results.certainty, CurveCertainty::Certified);
        let attached_results = attached_results.into_value();
        assert!(attached_results.intersection().is_empty());
        assert_eq!(attached_results.union().boundary_loops().len(), 1);
        assert_eq!(attached_results.difference().boundary_loops().len(), 1);
        assert_eq!(attached_results.xor().boundary_loops().len(), 1);
        for result in [attached_results.union(), attached_results.xor()] {
            assert_location(result, point(2, 2), RegionPointLocation::Inside);
            assert_location(result, point(5, 2), RegionPointLocation::Inside);
            assert_location(result, point(4, 2), RegionPointLocation::Inside);
        }

        let contained_results = boundary_contained
            .0
            .boolean_regions(&boundary_contained.1, &policy)
            .unwrap();
        assert_eq!(contained_results.certainty, CurveCertainty::Certified);
        let contained_results = contained_results.into_value();
        assert_eq!(contained_results.union().boundary_loops().len(), 1);
        assert_eq!(contained_results.intersection().boundary_loops().len(), 1);
        assert_eq!(contained_results.difference().boundary_loops().len(), 1);
        assert_eq!(contained_results.xor().boundary_loops().len(), 1);
        assert_location(
            contained_results.union(),
            point(2, 1),
            RegionPointLocation::Inside,
        );
        assert_location(
            contained_results.intersection(),
            point(2, 1),
            RegionPointLocation::Inside,
        );
        for result in [contained_results.difference(), contained_results.xor()] {
            assert_location(result, point(2, 1), RegionPointLocation::Outside);
            assert_location(result, point(2, 3), RegionPointLocation::Inside);
        }
    }
}

#[test]
fn circular_conic_batch_reuses_one_authoritative_topology() {
    let first = circle(Real::zero());
    let second = circle(Real::one());
    let policy = CurveContext::STRICT;
    let batch = first.boolean_regions(&second, &policy).unwrap();
    assert_eq!(batch.certainty, CurveCertainty::Certified);
    let batch = batch.into_value();
    assert_eq!(batch.authored_carrier_pair_count(), 16);
    assert!(batch.candidate_carrier_pair_count() > 0);
    assert!(batch.candidate_carrier_pair_count() < batch.authored_carrier_pair_count());
    assert!(batch.topology_fragment_count() > 0);
    assert!(batch.topology_point_classification_count() < batch.topology_fragment_count());

    let operations = [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ];
    let shared = [
        batch.union(),
        batch.intersection(),
        batch.difference(),
        batch.xor(),
    ];
    let independent = operations.map(|operation| {
        first
            .boolean_region(&second, operation, &policy)
            .unwrap()
            .into_value()
    });
    for (shared, independent) in shared.into_iter().zip(&independent) {
        for x_numerator in -5_i8..=7 {
            for y_numerator in -5_i8..=5 {
                let sample = Point2::new(
                    (Real::from(x_numerator) / Real::from(2_i8)).unwrap(),
                    (Real::from(y_numerator) / Real::from(2_i8)).unwrap(),
                );
                assert_eq!(
                    shared
                        .classify_point(&sample, &policy)
                        .unwrap()
                        .into_value(),
                    independent
                        .classify_point(&sample, &policy)
                        .unwrap()
                        .into_value(),
                    "shared and native circle results differ at ({x_numerator}/2, {y_numerator}/2)",
                );
            }
        }
    }
}

#[test]
fn mixed_line_circular_conic_batch_reuses_one_authoritative_topology() {
    let first = capsule(0);
    let second = capsule(2);
    let policy = CurveContext::STRICT;
    let batch = first.boolean_regions(&second, &policy).unwrap();
    assert_eq!(batch.certainty, CurveCertainty::Certified);
    let batch = batch.into_value();
    assert!(batch.authored_carrier_pair_count() > 16);
    assert!(batch.candidate_carrier_pair_count() > 0);
    assert!(batch.candidate_carrier_pair_count() < batch.authored_carrier_pair_count());
    assert!(batch.topology_fragment_count() > 0);
    assert!(batch.topology_point_classification_count() < batch.topology_fragment_count());

    let independent = [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ]
    .map(|operation| {
        first
            .boolean_region(&second, operation, &policy)
            .unwrap()
            .into_value()
    });
    for (shared, independent) in [
        batch.union(),
        batch.intersection(),
        batch.difference(),
        batch.xor(),
    ]
    .into_iter()
    .zip(&independent)
    {
        for x_numerator in -11_i8..=15 {
            for y_numerator in -7_i8..=7 {
                let sample = Point2::new(
                    (Real::from(x_numerator) / Real::from(2_i8)).unwrap(),
                    (Real::from(y_numerator) / Real::from(2_i8)).unwrap(),
                );
                assert_eq!(
                    shared
                        .classify_point(&sample, &policy)
                        .unwrap()
                        .into_value(),
                    independent
                        .classify_point(&sample, &policy)
                        .unwrap()
                        .into_value(),
                    "shared and native capsule results differ at ({x_numerator}/2, {y_numerator}/2)",
                );
            }
        }
    }
}

#[test]
fn circular_boolean_outputs_publish_native_boundaries_under_both_policies() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = circle_with_policy(Real::zero(), &policy);
        let second = circle_with_policy(Real::one(), &policy);
        let batch = first
            .boolean_regions(&second, &policy)
            .expect("overlapping circles must complete under either policy")
            .into_value();
        for region in [
            batch.union(),
            batch.intersection(),
            batch.difference(),
            batch.xor(),
        ] {
            let (lines, arcs) = native_segment_counts(region, &policy);
            assert_eq!(lines, 0);
            assert!(arcs > 0);
            assert!(region.boundary_loops().iter().all(|boundary| {
                boundary
                    .fragments()
                    .iter()
                    .all(|fragment| !fragment.is_algebraic_endpoint_images())
            }));
        }

        let first = capsule(0);
        let second = capsule(2);
        let batch = first
            .boolean_regions(&second, &policy)
            .expect("overlapping capsules must complete under either policy")
            .into_value();
        for region in [
            batch.union(),
            batch.intersection(),
            batch.difference(),
            batch.xor(),
        ] {
            let (lines, arcs) = native_segment_counts(region, &policy);
            assert!(lines > 0);
            assert!(arcs > 0);
        }
    }
}

#[test]
fn noncircular_conic_boolean_output_is_not_mislabeled_as_native_arc() {
    let region = symbolic_quadratic_cap(Real::from(-4_i8), &CurveContext::STRICT);
    let union = region
        .boolean_region(&region, BooleanOp::Union, &CurveContext::STRICT)
        .expect("identical exact conic regions have an exact union")
        .into_value();
    assert!(matches!(
        union
            .native_contours_fast_path(&CurveContext::STRICT)
            .expect("native classification is evidence, not an API failure")
            .into_value(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    ));
}

#[test]
fn elevated_circular_boolean_outputs_publish_native_boundaries() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = circle_with_policy(Real::zero(), &policy);
        let second = symbolic_elevated_circle(Real::one(), &policy);
        let batch = first
            .boolean_regions(&second, &policy)
            .expect("a native and degree-elevated circle must share exact circle topology")
            .into_value();
        for region in [
            batch.union(),
            batch.intersection(),
            batch.difference(),
            batch.xor(),
        ] {
            let (lines, arcs) = native_segment_counts(region, &policy);
            assert_eq!(lines, 0);
            assert!(arcs > 0);
            assert!(region.boundary_loops().iter().all(|boundary| {
                boundary
                    .fragments()
                    .iter()
                    .all(|fragment| !fragment.is_algebraic_endpoint_images())
            }));
        }
    }
}

#[test]
fn mixed_line_circular_conic_degeneracy_matrix_matches_native_results() {
    let cases = [
        (capsule_at(0, 0), capsule_at(2, 1)),
        (circle(Real::zero()), square(-1, -3, 1, 3)),
        (circle(Real::zero()), square(2, -1, 4, 1)),
        (circle(Real::zero()), square(3, -1, 5, 1)),
        (circle(Real::zero()), square(-1, -1, 1, 1)),
    ];
    let policy = CurveContext::STRICT;
    for (case_index, (first, second)) in cases.into_iter().enumerate() {
        let batch = first
            .boolean_regions(&second, &policy)
            .unwrap()
            .into_value();
        if matches!(case_index, 2 | 3) {
            assert!(batch.intersection().is_empty());
            assert!(matches!(
                batch
                    .intersection()
                    .filled_side_is_left(&policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided(sides) if sides.is_empty()
            ));
        }
        let independent = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ]
        .map(|operation| {
            first
                .boolean_region(&second, operation, &policy)
                .unwrap()
                .into_value()
        });
        for (operation_index, (shared, independent)) in [
            batch.union(),
            batch.intersection(),
            batch.difference(),
            batch.xor(),
        ]
        .into_iter()
        .zip(&independent)
        .enumerate()
        {
            for x in -5_i64..=7 {
                for y in -4_i64..=4 {
                    let sample = point(x, y);
                    assert_eq!(
                        shared
                            .classify_point(&sample, &policy)
                            .unwrap()
                            .into_value(),
                        independent
                            .classify_point(&sample, &policy)
                            .unwrap()
                            .into_value(),
                        "mixed case {case_index}, operation {operation_index} differs at ({x}, {y})",
                    );
                }
            }
        }
    }
}

#[test]
fn mixed_line_circular_conic_batch_obeys_the_approximate_512_terminal() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let disk = circle_with_policy(undecidable_zero, &CurveContext::APPROXIMATE_512);
    let right_half = square(0, -3, 3, 3);

    assert!(matches!(
        disk.boolean_regions(&right_half, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = disk
        .boolean_regions(&right_half, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should decide mixed line/conic contacts");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);

    for (region, sample, expected) in [
        (
            batch.value.union(),
            point(-1, 0),
            RegionPointLocation::Inside,
        ),
        (
            batch.value.intersection(),
            point(1, 0),
            RegionPointLocation::Inside,
        ),
        (
            batch.value.difference(),
            point(-1, 0),
            RegionPointLocation::Inside,
        ),
        (batch.value.xor(), point(1, 0), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            region
                .classify_point(&sample, &CurveContext::APPROXIMATE_512)
                .unwrap()
                .into_value(),
            Classification::Decided(expected),
        );
    }
}

#[test]
fn circular_conic_batch_obeys_the_approximate_512_terminal() {
    let (first_center, second_center) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first = circle_with_policy(first_center.clone(), &CurveContext::APPROXIMATE_512);
    let second = circle_with_policy(second_center, &CurveContext::APPROXIMATE_512);

    assert!(matches!(
        first.boolean_regions(&second, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = first
        .boolean_regions(&second, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should decide equal circle supports");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);
    assert!(batch.value.difference().is_empty());
    assert!(batch.value.xor().is_empty());
    assert_location(
        batch.value.union(),
        Point2::new(first_center, Real::zero()),
        RegionPointLocation::Inside,
    );
}

#[test]
fn approximate_policy_reports_a_consumed_terminal_instead_of_relabeling_it_exact() {
    let (first_x, second_x) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first = symbolic_rectangle(first_x);
    let second = symbolic_rectangle(second_x);

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

    assert!(matches!(
        first.boolean_regions(&second, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = first
        .boolean_regions(&second, &CurveContext::APPROXIMATE_512)
        .expect("the shared arrangement must obey the authorized 512-bit terminal");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);
    assert_location(
        batch.value.union(),
        Point2::new(Real::one(), (Real::one() / Real::from(2_u8)).unwrap()),
        RegionPointLocation::Inside,
    );
}

#[test]
fn curve_path_construction_obeys_the_approximate_512_terminal() {
    let (first_end_x, second_start_x) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first_end = Point2::new(first_end_x, Real::zero());
    let second_start = Point2::new(second_start_x, Real::zero());
    let curves = vec![
        Curve2::from(LineSeg2::try_new(point(0, 0), first_end).unwrap()),
        Curve2::from(LineSeg2::try_new(second_start, point(0, 1)).unwrap()),
    ];

    assert!(matches!(
        CurvePath2::try_new_with_policy(curves.clone(), &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let path = CurvePath2::try_new_with_policy(curves, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should certify symbolic path connectivity");
    assert_eq!(path.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(path.value.curves().len(), 2);
}

#[test]
fn general_curve_batch_obeys_the_approximate_512_terminal() {
    let (first_height, second_height) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first = symbolic_quadratic_cap(-first_height, &CurveContext::APPROXIMATE_512);
    let second = symbolic_quadratic_cap(-second_height, &CurveContext::APPROXIMATE_512);

    assert!(matches!(
        first.boolean_regions(&second, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = first
        .boolean_regions(&second, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should decide equivalent general curves");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(batch.value.union().boundary_loops().len(), 1);
    assert_eq!(batch.value.intersection().boundary_loops().len(), 1);
    assert_location(
        batch.value.union(),
        point(0, 2),
        RegionPointLocation::Inside,
    );
    assert_location(
        batch.value.intersection(),
        point(0, 2),
        RegionPointLocation::Inside,
    );
    assert_location(
        batch.value.union(),
        point(0, -2),
        RegionPointLocation::Outside,
    );
    assert!(batch.value.difference().is_empty());
    assert!(batch.value.xor().is_empty());
}

#[test]
fn line_general_batch_obeys_the_approximate_512_terminal() {
    let first = square(0, 0, 4, 4);
    let symbolic_zero = support::terminally_unresolved_zero();
    let second = symbolic_general_line_region(symbolic_zero, &CurveContext::APPROXIMATE_512);

    assert!(matches!(
        first.boolean_regions(&second, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = first
        .boolean_regions(&second, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should decide the general line image");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);
    assert_location(
        batch.value.union(),
        point(2, 2),
        RegionPointLocation::Inside,
    );
    assert_location(
        batch.value.intersection(),
        point(2, 2),
        RegionPointLocation::Inside,
    );
    assert!(batch.value.difference().is_empty());
    assert!(batch.value.xor().is_empty());
}

#[test]
fn conic_general_batch_obeys_the_approximate_512_terminal() {
    let (first_center, second_center) = support::terminally_equal_pair(Real::pi() + Real::e());
    let first = circle_with_policy(first_center.clone(), &CurveContext::APPROXIMATE_512);
    let second = symbolic_elevated_circle(second_center, &CurveContext::APPROXIMATE_512);

    assert!(matches!(
        first.boolean_regions(&second, &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let batch = first
        .boolean_regions(&second, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal should decide the conic/general shared image");
    assert_eq!(batch.certainty, CurveCertainty::Approximate512Consumed);
    assert_location(
        batch.value.union(),
        Point2::new(first_center.clone(), Real::zero()),
        RegionPointLocation::Inside,
    );
    assert_location(
        batch.value.intersection(),
        Point2::new(first_center, Real::zero()),
        RegionPointLocation::Inside,
    );
    assert!(batch.value.difference().is_empty());
    assert!(batch.value.xor().is_empty());
}

#[test]
fn point_query_reports_when_approximate_policy_decides_a_symbolic_boundary() {
    let (boundary_x, query_x) = support::terminally_equal_pair(Real::pi() + Real::e());
    let region = symbolic_rectangle(boundary_x);
    let point = Point2::new(query_x, (Real::one() / Real::from(2_u8)).unwrap());

    let strict = region
        .classify_point(&point, &CurveContext::STRICT)
        .expect("strict point classification must preserve uncertainty as data");
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(matches!(strict.value, Classification::Uncertain(_)));

    let approximate = region
        .classify_point(&point, &CurveContext::APPROXIMATE_512)
        .expect("the authorized 512-bit terminal should identify the symbolic boundary");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        Classification::Decided(RegionPointLocation::Boundary)
    );
}

#[test]
fn approximate_offset_reports_a_consumed_terminal_for_symbolic_zero_distance() {
    let source = square(0, 0, 4, 4);
    let distance = support::terminally_unresolved_zero();

    assert!(matches!(
        source.offset(distance.clone(), &sharp_offset(), &CurveContext::STRICT),
        Err(hypercurve::ExactCurveError::Blocked(_))
    ));
    let outcome = source
        .offset(distance, &sharp_offset(), &CurveContext::APPROXIMATE_512)
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
    let algebraic = boolean_paths(
        &curved,
        &cutter_path,
        BooleanOp::Difference,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &policy,
    );
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
    let narrow = boolean_paths(
        &curved,
        &square_path(-3, -1, 3, 2),
        BooleanOp::Intersection,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &policy,
    );
    let wide = boolean_paths(
        &curved,
        &square_path(-3, -1, 3, 3),
        BooleanOp::Intersection,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &policy,
    );
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

#[test]
fn retained_regions_clip_degree_equivalent_shared_images_to_carrier_ranges() {
    let quadratic_start = point(-2, 4);
    let quadratic_control = point(0, -4);
    let quadratic_end = point(2, 4);
    let quadratic = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            quadratic_start.clone(),
            quadratic_control.clone(),
            quadratic_end.clone(),
        )),
        Curve2::from(LineSeg2::try_new(quadratic_end.clone(), quadratic_start.clone()).unwrap()),
    ])
    .unwrap();
    let cubic_first_control = Point2::new(
        (Real::from(-2_i8) / Real::from(3_i8)).unwrap(),
        (Real::from(-4_i8) / Real::from(3_i8)).unwrap(),
    );
    let cubic_second_control = Point2::new(
        (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
        (Real::from(-4_i8) / Real::from(3_i8)).unwrap(),
    );
    let cubic = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            quadratic_start.clone(),
            cubic_first_control.clone(),
            cubic_second_control.clone(),
            quadratic_end.clone(),
        )),
        Curve2::from(LineSeg2::try_new(quadratic_end.clone(), quadratic_start.clone()).unwrap()),
    ])
    .unwrap();
    let reversed_cubic = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            quadratic_end.clone(),
            cubic_second_control,
            cubic_first_control,
            quadratic_start.clone(),
        )),
        Curve2::from(LineSeg2::try_new(quadratic_start, quadratic_end).unwrap()),
    ])
    .unwrap();
    let policy = CurveContext::STRICT;
    let narrow = boolean_paths(
        &quadratic,
        &square_path(-3, -1, 3, 2),
        BooleanOp::Intersection,
        CurveBoundaryInteriorSide2::Left,
        CurveBoundaryInteriorSide2::Left,
        &policy,
    );
    assert!(narrow.has_algebraic_fragments());
    for (cubic, interior_side) in [
        (&cubic, CurveBoundaryInteriorSide2::Left),
        (&reversed_cubic, CurveBoundaryInteriorSide2::Right),
    ] {
        let wide = boolean_paths(
            cubic,
            &square_path(-3, -1, 3, 3),
            BooleanOp::Intersection,
            interior_side,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        assert!(wide.has_algebraic_fragments());

        let results = narrow.boolean_regions(&wide, &policy).unwrap().into_value();
        assert_location(results.union(), point(0, 3), RegionPointLocation::Boundary);
        assert_location(
            results.intersection(),
            point(0, 3),
            RegionPointLocation::Outside,
        );
        assert!(results.difference().is_empty());
        let between_tops =
            Point2::new(Real::zero(), (Real::from(5_i8) / Real::from(2_i8)).unwrap());
        assert_location(results.xor(), between_tops, RegionPointLocation::Inside);
    }
}

#[test]
fn retained_regions_clip_mobius_reparameterized_conics_to_carrier_ranges() {
    let start = point(-2, 4);
    let control = point(0, -4);
    let end = point(2, 4);
    let quadratic = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            start.clone(),
            control.clone(),
            end.clone(),
        )),
        Curve2::from(LineSeg2::try_new(end.clone(), start.clone()).unwrap()),
    ])
    .unwrap();
    // Scaling homogeneous Bernstein control i by lambda^i composes the
    // original quadratic with t = lambda*s / (1 - s + lambda*s). The image
    // and traversal are unchanged, but corresponding clipped parameters are
    // neither identical nor unit complements.
    let reparameterized = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![start.clone(), control.clone(), end.clone()],
                vec![Real::one(), Real::from(2_i8), Real::from(4_i8)],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(end.clone(), start.clone()).unwrap()),
    ])
    .unwrap();
    let reversed_reparameterized = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![end.clone(), control, start.clone()],
                vec![Real::from(4_i8), Real::from(2_i8), Real::one()],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(start.clone(), end.clone()).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let narrow = boolean_paths(
            &quadratic,
            &square_path(-3, -1, 3, 2),
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        assert!(narrow.has_algebraic_fragments());
        for (wide_path, interior_side) in [
            (&reparameterized, CurveBoundaryInteriorSide2::Left),
            (&reversed_reparameterized, CurveBoundaryInteriorSide2::Right),
        ] {
            let wide = boolean_paths(
                wide_path,
                &square_path(-3, -1, 3, 3),
                BooleanOp::Intersection,
                interior_side,
                CurveBoundaryInteriorSide2::Left,
                &policy,
            );
            assert!(wide.has_algebraic_fragments());

            let results = narrow.boolean_regions(&wide, &policy).unwrap().into_value();
            assert_location(results.union(), point(0, 3), RegionPointLocation::Boundary);
            assert_location(
                results.intersection(),
                point(0, 3),
                RegionPointLocation::Outside,
            );
            assert!(results.difference().is_empty());
            let between_tops =
                Point2::new(Real::zero(), (Real::from(5_i8) / Real::from(2_i8)).unwrap());
            assert_location(results.xor(), between_tops, RegionPointLocation::Inside);
        }
    }
}

#[test]
fn retained_regions_clip_non_axis_monotone_mobius_cubic_components() {
    // In the affine frame u = (x + y) / 2 and v = (x - y) / 2, this cubic has
    // u(t) = 3t and v(t) = 18t(1-t). It is therefore image-injective, but both
    // authored x and y coordinates reverse direction in the open domain.
    let controls = [point(0, 0), point(7, -5), point(8, -4), point(3, 3)];
    let polynomial = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            controls[3].clone(),
        )),
        Curve2::from(LineSeg2::try_new(controls[3].clone(), controls[0].clone()).unwrap()),
    ])
    .unwrap();
    // Scaling homogeneous Bernstein control i by 2^i composes t with the
    // projective map 2s / (1 + s) without changing the image or traversal.
    let reparameterized = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                controls.to_vec(),
                vec![
                    Real::one(),
                    Real::from(2_i8),
                    Real::from(4_i8),
                    Real::from(8_i8),
                ],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(controls[3].clone(), controls[0].clone()).unwrap()),
    ])
    .unwrap();

    let polynomial_rational =
        RationalBezier2::try_new(controls.to_vec(), vec![Real::one(); 4]).unwrap();
    let projective_rational = RationalBezier2::try_new(
        controls.to_vec(),
        vec![
            Real::one(),
            Real::from(2_i8),
            Real::from(4_i8),
            Real::from(8_i8),
        ],
    )
    .unwrap();
    assert!(matches!(
        polynomial_rational
            .intersection_contacts(&projective_rational, &CurveContext::STRICT)
            .unwrap(),
        RationalBezierIntersectionContacts2::Overlap(_)
    ));

    let narrow_clip = square_path(-20, -10, 20, -1);
    let wide_clip = square_path(-20, -10, 20, 0);
    let narrow_sample = point(4, -2);
    let wide_only_sample = Point2::new(
        Real::from(2_i8),
        (Real::from(-1_i8) / Real::from(2_i8)).unwrap(),
    );

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let narrow = boolean_paths(
            &polynomial,
            &narrow_clip,
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        let wide = boolean_paths(
            &reparameterized,
            &wide_clip,
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        assert!(narrow.has_algebraic_fragments());

        let evidence = narrow.intersect_region(&wide, &policy).unwrap();
        assert!(evidence.value.is_complete());
        assert_eq!(evidence.value.contacts().len(), 2);
        assert_eq!(evidence.value.overlaps().len(), 1);

        let results = narrow.boolean_regions(&wide, &policy).unwrap().into_value();
        assert!(results.difference().is_empty());
        for sample in [&narrow_sample, &wide_only_sample] {
            assert_eq!(
                results
                    .union()
                    .classify_point(sample, &policy)
                    .unwrap()
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside)
            );
        }
        assert_eq!(
            results
                .intersection()
                .classify_point(&narrow_sample, &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            results
                .intersection()
                .classify_point(&wide_only_sample, &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            results
                .xor()
                .classify_point(&narrow_sample, &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            results
                .xor()
                .classify_point(&wide_only_sample, &policy)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn independent_nonlinear_line_parameters_compact_to_reusable_regions() {
    let first = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![point(0, 0), point(1, 0), point(4, 0)],
                vec![Real::one(); 3],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 4), point(0, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![point(0, 0), point(3, 0), point(4, 0)],
                vec![Real::one(); 3],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 4), point(0, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
    ])
    .unwrap();
    let second_reversed = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![point(4, 0), point(3, 0), point(0, 0)],
                vec![Real::one(); 3],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(point(0, 0), point(0, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(0, 4), point(4, 4)).unwrap()),
        Curve2::from(LineSeg2::try_new(point(4, 4), point(4, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let narrow_clip = square_path(-1, -1, 2, 5);
        let narrow_topology = first
            .intersection_topology(&narrow_clip, &policy)
            .unwrap()
            .into_value();
        assert!(
            narrow_topology
                .first()
                .iter()
                .chain(narrow_topology.second())
                .flat_map(|split| split.materializations())
                .flat_map(|materialization| materialization.fragments())
                .any(|fragment| fragment.is_algebraic_endpoint_images())
        );
        let narrow = boolean_paths(
            &first,
            &narrow_clip,
            BooleanOp::Intersection,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        );
        assert!(!narrow.has_algebraic_fragments());
        assert_eq!(
            narrow
                .boundary_loops()
                .iter()
                .map(|boundary| boundary.len())
                .sum::<usize>(),
            4
        );
        for (wide_path, interior_side) in [
            (&second, CurveBoundaryInteriorSide2::Left),
            (&second_reversed, CurveBoundaryInteriorSide2::Right),
        ] {
            let wide_clip = square_path(-1, -1, 3, 5);
            let wide_topology = wide_path
                .intersection_topology(&wide_clip, &policy)
                .unwrap()
                .into_value();
            assert!(
                wide_topology
                    .first()
                    .iter()
                    .chain(wide_topology.second())
                    .flat_map(|split| split.materializations())
                    .flat_map(|materialization| materialization.fragments())
                    .any(|fragment| fragment.is_algebraic_endpoint_images())
            );
            let wide = boolean_paths(
                wide_path,
                &wide_clip,
                BooleanOp::Intersection,
                interior_side,
                CurveBoundaryInteriorSide2::Left,
                &policy,
            );
            assert!(!wide.has_algebraic_fragments());

            let results = narrow.boolean_regions(&wide, &policy).unwrap().into_value();
            assert_eq!(
                results
                    .union()
                    .boundary_loops()
                    .iter()
                    .map(|boundary| boundary.len())
                    .sum::<usize>(),
                4
            );
            assert_eq!(
                results
                    .intersection()
                    .boundary_loops()
                    .iter()
                    .map(|boundary| boundary.len())
                    .sum::<usize>(),
                4
            );
            assert_eq!(
                results
                    .xor()
                    .boundary_loops()
                    .iter()
                    .map(|boundary| boundary.len())
                    .sum::<usize>(),
                4
            );
            assert_location(results.union(), point(3, 0), RegionPointLocation::Boundary);
            assert_location(
                results.intersection(),
                point(2, 0),
                RegionPointLocation::Boundary,
            );
            assert!(results.difference().is_empty());
            assert_location(results.xor(), point(3, 2), RegionPointLocation::Boundary);
            assert_location(results.xor(), point(1, 2), RegionPointLocation::Outside);
        }
    }
}
