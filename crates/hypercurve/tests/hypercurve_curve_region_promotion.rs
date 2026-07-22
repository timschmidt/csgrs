use hypercurve::{
    BezierFlatteningOptions, CircularArc2, Classification, Contour2, Curve2, CurvePath2,
    CurvePolicy, CurveRegion2, CurveRegionLoopRole, FillRule, FiniteProjectionOptions,
    LineArcRegion2, LineSeg2, Point2, PolylineReconstructionOptions, QuadraticBezier2, Real,
    RegionPointLocation, Segment2, Similarity2,
};
use hyperreal::SymbolicDependencyMask;

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> Contour2 {
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(min_x, min_y), p(max_x, min_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, min_y), p(max_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, max_y), p(min_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(min_x, max_y), p(min_x, min_y)).unwrap()),
    ])
    .unwrap()
}

fn reversed(contour: &Contour2) -> Contour2 {
    Contour2::try_new_with_fill_rule(
        contour
            .segments()
            .iter()
            .rev()
            .map(Segment2::reversed)
            .collect(),
        contour.fill_rule(),
    )
    .unwrap()
}

fn square_with_redundant_edge() -> Contour2 {
    let points = [p(0, 0), p(2, 0), p(4, 0), p(4, 4), p(0, 4), p(0, 0)];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn right_isosceles_triangle() -> Contour2 {
    let points = [p(0, 0), p(4, 0), p(0, 4), p(0, 0)];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn double_wound_square(fill_rule: FillRule) -> Contour2 {
    let corners = [p(0, 0), p(10, 0), p(10, 10), p(0, 10), p(0, 0)];
    let segments = corners
        .windows(2)
        .chain(corners.windows(2))
        .map(|edge| Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap()))
        .collect();
    Contour2::try_new_with_fill_rule(segments, fill_rule).unwrap()
}

fn path_from_contour(contour: &Contour2) -> CurvePath2 {
    CurvePath2::try_new(
        contour
            .segments()
            .iter()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line.clone()),
                Segment2::Arc(arc) => Curve2::from(arc.clone()),
            })
            .collect(),
    )
    .unwrap()
}

fn full_circle_path(radius: i64) -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(
            CircularArc2::try_from_center(p(radius, 0), p(-radius, 0), p(0, 0), false).unwrap(),
        ),
        Curve2::from(
            CircularArc2::try_from_center(p(-radius, 0), p(radius, 0), p(0, 0), false).unwrap(),
        ),
    ])
    .unwrap()
}

fn double_wound_quadratic_cap() -> CurvePath2 {
    let curve = Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4)));
    let close = Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap());
    CurvePath2::try_new(vec![curve.clone(), close.clone(), curve, close]).unwrap()
}

fn quadratic_fillet_path() -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(0, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -2), p(0, 0)).unwrap()),
    ])
    .unwrap()
}

fn bow_tie_path() -> CurvePath2 {
    let points = [p(0, 0), p(4, 4), p(0, 4), p(4, 0), p(0, 0)];
    CurvePath2::try_new(
        points
            .windows(2)
            .map(|edge| Curve2::from(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap()))
            .collect(),
    )
    .unwrap()
}

fn bow_tie_contour(fill_rule: FillRule) -> Contour2 {
    let points = [p(0, 0), p(4, 4), p(0, 4), p(4, 0), p(0, 0)];
    Contour2::try_new_with_fill_rule(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
        fill_rule,
    )
    .unwrap()
}

fn u_shape() -> Contour2 {
    let points = [
        p(0, 0),
        p(10, 0),
        p(10, 10),
        p(7, 10),
        p(7, 3),
        p(3, 3),
        p(3, 10),
        p(0, 10),
        p(0, 0),
    ];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn dumbbell_shape() -> Contour2 {
    let points = [
        p(0, 0),
        p(4, 0),
        p(4, 1),
        p(8, 1),
        p(8, 0),
        p(12, 0),
        p(12, 4),
        p(8, 4),
        p(8, 3),
        p(4, 3),
        p(4, 4),
        p(0, 4),
        p(0, 0),
    ];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected decided result, got {reason:?}"),
    }
}

#[test]
fn unified_native_constructor_retains_zero_signed_area_boundary_for_diagnostics() {
    let policy = CurvePolicy::certified();
    let contour = bow_tie_contour(FillRule::EvenOdd);

    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour.clone()], &policy).unwrap();
    let native = decided(region.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours(), std::slice::from_ref(&contour));
    assert!(native.hole_contours().is_empty());
}

#[test]
fn unified_region_offsets_quadratic_boundary_through_certified_exact_segmentation() {
    let policy = CurvePolicy::certified();
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 0), p(0, 4), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(-2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-2, -2), p(-2, 0)).unwrap()),
    ])
    .unwrap();
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    assert!(matches!(
        source.offset(Real::one(), &policy).unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Unsupported)
    ));
    let options = BezierFlatteningOptions::try_new(q(1, 32), 12, &policy).unwrap();

    let segmented = decided(source.segment_certified(&options, &policy).unwrap());
    assert_eq!(segmented.report().max_source_chord_error(), &q(1, 32));
    assert!(segmented.report().lossy_boundary());
    assert_eq!(segmented.report().loop_reports().len(), 1);
    assert_eq!(
        segmented.report().loop_reports()[0].role(),
        CurveRegionLoopRole::Material
    );
    assert_eq!(
        segmented.report().loop_reports()[0].fill_rule(),
        FillRule::NonZero
    );
    assert!(segmented.report().loop_reports()[0].output_segment_count() > 4);
    assert!(matches!(
        segmented
            .region()
            .native_contours_fast_path(&policy)
            .unwrap(),
        Classification::Decided(_)
    ));

    let offset = decided(
        source
            .offset_with_certified_segmentation(Real::one(), &options, &policy)
            .unwrap(),
    );

    assert!(!offset.region().is_empty());
    assert!(!offset.report().used_exact_native_fast_path());
    assert!(offset.report().lossy_boundary());
    assert_eq!(offset.report().max_source_chord_error(), &q(1, 32));
    assert_eq!(offset.report().loop_reports().len(), 1);
    assert!(offset.report().loop_reports()[0].output_segment_count() > 4);
    assert!(matches!(
        offset.region().native_contours_fast_path(&policy).unwrap(),
        Classification::Decided(_)
    ));
}

#[test]
fn unified_region_bounds_cover_native_and_higher_order_carriers_exactly() {
    let policy = CurvePolicy::certified();
    let native = CurveRegion2::try_from_line_arc_region(
        &LineArcRegion2::from_material_contours(vec![square(-3, -2, 7, 5)]),
        &policy,
    )
    .unwrap();
    let native_bounds = decided(native.bounds(&policy).unwrap());
    assert_eq!(native_bounds.min_x(), &Real::from(-3));
    assert_eq!(native_bounds.min_y(), &Real::from(-2));
    assert_eq!(native_bounds.max_x(), &Real::from(7));
    assert_eq!(native_bounds.max_y(), &Real::from(5));

    let curved = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[double_wound_quadratic_cap()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    let curved_bounds = decided(curved.bounds(&policy).unwrap());
    assert_eq!(curved_bounds.min_x(), &Real::from(-2));
    assert_eq!(curved_bounds.min_y(), &Real::zero());
    assert_eq!(curved_bounds.max_x(), &Real::from(2));
    assert_eq!(curved_bounds.max_y(), &Real::from(4));

    assert!(
        CurveRegion2::empty()
            .bounds(&policy)
            .unwrap()
            .is_uncertain()
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_components() {
    let policy = CurvePolicy::certified();
    let source =
        LineArcRegion2::from_material_contours(vec![square(0, 0, 2, 2), square(4, 0, 6, 2)]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let offset = decided(promoted.offset(Real::from(2), &policy).unwrap());
    let native = decided(offset.line_arc_region_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(60))
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_voids() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(
        vec![square(0, 0, 20, 16)],
        vec![square(5, 5, 7, 7), square(9, 5, 11, 7)],
    );
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let offset = decided(promoted.offset(Real::from(-2), &policy).unwrap());
    let native = decided(offset.line_arc_region_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert_eq!(native.hole_contours().len(), 1);
    assert_eq!(
        offset.classify_point(&p(8, 6), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_expansion_regularizes_a_closed_concavity() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::from_material_contours(vec![u_shape()]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let offset = decided(promoted.offset(Real::from(3), &policy).unwrap());
    let native = decided(offset.line_arc_region_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        offset.classify_point(&p(5, 8), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        offset.classify_point(&p(-2, -2), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        offset.classify_point(&p(14, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_contracts_nonconvex_material_before_its_medial_collapse() {
    let policy = CurvePolicy::certified();
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy).unwrap();

    let eroded = decided(source.offset(-Real::one(), &policy).unwrap());

    assert_eq!(
        eroded.classify_point(&p(1, 1), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        eroded.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_discards_nonconvex_material_after_wavefront_collapse() {
    let policy = CurvePolicy::certified();
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy).unwrap();

    let eroded = decided(source.offset(Real::from(-2), &policy).unwrap());

    assert!(eroded.is_empty());
    assert_eq!(
        eroded.classify_point(&p(5, 1), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_nonconvex_erosion_splits_at_a_collapsed_neck() {
    let policy = CurvePolicy::certified();
    let source =
        CurveRegion2::try_from_native_material_contours(vec![dumbbell_shape()], &policy).unwrap();

    let eroded = decided(source.offset(-q(3, 2), &policy).unwrap());
    let native = decided(eroded.line_arc_region_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 2);
    assert!(native.hole_contours().is_empty());
    for point in [p(2, 2), p(10, 2)] {
        assert_eq!(
            eroded.classify_point(&point, &policy).unwrap(),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
    assert_eq!(
        eroded.classify_point(&p(6, 2), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_convex_contraction_decides_collapse_and_over_contraction() {
    let policy = CurvePolicy::certified();
    let source =
        CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy).unwrap();

    let near = decided(source.offset(-q(3, 2), &policy).unwrap());
    let near_bounds = decided(near.bounds(&policy).unwrap());
    assert_eq!(near_bounds.min_x(), &q(3, 2));
    assert_eq!(near_bounds.min_y(), &q(3, 2));
    assert_eq!(near_bounds.max_x(), &q(5, 2));
    assert_eq!(near_bounds.max_y(), &q(5, 2));
    assert!(decided(source.offset(Real::from(-2), &policy).unwrap()).is_empty());
    assert!(decided(source.offset(Real::from(-3), &policy).unwrap()).is_empty());
}

#[test]
fn unified_region_convex_erosion_handles_orientation_and_redundant_edges() {
    let policy = CurvePolicy::certified();
    for contour in [reversed(&square(0, 0, 4, 4)), square_with_redundant_edge()] {
        let source =
            CurveRegion2::try_from_native_material_contours(vec![contour], &policy).unwrap();
        let eroded = decided(source.offset(Real::from(-1), &policy).unwrap());
        let bounds = decided(eroded.bounds(&policy).unwrap());
        assert_eq!(bounds.min_x(), &Real::one());
        assert_eq!(bounds.min_y(), &Real::one());
        assert_eq!(bounds.max_x(), &Real::from(3));
        assert_eq!(bounds.max_y(), &Real::from(3));
        assert_eq!(
            eroded.classify_point(&p(2, 2), &policy).unwrap(),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn unified_region_convex_erosion_keeps_symbolic_diagonal_offsets_exact() {
    let policy = CurvePolicy::certified();
    let source =
        CurveRegion2::try_from_native_material_contours(vec![right_isosceles_triangle()], &policy)
            .unwrap();
    let root_two = Real::from(2).sqrt().unwrap();

    let eroded = match source.offset(Real::from(-1), &policy).unwrap() {
        Classification::Decided(region) => region,
        Classification::Uncertain(reason) => panic!("symbolic erosion was uncertain: {reason:?}"),
    };
    let native = decided(eroded.line_arc_region_fast_path(&policy).unwrap());
    let vertices = native.material_contours()[0]
        .segments()
        .iter()
        .map(|segment| segment.start().clone())
        .collect::<Vec<_>>();
    let far_axis_coordinate = 3.0 - std::f64::consts::SQRT_2;
    for expected in [
        (1.0, 1.0),
        (far_axis_coordinate, 1.0),
        (1.0, far_axis_coordinate),
    ] {
        assert!(vertices.iter().any(|vertex| {
            let x = vertex.x().to_f64_lossy().unwrap();
            let y = vertex.y().to_f64_lossy().unwrap();
            (x - expected.0).abs() < 1.0e-12 && (y - expected.1).abs() < 1.0e-12
        }));
    }
    assert!(
        vertices
            .iter()
            .flat_map(|vertex| [vertex.x(), vertex.y()])
            .any(|coordinate| {
                let facts = coordinate.detailed_facts();
                !facts.base.exact_rational
                    && (facts
                        .symbolic
                        .dependencies
                        .contains(SymbolicDependencyMask::SQRT)
                        || facts
                            .symbolic
                            .dependencies
                            .contains(SymbolicDependencyMask::OPAQUE))
            }),
        "the diagonal offset must remain an exact non-rational computable value"
    );

    let collapse_distance = Real::from(4) - Real::from(2) * root_two;
    #[cfg(feature = "predicates")]
    let collapse_reason = hypercurve::UncertaintyReason::Predicate;
    #[cfg(not(feature = "predicates"))]
    let collapse_reason = hypercurve::UncertaintyReason::RealSign;
    assert_eq!(
        source.offset(-collapse_distance, &policy).unwrap(),
        Classification::Uncertain(collapse_reason),
        "hyperreal currently cannot certify the composed radical equality at the exact inradius"
    );
    assert!(decided(source.offset(Real::from(-2), &policy).unwrap()).is_empty());
}

#[test]
fn unified_region_positive_offset_removes_exactly_collapsed_convex_hole() {
    let policy = CurvePolicy::certified();
    let source = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 20, 20)],
        vec![square(5, 5, 15, 15)],
        &policy,
    )
    .unwrap();

    let expanded = decided(source.offset(Real::from(5), &policy).unwrap());
    assert_eq!(decided(expanded.loop_roles(&policy).unwrap()).len(), 1);
    assert_eq!(
        expanded.classify_point(&p(10, 10), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn native_self_crossing_walk_regularizes_with_both_fill_rules() {
    let policy = CurvePolicy::certified();
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let contour = bow_tie_contour(fill_rule);
        let classification =
            CurveRegion2::try_from_regularized_native_contour(&contour, &policy).unwrap();
        let region = decided(classification);
        let native = decided(region.line_arc_region_fast_path(&policy).unwrap());
        assert_eq!(native.material_contours().len(), 2);
        assert!(native.hole_contours().is_empty());
        assert_eq!(
            region.classify_point(&p(2, 3), &policy).unwrap(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            region.classify_point(&p(2, 1), &policy).unwrap(),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            region.classify_point(&p(0, 2), &policy).unwrap(),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(Real::from(8))
        );
    }
}

#[test]
fn native_self_overlap_regularization_honors_winding_multiplicity() {
    let policy = CurvePolicy::certified();

    let nonzero = decided(
        CurveRegion2::try_from_regularized_native_contour(
            &double_wound_square(FillRule::NonZero),
            &policy,
        )
        .unwrap(),
    );
    let native = decided(nonzero.line_arc_region_fast_path(&policy).unwrap());
    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        decided(nonzero.filled_area(&policy).unwrap()),
        Some(Real::from(100))
    );

    let even_odd = decided(
        CurveRegion2::try_from_regularized_native_contour(
            &double_wound_square(FillRule::EvenOdd),
            &policy,
        )
        .unwrap(),
    );
    assert!(even_odd.is_empty());
}

#[test]
fn region_promotion_retains_explicit_roles_and_line_fast_path() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10), square(2, 2, 8, 8)], Vec::new());

    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    assert!(promoted.is_line_image_region_cached());
    assert_eq!(
        decided(promoted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Material,]
    );
    assert_eq!(
        decided(promoted.filled_side_is_left(&policy).unwrap()),
        &[true, true]
    );
    let profiles = decided(promoted.boundary_profiles(&policy).unwrap());
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().is_empty()));

    for point in [p(-1, 5), p(1, 1), p(5, 5)] {
        assert_eq!(
            promoted.classify_point(&point, &policy).unwrap(),
            source.classify_point(&point, &policy)
        );
    }
    assert_eq!(
        promoted.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside),
        "nested explicit material must not be reinterpreted as an even-odd hole"
    );

    let prepared = promoted.prepare_point_classifier(&policy).unwrap();
    assert!(prepared.uses_native_fast_path());
    assert_eq!(
        prepared.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn transformed_promotion_retains_explicit_roles_without_the_source_fast_path() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10), square(2, 2, 8, 8)], Vec::new());
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let transformed = promoted
        .transform_affine(
            &Real::from(2),
            &Real::zero(),
            &Real::zero(),
            &Real::from(3),
            &Real::from(5),
            &Real::from(-4),
            &policy,
        )
        .unwrap();

    assert!(transformed.is_line_image_region_cached());
    assert_eq!(
        decided(transformed.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Material]
    );
    assert_eq!(
        transformed.classify_point(&p(15, 11), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside),
        "a transformed nested material island must retain its explicit role"
    );

    let prepared = transformed.prepare_point_classifier(&policy).unwrap();
    assert!(prepared.uses_native_fast_path());
    assert_eq!(
        prepared.classify_point(&p(15, 11), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn similarity_rotation_preserves_unified_region_semantics_and_fast_path() {
    let policy = CurvePolicy::certified();
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10)],
        vec![square(2, 2, 8, 8)],
        &policy,
    )
    .unwrap();
    let quarter_turn = Similarity2::try_from_real_affine(
        Real::zero(),
        Real::from(-1),
        Real::one(),
        Real::zero(),
        Real::from(20),
        Real::from(3),
    )
    .unwrap();

    let rotated = region.transform_similarity(&quarter_turn, &policy).unwrap();

    assert!(
        rotated
            .prepare_point_classifier(&policy)
            .unwrap()
            .uses_native_fast_path()
    );
    assert_eq!(
        rotated.classify_point(&p(15, 4), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        rotated.classify_point(&p(15, 8), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(rotated.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
}

#[test]
fn exact_profiles_assign_holes_to_the_smallest_containing_material() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(3, 3, 7, 7)],
    );
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let profiles = decided(promoted.boundary_profiles(&policy).unwrap());

    assert_eq!(profiles.len(), 2);
    assert!(profiles[0].holes().is_empty());
    assert_eq!(profiles[1].material_loop_index(), 1);
    assert_eq!(profiles[1].hole_loop_indices(), &[2]);
    assert_eq!(
        decided(promoted.filled_area(&policy).unwrap()),
        Some(Real::from(120))
    );
}

#[test]
fn affine_line_fast_path_preserves_nonzero_and_even_odd_fill_rules() {
    let policy = CurvePolicy::certified();
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let source = LineArcRegion2::from_material_contours(vec![double_wound_square(fill_rule)]);
        let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();
        let transformed = promoted
            .transform_affine(
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &policy,
            )
            .unwrap();

        assert_eq!(transformed.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert!(transformed.is_line_image_region_cached());
        assert_eq!(
            transformed.classify_point(&p(10, 5), &policy).unwrap(),
            Classification::Decided(expected)
        );
        let prepared = transformed.prepare_point_classifier(&policy).unwrap();
        assert!(prepared.uses_native_fast_path());
        assert_eq!(
            prepared.classify_point(&p(10, 5), &policy).unwrap(),
            Classification::Decided(expected)
        );
    }
}

#[test]
fn authored_loop_semantics_drive_nonzero_and_even_odd_classification() {
    let policy = CurvePolicy::certified();
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let path = path_from_contour(&double_wound_square(fill_rule));
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
        )
        .unwrap();

        assert_eq!(region.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert_eq!(
            region.classify_point(&p(5, 5), &policy).unwrap(),
            Classification::Decided(expected)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(if fill_rule == FillRule::NonZero {
                Real::from(100)
            } else {
                Real::zero()
            })
        );
        assert!(region.is_line_image_region_cached());
    }
}

#[test]
fn nonlinear_curved_winding_honors_authored_fill_rules_exactly() {
    let policy = CurvePolicy::certified();
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[double_wound_quadratic_cap()],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
        )
        .unwrap();

        assert_eq!(
            decided(region.offset(Real::zero(), &policy).unwrap()),
            region,
            "zero offset must preserve higher-order regions"
        );

        assert_eq!(
            region.classify_point(&p(0, 2), &policy).unwrap(),
            Classification::Decided(expected)
        );
        let expected_depth = i32::from(expected == RegionPointLocation::Inside);
        assert_eq!(
            region.signed_depth(&p(0, 2), &policy).unwrap(),
            Classification::Decided(expected_depth)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(if fill_rule == FillRule::NonZero {
                q(32, 3)
            } else {
                Real::zero()
            })
        );
        let prepared = region.prepare_point_classifier(&policy).unwrap();
        assert!(!prepared.uses_native_fast_path());
        assert_eq!(
            prepared.classify_point(&p(0, 2), &policy).unwrap(),
            Classification::Decided(expected)
        );
        assert_eq!(
            prepared.signed_depth(&p(0, 2), &policy).unwrap(),
            Classification::Decided(expected_depth)
        );

        let transformed = region
            .transform_affine(
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &policy,
            )
            .unwrap();
        assert_eq!(
            transformed.classify_point(&p(2, 2), &policy).unwrap(),
            Classification::Decided(expected)
        );
    }
}

#[test]
fn nonperiodic_self_contact_does_not_claim_a_green_integral_as_filled_area() {
    let policy = CurvePolicy::certified();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[bow_tie_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
    )
    .unwrap();

    assert_eq!(
        decided(region.filled_area(&policy).unwrap()),
        None,
        "a self-crossing traversal needs arrangement regularization before its Green integral is a filled-set area"
    );
}

#[test]
fn native_contour_constructors_and_signed_depth_need_no_region_wrapper() {
    let policy = CurvePolicy::certified();
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(4, 4, 6, 6)],
        &policy,
    )
    .unwrap();

    assert_eq!(
        decided(region.loop_roles(&policy).unwrap()),
        vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
        ]
    );
    assert_eq!(
        region.signed_depth(&p(1, 1), &policy).unwrap(),
        Classification::Decided(1)
    );
    assert_eq!(
        region.signed_depth(&p(3, 3), &policy).unwrap(),
        Classification::Decided(2)
    );
    assert_eq!(
        region.signed_depth(&p(5, 5), &policy).unwrap(),
        Classification::Decided(1)
    );
    assert_eq!(
        region.signed_depth(&p(0, 5), &policy).unwrap(),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
    let prepared = region.prepare_point_classifier(&policy).unwrap();
    assert!(prepared.uses_native_fast_path());
    assert_eq!(
        prepared.signed_depth(&p(3, 3), &policy).unwrap(),
        Classification::Decided(2)
    );

    let boundaries = vec![square(2, 2, 8, 8), square(0, 0, 10, 10)];
    let nested = decided(
        CurveRegion2::try_from_native_boundary_contours(boundaries.clone(), &policy).unwrap(),
    );
    let borrowed = decided(
        CurveRegion2::try_from_native_boundary_contours_borrowed(&boundaries, &policy).unwrap(),
    );
    assert_eq!(
        decided(nested.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        nested.signed_depth(&p(5, 5), &policy).unwrap(),
        Classification::Decided(0)
    );
    assert_eq!(
        borrowed.signed_depth(&p(5, 5), &policy).unwrap(),
        Classification::Decided(0)
    );
}

#[test]
fn report_bearing_native_nesting_returns_only_the_unified_region() {
    let policy = CurvePolicy::certified();
    let built = CurveRegion2::try_from_native_boundary_contours_with_report(
        vec![square(0, 0, 8, 8), square(2, 2, 6, 6)],
        &policy,
    )
    .unwrap();

    assert_eq!(built.report().material_contour_count(), Some(1));
    assert_eq!(built.report().hole_contour_count(), Some(1));
    assert_eq!(
        built.region().unwrap().loop_role_counts(&policy).unwrap(),
        Classification::Decided((1, 1))
    );
    assert!(matches!(
        built.region_classification(),
        Classification::Decided(region) if region.len() == 2
    ));
    assert!(matches!(
        built.into_region_classification(),
        Classification::Decided(region) if region.len() == 2
    ));
}

#[test]
fn authored_line_arc_paths_retain_the_native_offset_engine() {
    let policy = CurvePolicy::certified();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[full_circle_path(5)],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();

    let prepared = region.prepare_point_classifier(&policy).unwrap();
    assert!(prepared.uses_native_fast_path());
    let expanded = decided(region.offset(Real::from(2), &policy).unwrap());
    let bounds = decided(expanded.bounds(&policy).unwrap());
    assert_eq!(bounds.min_x(), &Real::from(-7));
    assert_eq!(bounds.min_y(), &Real::from(-7));
    assert_eq!(bounds.max_x(), &Real::from(7));
    assert_eq!(bounds.max_y(), &Real::from(7));
}

#[test]
fn authored_nested_material_roles_certify_filled_sides_directly() {
    let policy = CurvePolicy::certified();
    let outer = path_from_contour(&square(0, 0, 10, 10));
    let inner = path_from_contour(&square(2, 2, 8, 8));
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[outer, inner],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
        &[FillRule::NonZero, FillRule::NonZero],
    )
    .unwrap();

    assert_eq!(
        decided(region.filled_side_is_left(&policy).unwrap()),
        &[true, true]
    );
    assert_eq!(
        region.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert!(
        region
            .prepare_point_classifier(&policy)
            .unwrap()
            .uses_native_fast_path()
    );
}

#[test]
fn unordered_native_arrangement_materializes_the_unified_carrier_with_report() {
    let policy = CurvePolicy::certified();
    let mut segments = square(0, 0, 10, 10).segments().to_vec();
    segments.rotate_left(2);

    let arranged =
        CurveRegion2::arrange_unordered_segments(segments, FillRule::NonZero, &policy).unwrap();

    assert_eq!(arranged.report().source_segments().len(), 4);
    assert_eq!(arranged.report().fill_rule(), FillRule::NonZero);
    let region = decided(arranged.region_classification());
    assert!(region.is_line_image_region_cached());
    assert_eq!(
        region.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_region_chamfer_and_fillet_dispatch_through_native_fast_path() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::from_material_contours(vec![square(0, 0, 4, 4)]);
    let region = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &policy)
            .unwrap(),
    );
    assert_eq!(
        decided(chamfered.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    assert_eq!(
        chamfered.loop_fill_rules(),
        Some([FillRule::NonZero].as_slice())
    );
    assert_eq!(
        chamfered.classify_point(&p(2, 2), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );

    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &p(1, 1), false, &policy)
            .unwrap(),
    );
    assert_eq!(
        decided(filleted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    assert_eq!(
        filleted.loop_fill_rules(),
        Some([FillRule::NonZero].as_slice())
    );
    assert_eq!(
        filleted.classify_point(&p(2, 2), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_region_chamfer_and_fillet_edit_materialized_higher_order_loops() {
    let policy = CurvePolicy::certified();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[quadratic_fillet_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    assert!(
        !region
            .prepare_point_classifier(&policy)
            .unwrap()
            .uses_native_fast_path()
    );

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &policy)
            .unwrap(),
    );
    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &p(3, 1), false, &policy)
            .unwrap(),
    );

    assert_eq!(chamfered.boundary_loops()[0].len(), 6);
    assert_eq!(filleted.boundary_loops()[0].len(), 7);
    for edited in [&chamfered, &filleted] {
        assert_eq!(
            decided(edited.loop_roles(&policy).unwrap()),
            vec![CurveRegionLoopRole::Material]
        );
        assert_eq!(
            edited.loop_fill_rules(),
            Some([FillRule::NonZero].as_slice())
        );
    }
}

#[test]
fn unified_region_offset_expands_material_and_contracts_holes() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10)], vec![square(3, 3, 7, 7)]);
    let region = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    let offset = decided(region.offset(Real::one(), &policy).unwrap());

    assert_eq!(
        offset.classify_point(&p(0, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        offset.classify_point(&p(3, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Inside),
        "positive region offset must contract a hole"
    );
    assert_eq!(
        offset.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(140))
    );
}

#[test]
fn region_promotion_retains_hole_role_for_projection() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10)], vec![square(2, 2, 8, 8)]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();

    assert_eq!(
        decided(promoted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        decided(promoted.filled_side_is_left(&policy).unwrap()),
        &[true, false]
    );
    assert_eq!(
        promoted.classify_point(&p(5, 5), &policy).unwrap(),
        Classification::Decided(RegionPointLocation::Outside)
    );
    let exact_profiles = decided(promoted.boundary_profiles(&policy).unwrap());
    assert_eq!(exact_profiles.len(), 1);
    assert_eq!(exact_profiles[0].material_loop_index(), 0);
    assert_eq!(exact_profiles[0].hole_loop_indices(), &[1]);
    assert_eq!(exact_profiles[0].holes().len(), 1);

    let options = FiniteProjectionOptions::try_new(0.01).unwrap();
    let profiles = decided(
        promoted
            .project_to_finite_profiles(&options, &policy)
            .unwrap(),
    );
    assert_eq!(profiles.len(), 1);
    assert_eq!(profiles[0].holes().len(), 1);
}

#[test]
fn empty_region_promotion_is_decided_and_cached() {
    let policy = CurvePolicy::certified();
    let promoted =
        CurveRegion2::try_from_line_arc_region(&LineArcRegion2::empty(), &policy).unwrap();

    assert!(promoted.is_empty());
    assert!(promoted.is_line_image_region_cached());
    assert!(decided(promoted.loop_roles(&policy).unwrap()).is_empty());
    assert!(decided(promoted.filled_side_is_left(&policy).unwrap()).is_empty());
    assert!(decided(promoted.line_arc_region_fast_path(&policy).unwrap()).is_empty());
    assert_eq!(CurveRegion2::empty(), CurveRegion2::default());
}

#[test]
fn segmented_profiles_recover_exact_scalar_curves_and_explicit_roles() {
    let policy = CurvePolicy::certified();
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10)], vec![square(2, 2, 8, 8)]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy).unwrap();
    let projection = FiniteProjectionOptions::try_new(0.01).unwrap();
    let profiles = decided(
        promoted
            .segment_to_finite_profiles(&projection, &policy)
            .unwrap(),
    );

    let recovered = CurveRegion2::recover_from_finite_profiles_with_report(
        &profiles,
        PolylineReconstructionOptions::default(),
        &policy,
    )
    .unwrap();

    assert_eq!(recovered.report().material_loop_count(), 1);
    assert_eq!(recovered.report().hole_loop_count(), 1);
    assert!(recovered.report().lossy_boundary());
    assert_eq!(recovered.report().profiles().len(), 1);
    assert_eq!(recovered.report().profiles()[0].holes().len(), 1);
    assert_eq!(
        decided(recovered.region().loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    for point in [p(-1, 5), p(1, 1), p(5, 5)] {
        assert_eq!(
            recovered.region().classify_point(&point, &policy).unwrap(),
            source.classify_point(&point, &policy)
        );
    }
}
