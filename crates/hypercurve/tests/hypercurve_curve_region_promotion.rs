use hypercurve::{
    BezierFlatteningOptions, CircularArc2, Classification, Contour2, Curve2, CurveCertainty,
    CurveContext, CurveOutcome, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule,
    FiniteProjectionOptions, LineArcRegion2, LineSeg2, Point2, QuadraticBezier2, Real,
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

trait IntoCertifiedClassification<T> {
    fn into_certified_classification(self) -> Classification<T>;
}

impl<T> IntoCertifiedClassification<T> for Classification<T> {
    fn into_certified_classification(self) -> Classification<T> {
        self
    }
}

impl<T> IntoCertifiedClassification<T> for CurveOutcome<Classification<T>> {
    fn into_certified_classification(self) -> Classification<T> {
        assert_eq!(self.certainty, CurveCertainty::Certified);
        self.value
    }
}

fn decided<T>(classification: impl IntoCertifiedClassification<T>) -> T {
    match classification.into_certified_classification() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected decided result, got {reason:?}"),
    }
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}

#[test]
fn unified_native_constructor_retains_zero_signed_area_boundary_for_diagnostics() {
    let policy = CurveContext::STRICT;
    let contour = bow_tie_contour(FillRule::EvenOdd);

    let region = CurveRegion2::try_from_native_material_contours(vec![contour.clone()], &policy)
        .unwrap()
        .into_value();
    let native = decided(region.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours(), std::slice::from_ref(&contour));
    assert!(native.hole_contours().is_empty());
}

#[test]
fn unified_region_offsets_quadratic_boundary_through_certified_exact_segmentation() {
    let policy = CurveContext::STRICT;
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
        &policy,
    )
    .unwrap()
    .into_value();
    assert!(matches!(
        source.offset(Real::one(), &policy),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::Unsupported
    ));
    let options = BezierFlatteningOptions::try_new(q(1, 32), 12, &policy).unwrap();

    let segmented = decided(
        source
            .segment_certified(&options, &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(segmented.evidence().max_source_chord_error(), &q(1, 32));
    assert!(segmented.evidence().lossy_boundary());
    assert_eq!(segmented.evidence().loop_evidence().len(), 1);
    assert_eq!(
        segmented.evidence().loop_evidence()[0].role(),
        CurveRegionLoopRole::Material
    );
    assert_eq!(
        segmented.evidence().loop_evidence()[0].fill_rule(),
        FillRule::NonZero
    );
    assert!(segmented.evidence().loop_evidence()[0].output_segment_count() > 4);
    assert!(matches!(
        certified(
            segmented
                .region()
                .native_contours_fast_path(&policy)
                .unwrap()
        ),
        Classification::Decided(_)
    ));

    let offset = decided(
        source
            .offset_with_certified_segmentation(Real::one(), &options, &policy)
            .unwrap()
            .into_value(),
    );

    assert!(!offset.region().is_empty());
    assert!(!offset.evidence().used_exact_native_fast_path());
    assert!(offset.evidence().lossy_boundary());
    assert_eq!(offset.evidence().max_source_chord_error(), &q(1, 32));
    assert_eq!(offset.evidence().loop_evidence().len(), 1);
    assert!(offset.evidence().loop_evidence()[0].output_segment_count() > 4);
    assert!(matches!(
        certified(offset.region().native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}

#[test]
fn unified_region_bounds_cover_native_and_higher_order_carriers_exactly() {
    let policy = CurveContext::STRICT;
    let native = CurveRegion2::try_from_line_arc_region(
        &LineArcRegion2::from_material_contours(vec![square(-3, -2, 7, 5)]),
        &policy,
    )
    .unwrap()
    .into_value();
    let native_bounds = decided(native.bounds(&policy).unwrap());
    assert_eq!(native_bounds.min_x(), &Real::from(-3));
    assert_eq!(native_bounds.min_y(), &Real::from(-2));
    assert_eq!(native_bounds.max_x(), &Real::from(7));
    assert_eq!(native_bounds.max_y(), &Real::from(5));

    let curved = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[double_wound_quadratic_cap()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    let curved_bounds = decided(curved.bounds(&policy).unwrap());
    assert_eq!(curved_bounds.min_x(), &Real::from(-2));
    assert_eq!(curved_bounds.min_y(), &Real::zero());
    assert_eq!(curved_bounds.max_x(), &Real::from(2));
    assert_eq!(curved_bounds.max_y(), &Real::from(4));

    assert!(
        CurveRegion2::empty()
            .bounds(&policy)
            .unwrap()
            .map(|classification| classification.is_uncertain())
            .into_value()
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_components() {
    let policy = CurveContext::STRICT;
    let source =
        LineArcRegion2::from_material_contours(vec![square(0, 0, 2, 2), square(4, 0, 6, 2)]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    let offset = promoted
        .offset(Real::from(2), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(60))
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_voids() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(
        vec![square(0, 0, 20, 16)],
        vec![square(5, 5, 7, 7), square(9, 5, 11, 7)],
    );
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    let offset = promoted
        .offset(Real::from(-2), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert_eq!(native.hole_contours().len(), 1);
    assert_eq!(
        certified(offset.classify_point(&p(8, 6), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_expansion_regularizes_a_closed_concavity() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::from_material_contours(vec![u_shape()]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    let offset = promoted
        .offset(Real::from(3), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        certified(offset.classify_point(&p(5, 8), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(-2, -2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(14, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_contracts_nonconvex_material_before_its_medial_collapse() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source.offset(-Real::one(), &policy).unwrap().into_value();

    assert_eq!(
        certified(eroded.classify_point(&p(1, 1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        certified(eroded.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_discards_nonconvex_material_after_wavefront_collapse() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source.offset(Real::from(-2), &policy).unwrap().into_value();

    assert!(eroded.is_empty());
    assert_eq!(
        certified(eroded.classify_point(&p(5, 1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_nonconvex_erosion_splits_at_a_collapsed_neck() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![dumbbell_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source.offset(-q(3, 2), &policy).unwrap().into_value();
    let native = decided(eroded.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 2);
    assert!(native.hole_contours().is_empty());
    for point in [p(2, 2), p(10, 2)] {
        assert_eq!(
            certified(eroded.classify_point(&point, &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
    assert_eq!(
        certified(eroded.classify_point(&p(6, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_convex_contraction_decides_collapse_and_over_contraction() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy)
        .unwrap()
        .into_value();

    let near = source.offset(-q(3, 2), &policy).unwrap().into_value();
    let near_bounds = decided(near.bounds(&policy).unwrap());
    assert_eq!(near_bounds.min_x(), &q(3, 2));
    assert_eq!(near_bounds.min_y(), &q(3, 2));
    assert_eq!(near_bounds.max_x(), &q(5, 2));
    assert_eq!(near_bounds.max_y(), &q(5, 2));
    assert!(
        source
            .offset(Real::from(-2), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
    assert!(
        source
            .offset(Real::from(-3), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

#[test]
fn unified_region_convex_erosion_handles_orientation_and_redundant_edges() {
    let policy = CurveContext::STRICT;
    for contour in [reversed(&square(0, 0, 4, 4)), square_with_redundant_edge()] {
        let source = CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
            .unwrap()
            .into_value();
        let eroded = source.offset(Real::from(-1), &policy).unwrap().into_value();
        let bounds = decided(eroded.bounds(&policy).unwrap());
        assert_eq!(bounds.min_x(), &Real::one());
        assert_eq!(bounds.min_y(), &Real::one());
        assert_eq!(bounds.max_x(), &Real::from(3));
        assert_eq!(bounds.max_y(), &Real::from(3));
        assert_eq!(
            certified(eroded.classify_point(&p(2, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn unified_region_convex_erosion_keeps_symbolic_diagonal_offsets_and_collapse_exact() {
    let policy = CurveContext::STRICT;
    let source =
        CurveRegion2::try_from_native_material_contours(vec![right_isosceles_triangle()], &policy)
            .unwrap()
            .into_value();
    let root_two = Real::from(2).sqrt().unwrap();

    let eroded = source.offset(Real::from(-1), &policy).unwrap().into_value();
    let native = decided(eroded.native_contours_fast_path(&policy).unwrap());
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
    assert!(
        source
            .offset(-collapse_distance, &policy)
            .unwrap()
            .into_value()
            .is_empty(),
        "the exact radical inradius must collapse the triangle without a blocker"
    );
    assert!(
        source
            .offset(Real::from(-2), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

#[test]
fn unified_region_positive_offset_removes_exactly_collapsed_convex_hole() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 20, 20)],
        vec![square(5, 5, 15, 15)],
        &policy,
    )
    .unwrap()
    .into_value();

    let expanded = source.offset(Real::from(5), &policy).unwrap().into_value();
    assert_eq!(decided(expanded.loop_roles(&policy).unwrap()).len(), 1);
    assert_eq!(
        certified(expanded.classify_point(&p(10, 10), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_native_arrangement_exposes_immediate_evidence() {
    let source = square(0, 0, 4, 4).segments().to_vec();
    let result =
        CurveRegion2::arrange_unordered_segments(source, FillRule::NonZero, &CurveContext::STRICT)
            .unwrap()
            .into_value();

    assert!(result.region().is_some());
    assert_eq!(result.fill_rule(), FillRule::NonZero);
    assert_eq!(result.source_segment_count(), 4);
    assert!(result.status().unwrap().is_native_exact());
    assert_eq!(result.summary().materialized_region(), Some(true));
    assert_eq!(result.blocker(), None);
}

#[test]
fn native_self_crossing_walk_regularizes_with_both_fill_rules() {
    let policy = CurveContext::STRICT;
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let contour = bow_tie_contour(fill_rule);
        let classification = CurveRegion2::try_from_regularized_native_contour(&contour, &policy)
            .unwrap()
            .into_value();
        let region = decided(classification);
        let native = decided(region.native_contours_fast_path(&policy).unwrap());
        assert_eq!(native.material_contours().len(), 2);
        assert!(native.hole_contours().is_empty());
        assert_eq!(
            certified(region.classify_point(&p(2, 3), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(region.classify_point(&p(2, 1), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(region.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(Real::from(8))
        );
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_self_crossing_walks() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
            let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[bow_tie_path()],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value();
            let region = raw.regularized_region(&policy).unwrap().into_value();
            let native = decided(region.native_contours_fast_path(&policy).unwrap());
            assert_eq!(native.material_contours().len(), 2);
            assert!(native.hole_contours().is_empty());
            for (point, expected) in [
                (p(2, 3), RegionPointLocation::Inside),
                (p(2, 1), RegionPointLocation::Inside),
                (p(0, 2), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    certified(region.classify_point(&point, &policy).unwrap()),
                    Classification::Decided(expected)
                );
            }
            assert_eq!(
                decided(region.filled_area(&policy).unwrap()),
                Some(Real::from(8))
            );
        }
    }
}

#[test]
fn native_self_overlap_regularization_honors_winding_multiplicity() {
    let policy = CurveContext::STRICT;

    let nonzero = decided(
        CurveRegion2::try_from_regularized_native_contour(
            &double_wound_square(FillRule::NonZero),
            &policy,
        )
        .unwrap()
        .into_value(),
    );
    let native = decided(nonzero.native_contours_fast_path(&policy).unwrap());
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
        .unwrap()
        .into_value(),
    );
    assert!(even_odd.is_empty());
}

#[test]
fn authoritative_curve_region_arrangement_honors_coincident_winding_multiplicity() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let regularize = |fill_rule| {
            let contour = double_wound_square(fill_rule);
            let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[path_from_contour(&contour)],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value();
            raw.regularized_region(&policy).unwrap().into_value()
        };

        let nonzero = regularize(FillRule::NonZero);
        assert_eq!(
            decided(nonzero.filled_area(&policy).unwrap()),
            Some(Real::from(100))
        );
        assert!(regularize(FillRule::EvenOdd).is_empty());
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_signed_loop_composition() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let paths = [
            path_from_contour(&square(0, 0, 4, 4)),
            path_from_contour(&square(2, 0, 6, 4)),
        ];
        let union = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &paths,
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
            &[FillRule::NonZero, FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value()
        .regularized_region(&policy)
        .unwrap()
        .into_value();
        assert_eq!(
            decided(union.filled_area(&policy).unwrap()),
            Some(Real::from(24))
        );
        assert_eq!(
            certified(union.classify_point(&p(3, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );

        let cancellation = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &[paths[0].clone(), paths[0].clone()],
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            &[FillRule::NonZero, FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value()
        .regularized_region(&policy)
        .unwrap()
        .into_value();
        assert!(cancellation.is_empty());
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_nonlinear_winding() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let regularize = |fill_rule| {
            CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[double_wound_quadratic_cap()],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value()
            .regularized_region(&policy)
            .unwrap()
            .into_value()
        };
        let nonzero = regularize(FillRule::NonZero);
        assert_eq!(
            certified(nonzero.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            decided(nonzero.filled_area(&policy).unwrap()),
            Some(q(32, 3))
        );
        assert!(regularize(FillRule::EvenOdd).is_empty());
    }
}

#[test]
fn region_promotion_retains_explicit_roles_and_line_fast_path() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10), square(2, 2, 8, 8)], Vec::new());

    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

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
            certified(promoted.classify_point(&point, &policy).unwrap()),
            source.classify_point(&point, &policy)
        );
    }
    assert_eq!(
        certified(promoted.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "nested explicit material must not be reinterpreted as an even-odd hole"
    );

    assert!(matches!(
        certified(promoted.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}

#[test]
fn transformed_promotion_retains_explicit_roles_without_the_source_fast_path() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10), square(2, 2, 8, 8)], Vec::new());
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

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
        .unwrap()
        .into_value();

    assert_eq!(
        decided(transformed.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Material]
    );
    assert_eq!(
        certified(transformed.classify_point(&p(15, 11), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "a transformed nested material island must retain its explicit role"
    );

    assert!(matches!(
        certified(transformed.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}

#[test]
fn similarity_rotation_preserves_unified_region_semantics_and_fast_path() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10)],
        vec![square(2, 2, 8, 8)],
        &policy,
    )
    .unwrap()
    .into_value();
    let quarter_turn = Similarity2::try_from_real_affine(
        Real::zero(),
        Real::from(-1),
        Real::one(),
        Real::zero(),
        Real::from(20),
        Real::from(3),
    )
    .unwrap();

    let rotated = region
        .transform_similarity(&quarter_turn, &policy)
        .unwrap()
        .into_value();

    assert!(matches!(
        certified(rotated.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
    assert_eq!(
        certified(rotated.classify_point(&p(15, 4), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(rotated.classify_point(&p(15, 8), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(rotated.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
}

#[test]
fn exact_profiles_assign_holes_to_the_smallest_containing_material() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(3, 3, 7, 7)],
    );
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

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
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let source = LineArcRegion2::from_material_contours(vec![double_wound_square(fill_rule)]);
        let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
            .unwrap()
            .into_value();
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
            .unwrap()
            .into_value();

        assert_eq!(transformed.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert_eq!(
            certified(transformed.classify_point(&p(10, 5), &policy).unwrap()),
            Classification::Decided(expected)
        );
        assert!(matches!(
            certified(transformed.native_contours_fast_path(&policy).unwrap()),
            Classification::Decided(_)
        ));
    }
}

#[test]
fn authored_loop_semantics_drive_nonzero_and_even_odd_classification() {
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let path = path_from_contour(&double_wound_square(fill_rule));
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
            &policy,
        )
        .unwrap()
        .into_value();

        assert_eq!(region.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert_eq!(
            certified(region.classify_point(&p(5, 5), &policy).unwrap()),
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
    }
}

#[test]
fn nonlinear_curved_winding_honors_authored_fill_rules_exactly() {
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[double_wound_quadratic_cap()],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
            &policy,
        )
        .unwrap()
        .into_value();

        assert_eq!(
            region.offset(Real::zero(), &policy).unwrap().into_value(),
            region,
            "zero offset must preserve higher-order regions"
        );

        assert_eq!(
            certified(region.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(expected)
        );
        let expected_depth = i32::from(expected == RegionPointLocation::Inside);
        assert_eq!(
            certified(region.signed_depth(&p(0, 2), &policy).unwrap()),
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
            .unwrap()
            .into_value();
        assert_eq!(
            certified(transformed.classify_point(&p(2, 2), &policy).unwrap()),
            Classification::Decided(expected)
        );
    }
}

#[test]
fn nonperiodic_self_contact_does_not_claim_a_green_integral_as_filled_area() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[bow_tie_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.filled_area(&policy).unwrap()),
        None,
        "a self-crossing traversal needs arrangement regularization before its Green integral is a filled-set area"
    );
}

#[test]
fn native_contour_constructors_and_signed_depth_need_no_region_wrapper() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(4, 4, 6, 6)],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.loop_roles(&policy).unwrap()),
        vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
        ]
    );
    assert_eq!(
        certified(region.signed_depth(&p(1, 1), &policy).unwrap()),
        Classification::Decided(1)
    );
    assert_eq!(
        certified(region.signed_depth(&p(3, 3), &policy).unwrap()),
        Classification::Decided(2)
    );
    assert_eq!(
        certified(region.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(1)
    );
    assert_eq!(
        certified(region.signed_depth(&p(0, 5), &policy).unwrap()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
    let boundaries = vec![square(2, 2, 8, 8), square(0, 0, 10, 10)];
    let nested = decided(
        CurveRegion2::try_from_native_boundary_contours(boundaries.clone(), &policy)
            .unwrap()
            .into_value(),
    );
    let borrowed = decided(
        CurveRegion2::try_from_native_boundary_contours_borrowed(&boundaries, &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(
        decided(nested.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        certified(nested.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(0)
    );
    assert_eq!(
        certified(borrowed.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(0)
    );
}
#[test]
fn authored_line_arc_paths_retain_the_native_offset_engine() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[full_circle_path(5)],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
    let expanded = region.offset(Real::from(2), &policy).unwrap().into_value();
    let bounds = decided(expanded.bounds(&policy).unwrap());
    assert_eq!(bounds.min_x(), &Real::from(-7));
    assert_eq!(bounds.min_y(), &Real::from(-7));
    assert_eq!(bounds.max_x(), &Real::from(7));
    assert_eq!(bounds.max_y(), &Real::from(7));
}

#[test]
fn authored_nested_material_roles_certify_filled_sides_directly() {
    let policy = CurveContext::STRICT;
    let outer = path_from_contour(&square(0, 0, 10, 10));
    let inner = path_from_contour(&square(2, 2, 8, 8));
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[outer, inner],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
        &[FillRule::NonZero, FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.filled_side_is_left(&policy).unwrap()),
        &[true, true]
    );
    assert_eq!(
        certified(region.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}
#[test]
fn unified_region_chamfer_and_fillet_dispatch_through_native_fast_path() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::from_material_contours(vec![square(0, 0, 4, 4)]);
    let region = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &policy)
            .unwrap()
            .into_value(),
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
        certified(chamfered.classify_point(&p(2, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );

    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &p(1, 1), false, &policy)
            .unwrap()
            .into_value(),
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
        certified(filleted.classify_point(&p(2, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_region_chamfer_and_fillet_edit_materialized_higher_order_loops() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[quadratic_fillet_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Uncertain(_)
    ));

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &policy)
            .unwrap()
            .into_value(),
    );
    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &p(3, 1), false, &policy)
            .unwrap()
            .into_value(),
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

#[cfg(feature = "predicates")]
#[test]
fn materialized_boundary_paths_obey_terminal_policy_once() {
    let start = Point2::new(Real::pi() + Real::e(), Real::zero());
    let end = Point2::new(Real::e() + Real::pi(), Real::zero());
    let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        start,
        p(0, 1),
        end,
    ))])
    .expect("one-curve path construction has no adjacency decision");
    let constructed =
        CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::APPROXIMATE_512)
            .expect("the authorized terminal must construct the symbolic loop");
    assert_eq!(
        constructed.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let region = constructed.into_value();

    let strict = region
        .materialized_boundary_paths(&CurveContext::STRICT)
        .expect("strict materialization must preserve the symbolic closing seam uncertainty");
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );

    let approximate = region
        .materialized_boundary_paths(&CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal must materialize the exact boundary");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let Classification::Decided(paths) = approximate.value else {
        panic!("the symbolic boundary is exactly representable");
    };
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 1);
    assert!(matches!(
        paths[0].curves()[0].geometry(),
        hypercurve::CurveGeometry2::QuadraticBezier(_)
    ));

    assert_eq!(
        region
            .materialized_boundary_paths(&CurveContext::APPROXIMATE_512)
            .expect("terminal replay remains authorized")
            .certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        region
            .materialized_boundary_paths(&CurveContext::STRICT)
            .expect("strict replay remains an explicit classification")
            .value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );
}

#[cfg(feature = "predicates")]
#[test]
fn higher_order_region_fillet_obeys_terminal_policy_once() {
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[quadratic_fillet_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let center = Point2::new(Real::from(3) + undecidable_zero, Real::one());

    let strict = region
        .fillet_loop_vertex_by_parameters(
            0,
            1,
            q(3, 4),
            q(1, 2),
            &center,
            false,
            &CurveContext::STRICT,
        )
        .unwrap();
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );

    let approximate = region
        .fillet_loop_vertex_by_parameters(
            0,
            1,
            q(3, 4),
            q(1, 2),
            &center,
            false,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let Classification::Decided(filleted) = approximate.value else {
        panic!("the authorized terminal must complete the higher-order region fillet");
    };
    assert_eq!(filleted.boundary_loops()[0].len(), 7);
}

#[test]
fn unified_region_offset_expands_material_and_contracts_holes() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10)], vec![square(3, 3, 7, 7)]);
    let region = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    let offset = region.offset(Real::one(), &policy).unwrap().into_value();

    assert_eq!(
        certified(offset.classify_point(&p(0, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(3, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "positive region offset must contract a hole"
    );
    assert_eq!(
        certified(offset.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(140))
    );
}

#[test]
fn region_promotion_retains_hole_role_for_projection() {
    let policy = CurveContext::STRICT;
    let source = LineArcRegion2::new(vec![square(0, 0, 10, 10)], vec![square(2, 2, 8, 8)]);
    let promoted = CurveRegion2::try_from_line_arc_region(&source, &policy)
        .unwrap()
        .into_value();

    assert_eq!(
        decided(promoted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        decided(promoted.filled_side_is_left(&policy).unwrap()),
        &[true, false]
    );
    assert_eq!(
        certified(promoted.classify_point(&p(5, 5), &policy).unwrap()),
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
fn empty_region_promotion_is_decided_and_reusable() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_line_arc_region(&LineArcRegion2::empty(), &policy)
        .unwrap()
        .into_value();

    assert!(promoted.is_empty());
    assert!(decided(promoted.loop_roles(&policy).unwrap()).is_empty());
    assert!(decided(promoted.filled_side_is_left(&policy).unwrap()).is_empty());
    assert!(
        decided(promoted.native_contours_fast_path(&policy).unwrap())
            .material_contours()
            .is_empty()
    );
    assert_eq!(CurveRegion2::empty(), CurveRegion2::default());
}
