use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurvePolicy, CurveString2, FillRule, LineSeg2, Point2,
    Real, Segment2, SegmentKind, SymbolicDependencyMask,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), r(bulge))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn point_facts_preserve_exact_scale_and_symbolic_dependencies() {
    let rational = Point2::new(
        (Real::one() / Real::from(3_i8)).unwrap(),
        (Real::from(2_i8) / Real::from(3_i8)).unwrap(),
    );
    let rational_facts = rational.structural_facts();

    assert!(rational_facts.all_exact_rational());
    assert!(rational_facts.has_shared_denominator_schedule());
    assert_eq!(rational_facts.known_zero_mask, 0);
    assert_eq!(
        rational_facts.symbolic_dependencies,
        SymbolicDependencyMask::NONE
    );

    let symbolic = Point2::new(Real::pi(), Real::one());
    let symbolic_facts = symbolic.structural_facts();
    assert!(!symbolic_facts.all_exact_rational());
    assert!(
        symbolic_facts
            .symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
}

#[test]
fn line_segment_facts_certify_axis_alignment_without_float_predicates() {
    let horizontal = LineSeg2::try_new(p(0, 3), p(8, 3)).unwrap();
    let facts = horizontal.structural_facts();

    assert_eq!(facts.delta_known_zero_mask, 0b10);
    assert!(facts.is_axis_aligned());
    assert!(facts.coordinate_exact.all_exact_rational);

    let diagonal = LineSeg2::try_new(p(0, 0), p(8, 3)).unwrap();
    assert!(!diagonal.structural_facts().is_axis_aligned());
}

#[test]
fn prepared_curve_facts_summarize_segment_families_and_dependencies() {
    let line = Segment2::Line(
        LineSeg2::try_new(
            Point2::new(Real::pi(), Real::zero()),
            Point2::new(Real::pi(), r(4)),
        )
        .unwrap(),
    );
    let arc = Segment2::from_bulge(
        Point2::new(Real::pi(), r(4)),
        Point2::new(Real::pi() + Real::from(2_i8), r(4)),
        r(1),
    )
    .unwrap();
    let curve = CurveString2::try_new(vec![line, arc]).unwrap();

    let prepared = curve.prepare_topology_queries(&policy());
    let facts = prepared.facts();

    assert_eq!(facts.segment_kinds.lines, 1);
    assert_eq!(facts.segment_kinds.arcs, 1);
    assert_eq!(facts.segment_kinds.total(), 2);
    assert_eq!(facts.decided_segment_box_count, 1);
    assert!(!facts.has_decided_curve_box);
    assert!(!facts.all_exact_rational());
    assert!(
        facts
            .symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );
}

#[test]
fn prepared_region_facts_preserve_all_line_exact_grid_shape() {
    let contour = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 3, 0),
            vertex(0, 3, 0),
        ],
        FillRule::NonZero,
    )
    .unwrap();
    let region = hypercurve::Region2::from_material_contours(vec![contour]);
    let prepared = region.prepare_topology_queries(&policy());
    let facts = prepared.facts();

    assert_eq!(facts.material_contour_count, 1);
    assert_eq!(facts.hole_contour_count, 0);
    assert_eq!(facts.segment_kinds.lines, 4);
    assert_eq!(facts.segment_kinds.arcs, 0);
    assert!(facts.segment_kinds.all_lines());
    assert!(facts.scalar_exact.all_exact_rational);
    assert_eq!(facts.symbolic_dependencies, SymbolicDependencyMask::NONE);
    assert!(facts.has_decided_region_box);

    assert_eq!(
        prepared.classify_point(&p(1, 1), &policy()),
        Classification::Decided(hypercurve::RegionPointLocation::Inside)
    );
}

#[test]
fn native_segment_facts_report_kind_specific_shape() {
    let line = Segment2::Line(LineSeg2::try_new(p(0, 0), p(0, 5)).unwrap());
    let line_facts = line.structural_facts();
    assert_eq!(line_facts.kind, SegmentKind::Line);
    assert!(line_facts.axis_aligned_line);

    let arc = Segment2::from_bulge(p(-1, 0), p(1, 0), r(1)).unwrap();
    let arc_facts = arc.structural_facts();
    assert_eq!(arc_facts.kind, SegmentKind::Arc);
    assert!(arc_facts.exact_rational_arc_radius);
}

#[test]
fn symbolic_bulge_sign_selects_arc_orientation_exactly() {
    let positive = Segment2::from_bulge(p(-1, 0), p(1, 0), Real::pi()).unwrap();
    let Segment2::Arc(positive) = positive else {
        panic!("nonzero symbolic bulge should produce an arc");
    };
    assert!(!positive.is_clockwise());
    assert!(
        positive
            .structural_facts()
            .symbolic_dependencies
            .contains(SymbolicDependencyMask::PI)
    );

    let negative = Segment2::from_bulge(p(-1, 0), p(1, 0), -Real::pi()).unwrap();
    let Segment2::Arc(negative) = negative else {
        panic!("nonzero symbolic bulge should produce an arc");
    };
    assert!(negative.is_clockwise());
}
