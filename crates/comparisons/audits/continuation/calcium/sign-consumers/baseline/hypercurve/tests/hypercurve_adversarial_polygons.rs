use hypercurve::{
    Aabb2, BooleanOp, BulgeVertex2, Classification, Contour2, CurveContext, CurveRegion2, FillRule,
    OffsetCornerStyle2, Point2, PolylineReconstructionOptions, Real, Segment2,
};
use proptest::prelude::*;

#[derive(Clone, Debug)]
struct PolygonCase {
    source_points: Vec<(i32, i32)>,
    material: Contour2,
    holes: Vec<Contour2>,
    cutter: CurveRegion2,
}

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), Real::zero())
}

fn contour_from_points(points: &[(i32, i32)]) -> Contour2 {
    let vertices: Vec<_> = points.iter().map(|&(x, y)| vertex(x, y)).collect();
    Contour2::from_bulge_vertices_with_fill_rule(&vertices, FillRule::NonZero).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    contour_from_points(&[(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)])
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

fn reconstruction_options() -> PolylineReconstructionOptions {
    PolylineReconstructionOptions {
        // These adversarial cases are polygonal by construction. Keep arc
        // promotion out of this test so failures localize to polygon
        // reconstruction, clipping, and offset topology rather than sampled
        // curve fitting heuristics.
        min_arc_points: 64,
        distance_tolerance: 1e-8,
        duplicate_point_tolerance: 1e-12,
        ..PolylineReconstructionOptions::default()
    }
}

fn assert_point_finite(point: &Point2) {
    assert!(point.x().to_f64_lossy().is_some_and(f64::is_finite));
    assert!(point.y().to_f64_lossy().is_some_and(f64::is_finite));
}

fn assert_segment_finite(segment: &Segment2) {
    match segment {
        Segment2::Line(line) => {
            assert_point_finite(line.start());
            assert_point_finite(line.end());
        }
        Segment2::Arc(arc) => {
            assert_point_finite(arc.start());
            assert_point_finite(arc.end());
            assert_point_finite(arc.center());
            assert!(
                arc.radius_squared()
                    .to_f64_lossy()
                    .is_some_and(f64::is_finite)
            );
        }
    }
}

fn assert_contour_finite(contour: &Contour2) {
    assert!(!contour.is_empty());
    for segment in contour.segments() {
        assert_segment_finite(segment);
    }
    if let Classification::Decided(bounds) = Aabb2::from_contour(contour, &policy()).unwrap() {
        assert_point_finite(bounds.min());
        assert_point_finite(bounds.max());
    }
}

fn curve_region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
    CurveRegion2::try_from_native_contours(material, holes, &policy())
        .unwrap()
        .into_value()
}

fn assert_region_topology(region: &CurveRegion2, operation: &str) {
    let classification = region
        .native_contours_fast_path(&policy())
        .unwrap()
        .into_value();
    if let Classification::Decided(native) = classification {
        for contour in native
            .material_contours()
            .iter()
            .chain(native.hole_contours())
        {
            assert_contour_finite(contour);
        }
        return;
    }

    let role_counts = region.loop_role_counts(&policy()).unwrap().into_value();
    let Classification::Decided((material_count, hole_count)) = role_counts else {
        panic!("polygon {operation} did not retain decided loop roles: {role_counts:?}");
    };
    assert_eq!(material_count + hole_count, region.boundary_loops().len());
    for boundary in region.boundary_loops() {
        assert!(!boundary.fragments().is_empty());
    }
}

fn exercise_offsets(contour: &Contour2, distance: i32) {
    let policy = policy();
    assert_contour_finite(contour);

    let _ = contour.has_self_contacts(&policy).unwrap();
    let source = CurveRegion2::try_from_native_material_contours(vec![contour.clone()], &policy)
        .unwrap()
        .into_value();
    let offset = source
        .offset(s(distance), &OffsetCornerStyle2::Round, &policy)
        .unwrap()
        .into_value();
    assert_region_topology(&offset, "offset");
}

fn exercise_clipping(a: &CurveRegion2, b: &CurveRegion2) {
    let policy = policy();

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let region = a.boolean_region(b, op, &policy).unwrap().into_value();
        assert_region_topology(&region, "Boolean");
    }
}

fn exercise_reconstruction(points: &[(i32, i32)]) {
    let mut samples: Vec<_> = points.iter().map(|&(x, y)| p(x, y)).collect();
    if samples.len() > 3 {
        samples.insert(1, samples[1].clone());
        samples.push(samples[0].clone());
    }

    let contour =
        Contour2::reconstruct_from_closed_polyline(&samples, reconstruction_options()).unwrap();
    assert_contour_finite(&contour);
    let _ = contour.intersect_self(&policy()).unwrap();
    if contour.has_self_contacts(&policy()).unwrap() == Classification::Decided(false) {
        exercise_offsets(&contour, 1);
    }
}

fn large_concavity(width: i32, height: i32, throat: i32) -> Vec<(i32, i32)> {
    let arm = throat.max(1);
    vec![
        (0, 0),
        (width, 0),
        (width, height),
        (width - arm, height),
        (width - arm, arm),
        (arm, arm),
        (arm, height),
        (0, height),
    ]
}

fn slender_concavity(width: i32, height: i32, slot_x: i32, slot_width: i32) -> Vec<(i32, i32)> {
    let left = slot_x.clamp(1, width - slot_width - 1);
    let right = left + slot_width;
    vec![
        (0, 0),
        (width, 0),
        (width, height),
        (right, height),
        (right, 1),
        (left, 1),
        (left, height),
        (0, height),
    ]
}

fn comb_concavity(width: i32, height: i32, teeth: usize) -> Vec<(i32, i32)> {
    let mut points = vec![(0, 0), (width, 0), (width, height)];
    let step = (width / (teeth as i32 * 2 + 1)).max(1);
    for tooth in (0..teeth).rev() {
        let x_outer = (2 * tooth as i32 + 2) * step;
        let x_inner = (2 * tooth as i32 + 1) * step;
        points.push((x_outer, height));
        points.push((x_outer, 1));
        points.push((x_inner, 1));
        points.push((x_inner, height));
    }
    points.push((0, height));
    points
}

fn bowtie(size: i32) -> Vec<(i32, i32)> {
    vec![(0, 0), (size, size), (0, size), (size, 0)]
}

fn polygon_case(kind: u8, width: i32, height: i32, offset: i32, teeth: usize) -> PolygonCase {
    let width = width.max(8);
    let height = height.max(8);
    let source_points = match kind % 4 {
        0 => large_concavity(width, height, offset.max(2).min(width.min(height) / 3)),
        1 => slender_concavity(width, height, offset.max(1).min(width - 3), 1),
        2 => comb_concavity(width, height, teeth.clamp(2, 5)),
        _ => {
            let mut points = large_concavity(width, height, 2);
            points.splice(3..3, [(width / 2, height + offset.max(2))]);
            points
        }
    };
    let material = contour_from_points(&source_points);
    let hole = rectangle(
        width / 3,
        2,
        (width / 3 + 2).min(width - 2),
        4.min(height - 2),
    );
    let cutter = curve_region(
        vec![rectangle(
            width / 4,
            -1,
            width + offset.max(2),
            (height / 2).max(3),
        )],
        Vec::new(),
    );

    PolygonCase {
        source_points,
        material,
        holes: vec![hole],
        cutter,
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 96,
        max_shrink_iters: 256,
        ..ProptestConfig::default()
    })]

    #[test]
    fn adversarial_concave_polygons_keep_offset_outputs_finite(
        kind in 0_u8..4,
        width in 12_i32..80,
        height in 12_i32..80,
        offset in 1_i32..24,
        teeth in 2_usize..6,
        distance in 1_i32..4,
    ) {
        let case = polygon_case(kind, width, height, offset, teeth);
        exercise_offsets(&case.material, distance);
        for hole in &case.holes {
            exercise_offsets(hole, -distance);
        }
    }

    #[test]
    fn adversarial_regions_with_holes_support_clipping(
        kind in 0_u8..4,
        width in 16_i32..96,
        height in 16_i32..96,
        offset in 2_i32..32,
        teeth in 2_usize..6,
    ) {
        let case = polygon_case(kind, width, height, offset, teeth);
        let region = curve_region(vec![case.material], case.holes);
        exercise_clipping(&region, &case.cutter);
    }

    #[test]
    fn adversarial_closed_polyline_reconstruction_feeds_topology_safely(
        kind in 0_u8..4,
        width in 12_i32..80,
        height in 12_i32..80,
        offset in 1_i32..24,
        teeth in 2_usize..6,
    ) {
        let case = polygon_case(kind, width, height, offset, teeth);
        exercise_reconstruction(&case.source_points);
    }
}

#[test]
fn self_intersecting_closed_polyline_reconstruction_evidence_contacts() {
    let points = bowtie(12);
    let contour = contour_from_points(&points);

    assert_eq!(
        contour.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
    exercise_reconstruction(&points);
}

#[test]
fn reconstructed_slender_concavity_offsets_through_authoritative_region() {
    let case = polygon_case(1, 60, 12, 1, 2);
    let mut samples: Vec<_> = case.source_points.iter().map(|&(x, y)| p(x, y)).collect();
    samples.insert(1, samples[1].clone());
    samples.push(samples[0].clone());

    let contour =
        Contour2::reconstruct_from_closed_polyline(&samples, reconstruction_options()).unwrap();
    let source = CurveRegion2::try_from_native_material_contours(vec![contour], &policy())
        .unwrap()
        .into_value();
    let offset = source
        .offset(s(1), &OffsetCornerStyle2::Round, &policy())
        .unwrap()
        .into_value();
    assert_region_topology(&offset, "offset");
}

#[test]
fn reconstructed_collapsed_comb_offsets_through_authoritative_region() {
    let case = polygon_case(2, 40, 12, 1, 4);
    exercise_reconstruction(&case.source_points);
}

#[test]
fn polygon_with_hole_cut_through_slender_concavity_stays_structurally_valid() {
    let case = polygon_case(1, 64, 40, 31, 3);
    let region = curve_region(vec![case.material.clone()], case.holes.clone());

    exercise_offsets(&case.material, 1);
    for hole in &case.holes {
        exercise_offsets(hole, -1);
    }
    exercise_clipping(&region, &case.cutter);
    exercise_reconstruction(&case.source_points);
}
