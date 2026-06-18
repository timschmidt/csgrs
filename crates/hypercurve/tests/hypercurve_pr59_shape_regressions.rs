//! Native `hypercurve` coverage for closed-loop shape cases from an upstream
//! shape-boolean regression set.
//!
//! The source cases also covered shape transforms, open polyline clipping,
//! spatial-index refresh, and full shape area accounting. This file keeps only
//! the cases that map directly to `hypercurve`'s current region model: closed
//! material contours, hole contours, and boolean membership semantics.

use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, CurvePolicy, FillRule, Point2, Real,
    Region2, RegionBooleanQueryPath2, RegionPointLocation,
};

type HPoint = Point2;
type HReal = Real;
type HContour = Contour2;
type HRegion = Region2;
type Rect = (f64, f64, f64, f64);

fn s(value: f64) -> HReal {
    HReal::try_from(value).unwrap()
}

fn p(x: f64, y: f64) -> HPoint {
    HPoint::new(s(x), s(y))
}

fn vertex(x: f64, y: f64) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(0.0))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn rectangle((xmin, ymin, xmax, ymax): Rect) -> HContour {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmax, ymin),
        vertex(xmax, ymax),
        vertex(xmin, ymax),
    ])
    .unwrap()
}

fn region(materials: &[Rect], holes: &[Rect]) -> HRegion {
    Region2::new(
        materials.iter().copied().map(rectangle).collect(),
        holes.iter().copied().map(rectangle).collect(),
    )
}

fn inside(region: &HRegion, x: f64, y: f64) -> bool {
    match region.classify_point(&p(x, y), &policy()) {
        Classification::Decided(RegionPointLocation::Inside) => true,
        Classification::Decided(RegionPointLocation::Outside) => false,
        other => panic!("sample ({x}, {y}) should avoid boundaries, got {other:?}"),
    }
}

fn expected_boolean(in_a: bool, in_b: bool, op: BooleanOp) -> bool {
    match op {
        BooleanOp::Union => in_a || in_b,
        BooleanOp::Intersection => in_a && in_b,
        BooleanOp::Difference => in_a && !in_b,
        BooleanOp::Xor => in_a != in_b,
    }
}

fn assert_boolean_samples(
    first: &HRegion,
    second: &HRegion,
    op: BooleanOp,
    expected_materials: usize,
    expected_holes: usize,
    samples: &[(f64, f64)],
) -> HRegion {
    let result = first
        .boolean_region(second, op, FillRule::NonZero, &policy())
        .unwrap();
    let Classification::Decided(result) = result else {
        panic!("expected decided PR #59 region boolean case for {op:?}, got {result:?}");
    };

    assert_eq!(
        result.material_contours().len(),
        expected_materials,
        "material count for {op:?}"
    );
    assert_eq!(
        result.hole_contours().len(),
        expected_holes,
        "hole count for {op:?}"
    );

    for &(x, y) in samples {
        assert_eq!(
            inside(&result, x, y),
            expected_boolean(inside(first, x, y), inside(second, x, y), op),
            "sample ({x}, {y}) differed for {op:?}"
        );
    }

    result
}

#[test]
fn boolean_region_report_retains_boundary_role_assignment() {
    let first = region(&[(0.0, 0.0, 4.0, 4.0)], &[]);
    let second = region(&[(2.0, -1.0, 6.0, 3.0)], &[]);

    let built = first
        .boolean_region_with_report(&second, BooleanOp::Union, FillRule::NonZero, &policy())
        .unwrap();
    let report = built.report();

    assert!(report.status().is_native_exact());
    assert_eq!(report.op(), BooleanOp::Union);
    assert_eq!(report.query_path(), RegionBooleanQueryPath2::Direct);
    assert_eq!(report.first_material_contour_count(), 1);
    assert_eq!(report.first_hole_contour_count(), 0);
    assert_eq!(report.second_material_contour_count(), 1);
    assert_eq!(report.second_hole_contour_count(), 0);
    assert_eq!(report.boundary_candidate_pair_count(), 1);
    assert_eq!(report.boundary_skipped_aabb_pair_count(), 0);
    assert_eq!(report.boundary_tested_pair_count(), 1);
    assert_eq!(report.boundary_intersecting_pair_count(), 1);
    assert_eq!(report.boundary_contour_count(), Some(1));
    assert_eq!(report.result_material_contour_count(), Some(1));
    assert_eq!(report.result_hole_contour_count(), Some(0));
    assert_eq!(report.blocker(), None);

    let boundary_report = report.boundary_build_report().unwrap();
    assert_eq!(boundary_report.source_contour_count(), 1);
    assert_eq!(boundary_report.role_reports().len(), 1);
    assert_eq!(
        boundary_report.role_reports()[0].role(),
        hypercurve::RegionBoundaryContourRole2::Material
    );
    assert!(
        boundary_report.role_reports()[0]
            .containing_contour_indices()
            .is_empty()
    );
    assert_eq!(boundary_report.role_reports()[0].nesting_depth(), 0);

    let result = built.region().unwrap();
    assert!(inside(result, 1.0, 1.0));
    assert!(inside(result, 3.0, 1.0));
    assert!(inside(result, 5.0, 1.0));
    assert!(!inside(result, 7.0, 1.0));
}

#[test]
fn prepared_boolean_region_report_matches_plain_materialization() {
    let first = region(&[(0.0, 0.0, 4.0, 4.0)], &[]);
    let second = region(&[(2.0, -1.0, 6.0, 3.0)], &[]);
    let prepared_second = second.prepare_topology_queries(&policy());

    let built = first
        .as_view()
        .boolean_region_with_report_against_prepared_region(
            &prepared_second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    let plain = first
        .boolean_region_with_report(
            &second,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();

    assert!(built.report().status().is_native_exact());
    assert_eq!(
        built.report().query_path(),
        RegionBooleanQueryPath2::Prepared
    );
    assert_eq!(plain.report().query_path(), RegionBooleanQueryPath2::Direct);
    assert_eq!(built.report().boundary_contour_count(), Some(1));
    assert_eq!(built.report().result_material_contour_count(), Some(1));
    assert_eq!(built.report().result_hole_contour_count(), Some(0));
    assert_eq!(
        built.report().boundary_candidate_pair_count(),
        plain.report().boundary_candidate_pair_count()
    );
    assert_eq!(
        built.report().boundary_skipped_aabb_pair_count(),
        plain.report().boundary_skipped_aabb_pair_count()
    );
    assert_eq!(
        built.report().boundary_tested_pair_count(),
        plain.report().boundary_tested_pair_count()
    );
    assert_eq!(
        built.report().boundary_intersecting_pair_count(),
        plain.report().boundary_intersecting_pair_count()
    );
    assert_eq!(
        built.report().boundary_build_report(),
        plain.report().boundary_build_report()
    );
    assert!(inside(built.region().unwrap(), 3.0, 1.0));
    assert!(!inside(built.region().unwrap(), 1.0, 1.0));
}

#[test]
fn pr59_multi_island_disjoint_identities() {
    let first = region(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)], &[]);
    let second = region(&[(100.0, 0.0, 110.0, 10.0)], &[]);
    let samples = [(5.0, 5.0), (25.0, 5.0), (105.0, 5.0), (50.0, 5.0)];

    assert_boolean_samples(&first, &second, BooleanOp::Union, 3, 0, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Intersection, 0, 0, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Difference, 2, 0, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Xor, 3, 0, &samples);
}

#[test]
fn pr59_bridge_overlaps_two_islands() {
    let islands = region(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)], &[]);
    let bridge = region(&[(5.0, -5.0, 25.0, 15.0)], &[]);
    let samples = [
        (2.5, 5.0),
        (7.5, 5.0),
        (15.0, 0.0),
        (22.5, 5.0),
        (27.5, 5.0),
        (15.0, 12.5),
        (15.0, 20.0),
    ];

    assert_boolean_samples(&islands, &bridge, BooleanOp::Union, 1, 0, &samples);
    assert_boolean_samples(&islands, &bridge, BooleanOp::Intersection, 2, 0, &samples);
    assert_boolean_samples(&islands, &bridge, BooleanOp::Difference, 2, 0, &samples);
}

#[test]
fn pr59_chain_bridge_merges_three_islands() {
    let islands = region(
        &[
            (0.0, 0.0, 6.0, 6.0),
            (10.0, 0.0, 16.0, 6.0),
            (20.0, 0.0, 26.0, 6.0),
        ],
        &[],
    );
    let bridge = region(&[(3.0, -2.0, 23.0, 8.0)], &[]);
    let samples = [
        (1.0, 1.0),
        (5.0, 1.0),
        (8.0, 0.0),
        (13.0, 3.0),
        (18.0, 0.0),
        (24.0, 3.0),
        (12.0, 7.0),
        (30.0, 3.0),
    ];

    assert_boolean_samples(&islands, &bridge, BooleanOp::Union, 1, 0, &samples);
    assert_boolean_samples(&islands, &bridge, BooleanOp::Intersection, 3, 0, &samples);
    assert_boolean_samples(&islands, &bridge, BooleanOp::Difference, 2, 0, &samples);
    assert_boolean_samples(&islands, &bridge, BooleanOp::Xor, 3, 1, &samples);
}

#[test]
fn pr59_ring_difference_adds_second_hole() {
    let ring = region(&[(0.0, 0.0, 10.0, 10.0)], &[(3.0, 3.0, 7.0, 7.0)]);
    let cutter = region(&[(1.0, 1.0, 2.0, 2.0)], &[]);
    let samples = [(1.5, 1.5), (2.5, 2.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_samples(&ring, &cutter, BooleanOp::Difference, 1, 2, &samples);
    assert_boolean_samples(&ring, &cutter, BooleanOp::Intersection, 1, 0, &samples);
    assert_boolean_samples(&ring, &cutter, BooleanOp::Xor, 1, 2, &samples);
}

#[test]
fn pr59_near_coincident_island_in_hole_is_not_cancelled() {
    let moat = 1e-4;
    let donut = region(&[(-10.0, -10.0, 10.0, 10.0)], &[(-5.0, -5.0, 5.0, 5.0)]);
    let island = region(&[(-5.0 + moat, -5.0 + moat, 5.0 - moat, 5.0 - moat)], &[]);
    let samples = [
        (0.0, 0.0),
        (4.99995, 0.0),
        (5.00005, 0.0),
        (8.0, 0.0),
        (11.0, 0.0),
    ];

    assert_boolean_samples(&donut, &island, BooleanOp::Union, 2, 1, &samples);
    assert_boolean_samples(&donut, &island, BooleanOp::Intersection, 0, 0, &samples);
    assert_boolean_samples(&donut, &island, BooleanOp::Difference, 1, 1, &samples);
    assert_boolean_samples(&donut, &island, BooleanOp::Xor, 2, 1, &samples);
}

#[test]
fn pr59_large_coordinate_hole_overlap_keeps_membership() {
    let base = 1_000_000_000.0;
    let first = region(
        &[(base, base, base + 100.0, base + 100.0)],
        &[(base + 20.0, base + 20.0, base + 80.0, base + 80.0)],
    );
    let second = region(
        &[(base + 50.0, base - 10.0, base + 120.0, base + 60.0)],
        &[],
    );
    let samples = [
        (base + 10.0, base + 10.0),
        (base + 40.0, base + 40.0),
        (base + 55.0, base + 10.0),
        (base + 55.0, base + 40.0),
        (base + 90.0, base + 50.0),
        (base + 110.0, base + 50.0),
    ];

    assert_boolean_samples(&first, &second, BooleanOp::Union, 1, 1, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Intersection, 1, 0, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Difference, 1, 0, &samples);
    assert_boolean_samples(&first, &second, BooleanOp::Xor, 3, 0, &samples);
}

#[test]
fn pr59_deep_island_lake_nesting_survives_all_ops() {
    let nested = region(
        &[
            (0.0, 0.0, 100.0, 100.0),
            (20.0, 20.0, 80.0, 80.0),
            (40.0, 40.0, 60.0, 60.0),
        ],
        &[(10.0, 10.0, 90.0, 90.0), (30.0, 30.0, 70.0, 70.0)],
    );
    let rect_in_deep_lake = region(&[(35.0, 35.0, 38.0, 38.0)], &[]);
    let samples = [
        (5.0, 5.0),
        (15.0, 15.0),
        (25.0, 25.0),
        (36.0, 36.0),
        (45.0, 45.0),
        (65.0, 65.0),
        (85.0, 85.0),
        (95.0, 95.0),
    ];

    assert_boolean_samples(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Union,
        4,
        2,
        &samples,
    );
    assert_boolean_samples(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Intersection,
        0,
        0,
        &samples,
    );
    assert_boolean_samples(
        &nested,
        &rect_in_deep_lake,
        BooleanOp::Difference,
        3,
        2,
        &samples,
    );
    assert_boolean_samples(&nested, &rect_in_deep_lake, BooleanOp::Xor, 4, 2, &samples);
}
