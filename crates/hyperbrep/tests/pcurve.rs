use hyperbrep::{
    BrepPcurve, BrepPcurveImageEqualityReport, BrepPcurveImageRelation, BrepPlanarError,
    BrepPlanarFaceEdgeUseRelation, BrepPlanarFaceEdgeUseReport, BrepPlanarFacePointLocation,
    BrepPlanarFacePointReport, BrepPlanarFaceRegion, BrepPlanarResult, BrepPlanarTrimLoop,
    BrepPlanarTrimLoopRole, BrepSurfaceId,
};
use hypercurve::{
    BulgeVertex2, Classification, Contour2, Curve2, CurveGeometry2, CurvePath2, CurvePolicy,
    CurveString2, FillRule, Point2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), r(0))
}

fn open_curve(points: &[(i32, i32)]) -> CurvePath2 {
    let vertices = points
        .iter()
        .map(|&(x, y)| vertex(x, y))
        .collect::<Vec<_>>();
    let curve = CurveString2::from_bulge_vertices(&vertices).unwrap();
    CurvePath2::try_new(
        curve
            .segments()
            .iter()
            .cloned()
            .map(|segment| match segment {
                hypercurve::Segment2::Line(line) => Curve2::new(CurveGeometry2::Line(line)),
                hypercurve::Segment2::Arc(arc) => Curve2::new(CurveGeometry2::CircularArc(arc)),
            })
            .collect(),
    )
    .unwrap()
}

fn rectangle(points: &[(i32, i32)], fill_rule: FillRule) -> Contour2 {
    let vertices = points
        .iter()
        .map(|&(x, y)| vertex(x, y))
        .collect::<Vec<_>>();
    Contour2::from_bulge_vertices_with_fill_rule(&vertices, fill_rule).unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn trim(surface: BrepSurfaceId, points: &[(i32, i32)]) -> BrepPlanarTrimLoop {
    BrepPlanarTrimLoop::new(surface, rectangle(points, FillRule::NonZero))
}

fn assert_topology_error<T>(result: BrepPlanarResult<T>) {
    assert!(matches!(result, Err(BrepPlanarError::InvalidReport(_))));
}

#[test]
fn planar_open_pcurve_equality_reports_directed_and_reversed_images() {
    let surface = BrepSurfaceId::new(7);
    let directed = BrepPcurve::new(surface, open_curve(&[(0, 0), (2, 0), (2, 3)]));
    let same = BrepPcurve::new(surface, open_curve(&[(0, 0), (2, 0), (2, 3)]));
    let reversed = BrepPcurve::new(surface, open_curve(&[(2, 3), (2, 0), (0, 0)]));

    let same_report = directed.image_equality_report(&same).unwrap();
    assert_eq!(
        same_report.relation(),
        BrepPcurveImageRelation::SameDirected
    );
    assert_eq!(same_report.surface(), Some(surface));
    assert_eq!(same_report.curve_count(), 2);

    let reversed_report = directed.image_equality_report(&reversed).unwrap();
    assert_eq!(
        reversed_report.relation(),
        BrepPcurveImageRelation::SameReversed
    );
    assert!(reversed_report.relation().is_same_image());
    assert!(reversed_report.relation().is_reversed());
}

#[test]
fn planar_pcurve_equality_blocks_surface_mismatch_before_uv_match() {
    let first = BrepPcurve::new(BrepSurfaceId::new(1), open_curve(&[(0, 0), (1, 0)]));
    let second = BrepPcurve::new(BrepSurfaceId::new(2), open_curve(&[(0, 0), (1, 0)]));

    let report = first.image_equality_report(&second).unwrap();
    assert_eq!(report.relation(), BrepPcurveImageRelation::SurfaceMismatch);
    assert_eq!(report.surface(), None);
    assert_eq!(report.curve_count(), 0);
}

#[test]
fn planar_pcurve_owns_top_level_nurbs_and_reports_native_trim_limit() {
    let surface = BrepSurfaceId::new(3);
    let curve = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(1, 2), p(3, 0)],
        vec![r(1), r(2), r(1)],
        vec![r(0), r(0), r(0), r(1), r(1), r(1)],
        None,
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![curve]).unwrap();
    let directed = BrepPcurve::new(surface, path.clone());
    let reversed = BrepPcurve::new(surface, path.reversed().unwrap());

    assert_eq!(directed.path().curves().len(), 1);
    assert!(!reversed.is_reversed_path_cached());
    assert_eq!(
        directed
            .image_equality_report(&reversed)
            .unwrap()
            .relation(),
        BrepPcurveImageRelation::SameReversed
    );
    assert!(reversed.is_reversed_path_cached());

    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();
    assert_eq!(
        face.edge_use_report(&directed).unwrap().relation(),
        BrepPlanarFaceEdgeUseRelation::UnsupportedCurveFamily
    );
    assert!(directed.is_native_segment_view_cached());
}

#[test]
fn planar_pcurve_image_report_constructor_rejects_inconsistent_evidence() {
    let surface = BrepSurfaceId::new(5);

    assert_topology_error(BrepPcurveImageEqualityReport::new(
        BrepPcurveImageRelation::SameDirected,
        Some(surface),
        0,
    ));
    assert_topology_error(BrepPcurveImageEqualityReport::new(
        BrepPcurveImageRelation::Different,
        None,
        0,
    ));
    assert_topology_error(BrepPcurveImageEqualityReport::new(
        BrepPcurveImageRelation::SurfaceMismatch,
        Some(surface),
        1,
    ));
}

#[test]
fn planar_trim_loop_equality_accepts_cyclic_rotation_and_ignores_fill_rule() {
    let surface = BrepSurfaceId::new(9);
    let first = BrepPlanarTrimLoop::new(
        surface,
        rectangle(&[(0, 0), (4, 0), (4, 3), (0, 3)], FillRule::NonZero),
    );
    let rotated = BrepPlanarTrimLoop::new(
        surface,
        rectangle(&[(4, 3), (0, 3), (0, 0), (4, 0)], FillRule::EvenOdd),
    );

    let report = first.image_equality_report(&rotated).unwrap();
    assert_eq!(report.relation(), BrepPcurveImageRelation::SameDirected);
    assert_eq!(report.curve_count(), 4);
}

#[test]
fn planar_trim_loop_equality_reports_reversed_and_different_images() {
    let surface = BrepSurfaceId::new(11);
    let first = BrepPlanarTrimLoop::new(
        surface,
        rectangle(&[(0, 0), (4, 0), (4, 3), (0, 3)], FillRule::NonZero),
    );
    let reversed = BrepPlanarTrimLoop::new(
        surface,
        rectangle(&[(0, 3), (4, 3), (4, 0), (0, 0)], FillRule::NonZero),
    );
    let different = BrepPlanarTrimLoop::new(
        surface,
        rectangle(&[(0, 0), (5, 0), (5, 3), (0, 3)], FillRule::NonZero),
    );

    assert_eq!(
        first.image_equality_report(&reversed).unwrap().relation(),
        BrepPcurveImageRelation::SameReversed
    );
    assert_eq!(
        first.image_equality_report(&different).unwrap().relation(),
        BrepPcurveImageRelation::Different
    );
}

#[test]
fn retained_planar_face_classifies_uv_points_against_material_and_holes() {
    let surface = BrepSurfaceId::new(21);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![trim(surface, &[(3, 3), (7, 3), (7, 7), (3, 7)])],
    )
    .unwrap();

    let inside = decided(
        face.classify_uv_point(surface, &p(1, 1), &policy())
            .unwrap(),
    );
    assert_eq!(inside.location(), BrepPlanarFacePointLocation::Inside);
    assert_eq!(inside.surface(), Some(surface));
    assert_eq!(inside.material_loop_count(), 1);
    assert_eq!(inside.hole_loop_count(), 1);

    let in_hole = decided(
        face.classify_uv_point(surface, &p(5, 5), &policy())
            .unwrap(),
    );
    assert_eq!(in_hole.location(), BrepPlanarFacePointLocation::Outside);
    assert!(in_hole.location().is_trim_classification());

    let boundary = decided(
        face.classify_uv_point(surface, &p(3, 5), &policy())
            .unwrap(),
    );
    assert_eq!(boundary.location(), BrepPlanarFacePointLocation::Boundary);

    let outside = decided(
        face.classify_uv_point(surface, &p(12, 5), &policy())
            .unwrap(),
    );
    assert_eq!(outside.location(), BrepPlanarFacePointLocation::Outside);
}

#[test]
fn retained_planar_face_reports_surface_mismatch_before_trim_classification() {
    let surface = BrepSurfaceId::new(31);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();

    let report = decided(
        face.classify_uv_point(BrepSurfaceId::new(32), &p(1, 1), &policy())
            .unwrap(),
    );
    assert_eq!(
        report.location(),
        BrepPlanarFacePointLocation::SurfaceMismatch
    );
    assert_eq!(report.surface(), None);
    assert!(!report.location().is_trim_classification());
}

#[test]
fn retained_planar_face_point_report_constructor_rejects_inconsistent_evidence() {
    let surface = BrepSurfaceId::new(45);

    assert_topology_error(BrepPlanarFacePointReport::new(
        BrepPlanarFacePointLocation::Inside,
        None,
        1,
        0,
    ));
    assert_topology_error(BrepPlanarFacePointReport::new(
        BrepPlanarFacePointLocation::SurfaceMismatch,
        Some(surface),
        1,
        0,
    ));
    assert_topology_error(BrepPlanarFacePointReport::new(
        BrepPlanarFacePointLocation::Outside,
        Some(surface),
        0,
        0,
    ));
}

#[test]
fn prepared_retained_planar_face_matches_plain_uv_classification() {
    let surface = BrepSurfaceId::new(51);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![trim(surface, &[(3, 3), (7, 3), (7, 7), (3, 7)])],
    )
    .unwrap();
    let policy = policy();
    let prepared = face.prepare_point_queries(&policy);

    assert_eq!(prepared.face(), &face);
    assert_eq!(prepared.surface(), surface);
    assert_eq!(prepared.material_loop_count(), 1);
    assert_eq!(prepared.hole_loop_count(), 1);
    assert_eq!(prepared.prepared_region().material_contours().len(), 1);
    assert_eq!(prepared.prepared_region().hole_contours().len(), 1);
    assert_eq!(face.prepare_topology_queries(&policy).surface(), surface);

    for point in [p(1, 1), p(5, 5), p(3, 5), p(12, 5)] {
        let plain = face.classify_uv_point(surface, &point, &policy).unwrap();
        let prepared = prepared
            .classify_uv_point(surface, &point, &policy)
            .unwrap();
        assert_eq!(prepared, plain, "prepared query diverged at {point:?}");
    }
}

#[test]
fn prepared_retained_planar_face_blocks_surface_mismatch_before_cached_region() {
    let surface = BrepSurfaceId::new(61);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();
    let policy = policy();
    let prepared = face.prepare_point_queries(&policy);

    let report = decided(
        prepared
            .classify_uv_point(BrepSurfaceId::new(62), &p(1, 1), &policy)
            .unwrap(),
    );
    assert_eq!(
        report.location(),
        BrepPlanarFacePointLocation::SurfaceMismatch
    );
    assert_eq!(report.surface(), None);
    assert_eq!(report.material_loop_count(), 1);
    assert_eq!(report.hole_loop_count(), 0);
    assert!(!report.location().is_trim_classification());
}

#[test]
fn retained_planar_face_reports_material_and_hole_edge_uses() {
    let surface = BrepSurfaceId::new(71);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![trim(surface, &[(3, 3), (7, 3), (7, 7), (3, 7)])],
    )
    .unwrap();

    let material = BrepPcurve::new(surface, open_curve(&[(10, 0), (10, 10)]));
    let material_report = face.edge_use_report(&material).unwrap();
    assert_eq!(
        material_report.relation(),
        BrepPlanarFaceEdgeUseRelation::BoundarySameDirected
    );
    assert!(material_report.relation().is_boundary());
    assert!(!material_report.relation().is_reversed());
    assert_eq!(material_report.surface(), Some(surface));
    assert_eq!(
        material_report.trim_role(),
        Some(BrepPlanarTrimLoopRole::Material)
    );
    assert_eq!(material_report.trim_loop_index(), Some(0));
    assert_eq!(material_report.trim_segment_index(), Some(1));
    assert_eq!(material_report.segment_count(), 1);

    let reversed_hole = BrepPcurve::new(surface, open_curve(&[(7, 7), (7, 3)]));
    let hole_report = face.edge_use_report(&reversed_hole).unwrap();
    assert_eq!(
        hole_report.relation(),
        BrepPlanarFaceEdgeUseRelation::BoundarySameReversed
    );
    assert!(hole_report.relation().is_reversed());
    assert_eq!(hole_report.trim_role(), Some(BrepPlanarTrimLoopRole::Hole));
    assert!(hole_report.trim_role().unwrap().is_hole());
    assert_eq!(hole_report.trim_loop_index(), Some(0));
    assert_eq!(hole_report.trim_segment_index(), Some(1));
}

#[test]
fn retained_planar_face_edge_use_accepts_cyclic_multisegment_subchains() {
    let surface = BrepSurfaceId::new(81);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();

    let wraps_closure = BrepPcurve::new(surface, open_curve(&[(0, 4), (0, 0), (4, 0)]));
    let report = face.edge_use_report(&wraps_closure).unwrap();
    assert_eq!(
        report.relation(),
        BrepPlanarFaceEdgeUseRelation::BoundarySameDirected
    );
    assert_eq!(report.trim_segment_index(), Some(3));
    assert_eq!(report.segment_count(), 2);

    let reversed_wrap = BrepPcurve::new(surface, open_curve(&[(4, 0), (0, 0), (0, 4)]));
    let report = face.edge_use_report(&reversed_wrap).unwrap();
    assert_eq!(
        report.relation(),
        BrepPlanarFaceEdgeUseRelation::BoundarySameReversed
    );
    assert_eq!(report.trim_segment_index(), Some(0));
    assert_eq!(report.segment_count(), 2);
}

#[test]
fn retained_planar_face_edge_use_rejects_surface_mismatch_and_nonboundary_chords() {
    let surface = BrepSurfaceId::new(91);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();

    let wrong_surface = BrepPcurve::new(BrepSurfaceId::new(92), open_curve(&[(0, 0), (4, 0)]));
    let report = face.edge_use_report(&wrong_surface).unwrap();
    assert_eq!(
        report.relation(),
        BrepPlanarFaceEdgeUseRelation::SurfaceMismatch
    );
    assert_eq!(report.surface(), None);
    assert_eq!(report.segment_count(), 0);

    let diagonal = BrepPcurve::new(surface, open_curve(&[(0, 0), (4, 4)]));
    let report = face.edge_use_report(&diagonal).unwrap();
    assert_eq!(
        report.relation(),
        BrepPlanarFaceEdgeUseRelation::NotTrimBoundary
    );
    assert_eq!(report.surface(), Some(surface));
    assert_eq!(report.trim_role(), None);
    assert_eq!(report.segment_count(), 0);
}

#[test]
fn retained_planar_face_edge_use_report_constructor_rejects_inconsistent_evidence() {
    let surface = BrepSurfaceId::new(95);

    assert_topology_error(BrepPlanarFaceEdgeUseReport::new(
        BrepPlanarFaceEdgeUseRelation::BoundarySameDirected,
        Some(surface),
        Some(BrepPlanarTrimLoopRole::Material),
        Some(0),
        Some(0),
        0,
    ));
    assert_topology_error(BrepPlanarFaceEdgeUseReport::new(
        BrepPlanarFaceEdgeUseRelation::BoundarySameDirected,
        Some(surface),
        Some(BrepPlanarTrimLoopRole::Material),
        Some(0),
        Some(0),
        1,
    ));
    assert_topology_error(BrepPlanarFaceEdgeUseReport::new(
        BrepPlanarFaceEdgeUseRelation::NotTrimBoundary,
        Some(surface),
        Some(BrepPlanarTrimLoopRole::Material),
        Some(0),
        Some(0),
        1,
    ));
    assert_topology_error(BrepPlanarFaceEdgeUseReport::new(
        BrepPlanarFaceEdgeUseRelation::SurfaceMismatch,
        Some(surface),
        None,
        None,
        None,
        0,
    ));
}

#[test]
fn prepared_retained_planar_face_edge_use_matches_plain_report() {
    let surface = BrepSurfaceId::new(101);
    let face = BrepPlanarFaceRegion::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (6, 0), (6, 6), (0, 6)])],
        vec![trim(surface, &[(2, 2), (4, 2), (4, 4), (2, 4)])],
    )
    .unwrap();
    let policy = policy();
    let prepared = face.prepare_topology_queries(&policy);
    let queries = [
        BrepPcurve::new(surface, open_curve(&[(0, 0), (6, 0)])),
        BrepPcurve::new(surface, open_curve(&[(4, 4), (4, 2)])),
        BrepPcurve::new(surface, open_curve(&[(0, 0), (6, 6)])),
        BrepPcurve::new(BrepSurfaceId::new(102), open_curve(&[(0, 0), (6, 0)])),
    ];

    for query in &queries {
        assert_eq!(
            prepared.edge_use_report(query).unwrap(),
            face.edge_use_report(query).unwrap(),
            "prepared edge-use report diverged for {query:?}"
        );
    }
}

#[test]
fn retained_planar_face_rejects_missing_material_or_mixed_surface_trims() {
    let surface = BrepSurfaceId::new(41);
    let other = BrepSurfaceId::new(42);
    assert_eq!(
        BrepPlanarFaceRegion::try_new(surface, Vec::new(), Vec::new()).unwrap_err(),
        BrepPlanarError::MissingMaterialLoop
    );
    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
            vec![trim(other, &[(1, 1), (2, 1), (2, 2), (1, 2)])],
        )
        .unwrap_err(),
        BrepPlanarError::SurfaceMismatch {
            expected: surface,
            actual: other,
        }
    );
    let material = trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)]);
    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![material.clone(), material.clone()],
            Vec::new(),
        )
        .unwrap_err(),
        BrepPlanarError::DuplicateTrimLoop
    );
    assert_eq!(
        BrepPlanarFaceRegion::try_new(surface, vec![material.clone()], vec![material]).unwrap_err(),
        BrepPlanarError::DuplicateTrimLoop
    );
}

#[test]
fn retained_planar_face_rejects_self_contacting_trims() {
    let surface = BrepSurfaceId::new(51);

    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![trim(surface, &[(0, 0), (4, 4), (0, 4), (4, 0)])],
            Vec::new(),
        )
        .unwrap_err(),
        BrepPlanarError::SelfContactingTrimLoop
    );

    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![trim(surface, &[(0, 0), (8, 0), (8, 8), (0, 8)])],
            vec![trim(surface, &[(2, 2), (6, 6), (2, 6), (6, 2)])],
        )
        .unwrap_err(),
        BrepPlanarError::SelfContactingTrimLoop
    );
}

#[test]
fn retained_planar_face_rejects_unowned_or_crossing_holes() {
    let surface = BrepSurfaceId::new(111);
    let material = trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)]);

    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![material.clone()],
            vec![trim(surface, &[(5, 5), (6, 5), (6, 6), (5, 6)])],
        )
        .unwrap_err(),
        BrepPlanarError::UnownedHole
    );
    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![material],
            vec![trim(surface, &[(2, 2), (5, 2), (5, 3), (2, 3)])],
        )
        .unwrap_err(),
        BrepPlanarError::IntersectingTrimLoops
    );
}

#[test]
fn retained_planar_face_rejects_crossing_same_role_trims() {
    let surface = BrepSurfaceId::new(121);

    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![
                trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)]),
                trim(surface, &[(2, 2), (6, 2), (6, 6), (2, 6)]),
            ],
            Vec::new(),
        )
        .unwrap_err(),
        BrepPlanarError::IntersectingTrimLoops
    );

    assert_eq!(
        BrepPlanarFaceRegion::try_new(
            surface,
            vec![trim(surface, &[(0, 0), (10, 0), (10, 10), (0, 10)])],
            vec![
                trim(surface, &[(2, 2), (6, 2), (6, 6), (2, 6)]),
                trim(surface, &[(4, 4), (8, 4), (8, 8), (4, 8)]),
            ],
        )
        .unwrap_err(),
        BrepPlanarError::IntersectingTrimLoops
    );
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}
