use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurveError, CurvePolicy, CurveString2, FillRule,
    PlanarPcurveImageRelation2, Point2, Real, RetainedPlanarFace2,
    RetainedPlanarFacePointLocation2, RetainedPlanarPcurve2, RetainedPlanarSurfaceIdentity2,
    RetainedPlanarTrimLoop2,
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

fn open_curve(points: &[(i32, i32)]) -> CurveString2 {
    let vertices = points
        .iter()
        .map(|&(x, y)| vertex(x, y))
        .collect::<Vec<_>>();
    CurveString2::from_bulge_vertices(&vertices).unwrap()
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

fn trim(surface: RetainedPlanarSurfaceIdentity2, points: &[(i32, i32)]) -> RetainedPlanarTrimLoop2 {
    RetainedPlanarTrimLoop2::new(surface, rectangle(points, FillRule::NonZero))
}

#[test]
fn planar_open_pcurve_equality_reports_directed_and_reversed_images() {
    let surface = RetainedPlanarSurfaceIdentity2::new(7);
    let directed = RetainedPlanarPcurve2::new(surface, open_curve(&[(0, 0), (2, 0), (2, 3)]));
    let same = RetainedPlanarPcurve2::new(surface, open_curve(&[(0, 0), (2, 0), (2, 3)]));
    let reversed = RetainedPlanarPcurve2::new(surface, open_curve(&[(2, 3), (2, 0), (0, 0)]));

    let same_report = directed.image_equality_report(&same);
    assert_eq!(
        same_report.relation(),
        PlanarPcurveImageRelation2::SameDirected
    );
    assert_eq!(same_report.surface(), Some(surface));
    assert_eq!(same_report.segment_count(), 2);

    let reversed_report = directed.image_equality_report(&reversed);
    assert_eq!(
        reversed_report.relation(),
        PlanarPcurveImageRelation2::SameReversed
    );
    assert!(reversed_report.relation().is_same_image());
    assert!(reversed_report.relation().is_reversed());
}

#[test]
fn planar_pcurve_equality_blocks_surface_mismatch_before_uv_match() {
    let first = RetainedPlanarPcurve2::new(
        RetainedPlanarSurfaceIdentity2::new(1),
        open_curve(&[(0, 0), (1, 0)]),
    );
    let second = RetainedPlanarPcurve2::new(
        RetainedPlanarSurfaceIdentity2::new(2),
        open_curve(&[(0, 0), (1, 0)]),
    );

    let report = first.image_equality_report(&second);
    assert_eq!(
        report.relation(),
        PlanarPcurveImageRelation2::SurfaceMismatch
    );
    assert_eq!(report.surface(), None);
    assert_eq!(report.segment_count(), 0);
}

#[test]
fn planar_trim_loop_equality_accepts_cyclic_rotation_and_ignores_fill_rule() {
    let surface = RetainedPlanarSurfaceIdentity2::new(9);
    let first = RetainedPlanarTrimLoop2::new(
        surface,
        rectangle(&[(0, 0), (4, 0), (4, 3), (0, 3)], FillRule::NonZero),
    );
    let rotated = RetainedPlanarTrimLoop2::new(
        surface,
        rectangle(&[(4, 3), (0, 3), (0, 0), (4, 0)], FillRule::EvenOdd),
    );

    let report = first.image_equality_report(&rotated);
    assert_eq!(report.relation(), PlanarPcurveImageRelation2::SameDirected);
    assert_eq!(report.segment_count(), 4);
}

#[test]
fn planar_trim_loop_equality_reports_reversed_and_different_images() {
    let surface = RetainedPlanarSurfaceIdentity2::new(11);
    let first = RetainedPlanarTrimLoop2::new(
        surface,
        rectangle(&[(0, 0), (4, 0), (4, 3), (0, 3)], FillRule::NonZero),
    );
    let reversed = RetainedPlanarTrimLoop2::new(
        surface,
        rectangle(&[(0, 3), (4, 3), (4, 0), (0, 0)], FillRule::NonZero),
    );
    let different = RetainedPlanarTrimLoop2::new(
        surface,
        rectangle(&[(0, 0), (5, 0), (5, 3), (0, 3)], FillRule::NonZero),
    );

    assert_eq!(
        first.image_equality_report(&reversed).relation(),
        PlanarPcurveImageRelation2::SameReversed
    );
    assert_eq!(
        first.image_equality_report(&different).relation(),
        PlanarPcurveImageRelation2::Different
    );
}

#[test]
fn retained_planar_face_classifies_uv_points_against_material_and_holes() {
    let surface = RetainedPlanarSurfaceIdentity2::new(21);
    let face = RetainedPlanarFace2::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![trim(surface, &[(3, 3), (7, 3), (7, 7), (3, 7)])],
    )
    .unwrap();

    let inside = decided(
        face.classify_uv_point(surface, &p(1, 1), &policy())
            .unwrap(),
    );
    assert_eq!(inside.location(), RetainedPlanarFacePointLocation2::Inside);
    assert_eq!(inside.surface(), Some(surface));
    assert_eq!(inside.material_loop_count(), 1);
    assert_eq!(inside.hole_loop_count(), 1);

    let in_hole = decided(
        face.classify_uv_point(surface, &p(5, 5), &policy())
            .unwrap(),
    );
    assert_eq!(
        in_hole.location(),
        RetainedPlanarFacePointLocation2::Outside
    );
    assert!(in_hole.location().is_trim_classification());

    let boundary = decided(
        face.classify_uv_point(surface, &p(3, 5), &policy())
            .unwrap(),
    );
    assert_eq!(
        boundary.location(),
        RetainedPlanarFacePointLocation2::Boundary
    );

    let outside = decided(
        face.classify_uv_point(surface, &p(12, 5), &policy())
            .unwrap(),
    );
    assert_eq!(
        outside.location(),
        RetainedPlanarFacePointLocation2::Outside
    );
}

#[test]
fn retained_planar_face_reports_surface_mismatch_before_trim_classification() {
    let surface = RetainedPlanarSurfaceIdentity2::new(31);
    let face = RetainedPlanarFace2::try_new(
        surface,
        vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
        Vec::new(),
    )
    .unwrap();

    let report = decided(
        face.classify_uv_point(RetainedPlanarSurfaceIdentity2::new(32), &p(1, 1), &policy())
            .unwrap(),
    );
    assert_eq!(
        report.location(),
        RetainedPlanarFacePointLocation2::SurfaceMismatch
    );
    assert_eq!(report.surface(), None);
    assert!(!report.location().is_trim_classification());
}

#[test]
fn retained_planar_face_rejects_missing_material_or_mixed_surface_trims() {
    let surface = RetainedPlanarSurfaceIdentity2::new(41);
    let other = RetainedPlanarSurfaceIdentity2::new(42);
    assert_eq!(
        RetainedPlanarFace2::try_new(surface, Vec::new(), Vec::new()).unwrap_err(),
        CurveError::InvalidPlanarFace
    );
    assert_eq!(
        RetainedPlanarFace2::try_new(
            surface,
            vec![trim(surface, &[(0, 0), (4, 0), (4, 4), (0, 4)])],
            vec![trim(other, &[(1, 1), (2, 1), (2, 2), (1, 2)])],
        )
        .unwrap_err(),
        CurveError::InvalidPlanarFace
    );
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}
