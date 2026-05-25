use hypercurve::{
    BulgeVertex2, Contour2, CurveString2, FillRule, PlanarPcurveImageRelation2, Point2, Real,
    RetainedPlanarPcurve2, RetainedPlanarSurfaceIdentity2, RetainedPlanarTrimLoop2,
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
