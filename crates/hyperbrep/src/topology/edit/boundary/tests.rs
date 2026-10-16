use super::*;

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn path(points: &[(i32, i32)]) -> CurvePath2 {
    CurvePath2::try_new(
        (0..points.len())
            .map(|i| {
                let a = points[i];
                let b = points[(i + 1) % points.len()];
                Curve2::from(
                    LineSeg2::try_new(
                        CurvePoint2::new(a.0.into(), a.1.into()),
                        CurvePoint2::new(b.0.into(), b.1.into()),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}
fn fixture() -> (ArenaModel, FaceId) {
    crate::construction::planar_face(
        &path(&[(0, 0), (8, 0), (8, 8), (0, 8)]),
        &[path(&[(2, 2), (2, 4), (4, 4), (4, 2)])],
        p(0, 0, 0),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap()
}
fn trace(model: &ArenaModel, face: FaceId, start: Point3, end: Point3) -> SurfaceIntersectionCurve {
    SurfaceIntersectionCurve::on_plane(
        Curve3::line(start, end).unwrap(),
        model.surface(model.face(face).unwrap().surface()).unwrap(),
    )
    .unwrap()
}

#[test]
fn complete_boundary_coverage_retains_subdivision_direction_and_holes() {
    let (source, face) = fixture();
    let edge = source
        .edge_use(
            source
                .wire(source.face(face).unwrap().outer().unwrap())
                .unwrap()
                .edge_uses()[0],
        )
        .unwrap()
        .edge();
    let (model, _) = source
        .split_edge(edge, ((Real::one() / Real::from(2)).unwrap()).into())
        .unwrap();
    let original = model.to_json().unwrap();
    for (start, end, count) in [
        (p(0, 0, 0), p(8, 0, 0), 2),
        (p(8, 0, 0), p(0, 0, 0), 2),
        (p(1, 0, 0), p(3, 0, 0), 1),
        (p(2, 2, 0), p(2, 4, 0), 1),
    ] {
        let trace = trace(&model, face, start, end);
        let (unchanged, partition) = model
            .split_face_by_surface_curves(
                face,
                std::slice::from_ref(&trace),
                SurfaceIntersectionOperand::First,
            )
            .unwrap();
        assert_eq!(partition.faces, vec![face]);
        assert_eq!(unchanged.to_json().unwrap(), original);
        let record = &partition.traces[0];
        assert!(record.splits.is_empty());
        assert_eq!(record.segments.len(), 1);
        let boundary = record.boundary.as_ref().unwrap();
        assert_eq!(boundary.source.to_json().unwrap(), original);
        assert_eq!(boundary.overlaps.len(), count);
        assert_eq!(
            boundary.overlaps.first().unwrap().trace_range.start(),
            &Real::zero()
        );
        assert_eq!(
            boundary.overlaps.last().unwrap().trace_range.end(),
            &Real::one()
        );
        for overlap in &boundary.overlaps {
            let use_ = boundary.source.edge_use(overlap.edge_use).unwrap();
            let edge_curve = boundary.source.pcurve(use_.pcurve()).unwrap();
            let materialized = boundary.intersection.first_pcurve().materialize().unwrap();
            for (a, b) in [
                (overlap.trace_range.start(), overlap.boundary_range.start()),
                (overlap.trace_range.end(), overlap.boundary_range.end()),
            ] {
                let actual = materialized.curve().point_at(&a.clone().into()).unwrap();
                let expected = edge_curve.curve().point_at(&b.clone().into()).unwrap();
                assert_eq!(actual, expected);
            }
        }
        assert!(matches!(
            model.split_face_by_surface_curve(face, &trace, SurfaceIntersectionOperand::First),
            Err(TopologyEditError::DegenerateFaceSplit)
        ));
    }
}

#[test]
fn partial_coverage_and_near_offsets_do_not_become_boundary_proofs() {
    let (model, face) = fixture();
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    for (a, b) in [
        (p(-1, 0, 0), p(9, 0, 0)),
        (p(0, 2, 0), p(8, 2, 0)),
        (
            Point3::new(0.into(), tiny.clone(), 0.into()),
            Point3::new(8.into(), tiny, 0.into()),
        ),
    ] {
        let trace = trace(&model, face, a, b);
        assert!(
            model
                .retained_surface_trace_on_boundary(face, &trace, SurfaceIntersectionOperand::First)
                .unwrap()
                .is_none()
        );
    }
    assert_eq!(model.face_area(face).unwrap(), Real::from(60));
}

#[test]
fn boundary_endpoint_agreement_does_not_certify_a_forged_spatial_image() {
    let (model, face) = fixture();
    let original = model.to_json().unwrap();
    let pcurve = Curve2::from(
        LineSeg2::try_new(
            CurvePoint2::new(0.into(), 0.into()),
            CurvePoint2::new(8.into(), 0.into()),
        )
        .unwrap(),
    );
    let forged = SurfaceIntersectionCurve::from_exact_pcurves(
        Curve3::rational_bezier(
            vec![p(0, 0, 0), p(4, 0, 1), p(8, 0, 0)],
            vec![Real::one(); 3],
        )
        .unwrap(),
        pcurve.clone(),
        pcurve,
    )
    .unwrap();
    assert!(matches!(
        model.split_face_by_surface_curves(face, &[forged], SurfaceIntersectionOperand::First),
        Err(TopologyEditError::Build(
            BuildError::UnsupportedEdgeUseAgreement { .. }
        ))
    ));
    assert_eq!(model.to_json().unwrap(), original);
    let valid = trace(&model, face, p(0, 0, 0), p(8, 0, 0));
    assert!(
        model
            .split_face_by_surface_curves(face, &[valid], SurfaceIntersectionOperand::First)
            .is_ok()
    );
}
