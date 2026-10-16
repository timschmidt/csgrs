use super::*;

#[test]
fn one_pcurve_shared_by_planar_and_axial_charts_is_separated_on_scaling() {
    let (cylinder, _) = crate::construction::cylinder(Real::one(), Real::one()).unwrap();
    let width = (Real::pi() / Real::from(2)).unwrap();
    let points = [
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(width.clone(), Real::zero()),
        CurvePoint2::new(width, Real::one()),
        CurvePoint2::new(Real::zero(), Real::one()),
    ];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let (plane, _) = crate::construction::planar_face(
        &path,
        &[],
        Point3::new(20.into(), 0.into(), 0.into()),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap();
    let mut builder = ArenaModelBuilder::new();
    cylinder.append_to_builder(&mut builder).unwrap();
    plane.append_to_builder(&mut builder).unwrap();
    let mut combined = builder.finish().unwrap();
    let mut target = None;
    let mut source = None;
    for (id, edge_use) in combined.edge_uses() {
        let Some(line) = combined.pcurve(edge_use.pcurve()).unwrap().line_segment() else {
            continue;
        };
        if line.start() != &points[3] || line.end() != &points[0] {
            continue;
        }
        let face = combined.data.wire_face[combined.data.edge_use_wire[id.index()].index()];
        let kind = combined.data.surfaces[combined.data.faces[face.index()].surface.index()].kind();
        if kind == SurfaceKind::Cylinder {
            source = Some(edge_use.pcurve());
        }
        if kind == SurfaceKind::Plane && combined.data.face_shell[face.index()].index() == 1 {
            target = Some(id);
        }
    }
    let data = Arc::make_mut(&mut combined.data);
    data.edge_uses[target.unwrap().index()].pcurve = source.unwrap();
    invalidate_model_certificates(data);
    let combined = combined.to_raw().unwrap().validate().unwrap();
    for reflected in [false, true] {
        let map = Matrix4::affine_nonuniform_scale([
            Real::from(if reflected { -2 } else { 2 }),
            Real::from(2),
            Real::from(2),
        ]);
        let moved = combined.transformed(&map).unwrap();
        assert_eq!(moved.counts().pcurves, combined.counts().pcurves + 1);
        let json = moved.to_json().unwrap();
        let replay = crate::ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(replay.to_json().unwrap(), json);
        let solid = replay.solids().next().unwrap().0;
        assert_eq!(
            compare_reals(
                &replay.solid_volume(solid).unwrap(),
                &(Real::from(8) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let face = replay.faces().last().unwrap().0;
        assert_eq!(
            compare_reals(
                &replay.face_area(face).unwrap(),
                &(Real::from(2) * Real::pi()),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
}
