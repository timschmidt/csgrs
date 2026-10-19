//! Independent adversarial controls for the complete image dispatch.
use super::*;

fn circle_image(
    surface: Surface,
    center: Point3,
    height: Real,
    radial: Vector3,
) -> Result<(), BuildError> {
    let mut builder = ArenaModelBuilder::new();
    let point = center.clone() + radial.clone();
    let vertex = builder.vertex(point.clone().into()).unwrap();
    let curve = builder
        .curve(
            Curve3::circle_arc(
                center,
                radial,
                Vector3::y(),
                Real::one(),
                Real::zero(),
                Real::tau(),
            )
            .unwrap(),
        )
        .unwrap();
    let edge = builder
        .edge(
            vertex,
            vertex,
            curve,
            ParameterDomain::new(Real::zero(), Real::tau())
                .unwrap()
                .into(),
        )
        .unwrap();
    let pcurve = builder
        .pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::zero(), height.clone()),
                CurvePoint2::new(Real::tau(), height.clone()),
            )
            .unwrap(),
        )))
        .unwrap();
    let edge_use = builder
        .edge_use(
            edge,
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::affine(Real::tau(), Real::zero()).unwrap(),
        )
        .unwrap();
    // The exact points differ even though the centers and the two tangents
    // at parameter zero coincide. No numerical oracle or tolerance is needed.
    let expected = surface
        .evaluate_real(&Point2::new(Real::zero(), height))
        .unwrap();
    assert!(!points_equal(&point, &expected).unwrap());
    builder.validate_edge_use_image(
        builder.curve_ref(curve).unwrap(),
        builder.edge_ref(edge).unwrap(),
        builder.edge_use_ref(edge_use).unwrap(),
        builder.pcurve_ref(pcurve).unwrap(),
        &surface,
    )
}

#[test]
fn cylinder_image_rejects_wrong_radial_phase_and_tilted_plane() {
    let surface = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Real::one(),
    )
    .unwrap();
    let results = [-Vector3::x(), Vector3::z()]
        .map(|radial| circle_image(surface.clone(), Point3::origin(), Real::zero(), radial));
    assert_eq!(
        results,
        [
            Err(BuildError::EdgeUseSupportMismatch),
            Err(BuildError::EdgeUseSupportMismatch)
        ]
    );
}

#[test]
fn sphere_image_rejects_wrong_radial_phase_and_tilted_plane() {
    let surface = Surface::sphere(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Real::one(),
    )
    .unwrap();
    let results = [-Vector3::x(), Vector3::z()]
        .map(|radial| circle_image(surface.clone(), Point3::origin(), Real::zero(), radial));
    assert_eq!(
        results,
        [
            Err(BuildError::EdgeUseSupportMismatch),
            Err(BuildError::EdgeUseSupportMismatch)
        ]
    );
}

#[test]
fn cone_image_rejects_wrong_radial_phase_and_tilted_plane() {
    let surface = Surface::cone(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        (Real::pi() / Real::from(4)).unwrap(),
    )
    .unwrap();
    let results = [-Vector3::x(), Vector3::z()].map(|radial| {
        circle_image(
            surface.clone(),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Real::from(2).sqrt().unwrap(),
            radial,
        )
    });
    assert_eq!(
        results,
        [
            Err(BuildError::EdgeUseSupportMismatch),
            Err(BuildError::EdgeUseSupportMismatch)
        ]
    );
}
