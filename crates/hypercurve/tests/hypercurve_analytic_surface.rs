use hypercurve::{
    Classification, CurveError, CurvePolicy, Point3, Real, RetainedAnalyticPoleRelation3,
    RetainedAnalyticSeamRelation3, RetainedAnalyticSurfaceIdentity3, RetainedCylinderAxialDomain3,
    RetainedCylinderFrame3, RetainedCylinderPointRelation3, SignedAxis3,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn z_cylinder() -> RetainedCylinderFrame3 {
    RetainedCylinderFrame3::try_new(
        RetainedAnalyticSurfaceIdentity3::new(7),
        p(0, 0, 0),
        SignedAxis3::PosZ,
        SignedAxis3::PosX,
        r(25),
        RetainedCylinderAxialDomain3::bounded(r(-2), r(8), &policy()).unwrap(),
        &policy(),
    )
    .unwrap()
}

#[test]
fn retained_cylinder_rejects_invalid_axis_seam_radius_and_domain() {
    let surface = RetainedAnalyticSurfaceIdentity3::new(1);
    assert_eq!(surface.source_index(), 1);
    assert_eq!(
        RetainedCylinderFrame3::try_new(
            surface,
            p(0, 0, 0),
            SignedAxis3::PosZ,
            SignedAxis3::NegZ,
            r(1),
            RetainedCylinderAxialDomain3::unbounded(),
            &policy(),
        )
        .unwrap_err(),
        CurveError::InvalidAnalyticSurfaceFrame
    );
    assert_eq!(
        RetainedCylinderFrame3::try_new(
            surface,
            p(0, 0, 0),
            SignedAxis3::PosZ,
            SignedAxis3::PosX,
            r(0),
            RetainedCylinderAxialDomain3::unbounded(),
            &policy(),
        )
        .unwrap_err(),
        CurveError::InvalidAnalyticSurfaceFrame
    );
    assert_eq!(
        RetainedCylinderAxialDomain3::bounded(r(4), r(3), &policy()).unwrap_err(),
        CurveError::InvalidAnalyticSurfaceFrame
    );
}

#[test]
fn retained_cylinder_classifies_surface_seam_and_domain_boundary() {
    let cylinder = z_cylinder();
    assert_eq!(cylinder.surface(), RetainedAnalyticSurfaceIdentity3::new(7));
    assert_eq!(cylinder.axis(), SignedAxis3::PosZ);
    assert_eq!(cylinder.seam(), SignedAxis3::PosX);
    assert_eq!(cylinder.radius_squared(), &r(25));

    let seam = decided(cylinder.classify_point(&p(5, 0, 3), &policy()));
    assert_eq!(seam.relation(), RetainedCylinderPointRelation3::OnSeam);
    assert_eq!(seam.seam(), RetainedAnalyticSeamRelation3::OnSeam);
    assert_eq!(seam.pole(), RetainedAnalyticPoleRelation3::NotApplicable);
    assert_eq!(seam.axial_coordinate(), &r(3));
    assert_eq!(seam.radial_squared(), &r(25));
    assert_eq!(seam.radius_squared(), &r(25));
    assert!(!seam.axial_domain_boundary());

    let ordinary = decided(cylinder.classify_point(&p(0, 5, 8), &policy()));
    assert_eq!(
        ordinary.relation(),
        RetainedCylinderPointRelation3::OnSurface
    );
    assert_eq!(ordinary.seam(), RetainedAnalyticSeamRelation3::NotOnSeam);
    assert!(ordinary.axial_domain_boundary());
}

#[test]
fn retained_cylinder_seam_obeys_signed_seam_ray() {
    let positive = z_cylinder();
    let negative = RetainedCylinderFrame3::try_new(
        RetainedAnalyticSurfaceIdentity3::new(8),
        p(0, 0, 0),
        SignedAxis3::PosZ,
        SignedAxis3::NegX,
        r(25),
        RetainedCylinderAxialDomain3::unbounded(),
        &policy(),
    )
    .unwrap();

    assert_eq!(
        decided(positive.classify_point(&p(-5, 0, 0), &policy())).relation(),
        RetainedCylinderPointRelation3::OnSurface
    );
    assert_eq!(
        decided(negative.classify_point(&p(-5, 0, 0), &policy())).relation(),
        RetainedCylinderPointRelation3::OnSeam
    );
}

#[test]
fn retained_cylinder_reports_radius_and_axial_domain_failures_separately() {
    let cylinder = z_cylinder();

    let off_radius = decided(cylinder.classify_point(&p(4, 0, 0), &policy()));
    assert_eq!(
        off_radius.relation(),
        RetainedCylinderPointRelation3::OutsideRadius
    );
    assert_eq!(off_radius.seam(), RetainedAnalyticSeamRelation3::NotOnSeam);
    assert_eq!(off_radius.radial_squared(), &r(16));

    let outside_domain = decided(cylinder.classify_point(&p(5, 0, 9), &policy()));
    assert_eq!(
        outside_domain.relation(),
        RetainedCylinderPointRelation3::OutsideAxialDomain
    );
    assert_eq!(outside_domain.axial_coordinate(), &r(9));
    assert_eq!(outside_domain.radial_squared(), &r(25));
}

#[test]
fn retained_cylinder_negative_axis_reverses_axial_coordinate() {
    let cylinder = RetainedCylinderFrame3::try_new(
        RetainedAnalyticSurfaceIdentity3::new(9),
        p(1, 2, 3),
        SignedAxis3::NegY,
        SignedAxis3::PosX,
        r(9),
        RetainedCylinderAxialDomain3::bounded(r(-1), r(1), &policy()).unwrap(),
        &policy(),
    )
    .unwrap();

    let report = decided(cylinder.classify_point(&p(4, 1, 3), &policy()));
    assert_eq!(report.relation(), RetainedCylinderPointRelation3::OnSeam);
    assert_eq!(report.axial_coordinate(), &r(1));
    assert!(report.axial_domain_boundary());
}
