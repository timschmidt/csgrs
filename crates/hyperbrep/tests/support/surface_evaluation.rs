use hyperbrep::*;
use hypercurve::{Curve2, CurvePoint2, Point2 as Uv};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "../../../hypercurve/tests/support/selected_fiber_images.rs"]
mod fiber_recipe;

fn q(a: i64, b: i64) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

fn plane() -> Surface {
    Surface::plane(
        Point3::new(1.into(), 2.into(), 3.into()),
        Vector3::new([2.into(), 1.into(), (-1).into()]),
        Vector3::new([(-1).into(), 3.into(), 2.into()]),
    )
    .unwrap()
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    restored
}

pub fn joint_plane_images_compose_and_replay() {
    // alpha²=2, beta³=alpha, alpha and beta positive. Neither coordinate
    // replaces the joint polynomial-fiber selector.
    let uv = fiber_recipe::source();
    assert!(uv.coordinates().is_none());
    let surface = plane();
    let point = surface.point_at(&uv).unwrap();
    assert!(point.try_coordinates().is_none());
    assert!(point.curve_source().is_none());
    assert_eq!(point.surface_source().unwrap().1, &uv);
    let normal = Vector3::new([5.into(), (-3).into(), 7.into()]);
    // x=1+2alpha-beta, y=2+alpha+3beta, z=3-alpha+2beta.
    assert_eq!(
        point.compare_linear_form(&normal, &20.into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        point
            .compare_linear_form(&Vector3::new([3.into(), 1.into(), 0.into()]), &12.into())
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        point
            .compare_linear_form(&Vector3::new([3.into(), 1.into(), 0.into()]), &19.into())
            .unwrap(),
        Ordering::Less
    );
    assert_eq!(
        point
            .compare_linear_form(&Vector3::new([(-1).into(), 2.into(), 0.into()]), &10.into())
            .unwrap(),
        Ordering::Greater
    );
    point.bounds().unwrap();

    let chord = Curve2::try_line(uv, Uv::from_values(4, 4).into()).unwrap();
    let (u, v) = surface.plane_directions().unwrap();
    let carrier = Curve3::planar_image(
        chord,
        surface.plane_origin().unwrap().clone(),
        u.clone(),
        v.clone(),
    )
    .unwrap();
    let curve_point = carrier.point_at(carrier.domain().start()).unwrap();
    assert!(curve_point.coincides_with(&point).unwrap());
    let tiny = q(1, 1_000_000_000_000);
    let separated = point
        .transformed(&Matrix4::affine_translation([
            tiny.clone(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    assert!(!separated.coincides_with(&point).unwrap());
    assert_eq!(
        separated.compare_linear_form(&normal, &20.into()).unwrap(),
        Ordering::Greater
    );

    // A shear and reflection act on the plane frame, leaving the UV authority intact.
    let shear = Matrix4::from_row_major([
        (-1).into(),
        2.into(),
        0.into(),
        4.into(),
        0.into(),
        1.into(),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let sheared = point.transformed(&shear).unwrap();
    assert_eq!(
        sheared
            .compare_linear_form(
                &Vector3::new([(-5).into(), 7.into(), 7.into()]),
                &(-7).into()
            )
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        sheared.surface_source().unwrap().1,
        point.surface_source().unwrap().1
    );
    let mut carrier_builder = ModelBuilder::new();
    carrier_builder.curve(carrier).unwrap();
    let carrier_model = carrier_builder
        .finish()
        .unwrap()
        .transformed(&shear)
        .unwrap();
    let projected = carrier_model.curves().next().unwrap();
    assert!(
        sheared
            .coincides_with(&projected.point_at(projected.domain().start()).unwrap())
            .unwrap()
    );

    let translated = Curve3::translation(point.clone(), Vector3::z()).unwrap();
    let rotated = Curve3::rotation(
        point.clone(),
        Point3::origin(),
        Vector3::z(),
        0.into(),
        (Real::pi() / Real::from(2)).unwrap(),
    )
    .unwrap();
    let mut builder = ModelBuilder::new();
    builder.surface(surface).unwrap();
    for curve in [translated, rotated] {
        let start = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let end = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let range = curve.domain().clone();
        let id = builder.curve(curve).unwrap();
        builder.edge(&start, &end, &id, range).unwrap();
    }
    builder.vertex(sheared).unwrap();
    let original = builder.finish().unwrap();
    let reflection = Matrix4::from_row_major([
        (-1).into(),
        0.into(),
        0.into(),
        4.into(),
        0.into(),
        1.into(),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    for index in 0..3 {
        let model = replay(&original);
        assert!(
            model
                .vertices()
                .next()
                .unwrap()
                .point()
                .coincides_with(&point)
                .unwrap()
        );
        let moved = replay(&model.transformed(&reflection).unwrap());
        assert!(
            moved
                .vertices()
                .next()
                .unwrap()
                .point()
                .coincides_with(&point.transformed(&reflection).unwrap())
                .unwrap()
        );
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(directory) = std::env::var_os("HYPERBREP_SURFACE_EVALUATION_ARTIFACT_DIR") {
            std::fs::create_dir_all(&directory).unwrap();
            std::fs::write(
                std::path::PathBuf::from(directory).join(format!("plane-{index}.json")),
                moved.to_json().unwrap(),
            )
            .unwrap();
        }
        #[cfg(target_arch = "wasm32")]
        let _ = index;
    }
}

fn selected_uv(x: Real, scale: Real) -> CurvePoint2 {
    let source = fiber_recipe::source();
    source
        .affine_form(0.into(), 1.into(), 0.into())
        .parameter()
        .unwrap()
        .affine_point(0.into(), x, scale, 0.into())
        .unwrap()
}

pub fn selected_domains_and_archive_admission() {
    let sphere = Surface::sphere(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let cone = Surface::cone(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        q(1, 4) * Real::pi(),
    )
    .unwrap();
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let torus = Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        2.into(),
        1.into(),
    )
    .unwrap();
    // Periodic axes never impose a finite seam interval. Latitude and cone
    // distance still reject out-of-domain values before representation dispatch.
    for surface in [&sphere, &cone, &cylinder, &torus] {
        assert!(matches!(
            surface.point_at(&selected_uv(100.into(), 1.into())),
            Err(GeometryError::UnsupportedSelectedEvaluation)
        ));
    }
    assert!(matches!(
        sphere.point_at(&selected_uv(100.into(), 2.into())),
        Err(GeometryError::SurfaceParameterOutsideDomain)
    ));
    assert!(matches!(
        cone.point_at(&selected_uv(0.into(), (-1).into())),
        Err(GeometryError::SurfaceParameterOutsideDomain)
    ));
    let profile = Curve3::line(
        Point3::new(1.into(), 0.into(), 0.into()),
        Point3::new(2.into(), 0.into(), 0.into()),
    )
    .unwrap();
    let extrusion = Surface::extrusion(profile.clone(), Vector3::z()).unwrap();
    let revolution = Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap();
    let extrusion_point = extrusion.point_at(&selected_uv(q(1, 2), 1.into())).unwrap();
    assert!(extrusion_point.surface_source().is_some());
    assert_eq!(
        extrusion_point
            .compare_linear_form(&Vector3::x(), &q(3, 2))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        extrusion_point
            .compare_linear_form(&Vector3::y(), &0.into())
            .unwrap(),
        Ordering::Equal
    );
    assert!(matches!(
        extrusion.point_at(&selected_uv(2.into(), 1.into())),
        Err(GeometryError::SurfaceParameterOutsideDomain)
    ));
    assert!(matches!(
        revolution.point_at(&selected_uv(100.into(), q(1, 2))),
        Err(GeometryError::UnsupportedSelectedEvaluation)
    ));
    assert!(matches!(
        revolution.point_at(&selected_uv(0.into(), 1.into())),
        Err(GeometryError::SurfaceParameterOutsideDomain)
    ));

    let controls = vec![
        vec![Point3::origin(), Point3::new(0.into(), 1.into(), 0.into())],
        vec![
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::new(1.into(), 1.into(), 1.into()),
        ],
    ];
    let bezier = Surface::rational_bezier(controls.clone(), vec![vec![1.into(); 2]; 2]).unwrap();
    let nurbs = Surface::nurbs(
        1,
        1,
        controls,
        vec![vec![1.into(); 2]; 2],
        vec![0.into(), 0.into(), 1.into(), 1.into()],
        vec![0.into(), 0.into(), 1.into(), 1.into()],
    )
    .unwrap();
    for surface in [bezier, nurbs] {
        for x in [Real::zero(), Real::one()] {
            let uv = selected_uv(x.clone(), q(1, 2));
            assert!(uv.coordinates().is_none());
            let image = surface.point_at(&uv).unwrap();
            assert_eq!(image.surface_source().unwrap().1, &uv);
            // The original net is S(u,v)=(v,u,uv), including both closed u endpoints.
            assert_eq!(
                image.compare_linear_form(&Vector3::y(), &x).unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                image
                    .compare_linear_form(&Vector3::new([-x, 0.into(), 1.into()]), &0.into())
                    .unwrap(),
                Ordering::Equal
            );
        }
        assert!(matches!(
            surface.point_at(&selected_uv(-q(1, 1_000_000_000_000), q(1, 2))),
            Err(GeometryError::SurfaceParameterOutsideDomain)
        ));
        assert!(matches!(
            surface.point_at(&selected_uv(0.into(), 1.into())),
            Err(GeometryError::SurfaceParameterOutsideDomain)
        ));
    }

    let mut builder = ModelBuilder::new();
    builder
        .vertex(plane().point_at(&fiber_recipe::source()).unwrap())
        .unwrap();
    let model = builder.finish().unwrap();
    let raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    assert_eq!(raw["version"], 12);
    let mut old = raw.clone();
    old["version"] = 10.into();
    assert!(matches!(
        RawModel::from_json(&old.to_string(), &ArchiveLimits::default()),
        Err(PersistenceError::UnsupportedVersion(10))
    ));
    let mut bad = raw.clone();
    bad["vertices"][0]["SurfaceImage"]["source"]["Plane"]["v"] =
        bad["vertices"][0]["SurfaceImage"]["source"]["Plane"]["u"].clone();
    assert!(matches!(
        RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::DegeneratePlaneBasis
        )))
    ));
    let mut oversized = raw.clone();
    oversized["vertices"][0]["SurfaceImage"]["source"]["Plane"]["origin"][0] =
        "invalid scalar behind nested degree gate".into();
    assert!(matches!(
        RawModel::from_json(
            &oversized.to_string(),
            &ArchiveLimits {
                degree: 2,
                ..ArchiveLimits::default()
            }
        ),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Degree,
            observed: 3,
            limit: 2
        }))
    ));
    assert!(matches!(
        RawModel::from_json(
            &oversized.to_string(),
            &ArchiveLimits {
                records: 3,
                ..ArchiveLimits::default()
            }
        ),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Records,
            observed: 4,
            limit: 3
        }))
    ));
    replay(&model);

    // Exercise nested point admission without a vertex record that could
    // independently trigger the same degree quota first.
    let point = plane().point_at(&fiber_recipe::source()).unwrap();
    for curve in [
        Curve3::translation(point.clone(), Vector3::z()).unwrap(),
        Curve3::rotation(
            point,
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            Real::pi(),
        )
        .unwrap(),
    ] {
        let mut builder = ModelBuilder::new();
        builder.curve(curve).unwrap();
        let model = builder.finish().unwrap();
        let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
        let record = raw["curves"][0]
            .as_object_mut()
            .unwrap()
            .values_mut()
            .next()
            .unwrap();
        let image = &mut record["origin"]["SurfaceImage"];
        assert!(image["parameter"].get("PolynomialFiber").is_some());
        image["source"]["Plane"]["origin"][0] = "invalid scalar behind sweep origin gate".into();
        assert!(matches!(
            RawModel::from_json(
                &raw.to_string(),
                &ArchiveLimits {
                    degree: 2,
                    ..ArchiveLimits::default()
                }
            ),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
                resource: ArchiveResource::Degree,
                observed: 3,
                limit: 2
            }))
        ));
        replay(&model);
    }
    let mut wrong_selector = raw;
    wrong_selector["vertices"][0]["SurfaceImage"]["parameter"]["PolynomialFiber"]["interval"] =
        serde_json::json!([Real::from(3), Real::from(4)]);
    assert!(matches!(
        RawModel::from_json(&wrong_selector.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PlanarCurve(_)
        )))
    ));
    replay(&model);
}

pub fn selected_surface_policy_and_recovery() {
    let uv = fiber_recipe::source();
    let surface = plane();
    let point = surface.point_at(&uv).unwrap();
    let direct = OperationContext::strict(WorkBudget::ZERO).evaluate(|| surface.point_at(&uv));
    assert!(
        direct
            .result
            .unwrap()
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    assert_eq!(direct.usage, WorkUsage::default());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let cold = surface.point_at(&uv).unwrap();
            for _ in 0..3 {
                assert!(cold.coincides_with(&point).unwrap());
                assert_eq!(
                    cold.compare_linear_form(
                        &Vector3::new([5.into(), (-3).into(), 7.into()]),
                        &20.into()
                    )
                    .unwrap(),
                    Ordering::Equal
                );
            }
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
    // Force fresh solver work through archive reconstruction, then retry the
    // exact same image with the normal budget. Exhaustion never becomes a point.
    let mut builder = ModelBuilder::new();
    builder.vertex(point).unwrap();
    let json = builder.finish().unwrap().to_json().unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    assert_eq!(denied.result.unwrap_err().limit, 0);
    RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
}
