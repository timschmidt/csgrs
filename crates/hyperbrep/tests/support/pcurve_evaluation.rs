use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveParameter2, Point2 as Uv,
};

fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}
fn selected() -> CurveParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
    .into()
}
fn section(nurbs: bool) -> SurfaceIntersectionCurve {
    // Profile (t³,t,0); the section z=x is (t³,t,t³).
    let points = (0..4)
        .map(|i| Point3::new(if i == 3 { 1.into() } else { 0.into() }, q(i, 3), 0.into()))
        .collect();
    let profile = if nurbs {
        Curve3::nurbs(
            3,
            points,
            vec![Real::one(); 4],
            vec![
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
                1.into(),
                1.into(),
                1.into(),
            ],
        )
        .unwrap()
    } else {
        Curve3::rational_bezier(points, vec![Real::one(); 4]).unwrap()
    };
    let surface = Surface::extrusion(profile, Vector3::z()).unwrap();
    let plane = Surface::plane(
        Point3::origin(),
        Vector3::from_xyz(1.into(), 0.into(), 1.into()),
        Vector3::y(),
    )
    .unwrap();
    let SurfaceSurfaceIntersection::Curve(section) = surface.intersect_surface(&plane).unwrap()
    else {
        panic!("complete native section")
    };
    *section
}

pub fn selected_section_pcurves_preserve_images_and_replay() {
    let cutting = Surface::plane(
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let expected = [
        selected()
            .affine_point(1.into(), 0.into(), 0.into(), q(1, 2))
            .unwrap(),
        selected()
            .affine_point(0.into(), q(1, 2), 1.into(), 0.into())
            .unwrap(),
    ];
    for nurbs in [false, true] {
        let original = section(nurbs);
        for (index, trace) in [
            original.clone(),
            original.reversed().unwrap(),
            original.subcurve(&q(1, 4), &q(7, 8)).unwrap(),
        ]
        .into_iter()
        .enumerate()
        {
            let CurveSurfaceIntersection::Points(contacts) =
                cutting.intersect_curve(trace.curve()).unwrap()
            else {
                panic!("one selected contact")
            };
            assert_eq!(contacts.len(), 1);
            let contact = &contacts[0];
            assert!(contact.parameter.try_real().is_none());
            // The plane consumes the selected joint UV directly. Its point
            // agrees with the independent spatial section/cutting-plane result.
            let section_plane = Surface::plane(
                Point3::origin(),
                Vector3::from_xyz(1.into(), 0.into(), 1.into()),
                Vector3::y(),
            )
            .unwrap();
            let uv = trace.second_pcurve().point_at(&contact.parameter).unwrap();
            let surface_point = section_plane.point_at(&uv).unwrap();
            assert!(surface_point.try_coordinates().is_none());
            assert!(surface_point.coincides_with(&contact.point).unwrap());
            assert_eq!(
                surface_point
                    .compare_linear_form(&Vector3::x(), &q(1, 2))
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            let mut builder = ModelBuilder::new();
            builder.vertex(contact.point.clone()).unwrap();
            builder.curve(trace.curve().clone()).unwrap();
            for (pcurve, expected) in [trace.first_pcurve(), trace.second_pcurve()]
                .into_iter()
                .zip(&expected)
            {
                let image = pcurve.point_at(&contact.parameter).unwrap();
                assert!(image.coordinates().is_none());
                assert!(image.coincides_with(expected).unwrap());
                let chord = Curve2::try_line(image, Uv::from_values(4, 4).into()).unwrap();
                builder.pcurve(Pcurve::new(chord)).unwrap();
                assert!(matches!(
                    pcurve.point_at(&Real::from(-1).into()),
                    Err(GeometryError::ParameterOutsideDomain)
                ));
            }
            let json = builder.finish().unwrap().to_json().unwrap();
            #[cfg(not(target_arch = "wasm32"))]
            if let Ok(directory) = std::env::var("HYPERBREP_PCURVE_EVALUATION_ARTIFACT_DIR") {
                let directory = std::path::PathBuf::from(directory);
                std::fs::create_dir_all(&directory).unwrap();
                std::fs::write(
                    directory.join(format!("section-{nurbs}-{index}.json")),
                    &json,
                )
                .unwrap();
            }
            #[cfg(target_arch = "wasm32")]
            let _ = index;

            for _ in 0..3 {
                let restored = RawModel::from_json(&json, &ArchiveLimits::default())
                    .unwrap()
                    .validate()
                    .unwrap();
                assert_eq!(restored.to_json().unwrap(), json);
                for (pcurve, expected) in restored.pcurves().zip(&expected) {
                    assert!(pcurve.curve().start().coincides_with(expected).unwrap());
                }
                let transform = Matrix4::from_row_major([
                    1.into(),
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
                let moved = restored.transformed(&transform).unwrap();
                let moved =
                    RawModel::from_json(&moved.to_json().unwrap(), &ArchiveLimits::default())
                        .unwrap()
                        .validate()
                        .unwrap();
                assert!(
                    contact
                        .point
                        .transformed(&transform)
                        .unwrap()
                        .coincides_with(moved.vertices().next().unwrap().point())
                        .unwrap()
                );
            }
        }
    }
}

pub fn selected_evaluation_policy_budget_and_recovery() {
    let trace = section(false);
    let plane = Surface::plane(
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let evaluate = || -> GeometryResult<hypercurve::CurvePoint2> {
        let CurveSurfaceIntersection::Points(points) = plane.intersect_curve(trace.curve())? else {
            panic!("one selected section contact")
        };
        trace.first_pcurve().point_at(&points[0].parameter)
    };
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(evaluate);
    assert_eq!(denied.result.unwrap_err().limit, 0);
    let expected = selected()
        .affine_point(1.into(), 0.into(), 0.into(), q(1, 2))
        .unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            let image = evaluate().unwrap();
            assert!(image.coincides_with(&expected).unwrap());
        });
        assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
    }
}
