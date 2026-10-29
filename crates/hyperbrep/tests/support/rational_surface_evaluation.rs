use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "../../../hypercurve/tests/support/rational_point_forms.rs"]
mod point_recipe;

fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

fn tensor(nurbs: bool, weighted: bool) -> Surface {
    // Homogeneous map (u²,16v³,u²+16v³,1+uv), or weight one.
    // A nonuniform quadratic B-spline reproduces u² via its blossom t1*t2.
    let (u_mean, u_squared) = if nurbs {
        (
            vec![0.into(), q(1, 4), q(3, 4), 1.into()],
            vec![0.into(), 0.into(), q(1, 2), 1.into()],
        )
    } else {
        (
            vec![0.into(), q(1, 2), 1.into()],
            vec![0.into(), 0.into(), 1.into()],
        )
    };
    let weights = (0..4)
        .map(|j| {
            u_mean
                .iter()
                .map(|u| {
                    if weighted {
                        Real::one() + u * q(j, 3)
                    } else {
                        Real::one()
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let controls = (0..4)
        .map(|j| {
            u_squared
                .iter()
                .enumerate()
                .map(|(i, x)| {
                    let y = Real::from(if j == 3 { 16 } else { 0 });
                    Point3::new(
                        (x / &weights[j][i]).unwrap(),
                        (&y / &weights[j][i]).unwrap(),
                        ((x + y) / &weights[j][i]).unwrap(),
                    )
                })
                .collect()
        })
        .collect();
    if nurbs {
        Surface::nurbs(
            2,
            3,
            controls,
            weights,
            vec![
                0.into(),
                0.into(),
                0.into(),
                q(1, 2),
                1.into(),
                1.into(),
                1.into(),
            ],
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
        Surface::rational_bezier(controls, weights).unwrap()
    }
}

fn independent_x() -> GeometricPoint3 {
    // X = 1/(8(1+uv)), (uv)^3=1/1024, so
    // 2 - 48X + 384X² - 1025X³ = 0, X in (1/10,3/25).
    let root = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            2.into(),
            (-48).into(),
            384.into(),
            (-1025).into(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(q(1, 10), q(3, 25)).unwrap(),
    )
    .unwrap();
    let parameter: hypercurve::CurveParameter2 = BezierParameter2::Algebraic(root).into();
    let uv = parameter
        .affine_point(1.into(), 0.into(), 0.into(), 0.into())
        .unwrap();
    Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())
        .unwrap()
        .point_at(&uv)
        .unwrap()
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    record_archive(&json);
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    restored
}

pub fn joint_tensor_images_compose_transform_and_replay() {
    let uv = point_recipe::joint_uv();
    let expected_x = independent_x();
    for nurbs in [false, true] {
        for weighted in [false, true] {
            let surface = tensor(nurbs, weighted);
            let point = surface.point_at(&uv).unwrap();
            assert!(point.try_coordinates().is_none());
            assert!(point.curve_source().is_none());
            assert_eq!(point.surface_source().unwrap().1, &uv);
            assert_eq!(
                point
                    .compare_linear_form(
                        &Vector3::new([(-1).into(), (-1).into(), 1.into()]),
                        &0.into()
                    )
                    .unwrap(),
                Ordering::Equal
            );
            if weighted {
                assert_eq!(
                    point.compare_coordinate(&expected_x, Axis::X).unwrap(),
                    Ordering::Equal
                );
            } else {
                assert_eq!(
                    point.compare_linear_form(&Vector3::x(), &q(1, 8)).unwrap(),
                    Ordering::Equal
                );
                let expected_uv = uv
                    .affine_form(1.into(), 0.into(), 0.into())
                    .parameter()
                    .unwrap()
                    .affine_point(0.into(), q(1, 8), 1.into(), 0.into())
                    .unwrap();
                let plane = Surface::plane(
                    Point3::origin(),
                    Vector3::new([1.into(), 0.into(), 1.into()]),
                    Vector3::new([0.into(), 1.into(), 1.into()]),
                )
                .unwrap();
                assert!(
                    point
                        .coincides_with(&plane.point_at(&expected_uv).unwrap())
                        .unwrap()
                );
            }
            let bounds = point.bounds().unwrap();
            for axis in [Axis::X, Axis::Y, Axis::Z] {
                assert_ne!(
                    point
                        .compare_coordinate(&bounds.mins.clone().into(), axis)
                        .unwrap(),
                    Ordering::Less
                );
                assert_ne!(
                    point
                        .compare_coordinate(&bounds.maxs.clone().into(), axis)
                        .unwrap(),
                    Ordering::Greater
                );
            }

            assert_ne!(
                point
                    .compare_coordinate(&bounds.mins.clone().into(), Axis::X)
                    .unwrap(),
                Ordering::Less
            );
            assert_ne!(
                point
                    .compare_coordinate(&bounds.maxs.clone().into(), Axis::Z)
                    .unwrap(),
                Ordering::Greater
            );
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
            let transformed = point.transformed(&shear).unwrap();
            let normal = Vector3::new([1.into(), (-3).into(), 1.into()]);
            assert_eq!(
                transformed
                    .compare_linear_form(&normal, &11.into())
                    .unwrap(),
                Ordering::Equal
            );
            let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
            let displaced = transformed
                .transformed(&Matrix4::affine_translation([0.into(), 0.into(), tiny]))
                .unwrap();
            assert!(!displaced.coincides_with(&transformed).unwrap());
            assert_eq!(
                displaced.compare_linear_form(&normal, &11.into()).unwrap(),
                Ordering::Greater
            );
            let mut builder = ModelBuilder::new();
            builder.surface(surface).unwrap();
            builder.vertex(transformed).unwrap();
            for carrier in [
                Curve3::translation(point.clone(), Vector3::z()).unwrap(),
                Curve3::rotation(
                    point.clone(),
                    Point3::origin(),
                    Vector3::z(),
                    0.into(),
                    (Real::pi() / Real::from(2)).unwrap(),
                )
                .unwrap(),
            ] {
                let start = builder
                    .vertex(carrier.point_at(carrier.domain().start()).unwrap())
                    .unwrap();
                let end = builder
                    .vertex(carrier.point_at(carrier.domain().end()).unwrap())
                    .unwrap();
                let range = carrier.domain().clone();
                let id = builder.curve(carrier).unwrap();
                builder.edge(&start, &end, &id, range).unwrap();
            }
            let model = builder.finish().unwrap().transformed(&shear).unwrap();
            let mut restored = replay(&model);
            for _ in 0..2 {
                restored = replay(&restored);
            }
            for (original, replayed) in model.vertices().zip(restored.vertices()) {
                assert!(original.point().coincides_with(replayed.point()).unwrap());
            }
        }
    }
}

pub fn selected_rational_extrusions_and_knot_boundaries() {
    let uv = point_recipe::joint_uv();
    let controls = vec![
        Point3::origin(),
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Point3::new(1.into(), 1.into(), 0.into()),
    ];
    let profiles = [
        Curve3::rational_bezier(controls.clone(), vec![1.into(); 3]).unwrap(),
        Curve3::nurbs(
            2,
            controls,
            vec![1.into(); 3],
            vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
        )
        .unwrap(),
        Curve3::line(Point3::origin(), Point3::new(1.into(), 0.into(), 0.into())).unwrap(),
    ];
    for profile in profiles {
        let is_line = profile.kind() == Curve3Kind::Line;
        let surface = Surface::extrusion(profile, Vector3::z()).unwrap();
        let point = surface.point_at(&uv).unwrap();
        assert_eq!(
            point
                .compare_linear_form(&Vector3::y(), &if is_line { 0.into() } else { q(1, 8) })
                .unwrap(),
            Ordering::Equal
        );
        point.bounds().unwrap();
        let mut builder = ModelBuilder::new();
        builder.vertex(point).unwrap();
        replay(&builder.finish().unwrap());
    }
    // The selected fiber lies exactly on the internal u knot and retains v.
    let boundary = hypercurve::CurvePointDefinition2::PolynomialFiber {
        equation: hypersolve::BivariatePolynomial::new(vec![
            vec![0.into(), 0.into(), 0.into(), 16.into()],
            vec![(-1).into()],
        ]),
        parameter: BezierParameter2::Exact(q(1, 2)).into(),
        root: hypersolve::IsolatedRootInterval {
            lower: q(1, 4),
            upper: q(3, 8),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let image = tensor(true, false).point_at(&boundary).unwrap();
    assert_eq!(
        image.compare_linear_form(&Vector3::x(), &q(1, 4)).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        image.compare_linear_form(&Vector3::y(), &q(1, 2)).unwrap(),
        Ordering::Equal
    );
    let mut builder = ModelBuilder::new();
    builder.vertex(image).unwrap();
    replay(&builder.finish().unwrap());
}

pub fn rational_surface_policy_and_admission() {
    let uv = point_recipe::joint_uv();
    let surface = tensor(false, true);
    let point = surface.point_at(&uv).unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            for _ in 0..3 {
                let cold = surface.point_at(&uv).unwrap();
                assert!(cold.coincides_with(&point).unwrap());
                assert_eq!(
                    cold.compare_linear_form(
                        &Vector3::new([(-1).into(), (-1).into(), 1.into()]),
                        &0.into()
                    )
                    .unwrap(),
                    Ordering::Equal
                );
            }
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
    let mut builder = ModelBuilder::new();
    builder.vertex(point).unwrap();
    let model = builder.finish().unwrap();
    let json = model.to_json().unwrap();
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    assert_eq!(denied.result.unwrap_err().limit, 0);
    replay(&model);
    let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
    let mut invalid_weight = raw.clone();
    invalid_weight["vertices"][0]["SurfaceImage"]["source"]["RationalBezier"]["weights"][0][0] =
        serde_json::json!(Real::zero());
    assert!(matches!(
        RawModel::from_json(&invalid_weight.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::InvalidWeight
        )))
    ));
    let mut oversized = raw.clone();
    oversized["vertices"][0]["SurfaceImage"]["source"]["RationalBezier"]["control_points"][0][0]
        [0] = "invalid scalar behind tensor degree admission".into();
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
    let mut bad_selector = raw;
    bad_selector["vertices"][0]["SurfaceImage"]["parameter"]["PolynomialFiber"]["interval"] =
        serde_json::json!([Real::from(3), Real::from(4)]);
    assert!(matches!(
        RawModel::from_json(&bad_selector.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate(),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PlanarCurve(_)
        )))
    ));
    replay(&model);
}

fn record_archive(text: &str) {
    let _ = text;
    #[cfg(not(target_arch = "wasm32"))]
    if let Ok(folder) = std::env::var("HYPERBREP_RATIONAL_SURFACE_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(
            folder.join(format!(
                "rational-{:03}.json",
                NEXT.fetch_add(1, Ordering::Relaxed)
            )),
            text,
        )
        .unwrap();
    }
}

pub fn nurbs_second_span_and_final_endpoint() {
    original_nonunit_knot_coordinates();
    let uv = point_recipe::joint_uv();
    let scaled = hypercurve::Curve2::try_line(uv, hypercurve::Point2::from_values(0, 0).into())
        .unwrap()
        .transform_similarity(
            &hypercurve::Similarity2::try_from_real_affine(
                2.into(),
                0.into(),
                0.into(),
                2.into(),
                0.into(),
                0.into(),
            )
            .unwrap(),
        )
        .unwrap()
        .start();
    let root = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            2.into(),
            (-12).into(),
            24.into(),
            (-17).into(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(q(1, 3), q(3, 8)).unwrap(),
    )
    .unwrap();
    let parameter: hypercurve::CurveParameter2 = BezierParameter2::Algebraic(root).into();
    let expected = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())
        .unwrap()
        .point_at(
            &parameter
                .affine_point(1.into(), 0.into(), 0.into(), 0.into())
                .unwrap(),
        )
        .unwrap();
    for weighted in [false, true] {
        let point = tensor(true, weighted).point_at(&scaled).unwrap();
        if weighted {
            assert_eq!(
                point.compare_coordinate(&expected, Axis::X).unwrap(),
                Ordering::Equal
            );
        } else {
            assert_eq!(
                point.compare_linear_form(&Vector3::x(), &q(1, 2)).unwrap(),
                Ordering::Equal
            );
        }
        assert_eq!(
            point
                .compare_linear_form(
                    &Vector3::new([(-1).into(), (-1).into(), 1.into()]),
                    &0.into()
                )
                .unwrap(),
            Ordering::Equal
        );
        point.bounds().unwrap();
        let mut builder = ModelBuilder::new();
        builder.vertex(point).unwrap();
        replay(&builder.finish().unwrap());
    }
    let endpoint = hypercurve::CurvePointDefinition2::PolynomialFiber {
        equation: hypersolve::BivariatePolynomial::new(vec![
            vec![0.into(), 0.into(), 0.into(), 16.into()],
            vec![(-1).into()],
        ]),
        parameter: BezierParameter2::Exact(1.into()).into(),
        root: hypersolve::IsolatedRootInterval {
            lower: q(3, 8),
            upper: q(1, 2),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let point = tensor(true, false).point_at(&endpoint).unwrap();
    assert!(
        point
            .coincides_with(&Point3::new(1.into(), 1.into(), 2.into()).into())
            .unwrap()
    );
    let mut builder = ModelBuilder::new();
    builder.vertex(point).unwrap();
    replay(&builder.finish().unwrap());
}

fn original_nonunit_knot_coordinates() {
    let uv = point_recipe::joint_uv();
    let shifted =
        hypercurve::Curve2::try_line(uv.clone(), hypercurve::Point2::from_values(0, 0).into())
            .unwrap()
            .transform_similarity(
                &hypercurve::Similarity2::try_from_real_affine(
                    1.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                    2.into(),
                    0.into(),
                )
                .unwrap(),
            )
            .unwrap()
            .start();
    let controls = (0..4)
        .map(|j| {
            [Real::zero(), Real::zero(), q(1, 2), Real::one()]
                .into_iter()
                .map(|x| {
                    let y = Real::from(if j == 3 { 16 } else { 0 });
                    Point3::new(x.clone(), y.clone(), x + y)
                })
                .collect()
        })
        .collect();
    let surface = Surface::nurbs(
        2,
        3,
        controls,
        vec![vec![Real::one(); 4]; 4],
        vec![
            2.into(),
            2.into(),
            2.into(),
            q(5, 2),
            3.into(),
            3.into(),
            3.into(),
        ],
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
    .unwrap();
    let actual = surface.point_at(&shifted).unwrap();
    assert!(
        actual
            .coincides_with(&tensor(true, false).point_at(&uv).unwrap())
            .unwrap()
    );
    assert_eq!(
        actual.compare_linear_form(&Vector3::x(), &q(1, 8)).unwrap(),
        Ordering::Equal
    );
    actual.bounds().unwrap();
    let mut builder = ModelBuilder::new();
    builder.vertex(actual).unwrap();
    replay(&builder.finish().unwrap());
}
