use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, Curve2, QuadraticBezier2,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected_profile() -> Curve3 {
    profile_ending_at(2, 3.into(), q(3, 4), q(7, 8))
}

fn profile_ending_at(coefficient: i32, shift: Real, lower: Real, upper: Real) -> Curve3 {
    let root = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            (-1).into(),
            0.into(),
            0.into(),
            coefficient.into(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(lower, upper).unwrap(),
    )
    .unwrap();
    let source = Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            hypercurve::Point2::from_values(0, 0),
            hypercurve::Point2::from_values(2, 2),
            hypercurve::Point2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(q(1, 4)),
            BezierParameter2::Algebraic(root),
        )
        .unwrap(),
    )
    .unwrap();
    Curve3::planar_image(
        source,
        Point3::new(shift, 0.into(), 0.into()),
        Vector3::x(),
        Vector3::z(),
    )
    .unwrap()
}

pub fn selected_meridian_patch_retains_latitudes_and_replays() {
    let profile = selected_profile();
    assert!(profile.domain().end().try_real().is_none());
    let sheet = builder::revolution_patch(
        profile,
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        (Real::pi() / Real::from(2)).unwrap(),
    )
    .expect("selected meridian endpoints must retain their exact latitude generators");
    let model = sheet.model();
    assert_eq!(model.faces().len(), 1);
    assert_eq!(model.edges().len(), 4);
    assert_eq!(model.solids().len(), 0);
    let surface = model.faces().next().unwrap().surface();
    // The original quadratic is (2, 1) at t=1/2, and its unit left
    // parallel is (2, 2). The radial frame adds three to the radius.
    for (angle, expected) in [
        (Real::zero(), Point3::new(5.into(), 0.into(), 2.into())),
        (
            (Real::pi() / Real::from(2)).unwrap(),
            Point3::new(0.into(), 5.into(), 2.into()),
        ),
    ] {
        let actual = surface.point_at(&Point2::new(angle, q(1, 2))).unwrap();
        assert!(
            hyperlimit::point3_equal(&actual, &expected, hyperlimit::PredicatePolicy::STRICT)
                .value()
                .unwrap()
        );
    }
    assert!(
        model
            .vertices()
            .any(|v| v.point().try_coordinates().is_none())
    );
    replay(model);
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    record_archive(&json);
    let mut cold = model.clone();
    for _ in 0..3 {
        cold = RawModel::from_json(&cold.to_json().unwrap(), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
    }
    cold
}

fn record_archive(text: &str) {
    let _ = text;
    #[cfg(not(target_arch = "wasm32"))]
    if let Ok(folder) = std::env::var("HYPERBREP_SELECTED_REVOLUTION_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let index = NEXT.fetch_add(1, Ordering::Relaxed);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(folder.join(format!("revolution-{index:03}.json")), text).unwrap();
    }
}

fn quarter() -> Real {
    (Real::pi() / Real::from(2)).unwrap()
}

fn reflected_similarity() -> Matrix4 {
    Matrix4::from_row_major([
        (-2).into(),
        0.into(),
        0.into(),
        7.into(),
        0.into(),
        2.into(),
        0.into(),
        (-3).into(),
        0.into(),
        0.into(),
        2.into(),
        5.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}

pub fn selected_rotation_retains_points_and_angular_charts() {
    use std::cmp::Ordering;
    let source = selected_profile();
    let point = source.point_at(source.domain().end()).unwrap();
    assert!(point.try_coordinates().is_none());
    let rotation = Curve3::rotation(
        point.clone(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        Real::pi(),
    )
    .unwrap();
    assert_eq!(rotation.kind(), Curve3Kind::Rotation);
    assert!(
        rotation
            .rotation_geometry()
            .unwrap()
            .origin
            .curve_source()
            .is_some()
    );
    let at_quarter = rotation.point_at(&quarter().into()).unwrap();
    assert_eq!(
        at_quarter
            .compare_linear_form(&Vector3::x(), &Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        at_quarter.compare_coordinate(&point, Axis::Z).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        at_quarter
            .compare_linear_form(&Vector3::y(), &Real::from(3))
            .unwrap(),
        Ordering::Greater
    );
    let reversed = rotation.reversed().unwrap();
    assert!(
        reversed
            .point_at(&Real::zero().into())
            .unwrap()
            .coincides_with(&rotation.point_at(&Real::pi().into()).unwrap())
            .unwrap()
    );
    let (left, right) = reversed.split_at(&quarter()).unwrap();
    assert_eq!(left.kind(), Curve3Kind::Rotation);
    assert_eq!(right.kind(), Curve3Kind::Rotation);
    assert!(
        left.point_at(left.domain().end())
            .unwrap()
            .coincides_with(&right.point_at(right.domain().start()).unwrap())
            .unwrap()
    );
    let sheet = builder::revolution_patch(
        selected_profile(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        quarter(),
    )
    .unwrap();
    let transformed = sheet.model().transformed(&reflected_similarity()).unwrap();
    let transformed_orbit = transformed
        .edges()
        .find(|e| {
            e.curve()
                .rotation_geometry()
                .is_some_and(|orbit| orbit.origin.try_coordinates().is_none())
        })
        .unwrap();
    assert!(
        transformed_orbit
            .curve()
            .point_at(&quarter().into())
            .unwrap()
            .coincides_with(&at_quarter.transformed(&reflected_similarity()).unwrap())
            .unwrap()
    );
    assert!(rotation.bounds().is_ok());
    assert!(matches!(
        rotation.point_at(source.domain().end()),
        Err(GeometryError::UnsupportedSelectedEvaluation)
    ));
    assert!(matches!(
        rotation.derivative_at(&quarter(), 1),
        Err(GeometryError::UnrepresentablePoint)
    ));
    // Authored origins use the same retained-orbit tag and admit ordinary
    // vector derivatives. These values follow directly from a radius-five orbit.
    let native = Curve3::rotation(
        Point3::new(5.into(), 0.into(), 2.into()).into(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        Real::pi(),
    )
    .unwrap();
    assert_eq!(native.kind(), Curve3Kind::Rotation);
    let derivative = native.derivative_at(&quarter(), 1).unwrap();
    assert_eq!(
        derivative.vector(),
        &Vector3::new([(-5).into(), 0.into(), 0.into()])
    );
    for (order, expected) in [
        (2, Vector3::new([0.into(), (-5).into(), 0.into()])),
        (3, Vector3::new([5.into(), 0.into(), 0.into()])),
        (4, Vector3::new([0.into(), 5.into(), 0.into()])),
    ] {
        assert_eq!(
            native.derivative_at(&quarter(), order).unwrap().vector(),
            &expected
        );
    }
    assert_eq!(
        native
            .reversed()
            .unwrap()
            .derivative_at(&quarter(), 1)
            .unwrap()
            .vector(),
        &Vector3::new([5.into(), 0.into(), 0.into()])
    );
    let bounds = native.bounds().unwrap();
    assert_eq!(bounds.mins, Point3::new((-5).into(), (-5).into(), 2.into()));
    assert_eq!(bounds.maxs, Point3::new(5.into(), 5.into(), 2.into()));
}

fn check_patch_samples(model: &Model, reversed: bool, reflected: bool) {
    let surface = model.faces().next().unwrap().surface();
    let v = if reversed { -q(1, 2) } else { q(1, 2) };
    let samples = if reflected {
        [
            (
                Real::zero(),
                Point3::new((-3).into(), (-3).into(), 9.into()),
            ),
            (-quarter(), Point3::new(7.into(), 7.into(), 9.into())),
        ]
    } else {
        [
            (Real::zero(), Point3::new(5.into(), 0.into(), 2.into())),
            (quarter(), Point3::new(0.into(), 5.into(), 2.into())),
        ]
    };
    for (u, expected) in samples {
        let actual = surface.point_at(&Point2::new(u, v.clone())).unwrap();
        assert!(
            hyperlimit::point3_equal(&actual, &expected, hyperlimit::PredicatePolicy::STRICT)
                .value()
                .unwrap()
        );
    }
    for edge in model.edges() {
        assert!(
            edge.start()
                .point()
                .coincides_with(&edge.curve().point_at(edge.domain().start()).unwrap())
                .unwrap()
        );
        assert!(
            edge.end()
                .point()
                .coincides_with(&edge.curve().point_at(edge.domain().end()).unwrap())
                .unwrap()
        );
    }
}

pub fn selected_revolution_supports_reversal_reflection_and_subdivision() {
    for reversed in [false, true] {
        let profile = selected_profile();
        let profile = if reversed {
            profile.reversed().unwrap()
        } else {
            profile
        };
        let sheet = builder::revolution_patch(
            profile,
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            quarter(),
        )
        .unwrap();
        let cold = replay(sheet.model());
        assert_eq!(
            cold.edges()
                .filter(|e| e.curve().kind() == Curve3Kind::Rotation)
                .count(),
            2
        );
        check_patch_samples(&cold, reversed, false);
        let transformed = replay(&cold.transformed(&reflected_similarity()).unwrap());
        check_patch_samples(&transformed, reversed, true);
        let meridian = transformed
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::PlanarImage)
            .unwrap();
        let selected = profile_ending_at(3, 3.into(), q(2, 3), q(3, 4));
        let parameter = if reversed {
            selected.reversed().unwrap().domain().start().clone()
        } else {
            selected.domain().end().clone()
        };
        assert!(parameter.try_real().is_none());
        let split = meridian.split(parameter).unwrap();
        let split = replay(split.model());
        assert_eq!(split.edges().len(), 5);
        let latitude = split
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::Rotation)
            .unwrap();
        let split = latitude
            .split((quarter() / Real::from(2)).unwrap().into())
            .unwrap();
        let split = replay(split.model());
        assert_eq!(split.edges().len(), 6);
        check_patch_samples(&split, reversed, true);
    }
}

pub fn selected_revolution_rejects_axis_contacts_and_invalid_domains() {
    let crossing = profile_ending_at(2, (-2).into(), q(3, 4), q(7, 8));
    // At t=1/2 this full nonlinear parallel has radius zero, despite its
    // nonzero endpoint radii. Endpoint-only clearance would admit it.
    assert!(matches!(
        builder::revolution_patch(
            crossing,
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            quarter()
        ),
        Err(ConstructionError::ProfileCrossesRevolutionAxis)
    ));
    assert!(matches!(
        builder::revolution_patch(
            selected_profile(),
            Point3::origin(),
            Vector3::zeros(),
            Real::zero(),
            quarter()
        ),
        Err(ConstructionError::Build(BuildError::Geometry(
            GeometryError::InvalidRevolutionAxis
        )))
    ));
    assert!(matches!(
        builder::revolution_patch(
            selected_profile(),
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            Real::zero()
        ),
        Err(ConstructionError::Build(BuildError::Geometry(
            GeometryError::InvalidParameterDomain
        )))
    ));
    assert!(matches!(
        builder::revolution_patch(
            selected_profile(),
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            Real::tau()
        ),
        Err(ConstructionError::RevolutionPatchSweepTooLarge)
    ));
    assert!(matches!(
        Curve3::rotation(
            Point3::new(0.into(), 0.into(), 2.into()).into(),
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            quarter()
        ),
        Err(GeometryError::SingularSurfaceParameter)
    ));
}

pub fn selected_revolution_archives_reject_wrong_images_and_recover() {
    let sheet = builder::revolution_patch(
        selected_profile(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        quarter(),
    )
    .unwrap();
    let good: serde_json::Value = serde_json::from_str(&sheet.model().to_json().unwrap()).unwrap();
    for mode in 0..6 {
        let mut bad = good.clone();
        if mode < 4 {
            let rotation = bad["curves"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|v| v.get_mut("Rotation"))
                .unwrap();
            match mode {
                0 => rotation["angle_direction"] = serde_json::json!(0),
                1 => rotation["angle_offset"] = serde_json::json!(Real::one()),
                2 => rotation["axis"][2] = serde_json::json!(Real::from(2)),
                _ => rotation["axis_origin"][0] = serde_json::json!(Real::one()),
            }
        } else {
            let map = bad["edge_uses"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|v| v["parameter_correspondence"].get_mut("Coordinate"))
                .unwrap();
            map["scale"] = serde_json::json!(Real::from(if mode == 4 { 0 } else { 2 }));
        }
        let error = RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
            .and_then(RawModel::validate)
            .unwrap_err();
        assert!(
            matches!(
                error,
                PersistenceError::Build(_) | PersistenceError::Validation(_)
            ),
            "{error:?}"
        );
        replay(sheet.model());
    }
}

pub fn selected_revolution_preserves_certainty_and_budget_recovery() {
    let sheet = builder::revolution_patch(
        selected_profile(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        quarter(),
    )
    .unwrap();
    let json = sheet.model().to_json().unwrap();
    assert!(matches!(
        sheet
            .model()
            .transformed(&Matrix4::affine_nonuniform_scale([
                2.into(),
                1.into(),
                1.into(),
            ])),
        Err(GeometryError::UnsupportedTransform)
    ));

    // One standalone authored orbit charges its curve record plus its origin.
    // The metadata limit must fire before decoding a deliberately invalid scalar.
    let mut builder = ModelBuilder::new();
    builder
        .curve(
            Curve3::rotation(
                Point3::new(1.into(), 0.into(), 0.into()).into(),
                Point3::origin(),
                Vector3::z(),
                Real::zero(),
                quarter(),
            )
            .unwrap(),
        )
        .unwrap();
    let native = builder.finish().unwrap();
    let text = native.to_json().unwrap();
    let mut bad: serde_json::Value = serde_json::from_str(&text).unwrap();
    bad["curves"][0]["Rotation"]["axis"][0] = serde_json::json!("invalid scalar");
    let small = ArchiveLimits {
        records: 1,
        ..ArchiveLimits::default()
    };
    for text in [&text, &bad.to_string()] {
        assert!(matches!(
            RawModel::from_json(text, &small),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
                resource: ArchiveResource::Records,
                limit: 1,
                observed: 2,
            }))
        ));
    }
    let enough = ArchiveLimits {
        records: 2,
        ..ArchiveLimits::default()
    };
    assert_eq!(
        RawModel::from_json(&text, &enough)
            .unwrap()
            .validate()
            .unwrap()
            .to_json()
            .unwrap(),
        text
    );
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
    });
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || replay(sheet.model()));
        assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
    }
}

pub fn retained_rotation_closed_edges_split_and_replay() {
    let source = selected_profile();
    let point = source.point_at(source.domain().end()).unwrap();
    let curve = Curve3::rotation(
        point.clone(),
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        Real::tau(),
    )
    .unwrap();
    let domain = curve.domain().clone();
    let mut builder = ModelBuilder::new();
    let vertex = builder.vertex(point).unwrap();
    let curve = builder.curve(curve).unwrap();
    builder.edge(&vertex, &vertex, &curve, domain).expect(
        "a nondegenerate retained orbit has the same exact period proof as a native circle",
    );
    let model = replay(&builder.finish().unwrap());
    let edge = model.edges().next().unwrap();
    assert_eq!(edge.start(), edge.end());
    let split = edge.split(Real::pi().into()).unwrap();
    let split = replay(split.model());
    assert_eq!(split.edges().len(), 2);
    assert_eq!(split.vertices().len(), 2);
    let reflected = replay(&split.transformed(&reflected_similarity()).unwrap());
    for edge in reflected.edges() {
        assert!(
            edge.start()
                .point()
                .coincides_with(&edge.curve().point_at(edge.domain().start()).unwrap())
                .unwrap()
        );
        assert!(
            edge.end()
                .point()
                .coincides_with(&edge.curve().point_at(edge.domain().end()).unwrap())
                .unwrap()
        );
    }
}

pub fn selected_revolution_oblique_axes_tangencies_and_exact_gaps() {
    use std::cmp::Ordering;
    let source = selected_profile();
    let definition = source.planar_geometry().unwrap();
    let axis = Vector3::new([0.into(), q(3, 5), q(4, 5)]);
    let axis_origin = Point3::new(7.into(), (-2).into(), 5.into());
    let profile = Curve3::planar_image(
        definition.source.clone(),
        Point3::new(10.into(), (-2).into(), 5.into()),
        Vector3::x(),
        axis.clone(),
    )
    .unwrap();
    let sheet =
        builder::revolution_patch(profile, axis_origin, axis, -quarter(), quarter()).unwrap();
    let model = replay(sheet.model());
    let surface = model.faces().next().unwrap().surface();
    for (angle, expected) in [
        (-quarter(), Point3::new(7.into(), -q(24, 5), q(48, 5))),
        (Real::zero(), Point3::new(12.into(), -q(4, 5), q(33, 5))),
        (quarter(), Point3::new(7.into(), q(16, 5), q(18, 5))),
    ] {
        let actual = surface.point_at(&Point2::new(angle, q(1, 2))).unwrap();
        assert!(
            hyperlimit::point3_equal(&actual, &expected, hyperlimit::PredicatePolicy::STRICT)
                .value()
                .unwrap()
        );
    }
    // r=(2t-1)^2 and z=t meet the axis tangentially at t=1/2.
    let (start, end) = definition.source.parameter_endpoints();
    let tangent = Curve2::from(QuadraticBezier2::new(
        hypercurve::Point2::new(1.into(), 0.into()),
        hypercurve::Point2::new((-1).into(), q(1, 2)),
        hypercurve::Point2::new(1.into(), 1.into()),
    ))
    .restrict_source(hypercurve::CurveParameterRange2::try_new(start.clone(), end.clone()).unwrap())
    .unwrap();
    let tangent =
        Curve3::planar_image(tangent, Point3::origin(), Vector3::x(), Vector3::z()).unwrap();
    assert!(matches!(
        builder::revolution_patch(
            tangent,
            Point3::origin(),
            Vector3::z(),
            Real::zero(),
            quarter()
        ),
        Err(ConstructionError::ProfileCrossesRevolutionAxis)
    ));
    // An exact positive radius of 10^-12 remains a nondegenerate orbit even
    // while the independent height is retained as a selected cubic root.
    let gap = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let line = Curve3::line(
        Point3::new(gap.clone(), 0.into(), 0.into()),
        Point3::new(gap.clone(), 0.into(), 1.into()),
    )
    .unwrap();
    let point = line.point_at(source.domain().end()).unwrap();
    let orbit = Curve3::rotation(
        point,
        Point3::origin(),
        Vector3::z(),
        Real::zero(),
        quarter(),
    )
    .unwrap();
    let endpoint = orbit.point_at(orbit.domain().end()).unwrap();
    assert_eq!(
        endpoint
            .compare_linear_form(&Vector3::x(), &Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        endpoint.compare_linear_form(&Vector3::y(), &gap).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        endpoint
            .compare_linear_form(&Vector3::y(), &Real::zero())
            .unwrap(),
        Ordering::Greater
    );
}
