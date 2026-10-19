use hyperbrep::*;
use hypercurve::{
    BezierParameter2, BezierParameterRange2, Curve2, CurveCertainty, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, Point2, PredicatePolicy, QuadraticBezier2, Real,
};

// Original full parallel-cap diagnostic, including its quadratic straight
// closing segments. A native-line simplification is not this acceptance input.
pub fn original_parallel_cap() -> CurveRegion2 {
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 2),
        Point2::from_values(4, 0),
    )
    .parallel_left(Real::one())
    .unwrap();
    let left = parallel.point_at(&Real::zero()).unwrap();
    let right = parallel.point_at(&Real::one()).unwrap();
    let analytic = Curve2::try_analytic_parallel(
        parallel,
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(Real::one()),
            BezierParameter2::Exact(Real::zero()),
        )
        .unwrap(),
    )
    .unwrap();
    let lower_left = Point2::new(left.x().clone(), (-2).into());
    let lower_right = Point2::new(right.x().clone(), (-2).into());
    let quadratic_line = |a: Point2, b: Point2| {
        let midpoint = a.lerp(&b, (Real::one() / Real::from(2)).unwrap());
        Curve2::from(QuadraticBezier2::new(a, midpoint, b))
    };
    let path = CurvePath2::try_new(vec![
        analytic,
        quadratic_line(left, lower_left.clone()),
        quadratic_line(lower_left, lower_right.clone()),
        quadratic_line(lower_right, right),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap()
}

pub fn original_nonlinear_parallel_cap_extrudes_and_replays() {
    let region = original_parallel_cap();
    let outcome = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
        builder::extrude_curve_region(&region, Real::zero(), Real::from(3))
    });
    assert_eq!(outcome.certainty(), CurveCertainty::Certified);
    let bodies = outcome.into_unverified().unwrap();
    assert_eq!(bodies.len(), 1);
    for wire in bodies.model().wires().take(2) {
        let curves = wire
            .edge_uses()
            .map(|usage| usage.pcurve().geometry().curve().clone())
            .collect();
        let path = CurvePath2::try_new(curves).unwrap();
        path.simple_loop_orientation().unwrap();
        let curves = path.curves();
        let contact = curves[0].intersect_curve(&curves[1]).unwrap();
        assert!(contact.is_complete());
        assert_eq!(contact.contacts().len(), 1);
        assert_eq!(
            contact.contacts()[0]
                .first()
                .parameter()
                .unwrap()
                .compare(curves[0].parameter_endpoints().1)
                .unwrap(),
            std::cmp::Ordering::Equal
        );
    }
    let json = bodies.model().to_json().unwrap();
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("HYPERBREP_PLANAR_LIFT_ARTIFACT_DIR") {
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            std::path::PathBuf::from(directory).join("original-cap.json"),
            &json,
        )
        .unwrap();
    }
    let rebuilt = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(rebuilt.to_json().unwrap(), json);
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal)
    );
}

fn point_equal(a: &Point3, b: &Point3) {
    equal(&a.x, &b.x);
    equal(&a.y, &b.y);
    equal(&a.z, &b.z);
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("HYPERBREP_PLANAR_LIFT_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            std::path::PathBuf::from(directory).join(format!(
                "replay-{:03}.json",
                NEXT.fetch_add(1, Ordering::Relaxed)
            )),
            &json,
        )
        .unwrap();
    }
    cold
}

fn arch(a: Real, b: Real) -> Curve2 {
    Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(2, 2),
            Point2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(BezierParameter2::Exact(a), BezierParameter2::Exact(b))
            .unwrap(),
    )
    .unwrap()
}

pub fn planar_images_preserve_exact_frame_and_chart() {
    for (a, b) in [(0, 1), (1, 0), (-1, 2), (2, -1)] {
        let source = arch(a.into(), b.into());
        let origin = Point3::new(5.into(), (-2).into(), 4.into());
        let u = Vector3::new([2.into(), 0.into(), 1.into()]);
        let v = Vector3::new([1.into(), 3.into(), 0.into()]);
        let image = Curve3::planar_image(source.clone(), origin.clone(), u, v).unwrap();
        assert_eq!(image.kind(), Curve3Kind::PlanarImage);
        assert_eq!(image.planar_geometry().unwrap().source, &source);
        let mut builder = ModelBuilder::new();
        builder.curve(image).unwrap();
        let cold = replay(&builder.finish().unwrap());
        let image = cold.curves().next().unwrap().geometry();
        // At source t=1/2 the offset is (2,2); derivative is (6,0).
        let midpoint = Point3::new(11.into(), 4.into(), 6.into());
        point_equal(
            image
                .point_at(&q(1, 2).into())
                .unwrap()
                .try_coordinates()
                .unwrap(),
            &midpoint,
        );
        let derivative = image.derivative_at(&q(1, 2).into(), 1).unwrap();
        let expected = Vector3::new([(12 * (b - a)).into(), 0.into(), (6 * (b - a)).into()]);
        for i in 0..3 {
            equal(&derivative.try_coordinates().unwrap()[i], &expected[i]);
        }
        let CurveParameterLocation::Parameters(parameters) =
            image.parameters_of(&midpoint).unwrap()
        else {
            panic!("point membership");
        };
        assert_eq!(parameters.len(), 1);
        assert_eq!(
            parameters[0].compare(&q(1, 2).into()).unwrap(),
            std::cmp::Ordering::Equal
        );
        assert!(matches!(
            image
                .parameters_of(&(midpoint.clone() + Vector3::y()))
                .unwrap(),
            CurveParameterLocation::None
        ));
        let reversed = image.reversed().unwrap();
        point_equal(&reversed.start().unwrap(), &image.end().unwrap());
        let (left, right) = image.split_at(&q(1, 2)).unwrap();
        point_equal(&left.end().unwrap(), &midpoint);
        point_equal(&right.start().unwrap(), &midpoint);
        let part = image.subcurve(&q(1, 4), &q(3, 4)).unwrap();
        point_equal(
            part.point_at(&q(1, 2).into())
                .unwrap()
                .try_coordinates()
                .unwrap(),
            &midpoint,
        );
        let bounds = image.bounds().unwrap();
        for (value, low, high) in [
            (&midpoint.x, &bounds.mins.x, &bounds.maxs.x),
            (&midpoint.y, &bounds.mins.y, &bounds.maxs.y),
            (&midpoint.z, &bounds.mins.z, &bounds.maxs.z),
        ] {
            assert_ne!(
                hyperlimit::compare_reals(value, low, STRICT_PREDICATES)
                    .value()
                    .unwrap(),
                std::cmp::Ordering::Less
            );
            assert_ne!(
                hyperlimit::compare_reals(value, high, STRICT_PREDICATES)
                    .value()
                    .unwrap(),
                std::cmp::Ordering::Greater
            );
        }
        assert!(matches!(
            image.derivative_at(&q(1, 2).into(), 0),
            Err(GeometryError::InvalidDerivativeOrder)
        ));
        assert!(image.derivative_at(&q(1, 2).into(), 2).is_err());
    }
}

fn location(model: &Model, point: &Point3) -> SolidPointLocation {
    let mut result = SolidPointLocation::Outside;
    for solid in model.solids() {
        match solid.classify_point(point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => result = SolidPointLocation::Boundary,
            SolidPointLocation::Outside => {}
        }
    }
    result
}

fn check_cap(model: &Model, transform: Option<&Matrix4>) {
    let gap: Real = "1e-12".parse().unwrap();
    for (y, z, expected) in [
        (Real::zero(), q(3, 2), SolidPointLocation::Inside),
        (2.into(), q(3, 2), SolidPointLocation::Boundary),
        (Real::from(2) - &gap, q(3, 2), SolidPointLocation::Inside),
        (Real::from(2) + &gap, q(3, 2), SolidPointLocation::Outside),
        ((-2).into(), q(3, 2), SolidPointLocation::Boundary),
        (Real::from(-2) - &gap, q(3, 2), SolidPointLocation::Outside),
        (0.into(), 0.into(), SolidPointLocation::Boundary),
        (0.into(), -&gap, SolidPointLocation::Outside),
        (0.into(), 3.into(), SolidPointLocation::Boundary),
        (0.into(), Real::from(3) + &gap, SolidPointLocation::Outside),
    ] {
        let point = Point3::new(2.into(), y, z);
        let point =
            transform.map_or_else(|| point.clone(), |t| t.transform_point3(&point).unwrap());
        assert_eq!(location(model, &point), expected, "{point:?}");
    }
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

pub fn parallel_cap_classification_and_edge_split() {
    let bodies =
        builder::extrude_curve_region(&original_parallel_cap(), 0.into(), 3.into()).unwrap();
    check_cap(bodies.model(), None);
    check_cap(&replay(bodies.model()), None);
    let edge = bodies
        .model()
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::PlanarImage)
        .unwrap();
    let split = edge.split(q(1, 2).into()).unwrap();
    check_cap(&replay(split.model()), None);
}

pub fn parallel_cap_affine_transform_and_cold_replay() {
    let bodies =
        builder::extrude_curve_region(&original_parallel_cap(), 0.into(), 3.into()).unwrap();
    let transform = Matrix4::from_row_major([
        (-2).into(),
        1.into(),
        0.into(),
        7.into(),
        0.into(),
        3.into(),
        1.into(),
        (-1).into(),
        0.into(),
        0.into(),
        2.into(),
        5.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let transformed = bodies.model().transformed(&transform).unwrap();
    check_cap(&transformed, Some(&transform));
    check_cap(&replay(&transformed), Some(&transform));
}

pub fn parallel_cap_repeated_slab_booleans() {
    let region = original_parallel_cap();
    let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
    let mut current = boolean::union(&BodySet::empty(), &original).unwrap();
    for _ in 0..2 {
        let middle = boolean::intersection(&current, &cutter).unwrap();
        let ends = boolean::difference(&current, &cutter).unwrap();
        assert_eq!(ends.len(), 2);
        for (z, in_middle) in [(q(1, 2), false), (q(3, 2), true), (q(5, 2), false)] {
            let point = Point3::new(2.into(), 0.into(), z);
            assert_eq!(
                location(&replay(middle.model()), &point),
                if in_middle {
                    SolidPointLocation::Inside
                } else {
                    SolidPointLocation::Outside
                }
            );
            assert_eq!(
                location(&replay(ends.model()), &point),
                if in_middle {
                    SolidPointLocation::Outside
                } else {
                    SolidPointLocation::Inside
                }
            );
        }
        current = boolean::union(&ends, &middle).unwrap();
        check_cap(&replay(current.model()), None);
        assert!(boolean::difference(&current, &original).unwrap().is_empty());
        assert!(boolean::difference(&original, &current).unwrap().is_empty());
        let empty = boolean::difference(&current, &current).unwrap();
        assert!(empty.is_empty());
        assert_eq!(replay(empty.model()).counts().solids, 0);
        current = boolean::union(&empty, &current).unwrap();
    }
    check_cap(
        boolean::difference(&current, &BodySet::empty())
            .unwrap()
            .model(),
        None,
    );
    assert!(
        boolean::intersection(&current, &BodySet::empty())
            .unwrap()
            .is_empty()
    );
    assert!(
        boolean::difference(&BodySet::empty(), &current)
            .unwrap()
            .is_empty()
    );
}

pub fn parallel_extrusion_point_queries() {
    let profile = Curve3::planar_image(
        arch(1.into(), 0.into()),
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap();
    let surface =
        Surface::extrusion(profile, Vector3::new([1.into(), 0.into(), 3.into()])).unwrap();
    let (location, v) = surface
        .extrusion_parameters_of(&Point3::new(3.into(), 2.into(), 3.into()))
        .unwrap();
    equal(&v, &Real::one());
    let CurveParameterLocation::Parameters(parameters) = location else {
        panic!("extrusion point membership");
    };
    assert_eq!(parameters.len(), 1);
    assert_eq!(
        parameters[0].compare(&q(1, 2).into()).unwrap(),
        std::cmp::Ordering::Equal
    );
    let (outside, _) = surface
        .extrusion_parameters_of(&Point3::new(3.into(), 3.into(), 3.into()))
        .unwrap();
    assert!(matches!(outside, CurveParameterLocation::None));
}

pub fn authored_parallel_paths_share_admission() {
    let region = original_parallel_cap();
    let path = CurvePath2::try_new(region.boundary_loops()[0].curves().to_vec()).unwrap();
    for path in [path.clone(), path.reversed().unwrap()] {
        for bodies in [
            builder::extrude_path(&path, 0.into(), 3.into()).unwrap(),
            builder::extrude_path_region(&path, &[], 0.into(), 3.into()).unwrap(),
            builder::extrude_path_regions(&[(path.clone(), Vec::new())], 0.into(), 3.into())
                .unwrap(),
        ] {
            check_cap(&replay(bodies.model()), None);
        }
        let face =
            builder::planar_face(&path, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let cold = replay(face.model());
        assert_eq!(cold.faces().count(), 1);
        assert_eq!(cold.solids().count(), 0);
    }
}

pub fn planar_image_archives_reject_forged_interior_and_frames() {
    use serde_json::{Value, json};
    let bodies =
        builder::extrude_curve_region(&original_parallel_cap(), 0.into(), 3.into()).unwrap();
    let good: Value = serde_json::from_str(&bodies.model().to_json().unwrap()).unwrap();
    for mode in 0..5 {
        let mut raw = good.clone();
        let image = raw["curves"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find_map(|c| c.get_mut("PlanarImage"))
            .unwrap();
        match mode {
            0 => {
                // Preserve both exact endpoints, but alter the whole interior.
                // Both source endpoints have y=1/sqrt(2); y -> 2*y-1/sqrt(2).
                let y = (Real::one() / Real::from(2).sqrt().unwrap()).unwrap();
                image["v"][1] = json!(Real::from(2));
                image["origin"][1] = json!(-y);
            }
            1 => image["v"] = image["u"].clone(),
            2 => image["source_chart"][0] = json!(Real::zero()),
            3 => image["source_chart"][1] = json!(Real::from(3)),
            _ => image["source"]["AnalyticParallel"]["range"][0] = json!({"Selected":1000000}),
        }
        let result = RawModel::from_json(&raw.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate();
        assert!(result.is_err(), "forgery mode {mode}");
        if mode == 0 {
            assert!(
                matches!(
                    result,
                    Err(PersistenceError::Build(BuildError::EdgeUseSupportMismatch))
                        | Err(PersistenceError::Build(BuildError::Geometry(_)))
                ),
                "{result:?}"
            );
        }
    }
    check_cap(&replay(bodies.model()), None);
}

pub fn planar_image_limits_and_typed_frontiers_recover() {
    use serde_json::json;
    let image = Curve3::planar_image(
        arch(0.into(), 1.into()),
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap();
    let mut builder = ModelBuilder::new();
    builder.curve(image.clone()).unwrap();
    let model = builder.finish().unwrap();
    let json_text = model.to_json().unwrap();
    for (limits, resource, observed) in [
        (
            ArchiveLimits {
                records: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            2,
        ),
        (
            ArchiveLimits {
                records: 2,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            3,
        ),
        (
            ArchiveLimits {
                degree: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Degree,
            2,
        ),
        (
            ArchiveLimits {
                geometry_elements: 2,
                ..ArchiveLimits::default()
            },
            ArchiveResource::GeometryElements,
            3,
        ),
    ] {
        let Err(PersistenceError::ResourceLimit(error)) = RawModel::from_json(&json_text, &limits)
        else {
            panic!("admission must charge nested source");
        };
        assert_eq!(error.resource, resource);
        assert_eq!(error.observed, observed);
        replay(&model);
    }
    let mut malformed: serde_json::Value = serde_json::from_str(&json_text).unwrap();
    malformed["curves"][0]["PlanarImage"]["origin"][0] = json!("not a scalar");
    assert!(matches!(
        RawModel::from_json(
            &malformed.to_string(),
            &ArchiveLimits {
                records: 1,
                ..ArchiveLimits::default()
            }
        ),
        Err(PersistenceError::ResourceLimit(_))
    ));
    assert!(matches!(
        Curve3::planar_image(
            arch(0.into(), 1.into()),
            Point3::origin(),
            Vector3::x(),
            Vector3::x()
        ),
        Err(GeometryError::DegeneratePlaneBasis)
    ));
    let selected = hypercurve::BezierAlgebraicParameter2::try_isolate(
        hypercurve::BezierParameterPolynomial::try_new_power_basis(vec![
            (-1).into(),
            0.into(),
            0.into(),
            2.into(),
        ])
        .unwrap(),
        hypercurve::BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
    )
    .unwrap();
    let source = Curve2::try_analytic_parallel(
        arch(0.into(), 1.into())
            .analytic_parallel()
            .unwrap()
            .parallel()
            .clone(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(0.into()),
            BezierParameter2::Algebraic(selected),
        )
        .unwrap(),
    )
    .unwrap();
    let selected_parameter = Pcurve::new(source.clone()).domain_end().unwrap();
    // Selected spatial evaluation is now a required exact success; the
    // constructor's selected-domain capability below is checked separately.
    let point = image.point_at(&selected_parameter).unwrap();
    assert!(point.try_coordinates().is_none());
    assert_eq!(
        point
            .compare_linear_form(&Vector3::z(), &Real::zero())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    // Capability migration: the same selected source now retains its finite
    // domain. Coordinate materialization remains an explicit capability.
    let selected =
        Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    assert!(selected.domain().end().try_real().is_none());
    assert!(
        selected
            .point_at(selected.domain().end())
            .unwrap()
            .try_coordinates()
            .is_none()
    );
    let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    // The entire finite planar image is now certified on its supporting plane.
    assert!(matches!(
        plane.intersect_curve(&image).unwrap(),
        CurveSurfaceIntersection::Contained
    ));
    assert!(plane.planar_pcurve(&image).unwrap().is_some());
    let tangent = Surface::extrusion(image, Vector3::x()).unwrap();
    assert!(matches!(
        tangent.extrusion_parameters_of(&Point3::origin()),
        Err(GeometryError::UnsupportedIntersection)
    ));
    let region = original_parallel_cap();
    let budget = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| builder::extrude_curve_region(&region, 0.into(), 3.into()));
    match budget.result {
        Ok(result) => check_cap(result.unwrap().model(), None),
        Err(error) => assert_eq!(error.limit, 0),
    }
    let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    assert!(matches!(
        bodies.model().solids().next().unwrap().volume(),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    ));
    check_cap(&replay(bodies.model()), None);
}

pub fn parallel_caps_keep_holes_and_components() {
    let outer = CurvePath2::try_new(
        original_parallel_cap().boundary_loops()[0]
            .curves()
            .to_vec(),
    )
    .unwrap();
    let points = [
        Point2::from_values(1, -1),
        Point2::from_values(3, -1),
        Point2::from_values(3, 0),
        Point2::from_values(1, 0),
    ];
    let hole = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    hypercurve::LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        10.into(),
        0.into(),
    )
    .unwrap();
    let distant = CurvePath2::try_new(
        outer
            .curves()
            .iter()
            .map(|c| c.transform_similarity(&shift).unwrap())
            .collect(),
    )
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[outer.clone(), hole.clone(), distant],
        &[
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
            CurveRegionLoopRole::Material,
        ],
        &[FillRule::NonZero; 3],
    )
    .unwrap();
    let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    assert_eq!(bodies.len(), 2);
    let cold = replay(bodies.model());
    for (x, y, expected) in [
        (q(2, 1), q(-1, 2), SolidPointLocation::Outside),
        (q(1, 1), q(-1, 2), SolidPointLocation::Boundary),
        (q(2, 1), q(1, 1), SolidPointLocation::Inside),
        (q(12, 1), q(0, 1), SolidPointLocation::Inside),
    ] {
        assert_eq!(location(&cold, &Point3::new(x, y, 1.into())), expected);
    }
    let authored = builder::extrude_path_region(&outer, &[hole], 0.into(), 3.into()).unwrap();
    assert_eq!(
        location(
            &replay(authored.model()),
            &Point3::new(2.into(), q(-1, 2), 1.into())
        ),
        SolidPointLocation::Outside
    );
    // Crossing closure segments must not obtain a simple-loop certificate.
    let parallel = arch(1.into(), 0.into());
    let owner = parallel.analytic_parallel().unwrap();
    let left = owner.parallel().point_at(&Real::zero()).unwrap();
    let right = owner.parallel().point_at(&Real::one()).unwrap();
    let ll = Point2::new(left.x().clone(), (-2).into());
    let lr = Point2::new(right.x().clone(), (-2).into());
    let line = |a, b| Curve2::from(hypercurve::LineSeg2::try_new(a, b).unwrap());
    let crossing = CurvePath2::try_new(vec![
        parallel,
        line(left, lr.clone()),
        line(lr, ll.clone()),
        line(ll, right),
    ])
    .unwrap();
    assert!(builder::extrude_path(&crossing, 0.into(), 3.into()).is_err());
}

pub fn planar_lifts_share_native_source_evaluation() {
    let p = Point2::from_values;
    let sources = vec![
        Curve2::from(hypercurve::LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 2), p(4, 0))),
        Curve2::from(hypercurve::CubicBezier2::new(
            p(0, 0),
            p(1, 2),
            p(3, 2),
            p(4, 0),
        )),
        Curve2::from(
            hypercurve::RationalBezier2::try_new(
                vec![p(0, 0), p(2, 2), p(4, 0)],
                vec![1.into(), 2.into(), 1.into()],
            )
            .unwrap(),
        ),
        Curve2::try_nurbs(
            2,
            vec![p(0, 0), p(2, 2), p(4, 0)],
            vec![1.into(), 2.into(), 1.into()],
            vec![2.into(), 2.into(), 2.into(), 4.into(), 4.into(), 4.into()],
        )
        .unwrap(),
        Curve2::from(
            hypercurve::CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap(),
        ),
    ];
    for source in sources {
        if matches!(
            source.family(),
            hypercurve::CurveFamily2::QuadraticBezier | hypercurve::CurveFamily2::CubicBezier
        ) {
            let degree = if source.family() == hypercurve::CurveFamily2::QuadraticBezier {
                2
            } else {
                3
            };
            let mut builder = ModelBuilder::new();
            builder.pcurve(Pcurve::new(source.clone())).unwrap();
            let model = builder.finish().unwrap();
            let json = model.to_json().unwrap();
            assert_eq!(
                replay(&model).pcurves().next().unwrap().geometry().kind(),
                source.family()
            );
            for limits in [
                ArchiveLimits {
                    degree: degree - 1,
                    ..ArchiveLimits::default()
                },
                ArchiveLimits {
                    geometry_elements: degree,
                    ..ArchiveLimits::default()
                },
            ] {
                assert!(matches!(
                    RawModel::from_json(&json, &limits),
                    Err(PersistenceError::ResourceLimit(_))
                ));
            }
        }
        let image = Curve3::planar_image(
            source.clone(),
            Point3::new(0.into(), 0.into(), 7.into()),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        let mut builder = ModelBuilder::new();
        builder.curve(image.clone()).unwrap();
        let cold = replay(&builder.finish().unwrap());
        for image in [&image, cold.curves().next().unwrap().geometry()] {
            let chart = image.planar_geometry().unwrap();
            for s in [Real::zero(), q(1, 4), q(1, 2), Real::one()] {
                let t = chart.source_offset + chart.source_scale * &s;
                let expected = source.point_at(&t.into()).unwrap();
                let actual = image.point_at(&s.into()).unwrap();
                let actual = actual.try_coordinates().unwrap();
                assert!(
                    expected
                        .coincides_with(&Point2::new(actual.x.clone(), actual.y.clone()).into())
                        .unwrap()
                );
                equal(&actual.z, &Real::from(7));
            }
            // Entire-curve bounds are conservative for an affine subchart too.
            image.bounds().unwrap();
            image.derivative_at(&q(1, 2).into(), 1).unwrap();
            image.reversed().unwrap().split_at(&q(1, 2)).unwrap();
        }
    }
}
