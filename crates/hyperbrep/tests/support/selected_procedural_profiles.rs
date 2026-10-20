use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, BooleanOp, Curve2, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, LineSeg2, Point2 as Uv, QuadraticBezier2,
};
use std::cmp::Ordering;

#[allow(dead_code)]
#[path = "parallel_profile_extrusion.rs"]
pub(super) mod original;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn source() -> Curve2 {
    let root = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(vec![
            (-1).into(),
            0.into(),
            0.into(),
            2.into(),
        ])
        .unwrap(),
        BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
    )
    .unwrap();
    Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            Uv::from_values(0, 0),
            Uv::from_values(2, 2),
            Uv::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(q(1, 4)),
            BezierParameter2::Algebraic(root),
        )
        .unwrap(),
    )
    .unwrap()
}

fn lift(source: Curve2) -> Curve3 {
    Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().expect("encode retained definitions");
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .expect("admit retained archive syntax")
        .validate()
        .expect("validate retained archive geometry");
    assert_eq!(restored.to_json().unwrap(), json);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(folder) = std::env::var_os("HYPERBREP_SELECTED_PROCEDURAL_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        let index = NEXT.fetch_add(1, Ordering::Relaxed);
        std::fs::write(folder.join(format!("selected-{index:03}.json")), &json).unwrap();
    }
    restored
}

pub fn selected_origin_lines_retain_source_and_replay() {
    let source = lift(source());
    let origin = source.point_at(source.domain().end()).unwrap();
    let line =
        Curve3::translation(origin.clone(), Vector3::new([0.into(), 0.into(), 3.into()])).unwrap();
    assert_eq!(line.kind(), Curve3Kind::Translation);
    assert!(
        line.translation_geometry()
            .unwrap()
            .origin
            .coincides_with(&origin)
            .unwrap()
    );
    assert!(matches!(
        Curve3::translation(origin.clone(), Vector3::zeros()),
        Err(GeometryError::DegenerateLine)
    ));
    assert!(matches!(
        line.point_at(&Real::from(2).into()),
        Err(GeometryError::ParameterOutsideDomain)
    ));
    let middle = line.point_at(&q(1, 2).into()).unwrap();
    assert!(middle.try_coordinates().is_none());
    assert_eq!(
        middle.compare_coordinate(&origin, Axis::X).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        middle.compare_coordinate(&origin, Axis::Y).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        middle.compare_linear_form(&Vector3::z(), &q(3, 2)).unwrap(),
        Ordering::Equal
    );
    let (left, right) = line.split_at(&q(1, 2)).unwrap();
    assert!(
        left.point_at(left.domain().end())
            .unwrap()
            .coincides_with(&middle)
            .unwrap()
    );
    assert!(
        right
            .point_at(right.domain().start())
            .unwrap()
            .coincides_with(&middle)
            .unwrap()
    );
    let reversed = line.reversed().unwrap();
    assert!(
        reversed
            .point_at(reversed.domain().end())
            .unwrap()
            .coincides_with(&origin)
            .unwrap()
    );
    let mut builder = ModelBuilder::new();
    builder.vertex(origin).unwrap();
    builder.vertex(middle).unwrap();
    builder.curve(line).unwrap();
    let cold = replay(&builder.finish().unwrap());
    let geometry = cold.curves().next().unwrap();
    assert_eq!(geometry.geometry().kind(), Curve3Kind::Translation);
    geometry.geometry().bounds().unwrap();
    let reflected = replay(
        &cold
            .transformed(&Matrix4::from_row_major([
                (-2).into(),
                1.into(),
                0.into(),
                5.into(),
                0.into(),
                3.into(),
                1.into(),
                (-2).into(),
                0.into(),
                0.into(),
                2.into(),
                7.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]))
            .unwrap(),
    );
    assert_eq!(
        reflected.curves().next().unwrap().geometry().kind(),
        Curve3Kind::Translation
    );
}

pub fn selected_profile_domains_retain_endpoints_and_projection() {
    let source = source();
    let pcurve = Pcurve::new(source.clone());
    let curve = lift(source);
    assert!(curve.domain().end().try_real().is_none());
    assert_eq!(
        curve
            .domain()
            .start()
            .compare(&pcurve.domain_start().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        curve
            .domain()
            .end()
            .compare(&pcurve.domain_end().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    let endpoint = curve.point_at(curve.domain().end()).unwrap();
    assert!(endpoint.try_coordinates().is_none());
    assert_eq!(
        endpoint
            .compare_linear_form(&Vector3::z(), &Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert!(matches!(
        curve.point_at(&Real::one().into()),
        Err(GeometryError::ParameterOutsideDomain)
    ));
    assert!(matches!(
        curve.domain().represented_domain(),
        Err(GeometryError::UnrepresentableParameter)
    ));
    curve.bounds().unwrap();
    let (left, right) = curve.split_at(&q(1, 2)).unwrap();
    assert_eq!(
        left.domain().end().compare(&q(1, 2).into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        right.domain().end().compare(curve.domain().end()).unwrap(),
        Ordering::Equal
    );
    assert!(
        left.point_at(left.domain().end())
            .unwrap()
            .coincides_with(&right.point_at(right.domain().start()).unwrap())
            .unwrap()
    );
    let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let projected = plane.planar_pcurve(&curve).unwrap().unwrap();
    let (scale, offset) = projected.correspondence().affine_coefficients().unwrap();
    assert_eq!(scale, &Real::one());
    assert_eq!(offset, &Real::zero());
    assert!(
        projected
            .curve()
            .end()
            .coincides_with(&pcurve.curve().end())
            .unwrap()
    );
    let extruded = Surface::extrusion(curve.clone(), Vector3::z()).unwrap();
    let SurfaceParameterDomain::Closed(range) = extruded.domain().u() else {
        panic!("finite profile chart");
    };
    assert_eq!(
        range.end().compare(curve.domain().end()).unwrap(),
        Ordering::Equal
    );
    let sample = extruded
        .point_at(&{
            let uv = &Point2::new(q(1, 2), 3.into());
            hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
        })
        .map(|point| {
            point
                .try_coordinates()
                .expect("represented surface fixture")
                .clone()
        })
        .unwrap();
    assert_eq!(sample, Point3::new(2.into(), 2.into(), 3.into()));
}

pub fn selected_profile_domains_reverse_transform_and_replay() {
    let curve = lift(source());
    let reversed = curve.reversed().unwrap();
    assert!(reversed.domain().start().try_real().is_none());
    assert_eq!(
        reversed
            .domain()
            .start()
            .compare(&(-q(7, 8)).into())
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        reversed
            .domain()
            .start()
            .compare(&(-q(3, 4)).into())
            .unwrap(),
        Ordering::Less
    );
    let point = curve.point_at(curve.domain().end()).unwrap();
    assert!(
        point
            .coincides_with(&reversed.point_at(reversed.domain().start()).unwrap())
            .unwrap()
    );
    let twice = reversed.reversed().unwrap();
    assert_eq!(
        twice.domain().end().compare(curve.domain().end()).unwrap(),
        Ordering::Equal
    );
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
    for source in [curve, reversed] {
        let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let projected = plane.planar_pcurve(&source).unwrap().unwrap();
        let lifted = lift(projected.curve().clone());
        assert!(
            lifted
                .point_at(lifted.domain().start())
                .unwrap()
                .coincides_with(&source.point_at(source.domain().start()).unwrap())
                .unwrap()
        );
        let mut builder = ModelBuilder::new();
        builder
            .vertex(source.point_at(source.domain().start()).unwrap())
            .unwrap();
        builder
            .vertex(source.point_at(source.domain().end()).unwrap())
            .unwrap();
        builder.curve(source.clone()).unwrap();
        let cold = replay(&builder.finish().unwrap());
        let restored = cold.curves().next().unwrap();
        assert_eq!(
            restored
                .geometry()
                .domain()
                .start()
                .compare(source.domain().start())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            restored
                .geometry()
                .domain()
                .end()
                .compare(source.domain().end())
                .unwrap(),
            Ordering::Equal
        );
        let mapped = replay(&cold.transformed(&transform).unwrap());
        for (old, new) in cold.vertices().zip(mapped.vertices()) {
            assert!(
                old.point()
                    .transformed(&transform)
                    .unwrap()
                    .coincides_with(new.point())
                    .unwrap()
            );
        }
    }
}

pub fn clipped_parallel_cap() -> CurveRegion2 {
    let points = [
        Uv::from_values(0, -3),
        Uv::from_values(1, -3),
        Uv::from_values(1, 3),
        Uv::from_values(0, 3),
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
    let clip = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    original::original_parallel_cap()
        .boolean_region(&clip, BooleanOp::Intersection)
        .unwrap()
}

pub fn full_clipped_parallel_cap_constructs_and_replays() {
    let region = clipped_parallel_cap();
    assert!(
        region
            .boundary_loops()
            .iter()
            .flat_map(|b| b.curves())
            .any(|curve| curve.parameter_endpoints().0.scalar().is_none())
    );
    let bodies = builder::extrude_curve_region(&region, Real::zero(), Real::from(3)).unwrap();
    assert_eq!(bodies.len(), 1);
    let restored = replay(bodies.model());
    for model in [bodies.model(), &restored] {
        let solid = model.solids().next().unwrap();
        for (point, expected) in [
            (
                Point3::new(q(1, 2), 0.into(), 1.into()),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(2.into(), 0.into(), 1.into()),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(0.into(), 0.into(), 1.into()),
                SolidPointLocation::Boundary,
            ),
        ] {
            assert_eq!(solid.classify_point(&point).unwrap(), expected);
        }
    }
}

fn check_selected_cap(model: &Model, transform: Option<&Matrix4>) {
    let gap: Real = "1e-12".parse().unwrap();
    for (point, expected) in [
        (
            Point3::new(q(1, 2), 0.into(), 1.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(2.into(), 0.into(), 1.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(0.into(), 0.into(), 1.into()),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(-&gap, 0.into(), 1.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(gap.clone(), 0.into(), 1.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(1, 2), 0.into(), -gap),
            SolidPointLocation::Outside,
        ),
    ] {
        let point = transform.map_or_else(
            || point.clone(),
            |map| map.transform_point3(&point).unwrap(),
        );
        let mut actual = SolidPointLocation::Outside;
        for solid in model.solids() {
            match solid.classify_point(&point).unwrap() {
                SolidPointLocation::Inside => {
                    actual = SolidPointLocation::Inside;
                    break;
                }
                SolidPointLocation::Boundary => actual = SolidPointLocation::Boundary,
                SolidPointLocation::Outside => {}
            }
        }
        assert_eq!(actual, expected);
    }
}

pub fn selected_profile_solids_split_both_edge_directions() {
    let bodies =
        builder::extrude_curve_region(&clipped_parallel_cap(), 0.into(), 3.into()).unwrap();
    let cold = replay(bodies.model());
    let axial = cold
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::Translation)
        .unwrap();
    let split = axial.split(q(1, 2).into()).unwrap();
    assert_eq!(split.details().edge_uses().len(), 2);
    check_selected_cap(&replay(split.model()), None);
    let profile = cold
        .edges()
        .find(|edge| {
            edge.curve()
                .geometry()
                .planar_geometry()
                .is_some_and(|p| p.source.analytic_parallel().is_some())
        })
        .unwrap();
    let plane = Surface::plane(
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let CurveSurfaceIntersection::Points(points) =
        plane.intersect_curve(profile.curve().geometry()).unwrap()
    else {
        panic!("finite selected contact");
    };
    assert_eq!(points.len(), 1);
    let split = profile.split(points[0].parameter.clone()).unwrap();
    assert_eq!(split.details().edge_uses().len(), 2);
    check_selected_cap(&replay(split.model()), None);
}

pub fn selected_profile_solids_reflect_and_replay() {
    let bodies =
        builder::extrude_curve_region(&clipped_parallel_cap(), 0.into(), 3.into()).unwrap();
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
    let mapped = replay(bodies.model()).transformed(&transform).unwrap();
    check_selected_cap(&mapped, Some(&transform));
    check_selected_cap(&replay(&mapped), Some(&transform));
}

pub fn selected_profile_solids_continue_slab_booleans() {
    let region = clipped_parallel_cap();
    let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
    let cold = replay(original.model());
    let mut current = BodySet::from_solid(cold.solids().next().unwrap());
    for _ in 0..2 {
        let middle = boolean::intersection(&current, &cutter).unwrap();
        let ends = boolean::difference(&current, &cutter).unwrap();
        assert_eq!(ends.len(), 2);
        for (model, states) in [
            (middle.model(), [false, true, false]),
            (ends.model(), [true, false, true]),
        ] {
            let cold = replay(model);
            for (z, inside) in [q(1, 2), q(3, 2), q(5, 2)].into_iter().zip(states) {
                assert_eq!(
                    cold.solids().any(|solid| solid
                        .classify_point(&Point3::new(q(1, 2), 0.into(), z.clone()))
                        .unwrap()
                        == SolidPointLocation::Inside),
                    inside
                );
            }
        }
        let joined = boolean::union(&ends, &middle).unwrap();
        let cold = replay(joined.model());
        current = BodySet::from_solid(cold.solids().next().unwrap());
        check_selected_cap(current.model(), None);
        assert!(boolean::difference(&current, &original).unwrap().is_empty());
        assert!(boolean::difference(&original, &current).unwrap().is_empty());
        let empty = boolean::difference(&current, &current).unwrap();
        assert!(empty.is_empty());
        assert_eq!(replay(empty.model()).counts().solids, 0);
        current = boolean::union(&empty, &current).unwrap();
    }
    check_selected_cap(
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

pub fn selected_chord_charts_keep_original_boundary() {
    let region = clipped_parallel_cap();
    let mut changed = 0;
    for curve in region.boundary_loops().iter().flat_map(|b| b.curves()) {
        for source in [curve.clone(), curve.reversed().unwrap()] {
            let chart = source.scalar_chart().unwrap();
            assert_eq!(chart.source(), &source);
            let image = chart.curve();
            assert!(!image.parameter_domain().start().is_algebraic_chord());
            assert!(!image.parameter_domain().end().is_algebraic_chord());
            for (original, mapped) in [
                (
                    source.parameter_endpoints().0,
                    image.parameter_endpoints().0,
                ),
                (
                    source.parameter_endpoints().1,
                    image.parameter_endpoints().1,
                ),
            ] {
                assert_eq!(
                    chart.parameter(original).unwrap().compare(mapped).unwrap(),
                    Ordering::Equal
                );
                let uv = mapped
                    .affine_point(Real::one(), Real::zero(), Real::zero(), Real::from(3))
                    .unwrap();
                let u = uv
                    .affine_form(Real::one(), Real::zero(), Real::zero())
                    .parameter()
                    .unwrap();
                let v = uv
                    .affine_form(Real::zero(), Real::one(), Real::zero())
                    .parameter()
                    .unwrap();
                assert_eq!(u.compare(mapped).unwrap(), Ordering::Equal);
                assert_eq!(v.compare(&Real::from(3).into()).unwrap(), Ordering::Equal);
                assert_eq!(v.scalar(), Some(&Real::from(3)));
                assert!(
                    source
                        .point_at(original)
                        .unwrap()
                        .coincides_with(&image.point_at(mapped).unwrap())
                        .unwrap()
                );
            }
            if source.parameter_domain().start().is_algebraic_chord() {
                changed += 1;
                assert!(image.source_restriction().is_some());
            }
        }
    }
    assert_eq!(changed, 4);
}

fn first_record_mut<'a>(
    value: &'a mut serde_json::Value,
    key: &str,
) -> Option<&'a mut serde_json::Value> {
    match value {
        serde_json::Value::Object(object) => {
            if object.contains_key(key) {
                return object.get_mut(key);
            }
            object.values_mut().find_map(|v| first_record_mut(v, key))
        }
        serde_json::Value::Array(values) => {
            values.iter_mut().find_map(|v| first_record_mut(v, key))
        }
        _ => None,
    }
}

pub fn selected_profile_archives_reject_forged_proofs_and_recover() {
    use serde_json::{Value, json};
    let bodies =
        builder::extrude_curve_region(&clipped_parallel_cap(), 0.into(), 3.into()).unwrap();
    let good: Value = serde_json::from_str(&bodies.model().to_json().unwrap()).unwrap();
    let admit = |value: &Value| {
        RawModel::from_json(&value.to_string(), &ArchiveLimits::default())
            .and_then(RawModel::validate)
    };
    for mode in 0..7 {
        let mut raw = good.clone();
        match mode {
            0 => {
                first_record_mut(&mut raw["edge_uses"], "Coordinate").unwrap()["scale"] =
                    json!(Real::zero())
            }
            1 => {
                first_record_mut(&mut raw["edge_uses"], "Coordinate").unwrap()["scale"] =
                    json!(Real::from(2))
            }
            2 => {
                first_record_mut(&mut raw["curves"], "Translation").unwrap()["displacement"][0] =
                    json!(Real::one())
            }
            3 => {
                first_record_mut(&mut raw["selection"], "AffineForm").unwrap()["coefficients"][2] =
                    json!(Real::from(99))
            }
            4 => {
                first_record_mut(&mut raw["selection"], "AffineForm").unwrap()["point"]["Parallel"]
                    ["distance"] = json!(Real::from(2))
            }
            5 => {
                first_record_mut(&mut raw["selection"], "AffineForm").unwrap()["point"]["Parallel"]
                    ["parameter"] = json!({"Selected":1000000})
            }
            _ => {
                let parameters = raw["selection"]["parameters"].as_array_mut().unwrap();
                let (index, form) = parameters
                    .iter_mut()
                    .enumerate()
                    .find_map(|(i, p)| p.get_mut("AffineForm").map(|f| (i, f)))
                    .unwrap();
                form["point"]["Parallel"]["parameter"] = json!({"Selected":index});
            }
        }
        let result = admit(&raw);
        assert!(
            matches!(
                result,
                Err(PersistenceError::Build(_))
                    | Err(PersistenceError::Validation(_))
                    | Err(PersistenceError::InvalidReference { .. })
            ),
            "forgery {mode}: {result:?}"
        );
        if mode == 0 {
            assert!(matches!(
                result,
                Err(PersistenceError::Build(
                    BuildError::DegenerateParameterCorrespondence
                ))
            ));
        }
        if mode >= 5 {
            assert!(matches!(
                result,
                Err(PersistenceError::InvalidReference { .. })
            ));
        }
        check_selected_cap(&admit(&good).unwrap(), None);
    }
    // Upstream scalar import guards remain visible at the native archive boundary.
    let real: Value = serde_json::to_value(Real::one()).unwrap();
    for computable in [
        Value::Null,
        json!({"internal":{"Inverse":{"internal":{"Ratio":serde_json::to_value(Real::zero()).unwrap()["rational"].clone()}}}}),
    ] {
        let mut raw = good.clone();
        let mut invalid = real.clone();
        invalid["class"] = json!("Irrational");
        invalid["computable"] = computable;
        first_record_mut(&mut raw["curves"], "PlanarImage").unwrap()["origin"][0] = invalid;
        assert!(matches!(admit(&raw), Err(PersistenceError::Json(_))));
        check_selected_cap(&admit(&good).unwrap(), None);
    }
    let reflected = bodies
        .model()
        .transformed(&Matrix4::from_row_major([
            (-1).into(),
            0.into(),
            0.into(),
            7.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]))
        .unwrap();
    let mut raw: Value = serde_json::from_str(&reflected.to_json().unwrap()).unwrap();
    first_record_mut(&mut raw, "RationalExpression").expect("reflected correlated expression")["denominator"] =
        json!([Real::zero()]);
    assert!(matches!(
        admit(&raw),
        Err(PersistenceError::Build(BuildError::Geometry(
            GeometryError::PlanarCurve(hypercurve::ExactCurveError::Invalid {
                cause: hypercurve::CurveError::InvalidCurveParameter,
                ..
            })
        )))
    ));
    let limits = ArchiveLimits {
        records: 0,
        ..ArchiveLimits::default()
    };
    assert!(matches!(
        RawModel::from_json(&good.to_string(), &limits),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Records,
            ..
        }))
    ));
    first_record_mut(&mut raw, "RationalExpression").unwrap()["x_numerator"] =
        json!(vec![Real::one(); 40]);
    let limits = ArchiveLimits {
        degree: 16,
        ..ArchiveLimits::default()
    };
    assert!(matches!(
        RawModel::from_json(&raw.to_string(), &limits),
        Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource: ArchiveResource::Degree,
            ..
        }))
    ));
    check_selected_cap(&admit(&good).unwrap(), None);
}

pub fn selected_profiles_keep_holes_components_and_reversal() {
    let original = clipped_parallel_cap();
    let outer = CurvePath2::try_new(original.boundary_loops()[0].curves().to_vec()).unwrap();
    let p = [
        Uv::new(q(1, 4), q(-1, 2)),
        Uv::new(q(3, 4), q(-1, 2)),
        Uv::new(q(3, 4), q(1, 2)),
        Uv::new(q(1, 4), q(1, 2)),
    ];
    let hole = CurvePath2::try_new(
        (0..4)
            .map(|i| Curve2::from(LineSeg2::try_new(p[i].clone(), p[(i + 1) % 4].clone()).unwrap()))
            .collect(),
    )
    .unwrap();
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        3.into(),
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
    let gap: Real = "1e-12".parse().unwrap();
    for reverse in [false, true] {
        let paths = [outer.clone(), hole.clone(), distant.clone()]
            .into_iter()
            .map(|p| if reverse { p.reversed().unwrap() } else { p })
            .collect::<Vec<_>>();
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &paths,
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
        for (x, y, wanted) in [
            (q(1, 2), Real::zero(), SolidPointLocation::Outside),
            (q(1, 8), Real::zero(), SolidPointLocation::Inside),
            (q(1, 4), Real::zero(), SolidPointLocation::Boundary),
            (q(1, 4) - &gap, Real::zero(), SolidPointLocation::Inside),
            (q(1, 4) + &gap, Real::zero(), SolidPointLocation::Outside),
            (q(7, 2), Real::zero(), SolidPointLocation::Inside),
            (Real::from(2), Real::zero(), SolidPointLocation::Outside),
        ] {
            let point = Point3::new(x, y, 1.into());
            let locations = cold
                .solids()
                .map(|s| s.classify_point(&point).unwrap())
                .collect::<Vec<_>>();
            let actual = if locations.contains(&SolidPointLocation::Inside) {
                SolidPointLocation::Inside
            } else if locations.contains(&SolidPointLocation::Boundary) {
                SolidPointLocation::Boundary
            } else {
                SolidPointLocation::Outside
            };
            assert_eq!(actual, wanted);
        }
    }
}

pub fn selected_profiles_preserve_certainty_and_budget_recovery() {
    let bodies =
        builder::extrude_curve_region(&clipped_parallel_cap(), 0.into(), 3.into()).unwrap();
    let json = bodies.model().to_json().unwrap();
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
        let outcome = hypercurve::evaluate_under(policy, || {
            let cold = replay(bodies.model());
            check_selected_cap(&cold, None);
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
    check_selected_cap(&replay(bodies.model()), None);
}
