use hyperbrep::*;
use hypercurve::{
    BezierParameter2, BezierParameterRange2, CubicBezier2, Curve2, Point2 as Uv, QuadraticBezier2,
};
use std::cmp::Ordering;

// Reuse the unchanged original full cap, including quadratic closing segments.
#[allow(dead_code)]
#[path = "parallel_profile_extrusion.rs"]
mod prior;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root(scale: i32) -> Parameter {
    let curve = root_source(scale);
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("selected root")
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn root_source(scale: i32) -> Curve3 {
    Curve3::rational_bezier(
        [-1, -1, -1, 2 * scale * scale * scale - 1]
            .map(|x| Point3::new(x.into(), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 4],
    )
    .unwrap()
}

fn frame(source: Curve2) -> Curve3 {
    Curve3::planar_image(
        source,
        Point3::new(5.into(), (-2).into(), 7.into()),
        Vector3::new([2.into(), 1.into(), (-1).into()]),
        Vector3::new([1.into(), 3.into(), 2.into()]),
    )
    .unwrap()
}

fn polynomial() -> Curve2 {
    CubicBezier2::new(
        Uv::from_values(0, 0),
        Uv::new(q(1, 3), 0.into()),
        Uv::new(q(2, 3), 0.into()),
        Uv::from_values(1, 1),
    )
    .into()
}

fn parallel(a: i32, b: i32) -> Curve2 {
    let source = QuadraticBezier2::new(
        Uv::from_values(0, 0),
        Uv::from_values(2, 2),
        Uv::from_values(4, 0),
    )
    .parallel_left(Real::one())
    .unwrap();
    Curve2::try_analytic_parallel(
        source,
        BezierParameterRange2::try_new(
            BezierParameter2::Exact(a.into()),
            BezierParameter2::Exact(b.into()),
        )
        .unwrap(),
    )
    .unwrap()
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let model = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), json);
    model
}

fn archive(model: &Model, name: &str) {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("HYPERBREP_SELECTED_PLANAR_ARTIFACT_DIR") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            directory.join(format!("{name}.json")),
            model.to_json().unwrap(),
        )
        .unwrap();
    }
    #[cfg(target_arch = "wasm32")]
    let _ = (model, name);
}

pub fn selected_planar_points_keep_frame_and_source_correlation() {
    let curve = frame(polynomial());
    let parameter = root(1);
    let point = curve.point_at(&parameter).unwrap();
    assert!(point.try_coordinates().is_none());
    let (source, retained) = point.curve_source().unwrap();
    assert_eq!(source.kind(), Curve3Kind::PlanarImage);
    assert_eq!(retained.compare(&parameter).unwrap(), Ordering::Equal);
    // For P=(t,t^3), 2t^3=1: X=11/2+2t, Z=8-t, so X+2Z=43/2.
    assert_eq!(
        point
            .compare_linear_form(&Vector3::new([1.into(), 0.into(), 2.into()]), &q(43, 2))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        point
            .compare_linear_form(&Vector3::new([1.into(), (-1).into(), 1.into()]), &14.into())
            .unwrap(),
        Ordering::Equal
    );
    let native = Curve3::rational_bezier(
        [
            Point3::new(5.into(), (-2).into(), 7.into()),
            Point3::new(q(17, 3), q(-5, 3), q(20, 3)),
            Point3::new(q(19, 3), q(-4, 3), q(19, 3)),
            Point3::new(8.into(), 2.into(), 8.into()),
        ]
        .to_vec(),
        vec![Real::one(); 4],
    )
    .unwrap();
    assert!(
        point
            .coincides_with(&native.point_at(&root(1)).unwrap())
            .unwrap()
    );
    let lower = GeometricPoint3::from_coordinates(Point3::new(7.into(), 0.into(), 7.into()));
    let upper = GeometricPoint3::from_coordinates(Point3::new(q(29, 4), 1.into(), q(29, 4)));
    assert_eq!(
        point.compare_coordinate(&lower, Axis::X).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        point.compare_coordinate(&upper, Axis::X).unwrap(),
        Ordering::Less
    );
    point.bounds().unwrap();
    for (a, b) in [(0, 1), (1, 0), (-1, 2), (2, -1)] {
        let image = frame(parallel(a, b));
        let selected = image.point_at(&parameter).unwrap();
        assert!(selected.try_coordinates().is_none());
        assert_eq!(
            selected
                .compare_linear_form(&Vector3::new([1.into(), (-1).into(), 1.into()]), &14.into())
                .unwrap(),
            Ordering::Equal
        );
        selected.bounds().unwrap();
    }
}

pub fn selected_planar_points_replay_sources_and_affine_transforms() {
    let transform = transform();
    for (i, source) in [polynomial(), parallel(1, 0)].into_iter().enumerate() {
        let curve = frame(source);
        let point = curve.point_at(&root(1)).unwrap();
        for attached in [false, true] {
            let mut builder = ModelBuilder::new();
            builder.vertex(point.clone()).unwrap();
            if attached {
                builder.curve(curve.clone()).unwrap();
            }
            let model = builder.finish().unwrap();
            let cold = reload(&model);
            archive(&cold, &format!("point-{i}-{attached}"));
            let restored = cold.vertices().next().unwrap().point();
            assert!(point.coincides_with(restored).unwrap());
            assert!(restored.try_coordinates().is_none());
            assert!(restored.curve_source().is_some());
            let mapped = point.transformed(&transform).unwrap();
            let transformed = reload(&cold.transformed(&transform).unwrap());
            archive(&transformed, &format!("point-transformed-{i}-{attached}"));
            assert!(
                mapped
                    .coincides_with(transformed.vertices().next().unwrap().point())
                    .unwrap()
            );
            mapped.bounds().unwrap();
        }
    }
}

fn transform() -> Matrix4 {
    Matrix4::from_row_major([
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
    ])
}

fn check_cap(model: &Model) {
    let solid = model.solids().next().unwrap();
    for (point, expected) in [
        (
            Point3::new(2.into(), 0.into(), q(3, 2)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(2.into(), 2.into(), q(3, 2)),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(2.into(), 3.into(), q(3, 2)),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected);
    }
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

pub fn selected_parallel_edge_splits_replay_the_whole_cap() {
    let bodies =
        builder::extrude_curve_region(&prior::original_parallel_cap(), 0.into(), 3.into()).unwrap();
    let edge = bodies
        .model()
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::PlanarImage)
        .unwrap();
    let split = edge.split(root(1)).unwrap();
    assert!(split.details().vertex().point().try_coordinates().is_none());
    check_cap(split.model());
    let cold = reload(split.model());
    check_cap(&cold);
    let first = cold
        .edges()
        .find(|e| {
            e.curve().kind() == Curve3Kind::PlanarImage
                && e.domain().end().compare(&root(1)).unwrap() == Ordering::Equal
        })
        .unwrap();
    let second = first.split(root(2)).unwrap();
    check_cap(&reload(second.model()));
    archive(second.model(), "split-twice");
}

fn split_cap() -> Model {
    let bodies =
        builder::extrude_curve_region(&prior::original_parallel_cap(), 0.into(), 3.into()).unwrap();
    let edge = bodies
        .model()
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::PlanarImage)
        .unwrap();
    reload(edge.split(root(1)).unwrap().model())
}

fn location(model: &Model, point: &Point3) -> SolidPointLocation {
    let mut result = SolidPointLocation::Outside;
    for solid in model.solids() {
        match solid.classify_point(point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => result = SolidPointLocation::Boundary,
            SolidPointLocation::Outside => (),
        }
    }
    result
}

pub fn selected_parallel_splits_continue_through_boolean_sets() {
    let original = split_cap();
    let original = BodySet::from_solid(original.solids().next().unwrap());
    let cutter =
        builder::extrude_curve_region(&prior::original_parallel_cap(), 1.into(), 2.into()).unwrap();
    let mut current = boolean::union(&BodySet::empty(), &original).unwrap();
    for i in 0..2 {
        let middle = boolean::intersection(&current, &cutter).unwrap();
        let ends = boolean::difference(&current, &cutter).unwrap();
        assert_eq!(ends.len(), 2);
        for (z, in_middle) in [(q(1, 2), false), (q(3, 2), true), (q(5, 2), false)] {
            let point = Point3::new(2.into(), 0.into(), z);
            assert_eq!(
                location(&reload(middle.model()), &point),
                if in_middle {
                    SolidPointLocation::Inside
                } else {
                    SolidPointLocation::Outside
                }
            );
            assert_eq!(
                location(&reload(ends.model()), &point),
                if in_middle {
                    SolidPointLocation::Outside
                } else {
                    SolidPointLocation::Inside
                }
            );
        }
        current = boolean::union(&ends, &middle).unwrap();
        check_cap(&reload(current.model()));
        assert!(boolean::difference(&current, &original).unwrap().is_empty());
        assert!(boolean::difference(&original, &current).unwrap().is_empty());
        let empty = boolean::difference(&current, &current).unwrap();
        assert!(empty.is_empty());
        assert_eq!(reload(empty.model()).counts().solids, 0);
        archive(middle.model(), &format!("middle-{i}"));
        archive(ends.model(), &format!("ends-{i}"));
        archive(current.model(), &format!("reunited-{i}"));
        archive(empty.model(), &format!("empty-{i}"));
        current = boolean::union(&empty, &current).unwrap();
    }
    check_cap(
        boolean::difference(&current, &BodySet::empty())
            .unwrap()
            .model(),
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

pub fn selected_parallel_splits_transform_and_preserve_near_separation() {
    let original = split_cap();
    let transform = transform();
    let mapped = original.transformed(&transform).unwrap();
    for (i, model) in [
        original.clone(),
        reload(&original),
        mapped.clone(),
        reload(&mapped),
    ]
    .into_iter()
    .enumerate()
    {
        let gap: Real = "1e-12".parse().unwrap();
        for (y, expected) in [
            (Real::from(2) - &gap, SolidPointLocation::Inside),
            (2.into(), SolidPointLocation::Boundary),
            (Real::from(2) + &gap, SolidPointLocation::Outside),
        ] {
            let point = Point3::new(2.into(), y, q(3, 2));
            let point = if i < 2 {
                point
            } else {
                transform.transform_point3(&point).unwrap()
            };
            assert_eq!(location(&model, &point), expected);
        }
        assert!(
            model
                .vertices()
                .any(|v| v.point().try_coordinates().is_none())
        );
        for edge in model.edges() {
            assert_eq!(edge.uses().count(), 2);
        }
        archive(&model, &format!("split-transform-{i}"));
    }
}

pub fn selected_planar_archives_reject_forged_sources_and_recover() {
    use serde_json::{Value, json};
    let model = split_cap();
    let good: Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    for mode in 0..6 {
        let mut raw = good.clone();
        if mode == 5 {
            let parallel = raw["pcurves"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .filter_map(|p| p.get_mut("AnalyticParallel"))
                .find(|p| {
                    p["range"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .any(|p| p.get("Selected").is_some())
                })
                .unwrap();
            // The exact trim correspondence is unchanged. Complete source
            // agreement must reject this even without scalar endpoint samples.
            parallel["distance"] = json!(Real::from(2));
        } else if mode < 3 {
            let image = raw["curves"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|c| c.get_mut("PlanarImage"))
                .unwrap();
            match mode {
                0 => image["origin"][2] = json!(Real::one()),
                1 => image["source_chart"][0] = json!(q(-1, 2)),
                _ => image["v"] = image["u"].clone(),
            }
        } else {
            let point = raw["vertices"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|v| v.get_mut("CurveImage"))
                .unwrap();
            point["parameter"] = if mode == 3 {
                json!({"Selected":1000000})
            } else {
                json!({"Scalar":Real::zero()})
            };
        }
        let result = RawModel::from_json(&raw.to_string(), &ArchiveLimits::default())
            .and_then(RawModel::validate);
        assert!(result.is_err(), "forgery mode {mode}");
        if mode == 5 {
            assert!(
                matches!(
                    result,
                    Err(PersistenceError::Build(BuildError::EdgeUseSupportMismatch))
                ),
                "{result:?}"
            );
        }
        check_cap(&reload(&model));
    }
}

pub fn selected_planar_policies_and_work_exhaustion_recover() {
    let source = root_source(1);
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| source.parameters_of(&Point3::origin()));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    source.parameters_of(&Point3::origin()).unwrap();
    let curve = frame(parallel(0, 1));
    let parameter = root(1);
    let point = curve.point_at(&parameter).unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || {
            point.compare_linear_form(&Vector3::new([1.into(), (-1).into(), 1.into()]), &14.into())
        });
        assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
        assert_eq!(result.into_unverified().unwrap(), Ordering::Equal);
    }
    let zero = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        let point = frame(parallel(0, 1)).point_at(&parameter)?;
        point.compare_coordinate(
            &GeometricPoint3::from_coordinates(Point3::origin()),
            Axis::X,
        )
    });
    match zero.result {
        Ok(result) => assert_eq!(result.unwrap(), Ordering::Greater),
        Err(error) => assert_eq!(error.limit, 0),
    }
    assert_eq!(
        point
            .compare_coordinate(
                &GeometricPoint3::from_coordinates(Point3::origin()),
                Axis::X
            )
            .unwrap(),
        Ordering::Greater
    );
    check_cap(&split_cap());
}

pub fn selected_planar_poles_remain_typed_failures() {
    let source: Curve2 = hypercurve::RationalBezier2::try_new(
        vec![
            Uv::from_values(0, 0),
            Uv::from_values(1, 1),
            Uv::from_values(2, 1),
            Uv::from_values(3, 0),
        ],
        vec![(-1).into(), (-1).into(), (-1).into(), 1.into()],
    )
    .unwrap()
    .into();
    // W(t)=2t^3-1 is zero at the selected root, while both ends are finite.
    let curve = frame(source);
    let mut builder = ModelBuilder::new();
    builder.curve(curve.clone()).unwrap();
    let model = builder.finish().unwrap();
    let cold = reload(&model);
    for curve in [&curve, cold.curves().next().unwrap().geometry()] {
        assert!(matches!(
            curve.point_at(&root(1)),
            Err(GeometryError::PlanarCurve(_))
        ));
        assert!(curve.bounds().is_err());
        curve.point_at(&Real::zero().into()).unwrap();
        curve.point_at(&Real::one().into()).unwrap();
    }
    frame(polynomial())
        .point_at(&root(1))
        .unwrap()
        .bounds()
        .unwrap();
    archive(&cold, "pole-source");
}
