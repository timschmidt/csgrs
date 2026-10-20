use hyperbrep::*;
use hypercurve::{CurvePath2, CurveRegion2};

#[allow(dead_code)]
#[path = "selected_procedural_profiles.rs"]
pub(super) mod selected;
use selected::original;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn radial_path(region: &CurveRegion2) -> CurvePath2 {
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        3.into(),
        0.into(),
    )
    .unwrap();
    CurvePath2::try_new(
        region.boundary_loops()[0]
            .curves()
            .iter()
            .map(|curve| curve.transform_similarity(&shift).unwrap())
            .collect(),
    )
    .unwrap()
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .expect("cold replay must establish the complete solid");
    assert_eq!(restored.to_json().unwrap(), json);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(folder) = std::env::var_os("HYPERBREP_PROCEDURAL_REVOLUTION_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(
            folder.join(format!(
                "solid-{:03}.json",
                NEXT.fetch_add(1, Ordering::Relaxed)
            )),
            &json,
        )
        .unwrap();
    }
    restored
}

pub fn full_parallel_cap_revolves_and_replays() {
    let profile = radial_path(&original::original_parallel_cap());
    let bodies = builder::revolve_path(&profile)
        .expect("the full exact nonlinear parallel cap must form a solid revolution");
    assert_eq!(bodies.len(), 1);
    assert!(
        bodies
            .model()
            .curves()
            .any(|c| c.geometry().kind() == Curve3Kind::Rotation)
    );
    assert!(bodies.model().curves().any(|c| {
        c.geometry()
            .planar_geometry()
            .is_some_and(|p| p.source.analytic_parallel().is_some())
    }));
    let cold = replay(bodies.model());
    for model in [bodies.model(), &cold] {
        let solid = model.solids().next().unwrap();
        for (point, expected) in [
            (
                Point3::new(5.into(), 0.into(), 0.into()),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(0.into(), 5.into(), 2.into()),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(5.into(), 0.into(), 3.into()),
                SolidPointLocation::Outside,
            ),
            (Point3::origin(), SolidPointLocation::Outside),
        ] {
            assert_eq!(solid.classify_point(&point).unwrap(), expected);
        }
    }
}

pub fn selected_parallel_cap_revolves_and_replays() {
    let profile = radial_path(&selected::clipped_parallel_cap());
    assert!(
        profile
            .curves()
            .iter()
            .any(|curve| curve.parameter_endpoints().0.scalar().is_none())
    );
    let bodies = builder::revolve_path(&profile)
        .expect("selected endpoints must not require scalar coordinate materialization");
    assert_eq!(bodies.len(), 1);
    assert!(
        bodies
            .model()
            .curves()
            .any(|c| c.geometry().kind() == Curve3Kind::Rotation)
    );
    assert!(bodies.model().curves().any(|c| {
        c.geometry()
            .planar_geometry()
            .is_some_and(|p| p.source.analytic_parallel().is_some())
    }));
    let cold = replay(bodies.model());
    for model in [bodies.model(), &cold] {
        let solid = model.solids().next().unwrap();
        for (radius, expected) in [
            (q(7, 2), SolidPointLocation::Inside),
            (3.into(), SolidPointLocation::Boundary),
            (4.into(), SolidPointLocation::Boundary),
            (5.into(), SolidPointLocation::Outside),
        ] {
            for point in [
                Point3::new(radius.clone(), 0.into(), 0.into()),
                Point3::new(0.into(), -radius, 0.into()),
            ] {
                assert_eq!(solid.classify_point(&point).unwrap(), expected);
            }
        }
        assert!(
            model
                .vertices()
                .any(|v| v.point().try_coordinates().is_none())
        );
    }
}

fn check_selected(model: &Model, transform: Option<&Matrix4>) {
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    let solid = model.solids().next().unwrap();
    for (r, z, expected) in [
        (q(7, 2), Real::zero(), SolidPointLocation::Inside),
        (Real::from(3), Real::zero(), SolidPointLocation::Boundary),
        (
            Real::from(3) - &tiny,
            Real::zero(),
            SolidPointLocation::Outside,
        ),
        (
            Real::from(3) + &tiny,
            Real::zero(),
            SolidPointLocation::Inside,
        ),
        (
            Real::from(4) - &tiny,
            Real::zero(),
            SolidPointLocation::Inside,
        ),
        (
            Real::from(4) + &tiny,
            Real::zero(),
            SolidPointLocation::Outside,
        ),
        (q(7, 2), Real::from(-2), SolidPointLocation::Boundary),
        (q(7, 2), Real::from(-2) - &tiny, SolidPointLocation::Outside),
        (Real::zero(), Real::zero(), SolidPointLocation::Outside),
    ] {
        for point in [
            Point3::new(r.clone(), 0.into(), z.clone()),
            Point3::new(0.into(), -r, z),
        ] {
            let point =
                transform.map_or_else(|| point.clone(), |t| t.transform_point3(&point).unwrap());
            assert_eq!(solid.classify_point(&point).unwrap(), expected);
        }
    }
    assert!(matches!(
        solid.volume(),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    ));
}

pub fn procedural_revolution_reflection_reversal_and_exact_gaps() {
    let profile = radial_path(&selected::clipped_parallel_cap());
    for path in [profile.clone(), profile.reversed().unwrap()] {
        let bodies = builder::revolve_path(&path).unwrap();
        check_selected(bodies.model(), None);
        for sign in [-2, 2] {
            // z maps to -y: the replay must use the actual transformed axis.
            let transform = Matrix4::from_row_major([
                sign.into(),
                0.into(),
                0.into(),
                7.into(),
                0.into(),
                0.into(),
                (-2).into(),
                (-3).into(),
                0.into(),
                2.into(),
                0.into(),
                5.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]);
            let moved = bodies.transformed(&transform).unwrap();
            check_selected(moved.model(), Some(&transform));
            check_selected(&replay(moved.model()), Some(&transform));
        }
    }
}

pub fn procedural_revolution_subdivision_keeps_shared_incidence() {
    let bodies = builder::revolve_path(&radial_path(&selected::clipped_parallel_cap())).unwrap();
    let cold = replay(bodies.model());
    let edge = cold
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::Rotation)
        .unwrap();
    let split = edge
        .split((Real::pi() / Real::from(4)).unwrap().into())
        .unwrap();
    assert_eq!(split.details().edge_uses().len(), 2);
    check_selected(&replay(split.model()), None);
    let meridian = split
        .model()
        .edges()
        .find(|edge| {
            edge.curve()
                .geometry()
                .planar_geometry()
                .is_some_and(|p| p.source.analytic_parallel().is_some() && p.u == &Vector3::x())
        })
        .unwrap();
    let plane = Surface::plane(
        Point3::new(q(7, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let CurveSurfaceIntersection::Points(points) =
        plane.intersect_curve(meridian.curve().geometry()).unwrap()
    else {
        panic!("the meridian must cross r=7/2 once")
    };
    assert_eq!(points.len(), 1);
    assert!(points[0].parameter.try_real().is_none());
    let split = meridian.split(points[0].parameter.clone()).unwrap();
    assert_eq!(split.details().edge_uses().len(), 2);
    check_selected(&replay(split.model()), None);
}

fn rectangle(x0: Real, x1: Real, y0: Real, y1: Real) -> CurvePath2 {
    let p = [
        hypercurve::Point2::new(x0.clone(), y0.clone()),
        hypercurve::Point2::new(x1.clone(), y0),
        hypercurve::Point2::new(x1, y1.clone()),
        hypercurve::Point2::new(x0, y1),
    ];
    CurvePath2::try_new(
        (0..4)
            .map(|i| {
                hypercurve::Curve2::from(
                    hypercurve::LineSeg2::try_new(p[i].clone(), p[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

pub fn procedural_revolution_retains_native_and_selected_cavities() {
    let outer = radial_path(&selected::clipped_parallel_cap());
    let hole = rectangle(q(13, 4), q(15, 4), q(-1, 2), q(1, 2));
    let bodies = builder::revolve_path_region(&outer, &[hole]).unwrap();
    for model in [bodies.model(), &replay(bodies.model())] {
        let solid = model.solids().next().unwrap();
        assert_eq!(solid.voids().len(), 1);
        for (radius, expected) in [
            (q(7, 2), SolidPointLocation::Outside),
            (q(25, 8), SolidPointLocation::Inside),
            (q(13, 4), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                solid
                    .classify_point(&Point3::new(radius, 0.into(), 0.into()))
                    .unwrap(),
                expected
            );
        }
    }
    let outer = radial_path(&original::original_parallel_cap());
    let map = hypercurve::Similarity2::try_from_real_affine(
        q(1, 4),
        0.into(),
        0.into(),
        q(1, 4),
        3.into(),
        0.into(),
    )
    .unwrap();
    let selected_hole = radial_path(&selected::clipped_parallel_cap())
        .transform_similarity(&map)
        .unwrap();
    let bodies = builder::revolve_path_region(&outer, &[selected_hole]).unwrap();
    for model in [bodies.model(), &replay(bodies.model())] {
        let solid = model.solids().next().unwrap();
        for (radius, expected) in [
            (q(31, 8), SolidPointLocation::Outside),
            (q(7, 2), SolidPointLocation::Inside),
            (q(15, 4), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                solid
                    .classify_point(&Point3::new(radius, 0.into(), 0.into()))
                    .unwrap(),
                expected
            );
        }
    }
}

pub fn procedural_revolution_rejects_invalid_profiles_and_cavities() {
    let profile = selected::clipped_parallel_cap();
    let path = CurvePath2::try_new(profile.boundary_loops()[0].curves().to_vec()).unwrap();
    assert!(matches!(
        builder::revolve_path(&path),
        Err(ConstructionError::ProfileCrossesRevolutionAxis)
    ));
    let shifted = radial_path(&profile);
    for hole in [
        rectangle(4.into(), 5.into(), q(-1, 2), q(1, 2)),
        rectangle(q(7, 2), q(9, 2), q(-1, 2), q(1, 2)),
    ] {
        assert!(builder::revolve_path_region(&shifted, &[hole]).is_err());
    }
    let first = rectangle(q(13, 4), q(15, 4), (-1).into(), 1.into());
    let second = rectangle(q(27, 8), q(29, 8), q(-1, 2), q(1, 2));
    assert!(builder::revolve_path_region(&shifted, &[first, second]).is_err());
}

pub fn procedural_revolution_certainty_budget_and_archive_recovery() {
    let bodies = builder::revolve_path(&radial_path(&selected::clipped_parallel_cap())).unwrap();
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
        let result =
            hypercurve::evaluate_under(policy, || check_selected(&replay(bodies.model()), None));
        assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
    }
    let good: serde_json::Value = serde_json::from_str(&json).unwrap();
    for mode in 0..4 {
        let mut bad = good.clone();
        match mode {
            0 => {
                bad["shells"][0].as_array_mut().unwrap().pop();
            }
            1 => bad["faces"][0]["orientation"] = serde_json::json!("Reversed"),
            2 => bad["curves"][0]["Rotation"]["axis"][0] = serde_json::json!(Real::one()),
            _ => {
                bad["edge_uses"][0]["parameter_correspondence"]["Coordinate"]["scale"] =
                    serde_json::json!(Real::from(2))
            }
        }
        assert!(
            RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
                .and_then(RawModel::validate)
                .is_err(),
            "forgery {mode}"
        );
        check_selected(&replay(bodies.model()), None);
    }
}

pub fn procedural_revolution_consumes_regions_holes_components_and_empty() {
    use hypercurve::{BooleanOp, CurveRegionLoopRole, FillRule};
    let outer = radial_path(&selected::clipped_parallel_cap());
    let hole = rectangle(q(13, 4), q(15, 4), q(-1, 2), q(1, 2));
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[outer, hole],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
        &[FillRule::NonZero; 2],
    )
    .unwrap();
    let shift = hypercurve::Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        5.into(),
    )
    .unwrap();
    let upper = region.transform_similarity(&shift).unwrap();
    let both = region.boolean_region(&upper, BooleanOp::Union).unwrap();
    let bodies = builder::revolve_curve_region(&both).unwrap();
    assert_eq!(bodies.len(), 2);
    for model in [bodies.model(), &replay(bodies.model())] {
        for z in [Real::zero(), Real::from(5)] {
            let material = Point3::new(q(25, 8), 0.into(), z.clone());
            let cavity = Point3::new(q(7, 2), 0.into(), z);
            assert_eq!(
                model
                    .solids()
                    .filter(|s| s.classify_point(&material).unwrap() == SolidPointLocation::Inside)
                    .count(),
                1
            );
            assert!(
                model
                    .solids()
                    .all(|s| s.classify_point(&cavity).unwrap() == SolidPointLocation::Outside)
            );
        }
    }
    let empty = region
        .boolean_region(&region, BooleanOp::Difference)
        .unwrap();
    for input in [empty, CurveRegion2::empty()] {
        let empty = builder::revolve_curve_region(&input).unwrap();
        assert!(empty.is_empty());
        assert_eq!(replay(empty.model()).solids().len(), 0);
        let recovered = boolean::union(&empty, &bodies).unwrap();
        assert_eq!(recovered.len(), 2);
        assert!(boolean::intersection(&empty, &bodies).unwrap().is_empty());
        assert!(boolean::difference(&empty, &bodies).unwrap().is_empty());
    }
}

pub fn unified_revolution_preserves_native_images_and_exact_volume() {
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[rectangle(2.into(), 4.into(), 0.into(), 2.into())],
        &[hypercurve::CurveRegionLoopRole::Material],
        &[hypercurve::FillRule::NonZero],
    )
    .unwrap();
    let bodies = builder::revolve_curve_region(&region).unwrap();
    for model in [bodies.model(), &replay(bodies.model())] {
        assert!(model.curves().all(|c| matches!(
            c.geometry().kind(),
            Curve3Kind::Line | Curve3Kind::CircleArc
        )));
        let solid = model.solids().next().unwrap();
        assert_eq!(
            hyperlimit::compare_reals(
                &solid.volume().unwrap(),
                &(Real::from(24) * Real::pi()),
                STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            solid
                .classify_point(&Point3::new(3.into(), 0.into(), 1.into()))
                .unwrap(),
            SolidPointLocation::Inside
        );
    }
}
