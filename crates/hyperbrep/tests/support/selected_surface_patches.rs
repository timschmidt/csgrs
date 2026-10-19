use hyperbrep::*;
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierParameterRange2, Curve2, CurvePath2, QuadraticBezier2,
};

#[allow(dead_code)]
#[path = "selected_procedural_profiles.rs"]
mod procedural;

#[allow(dead_code)]
#[path = "selected_profile_source.rs"]
mod parabola;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn selected_profile() -> Curve3 {
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
    profile_ending_at(BezierParameter2::Algebraic(root))
}

fn profile_ending_at(end: BezierParameter2) -> Curve3 {
    let curve = Curve2::try_analytic_parallel(
        QuadraticBezier2::new(
            hypercurve::Point2::from_values(0, 0),
            hypercurve::Point2::from_values(2, 2),
            hypercurve::Point2::from_values(4, 0),
        )
        .parallel_left(Real::one())
        .unwrap(),
        BezierParameterRange2::try_new(BezierParameter2::Exact(q(1, 4)), end).unwrap(),
    )
    .unwrap();
    Curve3::planar_image(curve, Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}

pub fn selected_extrusion_patch_builds_and_replays() {
    let profile = selected_profile();
    assert!(profile.domain().end().try_real().is_none());
    let sheet = builder::extrusion_patch(profile, Vector3::z(), (-1).into(), 2.into())
        .expect("retained profile endpoints must construct a complete exact extrusion patch");
    let json = sheet.model().to_json().unwrap();
    record_archive(&json);
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(cold.faces().len(), 1);
    assert_eq!(cold.edges().len(), 4);
}

pub fn selected_planar_face_builds_and_replays() {
    let region = procedural::clipped_parallel_cap();
    assert_eq!(region.boundary_loops().len(), 1);
    let path = CurvePath2::try_new(region.boundary_loops()[0].curves().to_vec()).unwrap();
    let sheet = builder::planar_face(&path, &[], Point3::origin(), Vector3::x(), Vector3::y())
        .expect("the complete selected profile must also construct a standalone planar face");
    let json = sheet.model().to_json().unwrap();
    record_archive(&json);
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(cold.faces().len(), 1);
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
        assert_eq!(cold.solids().len(), 0);
    }
    cold
}

fn record_archive(text: &str) {
    let _ = text;
    #[cfg(not(target_arch = "wasm32"))]
    if let Ok(folder) = std::env::var("HYPERBREP_SELECTED_PATCH_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let index = NEXT.fetch_add(1, Ordering::Relaxed);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(folder.join(format!("patch-{index:03}.json")), text).unwrap();
    }
}

fn affine() -> Matrix4 {
    Matrix4::from_row_major([
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
    ])
}

fn check_patch_samples(model: &Model, reversed: bool, transformed: bool) {
    let surface = model.faces().next().unwrap().surface();
    assert_eq!(surface.kind(), SurfaceKind::Extrusion);
    // Reflection reverses the surface's u chart as well as its face wires;
    // the spatial edge parameter chart itself remains unchanged.
    let u = if reversed != transformed {
        -q(1, 2)
    } else {
        q(1, 2)
    };
    // At t=1/2 the parabola is (2,1) and its unit left parallel is
    // (2,2). The direction is (1,2,3); these are independent exact values.
    let samples = if transformed {
        [
            ((-1).into(), Point3::new(3.into(), (-5).into(), 1.into())),
            (2.into(), Point3::new(3.into(), 22.into(), 19.into())),
        ]
    } else {
        [
            ((-1).into(), Point3::new(1.into(), 0.into(), (-3).into())),
            (2.into(), Point3::new(4.into(), 6.into(), 6.into())),
        ]
    };
    for (v, expected) in samples {
        let actual = surface.point_at(&Point2::new(u.clone(), v)).unwrap();
        assert!(
            hyperlimit::point3_equal(&actual, &expected, hyperlimit::PredicatePolicy::STRICT)
                .value()
                .unwrap()
        );
    }
    assert!(
        model
            .vertices()
            .any(|vertex| vertex.point().try_coordinates().is_none())
    );
    for edge in model.edges() {
        assert_eq!(edge.uses().len(), 1);
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

pub fn selected_patches_reverse_transform_and_split() {
    for reversed in [false, true] {
        let profile = selected_profile();
        let profile = if reversed {
            profile.reversed().unwrap()
        } else {
            profile
        };
        let sheet = builder::extrusion_patch(
            profile,
            Vector3::new([1.into(), 2.into(), 3.into()]),
            (-1).into(),
            2.into(),
        )
        .unwrap();
        let cold = replay(sheet.model());
        check_patch_samples(&cold, reversed, false);
        let transformed = replay(&cold.transformed(&affine()).unwrap());
        check_patch_samples(&transformed, reversed, true);
        let edge = transformed
            .edges()
            .find(|edge| edge.curve().kind() == Curve3Kind::PlanarImage)
            .unwrap();
        let root = BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                3.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(2, 3), q(3, 4)).unwrap(),
        )
        .unwrap();
        // Obtain the selected parameter through the public spatial chart,
        // including the same chart reversal used by the target edge.
        let selection = profile_ending_at(BezierParameter2::Algebraic(root));
        let parameter = if reversed {
            selection.reversed().unwrap().domain().start().clone()
        } else {
            selection.domain().end().clone()
        };
        assert!(parameter.try_real().is_none());
        let split = edge.split(parameter).unwrap();
        let split = replay(split.model());
        assert_eq!(split.edges().len(), 5);
        check_patch_samples(&split, reversed, true);
        let axial = split
            .edges()
            .find(|edge| edge.curve().kind() == Curve3Kind::Translation)
            .unwrap();
        let split = axial.split(q(1, 2).into()).unwrap();
        let split = replay(split.model());
        assert_eq!(split.edges().len(), 6);
        check_patch_samples(&split, reversed, true);
    }
}

fn rectangle(x0: Real, y0: Real, x1: Real, y1: Real) -> CurvePath2 {
    let points = [
        hypercurve::Point2::new(x0.clone(), y0.clone()),
        hypercurve::Point2::new(x1.clone(), y0),
        hypercurve::Point2::new(x1, y1.clone()),
        hypercurve::Point2::new(x0, y1),
    ];
    CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    hypercurve::LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn face_region(model: &Model) -> hypercurve::CurveRegion2 {
    let face = model.faces().next().unwrap();
    let paths = std::iter::once(face.outer().unwrap())
        .chain(face.inner())
        .map(|wire| {
            CurvePath2::try_new(
                wire.edge_uses()
                    .map(|edge_use| edge_use.pcurve().curve().clone())
                    .collect(),
            )
            .unwrap()
        })
        .collect::<Vec<_>>();
    hypercurve::CurveRegion2::try_from_boundary_paths(&paths, hypercurve::FillRule::NonZero)
        .unwrap()
}

pub fn selected_planar_faces_keep_holes_frames_and_orientation() {
    let (region, _) = parabola::profile(false, false, false);
    let path = CurvePath2::try_new(region.boundary_loops()[0].curves().to_vec()).unwrap();
    let hole = rectangle(-q(1, 4), q(1, 4), q(1, 4), q(1, 2));
    for reversed in [false, true] {
        let path = if reversed {
            path.reversed().unwrap()
        } else {
            path.clone()
        };
        let hole = if reversed {
            hole.reversed().unwrap()
        } else {
            hole.clone()
        };
        let sheet = builder::planar_face(
            &path,
            &[hole],
            Point3::new(5.into(), (-2).into(), 7.into()),
            Vector3::new([2.into(), 0.into(), 1.into()]),
            Vector3::new([1.into(), 3.into(), 0.into()]),
        )
        .unwrap();
        for (transformed, model) in [
            (false, replay(sheet.model())),
            (
                true,
                replay(&sheet.transformed(&affine()).unwrap().into_model()),
            ),
        ] {
            assert_eq!(model.faces().next().unwrap().inner().len(), 1);
            let area_scale = Real::from(if transformed { 1697 } else { 46 })
                .sqrt()
                .unwrap();
            let expected_area = (q(8, 3) * Real::from(2).sqrt().unwrap() - q(1, 8)) * area_scale;
            assert_eq!(
                hyperlimit::compare_reals(
                    &model.faces().next().unwrap().area().unwrap(),
                    &expected_area,
                    hyperlimit::PredicatePolicy::STRICT,
                )
                .value(),
                Some(std::cmp::Ordering::Equal)
            );
            let represented = face_region(&model);
            // Material is x^2 <= y <= 2, less the authored rectangular hole.
            for (point, expected) in [
                (
                    hypercurve::Point2::new(0.into(), 1.into()),
                    hypercurve::RegionPointLocation::Inside,
                ),
                (
                    hypercurve::Point2::new(0.into(), q(3, 8)),
                    hypercurve::RegionPointLocation::Outside,
                ),
                (
                    hypercurve::Point2::new(0.into(), (-1).into()),
                    hypercurve::RegionPointLocation::Outside,
                ),
                (
                    hypercurve::Point2::new(2.into(), 1.into()),
                    hypercurve::RegionPointLocation::Outside,
                ),
                (
                    hypercurve::Point2::new(0.into(), 2.into()),
                    hypercurve::RegionPointLocation::Boundary,
                ),
            ] {
                assert_eq!(represented.classify_point(&point.into()).unwrap(), expected);
            }
            // The original plane is 3x-y-6z=-25. Substituting the affine
            // inverse gives -18x+2y-37z=-653 in the transformed frame.
            let (normal, offset) = if transformed {
                (
                    Vector3::new([(-18).into(), 2.into(), (-37).into()]),
                    Real::from(-653),
                )
            } else {
                (
                    Vector3::new([3.into(), (-1).into(), (-6).into()]),
                    Real::from(-25),
                )
            };
            for vertex in model.vertices() {
                assert_eq!(
                    vertex
                        .point()
                        .compare_linear_form(&normal, &offset)
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
            }
        }
    }
    let outside = rectangle(4.into(), 4.into(), 5.into(), 5.into());
    assert!(matches!(
        builder::planar_face(
            &path,
            &[outside],
            Point3::origin(),
            Vector3::x(),
            Vector3::y()
        ),
        Err(ConstructionError::HoleOutside)
    ));
    let nested = rectangle(-q(1, 8), q(5, 16), q(1, 8), q(7, 16));
    assert!(matches!(
        builder::planar_face(
            &path,
            &[hole, nested],
            Point3::origin(),
            Vector3::x(),
            Vector3::y()
        ),
        Err(ConstructionError::NestedHoles)
    ));
}

pub fn selected_planar_patches_keep_holes_and_contact_edits() {
    let region = procedural::clipped_parallel_cap();
    let path = CurvePath2::try_new(region.boundary_loops()[0].curves().to_vec()).unwrap();
    let hole = rectangle(q(1, 4), -q(1, 2), q(3, 4), -q(1, 4));
    let gap: Real = "1e-12".parse().unwrap();
    for reversed in [false, true] {
        let outer = if reversed {
            path.reversed().unwrap()
        } else {
            path.clone()
        };
        let inner = if reversed {
            hole.reversed().unwrap()
        } else {
            hole.clone()
        };
        let sheet = builder::planar_face(
            &outer,
            &[inner],
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        let cold = replay(sheet.model());
        assert!(
            cold.vertices()
                .any(|v| v.point().try_coordinates().is_none())
        );
        let profile = cold
            .edges()
            .find(|edge| {
                edge.curve()
                    .geometry()
                    .planar_geometry()
                    .is_some_and(|image| image.source.analytic_parallel().is_some())
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
            panic!("the retained parallel has one transverse contact");
        };
        assert_eq!(points.len(), 1);
        assert!(points[0].parameter.try_real().is_none());
        let split = profile.split(points[0].parameter.clone()).unwrap();
        assert_eq!(split.model().edges().len(), cold.edges().len() + 1);
        let split = replay(split.model());
        for (reflected, model) in [
            (false, cold),
            (false, split.clone()),
            (true, replay(&split.transformed(&affine()).unwrap())),
        ] {
            assert_eq!(model.faces().next().unwrap().inner().len(), 1);
            let represented = face_region(&model);
            for (x, y, expected) in [
                (
                    q(1, 2),
                    Real::zero(),
                    hypercurve::RegionPointLocation::Inside,
                ),
                (q(1, 2), -q(3, 8), hypercurve::RegionPointLocation::Outside),
                (
                    Real::zero(),
                    Real::zero(),
                    hypercurve::RegionPointLocation::Boundary,
                ),
                (
                    gap.clone(),
                    Real::zero(),
                    hypercurve::RegionPointLocation::Inside,
                ),
                (
                    -&gap,
                    Real::zero(),
                    hypercurve::RegionPointLocation::Outside,
                ),
                (
                    Real::from(2),
                    Real::zero(),
                    hypercurve::RegionPointLocation::Outside,
                ),
            ] {
                let x = if reflected { -x } else { x };
                assert_eq!(
                    represented
                        .classify_point(&hypercurve::Point2::new(x, y).into())
                        .unwrap(),
                    expected,
                );
            }
        }
    }
}

pub fn selected_patch_archives_reject_wrong_images_and_recover() {
    let sheet =
        builder::extrusion_patch(selected_profile(), Vector3::z(), (-1).into(), 2.into()).unwrap();
    let good: serde_json::Value = serde_json::from_str(&sheet.model().to_json().unwrap()).unwrap();
    for mode in 0..3 {
        let mut bad = good.clone();
        if mode < 2 {
            let map = bad["edge_uses"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|v| v["parameter_correspondence"].get_mut("Coordinate"))
                .unwrap();
            map["scale"] = serde_json::json!(Real::from(if mode == 0 { 0 } else { 2 }));
        } else {
            let line = bad["curves"]
                .as_array_mut()
                .unwrap()
                .iter_mut()
                .find_map(|v| v.get_mut("Translation"))
                .unwrap();
            line["displacement"][0] = serde_json::json!(Real::one());
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

pub fn selected_patches_keep_failure_certainty_and_budget_contracts() {
    assert!(matches!(
        builder::extrusion_patch(selected_profile(), Vector3::z(), 2.into(), 2.into()),
        Err(ConstructionError::Build(BuildError::Geometry(
            GeometryError::InvalidParameterDomain
        )))
    ));
    assert!(matches!(
        builder::extrusion_patch(selected_profile(), Vector3::zeros(), 0.into(), 1.into()),
        Err(ConstructionError::Build(BuildError::Geometry(
            GeometryError::DegenerateExtrusionDirection
        )))
    ));
    let sheet =
        builder::extrusion_patch(selected_profile(), Vector3::z(), (-1).into(), 2.into()).unwrap();
    let json = sheet.model().to_json().unwrap();
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
        let outcome = hypercurve::evaluate_under(policy, || replay(sheet.model()));
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
}
