//! Exact expectations shared by native and WASM similarity consumers.
use hyperbrep::{
    ArchiveLimits, BodySet, Matrix4, Model, Point2, Point3, RawModel, Real, SolidPointLocation,
    builder,
};
use hyperlimit::{compare_reals, point3_equal};

#[allow(dead_code)]
#[path = "periodic_cylinder.rs"]
mod periodic;

pub fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

pub fn equal(actual: &Real, expected: &Real) {
    equal_named(actual, expected, "exact equality");
}

fn equal_named(actual: &Real, expected: &Real, label: &str) {
    assert_eq!(
        compare_reals(actual, expected, hyperbrep::STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal),
        "{label}: actual={actual:?}, expected={expected:?}"
    );
}

pub fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

pub fn similarity(scale: &Real, reflected: bool) -> Matrix4 {
    let sign = if reflected { -Real::one() } else { Real::one() };
    Matrix4::from_row_major([
        scale * ratio(3, 5) * &sign,
        -scale * ratio(4, 5),
        0.into(),
        3.into(),
        scale * ratio(4, 5) * sign,
        scale * ratio(3, 5),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        scale.clone(),
        7.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}

pub fn reload(model: &Model) -> Model {
    reload_named(model, "archive replay")
}

fn reload_named(model: &Model, label: &str) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json, "{label}");
    loaded
}

pub fn check_images(model: &Model) {
    for edge_use in model.edge_uses() {
        let pcurve = edge_use.pcurve();
        for parameter in [pcurve.domain_start().unwrap(), pcurve.domain_end().unwrap()] {
            let uv = pcurve.point_at(&parameter).unwrap();
            let Some(uv) = uv.coordinates() else { continue };
            let on_surface = edge_use
                .wire()
                .face()
                .surface()
                .point_at(&{
                    let uv = &Point2::new(uv.x().clone(), uv.y().clone());
                    hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                })
                .map(|point| {
                    point
                        .try_coordinates()
                        .expect("represented surface fixture")
                        .clone()
                })
                .unwrap();
            let edge_parameter = edge_use.edge_parameter_at(&parameter).unwrap();
            let on_edge = edge_use.edge().curve().point_at(&edge_parameter).unwrap();
            assert!(on_edge.coincides_with(&on_surface.into()).unwrap());
        }
    }
}

pub fn primitives() {
    let cases: Vec<(&str, BodySet, Real, Real, Point3, Point3)> = vec![
        (
            "cylinder",
            builder::cylinder(5.into(), 7.into()).unwrap(),
            Real::from(175) * Real::pi(),
            Real::from(120) * Real::pi(),
            point(0, 0, 3),
            point(5, 0, 3),
        ),
        (
            "sphere",
            builder::sphere(5.into()).unwrap(),
            ratio(500, 3) * Real::pi(),
            Real::from(100) * Real::pi(),
            point(0, 0, 0),
            point(5, 0, 0),
        ),
        (
            "cone",
            builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap(),
            Real::from(78) * Real::pi(),
            (Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap()) * Real::pi(),
            point(0, 0, 3),
            point(5, 0, 0),
        ),
        (
            "torus",
            builder::torus(5.into(), 2.into()).unwrap(),
            Real::from(40) * Real::pi() * Real::pi(),
            Real::from(40) * Real::pi() * Real::pi(),
            point(5, 0, 0),
            point(7, 0, 0),
        ),
        (
            "revolution",
            builder::revolve(&[
                Point2::new(2.into(), 0.into()),
                Point2::new(4.into(), 0.into()),
                Point2::new(4.into(), 3.into()),
                Point2::new(2.into(), 3.into()),
            ])
            .unwrap(),
            Real::from(36) * Real::pi(),
            Real::from(60) * Real::pi(),
            point(3, 0, 1),
            point(4, 0, 1),
        ),
    ];
    for (name, source, volume, area, inside, boundary) in cases {
        for scale in [
            ratio(1, 1000),
            ratio(127, 5000),
            Real::from(2),
            Real::from(2).sqrt().unwrap(),
        ] {
            for reflected in [false, true] {
                let transform = similarity(&scale, reflected);
                let moved = source
                    .transformed(&transform)
                    .unwrap_or_else(|e| panic!("{name}, {scale:?}, reflected={reflected}: {e:?}"));
                let loaded = reload(moved.model());
                for model in [moved.model(), &loaded] {
                    let body = model.solids().next().unwrap();
                    equal(
                        &body.volume().unwrap(),
                        &(&volume * &scale * &scale * &scale),
                    );
                    equal(
                        &model
                            .faces()
                            .map(|f| f.area().unwrap())
                            .fold(Real::zero(), |sum, a| sum + a),
                        &(&area * &scale * &scale),
                    );
                    for (p, expected) in [
                        (&inside, SolidPointLocation::Inside),
                        (&boundary, SolidPointLocation::Boundary),
                        (&point(20, 20, 20), SolidPointLocation::Outside),
                    ] {
                        assert_eq!(
                            body.classify_point(&transform.transform_point3(p).unwrap())
                                .unwrap(),
                            expected,
                            "{name}"
                        );
                    }
                    check_images(model);
                }
                let inverse = loaded.transformed(&transform.inverse().unwrap()).unwrap();
                let restored = reload(&inverse);
                equal(
                    &restored.solids().next().unwrap().volume().unwrap(),
                    &volume,
                );
                for (before, after) in source.model().vertices().zip(restored.vertices()) {
                    assert_eq!(
                        point3_equal(
                            before.point().try_coordinates().unwrap(),
                            after.point().try_coordinates().unwrap(),
                            hyperbrep::STRICT_PREDICATES
                        )
                        .value(),
                        Some(true)
                    );
                }
            }
        }
    }
}

pub fn clipped_and_voids() {
    use hyperbrep::{boolean, builder::SphereVoid};
    let sphere = builder::sphere(5.into()).unwrap();
    let second = sphere
        .transformed(&Matrix4::affine_translation([0.into(), 0.into(), 6.into()]))
        .unwrap();
    let cylinder = builder::cylinder(3.into(), 12.into())
        .unwrap()
        .transformed(&Matrix4::affine_translation([
            0.into(),
            0.into(),
            (-6).into(),
        ]))
        .unwrap();
    let inner_cylinder = builder::cylinder(1.into(), 2.into())
        .unwrap()
        .transformed(&Matrix4::affine_translation([
            0.into(),
            0.into(),
            (-1).into(),
        ]))
        .unwrap();
    let half = builder::cuboid(point(-20, -20, 0), point(20, 20, 20)).unwrap();
    let longitudinal = builder::cuboid(point(0, -20, -20), point(20, 20, 20)).unwrap();
    let slab = builder::cuboid(point(-20, -20, -3), point(20, 20, 3)).unwrap();
    let torus = builder::torus(5.into(), 2.into()).unwrap();
    let cone = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let diagonal = (Real::one() / Real::from(2).sqrt().unwrap()).unwrap();
    let rotation = Matrix4::affine_orthonormal(
        [
            [diagonal.clone(), -diagonal.clone(), Real::zero()],
            [diagonal.clone(), diagonal, Real::zero()],
            [Real::zero(), Real::zero(), Real::one()],
        ],
        [Real::zero(), Real::zero(), Real::zero()],
    );
    let oblique_cut = longitudinal.transformed(&rotation).unwrap();
    let oblique_cone = cone.transformed(&rotation).unwrap();
    let torus_slab = builder::cuboid(point(-20, -20, -1), point(20, 20, 1)).unwrap();
    let profile = |points: &[(i32, i32)]| {
        points
            .iter()
            .map(|&(x, y)| Point2::new(x.into(), y.into()))
            .collect::<Vec<_>>()
    };
    let cases = vec![
        (
            "spherical void",
            builder::sphere_with_voids(
                5.into(),
                &[SphereVoid {
                    center: point(1, 0, 0),
                    radius: 1.into(),
                }],
            )
            .unwrap(),
            ratio(496, 3) * Real::pi(),
        ),
        (
            "sphere union",
            boolean::union(&sphere, &second).unwrap(),
            ratio(896, 3) * Real::pi(),
        ),
        (
            "sphere lens",
            boolean::intersection(&sphere, &second).unwrap(),
            ratio(104, 3) * Real::pi(),
        ),
        (
            "sphere difference",
            boolean::difference(&sphere, &second).unwrap(),
            Real::from(132) * Real::pi(),
        ),
        (
            "spherical cap",
            boolean::intersection(&sphere, &half).unwrap(),
            ratio(250, 3) * Real::pi(),
        ),
        (
            "spherical band",
            boolean::intersection(&sphere, &slab).unwrap(),
            Real::from(132) * Real::pi(),
        ),
        (
            "radial intersection",
            boolean::intersection(&sphere, &cylinder).unwrap(),
            ratio(244, 3) * Real::pi(),
        ),
        (
            "radial difference",
            boolean::difference(&sphere, &cylinder).unwrap(),
            ratio(256, 3) * Real::pi(),
        ),
        (
            "finite union",
            boolean::union(&sphere, &cylinder).unwrap(),
            ratio(580, 3) * Real::pi(),
        ),
        (
            "two components",
            boolean::difference(&cylinder, &sphere).unwrap(),
            ratio(80, 3) * Real::pi(),
        ),
        (
            "cylindrical void",
            boolean::difference(&sphere, &inner_cylinder).unwrap(),
            ratio(494, 3) * Real::pi(),
        ),
        (
            "sphere inside cylinder",
            boolean::difference(&cylinder, &builder::sphere(1.into()).unwrap()).unwrap(),
            ratio(320, 3) * Real::pi(),
        ),
        (
            "torus axial band",
            boolean::intersection(&torus, &torus_slab).unwrap(),
            ratio(40, 3) * Real::pi() * Real::pi()
                + Real::from(20) * Real::pi() * Real::from(3).sqrt().unwrap(),
        ),
        (
            "torus longitudinal half",
            boolean::intersection(&torus, &oblique_cut).unwrap(),
            Real::from(20) * Real::pi() * Real::pi(),
        ),
        (
            "cone half",
            boolean::intersection(&oblique_cone, &longitudinal).unwrap(),
            Real::from(39) * Real::pi(),
        ),
        (
            "revolution cavity",
            builder::revolve_region(
                &profile(&[(1, 0), (4, 0), (4, 4), (1, 4)]),
                &[profile(&[(2, 1), (3, 1), (3, 2), (2, 2)])],
            )
            .unwrap(),
            Real::from(55) * Real::pi(),
        ),
    ];
    let volume = |model: &Model| {
        model
            .solids()
            .map(|b| b.volume().unwrap())
            .fold(Real::zero(), |s, v| s + v)
    };
    for (name, source, expected) in cases {
        equal_named(
            &volume(source.model()),
            &expected,
            &format!("{name}: source"),
        );
        for scale in [ratio(127, 5000), Real::from(2).sqrt().unwrap()] {
            for reflected in [false, true] {
                let transform = similarity(&scale, reflected);
                let moved = source
                    .transformed(&transform)
                    .unwrap_or_else(|e| panic!("{name}: {e:?}"));
                let loaded = reload(moved.model());
                equal_named(
                    &volume(&loaded),
                    &(&expected * &scale * &scale * &scale),
                    &format!("{name}: scaled {scale:?} reflected={reflected}"),
                );
                for (before, after) in source.model().solids().zip(loaded.solids()) {
                    for point in [
                        point(0, 0, 0),
                        point(0, 0, 4),
                        point(2, 0, 1),
                        point(4, 0, 1),
                        point(8, 0, 0),
                        point(0, 0, -5),
                    ] {
                        assert_eq!(
                            after
                                .classify_point(&transform.transform_point3(&point).unwrap())
                                .unwrap(),
                            before.classify_point(&point).unwrap(),
                            "{name}"
                        );
                    }
                }
                check_images(&loaded);
            }
        }
    }
}

pub fn selected_and_repeated() {
    use hyperbrep::{Curve3, Curve3Kind, CurveParameterLocation, Parameter, boolean};
    let selector = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(ratio(n, 5), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(parameters) =
        selector.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("selected root")
    };
    let root: Parameter = parameters[0].clone();
    assert!(root.try_real().is_none());
    let source = builder::cylinder(5.into(), 7.into()).unwrap();
    let split = source
        .model()
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::Line)
        .unwrap()
        .split(root.clone())
        .unwrap();
    let mut current = split.model().clone();
    let map = similarity(&ratio(127, 5000), true);
    let inverse = map.clone().inverse().unwrap();
    for index in 0..3 {
        let moved = reload(&current.transformed(&map).unwrap());
        for (before, after) in current.edges().zip(moved.edges()) {
            assert_eq!(
                before
                    .domain()
                    .start()
                    .compare(after.domain().start())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            assert_eq!(
                before.domain().end().compare(after.domain().end()).unwrap(),
                std::cmp::Ordering::Equal
            );
        }
        assert_eq!(
            current
                .vertices()
                .filter(|v| v.point().try_coordinates().is_none())
                .count(),
            moved
                .vertices()
                .filter(|v| v.point().try_coordinates().is_none())
                .count()
        );
        let continued = moved
            .edges()
            .find(|e| e.domain().end().try_real().is_none())
            .unwrap()
            .split(ratio(32 + index, 64).into())
            .unwrap();
        current = reload(&continued.model().transformed(&inverse).unwrap());
        equal(
            &current.solids().next().unwrap().volume().unwrap(),
            &(Real::from(175) * Real::pi()),
        );
    }
    let result = BodySet::from_solid(current.solids().next().unwrap());
    equal(
        &boolean::intersection(&result, &source)
            .unwrap()
            .bodies()
            .next()
            .unwrap()
            .volume()
            .unwrap(),
        &(Real::from(175) * Real::pi()),
    );
    assert!(boolean::difference(&result, &source).unwrap().is_empty());
    let empty = BodySet::empty().transformed(&map).unwrap();
    assert!(empty.is_empty());
    assert_eq!(boolean::union(&empty, &result).unwrap().bodies().count(), 1);
}

pub fn rejection_and_recovery() {
    use hyperbrep::GeometryError;
    let source = builder::cylinder(5.into(), 7.into()).unwrap();
    let json = source.model().to_json().unwrap();
    let nonuniform = Matrix4::affine_nonuniform_scale([2.into(), 3.into(), 4.into()]);
    assert!(matches!(
        source.transformed(&nonuniform),
        Err(GeometryError::UnsupportedTransform)
    ));
    assert!(matches!(
        source.transformed(&Matrix4::affine_nonuniform_scale([
            1.into(),
            0.into(),
            1.into()
        ])),
        Err(GeometryError::SingularTransform)
    ));
    let mut perspective = Matrix4::identity();
    perspective.0[3][0] = ratio(1, 1000);
    assert!(matches!(
        source.transformed(&perspective),
        Err(GeometryError::NonAffineTransform)
    ));
    assert_eq!(source.model().to_json().unwrap(), json);
    let moved = source
        .transformed(&Matrix4::affine_nonuniform_scale([
            (-2).into(),
            (-2).into(),
            (-2).into(),
        ]))
        .unwrap();
    equal(
        &reload(moved.model())
            .solids()
            .next()
            .unwrap()
            .volume()
            .unwrap(),
        &(Real::from(1400) * Real::pi()),
    );
    let box_model = builder::cuboid(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .transformed(&nonuniform)
        .unwrap();
    equal(
        &reload(box_model.model())
            .solids()
            .next()
            .unwrap()
            .volume()
            .unwrap(),
        &Real::from(576),
    );
}

pub fn periodic_and_near() {
    for turns in [-1, 0, 2] {
        let source = periodic::build(&[[0, 4, 0, 1]], 1, turns).unwrap();
        let seam = source
            .edges()
            .find(|edge| edge.curve().kind() == hyperbrep::Curve3Kind::Line)
            .unwrap();
        let split = seam.split(ratio(1, 2).into()).unwrap();
        for reflected in [false, true] {
            let scale = ratio(1, 1000);
            let map = similarity(&scale, reflected);
            let moved = reload(&split.model().transformed(&map).unwrap());
            equal(
                &moved.solids().next().unwrap().volume().unwrap(),
                &(Real::from(16) * Real::pi() * &scale * &scale * &scale),
            );
            let gap = ratio(1, 1_000_000) * ratio(1, 1_000_000);
            for (x, expected) in [
                (Real::from(2) - &gap, SolidPointLocation::Inside),
                (Real::from(2), SolidPointLocation::Boundary),
                (Real::from(2) + gap, SolidPointLocation::Outside),
            ] {
                let probe = map
                    .transform_point3(&Point3::new(x, Real::zero(), Real::from(2)))
                    .unwrap();
                assert_eq!(
                    moved
                        .solids()
                        .next()
                        .unwrap()
                        .classify_point(&probe)
                        .unwrap(),
                    expected
                );
            }
            check_images(&moved);
        }
    }
}

/// Both ruled surface families retain every admitted spatial profile carrier.
/// Interior edge images and fresh raw validation are independent of cached shells.
pub fn profile_families() {
    use hyperbrep::{Curve3, Curve3Kind, Vector3};
    use hypercurve::{CircularArc2, Contour2, Curve2, CurvePath2, LineSeg2, Segment2};
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let profiles = [
        Curve3::line(point(2, 0, 0), point(4, 0, 2)).unwrap(),
        Curve3::circle_arc(
            point(4, 0, 0),
            Vector3::x(),
            Vector3::z(),
            1.into(),
            Real::zero(),
            quarter.clone(),
        )
        .unwrap(),
        Curve3::ellipse_arc(
            point(4, 0, 0),
            Vector3::x(),
            Vector3::z(),
            1.into(),
            2.into(),
            Real::zero(),
            quarter.clone(),
        )
        .unwrap(),
        Curve3::rational_bezier(
            vec![point(2, 0, 0), point(3, 0, 1), point(4, 0, 2)],
            vec![1.into(), 2.into(), 3.into()],
        )
        .unwrap(),
        Curve3::nurbs(
            2,
            vec![point(2, 0, 0), point(3, 0, 1), point(4, 0, 2)],
            vec![1.into(), 2.into(), 3.into()],
            vec![2.into(), 2.into(), 2.into(), 5.into(), 5.into(), 5.into()],
        )
        .unwrap(),
    ];
    for profile in profiles {
        let family = profile.kind();
        for (kind, source) in [
            (
                "extrusion",
                builder::extrusion_patch(profile.clone(), Vector3::y(), 0.into(), 3.into())
                    .unwrap(),
            ),
            (
                "revolution",
                builder::revolution_patch(
                    profile,
                    Point3::origin(),
                    Vector3::z(),
                    0.into(),
                    quarter.clone(),
                )
                .unwrap(),
            ),
        ] {
            assert!(
                source
                    .model()
                    .edges()
                    .any(|edge| edge.curve().kind() == family)
            );
            let edge = source
                .model()
                .edges()
                .find(|edge| edge.curve().kind() == family)
                .unwrap();
            let (start, end) = edge.domain().try_real_endpoints().unwrap();
            let split = edge
                .split(((start + end) / Real::from(2)).unwrap().into())
                .unwrap()
                .into_model();
            let reflected = split.transformed(&similarity(&Real::one(), true)).unwrap();
            check_images(&reflected);
            reload_named(&reflected, &format!("split reflected {kind} {family:?}"));
            if kind == "extrusion" {
                // The unused curve normal may scale independently: retain the
                // previous plane-isometry and polynomial-affine domain.
                let affine = source
                    .model()
                    .transformed(&Matrix4::affine_nonuniform_scale([
                        1.into(),
                        2.into(),
                        1.into(),
                    ]))
                    .unwrap();
                check_images(&reload_named(&affine, &format!("affine {family:?}")));
            }
            for scale in [Real::one(), ratio(127, 5000), Real::from(2).sqrt().unwrap()] {
                for reflected in [false, true] {
                    let transform = similarity(&scale, reflected);
                    let moved = source.model().transformed(&transform).unwrap();
                    check_images(&moved);
                    let replay = reload_named(
                        &moved,
                        &format!("{kind} {family:?} scale={scale:?} reflected={reflected}"),
                    );
                    if kind == "extrusion"
                        && family == Curve3Kind::RationalBezier
                        && reflected
                        && scale == Real::one()
                    {
                        // Endpoints still agree: only the complete interior
                        // profile changes. Reversed-image admission must reject it.
                        let mut raw: serde_json::Value =
                            serde_json::from_str(&moved.to_json().unwrap()).unwrap();
                        raw["surfaces"][0]["Extrusion"]["profile"]["RationalBezier"]["control_points"]
                            [1][0] = serde_json::to_value(Real::from(123)).unwrap();
                        assert!(matches!(
                            RawModel::from_json(&raw.to_string(), &ArchiveLimits::default())
                                .unwrap()
                                .validate(),
                            Err(hyperbrep::PersistenceError::Build(
                                hyperbrep::BuildError::EdgeUseSupportMismatch
                            ))
                        ));
                    }
                    for (before, after) in source.model().edges().zip(replay.edges()) {
                        assert_eq!(
                            before.curve().kind(),
                            after.curve().kind(),
                            "{kind} {family:?}"
                        );
                        let (start, end) = before.domain().try_real_endpoints().unwrap();
                        for t in [
                            start.clone(),
                            ((start + end) / Real::from(2)).unwrap(),
                            end.clone(),
                        ] {
                            let expected = transform
                                .transform_point3(
                                    before
                                        .curve()
                                        .point_at(&t.clone().into())
                                        .unwrap()
                                        .try_coordinates()
                                        .unwrap(),
                                )
                                .unwrap();
                            assert!(
                                after
                                    .curve()
                                    .point_at(&t.into())
                                    .unwrap()
                                    .coincides_with(&expected.into())
                                    .unwrap(),
                                "{kind} {family:?} reflected={reflected}"
                            );
                        }
                    }
                }
            }
        }
    }
    let cp = |x, y| hypercurve::Point2::new(Real::from(x), Real::from(y));
    let contour = Contour2::try_new(vec![
        Segment2::Arc(CircularArc2::try_from_center(cp(4, 0), cp(2, 0), cp(3, 0), false).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(cp(2, 0), cp(4, 0), cp(3, 0), false).unwrap()),
    ])
    .unwrap();
    let curved = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(cp(2, 0), cp(4, 0)).unwrap()),
        Curve2::try_nurbs(
            2,
            vec![cp(4, 0), cp(5, 1), cp(4, 2)],
            vec![Real::one(); 3],
            vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
        )
        .unwrap(),
        Curve2::from(LineSeg2::try_new(cp(4, 2), cp(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(cp(2, 2), cp(2, 0)).unwrap()),
    ])
    .unwrap();
    for (source, expected, family) in [
        (
            builder::revolve_contour(&contour).unwrap(),
            Real::from(6) * Real::pi() * Real::pi(),
            Curve3Kind::CircleArc,
        ),
        (
            builder::revolve_path(&curved).unwrap(),
            ratio(148, 5) * Real::pi(),
            Curve3Kind::Nurbs,
        ),
    ] {
        assert!(source.model().edges().any(|e| e.curve().kind() == family));
        for scale in [ratio(127, 5000), Real::from(2).sqrt().unwrap()] {
            for reflected in [false, true] {
                let transform = similarity(&scale, reflected);
                let moved = source.transformed(&transform).unwrap();
                let loaded = reload(moved.model());
                equal(
                    &loaded.solids().next().unwrap().volume().unwrap(),
                    &(&expected * &scale * &scale * &scale),
                );
                check_images(&loaded);
                assert_eq!(
                    loaded
                        .solids()
                        .next()
                        .unwrap()
                        .classify_point(
                            &transform
                                .transform_point3(&point(
                                    3,
                                    0,
                                    if family == Curve3Kind::Nurbs { 1 } else { 0 }
                                ))
                                .unwrap()
                        )
                        .unwrap(),
                    SolidPointLocation::Inside
                );
            }
        }
    }
}
