//! R21: exact straight images with isolated stationary path parameters.
use hyperbrep::*;

pub(super) fn q(a: i64, b: i64) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

pub(super) fn square() -> Vec<Point2> {
    [(0, 0), (1, 0), (1, 1), (0, 1)]
        .map(|(x, y)| Point2::new(x.into(), y.into()))
        .to_vec()
}

fn vertical(levels: Vec<Real>) -> Curve3 {
    let weights = vec![Real::one(); levels.len()];
    Curve3::rational_bezier(
        levels
            .into_iter()
            .map(|z| Point3::new(0.into(), 0.into(), z))
            .collect(),
        weights,
    )
    .unwrap()
}

fn surface_area(set: &BodySet) -> Real {
    set.model()
        .faces()
        .fold(Real::zero(), |sum, face| sum + face.area().unwrap())
}

pub fn stationary_straight_images_construct_the_same_exact_prism() {
    for (index, levels) in [
        vec![0.into(), 1.into()],
        vec![0.into(), 0.into(), 1.into()],
        vec![0.into(), 0.into(), 0.into(), 1.into()],
        // 4(t - 1/2)^3 + 1/2, with an isolated interior stationary point.
        vec![0.into(), 1.into(), 0.into(), 1.into()],
        // 16(t - 1/2)^5 + 1/2: a fourth-order stationary derivative.
        vec![0.into(), 1.into(), 0.into(), 1.into(), 0.into(), 1.into()],
    ]
    .into_iter()
    .enumerate()
    {
        let swept = builder::sweep_curve(
            &square(),
            Vector3::x(),
            Vector3::y(),
            vertical(levels.clone()),
        )
        .expect("strictly increasing straight image must be an ordinary prism");
        let solid = swept.bodies().next().unwrap();
        if index > 0 {
            assert_eq!(
                surface_area(&swept).exact_rational_normal_form(),
                Real::from(6).exact_rational_normal_form()
            );
        }
        assert_eq!(
            solid.volume().unwrap().exact_rational_normal_form(),
            Real::one().exact_rational_normal_form()
        );
        for (z, expected) in [
            (q(-1, 4), SolidPointLocation::Outside),
            (0.into(), SolidPointLocation::Boundary),
            (q(1, 2), SolidPointLocation::Inside),
            (1.into(), SolidPointLocation::Boundary),
            (q(5, 4), SolidPointLocation::Outside),
        ] {
            assert_eq!(
                solid
                    .classify_point(&Point3::new(q(1, 2), q(1, 2), z))
                    .unwrap(),
                expected
            );
        }
        if index > 0 {
            for face in swept.model().faces() {
                assert_eq!(face.surface().kind(), SurfaceKind::Plane);
                for (u, v) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                    let normal = face
                        .surface()
                        .normal_at(&hypercurve::Point2::from_values(u, v).into())
                        .unwrap();
                    let normal = normal.try_coordinates().unwrap();
                    assert_eq!(
                        hyperlimit::compare_reals(
                            &normal.norm_squared(),
                            &Real::one(),
                            hyperlimit::PredicatePolicy::STRICT
                        )
                        .value(),
                        Some(std::cmp::Ordering::Equal)
                    );
                }
            }
        }
        let expected = match index {
            0 => q(1, 4),
            1 => q(1, 16),
            2 => q(1, 64),
            3 => q(7, 16),
            _ => q(31, 64),
        };
        if index == 0 {
            let authored = swept
                .model()
                .edges()
                .find(|edge| {
                    edge.curve().kind() == Curve3Kind::RationalBezier
                        && edge
                            .start()
                            .point()
                            .try_coordinates()
                            .is_some_and(|p| p.x == Real::zero() && p.y == Real::zero())
                })
                .unwrap();
            assert_eq!(
                authored
                    .curve()
                    .point_at(&q(1, 4).into())
                    .unwrap()
                    .try_coordinates()
                    .unwrap()
                    .z
                    .exact_rational_normal_form(),
                expected.exact_rational_normal_form()
            );
        } else {
            let usage = swept
                .model()
                .wires()
                .flat_map(|wire| wire.edge_uses())
                .find(|usage| {
                    usage.direction() == Direction::Forward
                        && matches!(
                            usage.parameter_correspondence(),
                            ParameterCorrespondence::Coordinate { .. }
                        )
                })
                .expect("authored progress retained in pcurve correspondence");
            let mapped = usage.edge_parameter_at(&q(1, 4).into()).unwrap();
            assert_eq!(
                mapped.compare(&expected.clone().into()).unwrap(),
                std::cmp::Ordering::Equal
            );
            let inverse = usage.pcurve_parameter_at(&expected.into()).unwrap();
            assert_eq!(
                inverse.compare(&q(1, 4).into()).unwrap(),
                std::cmp::Ordering::Equal
            );
            if index >= 3 {
                let selected = usage.pcurve_parameter_at(&q(1, 4).into()).unwrap();
                let mapped = usage.edge_parameter_at(&selected).unwrap();
                assert_eq!(
                    mapped.compare(&q(1, 4).into()).unwrap(),
                    std::cmp::Ordering::Equal
                );
                #[cfg(not(target_arch = "wasm32"))]
                eprintln!(
                    "R21 degree-{} quarter-height inverse has scalar view: {}",
                    levels.len() - 1,
                    selected.try_real().is_some()
                );
                assert_eq!(
                    usage
                        .pcurve_parameter_at(&q(1, 2).into())
                        .unwrap()
                        .compare(&q(1, 2).into())
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
            }
        }
        let canonical =
            builder::cuboid(Point3::origin(), Point3::new(1.into(), 1.into(), 1.into())).unwrap();
        assert!(
            boolean::symmetric_difference(&swept, &canonical)
                .expect("whole-set equality with independent cuboid")
                .is_empty()
        );
    }
}

pub(super) fn shifted_cubic() -> Curve3 {
    vertical(vec![0.into(), 1.into(), 0.into(), 1.into()])
}

pub fn stationary_straight_images_continue_through_cuts_replay_and_reunion() {
    let swept =
        builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    let json = swept.model().to_json().unwrap();
    let model = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), json);
    let replayed = BodySet::from_solid(model.solids().next().unwrap());
    let slab = builder::cuboid(
        Point3::new((-1).into(), (-1).into(), q(1, 4)),
        Point3::new(2.into(), 2.into(), q(3, 4)),
    )
    .unwrap();
    let middle = boolean::intersection(&replayed, &slab).expect("stationary image cap section");
    let ends = boolean::difference(&replayed, &slab).expect("stationary image cap cuts");
    assert_eq!(middle.len(), 1);
    assert_eq!(ends.len(), 2);
    let middle_json = middle.model().to_json().unwrap();
    let middle_model = RawModel::from_json(&middle_json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let cold_middle = BodySet::from_solid(middle_model.solids().next().unwrap());
    let inner_slab = builder::cuboid(
        Point3::new((-1).into(), (-1).into(), q(3, 8)),
        Point3::new(2.into(), 2.into(), q(5, 8)),
    )
    .unwrap();
    let center = boolean::intersection(&cold_middle, &inner_slab).unwrap();
    let outer = boolean::difference(&cold_middle, &inner_slab).unwrap();
    assert_eq!(center.len(), 1);
    assert_eq!(outer.len(), 2);
    let reconstructed = boolean::union(&center, &outer).unwrap();
    assert!(
        boolean::symmetric_difference(&reconstructed, &middle)
            .unwrap()
            .is_empty()
    );
    let reunion = boolean::union(&middle, &ends).expect("cut image can be reconstructed");
    assert!(
        boolean::symmetric_difference(&reunion, &replayed)
            .unwrap()
            .is_empty()
    );
    let json = reunion.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let restored = BodySet::from_solid(cold.solids().next().unwrap());
    assert!(
        boolean::symmetric_difference(&restored, &swept)
            .unwrap()
            .is_empty()
    );
}

pub fn stationary_straight_images_reject_backtracking_and_lateral_stops() {
    for path in [
        vertical(vec![0.into(), 1.into(), q(1, 2)]),
        vertical(vec![0.into(), q(6, 25), -q(1, 50), q(11, 50)]),
        Curve3::rational_bezier(
            vec![
                Point3::origin(),
                Point3::new(1.into(), 0.into(), 0.into()),
                Point3::new(0.into(), 0.into(), 1.into()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap(),
        vertical(vec![0.into(), 0.into(), 0.into()]),
    ] {
        assert_eq!(
            builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), path).unwrap_err(),
            ConstructionError::NonMonotoneSweepPath
        );
    }
    assert!(
        Curve3::rational_bezier(
            vec![Point3::origin(), Point3::new(0.into(), 0.into(), 1.into())],
            vec![Real::one(), Real::zero()]
        )
        .is_err()
    );
}

fn elevate(levels: &[Real]) -> Vec<Real> {
    let count = levels.len();
    let mut elevated = vec![levels[0].clone()];
    for index in 1..count {
        elevated.push(
            ((&levels[index - 1] * Real::from(index as u64)
                + &levels[index] * Real::from((count - index) as u64))
                / Real::from(count as u64))
            .unwrap(),
        );
    }
    elevated.push(levels[count - 1].clone());
    elevated
}

pub fn stationary_straight_images_preserve_elevation_weights_oblique_frames_and_reversal() {
    let cubic = vec![0.into(), 1.into(), 0.into(), 1.into()];
    for (levels, weights) in [
        (cubic.clone(), vec![Real::one(); 4]),
        (elevate(&cubic), vec![Real::one(); 5]),
        (elevate(&elevate(&cubic)), vec![Real::one(); 6]),
        (
            vec![0.into(), 0.into(), 1.into()],
            vec![2.into(), 3.into(), 5.into()],
        ),
    ] {
        let controls = levels
            .iter()
            .map(|z| {
                Point3::new(
                    Real::one() + q(1, 3) * z,
                    Real::from(2) - q(2, 5) * z,
                    Real::from(3) + z,
                )
            })
            .collect::<Vec<_>>();
        let forward = builder::sweep_curve(
            &square(),
            Vector3::x(),
            Vector3::y(),
            Curve3::rational_bezier(controls.clone(), weights.clone()).unwrap(),
        )
        .unwrap();
        let reversed = builder::sweep_curve(
            &square(),
            Vector3::y(),
            Vector3::x(),
            Curve3::rational_bezier(
                controls.into_iter().rev().collect(),
                weights.into_iter().rev().collect(),
            )
            .unwrap(),
        )
        .unwrap();
        assert!(
            boolean::symmetric_difference(&forward, &reversed)
                .unwrap()
                .is_empty()
        );
        for z in [-1, 0, 1, 2, 3, 4, 5].map(|x| q(x, 4)) {
            for x in [-1, 0, 2, 4, 5].map(|x| q(x, 4)) {
                for y in [-1, 0, 2, 4, 5].map(|x| q(x, 4)) {
                    let source = [&x, &y, &z];
                    let order = |x: &Real, y: &Real| {
                        hyperlimit::compare_reals(x, y, hyperlimit::PredicatePolicy::STRICT)
                            .value()
                            .unwrap()
                    };
                    let expected =
                        if source.iter().any(|v| {
                            order(v, &Real::zero()).is_lt() || order(v, &Real::one()).is_gt()
                        }) {
                            SolidPointLocation::Outside
                        } else if source.iter().any(|v| {
                            order(v, &Real::zero()).is_eq() || order(v, &Real::one()).is_eq()
                        }) {
                            SolidPointLocation::Boundary
                        } else {
                            SolidPointLocation::Inside
                        };
                    let world = Point3::new(
                        Real::one() + &x + q(1, 3) * &z,
                        Real::from(2) + &y - q(2, 5) * &z,
                        Real::from(3) + &z,
                    );
                    for solid in [&forward, &reversed] {
                        assert_eq!(
                            solid
                                .bodies()
                                .next()
                                .unwrap()
                                .classify_point(&world)
                                .unwrap(),
                            expected
                        );
                    }
                }
            }
        }
    }
}

pub fn stationary_straight_images_keep_holes_and_tiny_material() {
    let outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
        .map(|(x, y)| Point2::new(x.into(), y.into()))
        .to_vec();
    let holes = vec![
        [(1, 1), (3, 1), (3, 3), (1, 3)]
            .map(|(x, y)| Point2::new(x.into(), y.into()))
            .to_vec(),
    ];
    let swept =
        builder::sweep_curve_region(&outer, &holes, Vector3::x(), Vector3::y(), shifted_cubic())
            .unwrap();
    let reference = builder::extrude_region(&outer, &holes, Real::zero(), Real::one()).unwrap();
    assert_eq!(
        surface_area(&swept).exact_rational_normal_form(),
        Real::from(48).exact_rational_normal_form()
    );
    assert!(
        boolean::symmetric_difference(&swept, &reference)
            .unwrap()
            .is_empty()
    );
    assert_eq!(
        swept
            .bodies()
            .next()
            .unwrap()
            .classify_point(&Point3::new(2.into(), 2.into(), q(1, 2)))
            .unwrap(),
        SolidPointLocation::Outside
    );
    let tiny = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let narrow = vec![
        Point2::new(0.into(), 0.into()),
        Point2::new(tiny.clone(), 0.into()),
        Point2::new(tiny.clone(), 1.into()),
        Point2::new(0.into(), 1.into()),
    ];
    let swept = builder::sweep_curve(&narrow, Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    assert_eq!(
        swept
            .bodies()
            .next()
            .unwrap()
            .volume()
            .unwrap()
            .exact_rational_normal_form(),
        tiny.exact_rational_normal_form()
    );
    assert_eq!(
        swept
            .bodies()
            .next()
            .unwrap()
            .classify_point(&Point3::new(
                (&tiny / Real::from(2)).unwrap(),
                q(1, 2),
                q(1, 2)
            ))
            .unwrap(),
        SolidPointLocation::Inside
    );
    let height = Real::from(2).root_n(5).unwrap();
    let path = vertical(vec![
        Real::zero(),
        height.clone(),
        Real::zero(),
        height.clone(),
    ]);
    let algebraic = builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), path).unwrap();
    let reference = builder::cuboid(
        Point3::origin(),
        Point3::new(Real::one(), Real::one(), height.clone()),
    )
    .unwrap();
    assert!(
        boolean::symmetric_difference(&algebraic, &reference)
            .unwrap()
            .is_empty()
    );
    let json = algebraic.model().to_json().unwrap();
    let replayed = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let body = replayed.solids().next().unwrap();
    assert_eq!(
        hyperlimit::compare_reals(
            &body.volume().unwrap(),
            &height,
            hyperlimit::PredicatePolicy::STRICT
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        body.classify_point(&Point3::new(
            q(1, 2),
            q(1, 2),
            (&height / Real::from(2)).unwrap()
        ))
        .unwrap(),
        SolidPointLocation::Inside
    );
}

pub fn stationary_plane_correspondence_rejects_changed_whole_image_and_backtracking() {
    let swept =
        builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    let model = swept.model();
    let original = model.to_json().unwrap();
    let usage = model
        .wires()
        .flat_map(|wire| wire.edge_uses())
        .find(|usage| {
            usage.direction() == Direction::Forward
                && matches!(
                    usage.parameter_correspondence(),
                    ParameterCorrespondence::Coordinate { .. }
                )
        })
        .unwrap();
    let start = usage.pcurve().point_at(&Real::zero().into()).unwrap();
    let constant = start.coordinates().unwrap().x().clone();
    for (levels, lateral) in [([0, 2, -1, 1], false), ([0, 1, 0, 1], true)] {
        let points = levels
            .into_iter()
            .enumerate()
            .map(|(index, level)| {
                hypercurve::Point2::new(
                    if lateral && index == 1 {
                        &constant + q(1, 100)
                    } else {
                        constant.clone()
                    },
                    level.into(),
                )
            })
            .collect();
        let replacement = Pcurve::new(hypercurve::Curve2::from(
            hypercurve::RationalBezier2::try_new(points, vec![Real::one(); 4]).unwrap(),
        ));
        let mut edit = model.edit();
        edit.replace_pcurve(usage.pcurve(), replacement).unwrap();
        assert!(
            edit.commit().is_err(),
            "changed interior must invalidate full-image authority"
        );
        assert_eq!(model.to_json().unwrap(), original);
    }
}

pub fn stationary_straight_images_continue_through_reflection_and_fixed_frame_entry() {
    let swept =
        builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    let transform = Matrix4::affine_nonuniform_scale([(-2).into(), 3.into(), 4.into()]);
    let image = swept.transformed(&transform).unwrap();
    let reference = builder::cuboid(
        Point3::new((-2).into(), 0.into(), 0.into()),
        Point3::new(0.into(), 3.into(), 4.into()),
    )
    .unwrap();
    assert!(
        boolean::symmetric_difference(&image, &reference)
            .unwrap()
            .is_empty()
    );
    let json = image.model().to_json().unwrap();
    let replayed = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let replayed = BodySet::from_solid(replayed.solids().next().unwrap());
    assert!(
        boolean::symmetric_difference(&replayed, &reference)
            .unwrap()
            .is_empty()
    );
    let origins = vec![
        Point3::origin(),
        Point3::origin(),
        Point3::new(0.into(), 0.into(), 1.into()),
    ];
    let fixed = RationalBezierSweepFrame::try_new(
        origins.clone(),
        vec![Vector3::x(); 3],
        vec![Vector3::y(); 3],
        vec![Real::one(); 3],
    )
    .unwrap();
    let image = builder::sweep_moving_frame(&square(), fixed).unwrap();
    assert!(
        boolean::symmetric_difference(&image, &swept)
            .unwrap()
            .is_empty()
    );
    let moving = RationalBezierSweepFrame::try_new(
        origins,
        vec![
            Vector3::x(),
            Vector3::x(),
            Vector3::from_xyz(2.into(), 0.into(), 0.into()),
        ],
        vec![Vector3::y(); 3],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert_eq!(
        builder::sweep_moving_frame(&square(), moving).unwrap_err(),
        ConstructionError::NonMonotoneSweepPath
    );
}

#[cfg(feature = "tessellation")]
pub fn stationary_straight_images_have_exact_planar_triangulations() {
    fn check(
        set: &BodySet,
        expected_six_volume: i64,
        expected_euler: i64,
        expected_triangles: usize,
    ) {
        use std::collections::{BTreeMap, BTreeSet};
        let mut vertices = BTreeSet::new();
        let mut edges = BTreeMap::new();
        let mut triangles = 0;
        let mut six_volume = Real::zero();
        for body in set.bodies() {
            for face in body.outer().faces() {
                let mesh = tessellation::triangulate_planar_face(face)
                    .expect("canonical spatial lines bound exact planar faces");
                let policy = tessellation::ChordalApproximationPolicy::uniform(
                    std::num::NonZeroUsize::new(3).unwrap(),
                    1,
                );
                let sampled = tessellation::approximate_face_chordally(face, policy)
                    .expect("authored and selected pcurve domains remain sampleable");
                assert_eq!(
                    sampled.source_relation(),
                    tessellation::ChordalSourceRelation::ExactAtVerticesOnly
                );
                assert_eq!(sampled.policy(), policy);
                assert!(sampled.source_face().model().same_revision(set.model()));
                for (uv, point) in sampled.parameters().iter().zip(sampled.points()) {
                    let source = face
                        .surface()
                        .point_at(&hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into())
                        .unwrap();
                    assert!(source.coincides_with(&point.clone().into()).unwrap());
                }
                let keys = mesh
                    .points()
                    .iter()
                    .map(|point| {
                        let coordinate = |value: &Real, choices: &[Real]| {
                            choices
                                .iter()
                                .position(|candidate| {
                                    hyperlimit::compare_reals(
                                        value,
                                        candidate,
                                        hyperlimit::PredicatePolicy::STRICT,
                                    )
                                    .value()
                                        == Some(std::cmp::Ordering::Equal)
                                })
                                .expect("independent exact prism corner lattice")
                        };
                        [
                            coordinate(&point.x, &[0.into(), 1.into(), 3.into(), 4.into()]),
                            coordinate(&point.y, &[0.into(), 1.into(), 3.into(), 4.into()]),
                            coordinate(&point.z, &[0.into(), q(1, 4), q(3, 4), 1.into()]),
                        ]
                    })
                    .collect::<Vec<_>>();
                for triangle in mesh.triangles() {
                    let [a, b, c] = triangle.map(|index| &mesh.points()[index]);
                    six_volume += Vector3::from(a.clone())
                        .dot(&Vector3::from(b.clone()).cross(&Vector3::from(c.clone())));
                    let indices = triangle.map(|index| keys[index]);
                    for index in indices {
                        vertices.insert(index);
                    }
                    for [a, b] in [
                        [indices[0], indices[1]],
                        [indices[1], indices[2]],
                        [indices[2], indices[0]],
                    ] {
                        assert_ne!(a, b);
                        *edges
                            .entry(if a < b { (a, b) } else { (b, a) })
                            .or_insert(0) += 1;
                    }
                    triangles += 1;
                }
            }
        }
        assert_eq!(triangles, expected_triangles);
        assert!(
            edges.values().all(|count| *count == 2),
            "complete closed triangle incidence"
        );
        assert_eq!(
            vertices.len() as i64 - edges.len() as i64 + triangles as i64,
            expected_euler
        );
        assert_eq!(
            six_volume.exact_rational_normal_form(),
            Real::from(expected_six_volume).exact_rational_normal_form()
        );
    }
    let cube =
        builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    check(&cube, 6, 2, 12);
    let side = cube
        .bodies()
        .next()
        .unwrap()
        .outer()
        .faces()
        .find(|face| {
            face.outer().unwrap().edge_uses().any(|usage| {
                matches!(
                    usage.parameter_correspondence(),
                    ParameterCorrespondence::Coordinate { .. }
                )
            })
        })
        .unwrap();
    let sampled = tessellation::approximate_face_chordally(
        side,
        tessellation::ChordalApproximationPolicy::uniform(
            std::num::NonZeroUsize::new(3).unwrap(),
            1,
        ),
    )
    .unwrap();
    assert!(
        sampled
            .points()
            .iter()
            .any(|point| hyperlimit::compare_reals(
                &point.z,
                &q(13, 27),
                hyperlimit::PredicatePolicy::STRICT
            )
            .value()
                == Some(std::cmp::Ordering::Equal)),
        "chordal policy retains authored t=1/3 sampling rather than uniform spatial progress"
    );
    let outer = [(0, 0), (4, 0), (4, 4), (0, 4)]
        .map(|(x, y)| Point2::new(x.into(), y.into()))
        .to_vec();
    let holes = vec![
        [(1, 1), (3, 1), (3, 3), (1, 3)]
            .map(|(x, y)| Point2::new(x.into(), y.into()))
            .to_vec(),
    ];
    let holed =
        builder::sweep_curve_region(&outer, &holes, Vector3::x(), Vector3::y(), shifted_cubic())
            .unwrap();
    check(&holed, 72, 0, 32);
    let slab = builder::cuboid(
        Point3::new((-1).into(), (-1).into(), q(1, 4)),
        Point3::new(2.into(), 2.into(), q(3, 4)),
    )
    .unwrap();
    let ends = boolean::difference(&cube, &slab).unwrap();
    check(&ends, 3, 4, 24);
}
