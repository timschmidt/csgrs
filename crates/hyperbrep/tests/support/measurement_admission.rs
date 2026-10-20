use hyperbrep::*;
use hypercurve::{Curve2, CurvePath2, CurveRegion2, FillRule, LineSeg2, RationalBezier2};

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::from_values(x, y)
}
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn line(a: hypercurve::Point2, b: hypercurve::Point2) -> Curve2 {
    LineSeg2::try_new(a, b).unwrap().into()
}

pub fn planar_spline_incidence_does_not_require_low_degree_root_solving() {
    let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    for degree in [3, 5] {
        for nurbs in [false, true] {
            let controls = (0..degree + if nurbs { 2 } else { 1 })
                .map(|i| {
                    Point3::new(
                        Real::from(u32::try_from(i).unwrap()),
                        Real::from(u32::try_from(i * i).unwrap()),
                        Real::zero(),
                    )
                })
                .collect::<Vec<_>>();
            let weights = (1..=controls.len())
                .map(|i| Real::from(u32::try_from(i).unwrap()))
                .collect::<Vec<_>>();
            let build = |controls: Vec<Point3>| {
                if nurbs {
                    Curve3::nurbs(
                        degree,
                        controls,
                        weights.clone(),
                        [
                            vec![Real::from(2); degree + 1],
                            vec![Real::from(3)],
                            vec![Real::from(4); degree + 1],
                        ]
                        .concat(),
                    )
                    .unwrap()
                } else {
                    Curve3::rational_bezier(controls, weights.clone()).unwrap()
                }
            };
            let curve = build(controls.clone());
            assert!(matches!(
                plane.intersect_curve(&curve).unwrap(),
                CurveSurfaceIntersection::Contained
            ));
            let pcurve = plane.planar_pcurve(&curve).unwrap().unwrap();
            assert_eq!(
                pcurve.curve().family(),
                if nurbs {
                    hypercurve::CurveFamily2::Nurbs
                } else {
                    hypercurve::CurveFamily2::RationalBezier
                }
            );
            let domain = pcurve.curve().parameter_domain();
            let middle = ((domain.start().scalar().unwrap() + domain.end().scalar().unwrap())
                / Real::from(2))
            .unwrap();
            let uv = pcurve.curve().point_at(&middle.clone().into()).unwrap();
            let uv = uv.coordinates().unwrap();
            let point = plane
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
            assert!(
                curve
                    .point_at(&pcurve.spatial_parameter_at(&middle.into()).unwrap())
                    .unwrap()
                    .coincides_with(&point.into())
                    .unwrap()
            );
            // Both endpoints remain on the plane; an interior coefficient proves
            // that endpoint incidence alone cannot establish complete containment.
            let mut bowed = controls;
            bowed[1].z = Real::one();
            let bowed = build(bowed);
            assert!(plane.planar_pcurve(&bowed).unwrap().is_none());
            // Only the endpoints vanish: the positive interior Bernstein/B-spline
            // coefficient has positive basis support throughout the open domain.
            let CurveSurfaceIntersection::Points(contacts) = plane.intersect_curve(&bowed).unwrap()
            else {
                panic!("complete endpoint contacts");
            };
            assert_eq!(contacts.len(), 2);
            for (contact, endpoint, kind) in [
                (
                    &contacts[0],
                    bowed.domain().start(),
                    CurveSurfaceContactKind::Simple,
                ),
                (
                    &contacts[1],
                    bowed.domain().end(),
                    CurveSurfaceContactKind::Repeated,
                ),
            ] {
                assert_eq!(
                    contact.parameter.compare(endpoint).unwrap(),
                    std::cmp::Ordering::Equal
                );
                assert_eq!(contact.kind, kind);
                assert_eq!(
                    contact
                        .point
                        .compare_linear_form(&Vector3::z(), &Real::zero())
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
            }
            let middle = ((bowed.domain().represented_start().unwrap()
                + bowed.domain().represented_end().unwrap())
                / Real::from(2))
            .unwrap();
            assert_eq!(
                bowed
                    .point_at(&middle.into())
                    .unwrap()
                    .compare_linear_form(&Vector3::z(), &Real::zero())
                    .unwrap(),
                std::cmp::Ordering::Greater
            );
        }
    }
}

pub fn profile(nurbs: bool, reverse: bool, components: bool) -> CurveRegion2 {
    let path = |offset: i32| {
        let controls = vec![
            p(offset, 0),
            p(offset + 1, -1),
            p(offset + 2, -1),
            p(offset + 3, -1),
            p(offset + 4, -1),
            p(offset + 5, 0),
        ];
        let weights = vec![1.into(), 2.into(), 3.into(), 4.into(), 5.into(), 1.into()];
        let curve = if nurbs {
            Curve2::try_nurbs(
                5,
                controls,
                weights,
                [vec![Real::from(2); 6], vec![Real::from(4); 6]].concat(),
            )
            .unwrap()
        } else {
            RationalBezier2::try_new(controls, weights).unwrap().into()
        };
        let path = CurvePath2::try_new(vec![
            curve,
            line(p(offset + 5, 0), p(offset + 5, 2)),
            line(p(offset + 5, 2), p(offset, 2)),
            line(p(offset, 2), p(offset, 0)),
        ])
        .unwrap();
        assert!(
            path.boundary_loop()
                .unwrap()
                .signed_area()
                .unwrap()
                .is_none()
        );
        if reverse {
            path.reversed().unwrap()
        } else {
            path
        }
    };
    let mut paths = vec![path(0)];
    if components {
        let points = [
            p(1, 1),
            p(3, 1),
            hypercurve::Point2::new(3.into(), q(3, 2)),
            hypercurve::Point2::new(1.into(), q(3, 2)),
        ];
        let hole = CurvePath2::try_new(
            (0..4)
                .map(|i| line(points[i].clone(), points[(i + 1) % 4].clone()))
                .collect(),
        )
        .unwrap();
        paths.extend([
            if reverse {
                hole.reversed().unwrap()
            } else {
                hole
            },
            path(8),
        ]);
    }
    let region = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(region.signed_area().unwrap().is_none());
    region
}

pub fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), text);
    cold
}

pub fn verify(model: &Model, components: bool, map: &Matrix4) {
    assert_eq!(model.solids().count(), if components { 2 } else { 1 });
    for solid in model.solids() {
        assert_eq!(
            solid.volume(),
            Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
        );
    }
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    // Bernstein coefficients at t=1/2 give (295/107,-105/107).
    // Positive weights and increasing control x prove this is a graph.
    let gap: Real = "1e-12".parse().unwrap();
    let mut cases = vec![
        (q(295, 107), q(-105, 107), SolidPointLocation::Boundary),
        (q(295, 107), q(-105, 107) + &gap, SolidPointLocation::Inside),
        (
            q(295, 107),
            q(-105, 107) - &gap,
            SolidPointLocation::Outside,
        ),
        (2.into(), 0.into(), SolidPointLocation::Inside),
        (2.into(), 2.into(), SolidPointLocation::Boundary),
        (2.into(), Real::from(2) - &gap, SolidPointLocation::Inside),
        (2.into(), Real::from(2) + &gap, SolidPointLocation::Outside),
        ((-1).into(), 1.into(), SolidPointLocation::Outside),
    ];
    if components {
        cases.extend([
            (2.into(), q(5, 4), SolidPointLocation::Outside),
            (2.into(), 1.into(), SolidPointLocation::Boundary),
            (2.into(), Real::one() - &gap, SolidPointLocation::Inside),
            (2.into(), Real::one() + &gap, SolidPointLocation::Outside),
            (10.into(), 1.into(), SolidPointLocation::Inside),
        ]);
    }
    for (x, y, expected) in cases {
        let point = map.transform_point3(&Point3::new(x, y, 1.into())).unwrap();
        let mut actual = SolidPointLocation::Outside;
        for solid in model.solids() {
            let location = solid.classify_point(&point).unwrap();
            if location != SolidPointLocation::Outside {
                actual = location;
                break;
            }
        }
        assert_eq!(actual, expected);
    }
}

fn path_regions(region: &CurveRegion2, reverse: bool) -> Vec<(CurvePath2, Vec<CurvePath2>)> {
    region
        .material_components()
        .unwrap()
        .iter()
        .map(|component| {
            let mut paths = component
                .boundary_loops()
                .iter()
                .map(|boundary| {
                    let path = CurvePath2::try_new(boundary.curves().to_vec()).unwrap();
                    if reverse {
                        path.reversed().unwrap()
                    } else {
                        path
                    }
                })
                .collect::<Vec<_>>();
            (paths.remove(0), paths)
        })
        .collect()
}

pub fn rational_prism_validity_does_not_require_a_closed_form_measurement() {
    for nurbs in [false, true] {
        for reverse in [false, true] {
            let region = profile(nurbs, reverse, false);
            let paths = path_regions(&region, reverse);
            let path = &paths[0].0;
            for result in [
                builder::extrude_path(path, 0.into(), 3.into()),
                builder::extrude_path_region(path, &[], 0.into(), 3.into()),
                builder::extrude_path_regions(&paths, 0.into(), 3.into()),
            ] {
                let bodies = result.unwrap();
                verify(bodies.model(), false, &Matrix4::identity());
                verify(&replay(bodies.model()), false, &Matrix4::identity());
            }
            let sheet =
                builder::planar_face(path, &[], Point3::origin(), Vector3::x(), Vector3::y())
                    .unwrap();
            for model in [sheet.model().clone(), replay(sheet.model())] {
                assert_eq!(model.solids().count(), 0);
                assert_eq!(model.faces().count(), 1);
                assert_eq!(
                    model.faces().next().unwrap().area(),
                    Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
                );
            }
            let outside_hole = CurvePath2::try_new(vec![
                line(p(6, 0), p(7, 0)),
                line(p(7, 0), p(7, 1)),
                line(p(7, 1), p(6, 1)),
                line(p(6, 1), p(6, 0)),
            ])
            .unwrap();
            assert!(matches!(
                builder::extrude_path_region(path, &[outside_hole], 0.into(), 3.into()),
                Err(ConstructionError::HoleOutside)
            ));
            let result = hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || {
                builder::extrude_curve_region(&region, 0.into(), 3.into())
            });
            assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
            let bodies = result.into_unverified().unwrap();
            verify(bodies.model(), false, &Matrix4::identity());
            verify(&replay(bodies.model()), false, &Matrix4::identity());
            let cap = bodies
                .model()
                .faces()
                .find(|f| {
                    f.surface().kind() == SurfaceKind::Plane
                        && f.outer()
                            .unwrap()
                            .edge_uses()
                            .any(|u| u.edge().curve().kind() != Curve3Kind::Line)
                })
                .unwrap();
            assert_eq!(
                cap.area(),
                Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
            );
            // Splitting changes topology, not the profile or its metric availability.
            let edge = bodies
                .model()
                .edges()
                .find(|e| e.curve().kind() != Curve3Kind::Line)
                .unwrap();
            let domain = edge.domain();
            let middle = ((domain.start().try_real().unwrap() + domain.end().try_real().unwrap())
                / Real::from(2))
            .unwrap();
            let split = edge.split(middle.into()).unwrap();
            verify(split.model(), false, &Matrix4::identity());
            verify(&replay(split.model()), false, &Matrix4::identity());
        }
    }
}

pub fn holes_components_and_affine_images_keep_optional_measurements() {
    let map = Matrix4::from_row_major([
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
    for nurbs in [false, true] {
        for reverse in [false, true] {
            let region = profile(nurbs, reverse, true);
            let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
            let direct =
                builder::extrude_path_regions(&path_regions(&region, reverse), 0.into(), 3.into())
                    .unwrap();
            for model in [
                bodies.model().clone(),
                replay(bodies.model()),
                direct.model().clone(),
                replay(direct.model()),
            ] {
                verify(&model, true, &Matrix4::identity());
                let image = model.transformed(&map).unwrap();
                verify(&image, true, &map);
                verify(&replay(&image), true, &map);
            }
        }
    }
}

pub fn measurement_absence_survives_empty_identities_and_repeated_slab_cuts() {
    for nurbs in [false, true] {
        for components in [false, true] {
            let region = profile(nurbs, false, components);
            let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
            let cold = replay(original.model());
            let mut selected = BodySet::empty();
            for solid in cold.solids() {
                selected = boolean::union(&selected, &BodySet::from_solid(solid)).unwrap();
            }
            for result in [
                boolean::union(&selected, &BodySet::empty()).unwrap(),
                boolean::difference(&selected, &BodySet::empty()).unwrap(),
            ] {
                verify(&replay(result.model()), components, &Matrix4::identity());
                let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
                let middle = boolean::intersection(&result, &cutter).unwrap();
                let ends = boolean::difference(&result, &cutter).unwrap();
                assert_eq!(ends.len(), if components { 4 } else { 2 });
                for part in [&middle, &ends] {
                    for solid in replay(part.model()).solids() {
                        assert_eq!(
                            solid.volume(),
                            Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
                        );
                    }
                }
                let restored = boolean::union(&ends, &middle).unwrap();
                verify(&replay(restored.model()), components, &Matrix4::identity());
                assert!(
                    boolean::difference(&restored, &selected)
                        .unwrap()
                        .is_empty()
                );
                assert!(
                    boolean::difference(&selected, &restored)
                        .unwrap()
                        .is_empty()
                );
            }
            for empty in [
                boolean::intersection(&selected, &BodySet::empty()).unwrap(),
                boolean::difference(&selected, &selected).unwrap(),
                boolean::difference(&BodySet::empty(), &selected).unwrap(),
            ] {
                assert!(empty.is_empty());
                assert_eq!(replay(empty.model()).counts().solids, 0);
            }
        }
    }
}

pub fn inward_and_damaged_archives_remain_invalid_without_volume() {
    let bodies =
        builder::extrude_curve_region(&profile(false, false, false), 0.into(), 3.into()).unwrap();
    let original: serde_json::Value =
        serde_json::from_str(&bodies.model().to_json().unwrap()).unwrap();
    let mut inward = original.clone();
    let flip = |value: &mut serde_json::Value| {
        *value = if value.as_str().unwrap() == "Forward" {
            "Reversed".into()
        } else {
            "Forward".into()
        };
    };
    for face in inward["faces"].as_array_mut().unwrap() {
        flip(&mut face["orientation"]);
    }
    for wire in inward["wires"].as_array_mut().unwrap() {
        wire["edge_uses"].as_array_mut().unwrap().reverse();
    }
    for curve in inward["pcurves"].as_array_mut().unwrap() {
        if let Some(line) = curve.get_mut("Line") {
            let start = line["start"].clone();
            line["start"] = line["end"].clone();
            line["end"] = start;
        } else {
            curve["RationalBezier"]["homogeneous_controls"]
                .as_array_mut()
                .unwrap()
                .reverse();
        }
    }
    for usage in inward["edge_uses"].as_array_mut().unwrap() {
        flip(&mut usage["direction"]);
        let affine = &mut usage["parameter_correspondence"]["Affine"];
        let scale: Real = serde_json::from_value(affine["scale"].clone()).unwrap();
        let offset: Real = serde_json::from_value(affine["offset"].clone()).unwrap();
        affine["scale"] = serde_json::to_value(-scale.clone()).unwrap();
        affine["offset"] = serde_json::to_value(offset + scale).unwrap();
    }
    let failure = RawModel::from_json(&inward.to_string(), &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap_err();
    assert!(
        matches!(
            failure,
            PersistenceError::Build(BuildError::InwardSolidShell(_))
        ),
        "{failure:?}"
    );
    let mut wrong_face = original.clone();
    flip(&mut wrong_face["faces"][0]["orientation"]);
    assert!(
        RawModel::from_json(&wrong_face.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .is_err()
    );
    let mut wrong_curve = original;
    let point = serde_json::to_value(Real::from(10)).unwrap();
    wrong_curve["curves"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|curve| curve.get("RationalBezier").is_some())
        .unwrap()["RationalBezier"]["control_points"][2][2] = point;
    assert!(
        RawModel::from_json(&wrong_curve.to_string(), &ArchiveLimits::default())
            .unwrap()
            .validate()
            .is_err()
    );
    verify(&replay(bodies.model()), false, &Matrix4::identity());
}
