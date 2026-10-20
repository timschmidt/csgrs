#[path = "support/closed_edge_sets.rs"]
mod body_sets;
#[path = "support/closed_cylinder.rs"]
mod fixture;
use hyperbrep::*;

#[test]
fn closed_edges_continue_through_disconnected_selections_and_empty_results() {
    body_sets::compose();
}

#[test]
fn closed_circle_edges_keep_single_vertex_identity_and_replay() {
    for (angle, turns, reversed) in [
        (Real::zero(), 0, false),
        (Real::zero(), -2, true),
        ((Real::pi() / Real::from(2)).unwrap(), 3, false),
        (-Real::pi(), -1, true),
    ] {
        let model = fixture::build(angle, turns, reversed).unwrap();
        assert_eq!(
            (
                model.counts().vertices,
                model.counts().edges,
                model.counts().edge_uses,
                model.counts().faces
            ),
            (2, 3, 6, 3)
        );
        for model in [&model, &fixture::replay(&model)] {
            for vertex in model.vertices() {
                assert_eq!(vertex.edges().count(), 2);
            }
            for edge in model
                .edges()
                .filter(|e| e.curve().kind() == Curve3Kind::CircleArc)
            {
                assert_eq!(edge.start(), edge.end());
                assert_eq!(edge.uses().count(), 2);
            }
            fixture::check(model);
            fixture::check(model);
        }
    }
}

#[test]
fn closed_edges_split_all_uses_and_preserve_the_seam_after_reflection() {
    for reversed in [false, true] {
        let model = fixture::build(Real::zero(), 0, reversed).unwrap();
        let edge = model
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
            .unwrap();
        let split = edge.split(Real::pi().into()).unwrap().into_model();
        assert_eq!(
            (
                split.counts().vertices,
                split.counts().edges,
                split.counts().edge_uses
            ),
            (3, 4, 8)
        );
        fixture::check(&fixture::replay(&split));
        let seam = split
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::Line)
            .unwrap();
        let split = seam
            .split((Real::one() / Real::from(2)).unwrap().into())
            .unwrap()
            .into_model();
        assert_eq!(
            (
                split.counts().vertices,
                split.counts().edges,
                split.counts().edge_uses
            ),
            (4, 5, 10)
        );
        let reflected = split
            .transformed(&Matrix4::affine_nonuniform_scale([
                -Real::one(),
                Real::one(),
                Real::one(),
            ]))
            .unwrap();
        assert_eq!(reflected.counts(), split.counts());
        fixture::check(&fixture::replay(&reflected));
        assert_eq!((model.counts().vertices, model.counts().edges), (2, 3));
    }
}

#[test]
fn closed_edges_compose_with_existing_cylinder_booleans() {
    let model = fixture::build(Real::zero(), 0, false).unwrap();
    let canonical = builder::cylinder(Real::from(2), Real::from(4)).unwrap();
    let a = BodySet::from_solid(model.solids().next().unwrap());
    let b = canonical;
    fixture::check(&fixture::replay(boolean::union(&a, &b).unwrap().model()));
    fixture::check(&fixture::replay(
        boolean::intersection(&a, &b).unwrap().model(),
    ));
    assert_eq!(boolean::difference(&a, &b).unwrap().bodies().count(), 0);
    assert_eq!(boolean::difference(&b, &a).unwrap().bodies().count(), 0);
}

#[test]
fn closed_edge_admission_requires_a_full_regular_period_and_actual_endpoint() {
    let mut b = ModelBuilder::new();
    let vertex = b
        .vertex(Point3::new(2.into(), 0.into(), 0.into()).into())
        .unwrap();
    let wrong = b
        .vertex(Point3::new(3.into(), 0.into(), 0.into()).into())
        .unwrap();
    let circle = b
        .curve(
            Curve3::circle_arc(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Real::from(2),
                Real::zero(),
                Real::tau(),
            )
            .unwrap(),
        )
        .unwrap();
    let half = ParameterDomain::new(Real::zero(), Real::pi()).unwrap();
    assert!(matches!(
        b.edge(&vertex, &vertex, &circle, half.into()),
        Err(BuildError::ClosedEdgePeriodMismatch)
    ));
    let full = ParameterDomain::new(Real::zero(), Real::tau()).unwrap();
    let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
    for end in [Real::tau() - &tiny, Real::tau() + &tiny] {
        assert!(matches!(
            b.edge(
                &vertex,
                &vertex,
                &circle,
                ParameterDomain::new(Real::zero(), end).unwrap().into()
            ),
            Err(BuildError::ClosedEdgePeriodMismatch)
        ));
    }
    let nearly_incident = b
        .vertex(Point3::new(Real::from(2) + tiny, 0.into(), 0.into()).into())
        .unwrap();
    assert!(matches!(
        b.edge(
            &nearly_incident,
            &nearly_incident,
            &circle,
            full.clone().into()
        ),
        Err(BuildError::EdgeEndpointMismatch { .. })
    ));
    assert!(matches!(
        b.edge(&wrong, &wrong, &circle, full.clone().into()),
        Err(BuildError::EdgeEndpointMismatch { .. })
    ));
    let point = Point3::new(2.into(), 0.into(), 0.into());
    let constant = b
        .curve(
            Curve3::rational_bezier(
                vec![point.clone(), point.clone(), point],
                vec![Real::one(); 3],
            )
            .unwrap(),
        )
        .unwrap();
    assert!(matches!(
        b.edge(&vertex, &vertex, &constant, ParameterDomain::unit().into()),
        Err(BuildError::UnsupportedClosedEdge(
            Curve3Kind::RationalBezier
        ))
    ));
    b.edge(&vertex, &vertex, &circle, full.into()).unwrap();
    let model = b.finish().unwrap();
    assert_eq!(model.counts().edges, 1);
    assert_eq!(model.vertices().next().unwrap().edges().count(), 1);
    assert_eq!(fixture::replay(&model).counts().edges, 1);
}

#[test]
fn a_regular_closed_ellipse_is_retained_without_inventing_split_vertices() {
    let mut b = ModelBuilder::new();
    let vertex = b
        .vertex(Point3::new(3.into(), 0.into(), 0.into()).into())
        .unwrap();
    let curve = b
        .curve(
            Curve3::ellipse_arc(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                3.into(),
                2.into(),
                Real::zero(),
                Real::tau(),
            )
            .unwrap(),
        )
        .unwrap();
    b.edge(
        &vertex,
        &vertex,
        &curve,
        ParameterDomain::new(Real::zero(), Real::tau())
            .unwrap()
            .into(),
    )
    .unwrap();
    let model = b.finish().unwrap();
    let loaded = fixture::replay(&model);
    let edge = loaded.edges().next().unwrap();
    assert_eq!(edge.start(), edge.end());
    assert_eq!(edge.start().edges().count(), 1);
    let split = edge.split(Real::pi().into()).unwrap().into_model();
    assert_eq!((split.counts().vertices, split.counts().edges), (2, 2));
    assert_eq!(fixture::replay(&split).counts(), split.counts());
}

#[test]
fn periodic_parameter_maps_keep_both_seam_endpoints_and_interior_locations() {
    for reversed in [false, true] {
        let model = fixture::build(Real::zero(), 2, reversed).unwrap();
        for model in [&model, &fixture::replay(&model)] {
            for edge in model
                .edges()
                .filter(|e| e.curve().kind() == Curve3Kind::CircleArc)
            {
                for use_ in edge.uses() {
                    let mut endpoints = Vec::new();
                    for i in 0..=4 {
                        let parameter: Parameter = (Real::tau() * Real::from(i) / Real::from(4))
                            .unwrap()
                            .into();
                        let uv_parameter = use_.pcurve_parameter_at(&parameter).unwrap();
                        assert_eq!(
                            use_.edge_parameter_at(&uv_parameter)
                                .unwrap()
                                .compare(&parameter)
                                .unwrap(),
                            std::cmp::Ordering::Equal
                        );
                        if i == 0 || i == 4 {
                            endpoints.push(uv_parameter);
                        }
                    }
                    assert_ne!(
                        endpoints[0].compare(&endpoints[1]).unwrap(),
                        std::cmp::Ordering::Equal
                    );
                }
            }
        }
    }
}

#[test]
fn untrusted_closed_edge_domains_points_and_pcurves_are_rechecked() {
    let model = fixture::build(Real::zero(), 0, false).unwrap();
    let raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    for fault in 0..6 {
        let mut bad = raw.clone();
        match fault {
            0 => bad["edges"][0]["domain"][1] = serde_json::json!({"Scalar": Real::pi()}),
            1 => bad["edges"][0]["end"] = serde_json::json!(1),
            2 => bad["vertices"][0]["Coordinates"][0] = serde_json::json!(Real::from(3)),
            3 => bad["pcurves"][0]["CircularArc"]["clockwise"] = serde_json::json!(false),
            4 => {
                bad["pcurves"][0]["CircularArc"]["end"] =
                    serde_json::json!([Real::zero(), Real::from(2)])
            }
            5 => {
                bad["edge_uses"][2]["parameter_correspondence"]["Affine"]["scale"] =
                    serde_json::json!(Real::pi())
            }
            _ => unreachable!(),
        }
        assert!(
            RawModel::from_json(&bad.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err(),
            "accepted fault {fault}"
        );
    }
}
