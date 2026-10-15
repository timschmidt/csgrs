use std::cmp::Ordering;

use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Direction, GeometricPoint3, GeometryError, Matrix4,
    Model, ModelBuilder, Parameter, ParameterCorrespondence, ParameterRange, Point3, Real,
    SolidPointLocation, TopologyEditError, Vector3, builder,
};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root() -> Parameter {
    // The selected unit-interval root of 32t^5 - 2t - 1 has no stored Real.
    let curve = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("expected a finite selected root");
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn check_correspondences(model: &Model) {
    for edge in model.edges() {
        for edge_use in edge.uses() {
            let pcurve = edge_use.pcurve();
            let endpoints = match edge_use.direction() {
                Direction::Forward => [edge.domain().start(), edge.domain().end()],
                Direction::Reversed => [edge.domain().end(), edge.domain().start()],
            };
            for ((local, spatial), expected_point) in
                [pcurve.domain_start().unwrap(), pcurve.domain_end().unwrap()]
                    .into_iter()
                    .zip(endpoints)
                    .zip([pcurve.curve().start(), pcurve.curve().end()])
            {
                assert_eq!(
                    edge_use
                        .edge_parameter_at(&local)
                        .unwrap()
                        .compare(spatial)
                        .unwrap(),
                    Ordering::Equal,
                    "edge {} use {}: local={local:?}, spatial={spatial:?}, map={:?}, native={}",
                    edge.index(),
                    edge_use.index(),
                    edge_use.parameter_correspondence(),
                    pcurve.curve().geometry().is_some()
                );
                assert_eq!(
                    edge_use
                        .pcurve_parameter_at(spatial)
                        .unwrap()
                        .compare(&local)
                        .unwrap(),
                    Ordering::Equal
                );
                assert!(
                    pcurve
                        .point_at(&local)
                        .unwrap()
                        .coincides_with(&expected_point)
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn selected_split_preserves_both_incident_uses_and_shared_spatial_point() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let parameter = root();
    let edge = source.model().edges().next().unwrap();
    let expected = edge.curve().point_at(&parameter).unwrap();
    let split = edge.split(parameter.clone()).unwrap();
    let details = split.details();
    assert_eq!(
        split.model().counts().vertices,
        source.model().counts().vertices + 1
    );
    assert_eq!(
        split.model().counts().edges,
        source.model().counts().edges + 1
    );
    assert_eq!(details.edge_uses().len(), 2);
    assert_eq!(details.vertex().edges().len(), 2);
    assert!(details.vertex().point().try_coordinates().is_none());
    assert!(details.vertex().point().coincides_with(&expected).unwrap());
    assert_eq!(
        details.first().domain().end().compare(&parameter).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        details
            .second()
            .domain()
            .start()
            .compare(&parameter)
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        details.first().curve().index(),
        details.second().curve().index()
    );
    assert!(
        source
            .model()
            .vertices()
            .all(|v| v.point().try_coordinates().is_some())
    );
    check_correspondences(split.model());
    split.model().bounds().unwrap().unwrap();
    let solid = split.model().solids().next().unwrap();
    assert_eq!(
        solid.classify_point(&point(1, 1, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        solid.classify_point(&point(3, 1, 1)).unwrap(),
        SolidPointLocation::Outside
    );
    assert_eq!(solid.volume().unwrap(), Real::from(24));
}

#[test]
fn selected_split_composes_and_rejects_repeated_endpoint() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let parameter = root();
    let first = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(parameter.clone())
        .unwrap();
    assert!(matches!(
        first.details().first().split(parameter.clone()),
        Err(TopologyEditError::Geometry(
            GeometryError::InvalidParameterDomain
        ))
    ));
    let second = first
        .details()
        .first()
        .split(rational(1, 2).into())
        .unwrap();
    check_correspondences(second.model());
    assert!(
        second
            .model()
            .vertices()
            .any(|v| v.point().try_coordinates().is_none())
    );
    let third = first
        .details()
        .second()
        .split(rational(3, 4).into())
        .unwrap();
    check_correspondences(third.model());
}

#[test]
fn selected_vertices_and_pcurves_survive_affine_transforms_and_reflection() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    for entries in [
        [1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1],
        [-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1],
    ] {
        let transform = Matrix4::from_row_major(entries.map(Real::from));
        let expected = split
            .details()
            .vertex()
            .point()
            .transformed(&transform)
            .unwrap();
        let moved = split.model().transformed(&transform).unwrap();
        let vertex = moved
            .vertices()
            .find(|v| v.point().try_coordinates().is_none())
            .unwrap();
        assert!(vertex.point().coincides_with(&expected).unwrap());
        check_correspondences(&moved);
        let restored = moved.transformed(&transform.inverse().unwrap()).unwrap();
        let vertex = restored
            .vertices()
            .find(|v| v.point().try_coordinates().is_none())
            .unwrap();
        assert!(
            vertex
                .point()
                .coincides_with(split.details().vertex().point())
                .unwrap()
        );
        check_correspondences(&restored);
        let continued = moved
            .edges()
            .next()
            .unwrap()
            .split(rational(1, 2).into())
            .unwrap();
        check_correspondences(continued.model());
    }
}

#[test]
fn selected_spline_trims_keep_their_authored_parameter_charts() {
    for nurbs in [false, true] {
        let controls = vec![point(0, 0, 0), point(1, -1, 0), point(2, 0, 0)];
        let weights = vec![Real::one(), Real::from(2), Real::one()];
        let (profile, parameter, kind) = if nurbs {
            let selector = Curve3::nurbs(
                5,
                [-5, -7, -9, -11, -13, 145]
                    .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
                    .to_vec(),
                vec![Real::one(); 6],
                [vec![Real::from(2); 6], vec![Real::from(5); 6]].concat(),
            )
            .unwrap();
            let CurveParameterLocation::Parameters(parameters) =
                selector.parameters_of(&Point3::origin()).unwrap()
            else {
                panic!("expected finite selected knot parameter");
            };
            (
                Curve3::nurbs(
                    2,
                    controls,
                    weights,
                    [2, 2, 2, 5, 5, 5].map(Real::from).to_vec(),
                )
                .unwrap(),
                parameters[0].clone(),
                Curve3Kind::Nurbs,
            )
        } else {
            (
                Curve3::rational_bezier(controls, weights).unwrap(),
                root(),
                Curve3Kind::RationalBezier,
            )
        };
        let expected = profile.point_at(&parameter).unwrap();
        let source =
            builder::extrusion_patch(profile, Vector3::z(), Real::zero(), Real::one()).unwrap();
        let edge = source
            .model()
            .edges()
            .find(|edge| edge.curve().kind() == kind)
            .unwrap();
        let split = edge.split(parameter.clone()).unwrap();
        assert!(
            split
                .details()
                .vertex()
                .point()
                .coincides_with(&expected)
                .unwrap()
        );
        assert!(split.details().first().domain().end().try_real().is_none());
        check_correspondences(split.model());
        let continued = split
            .details()
            .first()
            .split(
                if nurbs {
                    rational(7, 2)
                } else {
                    rational(1, 2)
                }
                .into(),
            )
            .unwrap();
        check_correspondences(continued.model());
        let loaded = reload(continued.model());
        check_correspondences(&loaded);
        assert!(
            loaded
                .vertices()
                .any(|vertex| vertex.point().try_coordinates().is_none())
        );
    }
}

#[test]
fn selected_cylinder_axial_split_preserves_solid_queries() {
    let source = builder::cylinder(Real::from(2), Real::from(3)).unwrap();
    let edge = source
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::Line)
        .unwrap();
    let split = edge.split(root()).unwrap();
    check_correspondences(split.model());
    let solid = split.model().solids().next().unwrap();
    assert_eq!(
        solid.volume().unwrap(),
        source.bodies().next().unwrap().volume().unwrap()
    );
    assert_eq!(
        solid.classify_point(&point(0, 0, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        solid.classify_point(&point(3, 0, 1)).unwrap(),
        SolidPointLocation::Outside
    );
}

#[test]
fn arbitrary_edits_cannot_reuse_subdivision_certificates() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    let mut edit = split.model().edit();
    edit.replace_vertex(split.details().vertex(), point(9, 9, 9).into())
        .unwrap();
    assert!(edit.commit().is_err());

    let edge_use = source.model().edge_uses().next().unwrap();
    let mut edit = source.model().edit();
    edit.replace_edge_use(
        edge_use,
        edge_use.edge(),
        edge_use.direction(),
        edge_use.pcurve(),
        ParameterCorrespondence::affine(Real::from(2), Real::zero()).unwrap(),
    )
    .unwrap();
    assert!(edit.commit().is_err());
}

#[test]
fn selected_solid_archive_replays_and_angular_cuts_remain_explicitly_unsupported() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    let loaded = reload(split.model());
    assert_eq!(
        loaded.solids().next().unwrap().volume().unwrap(),
        Real::from(24)
    );
    let replayed = split.model().edit().commit().unwrap();
    check_correspondences(&replayed);
    assert!(
        replayed
            .vertices()
            .any(|vertex| vertex.point().try_coordinates().is_none())
    );

    let cylinder = builder::cylinder(Real::from(2), Real::from(3)).unwrap();
    let json = cylinder.model().to_json().unwrap();
    let circular = cylinder
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    assert!(matches!(
        circular.split(root()),
        Err(TopologyEditError::Geometry(
            GeometryError::UnsupportedSelectedEvaluation
        ))
    ));
    assert_eq!(cylinder.model().to_json().unwrap(), json);
}

#[test]
fn selected_rational_pcurve_reversal_keeps_source_parameters_and_endpoint_images() {
    let curve = hypercurve::RationalBezier2::try_new(
        vec![
            hypercurve::Point2::new(Real::zero(), Real::zero()),
            hypercurve::Point2::new(Real::one(), -Real::one()),
            hypercurve::Point2::new(Real::from(2), Real::zero()),
        ],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    let pcurve = hyperbrep::Pcurve::new(curve.into());
    let (first, _) = pcurve.split_at(&root()).unwrap();
    let reversed = first.reversed().unwrap();
    assert_eq!(
        first
            .domain_start()
            .unwrap()
            .compare(&reversed.domain_end().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        first
            .domain_end()
            .unwrap()
            .compare(&reversed.domain_start().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        reversed
            .point_at(&reversed.domain_start().unwrap())
            .unwrap()
            .coincides_with(&first.curve().end())
            .unwrap()
    );
    assert!(
        reversed
            .point_at(&reversed.domain_end().unwrap())
            .unwrap()
            .coincides_with(&first.curve().start())
            .unwrap()
    );
    assert!(reversed.domain_start().unwrap().try_real().is_none());
}

#[test]
fn planar_selected_rational_pcurves_survive_model_reflection() {
    use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2, RationalBezier2};

    let uv = |x, y| Point2::new(Real::from(x), Real::from(y));
    let curved = RationalBezier2::try_new(
        vec![uv(0, 0), uv(1, -1), uv(2, 0)],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    let boundary = CurvePath2::try_new(vec![
        curved.into(),
        Curve2::from(LineSeg2::try_new(uv(2, 0), uv(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(uv(2, 2), uv(0, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(uv(0, 2), uv(0, 0)).unwrap()),
    ])
    .unwrap();
    let source =
        builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let reflection =
        Matrix4::from_row_major([-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1].map(Real::from));
    check_correspondences(&source.model().transformed(&reflection).unwrap());
    let edge = source
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::RationalBezier)
        .unwrap();
    let split = edge.split(root()).unwrap();
    let reflected = split.model().transformed(&reflection).unwrap();
    check_correspondences(&reflected);
    check_correspondences(&reload(&reflected));
    let expected = split
        .details()
        .vertex()
        .point()
        .transformed(&reflection)
        .unwrap();
    let actual = reflected
        .vertices()
        .find(|vertex| vertex.point().try_coordinates().is_none())
        .unwrap();
    assert!(actual.point().coincides_with(&expected).unwrap());
    let continued = reflected
        .edges()
        .next()
        .unwrap()
        .split(rational(1, 2).into())
        .unwrap();
    check_correspondences(continued.model());
}

#[test]
fn selected_trim_range_orders_exactly_and_builder_checks_endpoint_incidence() {
    let parameter = root();
    let range = ParameterRange::new(Real::zero().into(), parameter.clone()).unwrap();
    assert!(range.contains(&rational(1, 2).into()).unwrap());
    assert!(!range.contains(&rational(3, 4).into()).unwrap());
    assert!(range.try_real_endpoints().is_none());
    assert!(matches!(
        ParameterRange::new(parameter.clone(), parameter.clone()),
        Err(GeometryError::InvalidParameterDomain)
    ));
    assert!(matches!(
        ParameterRange::new(parameter.clone(), Real::zero().into()),
        Err(GeometryError::InvalidParameterDomain)
    ));

    let curve = Curve3::line(point(0, 0, 0), point(1, 0, 0)).unwrap();
    for correct in [true, false] {
        let mut builder = ModelBuilder::new();
        let start = builder.vertex(point(0, 0, 0).into()).unwrap();
        let end = builder
            .vertex(if correct {
                curve.point_at(&parameter).unwrap()
            } else {
                GeometricPoint3::from_coordinates(point(1, 0, 0))
            })
            .unwrap();
        let carrier = builder.curve(curve.clone()).unwrap();
        assert_eq!(
            builder.edge(&start, &end, &carrier, range.clone()).is_ok(),
            correct
        );
    }
}

#[test]
fn selected_planar_archive_reconstructs_evidence_and_splits_again() {
    use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2};
    let uv = |x, y| Point2::new(Real::from(x), Real::from(y));
    let points = [uv(0, 0), uv(2, 0), uv(2, 2), uv(0, 2)];
    let boundary = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let source =
        builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    let json = split.model().to_json().unwrap();
    let loaded = hyperbrep::RawModel::from_json(&json)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.counts(), split.model().counts());
    assert_eq!(
        loaded.faces().next().unwrap().area().unwrap(),
        Real::from(4)
    );
    assert_eq!(loaded.to_json().unwrap(), json);
    let selected = loaded
        .vertices()
        .find(|vertex| vertex.point().try_coordinates().is_none())
        .unwrap();
    assert!(
        selected
            .point()
            .coincides_with(split.details().vertex().point())
            .unwrap()
    );
    check_correspondences(&loaded);
    let repeated = loaded
        .edges()
        .next()
        .unwrap()
        .split(rational(1, 2).into())
        .unwrap();
    let repeated = hyperbrep::RawModel::from_json(&repeated.model().to_json().unwrap())
        .unwrap()
        .validate()
        .unwrap();
    check_correspondences(&repeated);
    let detached = Curve3::line(point(0, 0, 0), point(2, 0, 0))
        .unwrap()
        .point_at(&root())
        .unwrap();
    let selected = loaded
        .vertices()
        .find(|vertex| vertex.point().try_coordinates().is_none())
        .unwrap();
    let mut edit = loaded.edit();
    edit.replace_vertex(selected, detached.clone()).unwrap();
    let edited = edit.commit().unwrap();
    let raw: serde_json::Value = serde_json::from_str(&edited.to_json().unwrap()).unwrap();
    assert_eq!(
        raw["selection"]["point_sources"].as_array().unwrap().len(),
        1
    );
    let detached_replay = reload(&edited);
    assert!(
        detached_replay
            .vertices()
            .find(|vertex| vertex.point().try_coordinates().is_none())
            .unwrap()
            .point()
            .coincides_with(&detached)
            .unwrap()
    );
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let loaded = hyperbrep::RawModel::from_json(&json)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json);
    loaded
}

#[test]
fn selected_archive_rejects_forged_roots_sources_charts_and_incidence() {
    use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2};
    use serde_json::{Value, json};
    let uv = |x, y| Point2::new(Real::from(x), Real::from(y));
    let points = [uv(0, 0), uv(2, 0), uv(2, 2), uv(0, 2)];
    let boundary = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    let source =
        builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    let raw: Value = serde_json::from_str(&split.model().to_json().unwrap()).unwrap();
    let selected_vertex = raw["vertices"]
        .as_array()
        .unwrap()
        .iter()
        .position(|v| v.get("CurveImage").is_some())
        .unwrap();
    let selected_pcurve = raw["pcurves"]
        .as_array()
        .unwrap()
        .iter()
        .position(|v| v.get("SourceRange").is_some())
        .unwrap();
    let roots = raw["selection"]["parameters"].as_array().unwrap().len();
    let scalar = |n| serde_json::to_value(Real::from(n)).unwrap();
    let mut mutants = Vec::new();
    let mut add = |name, mutate: &dyn Fn(&mut Value)| {
        let mut value = raw.clone();
        mutate(&mut value);
        mutants.push((name, value));
    };
    add("no root in bracket", &|v| {
        v["selection"]["parameters"][0]["Root"]["interval"] =
            json!([scalar(0), serde_json::to_value(rational(1, 4)).unwrap()])
    });
    add("zero polynomial", &|v| {
        v["selection"]["parameters"][0]["Root"]["polynomial"] = json!([scalar(0)])
    });
    add("missing root", &|v| {
        v["vertices"][selected_vertex]["CurveImage"]["parameter"] = json!({"Selected": roots})
    });
    add("missing source", &|v| {
        v["vertices"][selected_vertex]["CurveImage"]["source"] = json!({"Model": 9999})
    });
    add("wrong point parameter", &|v| {
        v["vertices"][selected_vertex]["CurveImage"]["parameter"] = json!({"Scalar": scalar(0)})
    });
    add("pcurve wrong traversal", &|v| {
        v["pcurves"][selected_pcurve]["SourceRange"]["reversed"] = json!(true)
    });
    add("pcurve outside source", &|v| {
        v["pcurves"][selected_pcurve]["SourceRange"]["range"][0] = json!({"Scalar": scalar(-1)})
    });
    add("missing pcurve source", &|v| {
        v["pcurves"][selected_pcurve]["SourceRange"]["source"] = json!(9999)
    });
    add("wrong map and forged proofs", &|v| {
        v["edge_uses"][0]["parameter_correspondence"]["Affine"]["offset"] = scalar(1);
        v["edge_uses"][0]["image_certificate"] = json!(true);
        v["wires"][0]["simplicity_certificate"] = json!(true);
    });
    add("cyclic chart", &|v| {
        v["selection"]["parameters"].as_array_mut().unwrap().push(json!({"Image": {"root": roots, "numerator": [scalar(0), scalar(1)], "denominator": [scalar(1), scalar(0)]}}))
    });
    add("pole at selected root", &|v| {
        v["selection"]["parameters"].as_array_mut().unwrap().push(json!({"Image": {"root": 0, "numerator": [scalar(0), scalar(1)], "denominator": [scalar(0), scalar(0)]}}))
    });

    add("multiple roots in bracket", &|v| {
        v["selection"]["parameters"][0]["Root"]["polynomial"] = json!([
            serde_json::to_value(rational(3, 16)).unwrap(),
            scalar(-1),
            scalar(1)
        ]);
        v["selection"]["parameters"][0]["Root"]["interval"] = json!([scalar(0), scalar(1)]);
    });
    add("wrong source image", &|v| {
        v["selection"]["pcurve_sources"][0]["Line"]["end"][1] = scalar(1)
    });
    add("nested pcurve sources", &|v| {
        v["selection"]["pcurve_sources"][0] = json!({"SourceRange": {"source": 0, "range": [{"Scalar": scalar(0)}, {"Scalar": scalar(1)}], "reversed": false}})
    });
    for (name, value) in mutants {
        let json = serde_json::to_string(&value).unwrap();
        assert!(
            hyperbrep::RawModel::from_json(&json)
                .unwrap()
                .validate()
                .is_err(),
            "accepted {name}"
        );
    }
    let json = split.model().to_json().unwrap();
    assert!(hyperbrep::RawModel::from_json(&(json + " false")).is_err());
}
