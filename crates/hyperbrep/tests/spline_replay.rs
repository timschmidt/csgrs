//! Trimmed spline image agreement and repeated algebraic subdivision.

use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Matrix4, Model, Parameter, Point3, RawModel, Real,
    Vector3, builder,
};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2, RationalBezier2, RationalQuadraticBezier2};
use std::cmp::Ordering;

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn uv(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn selected(start: Real, end: Real) -> Parameter {
    let source = Curve3::nurbs(
        5,
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 6],
        [vec![start; 6], vec![end; 6]].concat(),
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        source.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("finite selected root");
    };
    assert_eq!(parameters.len(), 1);
    let root = parameters.remove(0);
    assert!(root.try_real().is_none());
    root
}

fn sheet(family: usize) -> Model {
    let controls = [uv(0, 0), uv(1, -1), uv(2, 0)];
    let weights = [Real::one(), Real::from(2), Real::one()];
    let curve: Curve2 = match family {
        0 => RationalBezier2::try_new(controls.to_vec(), weights.to_vec())
            .unwrap()
            .into(),
        1 => RationalQuadraticBezier2::try_new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            weights[0].clone(),
            weights[1].clone(),
            weights[2].clone(),
        )
        .unwrap()
        .into(),
        2 => Curve2::try_nurbs(
            2,
            controls.to_vec(),
            weights.to_vec(),
            [2, 2, 2, 5, 5, 5].map(Real::from).to_vec(),
        )
        .unwrap(),
        3 => Curve2::try_nurbs(
            2,
            vec![
                uv(0, 0),
                Point2::new(rational(1, 2), -Real::one()),
                uv(1, 0),
                Point2::new(rational(3, 2), -Real::one()),
                uv(2, 0),
            ],
            [1, 2, 1, 2, 1].map(Real::from).to_vec(),
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                rational(7, 2),
                rational(7, 2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        )
        .unwrap(),
        _ => unreachable!(),
    };
    let boundary = CurvePath2::try_new(vec![
        curve,
        LineSeg2::try_new(uv(2, 0), uv(2, 2)).unwrap().into(),
        LineSeg2::try_new(uv(2, 2), uv(0, 2)).unwrap().into(),
        LineSeg2::try_new(uv(0, 2), uv(0, 0)).unwrap().into(),
    ])
    .unwrap();
    builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y())
        .unwrap()
        .into_model()
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json);
    for edge in loaded.edges() {
        for edge_use in edge.uses() {
            for parameter in [edge.domain().start(), edge.domain().end()] {
                let local = edge_use.pcurve_parameter_at(parameter).unwrap();
                assert_eq!(
                    edge_use
                        .edge_parameter_at(&local)
                        .unwrap()
                        .compare(parameter)
                        .unwrap(),
                    Ordering::Equal
                );
            }
        }
    }
    loaded
}

#[test]
fn scalar_then_selected_spline_subdivisions_replay_the_correct_source_interval() {
    for family in 0..4 {
        let source = sheet(family);
        let edge = source
            .edges()
            .find(|e| e.curve().kind() != Curve3Kind::Line)
            .unwrap();
        let index = edge.index();
        let native_cut = if family >= 2 {
            rational(17, 4)
        } else {
            rational(3, 4)
        };
        let native = edge.split(native_cut.into()).unwrap();
        let loaded = reload(native.model());
        let (start, end, midpoint) = if family >= 2 {
            (Real::from(2), Real::from(5), rational(7, 2))
        } else {
            (Real::zero(), Real::one(), rational(1, 2))
        };
        let first = loaded
            .edges()
            .find(|e| e.index() == index)
            .unwrap()
            .split(selected(start.clone(), end))
            .unwrap();
        let loaded = reload(first.model());
        let second = loaded
            .edges()
            .find(|e| e.index() == index)
            .unwrap()
            .split(selected(start, midpoint))
            .unwrap();
        let loaded = reload(second.model());
        assert_eq!(
            loaded
                .vertices()
                .filter(|v| v.point().try_coordinates().is_none())
                .count(),
            2
        );
        let reflection = Matrix4::from_row_major(
            [-1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from),
        );
        let reflected = loaded.transformed(&reflection).unwrap();
        let reflected = reload(&reflected);
        assert_eq!(
            reload(&reflected).to_json().unwrap(),
            reflected.to_json().unwrap()
        );
    }
}

#[test]
fn matching_trim_endpoints_do_not_certify_a_changed_spline_interior() {
    for family in 0..4 {
        let source = sheet(family);
        let split = source
            .edges()
            .find(|e| e.curve().kind() != Curve3Kind::Line)
            .unwrap()
            .split(
                if family >= 2 {
                    rational(17, 4)
                } else {
                    rational(3, 4)
                }
                .into(),
            )
            .unwrap();
        let mut raw: serde_json::Value =
            serde_json::from_str(&split.model().to_json().unwrap()).unwrap();
        let pcurve = raw["pcurves"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|p| p.get("Line").is_none())
            .unwrap();
        let controls = if family == 1 {
            &mut pcurve["RationalQuadraticBezier"]["control_points"]
        } else {
            &mut pcurve[if family >= 2 {
                "Nurbs"
            } else {
                "RationalBezier"
            }]["homogeneous_controls"]
        };
        let value: Real = serde_json::from_value(controls[1][1].clone()).unwrap();
        controls[1][1] = serde_json::to_value(value - Real::one()).unwrap();
        assert!(
            RawModel::from_json(&raw.to_string(), &hyperbrep::ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err()
        );
    }
}

#[test]
fn affine_nurbs_pcurve_charts_replay_with_both_spatial_directions() {
    use serde_json::{Value, json};
    for reversed in [false, true] {
        let source = sheet(2);
        let mut raw: Value = serde_json::from_str(&source.to_json().unwrap()).unwrap();
        let curve_index = raw["curves"]
            .as_array()
            .unwrap()
            .iter()
            .position(|c| c.get("Nurbs").is_some())
            .unwrap();
        let edge_index = raw["edges"]
            .as_array()
            .unwrap()
            .iter()
            .position(|e| e["curve"] == json!(curve_index))
            .unwrap();
        let use_index = raw["edge_uses"]
            .as_array()
            .unwrap()
            .iter()
            .position(|u| u["edge"] == json!(edge_index))
            .unwrap();
        let pcurve_index = raw["edge_uses"][use_index]["pcurve"].as_u64().unwrap() as usize;
        raw["pcurves"][pcurve_index]["Nurbs"]["knots"] =
            serde_json::to_value([0, 0, 0, 1, 1, 1].map(Real::from)).unwrap();
        if reversed {
            raw["curves"][curve_index]["Nurbs"]["control_points"]
                .as_array_mut()
                .unwrap()
                .reverse();
            raw["curves"][curve_index]["Nurbs"]["weights"]
                .as_array_mut()
                .unwrap()
                .reverse();
            let start = raw["edges"][edge_index]["start"].clone();
            raw["edges"][edge_index]["start"] = raw["edges"][edge_index]["end"].clone();
            raw["edges"][edge_index]["end"] = start;
            raw["edge_uses"][use_index]["direction"] = json!("Reversed");
        }
        // The face-local spline uses [0,1]; the spatial spline uses [2,5].
        // The negative map reverses the spatial parameter, not the wire image.
        raw["edge_uses"][use_index]["parameter_correspondence"] = json!({"Affine": {
            "scale": serde_json::to_value(Real::from(if reversed { -3 } else { 3 })).unwrap(),
            "offset": serde_json::to_value(Real::from(if reversed { 5 } else { 2 })).unwrap(),
        }});
        let loaded = RawModel::from_json(&raw.to_string(), &hyperbrep::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let loaded = reload(&loaded);
        let split = loaded
            .edges()
            .find(|e| e.index() == edge_index)
            .unwrap()
            .split(rational(17, 4).into())
            .unwrap();
        let loaded = reload(split.model());
        let split = loaded
            .edges()
            .find(|e| e.index() == edge_index)
            .unwrap()
            .split(selected(Real::from(2), Real::from(5)))
            .unwrap();
        reload(split.model());
    }
}

#[test]
fn repeated_selected_solid_cuts_replay_without_reconstructing_root_coordinates() {
    let source = builder::cuboid(
        Point3::origin(),
        Point3::new(Real::from(2), Real::from(3), Real::from(4)),
    )
    .unwrap();
    let first = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(selected(Real::zero(), Real::one()))
        .unwrap();
    let mut model = reload(first.model());
    for denominator in [2, 4, 8, 16] {
        let split = model
            .edges()
            .next()
            .unwrap()
            .split(selected(Real::zero(), rational(1, denominator)))
            .unwrap();
        model = reload(split.model());
        let solid = model.solids().next().unwrap();
        assert_eq!(solid.volume().unwrap(), Real::from(24));
        assert_eq!(
            solid
                .classify_point(&Point3::new(Real::one(), Real::one(), Real::one()))
                .unwrap(),
            hyperbrep::SolidPointLocation::Inside
        );
        assert_eq!(
            solid
                .classify_point(&Point3::new(Real::from(3), Real::one(), Real::one()))
                .unwrap(),
            hyperbrep::SolidPointLocation::Outside
        );
    }
    assert_eq!(
        model
            .vertices()
            .filter(|v| v.point().try_coordinates().is_none())
            .count(),
        5
    );
    let raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
    let parameters = raw["selection"]["parameters"].as_array().unwrap();
    assert_eq!(
        parameters
            .iter()
            .filter(|p| p.get("Root").is_some())
            .count(),
        1
    );
    assert_eq!(
        raw["selection"]["pcurve_sources"].as_array().unwrap().len(),
        2
    );
}
