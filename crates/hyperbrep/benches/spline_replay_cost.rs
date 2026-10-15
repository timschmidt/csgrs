use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Model, Parameter, Point3, RawModel, Real, Vector3,
    builder,
};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2, RationalBezier2, RationalQuadraticBezier2};
use std::hint::black_box;
use std::time::Instant;
const ITERATIONS: usize = 100;

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

fn replay(json: &str, vertices: usize, selected: usize) -> usize {
    let loaded = RawModel::from_json(json).unwrap().validate().unwrap();
    assert_eq!(loaded.counts().vertices, vertices);
    assert_eq!(loaded.counts().edges, vertices);
    assert_eq!(
        loaded
            .vertices()
            .filter(|v| v.point().try_coordinates().is_none())
            .count(),
        selected
    );
    assert_eq!(loaded.to_json().unwrap(), json);
    vertices + selected
}

fn measure(name: &str, mut operation: impl FnMut() -> usize) {
    let start = Instant::now();
    let mut checksum = 0;
    for _ in 0..ITERATIONS {
        checksum += black_box(operation());
    }
    let elapsed = start.elapsed();
    println!(
        "{name} {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );
}

fn prepared(family: usize, selected_cuts: usize) -> Model {
    let source = sheet(family);
    let edge = source
        .edges()
        .find(|e| e.curve().kind() != Curve3Kind::Line)
        .unwrap();
    let index = edge.index();
    let mut model = edge
        .split(
            if family >= 2 {
                rational(17, 4)
            } else {
                rational(3, 4)
            }
            .into(),
        )
        .unwrap()
        .into_model();
    for denominator in [1, 2].into_iter().take(selected_cuts) {
        let (start, end) = if family >= 2 {
            (Real::from(2), Real::from(2) + rational(3, denominator))
        } else {
            (Real::zero(), rational(1, denominator))
        };
        let next = model
            .edges()
            .find(|e| e.index() == index)
            .unwrap()
            .split(selected(start, end))
            .unwrap()
            .into_model();
        model = next;
    }
    model
}

fn main() {
    // Authoring and root isolation are outside timing. Each replay independently
    // parses, certifies the complete mapped image, and re-encodes exact bytes.
    for (name, model, vertices, cuts) in [
        ("spline_replay/native_rational_full:", sheet(0), 4, 0),
        ("spline_replay/native_rational_trim:", prepared(0, 0), 5, 0),
        (
            "spline_replay/selected_rational_trim:",
            prepared(0, 2),
            7,
            2,
        ),
        (
            "spline_replay/selected_rational_quadratic_trim:",
            prepared(1, 2),
            7,
            2,
        ),
        (
            "spline_replay/selected_single_span_nurbs_trim:",
            prepared(2, 2),
            7,
            2,
        ),
        (
            "spline_replay/selected_multi_span_nurbs_trim:",
            prepared(3, 2),
            7,
            2,
        ),
    ] {
        let json = model.to_json().unwrap();
        let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
        let parameters = raw["selection"]["parameters"].as_array().unwrap();
        println!(
            "archive_metadata name={name} bytes={} vertices={vertices} selected={cuts} roots={} images={} pcurve_sources={}",
            json.len(),
            parameters
                .iter()
                .filter(|p| p.get("Root").is_some())
                .count(),
            parameters
                .iter()
                .filter(|p| p.get("Image").is_some())
                .count(),
            raw["selection"]["pcurve_sources"].as_array().unwrap().len()
        );
        measure(name, || replay(black_box(&json), vertices, cuts));
    }
    let json = prepared(3, 2).to_json().unwrap();
    measure("spline_replay/eight_multi_span_replays:", || {
        (0..8).map(|_| replay(black_box(&json), 7, 2)).sum()
    });
    let model = sheet(3);
    let curve = model
        .pcurves()
        .find(|p| p.kind() == hypercurve::CurveFamily2::Nurbs)
        .unwrap();
    assert!(curve.curve().has_certified_injective_image().unwrap());
    measure("spline_replay/prepared_multi_span_injectivity:", || {
        usize::from(curve.curve().has_certified_injective_image().unwrap())
    });
}
