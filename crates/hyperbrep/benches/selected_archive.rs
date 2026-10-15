use std::hint::black_box;
use std::time::Instant;

use hyperbrep::{
    Curve3, CurveParameterLocation, Model, Parameter, Point3, RawModel, Real, Vector3, builder,
};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2};

const ITERATIONS: usize = 100;

fn selected_root() -> Parameter {
    let curve = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| {
                Point3::new(
                    (Real::from(n) / Real::from(5)).unwrap(),
                    Real::zero(),
                    Real::zero(),
                )
            })
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("finite root");
    };
    assert_eq!(parameters.len(), 1);
    let root = parameters.remove(0);
    assert!(root.try_real().is_none());
    root
}

fn check(model: &Model, expected_edges: usize, selected: bool) -> usize {
    assert_eq!(model.counts().edges, expected_edges);
    assert_eq!(model.counts().vertices, expected_edges);
    assert_eq!(model.faces().next().unwrap().area().unwrap(), Real::from(4));
    assert_eq!(
        model
            .vertices()
            .any(|vertex| vertex.point().try_coordinates().is_none()),
        selected
    );
    expected_edges * 2
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

fn main() {
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
    let native = builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y())
        .unwrap()
        .into_model();
    let selected = native
        .edges()
        .next()
        .unwrap()
        .split(selected_root())
        .unwrap()
        .into_model();
    let mut repeated = selected.clone();
    for n in (1..=16).rev() {
        let next = repeated
            .edges()
            .next()
            .unwrap()
            .split((Real::from(n) / Real::from(64)).unwrap().into())
            .unwrap()
            .into_model();
        repeated = next;
    }
    // Source construction and root isolation are outside timing. Replay
    // includes untrusted JSON decoding, proof reconstruction and exact area.
    for (encode_name, replay_name, model, edges, retained) in [
        (
            "selected_archive/native_encode:",
            "selected_archive/native_replay:",
            &native,
            4,
            false,
        ),
        (
            "selected_archive/selected_encode:",
            "selected_archive/selected_replay:",
            &selected,
            5,
            true,
        ),
        (
            "selected_archive/16_cuts_encode:",
            "selected_archive/16_cuts_replay:",
            &repeated,
            21,
            true,
        ),
    ] {
        let json = model.to_json().unwrap();
        let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(
            raw["selection"]["parameters"].as_array().unwrap().len(),
            usize::from(retained)
        );
        assert_eq!(
            raw["selection"]["pcurve_sources"].as_array().unwrap().len(),
            usize::from(retained)
        );
        println!(
            "archive_metadata edges={edges} bytes={} selected_roots={}",
            json.len(),
            usize::from(retained)
        );
        measure(encode_name, || {
            let encoded = model.to_json().unwrap();
            assert_eq!(encoded, json);
            encoded.len()
        });
        measure(replay_name, || {
            let loaded =
                RawModel::from_json(black_box(&json), &hyperbrep::ArchiveLimits::default())
                    .unwrap()
                    .validate()
                    .unwrap();
            check(&loaded, edges, retained)
        });
    }
    let json = selected.to_json().unwrap();
    measure("selected_archive/replay_split_measure:", || {
        let loaded = RawModel::from_json(black_box(&json), &hyperbrep::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let split = loaded
            .edges()
            .next()
            .unwrap()
            .split((Real::one() / Real::from(2)).unwrap().into())
            .unwrap();
        check(split.model(), 6, true)
    });
}
