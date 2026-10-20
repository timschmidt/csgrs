use std::hint::black_box;
use std::time::Instant;

use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Model, Parameter, Point3, RawModel, Real,
    SolidPointLocation, builder,
};

const ITERATIONS: usize = 100;

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn root() -> Parameter {
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
    parameters.remove(0)
}

fn split_all(source: &Model) -> Model {
    let curves = source
        .edges()
        .filter(|e| e.curve().kind() == Curve3Kind::Line)
        .map(|e| e.curve().index())
        .collect::<Vec<_>>();
    let parameter = root();
    let mut model = source.clone();
    for curve in curves {
        let next = model
            .edges()
            .find(|e| e.curve().index() == curve)
            .unwrap()
            .split(parameter.clone())
            .unwrap()
            .into_model();
        model = next;
    }
    model
}

fn replay(json: &str) -> Model {
    RawModel::from_json(json, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap()
}

fn check(model: &Model, vertices: usize, edges: usize, volume: &Real) -> usize {
    assert_eq!(model.counts().vertices, vertices);
    assert_eq!(model.counts().edges, edges);
    let solid = model.solids().next().unwrap();
    assert_eq!(&solid.volume().unwrap(), volume);
    assert_eq!(
        solid.classify_point(&point(1, 1, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        solid.classify_point(&point(3, 1, 1)).unwrap(),
        SolidPointLocation::Outside
    );
    vertices + edges
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
    let native = builder::cuboid(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .into_model();
    let selected = native
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap()
        .into_model();
    let all = split_all(&native);
    let cylinder = split_all(
        builder::cylinder(Real::from(2), Real::from(3))
            .unwrap()
            .model(),
    );
    // Timed replay includes untrusted parsing, independent proof reconstruction,
    // exact volume and two point classifications. Authoring/isolation is outside.
    for (name, model, volume) in [
        (
            "selected_solids/native_cuboid_replay:",
            &native,
            Real::from(24),
        ),
        (
            "selected_solids/one_cuboid_edge_replay:",
            &selected,
            Real::from(24),
        ),
        (
            "selected_solids/all_cuboid_edges_replay:",
            &all,
            Real::from(24),
        ),
        (
            "selected_solids/all_cylinder_axial_edges_replay:",
            &cylinder,
            Real::from(12) * Real::pi(),
        ),
    ] {
        let json = model.to_json().unwrap();
        let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
        let parameters = raw["selection"]["parameters"].as_array().unwrap();
        println!(
            "archive_metadata name={name} vertices={} edges={} bytes={} parameter_records={} roots={} images={} pcurve_sources={}",
            model.counts().vertices,
            model.counts().edges,
            json.len(),
            parameters.len(),
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
        assert_eq!(replay(&json).to_json().unwrap(), json);
        measure(name, || {
            check(
                &replay(black_box(&json)),
                model.counts().vertices,
                model.counts().edges,
                &volume,
            )
        });
    }
    let json = all.to_json().unwrap();
    measure("selected_solids/replay_split_replay:", || {
        let loaded = replay(black_box(&json));
        let split = loaded
            .edges()
            .find(|e| e.domain().end().try_real().is_none())
            .unwrap()
            .split((Real::one() / Real::from(2)).unwrap().into())
            .unwrap();
        check(
            &replay(&split.model().to_json().unwrap()),
            21,
            25,
            &Real::from(24),
        )
    });
    measure("selected_solids/eight_replays:", || {
        let mut json = json.clone();
        let mut checksum = 0;
        for _ in 0..8 {
            let loaded = replay(&json);
            checksum += check(&loaded, 20, 24, &Real::from(24));
            let encoded = loaded.to_json().unwrap();
            assert_eq!(encoded, json);
            json = encoded;
        }
        checksum
    });
}
