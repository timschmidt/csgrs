use std::cmp::Ordering;
use std::hint::black_box;
use std::time::Instant;

use hyperbrep::{
    Curve3, CurveParameterLocation, Matrix4, Model, Parameter, Point3, Real, SolidPointLocation,
    builder,
};

const ITERATIONS: usize = 200;

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
        panic!("expected selected parameter");
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn check(model: &Model, selected: bool) -> usize {
    assert_eq!(
        model
            .vertices()
            .any(|v| v.point().try_coordinates().is_none()),
        selected
    );
    let solid = model.solids().next().unwrap();
    assert_eq!(solid.volume().unwrap(), Real::from(24));
    assert_eq!(
        solid.classify_point(&point(1, 1, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    for edge_use in model.edge_uses() {
        let edge = edge_use.edge();
        let parameter = edge.domain().end();
        let local = edge_use.pcurve_parameter_at(parameter).unwrap();
        let recovered = edge_use.edge_parameter_at(&local).unwrap();
        assert_eq!(recovered.compare(parameter).unwrap(), Ordering::Equal);
    }
    model.counts().edges + model.counts().vertices
}

fn measure(name: &str, mut operation: impl FnMut() -> usize) {
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..ITERATIONS {
        checksum += black_box(operation());
    }
    let elapsed = started.elapsed();
    println!(
        "{name} {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );
}

fn main() {
    // Root isolation and source construction are outside timing. Each edit
    // starts from an immutable source revision; topology publication is timed.
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .into_model();
    let selected = root();
    let half: Parameter = (Real::one() / Real::from(2)).unwrap().into();
    for (name, parameter, retained) in [
        (
            "selected_topology/authored_shared_edge_split:",
            &half,
            false,
        ),
        (
            "selected_topology/selected_shared_edge_split:",
            &selected,
            true,
        ),
    ] {
        measure(name, || {
            let split = source
                .edges()
                .next()
                .unwrap()
                .split(black_box(parameter.clone()))
                .unwrap();
            check(split.model(), retained)
        });
    }
    let selected_model = source
        .edges()
        .next()
        .unwrap()
        .split(selected)
        .unwrap()
        .into_model();
    let cuts: Vec<Parameter> = (1..=16)
        .rev()
        .map(|n| (Real::from(n) / Real::from(64)).unwrap().into())
        .collect();
    measure("selected_topology/selected_then_16_splits:", || {
        let mut model = selected_model.clone();
        for cut in &cuts {
            let next = model
                .edges()
                .next()
                .unwrap()
                .split(black_box(cut.clone()))
                .unwrap()
                .into_model();
            model = next;
        }
        assert_eq!(model.counts().vertices, 25);
        assert_eq!(model.counts().edges, 29);
        check(&model, true)
    });
    let reflection =
        Matrix4::from_row_major([-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1].map(Real::from));
    let inverse = reflection.clone().inverse().unwrap();
    measure("selected_topology/selected_reflection_roundtrip:", || {
        let moved = selected_model.transformed(black_box(&reflection)).unwrap();
        let restored = moved.transformed(black_box(&inverse)).unwrap();
        check(&restored, true)
    });
}
