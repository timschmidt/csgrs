//! Independent reconstruction of selected solid boundaries from untrusted bytes.

use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Matrix4, Model, Parameter, Point3, RawModel, Real,
    SolidPointLocation, builder,
};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root() -> Parameter {
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
        panic!("selected quintic root");
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json).unwrap().validate().unwrap();
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json);
    assert_eq!(
        loaded
            .vertices()
            .filter(|v| v.point().try_coordinates().is_none())
            .count(),
        model
            .vertices()
            .filter(|v| v.point().try_coordinates().is_none())
            .count()
    );
    loaded
}

fn check_solid(
    model: &Model,
    transform: &Matrix4,
    volume: Real,
    samples: &[(Point3, SolidPointLocation)],
) {
    let solid = model.solids().next().unwrap();
    for _ in 0..2 {
        assert_eq!(solid.volume().unwrap(), volume);
        for (point, location) in samples {
            assert_eq!(
                solid
                    .classify_point(&transform.transform_point3(point).unwrap())
                    .unwrap(),
                *location
            );
        }
    }
}

fn cuboid_samples() -> Vec<(Point3, SolidPointLocation)> {
    vec![
        (point(1, 1, 1), SolidPointLocation::Inside),
        (point(3, 1, 1), SolidPointLocation::Outside),
        (point(0, 1, 1), SolidPointLocation::Boundary),
        (point(0, 0, 0), SolidPointLocation::Boundary),
    ]
}

#[test]
fn each_cuboid_edge_replays_then_splits_again() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    for edge in source.model().edges() {
        let split = edge.split(root()).unwrap();
        let loaded = reload(split.model());
        check_solid(
            &loaded,
            &Matrix4::identity(),
            Real::from(24),
            &cuboid_samples(),
        );
        let continued = loaded
            .edges()
            .find(|edge| edge.domain().end().try_real().is_none())
            .unwrap()
            .split(rational(1, 2).into())
            .unwrap();
        let loaded = reload(continued.model());
        check_solid(
            &loaded,
            &Matrix4::identity(),
            Real::from(24),
            &cuboid_samples(),
        );
    }
}

fn split_all(model: &Model, kind: Curve3Kind) -> Model {
    let sources = model
        .edges()
        .filter(|e| e.curve().kind() == kind)
        .map(|e| e.curve().index())
        .collect::<Vec<_>>();
    let parameter = root();
    let mut model = model.clone();
    for source in sources {
        let split = model
            .edges()
            .find(|e| e.curve().index() == source)
            .unwrap()
            .split(parameter.clone())
            .unwrap();
        model = split.model().clone();
    }
    model
}

#[test]
fn all_cuboid_edges_replay_without_native_prism_bridges_and_preserve_affine_volume() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let selected = split_all(source.model(), Curve3Kind::Line);
    assert_eq!(selected.counts().vertices, 20);
    assert_eq!(selected.counts().edges, 24);
    for (entries, volume) in [
        ([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], 24),
        ([1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1], 48),
        ([-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1], 24),
    ] {
        let transform = Matrix4::from_row_major(entries.map(Real::from));
        let moved = selected.transformed(&transform).unwrap();
        let loaded = reload(&moved);
        check_solid(&loaded, &transform, Real::from(volume), &cuboid_samples());
        let continued = loaded
            .edges()
            .find(|e| e.domain().end().try_real().is_none())
            .unwrap()
            .split(rational(1, 2).into())
            .unwrap();
        let loaded = reload(continued.model());
        check_solid(&loaded, &transform, Real::from(volume), &cuboid_samples());
        assert_eq!(
            reload(&loaded).to_json().unwrap(),
            loaded.to_json().unwrap()
        );
    }
}

#[test]
fn selected_cylinder_axial_boundaries_rebuild_the_analytic_certificate() {
    let source = builder::cylinder(Real::from(2), Real::from(3)).unwrap();
    let samples = [
        (point(0, 0, 1), SolidPointLocation::Inside),
        (point(3, 0, 1), SolidPointLocation::Outside),
        (point(2, 0, 1), SolidPointLocation::Boundary),
        (point(0, 0, 0), SolidPointLocation::Boundary),
    ];
    for all in [false, true] {
        let selected = if all {
            split_all(source.model(), Curve3Kind::Line)
        } else {
            source
                .model()
                .edges()
                .find(|e| e.curve().kind() == Curve3Kind::Line)
                .unwrap()
                .split(root())
                .unwrap()
                .model()
                .clone()
        };
        let loaded = reload(&selected);
        let volume = Real::from(12) * Real::pi();
        check_solid(&loaded, &Matrix4::identity(), volume.clone(), &samples);
        let continued = loaded
            .edges()
            .find(|e| e.domain().end().try_real().is_none())
            .unwrap()
            .split(rational(1, 2).into())
            .unwrap();
        check_solid(
            &reload(continued.model()),
            &Matrix4::identity(),
            volume,
            &samples,
        );
    }
}

#[test]
fn selected_solid_import_rejects_broken_shell_incidence_orientation_and_geometry() {
    use serde_json::{Value, json};
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let selected = split_all(source.model(), Curve3Kind::Line);
    let value: Value = serde_json::from_str(&selected.to_json().unwrap()).unwrap();
    let mut broken = value.clone();
    broken["shells"][0].as_array_mut().unwrap().pop();
    assert!(
        RawModel::from_json(&broken.to_string())
            .unwrap()
            .validate()
            .is_err()
    );
    let mut broken = value.clone();
    let orientation = &mut broken["faces"][0]["orientation"];
    *orientation = if *orientation == json!("Forward") {
        json!("Reversed")
    } else {
        json!("Forward")
    };
    assert!(
        RawModel::from_json(&broken.to_string())
            .unwrap()
            .validate()
            .is_err()
    );
    let mut broken = value;
    let vertex = broken["vertices"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|v| v.get("CurveImage").is_some())
        .unwrap();
    vertex["CurveImage"]["parameter"] =
        json!({"Scalar": serde_json::to_value(Real::zero()).unwrap()});
    assert!(
        RawModel::from_json(&broken.to_string())
            .unwrap()
            .validate()
            .is_err()
    );
    check_solid(
        &reload(&selected),
        &Matrix4::identity(),
        Real::from(24),
        &cuboid_samples(),
    );
}
