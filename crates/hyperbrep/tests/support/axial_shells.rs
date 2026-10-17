//! Independent material expectations and continued consumption of axial shells.
use hyperbrep::*;

pub fn periodic_workflow(model: Model) {
    let source = BodySet::from_solid(model.solids().next().unwrap());
    let tool = builder::cuboid(point(0, -20, 1), point(20, 20, 3)).unwrap();
    let body = boolean::difference(&source, &tool).unwrap();
    let check = |model: &Model| {
        measures(
            model,
            &(Real::from(12) * Real::pi()),
            &(Real::from(24) * Real::pi() + Real::from(8)),
        );
        for (p, location) in [
            (point(-1, 0, 2), SolidPointLocation::Inside),
            (point(1, 0, 2), SolidPointLocation::Outside),
            (point(1, 0, 1), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                model.solids().next().unwrap().classify_point(&p).unwrap(),
                location
            );
        }
    };
    let cold = replay(body.model());
    check(&cold);
    let body = BodySet::from_solid(cold.solids().next().unwrap());
    check(&replay(boolean::difference(&body, &tool).unwrap().model()));
    let plug = boolean::intersection(&source, &tool).unwrap();
    let restored = boolean::union(&body, &plug).unwrap();
    measures(
        &replay(restored.model()),
        &(Real::from(16) * Real::pi()),
        &(Real::from(24) * Real::pi()),
    );
    let empty = boolean::difference(&body, &source).unwrap();
    assert!(empty.is_empty());
    check(boolean::union(&empty, &body).unwrap().model());
}

pub fn scaled_workflow() {
    let (a, b) = operands(2);
    let scale = (Real::from(3) / Real::from(2)).unwrap();
    let map = Matrix4::from_row_major([
        scale.clone(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        scale.clone(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        scale.clone(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let body = boolean::difference(&a, &b)
        .unwrap()
        .transformed(&map)
        .unwrap();
    let (volume, area) = expected(2);
    let verify = |model: &Model| {
        measures(
            model,
            &(&volume * &scale * &scale * &scale),
            &(&area * &scale * &scale),
        )
    };
    let cold = replay(body.model());
    verify(&cold);
    let body = BodySet::from_solid(cold.solids().next().unwrap());
    let a = a.transformed(&map).unwrap();
    let b = b.transformed(&map).unwrap();
    verify(&replay(boolean::difference(&a, &b).unwrap().model()));
    verify(&replay(boolean::difference(&body, &b).unwrap().model()));
    let empty = boolean::difference(&body, &a).unwrap();
    assert!(empty.is_empty());
    verify(boolean::union(&empty, &body).unwrap().model());
}

pub fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
pub fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal)
    );
}
pub fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    cold
}
pub fn expected(kind: usize) -> (Real, Real) {
    let delta: Real = "1e-12".parse().unwrap();
    let pi = Real::pi();
    match kind {
        0 | 1 => (
            Real::from(75) * &pi + (Real::from(25) * &pi * &delta / Real::from(2)).unwrap(),
            (Real::from(80) + Real::from(5) * &delta) * &pi + Real::from(60)
                - Real::from(10) * &delta,
        ),
        2 => (
            Real::from(125) * &pi,
            Real::from(125) * &pi + Real::from(20),
        ),
        3 => (
            Real::from(32_000) - Real::from(75) * &pi,
            Real::from(6340) + Real::from(55) * &pi,
        ),
        _ => panic!("unknown fixture"),
    }
}
pub fn operands(kind: usize) -> (BodySet, BodySet) {
    let delta: Real = "1e-12".parse().unwrap();
    let (low, high) = match kind {
        0 => ((-20).into(), Real::from(6) - delta),
        1 => (delta, 20.into()),
        2 => (2.into(), 4.into()),
        3 => ((-20).into(), 20.into()),
        _ => panic!("unknown fixture"),
    };
    let cylinder = builder::cylinder(5.into(), 6.into()).unwrap();
    let tool = builder::cuboid(
        Point3::new(0.into(), (-20).into(), low),
        Point3::new(20.into(), 20.into(), high),
    )
    .unwrap();
    if kind == 3 {
        (tool, cylinder)
    } else {
        (cylinder, tool)
    }
}
pub fn frame(reflected: bool) -> Matrix4 {
    let q = |n| (Real::from(n) / Real::from(5)).unwrap();
    let sign = Real::from(if reflected { -1 } else { 1 });
    Matrix4::from_row_major([
        0.into(),
        0.into(),
        1.into(),
        7.into(),
        q(3) * &sign,
        -q(4),
        0.into(),
        (-2).into(),
        q(4) * sign,
        q(3),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}
pub fn measures(model: &Model, volume: &Real, area: &Real) {
    assert_eq!(model.solids().count(), 1);
    equal(&model.solids().next().unwrap().volume().unwrap(), volume);
    equal(&model.faces().map(|f| f.area().unwrap()).sum(), area);
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}
pub fn verify(model: &Model, kind: usize, map: &Matrix4) {
    let (volume, area) = expected(kind);
    measures(model, &volume, &area);
    let delta: Real = "1e-12".parse().unwrap();
    let solid = model.solids().next().unwrap();
    let classify = |p: Point3| {
        solid
            .classify_point(&map.transform_point3(&p).unwrap())
            .unwrap()
    };
    use SolidPointLocation::{Boundary, Inside, Outside};
    for (p, expected) in if kind == 3 {
        vec![
            (point(1, 0, 3), Outside),
            (point(6, 0, 3), Inside),
            (point(5, 0, 3), Boundary),
            (
                Point3::new(Real::from(5) - &delta, 0.into(), 3.into()),
                Outside,
            ),
            (
                Point3::new(Real::from(5) + &delta, 0.into(), 3.into()),
                Inside,
            ),
        ]
    } else {
        vec![
            (point(-1, 0, 3), Inside),
            (point(1, 0, 3), Outside),
            (point(0, 0, 3), Boundary),
            (Point3::new(-&delta, 0.into(), 3.into()), Inside),
            (Point3::new(delta.clone(), 0.into(), 3.into()), Outside),
        ]
    } {
        assert_eq!(classify(p), expected);
    }
    if kind < 3 {
        let events = match kind {
            0 => vec![(Real::from(6) - &delta, false)],
            1 => vec![(delta.clone(), true)],
            _ => vec![(Real::from(2), true), (Real::from(4), false)],
        };
        for (event, loses_material) in events {
            assert_eq!(
                classify(Point3::new((-1).into(), 0.into(), event.clone())),
                Inside
            );
            assert_eq!(
                classify(Point3::new(1.into(), 0.into(), event.clone())),
                Boundary
            );
            assert_eq!(
                classify(Point3::new(0.into(), 0.into(), event.clone())),
                Boundary
            );
            let half = (&delta / Real::from(2)).unwrap();
            let below = classify(Point3::new(1.into(), 0.into(), &event - &half));
            let above = classify(Point3::new(1.into(), 0.into(), &event + half));
            assert_eq!(
                (below, above),
                if loses_material {
                    (Inside, Outside)
                } else {
                    (Outside, Inside)
                }
            );
        }
    }
}
pub fn continued(model: &Model, kind: usize, map: &Matrix4) {
    verify(model, kind, map);
    let body = BodySet::from_solid(model.solids().next().unwrap());
    let (a, b) = operands(kind);
    let a = a.transformed(map).unwrap();
    let b = b.transformed(map).unwrap();
    let recut = boolean::difference(&body, &b).unwrap();
    verify(&replay(recut.model()), kind, map);
    let plug = boolean::intersection(&a, &b).unwrap();
    let delta: Real = "1e-12".parse().unwrap();
    let h = match kind {
        0 | 1 => Real::from(6) - &delta,
        2 => Real::from(2),
        _ => Real::from(6),
    };
    measures(
        plug.model(),
        &(Real::from(25) * Real::pi() * &h / Real::from(2)).unwrap(),
        &((Real::from(5) * &h + Real::from(25)) * Real::pi() + Real::from(10) * h),
    );
    let reconstructed = boolean::union(&body, &plug).unwrap();
    let (volume, area) = if kind == 3 {
        (Real::from(32_000), Real::from(6400))
    } else {
        (Real::from(150) * Real::pi(), Real::from(110) * Real::pi())
    };
    measures(&replay(reconstructed.model()), &volume, &area);
    let half_tool = builder::cuboid(point(-30, 0, -30), point(30, 30, 30))
        .unwrap()
        .transformed(map)
        .unwrap();
    let lower = boolean::difference(&body, &half_tool).unwrap();
    let upper = boolean::intersection(&body, &half_tool).unwrap();
    let (volume, area) = expected(kind);
    let cross = match kind {
        0 | 1 => Real::from(30) + Real::from(5) * delta,
        2 => Real::from(50),
        _ => Real::from(770),
    };
    for half in [&lower, &upper] {
        measures(
            &replay(half.model()),
            &(&volume / Real::from(2)).unwrap(),
            &((&area / Real::from(2)).unwrap() + &cross),
        );
    }
    let restored = boolean::union(&lower, &upper).unwrap();
    verify(&replay(restored.model()), kind, map);
    let empty = boolean::difference(&body, &a).unwrap();
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    verify(boolean::union(&empty, &body).unwrap().model(), kind, map);
}
pub fn workflow(kind: usize, reflected: bool) -> BodySet {
    let (a, b) = operands(kind);
    let result = boolean::difference(&a, &b).unwrap();
    verify(result.model(), kind, &Matrix4::identity());
    verify(&replay(result.model()), kind, &Matrix4::identity());
    let map = frame(reflected);
    let moved = result.transformed(&map).unwrap();
    let cold = replay(moved.model());
    continued(&cold, kind, &map);
    let transformed =
        boolean::difference(&a.transformed(&map).unwrap(), &b.transformed(&map).unwrap()).unwrap();
    verify(&replay(transformed.model()), kind, &map);
    BodySet::from_solid(cold.solids().next().unwrap())
}
