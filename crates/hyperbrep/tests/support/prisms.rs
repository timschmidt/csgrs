//! Exact prism expectations are derived from circular area and rectangular cuts.
use hyperbrep::{
    BodySet, Curve3Kind, Matrix4, Model, Point3, Real, SolidPointLocation, boolean, builder,
};

#[allow(dead_code)]
#[path = "seam_cuts.rs"]
mod seam;
pub use seam::{equal, point as p, ratio as q};

fn samples(column: usize) -> Vec<(Point3, SolidPointLocation)> {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    vec![
        (p(-1, 0, 3), [I, O, I]),
        (p(1, 0, 3), [O, I, I]),
        (p(0, 0, 3), [B, B, I]),
        (p(-5, 0, 3), [B, O, B]),
        (p(5, 0, 3), [O, B, B]),
        (
            Point3::new(Real::from(-5) + &delta, 0.into(), 3.into()),
            [I, O, I],
        ),
        (
            Point3::new(Real::from(-5) - &delta, 0.into(), 3.into()),
            [O, O, O],
        ),
        (Point3::new(-&delta, 0.into(), 3.into()), [I, O, I]),
        (Point3::new(delta.clone(), 0.into(), 3.into()), [O, I, I]),
        (p(-1, 0, 0), [B, O, B]),
        (p(-1, 0, 6), [B, O, B]),
        (
            Point3::new((-1).into(), 0.into(), Real::from(6) - &delta),
            [I, O, I],
        ),
        (
            Point3::new((-1).into(), 0.into(), Real::from(6) + &delta),
            [O, O, O],
        ),
    ]
    .into_iter()
    .map(|(point, expected)| (point, expected[column]))
    .collect()
}

fn verify(model: &Model, column: usize, map: &Matrix4, scale: &Real, stage: &str) {
    assert_eq!(model.solids().count(), 1);
    let volume = Real::from(if column == 2 { 150 } else { 75 }) * Real::pi();
    let area = if column == 2 {
        Real::from(110) * Real::pi()
    } else {
        Real::from(55) * Real::pi() + Real::from(60)
    };
    assert_eq!(
        hyperlimit::compare_reals(
            &model.solids().next().unwrap().volume().unwrap(),
            &(volume * scale * scale * scale),
            hyperbrep::STRICT_PREDICATES,
        )
        .value(),
        Some(std::cmp::Ordering::Equal),
        "column={column} stage={stage}: exact volume",
    );
    let actual_area = model
        .faces()
        .map(|face| face.area().expect("a circular prism has exact face areas"))
        .fold(Real::zero(), |sum, value| sum + value);
    equal(&actual_area, &(area * scale * scale));
    assert!(
        model
            .edges()
            .any(|edge| edge.curve().kind() == Curve3Kind::CircleArc)
    );
    assert!(model.edges().all(|edge| matches!(
        edge.curve().kind(),
        Curve3Kind::Line | Curve3Kind::CircleArc
    )));
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    for (point, expected) in samples(column) {
        assert_eq!(
            model
                .solids()
                .next()
                .unwrap()
                .classify_point(&map.transform_point3(&point).unwrap())
                .unwrap(),
            expected
        );
    }
}

fn reload(model: &Model, stage: &str) -> Model {
    let json = model.to_json().unwrap();
    let raw = hyperbrep::RawModel::from_json(&json, &hyperbrep::ArchiveLimits::default()).unwrap();
    let replay = raw.validate().unwrap_or_else(|e| {
        #[cfg(not(target_arch = "wasm32"))]
        if let Ok(path) = std::env::var("HYPERBREP_REPLAY_FAILURE") {
            std::fs::write(path, &json).unwrap();
        }
        panic!("{stage}: {e:?}")
    });
    assert_eq!(json, replay.to_json().unwrap());
    replay
}

fn check(result: &BodySet, column: usize) -> u64 {
    check_in_frame(result, column, &Matrix4::identity(), "same height")
}

fn check_in_frame(result: &BodySet, column: usize, frame: &Matrix4, context: &str) -> u64 {
    assert_eq!(result.len(), 1);
    let json = result.model().to_json().unwrap();
    let replay = reload(result.model(), &format!("{context} column={column} result"));
    for (stage, model) in [("result", result.model()), ("archive replay", &replay)] {
        verify(
            model,
            column,
            frame,
            &Real::one(),
            &format!("{context} {stage}"),
        );
    }
    let edge = replay
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    let middle = ((edge.domain().start().try_real().unwrap()
        + edge.domain().end().try_real().unwrap())
        / Real::from(2))
    .unwrap();
    let split = edge.split(middle.into()).unwrap();
    verify(split.model(), column, frame, &Real::one(), "split");
    let replay = reload(split.model(), &format!("{context} column={column} split"));
    verify(&replay, column, frame, &Real::one(), "split replay");
    let scale = q(1, 1000);
    let map = seam::similarity(&scale, true);
    let composed = map.clone() * frame.clone();
    let moved = replay.transformed(&map).unwrap();
    verify(
        &reload(
            &moved,
            &format!("{context} column={column} transformed split"),
        ),
        column,
        &composed,
        &scale,
        "transformed replay",
    );
    json.bytes().fold(0xcbf29ce484222325_u64, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
    })
}

/// Same-height control: only representation preservation changes this workflow.
pub fn same_height_cylinder() -> u64 {
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let cutter = builder::cuboid(p(0, -20, 0), p(20, 20, 6)).unwrap();
    let difference = boolean::difference(&source, &cutter).unwrap();
    let intersection = boolean::intersection(&source, &cutter).unwrap();
    let mut receipt = check(&difference, 0);
    let mut record = |value: u64| receipt = (receipt ^ value).wrapping_mul(0x100000001b3);
    record(check(&intersection, 1));
    record(check(&boolean::intersection(&cutter, &source).unwrap(), 1));
    let difference = BodySet::from_solid(seam::reload(difference.model()).solids().next().unwrap());
    let intersection =
        BodySet::from_solid(seam::reload(intersection.model()).solids().next().unwrap());
    for (first, second) in [(&difference, &intersection), (&intersection, &difference)] {
        record(check(&boolean::union(first, second).unwrap(), 2));
    }
    record(check(
        &boolean::difference(&difference, &cutter).unwrap(),
        0,
    ));
    let empty = boolean::difference(&difference, &source).unwrap();
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    record(check(&boolean::union(&empty, &intersection).unwrap(), 1));
    receipt
}

pub fn rotation(kind: usize) -> Matrix4 {
    let (c, s) = match kind {
        0 => (Real::one(), Real::zero()),
        1 => {
            let r = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
            (r.clone(), r)
        }
        2 => (q(3, 5), q(4, 5)),
        _ => panic!("fixture rotation"),
    };
    Matrix4::from_row_major([
        c.clone(),
        -&s,
        0.into(),
        0.into(),
        s,
        c,
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}

/// The original 40-unit cutters remain intact alongside exact endpoint controls.
pub fn covering_cylinders() -> u64 {
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let mut receipt = 0xcbf29ce484222325_u64;
    for (slab, (lower, upper)) in [
        ((-20).into(), 20.into()),
        (0.into(), 20.into()),
        ((-20).into(), 6.into()),
        (-&delta, Real::from(6) + &delta),
    ]
    .into_iter()
    .enumerate()
    {
        for kind in 0..3 {
            let map = rotation(kind);
            let cutter = builder::cuboid(
                Point3::new(0.into(), (-20).into(), lower.clone()),
                Point3::new(20.into(), 20.into(), upper.clone()),
            )
            .unwrap()
            .transformed(&map)
            .unwrap();
            let difference = boolean::difference(&source, &cutter)
                .unwrap_or_else(|e| panic!("slab={slab} rotation={kind}: {e:?}"));
            let intersection = boolean::intersection(&source, &cutter).unwrap();
            let mut record = |set: &BodySet, column| {
                receipt = (receipt
                    ^ check_in_frame(set, column, &map, &format!("slab={slab} rotation={kind}")))
                .wrapping_mul(0x100000001b3);
            };
            record(&difference, 0);
            record(&intersection, 1);
            record(&boolean::intersection(&cutter, &source).unwrap(), 1);
            let a = BodySet::from_solid(seam::reload(difference.model()).solids().next().unwrap());
            let b =
                BodySet::from_solid(seam::reload(intersection.model()).solids().next().unwrap());
            record(&boolean::union(&a, &b).unwrap(), 2);
            record(&boolean::union(&b, &a).unwrap(), 2);
            record(&boolean::difference(&a, &cutter).unwrap(), 0);
            let empty = boolean::difference(&b, &cutter).unwrap();
            assert_eq!(
                empty.model().to_json().unwrap(),
                BodySet::empty().model().to_json().unwrap()
            );
            record(&boolean::union(&empty, &a).unwrap(), 0);
        }
    }
    receipt
}

fn verify_measures(model: &Model, volumes: &[Real], area: &Real, stage: &str) {
    let mut actual = model
        .solids()
        .map(|s| s.volume().unwrap())
        .collect::<Vec<_>>();
    assert_eq!(actual.len(), volumes.len(), "{stage}");
    for expected in volumes {
        let index = actual
            .iter()
            .position(|v| {
                hyperlimit::compare_reals(v, expected, hyperbrep::STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Equal)
            })
            .unwrap_or_else(|| panic!("{stage}: exact component volume"));
        actual.remove(index);
    }
    let sum = model
        .faces()
        .map(|f| f.area().unwrap())
        .fold(Real::zero(), |a, b| a + b);
    equal(&sum, area);
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

fn check_measures_and_replay(set: &BodySet, volumes: &[Real], area: &Real, stage: &str) -> Model {
    verify_measures(set.model(), volumes, area, stage);
    let model = reload(set.model(), stage);
    verify_measures(&model, volumes, area, stage);
    model
}

/// Through holes include a translated bore and an oppositely directed cutter.
pub fn covering_bores() -> u64 {
    let mut receipt = 0xcbf29ce484222325_u64;
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    for offset in [0, 1] {
        for reversed in [false, true] {
            for moved in 0..3 {
                let frame = match moved {
                    0 => Matrix4::identity(),
                    1 => seam::similarity(&Real::one(), true),
                    2 => Matrix4::from_row_major([
                        1.into(),
                        0.into(),
                        0.into(),
                        2.into(),
                        0.into(),
                        q(3, 5),
                        -q(4, 5),
                        (-3).into(),
                        0.into(),
                        q(4, 5),
                        q(3, 5),
                        1.into(),
                        0.into(),
                        0.into(),
                        0.into(),
                        1.into(),
                    ]),
                    _ => unreachable!(),
                };
                let source = builder::cylinder(5.into(), 6.into())
                    .unwrap()
                    .transformed(&frame)
                    .unwrap();
                let cutter_frame = Matrix4::from_row_major([
                    1.into(),
                    0.into(),
                    0.into(),
                    offset.into(),
                    0.into(),
                    1.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    0.into(),
                    Real::from(if reversed { -1 } else { 1 }),
                    Real::from(if reversed { 20 } else { -20 }),
                    0.into(),
                    0.into(),
                    0.into(),
                    1.into(),
                ]);
                let cutter = builder::cylinder(2.into(), 40.into())
                    .unwrap()
                    .transformed(&(frame.clone() * cutter_frame))
                    .unwrap();
                let stage = format!("bore offset={offset} reversed={reversed} moved={moved}");
                let result = boolean::difference(&source, &cutter)
                    .unwrap_or_else(|e| panic!("{stage}: {e:?}"));
                let expected = Real::from(126) * Real::pi();
                let model = check_measures_and_replay(
                    &result,
                    std::slice::from_ref(&expected),
                    &expected,
                    &stage,
                );
                let body = model.solids().next().unwrap();
                for (point, location) in [
                    (p(offset, 0, 3), SolidPointLocation::Outside),
                    (p(offset + 2, 0, 3), SolidPointLocation::Boundary),
                    (
                        Point3::new(Real::from(offset + 2) - &delta, 0.into(), 3.into()),
                        SolidPointLocation::Outside,
                    ),
                    (
                        Point3::new(Real::from(offset + 2) + &delta, 0.into(), 3.into()),
                        SolidPointLocation::Inside,
                    ),
                    (p(-5, 0, 3), SolidPointLocation::Boundary),
                    (p(5, 0, 3), SolidPointLocation::Boundary),
                    (p(-4, 0, 0), SolidPointLocation::Boundary),
                    (
                        Point3::new((-4).into(), 0.into(), -&delta),
                        SolidPointLocation::Outside,
                    ),
                ] {
                    assert_eq!(
                        body.classify_point(&frame.transform_point3(&point).unwrap())
                            .unwrap(),
                        location,
                        "{stage}"
                    );
                }
                let edge = model
                    .edges()
                    .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
                    .unwrap();
                let (a, b) = edge.domain().try_real_endpoints().unwrap();
                let split = edge.split(((a + b) * q(1, 2)).into()).unwrap();
                verify_measures(
                    &reload(split.model(), &stage),
                    std::slice::from_ref(&expected),
                    &expected,
                    &stage,
                );
                let again = boolean::difference(&BodySet::from_solid(body), &cutter)
                    .unwrap_or_else(|e| panic!("{stage} recut: {e:?}"));
                check_measures_and_replay(
                    &again,
                    std::slice::from_ref(&expected),
                    &expected,
                    &stage,
                );
                let plug = boolean::intersection(&source, &cutter).unwrap();
                for (a, b) in [(&result, &plug), (&plug, &result)] {
                    let restored = boolean::union(a, b).unwrap();
                    check_measures_and_replay(
                        &restored,
                        &[Real::from(150) * Real::pi()],
                        &(Real::from(110) * Real::pi()),
                        &stage,
                    );
                }
                let empty = boolean::difference(&result, &source).unwrap();
                assert_eq!(
                    empty.model().to_json().unwrap(),
                    BodySet::empty().model().to_json().unwrap()
                );
                let recovered = boolean::union(&empty, &result).unwrap();
                check_measures_and_replay(
                    &recovered,
                    std::slice::from_ref(&expected),
                    &expected,
                    &stage,
                );
                for byte in model.to_json().unwrap().bytes() {
                    receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
                }
            }
        }
    }
    receipt
}

/// A half annulus has two radial cut rectangles as well as both circular sides.
pub fn annular_cut() -> u64 {
    let outer = builder::cylinder(5.into(), 6.into()).unwrap();
    let inner = builder::cylinder(2.into(), 6.into()).unwrap();
    let source = boolean::difference(&outer, &inner).unwrap();
    let cutter = builder::cuboid(p(0, -20, -20), p(20, 20, 20)).unwrap();
    let result = boolean::difference(&source, &cutter).unwrap();
    let volume = Real::from(63) * Real::pi();
    let area = Real::from(63) * Real::pi() + Real::from(36);
    let model =
        check_measures_and_replay(&result, std::slice::from_ref(&volume), &area, "annular cut");
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    for (point, location) in [
        (p(-3, 0, 3), SolidPointLocation::Inside),
        (p(3, 0, 3), SolidPointLocation::Outside),
        (p(-1, 0, 3), SolidPointLocation::Outside),
        (p(-2, 0, 3), SolidPointLocation::Boundary),
        (p(0, 3, 3), SolidPointLocation::Boundary),
        (
            Point3::new(-&delta, 3.into(), 3.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(delta, 3.into(), 3.into()),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(
            model
                .solids()
                .next()
                .unwrap()
                .classify_point(&point)
                .unwrap(),
            location
        );
    }
    let remainder = boolean::intersection(&source, &cutter).unwrap();
    for (a, b) in [(&result, &remainder), (&remainder, &result)] {
        check_measures_and_replay(
            &boolean::union(a, b).unwrap(),
            &[Real::from(126) * Real::pi()],
            &(Real::from(126) * Real::pi()),
            "annulus restore",
        );
    }
    check_measures_and_replay(
        &boolean::difference(&result, &cutter).unwrap(),
        &[volume],
        &area,
        "annulus recut",
    );
    assert!(boolean::difference(&remainder, &cutter).unwrap().is_empty());
    model
        .to_json()
        .unwrap()
        .bytes()
        .fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        })
}

/// Partial-height slabs must not be silently treated as through cuts.
pub fn slab_boundaries() {
    use hyperbrep::{BooleanError, BuildError, ConstructionError};
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    for (lower, upper, partial) in [
        (Real::from(-20), Real::from(6) - &delta, true),
        (delta.clone(), Real::from(20), true),
        (Real::from(2), Real::from(4), true),
        (Real::from(6), Real::from(20), false),
        (Real::from(6) + &delta, Real::from(20), false),
    ] {
        let cutter = builder::cuboid(
            Point3::new(0.into(), (-20).into(), lower),
            Point3::new(20.into(), 20.into(), upper),
        )
        .unwrap();
        let result = boolean::difference(&source, &cutter);
        if partial {
            match result.unwrap_err() {
                BooleanError::FallbackFailed { optimized, graph } => {
                    assert!(matches!(
                        *optimized,
                        BooleanError::IncompatibleExtrusionSlabs
                    ));
                    assert!(matches!(
                        *graph,
                        BooleanError::Construction(ConstructionError::Build(
                            BuildError::UnsupportedSolidShell(_)
                        ))
                    ));
                }
                error => panic!("unexpected outcome: {error:?}"),
            }
        } else {
            check_measures_and_replay(
                &result.unwrap(),
                &[Real::from(150) * Real::pi()],
                &(Real::from(110) * Real::pi()),
                "touching or disjoint axial slab",
            );
            assert!(boolean::intersection(&source, &cutter).unwrap().is_empty());
        }
        assert_eq!(source.bodies().count(), 1);
    }
    let planar = builder::cuboid(p(-5, -5, 0), p(5, 5, 6)).unwrap();
    let cutter = builder::cuboid(
        p(0, -20, -20),
        Point3::new(20.into(), 20.into(), Real::from(6) - &delta),
    )
    .unwrap();
    let result = boolean::difference(&planar, &cutter).unwrap();
    let model = check_measures_and_replay(
        &result,
        &[Real::from(300) + Real::from(50) * &delta],
        &(Real::from(380) + Real::from(10) * &delta),
        "tiny planar cap",
    );
    assert_eq!(
        model
            .solids()
            .next()
            .unwrap()
            .classify_point(&Point3::new(
                1.into(),
                0.into(),
                Real::from(6) - &delta * q(1, 2)
            ))
            .unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        model
            .solids()
            .next()
            .unwrap()
            .classify_point(&p(1, 0, 3))
            .unwrap(),
        SolidPointLocation::Outside
    );
}
