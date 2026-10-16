//! Original quarter/band continuations, with independent exact expectations.
use hyperbrep::{BodySet, Matrix4, Model, Point3, Real, SolidPointLocation, boolean, builder};
#[allow(dead_code)]
#[path = "seam_cuts.rs"]
pub(super) mod seam;
use seam::{equal, point as p, ratio as q};

fn parent_kind(kind: usize) -> usize {
    match kind {
        0 => 0,
        1 | 3 => 1,
        2 | 4 => 2,
        _ => unreachable!(),
    }
}

pub fn fixture(kind: usize) -> (BodySet, BodySet, Real, Real) {
    let (source, first_cut, volume, area) = seam::fixture(parent_kind(kind));
    let half = boolean::intersection(&source, &first_cut).unwrap();
    let cutter = if kind < 3 {
        builder::cuboid(p(-20, 0, -20), p(20, 20, 20))
    } else {
        builder::cuboid(p(-20, -20, 1), p(20, 20, 20))
    }
    .unwrap();
    (half, cutter, volume, area)
}

fn expected(kind: usize, difference: bool) -> (Real, Real) {
    let pi = Real::pi();
    let pi2 = &pi * &pi;
    let sqrt3 = Real::from(3).sqrt().unwrap();
    let sqrt5 = Real::from(5).sqrt().unwrap();
    match kind {
        0 => (
            Real::from(10) * &pi2,
            Real::from(10) * pi2 + Real::from(24) * pi,
        ),
        1 => (
            Real::from(10) * &pi2,
            Real::from(10) * pi2 + Real::from(8) * pi,
        ),
        2 => (
            q(39, 2) * &pi,
            (Real::from(29) + Real::from(21) * sqrt5) * &pi * q(1, 4) + Real::from(42),
        ),
        3 if !difference => (
            Real::from(5) * &pi * (q(4, 3) * &pi - &sqrt3),
            q(20, 3) * pi2 + Real::from(10) * &pi * &sqrt3 + q(8, 3) * pi - Real::from(2) * sqrt3,
        ),
        3 => (
            Real::from(20) * &pi2 - Real::from(5) * &pi * (q(4, 3) * &pi - &sqrt3),
            q(40, 3) * pi2 + Real::from(10) * &pi * &sqrt3 + q(16, 3) * pi + Real::from(2) * sqrt3,
        ),
        4 if !difference => (
            q(665, 24) * &pi,
            (Real::from(97) + Real::from(65) * sqrt5) * pi * q(1, 8) + q(65, 2),
        ),
        4 => (
            q(271, 24) * &pi,
            (Real::from(181) + Real::from(19) * sqrt5) * pi * q(1, 8) + q(19, 2),
        ),
        _ => unreachable!(),
    }
}

fn measures(model: &Model, volume: &Real, area: &Real) {
    assert_eq!(model.solids().count(), 1);
    equal(&model.solids().next().unwrap().volume().unwrap(), volume);
    equal(
        &model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

fn samples(kind: usize, column: usize, map: &Matrix4) -> Vec<(Point3, SolidPointLocation)> {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let d = q(1, 1_000_000) * q(1, 1_000_000);
    // Columns describe intersection, difference and the original half. They
    // are derived from the analytic sets, independently of the BREP queries.
    let rows = match kind {
        0 => vec![
            (p(0, 5, 1), [I, O, I]),
            (p(0, -5, 1), [O, I, I]),
            (p(0, 5, -1), [O, O, O]),
            (p(5, 0, 1), [B, B, I]),
            (p(0, 5, 0), [B, O, B]),
            (Point3::new(5.into(), d.clone(), 1.into()), [I, O, I]),
            (Point3::new(5.into(), -&d, 1.into()), [O, I, I]),
            (
                Point3::new(0.into(), 5.into(), Real::from(2) - &d),
                [I, O, I],
            ),
            (p(0, 5, 2), [B, O, B]),
            (
                Point3::new(0.into(), 5.into(), Real::from(2) + &d),
                [O, O, O],
            ),
            (p(0, 0, 1), [O, O, O]),
        ],
        1 => vec![
            (p(3, 4, 1), [I, O, I]),
            (p(-3, 4, 1), [O, O, O]),
            (p(3, -4, 1), [O, I, I]),
            (p(5, 0, 1), [B, B, I]),
            (p(0, 5, 1), [B, O, B]),
            (Point3::new(5.into(), d.clone(), 1.into()), [I, O, I]),
            (Point3::new(5.into(), -&d, 1.into()), [O, I, I]),
            (
                Point3::new(3.into(), 4.into(), Real::from(2) - &d),
                [I, O, I],
            ),
            (p(3, 4, 2), [B, O, B]),
            (
                Point3::new(3.into(), 4.into(), Real::from(2) + &d),
                [O, O, O],
            ),
        ],
        2 => {
            let radial = |r: Real| Point3::new(&r * q(3, 5), r * q(4, 5), 3.into());
            vec![
                (p(1, 1, 3), [I, O, I]),
                (p(-1, 1, 3), [O, O, O]),
                (p(1, -1, 3), [O, I, I]),
                (p(0, 1, 3), [B, O, B]),
                (p(1, 0, 3), [B, B, I]),
                (Point3::new(1.into(), d.clone(), 3.into()), [I, O, I]),
                (Point3::new(1.into(), -&d, 3.into()), [O, I, I]),
                (radial(q(7, 2) - &d), [I, O, I]),
                (radial(q(7, 2)), [B, O, B]),
                (radial(q(7, 2) + &d), [O, O, O]),
                (p(1, 1, 0), [B, O, B]),
            ]
        }
        3 => vec![
            (Point3::new(5.into(), 0.into(), q(3, 2)), [I, O, I]),
            (Point3::new(5.into(), 0.into(), q(1, 2)), [O, I, I]),
            (Point3::new((-5).into(), 0.into(), q(3, 2)), [O, O, O]),
            (p(5, 0, 1), [B, B, I]),
            (Point3::new(0.into(), 5.into(), q(3, 2)), [B, O, B]),
            (Point3::new(5.into(), 0.into(), Real::one() + &d), [I, O, I]),
            (Point3::new(5.into(), 0.into(), Real::one() - &d), [O, I, I]),
            (
                Point3::new(5.into(), 0.into(), Real::from(2) - &d),
                [I, O, I],
            ),
            (p(5, 0, 2), [B, O, B]),
            (
                Point3::new(5.into(), 0.into(), Real::from(2) + &d),
                [O, O, O],
            ),
            (Point3::new(1.into(), 0.into(), q(3, 2)), [O, O, O]),
        ],
        4 => vec![
            (p(1, 0, 3), [I, O, I]),
            (p(-1, 0, 3), [O, O, O]),
            (p(1, 0, 0), [O, B, B]),
            (p(1, 0, 1), [B, B, I]),
            (p(0, 1, 3), [B, O, B]),
            (Point3::new(1.into(), 0.into(), Real::one() + &d), [I, O, I]),
            (Point3::new(1.into(), 0.into(), Real::one() - &d), [O, I, I]),
            (Point3::new(q(7, 2) - &d, 0.into(), 3.into()), [I, O, I]),
            (Point3::new(q(7, 2), 0.into(), 3.into()), [B, O, B]),
            (Point3::new(q(7, 2) + &d, 0.into(), 3.into()), [O, O, O]),
            (
                Point3::new(1.into(), 0.into(), Real::from(6) - &d),
                [I, O, I],
            ),
            (p(1, 0, 6), [B, O, B]),
            (
                Point3::new(1.into(), 0.into(), Real::from(6) + &d),
                [O, O, O],
            ),
        ],
        _ => unreachable!(),
    };
    rows.into_iter()
        .map(|(p, expected)| (map.transform_point3(&p).unwrap(), expected[column]))
        .collect()
}

fn locations(model: &Model, samples: &[(Point3, SolidPointLocation)]) {
    let solid = model.solids().next().unwrap();
    for (point, expected) in samples {
        assert_eq!(
            solid.classify_point(point).unwrap(),
            *expected,
            "point={point:?}"
        );
    }
}

fn check_output(
    result: &BodySet,
    volume: &Real,
    area: &Real,
    samples: &[(Point3, SolidPointLocation)],
) -> u64 {
    assert_eq!(result.len(), 1);
    let replay = seam::reload(result.model());
    for model in [result.model(), &replay] {
        measures(model, volume, area);
        locations(model, samples);
    }
    let edge = replay.edges().next().unwrap();
    let middle = ((edge.domain().start().try_real().unwrap()
        + edge.domain().end().try_real().unwrap())
        / Real::from(2))
    .unwrap();
    let split = edge.split(middle.into()).unwrap();
    let split_replay = seam::reload(split.model());
    measures(&split_replay, volume, area);
    locations(&split_replay, samples);
    let map = seam::similarity(&q(1, 1000), true);
    let transformed = split.model().transformed(&map).unwrap();
    measures(
        &seam::reload(&transformed),
        &(volume * q(1, 1_000_000_000)),
        &(area * q(1, 1_000_000)),
    );
    locations(
        &seam::reload(&transformed),
        &samples
            .iter()
            .map(|(point, expected)| (map.transform_point3(point).unwrap(), *expected))
            .collect::<Vec<_>>(),
    );
    result
        .model()
        .to_json()
        .unwrap()
        .bytes()
        .fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        })
}

pub fn workflow(kind: usize, map: &Matrix4, scale: &Real) -> u64 {
    let (half, cutter, parent_volume, parent_area) = fixture(kind);
    let half = half.transformed(map).unwrap();
    let cutter = cutter.transformed(map).unwrap();
    let scale2 = scale * scale;
    let scale3 = &scale2 * scale;
    let intersection_samples = samples(kind, 0, map);
    let difference_samples = samples(kind, 1, map);
    let parent_samples = samples(kind, 2, map);
    let (volume, area) = expected(kind, false);
    let intersection = boolean::intersection(&half, &cutter)
        .unwrap_or_else(|e| panic!("kind={kind} intersection: {e:?}"));
    let mut receipt = check_output(
        &intersection,
        &(volume.clone() * &scale3),
        &(area.clone() * &scale2),
        &intersection_samples,
    );
    let swapped = boolean::intersection(&cutter, &half)
        .unwrap_or_else(|e| panic!("kind={kind} swapped: {e:?}"));
    receipt = receipt.wrapping_mul(0x100000001b3)
        ^ check_output(
            &swapped,
            &(volume * &scale3),
            &(area * &scale2),
            &intersection_samples,
        );
    let difference = boolean::difference(&half, &cutter)
        .unwrap_or_else(|e| panic!("kind={kind} difference: {e:?}"));
    let (volume, area) = expected(kind, true);
    receipt = receipt.wrapping_mul(0x100000001b3)
        ^ check_output(
            &difference,
            &(volume * &scale3),
            &(area * &scale2),
            &difference_samples,
        );
    let first = seam::reload(intersection.model());
    let second = seam::reload(difference.model());
    let first = BodySet::from_solid(first.solids().next().unwrap());
    let second = BodySet::from_solid(second.solids().next().unwrap());
    for (a, b) in [(&first, &second), (&second, &first)] {
        let restored =
            boolean::union(a, b).unwrap_or_else(|e| panic!("kind={kind} reconstruct: {e:?}"));
        receipt = receipt.wrapping_mul(0x100000001b3)
            ^ check_output(
                &restored,
                &(parent_volume.clone() * &scale3),
                &(parent_area.clone() * &scale2),
                &parent_samples,
            );
    }
    let empty = boolean::difference(&intersection, &cutter).unwrap();
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    receipt
}

pub fn original(kind: usize) -> u64 {
    workflow(kind, &Matrix4::identity(), &Real::one())
}

pub fn transforms() -> (usize, u64) {
    let mut count = 0;
    let mut receipt = 0xcbf29ce484222325_u64;
    for scale in [q(1, 1000), q(127, 5000), Real::from(2).sqrt().unwrap()] {
        for reflected in [false, true] {
            for kind in 0..5 {
                receipt = receipt.wrapping_mul(0x100000001b3)
                    ^ workflow(kind, &seam::similarity(&scale, reflected), &scale);
                count += 1;
            }
        }
    }
    (count, receipt)
}

pub fn annular_partition() -> u64 {
    use hyperbrep::{Curve3, Vector3};
    use hypercurve::{CircularArc2, Curve2, CurvePath2, Point2};
    let circle = |radius: i32| {
        let points = [(radius, 0), (0, radius), (-radius, 0), (0, -radius)];
        CurvePath2::try_new(
            (0..4)
                .map(|i| {
                    let a = points[i];
                    let b = points[(i + 1) % 4];
                    Curve2::from(
                        CircularArc2::try_from_center(
                            Point2::new(a.0.into(), a.1.into()),
                            Point2::new(b.0.into(), b.1.into()),
                            Point2::new(0.into(), 0.into()),
                            false,
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap()
    };
    let source = builder::planar_face(
        &circle(5),
        &[circle(2)],
        p(0, 0, 0),
        Vector3::x(),
        Vector3::y(),
    )
    .unwrap();
    let original = source.model().to_json().unwrap();
    let cuts = vec![
        Curve3::line(p(-5, 0, 0), p(-2, 0, 0)).unwrap(),
        Curve3::line(p(2, 0, 0), p(5, 0, 0)).unwrap(),
    ];
    let mut canonical = None;
    for cuts in [
        cuts.clone(),
        cuts.iter().rev().map(|c| c.reversed().unwrap()).collect(),
    ] {
        let face = source.model().faces().next().unwrap();
        let result = face.split_by_curves(&cuts).unwrap();
        assert_eq!(result.details().faces().len(), 2);
        assert_eq!(result.details().traces().len(), 2);
        assert_eq!(result.source_model().to_json().unwrap(), original);
        let json = result.model().to_json().unwrap();
        if let Some(expected) = &canonical {
            assert_eq!(&json, expected);
        }
        canonical = Some(json.clone());
        for trace in result.details().traces() {
            assert!(trace.source_index() < cuts.len());
            assert_eq!(trace.segments().len(), 1);
            let actual = &trace.segments()[0];
            let expected = &cuts[trace.source_index()];
            let eq = |a, b| {
                hyperlimit::point3_equal(&a, &b, hyperbrep::STRICT_PREDICATES).value() == Some(true)
            };
            assert!(
                (eq(actual.start().unwrap(), expected.start().unwrap())
                    && eq(actual.end().unwrap(), expected.end().unwrap()))
                    || (eq(actual.start().unwrap(), expected.end().unwrap())
                        && eq(actual.end().unwrap(), expected.start().unwrap()))
            );
            assert_eq!(trace.splits().len(), 1);
            for split in trace.splits() {
                assert_eq!(split.model().to_json().unwrap(), json);
                assert_eq!(split.source_model().to_json().unwrap(), original);
            }
        }
        let replay = seam::reload(result.model());
        for face in replay.faces() {
            equal(&face.area().unwrap(), &(q(21, 2) * Real::pi()));
        }
        assert_eq!(replay.solids().count(), 0);
        let transformed = seam::reload(
            &replay
                .transformed(&seam::similarity(&q(127, 5000), true))
                .unwrap(),
        );
        for face in transformed.faces() {
            equal(
                &face.area().unwrap(),
                &(q(21, 2) * Real::pi() * q(127, 5000) * q(127, 5000)),
            );
        }
    }
    assert_eq!(source.model().to_json().unwrap(), original);
    canonical
        .unwrap()
        .bytes()
        .fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        })
}
