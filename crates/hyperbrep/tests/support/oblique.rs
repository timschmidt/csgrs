//! Relative oblique cuts and distant conic charts; exact independent expectations.
use hyperbrep::{
    ArchiveLimits, BodySet, Curve3, CurveParameterLocation, CurveSurfaceIntersection, Matrix4,
    Model, Point3, RawModel, Real, SolidPointLocation, Surface, Vector3, boolean, builder,
};

#[allow(dead_code)]
#[path = "seam_cuts.rs"]
mod seam;
pub use seam::{equal, point as p, ratio as q, similarity};

fn inverse(curve: &Curve3, point: &Point3, expected: &[Real]) {
    let result = curve.parameters_of(point).unwrap();
    if expected.is_empty() {
        assert!(matches!(result, CurveParameterLocation::None));
    } else {
        let CurveParameterLocation::Parameters(parameters) = result else {
            panic!("missing conic inverse")
        };
        assert_eq!(parameters.len(), expected.len());
        for (actual, expected) in parameters.iter().zip(expected) {
            equal(actual.try_real().unwrap(), expected);
            assert!(
                curve
                    .point_at(actual)
                    .unwrap()
                    .coincides_with(&point.clone().into())
                    .unwrap()
            );
        }
    }
}

pub fn conic_queries() {
    let third = (Real::pi() / Real::from(3)).unwrap();
    let sixth = (Real::pi() / Real::from(6)).unwrap();
    let tiny = q(1, 1_000_000) * q(1, 1_000_000);
    for turn in [-1_000_000_000_000_i64, -7, -1, 0, 1, 7, 1_000_000_000_000] {
        let start = Real::from(turn) * Real::tau();
        for ellipse in [false, true] {
            let rx = Real::from(if ellipse { 3 } else { 2 });
            let curve = if ellipse {
                Curve3::ellipse_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    rx.clone(),
                    2.into(),
                    start.clone(),
                    &start + Real::pi(),
                )
            } else {
                Curve3::circle_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    rx.clone(),
                    start.clone(),
                    &start + Real::pi(),
                )
            }
            .unwrap();
            let point = Point3::new(&rx * q(1, 2), Real::from(3).sqrt().unwrap(), 0.into());
            let plane = Surface::plane(
                Point3::new(&rx * q(1, 2), 0.into(), 0.into()),
                Vector3::y(),
                Vector3::z(),
            )
            .unwrap();
            for reverse in [false, true] {
                let curve = if reverse {
                    curve.reversed().unwrap()
                } else {
                    curve.clone()
                };
                let parameter = &start + &third * Real::from(if reverse { 2 } else { 1 });
                for curve in [
                    curve.clone(),
                    curve
                        .subcurve(&(&start + &sixth), &(&start + &sixth * Real::from(5)))
                        .unwrap(),
                ] {
                    inverse(&curve, &point, std::slice::from_ref(&parameter));
                    inverse(&curve, &p(0, -2, 0), &[]);
                    inverse(&curve, &(point.clone() + Vector3::z()), &[]);
                    let CurveSurfaceIntersection::Points(points) =
                        plane.intersect_curve(&curve).unwrap()
                    else {
                        panic!(
                            "missing plane contact: turn={turn} ellipse={ellipse} reverse={reverse}"
                        )
                    };
                    assert_eq!(points.len(), 1);
                    equal(points[0].parameter.try_real().unwrap(), &parameter);
                    assert!(
                        points[0]
                            .point
                            .clone()
                            .coincides_with(&point.clone().into())
                            .unwrap()
                    );
                }
            }
            let full = if ellipse {
                Curve3::ellipse_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    rx.clone(),
                    2.into(),
                    start.clone(),
                    &start + Real::tau(),
                )
            } else {
                Curve3::circle_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    rx.clone(),
                    start.clone(),
                    &start + Real::tau(),
                )
            }
            .unwrap();
            let seam = Point3::new(rx.clone(), 0.into(), 0.into());
            for curve in [full.clone(), full.reversed().unwrap()] {
                inverse(&curve, &seam, &[start.clone(), &start + Real::tau()]);
            }
            let gap = full
                .subcurve(&(&start + &tiny), &(&start + Real::pi()))
                .unwrap();
            inverse(&gap, &seam, &[]);
        }
    }
}

fn rotation() -> Matrix4 {
    let r = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    Matrix4::from_row_major([
        r.clone(),
        -&r,
        0.into(),
        0.into(),
        r.clone(),
        r,
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

fn check(
    result: &BodySet,
    volume: &Real,
    area: &Real,
    samples: &[(Point3, SolidPointLocation)],
) -> u64 {
    assert_eq!(result.len(), 1);
    let json = result.model().to_json().unwrap();
    let replay = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), json);
    let verify = |model: &Model, v: &Real, a: &Real, samples: &[(Point3, SolidPointLocation)]| {
        measures(model, v, a);
        for (p, expected) in samples {
            assert_eq!(
                model.solids().next().unwrap().classify_point(p).unwrap(),
                *expected
            );
        }
    };
    verify(result.model(), volume, area, samples);
    verify(&replay, volume, area, samples);
    let edge = replay.edges().next().unwrap();
    let middle = ((edge.domain().start().try_real().unwrap()
        + edge.domain().end().try_real().unwrap())
        / Real::from(2))
    .unwrap();
    let split = edge.split(middle.into()).unwrap();
    verify(&seam::reload(split.model()), volume, area, samples);
    let map = similarity(&q(1, 1000), true);
    let transformed = split.model().transformed(&map).unwrap();
    let transformed_samples = samples
        .iter()
        .map(|(p, where_)| (map.transform_point3(p).unwrap(), *where_))
        .collect::<Vec<_>>();
    verify(
        &seam::reload(&transformed),
        &(volume * q(1, 1_000_000_000)),
        &(area * q(1, 1_000_000)),
        &transformed_samples,
    );
    json.bytes().fold(0xcbf29ce484222325_u64, |h, b| {
        (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
    })
}

pub fn torus_band_operands(map: &Matrix4) -> (BodySet, BodySet) {
    let source = builder::torus(5.into(), 2.into()).unwrap();
    let cutter = builder::cuboid(p(0, -20, -20), p(20, 20, 20))
        .unwrap()
        .transformed(&rotation())
        .unwrap();
    let half = boolean::intersection(&source, &cutter)
        .unwrap()
        .transformed(map)
        .unwrap();
    let band = builder::cuboid(p(-20, -20, 1), p(20, 20, 20))
        .unwrap()
        .transformed(map)
        .unwrap();
    (half, band)
}

pub fn torus_band_workflow(map: &Matrix4, scale: &Real) -> u64 {
    let (half, band) = torus_band_operands(map);
    let pi = Real::pi();
    let pi2 = &pi * &pi;
    let sqrt3 = Real::from(3).sqrt().unwrap();
    let volumes = [
        Real::from(5) * &pi * (q(4, 3) * &pi - &sqrt3),
        Real::from(5) * &pi * (q(8, 3) * &pi + &sqrt3),
        Real::from(20) * &pi2,
    ];
    let areas = [
        q(20, 3) * &pi2 + Real::from(10) * &pi * &sqrt3 + q(8, 3) * &pi - Real::from(2) * &sqrt3,
        q(40, 3) * &pi2 + Real::from(10) * &pi * &sqrt3 + q(16, 3) * &pi + Real::from(2) * &sqrt3,
        Real::from(20) * &pi2 + Real::from(8) * &pi,
    ];
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let d = q(1, 1_000_000) * q(1, 1_000_000);
    let rows = vec![
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
    ];
    let samples = |column| {
        rows.iter()
            .map(|(p, cases)| {
                (
                    map.transform_point3(&rotation().transform_point3(p).unwrap())
                        .unwrap(),
                    cases[column],
                )
            })
            .collect::<Vec<_>>()
    };
    let scale2 = scale * scale;
    let scale3 = &scale2 * scale;
    let validate = |body: &BodySet, column| {
        check(
            body,
            &(&volumes[column] * &scale3),
            &(&areas[column] * &scale2),
            &samples(column),
        )
    };
    let mut receipt = validate(&half, 2);
    let intersection = boolean::intersection(&half, &band).unwrap();
    let difference = boolean::difference(&half, &band).unwrap();
    for (result, column) in [(&intersection, 0), (&difference, 1)] {
        receipt = receipt.wrapping_mul(0x100000001b3) ^ validate(result, column);
    }
    receipt = receipt.wrapping_mul(0x100000001b3)
        ^ validate(&boolean::intersection(&band, &half).unwrap(), 0);
    let intersection =
        BodySet::from_solid(seam::reload(intersection.model()).solids().next().unwrap());
    let difference = BodySet::from_solid(seam::reload(difference.model()).solids().next().unwrap());
    for (a, b) in [(&intersection, &difference), (&difference, &intersection)] {
        receipt = receipt.wrapping_mul(0x100000001b3) ^ validate(&boolean::union(a, b).unwrap(), 2);
    }
    let empty = boolean::difference(&intersection, &band).unwrap();
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    receipt
}
