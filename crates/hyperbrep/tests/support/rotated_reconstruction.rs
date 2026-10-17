//! Full-size rationally rotated torus cuts, with independent set expectations.
use super::seam::{equal, point as p, ratio as q};
use hyperbrep::{BodySet, Matrix4, Model, Point3, Real, SolidPointLocation, boolean, builder};

fn rotation() -> Matrix4 {
    Matrix4::from_row_major([
        q(3, 5),
        -q(4, 5),
        0.into(),
        0.into(),
        q(4, 5),
        q(3, 5),
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

fn body(model: &Model) -> BodySet {
    assert_eq!(model.solids().count(), 1);
    BodySet::from_solid(model.solids().next().unwrap())
}

fn measures(model: &Model, part: i32) {
    let pi = Real::pi();
    let full = Real::from(40) * &pi * &pi;
    let (volume, area) = if part == 0 {
        (full.clone(), full)
    } else {
        let half = full * q(1, 2);
        (half.clone(), half + Real::from(8) * pi)
    };
    assert_eq!(model.solids().count(), 1);
    equal(&model.solids().next().unwrap().volume().unwrap(), &volume);
    equal(
        &model
            .faces()
            .map(|face| face.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        &area,
    );
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

fn locations(model: &Model, part: i32) {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let d = q(1, 1_000_000) * q(1, 1_000_000);
    let positive = if part < 0 { O } else { I };
    let negative = if part > 0 { O } else { I };
    let seam = if part == 0 { I } else { B };
    let radial = |radius: Real| Point3::new(radius, 0.into(), 0.into());
    // Analytic torus (sqrt(x²+y²)-5)²+z² <= 4, intersected with
    // x>=0 or x<=0 before this exact rotation. Cut seams are interior
    // in the reconstructed whole; the central hole remains exterior.
    let rows = [
        (p(5, 0, 0), positive),
        (p(-5, 0, 0), negative),
        (p(0, 5, 0), seam),
        (p(0, 0, 0), O),
        (Point3::new(d.clone(), 5.into(), 0.into()), positive),
        (Point3::new(-&d, 5.into(), 0.into()), negative),
        (radial(Real::from(7) - &d), positive),
        (radial(7.into()), if part < 0 { O } else { B }),
        (radial(Real::from(7) + &d), O),
        (radial(Real::from(3) - &d), O),
        (radial(3.into()), if part < 0 { O } else { B }),
        (radial(Real::from(3) + &d), positive),
        (
            Point3::new(5.into(), 0.into(), Real::from(2) - &d),
            positive,
        ),
        (p(5, 0, 2), if part < 0 { O } else { B }),
        (Point3::new(5.into(), 0.into(), Real::from(2) + &d), O),
    ];
    let solid = model.solids().next().unwrap();
    for (point, expected) in rows {
        let point = rotation().transform_point3(&point).unwrap();
        assert_eq!(
            solid.classify_point(&point).unwrap(),
            expected,
            "part={part}, point={point:?}"
        );
    }
}

fn reload(model: &Model, context: &str) -> Model {
    let json = model.to_json().unwrap();
    let replay = hyperbrep::RawModel::from_json(&json, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap_or_else(|error| panic!("{context}: {error:?}"));
    assert_eq!(replay.to_json().unwrap(), json);
    replay
}

fn check(result: &BodySet, part: i32, context: &str) -> u64 {
    // Replay before numeric queries: no predicate on the live object may warm
    // the independently deserialized archive's scalar proof state.
    let replay = reload(result.model(), context);
    for model in [&replay, result.model()] {
        measures(model, part);
        locations(model, part);
    }
    result
        .model()
        .to_json()
        .unwrap()
        .bytes()
        .fold(0xcbf29ce484222325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        })
}

pub fn workflow(swapped: bool) -> u64 {
    let source = builder::torus(5.into(), 2.into()).unwrap();
    let cutter = builder::cuboid(p(0, -20, -20), p(20, 20, 20))
        .unwrap()
        .transformed(&rotation())
        .unwrap();
    let half = boolean::intersection(&source, &cutter).unwrap();
    let other = boolean::difference(&source, &cutter).unwrap();
    let union = |a: &BodySet, b: &BodySet| {
        if swapped {
            boolean::union(b, a)
        } else {
            boolean::union(a, b)
        }
        .unwrap()
    };
    let restored = union(&half, &other);
    let mut receipt = check(&restored, 0, "initial reconstruction");
    let cold_half = body(&reload(half.model(), "cold half"));
    let cold_other = body(&reload(other.model(), "cold other"));
    let cold_restored = union(&cold_half, &cold_other);
    receipt = receipt.wrapping_mul(0x100000001b3) ^ check(&cold_restored, 0, "cold reconstruction");

    let mut current = cold_restored;
    for cycle in 0..2 {
        let replay = reload(current.model(), &format!("cycle {cycle} input"));
        let edge = replay.edges().next().unwrap();
        let middle = ((edge.domain().start().try_real().unwrap()
            + edge.domain().end().try_real().unwrap())
            / Real::from(2))
        .unwrap();
        let split = edge.split(middle.into()).unwrap();
        let split = body(&reload(split.model(), &format!("cycle {cycle} split")));
        receipt = receipt.wrapping_mul(0x100000001b3)
            ^ check(&split, 0, &format!("cycle {cycle} split result"));
        let recut = boolean::intersection(&split, &cutter).unwrap();
        receipt =
            receipt.wrapping_mul(0x100000001b3) ^ check(&recut, 1, &format!("cycle {cycle} recut"));
        let remainder = boolean::difference(&split, &cutter).unwrap();
        receipt = receipt.wrapping_mul(0x100000001b3)
            ^ check(&remainder, -1, &format!("cycle {cycle} remainder"));
        current = union(&recut, &remainder);
        receipt = receipt.wrapping_mul(0x100000001b3)
            ^ check(&current, 0, &format!("cycle {cycle} reconstructed"));
        let empty = boolean::difference(&recut, &cutter).unwrap();
        assert_eq!(
            empty.model().to_json().unwrap(),
            BodySet::empty().model().to_json().unwrap()
        );
        let recovered = boolean::union(&empty, &current).unwrap();
        receipt = receipt.wrapping_mul(0x100000001b3)
            ^ check(&recovered, 0, &format!("cycle {cycle} empty recovery"));
    }
    receipt
}
