//! Full-size cone compositions checked against independently integrated sets.
use super::seam::{equal, point as p, ratio as q};
use hyperbrep::{BodySet, Matrix4, Model, Point3, Real, SolidPointLocation, boolean, builder};

#[derive(Clone, Copy, Debug)]
enum Part {
    Whole,
    Positive,
    Negative,
    Band,
    Lower,
}
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
fn fixture() -> (BodySet, BodySet) {
    let source = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let cutter = builder::cuboid(p(0, -20, -20), p(20, 20, 20))
        .unwrap()
        .transformed(&rotation())
        .unwrap();
    (source, cutter)
}
fn body(model: &Model) -> BodySet {
    assert_eq!(model.solids().count(), 1);
    BodySet::from_solid(model.solids().next().unwrap())
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
fn measures(model: &Model, part: Part) {
    let pi = Real::pi();
    let root = Real::from(5).sqrt().unwrap();
    // r(z)=5-z/2. Integrate pi*r(z)^2 over the relevant z interval;
    // lateral slant per unit z is sqrt(5)/2. A half adds its planar cut.
    let (volume, area) = match part {
        Part::Whole => (
            Real::from(78) * &pi,
            (Real::from(29) + Real::from(21) * &root) * &pi,
        ),
        Part::Positive | Part::Negative => (
            Real::from(39) * &pi,
            (Real::from(29) + Real::from(21) * &root) * &pi * q(1, 2) + Real::from(42),
        ),
        Part::Band => (
            q(665, 24) * &pi,
            (Real::from(97) + Real::from(65) * &root) * &pi * q(1, 8) + q(65, 2),
        ),
        Part::Lower => (
            q(271, 24) * &pi,
            (Real::from(181) + Real::from(19) * &root) * &pi * q(1, 8) + q(19, 2),
        ),
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
fn locations(model: &Model, part: Part) {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    let (low, high) = match part {
        Part::Band => (1, 6),
        Part::Lower => (0, 1),
        _ => (0, 6),
    };
    let z = q(low + high, 2);
    let radius = Real::from(5) - &z * q(1, 2);
    let positive = if matches!(part, Part::Negative) { O } else { I };
    let negative = if matches!(part, Part::Whole | Part::Negative) {
        I
    } else {
        O
    };
    let seam = if matches!(part, Part::Whole) { I } else { B };
    let positive_boundary = if matches!(part, Part::Negative) { O } else { B };
    let at = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let rows = [
        (at(1.into(), 0.into(), z.clone()), positive),
        (at((-1).into(), 0.into(), z.clone()), negative),
        (at(0.into(), 1.into(), z.clone()), seam),
        (at(delta.clone(), 1.into(), z.clone()), positive),
        (at(-&delta, 1.into(), z.clone()), negative),
        (at(&radius - &delta, 0.into(), z.clone()), positive),
        (at(radius.clone(), 0.into(), z.clone()), positive_boundary),
        (at(&radius + &delta, 0.into(), z.clone()), O),
        (at(-&radius + &delta, 0.into(), z.clone()), negative),
        (at(-&radius - &delta, 0.into(), z), O),
        (at(1.into(), 0.into(), Real::from(low) - &delta), O),
        (p(1, 0, low), positive_boundary),
        (at(1.into(), 0.into(), Real::from(low) + &delta), positive),
        (at(1.into(), 0.into(), Real::from(high) - &delta), positive),
        (p(1, 0, high), positive_boundary),
        (at(1.into(), 0.into(), Real::from(high) + &delta), O),
    ];
    let solid = model.solids().next().unwrap();
    for (point, expected) in rows {
        let point = rotation().transform_point3(&point).unwrap();
        assert_eq!(
            solid.classify_point(&point).unwrap(),
            expected,
            "part={part:?}, point={point:?}"
        );
    }
}
fn check(result: &BodySet, part: Part, context: &str) -> u64 {
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
fn split(result: &BodySet, context: &str) -> BodySet {
    let replay = reload(result.model(), context);
    let edge = replay.edges().next().unwrap();
    let middle = ((edge.domain().start().try_real().unwrap()
        + edge.domain().end().try_real().unwrap())
        / Real::from(2))
    .unwrap();
    let split = edge.split(middle.into()).unwrap();
    body(&reload(split.model(), context))
}
fn accumulate(receipt: &mut u64, value: u64) {
    *receipt = receipt.wrapping_mul(0x100000001b3) ^ value;
}
fn recover_empty(empty: &BodySet, result: &BodySet, part: Part, receipt: &mut u64) {
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    accumulate(
        receipt,
        check(
            &boolean::union(empty, result).unwrap(),
            part,
            "empty recovery",
        ),
    );
}
pub fn reconstruction(swapped: bool) -> u64 {
    let (source, cutter) = fixture();
    let half = boolean::intersection(&source, &cutter).unwrap();
    let other = boolean::difference(&source, &cutter).unwrap();
    let union = |a: &BodySet, b: &BodySet, context: &str| {
        if swapped {
            boolean::union(b, a)
        } else {
            boolean::union(a, b)
        }
        .unwrap_or_else(|error| panic!("{context}: {error:?}"))
    };
    let mut receipt = check(
        &union(&half, &other, "initial union"),
        Part::Whole,
        "initial reconstruction",
    );
    let cold_half = body(&reload(half.model(), "cold half"));
    let cold_other = body(&reload(other.model(), "cold other"));
    let mut current = union(&cold_half, &cold_other, "cold union");
    accumulate(
        &mut receipt,
        check(&current, Part::Whole, "cold reconstruction"),
    );
    for cycle in 0..2 {
        let split = split(&current, &format!("reconstruction cycle {cycle}"));
        accumulate(&mut receipt, check(&split, Part::Whole, "split"));
        let recut = boolean::intersection(&split, &cutter).unwrap();
        accumulate(&mut receipt, check(&recut, Part::Positive, "recut"));
        let remainder = boolean::difference(&split, &cutter).unwrap();
        accumulate(&mut receipt, check(&remainder, Part::Negative, "remainder"));
        current = union(&recut, &remainder, &format!("cycle {cycle} union"));
        accumulate(&mut receipt, check(&current, Part::Whole, "reconstructed"));
        recover_empty(
            &boolean::difference(&recut, &cutter).unwrap(),
            &current,
            Part::Whole,
            &mut receipt,
        );
    }
    receipt
}
pub fn band() -> u64 {
    let (source, cutter) = fixture();
    let half = boolean::intersection(&source, &cutter).unwrap();
    let axial = builder::cuboid(p(-20, -20, 1), p(20, 20, 20)).unwrap();
    let band = boolean::intersection(&half, &axial).unwrap();
    let mut receipt = check(&band, Part::Band, "initial band");
    let cold_half = body(&reload(half.model(), "cold half"));
    let mut current = boolean::intersection(&cold_half, &axial).unwrap();
    accumulate(&mut receipt, check(&current, Part::Band, "cold band"));
    let lower = boolean::difference(&cold_half, &axial).unwrap();
    accumulate(&mut receipt, check(&lower, Part::Lower, "lower half"));
    for cycle in 0..2 {
        current = split(&current, &format!("band cycle {cycle}"));
        accumulate(&mut receipt, check(&current, Part::Band, "split band"));
        current = boolean::intersection(&current, &axial).unwrap();
        accumulate(&mut receipt, check(&current, Part::Band, "recut band"));
        let restored = if cycle == 0 {
            boolean::union(&current, &lower)
        } else {
            boolean::union(&lower, &current)
        }
        .unwrap();
        accumulate(
            &mut receipt,
            check(&restored, Part::Positive, "band reconstruction"),
        );
        current = boolean::intersection(&restored, &axial).unwrap();
        accumulate(
            &mut receipt,
            check(&current, Part::Band, "band recut after reconstruction"),
        );
        recover_empty(
            &boolean::difference(&current, &axial).unwrap(),
            &current,
            Part::Band,
            &mut receipt,
        );
    }
    receipt
}

pub fn original_partition() {
    let json = include_str!("../fixtures/cone-rational-partition.json");
    let model = hyperbrep::RawModel::from_json(json, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), json);
}

pub fn captured_circle_query() {
    let raw: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/cone-circle-point-query.json")).unwrap();
    let point = |name: &str| {
        let [x, y]: [hyperbrep::Real; 2] = serde_json::from_value(raw[name].clone()).unwrap();
        hypercurve::Point2::new(x, y)
    };
    let arc = hypercurve::CircularArc2::try_from_center(
        point("start"),
        point("end"),
        point("center"),
        raw["clockwise"].as_bool().unwrap(),
    )
    .unwrap();
    // This clockwise arc runs from (-4,3) to (-sqrt(5),2*sqrt(5)).
    // It stays in the upper-left quadrant, so (5,0) is on its circle only.
    let curve = hypercurve::Curve2::from(arc);
    let hypercurve::CurvePointLocations2::Locations(locations) =
        curve.point_locations(&point("point").into()).unwrap()
    else {
        panic!("finite arc");
    };
    assert!(locations.is_empty());
}

pub fn overlap_contacts() {
    use hypercurve::{
        CircularArc2, Contour2, Curve2, CurvePointLocations2, CurveRegion2, LineSeg2, Point2,
        Segment2,
    };
    let raw: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/cone-circle-point-query.json")).unwrap();
    let point = |name: &str| {
        let [x, y]: [hyperbrep::Real; 2] = serde_json::from_value(raw[name].clone()).unwrap();
        Point2::new(x, y)
    };
    let boundary =
        CircularArc2::try_from_center(point("end"), point("start"), point("center"), false)
            .unwrap();
    let contour = Contour2::try_new(vec![
        Segment2::Arc(boundary.clone()),
        Segment2::Line(LineSeg2::try_new(point("start"), point("center")).unwrap()),
        Segment2::Line(LineSeg2::try_new(point("center"), point("end")).unwrap()),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_native_contours(vec![contour], Vec::new()).unwrap();
    for reversed in [false, true] {
        let mut source = Curve2::from(
            CircularArc2::try_from_center(
                Point2::from_values(-5, 0),
                Point2::from_values(0, 5),
                point("center"),
                true,
            )
            .unwrap(),
        );
        if reversed {
            source = source.reversed().unwrap();
        }
        let fragments = source.trim_inside_region_with_parameters(&region).unwrap();
        assert_eq!(fragments.len(), 1);
        let mut arc_contacts = 0;
        for contacts in [
            fragments[0].start_boundary_contacts(),
            fragments[0].end_boundary_contacts(),
        ] {
            for contact in contacts {
                let carrier = contact.carrier().curve();
                let start_point = carrier.start();
                let end_point = carrier.end();
                let a: hypercurve::CurvePoint2 = boundary.start().clone().into();
                let b: hypercurve::CurvePoint2 = boundary.end().clone().into();
                // Prepared carriers may promote the arc to a rational chart.
                // This sector's only boundary joining A and B is its arc.
                if !((start_point == a && end_point == b) || (start_point == b && end_point == a)) {
                    continue;
                }
                let (start, end) = carrier.parameter_endpoints();
                let expected = if contact.boundary_parameter() == start {
                    carrier.start()
                } else {
                    assert_eq!(contact.boundary_parameter(), end);
                    carrier.end()
                };
                assert_eq!(
                    contact.point(),
                    Some(&expected),
                    "retain the already owned endpoint image"
                );
                let CurvePointLocations2::Locations(locations) = Curve2::from(boundary.clone())
                    .point_locations(contact.point().unwrap())
                    .unwrap()
                else {
                    panic!("finite arc");
                };
                assert_eq!(locations.len(), 1);
                arc_contacts += 1;
            }
        }
        assert!(arc_contacts >= 2);
    }
}
