#![no_main]

use hyperbrep::{
    Curve3, Direction, ModelBuilder, Orientation, ParameterCorrespondence, ParameterDomain, Pcurve,
    Point3, Real, Surface, Vector3,
};
use hypercurve::{Curve2, LineSeg2, Point2};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 6 {
        return;
    }
    let coordinate = |index: usize| Real::from(i32::from(bytes[index]) - 128);
    let points = [
        Point3::new(coordinate(0), coordinate(1), Real::zero()),
        Point3::new(coordinate(2), coordinate(3), Real::zero()),
        Point3::new(coordinate(4), coordinate(5), Real::zero()),
    ];
    let parameters = points
        .iter()
        .map(|point| Point2::new(point.x.clone(), point.y.clone()))
        .collect::<Vec<_>>();
    let mut builder = ModelBuilder::new();
    let Ok(vertices) = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point))
        .collect::<Result<Vec<_>, _>>()
    else {
        return;
    };

    let mut edge_uses = Vec::new();
    for index in 0..3 {
        let next = (index + 1) % 3;
        let Ok(curve) = Curve3::line(points[index].clone(), points[next].clone()) else {
            return;
        };
        let Ok(curve) = builder.curve(curve) else {
            return;
        };
        let Ok(edge) = builder.edge(
            vertices[index],
            vertices[next],
            curve,
            ParameterDomain::unit(),
        ) else {
            return;
        };
        let Ok(line) = LineSeg2::try_new(parameters[index].clone(), parameters[next].clone())
        else {
            return;
        };
        let Ok(pcurve) = builder.pcurve(Pcurve::new(Curve2::from(line))) else {
            return;
        };
        let Ok(edge_use) = builder.edge_use(
            edge,
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::identity(),
        ) else {
            return;
        };
        edge_uses.push(edge_use);
    }
    let Ok(wire) = builder.wire(edge_uses) else {
        return;
    };
    let Ok(surface) = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()) else {
        return;
    };
    let Ok(surface) = builder.surface(surface) else {
        return;
    };
    let Ok(face) = builder.face(surface, Orientation::Forward, wire, Vec::new()) else {
        return;
    };
    let Ok(_) = builder.shell(vec![face]) else {
        return;
    };
    let _ = builder.finish();
});
