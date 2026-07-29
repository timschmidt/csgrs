#![no_main]

use hyperbrep::{Curve3, Direction, Model, RawModel, Real, builder};
use libfuzzer_sys::fuzz_target;

fn directed_start(model: &Model, edge_use: hyperbrep::EdgeUseId) -> hyperbrep::VertexId {
    let edge_use = model.edge_use(edge_use).expect("validated edge use");
    let edge = model.edge(edge_use.edge()).expect("validated edge");
    match edge_use.direction() {
        Direction::Forward => edge.start(),
        Direction::Reversed => edge.end(),
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }
    let positive = |index: usize| Real::from(i32::from(bytes[index]) + 1);
    if bytes[0] == 0xa5 {
        let radius = positive(1);
        let half_radius = (radius.clone() / Real::from(2)).expect("two is nonzero");
        let twice_radius = &radius * Real::from(2);
        let (sphere, sphere_solid) =
            builder::sphere(radius).expect("positive sphere is constructible");
        let (box_model, box_solid) = builder::cuboid(
            hyperbrep::Point3::new(half_radius.clone(), half_radius, -twice_radius.clone()),
            hyperbrep::Point3::new(twice_radius.clone(), twice_radius.clone(), twice_radius),
        )
        .expect("scaled clipping box is constructible");
        let graph =
            hyperbrep::boolean::intersection_graph(&sphere, sphere_solid, &box_model, box_solid)
                .expect("analytic graph construction is exact");
        let (edited, _) = graph
            .partition_second_faces()
            .expect("planar conic graph fragments are transferable");
        let expected = box_model
            .solid_volume(box_solid)
            .expect("source box has certified volume");
        assert_eq!(
            hyperlimit::compare_reals(
                &edited
                    .solid_volume(box_solid)
                    .expect("partitioned box keeps its certificate"),
                &expected,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = edited.to_json().expect("partitioned model serializes");
        let decoded = RawModel::from_json(&json)
            .expect("partitioned JSON decodes")
            .validate()
            .expect("partitioned JSON fully revalidates");
        assert_eq!(
            hyperlimit::compare_reals(
                &decoded
                    .solid_volume(box_solid)
                    .expect("decoded box keeps its certificate"),
                &expected,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        return;
    }
    let use_cylinder = bytes[0] & 2 != 0;
    let split_face = bytes[0] & 1 != 0;
    let (model, solid) = if use_cylinder {
        builder::cylinder(positive(1), positive(2)).expect("positive cylinder is constructible")
    } else {
        builder::cuboid(
            hyperbrep::Point3::origin(),
            hyperbrep::Point3::new(positive(1), positive(2), positive(3)),
        )
        .expect("positive cuboid is constructible")
    };

    let edited = if split_face {
        let (face, record) = model.faces().nth(1).expect("primitive has a top cap");
        let outer = record.outer().expect("top cap is trimmed");
        let uses = model.wire(outer).expect("validated outer wire").edge_uses();
        if bytes[0] & 4 == 0 {
            let start = directed_start(&model, uses[0]);
            let end = directed_start(&model, uses[uses.len() / 2]);
            model
                .split_face(face, start, end)
                .expect("primitive cap diagonal is an exact valid split")
                .0
        } else {
            let directed_point = |use_id, numerator: i32, denominator: i32| {
                let edge_use = model.edge_use(use_id).expect("validated use");
                let edge = model.edge(edge_use.edge()).expect("validated edge");
                let fraction = (Real::from(numerator) / Real::from(denominator))
                    .expect("positive denominator is nonzero");
                let span = edge.domain().end() - edge.domain().start();
                let offset = span * fraction;
                let parameter = match edge_use.direction() {
                    Direction::Forward => edge.domain().start() + offset,
                    Direction::Reversed => edge.domain().end() - offset,
                };
                model
                    .curve(edge.curve())
                    .expect("validated curve")
                    .point_at(&parameter)
                    .expect("edge fraction evaluates")
            };
            let opposite = uses[uses.len() / 2];
            if bytes[0] & 8 == 0 {
                let fragment = Curve3::line(
                    directed_point(uses[0], 1, 2),
                    directed_point(opposite, 1, 2),
                )
                .expect("opposite edge midpoints differ");
                model
                    .split_face_by_curve(face, &fragment)
                    .expect("exact cap trace attaches to both boundary edges")
                    .0
            } else {
                let diagonal = |first_use, second_use| {
                    let first = model
                        .vertex(directed_start(&model, first_use))
                        .expect("validated boundary vertex")
                        .point()
                        .clone();
                    let second = model
                        .vertex(directed_start(&model, second_use))
                        .expect("validated boundary vertex")
                        .point()
                        .clone();
                    Curve3::line(first, second).expect("opposite vertices differ")
                };
                let traces = match bytes[3] & 3 {
                    0 => vec![diagonal(uses[1], uses[3]), diagonal(uses[0], opposite)],
                    1 => vec![
                        diagonal(uses[1], uses[3]),
                        Curve3::line(
                            directed_point(uses[0], 1, 2),
                            directed_point(opposite, 1, 2),
                        )
                        .expect("opposite edge midpoints differ"),
                        diagonal(uses[0], opposite),
                    ],
                    _ => {
                        let trace = |numerator| {
                            Curve3::line(
                                directed_point(uses[0], numerator, 3),
                                directed_point(opposite, 3 - numerator, 3),
                            )
                            .expect("opposite edge fractions differ")
                        };
                        vec![trace(2), trace(1)]
                    }
                };
                model
                    .split_face_by_curves(face, &traces)
                    .expect("exact cap trace arrangement partitions deterministically")
                    .0
            }
        }
    } else {
        let (edge_id, edge) = model.edges().next().expect("primitive has edges");
        let midpoint = ((edge.domain().start() + edge.domain().end()) / Real::from(2))
            .expect("two is nonzero");
        model
            .split_edge(edge_id, midpoint)
            .expect("primitive edge midpoint is an exact valid split")
            .0
    };

    let _ = edited
        .solid_volume(solid)
        .expect("certificate survives edit");
    let json = edited.to_json().expect("edited model serializes");
    let decoded = RawModel::from_json(&json)
        .expect("edited JSON decodes")
        .validate()
        .expect("edited JSON fully revalidates");
    let _ = decoded
        .solid_volume(solid)
        .expect("decoded certificate survives edit");
});
