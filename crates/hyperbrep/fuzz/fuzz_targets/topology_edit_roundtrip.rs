#![no_main]

use hyperbrep::{
    Curve3, Direction, Model, RawModel, Real, Surface, SurfaceIntersectionOperand,
    SurfaceSurfaceIntersection, Vector3, builder,
};
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
    if bytes[0] == 0xa6 {
        let quarter = (Real::one() / Real::from(4)).expect("four is nonzero");
        let three_quarters = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        let outer = [
            hyperbrep::Point2::new(Real::zero(), Real::zero()),
            hyperbrep::Point2::new(Real::one(), Real::zero()),
            hyperbrep::Point2::new(Real::one(), Real::one()),
            hyperbrep::Point2::new(Real::zero(), Real::one()),
        ];
        let hole = vec![
            hyperbrep::Point2::new(quarter.clone(), quarter.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), quarter.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), three_quarters.clone()),
            hyperbrep::Point2::new(quarter.clone(), three_quarters.clone()),
        ];
        let (source, solid) = builder::extrude_region(&outer, &[hole], Real::zero(), Real::one())
            .expect("unit holed extrusion is constructible");
        let face = source
            .shell(source.solid(solid).expect("validated solid").outer())
            .expect("validated shell")
            .faces()[1];
        let surface_id = source.face(face).expect("validated cap").surface();
        let tensor_surface = Surface::rational_bezier(
            vec![
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::zero(), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::one(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::one(), Real::one()),
                ],
            ],
            vec![vec![Real::one(), Real::one()]; 2],
        )
        .expect("affine tensor surface is constructible");
        let mut edit = source.edit();
        edit.replace_surface(surface_id, tensor_surface.clone())
            .expect("affine cap replacement is valid");
        let tensor = edit.commit().expect("holed tensor cap certifies");
        let support = Surface::plane(
            hyperbrep::Point3::new(
                (Real::one() / Real::from(2)).expect("two is nonzero"),
                Real::zero(),
                Real::one(),
            ),
            Vector3::y(),
            Vector3::z(),
        )
        .expect("boundary support plane is constructible");
        let SurfaceSurfaceIntersection::Curve(trace) = tensor_surface
            .intersect_surface(&support)
            .expect("affine tensor section is exact")
        else {
            panic!("affine tensor section retains one curve");
        };
        let mut traces = vec![
            trace
                .subcurve(&Real::zero(), &quarter)
                .expect("lower material bridge"),
            trace
                .subcurve(&three_quarters, &Real::one())
                .expect("upper material bridge"),
        ];
        if bytes[1] & 1 != 0 {
            traces.reverse();
            traces = traces
                .into_iter()
                .map(|trace| trace.reversed().expect("exact trace reversal"))
                .collect();
        }
        let operand = if bytes[2] & 1 == 0 {
            SurfaceIntersectionOperand::First
        } else {
            SurfaceIntersectionOperand::Second
        };
        let (edited, partition) = tensor
            .split_face_by_surface_curves(face, &traces, operand)
            .expect("paired bridges partition the holed tensor cap");
        assert_eq!(partition.faces.len(), 2);
        let expected = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        assert_eq!(
            hyperlimit::compare_reals(
                &edited
                    .solid_volume(solid)
                    .expect("paired bridge edit keeps its certificate"),
                &expected,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = edited.to_json().expect("paired bridge model serializes");
        let decoded = RawModel::from_json(&json)
            .expect("paired bridge JSON decodes")
            .validate()
            .expect("paired bridge JSON fully revalidates");
        assert_eq!(
            hyperlimit::compare_reals(
                &decoded
                    .solid_volume(solid)
                    .expect("decoded paired bridge keeps its certificate"),
                &expected,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        return;
    }
    if bytes[0] == b'm' {
        let eighth = (Real::one() / Real::from(8)).expect("eight is nonzero");
        let quarter = (Real::one() / Real::from(4)).expect("four is nonzero");
        let three_eighths = (Real::from(3) / Real::from(8)).expect("eight is nonzero");
        let half = (Real::one() / Real::from(2)).expect("two is nonzero");
        let five_eighths = (Real::from(5) / Real::from(8)).expect("eight is nonzero");
        let three_quarters = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        let seven_eighths = (Real::from(7) / Real::from(8)).expect("eight is nonzero");
        let outer = [
            hyperbrep::Point2::new(Real::zero(), Real::zero()),
            hyperbrep::Point2::new(Real::one(), Real::zero()),
            hyperbrep::Point2::new(Real::one(), Real::one()),
            hyperbrep::Point2::new(Real::zero(), Real::one()),
        ];
        let holes = [
            vec![
                hyperbrep::Point2::new(quarter.clone(), eighth.clone()),
                hyperbrep::Point2::new(three_quarters.clone(), eighth.clone()),
                hyperbrep::Point2::new(three_quarters.clone(), three_eighths.clone()),
                hyperbrep::Point2::new(quarter.clone(), three_eighths.clone()),
            ],
            vec![
                hyperbrep::Point2::new(quarter.clone(), five_eighths.clone()),
                hyperbrep::Point2::new(three_quarters.clone(), five_eighths.clone()),
                hyperbrep::Point2::new(three_quarters.clone(), seven_eighths.clone()),
                hyperbrep::Point2::new(quarter.clone(), seven_eighths.clone()),
            ],
        ];
        let (source, solid) = builder::extrude_region(&outer, &holes, Real::zero(), Real::one())
            .expect("two-hole unit extrusion is constructible");
        let face = source
            .shell(source.solid(solid).expect("validated solid").outer())
            .expect("validated shell")
            .faces()[1];
        let surface_id = source.face(face).expect("validated cap").surface();
        let tensor_surface = Surface::rational_bezier(
            vec![
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::zero(), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::one(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::one(), Real::one()),
                ],
            ],
            vec![vec![Real::one(), Real::one()]; 2],
        )
        .expect("affine tensor surface is constructible");
        let mut edit = source.edit();
        edit.replace_surface(surface_id, tensor_surface.clone())
            .expect("affine cap replacement is valid");
        let tensor = edit.commit().expect("two-hole tensor cap certifies");
        let support = Surface::plane(
            hyperbrep::Point3::new(half, Real::zero(), Real::one()),
            Vector3::y(),
            Vector3::z(),
        )
        .expect("boundary support plane is constructible");
        let SurfaceSurfaceIntersection::Curve(trace) = tensor_surface
            .intersect_surface(&support)
            .expect("affine tensor section is exact")
        else {
            panic!("affine tensor section retains one curve");
        };
        let mut traces = vec![
            trace
                .subcurve(&Real::zero(), &eighth)
                .expect("first material bridge"),
            trace
                .subcurve(&three_eighths, &five_eighths)
                .expect("middle material bridge"),
            trace
                .subcurve(&seven_eighths, &Real::one())
                .expect("last material bridge"),
        ];
        if bytes[1] & 1 != 0 {
            traces.reverse();
            traces = traces
                .into_iter()
                .map(|trace| trace.reversed().expect("exact trace reversal"))
                .collect();
        }
        let operand = if bytes[2] & 1 == 0 {
            SurfaceIntersectionOperand::First
        } else {
            SurfaceIntersectionOperand::Second
        };
        let source_wires = tensor.counts().wires;
        let (edited, partition) = tensor
            .split_face_by_surface_curves(face, &traces, operand)
            .expect("bridge cycle partitions both threaded holes");
        assert_eq!(partition.faces.len(), 2);
        assert_eq!(edited.counts().wires + 1, source_wires);
        assert!(partition.traces.iter().all(|trace| {
            trace
                .splits
                .first()
                .and_then(hyperbrep::SurfaceCurveFaceSplit::bridge)
                .is_some_and(|bridge| {
                    bridge.face.edges.len() == 3
                        && bridge
                            .face
                            .wire_remap
                            .iter()
                            .filter(|wire| wire.is_none())
                            .count()
                            == 1
                })
        }));
        let expected = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        assert_eq!(
            hyperlimit::compare_reals(
                &edited
                    .solid_volume(solid)
                    .expect("bridge-cycle edit keeps its certificate"),
                &expected,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = edited.to_json().expect("bridge-cycle model serializes");
        assert_eq!(
            RawModel::from_json(&json)
                .expect("bridge-cycle JSON decodes")
                .validate()
                .expect("bridge-cycle JSON fully revalidates")
                .to_json()
                .expect("bridge-cycle replay serializes"),
            json
        );
        return;
    }
    if bytes[0] == b's' {
        let (tensor, tensor_face) = builder::rational_bezier_patch(
            vec![
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::zero(), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::one(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::one(), Real::one()),
                ],
            ],
            vec![vec![Real::one(), Real::one()]; 2],
        )
        .expect("unit affine tensor patch is constructible");
        let quarter = (Real::one() / Real::from(4)).expect("four is nonzero");
        let half = (Real::one() / Real::from(2)).expect("two is nonzero");
        let three_quarters = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        let third = (Real::one() / Real::from(3)).expect("three is nonzero");
        let two_thirds = (Real::from(2) / Real::from(3)).expect("three is nonzero");
        let start = hypercurve::Point2::new(quarter.clone(), half.clone());
        let end = hypercurve::Point2::new(three_quarters.clone(), half.clone());
        let outer = hypercurve::CurvePath2::try_new(vec![
            hypercurve::Curve2::from(hypercurve::QuadraticBezier2::new(
                start.clone(),
                hypercurve::Point2::new(half, three_quarters),
                end.clone(),
            )),
            hypercurve::Curve2::try_nurbs(
                3,
                vec![
                    end,
                    hypercurve::Point2::new(two_thirds, quarter.clone()),
                    hypercurve::Point2::new(third, quarter),
                    start,
                ],
                vec![Real::one(), positive(2), positive(3), Real::one()],
                vec![
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::zero(),
                    Real::one(),
                    Real::one(),
                    Real::one(),
                    Real::one(),
                ],
            )
            .expect("positive-weight cubic NURBS half-loop"),
        ])
        .expect("mixed spline loop is simple");
        let (plane, plane_face) = builder::planar_face(
            &outer,
            &[],
            hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
            Vector3::x(),
            Vector3::y(),
        )
        .expect("mixed spline plane region is constructible");
        let (edited, partition) = hyperbrep::partition_contained_face_by_plane_region(
            &tensor,
            tensor_face,
            &plane,
            plane_face,
        )
        .expect("mixed spline inverse pcurve is certified")
        .expect("mixed spline boundary partitions the tensor interior");
        assert_eq!(partition.faces.len(), 2);
        let json = edited.to_json().expect("mixed spline split serializes");
        assert_eq!(
            RawModel::from_json(&json)
                .expect("mixed spline JSON decodes")
                .validate()
                .expect("mixed spline JSON fully revalidates")
                .to_json()
                .expect("mixed spline replay serializes"),
            json
        );
        return;
    }
    if matches!(bytes[0], 0xa7 | b'p') {
        let tensor_weights = if bytes[2] & 2 == 0 {
            vec![vec![Real::one(), Real::one()]; 2]
        } else {
            vec![
                vec![Real::one(), Real::from(2)],
                vec![Real::from(3), Real::from(6)],
            ]
        };
        let (tensor, tensor_face) = builder::rational_bezier_patch(
            vec![
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::zero(), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::one(), Real::one()),
                    hyperbrep::Point3::new(Real::one(), Real::one(), Real::one()),
                ],
            ],
            tensor_weights,
        )
        .expect("unit affine tensor patch is constructible");
        let quarter = (Real::one() / Real::from(4)).expect("four is nonzero");
        let half = (Real::one() / Real::from(2)).expect("two is nonzero");
        let three_quarters = (Real::from(3) / Real::from(4)).expect("four is nonzero");
        let start = hypercurve::Point2::new(Real::zero(), quarter);
        let end = hypercurve::Point2::new(Real::one(), three_quarters);
        let control_y = if bytes[1] & 1 == 0 {
            Real::zero()
        } else {
            Real::one()
        };
        let outer = hypercurve::CurvePath2::try_new(vec![
            hypercurve::Curve2::from(hypercurve::QuadraticBezier2::new(
                start.clone(),
                hypercurve::Point2::new(half, control_y),
                end.clone(),
            )),
            hypercurve::Curve2::from(
                hypercurve::LineSeg2::try_new(
                    end,
                    hypercurve::Point2::new(Real::one(), Real::from(2)),
                )
                .expect("curved region right edge"),
            ),
            hypercurve::Curve2::from(
                hypercurve::LineSeg2::try_new(
                    hypercurve::Point2::new(Real::one(), Real::from(2)),
                    hypercurve::Point2::new(Real::zero(), Real::from(2)),
                )
                .expect("curved region upper edge"),
            ),
            hypercurve::Curve2::from(
                hypercurve::LineSeg2::try_new(
                    hypercurve::Point2::new(Real::zero(), Real::from(2)),
                    start,
                )
                .expect("curved region left edge"),
            ),
        ])
        .expect("curved plane region is simple");
        let (plane, plane_face) = builder::planar_face(
            &outer,
            &[],
            hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::one()),
            Vector3::x(),
            Vector3::y(),
        )
        .expect("curved plane region is constructible");
        let (edited, partition) = hyperbrep::partition_contained_face_by_plane_region(
            &tensor,
            tensor_face,
            &plane,
            plane_face,
        )
        .expect("curved affine inverse pcurve is certified")
        .expect("curved boundary partitions the tensor interior");
        assert_eq!(partition.faces.len(), 2);
        let json = edited.to_json().expect("curved tensor split serializes");
        assert_eq!(
            RawModel::from_json(&json)
                .expect("curved tensor JSON decodes")
                .validate()
                .expect("curved tensor JSON fully revalidates")
                .to_json()
                .expect("curved tensor replay serializes"),
            json
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
