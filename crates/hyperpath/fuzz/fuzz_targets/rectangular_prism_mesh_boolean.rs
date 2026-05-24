#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    AxisAlignedSweptSegmentPrism, CamOrthogonalIslandPocketCutter, CamRestMaterialCutter,
    CardinalRotation, LinePathSegment, NetId, PathMeshBooleanOperation, PathMeshBooleanProgramStep, PcbCardinalRectPad,
    PcbCompositeCopperBooleanSource, PcbConvexPolyPad, PcbCopperBooleanSource,
    PcbHoledOrthogonalCopperSource, PcbLayerZModel, PcbOrthogonalPolyPad, PcbTrace,
    SweptLineSegment, TraceLayer, boolean_path_mesh_program, boolean_path_mesh_sources,
    boolean_rectangular_prism_chain, boolean_rectangular_prisms,
    boolean_rectangular_prisms_with_boundary_policy, build_cam_rest_material_program,
    build_pcb_composite_copper_union_program, build_pcb_copper_union_program,
    build_pcb_holed_orthogonal_copper_program,
    pcb_cardinal_rect_pad_mesh_boolean_source, pcb_trace_mesh_boolean_source,
    rectangular_prism_from_i64_bounds,
};
use hyperreal::Real;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 14 {
        return;
    }
    let coord = |index: usize| -> i64 { i64::from(i8::from_ne_bytes([data[index]])) };
    let extent = |index: usize| -> i64 { i64::from(data[index] % 16) + 1 };
    let left_min = [coord(0), coord(1), coord(2)];
    let left_max = [
        left_min[0] + extent(3),
        left_min[1] + extent(4),
        left_min[2] + extent(5),
    ];
    let right_min = [coord(6), coord(7), coord(8)];
    let right_max = [
        right_min[0] + extent(9),
        right_min[1] + extent(10),
        right_min[2] + extent(11),
    ];
    let operation = match data[12] % 3 {
        0 => PathMeshBooleanOperation::Union,
        1 => PathMeshBooleanOperation::Intersection,
        _ => PathMeshBooleanOperation::Difference,
    };
    let policy = if data[13] & 1 == 0 {
        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject
    } else {
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells
    };

    let Ok(left) = rectangular_prism_from_i64_bounds(left_min, left_max, PredicatePolicy::default())
    else {
        return;
    };
    let Ok(right) =
        rectangular_prism_from_i64_bounds(right_min, right_max, PredicatePolicy::default())
    else {
        return;
    };
    let result = if policy == hypermesh::exact::ExactBoundaryBooleanPolicy::Reject {
        boolean_rectangular_prisms(left, right, operation)
    } else {
        boolean_rectangular_prisms_with_boundary_policy(left, right, operation, policy)
    };
    if let Ok(report) = result {
        report.validate_replay().unwrap();
        report.result.validate().unwrap();
    }

    if data.len() >= 20 && data[14] & 1 == 1 {
        let third_min = [coord(15), coord(16), coord(17)];
        let third_max = [
            third_min[0] + extent(3),
            third_min[1] + extent(4),
            third_min[2] + extent(5),
        ];
        if let Ok(third) =
            rectangular_prism_from_i64_bounds(third_min, third_max, PredicatePolicy::default())
        {
            let left =
                rectangular_prism_from_i64_bounds(left_min, left_max, PredicatePolicy::default())
                    .unwrap();
            let right =
                rectangular_prism_from_i64_bounds(right_min, right_max, PredicatePolicy::default())
                    .unwrap();
            if let Ok(chain) = boolean_rectangular_prism_chain(vec![left, right, third], operation)
            {
                chain.validate_replay().unwrap();
                chain.steps.last().unwrap().result.validate().unwrap();
            }
        }
    }

    if data.len() >= 20 && data[14] & 2 == 2 {
        let start = Point2::new(Real::from(coord(15)), Real::from(coord(16)));
        let end = if data[14] & 4 == 4 {
            Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone())
        } else {
            Point2::new(start.x.clone(), start.y.clone() + Real::from(extent(17)))
        };
        if let Ok(swept) =
            SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(3)))
        {
            let z_min = Real::from(coord(18));
            let z_max = z_min.clone() + Real::from(extent(19));
            if let Ok(slab) =
                AxisAlignedSweptSegmentPrism::new(swept, z_min, z_max, PredicatePolicy::default())
            {
                let right = rectangular_prism_from_i64_bounds(
                    right_min,
                    right_max,
                    PredicatePolicy::default(),
                )
                .unwrap();
                if let Ok(chain) =
                    boolean_path_mesh_sources(vec![slab.into(), right.into()], operation)
                {
                    chain.validate_replay().unwrap();
                    chain.steps.last().unwrap().result.validate().unwrap();
                }
            }
        }
    }

    if data.len() >= 20 && data[14] & 8 == 8 {
        let Ok(initial) =
            rectangular_prism_from_i64_bounds(left_min, left_max, PredicatePolicy::default())
        else {
            return;
        };
        let Ok(envelope) =
            rectangular_prism_from_i64_bounds(right_min, right_max, PredicatePolicy::default())
        else {
            return;
        };
        let start = Point2::new(Real::from(coord(15)), Real::from(coord(16)));
        let end = if data[14] & 4 == 4 {
            Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone())
        } else {
            Point2::new(start.x.clone(), start.y.clone() + Real::from(extent(17)))
        };
        if let Ok(swept) =
            SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(3)))
        {
            let z_min = Real::from(coord(18));
            let z_max = z_min.clone() + Real::from(extent(19));
            if let Ok(slab) =
                AxisAlignedSweptSegmentPrism::new(swept, z_min, z_max, PredicatePolicy::default())
            {
                if let Ok(program) = boolean_path_mesh_program(
                    initial.into(),
                    vec![
                        PathMeshBooleanProgramStep::new(
                            PathMeshBooleanOperation::Intersection,
                            envelope.into(),
                        ),
                        PathMeshBooleanProgramStep::new(operation, slab.into()),
                    ],
                ) {
                    program.validate_replay().unwrap();
                    program.steps.last().unwrap().result.validate().unwrap();
                }
            }
        }
    }

    if data.len() >= 20 && data[14] & 16 == 16 {
        let Ok(z_model) = PcbLayerZModel::new(
            Real::from(coord(2)),
            Real::from(extent(3) + extent(4)),
            Real::from(extent(3)),
            PredicatePolicy::default(),
        ) else {
            return;
        };
        let layer = TraceLayer(u16::from(data[15] % 8));
        let start = Point2::new(Real::from(coord(0)), Real::from(coord(1)));
        let end = if data[14] & 4 == 4 {
            Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone())
        } else {
            Point2::new(start.x.clone(), start.y.clone() + Real::from(extent(17)))
        };
        if let Ok(swept) =
            SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(5)))
        {
            let trace = PcbTrace::new(NetId(u32::from(data[16])), layer, swept);
            let pad = PcbCardinalRectPad::new(
                NetId(u32::from(data[16])),
                layer,
                Point2::new(Real::from(coord(6)), Real::from(coord(7))),
                Real::from(extent(9)),
                Real::from(extent(10)),
                if data[14] & 32 == 32 {
                    CardinalRotation::Deg90
                } else {
                    CardinalRotation::Deg0
                },
            )
            .unwrap();
            if let (Ok(trace_source), Ok(pad_source)) = (
                pcb_trace_mesh_boolean_source(&trace, &z_model, PredicatePolicy::default()),
                pcb_cardinal_rect_pad_mesh_boolean_source(&pad, &z_model, PredicatePolicy::default()),
            ) {
                if let Ok(program) = boolean_path_mesh_program(
                    trace_source,
                    vec![PathMeshBooleanProgramStep::new(
                        PathMeshBooleanOperation::Union,
                        pad_source,
                    )],
                ) {
                    program.validate_replay().unwrap();
                    program.steps.last().unwrap().result.validate().unwrap();
                }
            }
        }

        if data[14] & 64 == 64 {
            let first_start = Point2::new(Real::from(coord(0)), Real::from(coord(1)));
            let first_end = if data[14] & 4 == 4 {
                Point2::new(first_start.x.clone() + Real::from(extent(17)), first_start.y.clone())
            } else {
                Point2::new(first_start.x.clone(), first_start.y.clone() + Real::from(extent(17)))
            };
            let second_start = Point2::new(Real::from(coord(8)), Real::from(coord(9)));
            let second_end = if data[14] & 4 == 4 {
                Point2::new(
                    second_start.x.clone() + Real::from(extent(10)),
                    second_start.y.clone(),
                )
            } else {
                Point2::new(
                    second_start.x.clone(),
                    second_start.y.clone() + Real::from(extent(10)),
                )
            };
            if let (Ok(first_swept), Ok(second_swept)) = (
                SweptLineSegment::new(
                    LinePathSegment::new(first_start, first_end),
                    Real::from(extent(5)),
                ),
                SweptLineSegment::new(
                    LinePathSegment::new(second_start, second_end),
                    Real::from(extent(11)),
                ),
            ) {
                let first_trace = PcbTrace::new(NetId(u32::from(data[16])), layer, first_swept);
                let second_trace = PcbTrace::new(NetId(u32::from(data[16])), layer, second_swept);
                let pad = PcbCardinalRectPad::new(
                    NetId(u32::from(data[16])),
                    layer,
                    Point2::new(Real::from(coord(6)), Real::from(coord(7))),
                    Real::from(extent(9)),
                    Real::from(extent(10)),
                    if data[14] & 32 == 32 {
                        CardinalRotation::Deg90
                    } else {
                        CardinalRotation::Deg0
                    },
                )
                .unwrap();
                if let Ok(report) = build_pcb_copper_union_program(
                    vec![
                        PcbCopperBooleanSource::Trace(first_trace),
                        PcbCopperBooleanSource::Trace(second_trace),
                        PcbCopperBooleanSource::CardinalRectPad(pad),
                    ],
                    z_model.clone(),
                    PredicatePolicy::default(),
                ) {
                    report.validate_replay(PredicatePolicy::default()).unwrap();
                    report.program.steps.last().unwrap().result.validate().unwrap();
                }
            }
        }

        if data[14] & 2 == 2 {
            let center = Point2::new(Real::from(coord(6)), Real::from(coord(7)));
            let dx = Real::from(extent(9));
            let dy = Real::from(extent(10));
            let vertices = vec![
                Point2::new(center.x.clone(), center.y.clone() - dy.clone()),
                Point2::new(center.x.clone() + dx.clone(), center.y.clone()),
                Point2::new(center.x.clone(), center.y.clone() + dy),
                Point2::new(center.x.clone() - dx, center.y.clone()),
            ];
            if let Ok(poly) = PcbConvexPolyPad::new(NetId(u32::from(data[16])), layer, vertices) {
                let start = Point2::new(Real::from(coord(0)), Real::from(coord(1)));
                let end = Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone());
                if let Ok(swept) =
                    SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(5)))
                {
                    let trace = PcbTrace::new(NetId(u32::from(data[16])), layer, swept);
                    if let Ok(report) = build_pcb_copper_union_program(
                        vec![
                            PcbCopperBooleanSource::Trace(trace),
                            PcbCopperBooleanSource::ConvexPolyPad(poly),
                        ],
                        z_model.clone(),
                        PredicatePolicy::default(),
                    ) {
                        report.validate_replay(PredicatePolicy::default()).unwrap();
                        report.program.steps.last().unwrap().result.validate().unwrap();
                    }
                }
            }
        }

        if data[14] & 1 == 1 {
            let min_x = Real::from(coord(6));
            let min_y = Real::from(coord(7));
            let w = Real::from(extent(9) + extent(10));
            let h = Real::from(extent(11) + extent(12));
            let notch_w = Real::from(extent(9));
            let notch_h = Real::from(extent(10));
            let vertices = vec![
                Point2::new(min_x.clone(), min_y.clone()),
                Point2::new(min_x.clone() + w.clone(), min_y.clone()),
                Point2::new(min_x.clone() + w.clone(), min_y.clone() + notch_h.clone()),
                Point2::new(min_x.clone() + notch_w.clone(), min_y.clone() + notch_h),
                Point2::new(min_x.clone() + notch_w, min_y.clone() + h.clone()),
                Point2::new(min_x, min_y.clone() + h),
            ];
            if let Ok(poly) =
                PcbOrthogonalPolyPad::new(NetId(u32::from(data[16])), layer, vertices)
            {
                let start = Point2::new(Real::from(coord(0)), Real::from(coord(1)));
                let end = Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone());
                if let Ok(swept) =
                    SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(5)))
                {
                    let trace = PcbTrace::new(NetId(u32::from(data[16])), layer, swept);
                    if let Ok(report) = build_pcb_copper_union_program(
                        vec![
                            PcbCopperBooleanSource::Trace(trace),
                            PcbCopperBooleanSource::OrthogonalPolyPad(poly),
                        ],
                        z_model.clone(),
                        PredicatePolicy::default(),
                    ) {
                        report.validate_replay(PredicatePolicy::default()).unwrap();
                        report.program.steps.last().unwrap().result.validate().unwrap();
                    }
                }
            }
        }

        if data[17] & 128 == 128 {
            let min_x = Real::from(coord(6));
            let min_y = Real::from(coord(7));
            let w = Real::from(extent(9) + extent(10) + extent(11));
            let h = Real::from(extent(10) + extent(11) + extent(12));
            let hole_x0 = min_x.clone() + Real::from(extent(9));
            let hole_y0 = min_y.clone() + Real::from(extent(10));
            let hole_x1 = hole_x0.clone() + Real::from(extent(11));
            let hole_y1 = hole_y0.clone() + Real::from(extent(12));
            if let Ok(source) = PcbHoledOrthogonalCopperSource::new(
                NetId(u32::from(data[16])),
                layer,
                vec![
                    Point2::new(min_x.clone(), min_y.clone()),
                    Point2::new(min_x.clone() + w.clone(), min_y.clone()),
                    Point2::new(min_x.clone() + w, min_y.clone() + h.clone()),
                    Point2::new(min_x, min_y.clone() + h),
                ],
                vec![vec![
                    Point2::new(hole_x0.clone(), hole_y0.clone()),
                    Point2::new(hole_x1.clone(), hole_y0),
                    Point2::new(hole_x1, hole_y1.clone()),
                    Point2::new(hole_x0, hole_y1),
                ]],
                PredicatePolicy::default(),
            ) {
                if let Ok(report) = build_pcb_holed_orthogonal_copper_program(
                    source.clone(),
                    z_model.clone(),
                    PredicatePolicy::default(),
                ) {
                    report.validate_replay(PredicatePolicy::default()).unwrap();
                    report.program.steps.last().unwrap().result.validate().unwrap();
                }
                let trace_start = Point2::new(min_x.clone() + Real::from(extent(11)), min_y.clone());
                let trace_end = Point2::new(trace_start.x.clone() + Real::from(extent(17)), trace_start.y);
                if let Ok(swept) = SweptLineSegment::new(
                    LinePathSegment::new(trace_start, trace_end),
                    Real::from(extent(5)),
                ) {
                    let trace = PcbTrace::new(NetId(u32::from(data[16])), layer, swept);
                    if let Ok(report) = build_pcb_composite_copper_union_program(
                        vec![
                            PcbCompositeCopperBooleanSource::HoledOrthogonal(source),
                            PcbCompositeCopperBooleanSource::Solid(PcbCopperBooleanSource::Trace(
                                trace,
                            )),
                        ],
                        z_model.clone(),
                        PredicatePolicy::default(),
                    ) {
                        report.validate_replay(PredicatePolicy::default()).unwrap();
                        report.steps.last().unwrap().result.validate().unwrap();
                    }
                }
            }
        }
    }

    if data.len() >= 20 && data[14] & 128 == 128 {
        let stock_min = Point2::new(Real::from(left_min[0]), Real::from(left_min[1]));
        let stock_max = Point2::new(Real::from(left_max[0]), Real::from(left_max[1]));
        let Ok(stock) = hyperpath::RectangularPocket::new(stock_min, stock_max) else {
            return;
        };
        let start = Point2::new(Real::from(coord(15)), Real::from(coord(16)));
        let end = if data[14] & 4 == 4 {
            Point2::new(start.x.clone() + Real::from(extent(17)), start.y.clone())
        } else {
            Point2::new(start.x.clone(), start.y.clone() + Real::from(extent(17)))
        };
        if let Ok(swept) =
            SweptLineSegment::new(LinePathSegment::new(start, end), Real::from(extent(3)))
        {
            let pocket_min = Point2::new(Real::from(right_min[0]), Real::from(right_min[1]));
            let pocket_max = Point2::new(Real::from(right_max[0]), Real::from(right_max[1]));
            if let Ok(pocket) = hyperpath::RectangularPocket::new(pocket_min, pocket_max) {
                if let Ok(report) = build_cam_rest_material_program(
                    stock.clone(),
                    Real::from(left_min[2]),
                    Real::from(left_max[2]),
                    vec![
                        CamRestMaterialCutter::AxisAlignedSweep(swept),
                        CamRestMaterialCutter::RectangularPocket(pocket),
                    ],
                    PredicatePolicy::default(),
                ) {
                    report.validate_replay(PredicatePolicy::default()).unwrap();
                    report.program.steps.last().unwrap().result.validate().unwrap();
                }
            }
            let outer_min = Point2::new(Real::from(left_min[0]), Real::from(left_min[1]));
            let outer_w = Real::from(extent(3) + extent(4) + extent(5) + 2);
            let outer_h = Real::from(extent(6) + extent(7) + extent(8) + 2);
            let island_min = Point2::new(
                outer_min.x.clone() + Real::from(extent(3)),
                outer_min.y.clone() + Real::from(extent(4)),
            );
            let island_w = Real::from(extent(5));
            let island_h = Real::from(extent(6));
            if let Ok(island_pocket) = CamOrthogonalIslandPocketCutter::new(
                vec![
                    outer_min.clone(),
                    Point2::new(outer_min.x.clone() + outer_w.clone(), outer_min.y.clone()),
                    Point2::new(
                        outer_min.x.clone() + outer_w,
                        outer_min.y.clone() + outer_h.clone(),
                    ),
                    Point2::new(outer_min.x, outer_min.y + outer_h),
                ],
                vec![vec![
                    island_min.clone(),
                    Point2::new(island_min.x.clone() + island_w.clone(), island_min.y.clone()),
                    Point2::new(island_min.x.clone() + island_w, island_min.y.clone() + island_h.clone()),
                    Point2::new(island_min.x, island_min.y + island_h),
                ]],
                PredicatePolicy::default(),
            ) {
                if let Ok(report) = build_cam_rest_material_program(
                    stock,
                    Real::from(left_min[2]),
                    Real::from(left_max[2]),
                    vec![CamRestMaterialCutter::OrthogonalIslandPocket(island_pocket)],
                    PredicatePolicy::default(),
                ) {
                    report.validate_replay(PredicatePolicy::default()).unwrap();
                    report.program.steps.last().unwrap().result.validate().unwrap();
                }
            }
        }
    }
});
