#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    AxisAlignedSweptSegmentPrism, LinePathSegment, PathMeshBooleanOperation,
    PathMeshBooleanProgramStep, SweptLineSegment, boolean_path_mesh_program,
    boolean_path_mesh_sources, boolean_rectangular_prism_chain, boolean_rectangular_prisms,
    boolean_rectangular_prisms_with_boundary_policy, rectangular_prism_from_i64_bounds,
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
});
