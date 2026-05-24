#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    PathMeshBooleanOperation, boolean_rectangular_prisms,
    boolean_rectangular_prisms_with_boundary_policy, rectangular_prism_from_i64_bounds,
};
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
});
