#![no_main]

use hypermesh::{
    BooleanOp, EmberConfig, InputMesh, Point3, Real, Triangle, boolean_difference,
    boolean_intersection, boolean_operation, boolean_operation_with_certified_convex_inputs,
    boolean_symmetric_difference, boolean_triangle_soup,
    boolean_triangle_soup_with_certified_convex_inputs, boolean_union,
    certify_output_polygon_closure, triangulate_and_resolve_certified,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn cube(center: [i64; 3], half_extent: i64) -> InputMesh {
    let [cx, cy, cz] = center;
    let min = [cx - half_extent, cy - half_extent, cz - half_extent];
    let max = [cx + half_extent, cy + half_extent, cz + half_extent];
    InputMesh::new(
        vec![
            p(min[0], min[1], min[2]),
            p(max[0], min[1], min[2]),
            p(max[0], max[1], min[2]),
            p(min[0], max[1], min[2]),
            p(min[0], min[1], max[2]),
            p(max[0], min[1], max[2]),
            p(max[0], max[1], max[2]),
            p(min[0], max[1], max[2]),
        ],
        vec![
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(0, 3, 2),
            Triangle::new(0, 2, 1),
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            Triangle::new(3, 7, 6),
            Triangle::new(3, 6, 2),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
        ],
    )
}

fn validate(result: &hypermesh::BooleanResult) {
    let closure = certify_output_polygon_closure(result).unwrap();
    assert!(closure.has_no_boundary());
    let soup = triangulate_and_resolve_certified(result).unwrap();
    assert!(hypermesh::triangle_soup_closure_evidence(&soup).has_no_boundary());
}

fuzz_target!(|data: [u8; 4]| {
    let shift = i64::from(data[0] % 7) - 3;
    let left = cube([0, 0, 0], 2);
    let right = cube([shift, i64::from(data[1] % 3) - 1, 0], 2);
    let refs = [left.as_ref(), right.as_ref()];
    let op = match data[2] % 4 {
        0 => BooleanOp::Union,
        1 => BooleanOp::Intersection,
        2 => BooleanOp::Difference,
        _ => BooleanOp::SymmetricDifference,
    };
    let config = EmberConfig::default();

    match data[3] % 4 {
        0 => {
            if let Ok(result) = boolean_operation(&refs, op, config) {
                validate(&result);
            }
        }
        1 => {
            if let Ok(soup) = boolean_triangle_soup(&refs, op, config) {
                assert!(hypermesh::triangle_soup_closure_evidence(&soup).has_no_boundary());
            }
        }
        2 => {
            if let Ok(result) = boolean_operation_with_certified_convex_inputs(
                &refs,
                op,
                &[true, true],
                config,
            ) {
                validate(&result);
                let soup = boolean_triangle_soup_with_certified_convex_inputs(
                    &refs,
                    op,
                    &[true, true],
                    config,
                )
                .unwrap();
                assert!(hypermesh::triangle_soup_closure_evidence(&soup).has_no_boundary());
            }
        }
        _ => {
            let result = match op {
                BooleanOp::Union => boolean_union(refs[0], refs[1], config),
                BooleanOp::Intersection => boolean_intersection(refs[0], refs[1], config),
                BooleanOp::Difference => boolean_difference(refs[0], refs[1], config),
                BooleanOp::SymmetricDifference => {
                    boolean_symmetric_difference(refs[0], refs[1], config)
                }
            };
            if let Ok(result) = result {
                validate(&result);
            }
        }
    }
});
