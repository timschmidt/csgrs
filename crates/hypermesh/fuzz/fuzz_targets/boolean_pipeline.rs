#![no_main]

mod support;

use hypermesh::{
    BooleanOp, EmberConfig, HypermeshError, TriangleMeshRef, boolean_mesh, boolean_operation,
};
use libfuzzer_sys::fuzz_target;
use support::{
    Bytes, CONTEXT, combine_meshes, convex_mesh, operation, subdivide_once, validate_result,
    validate_soup, value, volume_numerator,
};

fn require_result(
    meshes: &[hypermesh::TriangleMeshRef<'_>],
    operation: BooleanOp,
) -> hypermesh::BooleanMesh {
    let result = value(boolean_operation(
        &CONTEXT,
        meshes,
        operation,
        EmberConfig::default(),
    ))
    .unwrap_or_else(|error| panic!("supported exact Boolean input failed: {error:?}"));
    validate_result(&result, operation, meshes.len())
}

fn require_soup(
    meshes: &[hypermesh::TriangleMeshRef<'_>],
    operation: BooleanOp,
) -> hypermesh::BooleanMesh {
    let soup = value(boolean_mesh(
        &CONTEXT,
        meshes,
        operation,
        EmberConfig::default(),
    ))
    .unwrap_or_else(|error| panic!("supported exact immediate Boolean input failed: {error:?}"));
    validate_soup(&soup);
    soup
}

fuzz_target!(|data: [u8; 48]| {
    let mut bytes = Bytes::new(&data);
    let mode = bytes.next() % 9;
    let op = operation(bytes.next());
    let mesh_count = 2 + usize::from(bytes.next() % 2);
    let mut meshes = (0..mesh_count)
        .map(|_| convex_mesh(&mut bytes))
        .collect::<Vec<_>>();

    if bytes.next() & 3 == 0 {
        meshes[0] = subdivide_once(meshes[0].clone());
    }
    if bytes.next() & 7 == 0 {
        let component = convex_mesh(&mut bytes);
        meshes[0] = combine_meshes(&[meshes[0].clone(), component]);
    }

    let refs = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
    match mode {
        0 => {
            require_result(&refs, op);
        }
        1 => {
            require_soup(&refs, op);
        }
        2 => {
            let polygon_soup = require_result(&refs, op);
            let immediate_soup = require_soup(&refs, op);
            assert_eq!(
                volume_numerator(&polygon_soup),
                volume_numerator(&immediate_soup),
            );
        }
        3 => {
            let pair = [
                convex_mesh(&mut bytes).with_certified_convexity(),
                convex_mesh(&mut bytes).with_certified_convexity(),
            ];
            let pair_refs = [pair[0].as_ref(), pair[1].as_ref()];
            let raw_refs = [
                TriangleMeshRef::new(&pair[0].positions, &pair[0].triangles),
                TriangleMeshRef::new(&pair[1].positions, &pair[1].triangles),
            ];
            let generic = require_result(&raw_refs, op);
            let certified =
                value(boolean_operation(&CONTEXT, &pair_refs, op, EmberConfig::default()))
            .unwrap_or_else(|error| panic!("certified-convex Boolean failed: {error:?}"));
            let certified_soup = validate_result(&certified, op, 2);
            let immediate = value(boolean_mesh(
                &CONTEXT,
                &pair_refs,
                op,
                EmberConfig::default(),
            ))
            .unwrap_or_else(|error| panic!("certified-convex immediate Boolean failed: {error:?}"));
            validate_soup(&immediate);
            assert_eq!(
                volume_numerator(&generic),
                volume_numerator(&certified_soup)
            );
            assert_eq!(volume_numerator(&generic), volume_numerator(&immediate));
        }
        4 => {
            let pair_refs = [refs[0], refs[1]];
            let generic = require_result(&pair_refs, op);
            let immediate = require_soup(&pair_refs, op);
            assert_eq!(volume_numerator(&generic), volume_numerator(&immediate));
        }
        5 => {
            let commutative = match bytes.next() % 3 {
                0 => BooleanOp::Union,
                1 => BooleanOp::Intersection,
                _ => BooleanOp::SymmetricDifference,
            };
            let forward = require_soup(&refs, commutative);
            let reversed_refs = refs.iter().rev().copied().collect::<Vec<_>>();
            let reversed = require_soup(&reversed_refs, commutative);
            assert_eq!(volume_numerator(&forward), volume_numerator(&reversed));
        }
        6 => {
            let repeated = [refs[0], refs[0]];
            let result = require_soup(&repeated, op);
            let source = require_soup(&[refs[0]], BooleanOp::Union);
            let expected = match op {
                BooleanOp::Union | BooleanOp::Intersection => volume_numerator(&source),
                BooleanOp::Difference | BooleanOp::SymmetricDifference => hypermesh::Real::zero(),
            };
            assert_eq!(volume_numerator(&result), expected);
        }
        7 => {
            let config = EmberConfig {
                max_depth: usize::from(bytes.next() % 3),
            };
            match value(boolean_operation(&CONTEXT, &refs, op, config)) {
                Ok(result) => {
                    validate_result(&result, op, refs.len());
                }
                Err(HypermeshError::SubdivisionDepthLimit { depth, .. }) => {
                    assert!(depth <= config.max_depth);
                }
                Err(error) => panic!("bounded exact Boolean returned unexpected error: {error:?}"),
            }
        }
        _ => {
            for candidate in [
                BooleanOp::Union,
                BooleanOp::Intersection,
                BooleanOp::Difference,
                BooleanOp::SymmetricDifference,
            ] {
                require_soup(&refs, candidate);
            }
        }
    }
});
