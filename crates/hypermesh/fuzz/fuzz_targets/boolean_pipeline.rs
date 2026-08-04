#![no_main]

mod support;

use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanOp, BooleanProgram, HypermeshError,
    TriangleMeshRef, boolean,
};
use libfuzzer_sys::fuzz_target;
use support::{
    Bytes, CONTEXT, combine_meshes, convex_mesh, operation, subdivide_once, validate_batch, value,
    volume_numerator,
};

fn require_program(
    meshes: &[TriangleMeshRef<'_>],
    program: BooleanProgram<'_>,
) -> BooleanMeshBatch {
    let batch = value(boolean(&CONTEXT, meshes, program))
        .unwrap_or_else(|error| panic!("supported exact Boolean input failed: {error:?}"));
    validate_batch(&batch);
    batch
}

fn require_operation(meshes: &[TriangleMeshRef<'_>], operation: BooleanOp) -> BooleanMeshBatch {
    require_program(meshes, BooleanProgram::Operation(operation))
}

fn volume(batch: &BooleanMeshBatch, output: usize) -> hypermesh::Real {
    volume_numerator(&batch.vertices, &batch.results[output])
}

fn operation_index(operation: BooleanOp) -> usize {
    match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::SymmetricDifference => 3,
    }
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
            require_operation(&refs, op);
        }
        1 => {
            let nodes = [
                BooleanExpression::Operation(BooleanOp::Union),
                BooleanExpression::Operation(BooleanOp::Intersection),
                BooleanExpression::Operation(BooleanOp::Difference),
                BooleanExpression::Operation(BooleanOp::SymmetricDifference),
            ];
            let roots = [0, 1, 2, 3];
            let batch = require_program(
                &refs,
                BooleanProgram::Expressions {
                    nodes: &nodes,
                    roots: &roots,
                },
            );
            let single = require_operation(&refs, op);
            assert_eq!(volume(&batch, operation_index(op)), volume(&single, 0));
        }
        2 => {
            let raw_refs = meshes
                .iter()
                .map(|mesh| TriangleMeshRef::new(&mesh.positions, &mesh.triangles))
                .collect::<Vec<_>>();
            assert_eq!(
                volume(&require_operation(&refs, op), 0),
                volume(&require_operation(&raw_refs, op), 0),
            );
        }
        3 => {
            let pair = [
                convex_mesh(&mut bytes),
                convex_mesh(&mut bytes),
            ];
            let retained = [pair[0].as_ref(), pair[1].as_ref()];
            let raw = [
                TriangleMeshRef::new(&pair[0].positions, &pair[0].triangles),
                TriangleMeshRef::new(&pair[1].positions, &pair[1].triangles),
            ];
            assert_eq!(
                volume(&require_operation(&retained, op), 0),
                volume(&require_operation(&raw, op), 0),
            );
        }
        4 => {
            let commutative = match bytes.next() % 3 {
                0 => BooleanOp::Union,
                1 => BooleanOp::Intersection,
                _ => BooleanOp::SymmetricDifference,
            };
            let reversed = refs.iter().rev().copied().collect::<Vec<_>>();
            assert_eq!(
                volume(&require_operation(&refs, commutative), 0),
                volume(&require_operation(&reversed, commutative), 0),
            );
        }
        5 => {
            let repeated = [refs[0], refs[0]];
            let result = require_operation(&repeated, op);
            let source = require_operation(&[refs[0]], BooleanOp::Union);
            let expected = match op {
                BooleanOp::Union | BooleanOp::Intersection => volume(&source, 0),
                BooleanOp::Difference | BooleanOp::SymmetricDifference => hypermesh::Real::zero(),
            };
            assert_eq!(volume(&result, 0), expected);
        }
        6 => {
            let nodes = [
                BooleanExpression::Operand(0),
                BooleanExpression::Operand(1),
                BooleanExpression::Not(0),
                BooleanExpression::Not(1),
                BooleanExpression::And([0, 3]),
                BooleanExpression::And([1, 2]),
                BooleanExpression::Or([0, 1]),
                BooleanExpression::Not(6),
            ];
            let roots = [4, 5, 7];
            let batch = require_program(
                &refs[..2],
                BooleanProgram::Expressions {
                    nodes: &nodes,
                    roots: &roots,
                },
            );
            let reversed = [refs[1], refs[0]];
            assert_eq!(
                volume(&batch, 0),
                volume(&require_operation(&refs[..2], BooleanOp::Difference), 0),
            );
            assert_eq!(
                volume(&batch, 1),
                volume(&require_operation(&reversed, BooleanOp::Difference), 0),
            );
            assert!(batch.results[2].exterior_inside);
            assert_eq!(
                batch.into_triangle_meshes().unwrap_err(),
                HypermeshError::UnboundedBooleanOutput { output: 2 }
            );
        }
        7 => {
            let single = require_operation(&refs, op);
            let meshes = single.clone().into_triangle_meshes().unwrap();
            assert_eq!(meshes.len(), 1);
            assert_eq!(meshes[0].triangles.len(), single.results[0].triangles.len());
        }
        _ => {
            for candidate in [
                BooleanOp::Union,
                BooleanOp::Intersection,
                BooleanOp::Difference,
                BooleanOp::SymmetricDifference,
            ] {
                require_operation(&refs, candidate);
            }
        }
    }
});
