#![no_main]

mod support;

use hypermesh::{
    BooleanExpression, BooleanOp, BooleanProgram, HypermeshError, Triangle, TriangleMesh, boolean,
};
use libfuzzer_sys::fuzz_target;
use support::{Bytes, CONTEXT, box_mesh, operation, value};

fn rejection(mesh: &TriangleMesh, operation: BooleanOp) -> HypermeshError {
    value(boolean(
        &CONTEXT,
        &[mesh.as_ref()],
        BooleanProgram::Operation(operation),
    ))
    .unwrap_err()
}

fuzz_target!(|data: [u8; 4]| {
    let mut bytes = Bytes::new(&data);
    let mutation = bytes.next() % 11;
    let op = operation(bytes.next());
    let mesh = box_mesh([-2, -2, -2], [2, 2, 2]);

    match mutation {
        0 => {
            let error = value(boolean(
                &CONTEXT,
                &[],
                BooleanProgram::Operation(op),
            ))
            .unwrap_err();
            assert_eq!(error, HypermeshError::EmptyInput);
        }
        1 => {
            let invalid = if bytes.next() & 1 == 0 {
                TriangleMesh::new(Vec::new(), mesh.triangles.to_vec())
            } else {
                TriangleMesh::new(mesh.positions.to_vec(), Vec::new())
            };
            assert_eq!(
                rejection(&invalid, op),
                HypermeshError::EmptyMesh { mesh_index: 0 },
            );
        }
        2 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let invalid = mesh.positions.len() + usize::from(bytes.next() % 8);
            let mut triangles = mesh.triangles.to_vec();
            triangles[triangle].v0 = invalid;
            let invalid_mesh = TriangleMesh::new(mesh.positions.to_vec(), triangles);
            assert_eq!(
                rejection(&invalid_mesh, op),
                HypermeshError::VertexIndexOutOfBounds {
                    index: invalid,
                    vertex_count: mesh.positions.len(),
                },
            );
        }
        3 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let mut triangles = mesh.triangles.to_vec();
            triangles[triangle].v1 = triangles[triangle].v0;
            let invalid_mesh = TriangleMesh::new(mesh.positions.to_vec(), triangles);
            assert!(matches!(
                rejection(&invalid_mesh, op),
                HypermeshError::DegenerateTriangle {
                    mesh_index: 0,
                    triangle_index
                } if triangle_index == triangle
            ));
        }
        4 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let mut triangles = mesh.triangles.to_vec();
            triangles.remove(triangle);
            let invalid_mesh = TriangleMesh::new(mesh.positions.to_vec(), triangles);
            assert!(matches!(
                rejection(&invalid_mesh, op),
                HypermeshError::OpenInput {
                    mesh_index: 0,
                    boundary_edges: 3,
                }
            ));
        }
        5 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let mut triangles = mesh.triangles.to_vec();
            let [a, b, c] = triangles[triangle].indices();
            triangles[triangle] = Triangle::new(a, c, b);
            let invalid_mesh = TriangleMesh::new(mesh.positions.to_vec(), triangles);
            assert!(matches!(
                rejection(&invalid_mesh, op),
                HypermeshError::NonPwnInput {
                    mesh_index: 0,
                    unbalanced_edges: 3,
                }
            ));
        }
        6 => {
            let mut triangles = mesh.triangles.to_vec();
            triangles.pop();
            let invalid_mesh = TriangleMesh::new(mesh.positions.to_vec(), triangles);
            let error = rejection(&invalid_mesh, op);
            assert!(matches!(
                &error,
                HypermeshError::OpenInput {
                    mesh_index: 0,
                    boundary_edges: 3,
                }
            ), "{error:?}");
        }
        invalid_program => {
            let operand = [BooleanExpression::Operand(1)];
            let forward = [BooleanExpression::Not(0)];
            let valid = [BooleanExpression::Operand(0)];
            let (nodes, roots): (&[_], &[_]) = match invalid_program {
                7 => (&valid, &[]),
                8 => (&operand, &[0]),
                9 => (&forward, &[0]),
                _ => (&valid, &[1]),
            };
            assert!(matches!(
                value(boolean(
                    &CONTEXT,
                    &[mesh.as_ref()],
                    BooleanProgram::Expressions { nodes, roots },
                )),
                Err(HypermeshError::InvalidBooleanProgram { .. })
            ));
        }
    }
});
