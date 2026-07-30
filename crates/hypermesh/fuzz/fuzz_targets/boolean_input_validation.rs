#![no_main]

mod support;

use hypermesh::{
    BooleanOp, EmberConfig, HypermeshError, Triangle, TriangleMesh, boolean_mesh,
    boolean_operation, boolean_operation_with_certified_convex_inputs,
};
use libfuzzer_sys::fuzz_target;
use support::{Bytes, CONTEXT, box_mesh, operation, value};

fn rejection(mesh: &TriangleMesh, operation: BooleanOp, immediate: bool) -> HypermeshError {
    if immediate {
        value(boolean_mesh(
            &CONTEXT,
            &[mesh.as_ref()],
            operation,
            EmberConfig::default(),
        ))
        .unwrap_err()
    } else {
        value(boolean_operation(
            &CONTEXT,
            &[mesh.as_ref()],
            operation,
            EmberConfig::default(),
        ))
        .unwrap_err()
    }
}

fuzz_target!(|data: [u8; 4]| {
    let mut bytes = Bytes::new(&data);
    let mutation = bytes.next() % 7;
    let op = operation(bytes.next());
    let immediate = bytes.next() & 1 != 0;
    let mesh = box_mesh([-2, -2, -2], [2, 2, 2]);

    match mutation {
        0 => {
            let error = if immediate {
                value(boolean_mesh(&CONTEXT, &[], op, EmberConfig::default())).unwrap_err()
            } else {
                value(boolean_operation(&CONTEXT, &[], op, EmberConfig::default())).unwrap_err()
            };
            assert_eq!(error, HypermeshError::EmptyInput);
        }
        1 => {
            let invalid = if bytes.next() & 1 == 0 {
                TriangleMesh::new(Vec::new(), mesh.triangles.to_vec())
            } else {
                TriangleMesh::new(mesh.positions.to_vec(), Vec::new())
            };
            assert_eq!(
                rejection(&invalid, op, immediate),
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
                rejection(&invalid_mesh, op, immediate),
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
                rejection(&invalid_mesh, op, immediate),
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
                rejection(&invalid_mesh, op, immediate),
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
                rejection(&invalid_mesh, op, immediate),
                HypermeshError::NonPwnInput {
                    mesh_index: 0,
                    unbalanced_edges: 3,
                }
            ));
        }
        _ => {
            let error = value(boolean_operation_with_certified_convex_inputs(
                &CONTEXT,
                &[mesh.as_ref()],
                op,
                &[],
                EmberConfig::default(),
            ))
            .unwrap_err();
            assert_eq!(error, HypermeshError::UnknownClassification);
        }
    }
});
