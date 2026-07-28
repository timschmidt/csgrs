#![no_main]

mod support;

use hypermesh::{
    BooleanOp, EmberConfig, HypermeshError, InputMesh, Triangle, boolean_operation,
    boolean_operation_with_certified_convex_inputs, boolean_triangle_soup,
};
use libfuzzer_sys::fuzz_target;
use support::{Bytes, box_mesh, operation};

fn rejection(mesh: &InputMesh, operation: BooleanOp, immediate: bool) -> HypermeshError {
    if immediate {
        boolean_triangle_soup(&[mesh.as_ref()], operation, EmberConfig::default()).unwrap_err()
    } else {
        boolean_operation(&[mesh.as_ref()], operation, EmberConfig::default()).unwrap_err()
    }
}

fuzz_target!(|data: [u8; 4]| {
    let mut bytes = Bytes::new(&data);
    let mutation = bytes.next() % 7;
    let op = operation(bytes.next());
    let immediate = bytes.next() & 1 != 0;
    let mut mesh = box_mesh([-2, -2, -2], [2, 2, 2]);

    match mutation {
        0 => {
            let error = if immediate {
                boolean_triangle_soup(&[], op, EmberConfig::default()).unwrap_err()
            } else {
                boolean_operation(&[], op, EmberConfig::default()).unwrap_err()
            };
            assert_eq!(error, HypermeshError::EmptyInput);
        }
        1 => {
            if bytes.next() & 1 == 0 {
                mesh.positions.clear();
            } else {
                mesh.triangles.clear();
            }
            assert_eq!(
                rejection(&mesh, op, immediate),
                HypermeshError::EmptyMesh { mesh_index: 0 },
            );
        }
        2 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let invalid = mesh.positions.len() + usize::from(bytes.next() % 8);
            mesh.triangles[triangle].v0 = invalid;
            assert_eq!(
                rejection(&mesh, op, immediate),
                HypermeshError::VertexIndexOutOfBounds {
                    index: invalid,
                    vertex_count: mesh.positions.len(),
                },
            );
        }
        3 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            mesh.triangles[triangle].v1 = mesh.triangles[triangle].v0;
            assert!(matches!(
                rejection(&mesh, op, immediate),
                HypermeshError::DegenerateTriangle {
                    mesh_index: 0,
                    triangle_index
                } if triangle_index == triangle
            ));
        }
        4 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            mesh.triangles.remove(triangle);
            assert!(matches!(
                rejection(&mesh, op, immediate),
                HypermeshError::OpenInput {
                    mesh_index: 0,
                    boundary_edges: 3,
                }
            ));
        }
        5 => {
            let triangle = usize::from(bytes.next()) % mesh.triangles.len();
            let [a, b, c] = mesh.triangles[triangle].indices();
            mesh.triangles[triangle] = Triangle::new(a, c, b);
            assert!(matches!(
                rejection(&mesh, op, immediate),
                HypermeshError::NonPwnInput {
                    mesh_index: 0,
                    unbalanced_edges: 3,
                }
            ));
        }
        _ => {
            let error = boolean_operation_with_certified_convex_inputs(
                &[mesh.as_ref()],
                op,
                &[],
                EmberConfig::default(),
            )
            .unwrap_err();
            assert_eq!(error, HypermeshError::UnknownClassification);
        }
    }
});
