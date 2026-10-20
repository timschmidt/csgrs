//! Solid containment must validate its source before query shortcuts can answer.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use std::sync::Arc;

use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hypermesh::{HypermeshError, MeshCertainty, Point3, Triangle};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn tetrahedron() -> TriangleMesh {
    TriangleMesh::new(
        vec![
            point(0, 0, 0),
            point(1, 0, 0),
            point(0, 1, 0),
            point(0, 0, 1),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn assert_invalid(mesh: &TriangleMesh, expected: HypermeshError) {
    // The first query lies outside every fixture's bounds. The second is a
    // source vertex, so neither bounds nor boundary exclusion may hide errors.
    for query in [point(10, 10, 10), point(0, 0, 0)] {
        for context in [
            GeometryContext::STRICT,
            GeometryContext::APPROXIMATE_512,
            GeometryContext::STRICT,
        ] {
            assert_eq!(
                mesh.contains_point(&context.mesh_context(), &query),
                Err(expected.clone())
            );
            assert_eq!(
                solid::contains_point_with_context(mesh, &query, &context),
                Err(expected.clone())
            );
        }
        assert_eq!(solid::contains_point(mesh, &query), Err(expected.clone()));
    }
}

#[test]
fn canonical_empty_membership_is_certified_false() {
    let mesh = TriangleMesh::new(Vec::new(), Vec::new());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let result =
            solid::contains_point_with_context(&mesh, &point(0, 0, 0), &context).unwrap();
        assert!(!result.value);
        assert_eq!(result.certainty, GeometryCertainty::Certified);
    }
}

#[test]
fn missing_one_geometry_buffer_is_not_canonical_empty() {
    for mesh in [
        TriangleMesh::new(vec![point(0, 0, 0)], Vec::new()),
        TriangleMesh::new(Vec::new(), vec![Triangle::new(0, 1, 2)]),
    ] {
        assert_invalid(&mesh, HypermeshError::EmptyMesh { mesh_index: 0 });
    }
}

#[test]
fn invalid_indices_survive_bounds_and_boundary_shortcuts() {
    let mesh =
        TriangleMesh::new(tetrahedron().positions.to_vec(), vec![Triangle::new(0, 1, 4)]);
    assert_invalid(
        &mesh,
        HypermeshError::VertexIndexOutOfBounds {
            index: 4,
            vertex_count: 4,
        },
    );
}

#[test]
fn open_input_survives_bounds_and_boundary_shortcuts() {
    let mesh =
        TriangleMesh::new(tetrahedron().positions.to_vec(), vec![Triangle::new(0, 2, 1)]);
    assert_invalid(
        &mesh,
        HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3,
        },
    );
}

#[test]
fn degenerate_input_survives_bounds_and_boundary_shortcuts() {
    let mesh =
        TriangleMesh::new(tetrahedron().positions.to_vec(), vec![Triangle::new(0, 0, 1)]);
    assert_invalid(
        &mesh,
        HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0,
        },
    );
}

#[test]
fn unbalanced_input_survives_bounds_and_boundary_shortcuts() {
    let mut mesh = tetrahedron();
    Arc::make_mut(&mut mesh.triangles)[0] = Triangle::new(0, 1, 2);
    assert_invalid(
        &mesh,
        HypermeshError::NonPwnInput {
            mesh_index: 0,
            unbalanced_edges: 3,
        },
    );
}

#[test]
fn warmed_validation_does_not_survive_geometry_mutation() {
    let mesh = tetrahedron();
    mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    let mut triangles_changed = mesh.clone();
    triangles_changed.triangles = mesh.triangles[..3].to_vec().into();
    assert_invalid(
        &triangles_changed,
        HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3,
        },
    );
    let mut positions_changed = mesh.clone();
    Arc::make_mut(&mut positions_changed.positions)[1] = point(0, 0, 0);
    assert_invalid(
        &positions_changed,
        HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0,
        },
    );
    assert!(
        mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .is_ok()
    );
}

#[test]
fn closed_tetrahedron_answers_match_independent_halfspace_expectations() {
    let mesh = tetrahedron();
    let eighth = (Real::one() / Real::from(8)).unwrap();
    // The exact interior is x,y,z > 0 and x+y+z < 1.
    let queries = [
        (Point3::new(eighth.clone(), eighth.clone(), eighth), true),
        (point(1, 1, 1), false),
        (point(0, 0, 0), false),
        (point(10, 10, 10), false),
    ];
    for input in [mesh.clone(), mesh.reversed_winding()] {
        for context in [
            GeometryContext::STRICT,
            GeometryContext::APPROXIMATE_512,
            GeometryContext::STRICT,
        ] {
            for (query, expected) in &queries {
                let result =
                    solid::contains_point_with_context(&input, query, &context).unwrap();
                assert_eq!(result.value, *expected);
                assert_eq!(result.certainty, GeometryCertainty::Certified);
            }
        }
    }
}

#[test]
fn simultaneous_clones_reuse_only_valid_source_facts() {
    let mesh = tetrahedron();
    let workers = (0..4)
        .map(|_| {
            let mesh = mesh.clone();
            std::thread::spawn(move || {
                for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                    let result = mesh
                        .contains_point(&context.mesh_context(), &point(10, 10, 10))
                        .unwrap();
                    assert!(!result.value);
                    assert_eq!(result.certainty, MeshCertainty::Certified);
                }
            })
        })
        .collect::<Vec<_>>();
    for worker in workers {
        worker.join().unwrap();
    }
}

#[test]
fn approximate_topology_fact_keeps_its_receipt_and_cannot_certify_strict_queries() {
    let source = tetrahedron();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero =
        uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one());
    let mut positions = source.positions.to_vec();
    positions.push(Point3::new(unresolved_zero, Real::zero(), Real::zero()));
    let mut triangles = source.triangles.to_vec();
    triangles[0] = Triangle::new(4, 2, 1);
    let mesh = TriangleMesh::new(positions, triangles);
    let query = point(10, 10, 10);
    // A duplicated corner needs a policy decision to close its geometric seam.
    assert!(matches!(
        mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context()),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    assert_eq!(
        mesh.validate_closed_pwn(&GeometryContext::APPROXIMATE_512.mesh_context())
            .unwrap()
            .certainty,
        MeshCertainty::Approximate512Consumed
    );
    for _ in 0..2 {
        assert!(matches!(
            solid::contains_point(&mesh, &query),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        let result = solid::contains_point_with_context(
            &mesh,
            &query,
            &GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert!(!result.value);
        assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
    }
}
