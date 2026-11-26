//! Solid containment must validate its source before query shortcuts can answer.
#[path = "support/uncertainty.rs"]
/// Shared exact-zero control and independently unresolved identity for seam validation.
mod uncertainty;

use std::sync::Arc;

use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hypermesh::{HypermeshError, MeshCertainty, Point3, Triangle};

/// Imports three signed integer coordinates exactly.
fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

/// Builds the outward coordinate tetrahedron used by valid and deliberately corrupted fixtures.
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

/// Requires the same typed error at a distant query and the origin through native/modeling APIs.
/// Runs strict/approximate/strict contexts plus the ordinary strict wrapper so neither bounds
/// nor source-boundary shortcuts can conceal malformed input.
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
                solid::contains_point_with_context(mesh, &query, context),
                Err(expected.clone())
            );
        }
        assert_eq!(solid::contains_point(mesh, &query), Err(expected.clone()));
    }
}

#[test]
/// Requires certified false membership for both-empty-buffer geometry under either policy.
fn canonical_empty_membership_is_certified_false() {
    let mesh = TriangleMesh::new(Vec::new(), Vec::new());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let result =
            solid::contains_point_with_context(&mesh, &point(0, 0, 0), context).unwrap();
        assert!(!result.value);
        assert_eq!(result.certainty, GeometryCertainty::Certified);
    }
}

#[test]
/// Distinguishes position-only and triangle-only carriers from canonical empty via `EmptyMesh`.
fn missing_one_geometry_buffer_is_not_canonical_empty() {
    for mesh in [
        TriangleMesh::new(vec![point(0, 0, 0)], Vec::new()),
        TriangleMesh::new(Vec::new(), vec![Triangle::new(0, 1, 2)]),
    ] {
        assert_invalid(&mesh, HypermeshError::EmptyMesh { mesh_index: 0 });
    }
}

#[test]
/// Requires the out-of-range index and vertex count to survive all query shortcut paths.
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
/// Requires an isolated valid triangle to report three open edges before containment can answer.
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
/// Requires the mesh/triangle-indexed degenerate-face error for a repeated-vertex triangle.
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
/// Reverses one tetrahedron face and requires three unbalanced geometric edges to remain typed.
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
/// Warms native validation, then separately removes a cloned face and collapses a cloned position.
/// Requires fresh open/degenerate errors and verifies the unmodified source still validates.
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
/// Checks four points against independent tetrahedron halfspaces, including boundary exclusion.
/// Global winding reversal preserves these certified answers across strict/approximate/strict passes.
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
                    solid::contains_point_with_context(&input, query, context).unwrap();
                assert_eq!(result.value, *expected);
                assert_eq!(result.certainty, GeometryCertainty::Certified);
            }
        }
    }
}

#[test]
/// Spawns four cloned-source workers, each requiring certified exterior answers under both policies.
/// Joining every worker exposes panics; no barrier forces a particular cache interleaving.
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
/// Replaces one corner reference by an independently indexed unresolved-zero coordinate.
/// Approximate seam validation must retain qualification; repeated strict exterior queries still
/// fail rather than borrowing that fact, while approximate queries return qualified false.
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
            GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert!(!result.value);
        assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
    }
}
