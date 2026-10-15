//! Regularized-set identities and downstream use of canonical empty solids.
//!
//! Box expectations are derived from their exact boundary planes and volume,
//! independently of the Boolean implementation and its triangle ordering.

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    adapter::RawTriangleMesh,
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hypermesh::{BooleanExpression, BooleanOp, BooleanProgram, HypermeshError, Triangle};

const OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Intersection,
    BooleanOp::Difference,
    BooleanOp::SymmetricDifference,
];

fn assert_empty(mesh: &TriangleMesh) {
    assert!(mesh.positions.is_empty());
    assert!(mesh.triangles.is_empty());
    assert!(!solid::contains_point(mesh, &Point3::origin()).unwrap());
}

fn assert_box(mesh: &TriangleMesh, min: [i32; 3], max: [i32; 3]) {
    assert!(!mesh.triangles.is_empty());
    let lower = min.map(Real::from);
    let upper = max.map(Real::from);
    let mut six_volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|i| &mesh.positions[i]);
        let coordinates = points.map(|p| [&p.x, &p.y, &p.z]);
        for point in coordinates {
            for axis in 0..3 {
                assert!(point[axis] >= &lower[axis] && point[axis] <= &upper[axis]);
            }
        }
        assert!((0..3).any(|axis| {
            coordinates.iter().all(|p| p[axis] == &lower[axis])
                || coordinates.iter().all(|p| p[axis] == &upper[axis])
        }));
        let [a, b, c] = points.map(|p| p.to_vector());
        six_volume += a.dot(&b.cross(&c));
    }
    let expected = 6 * (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2]);
    assert_eq!(six_volume, Real::from(expected));
    let closure = mesh
        .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    assert!(closure.value);
    assert_eq!(closure.certainty, hypermesh::MeshCertainty::Certified);
}

#[test]
fn empty_identities_remain_composable_under_both_policies() {
    let empty = solid::empty();
    let cube = solid::cube(Real::from(2));
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for operation in OPERATIONS {
            let outcome = solid::boolean_with_context(&empty, &empty, operation, &context)
                .expect("two canonical empty sets compose");
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert_empty(&outcome.value);
            assert_box(&outcome.value.try_union(&cube).unwrap(), [0; 3], [2; 3]);

            for empty_on_left in [false, true] {
                let (left, right) = if empty_on_left {
                    (&empty, &cube)
                } else {
                    (&cube, &empty)
                };
                let outcome = solid::boolean_with_context(left, right, operation, &context)
                    .expect("canonical empty is a supported modeling operand");
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                if operation == BooleanOp::Intersection
                    || (operation == BooleanOp::Difference && empty_on_left)
                {
                    assert_empty(&outcome.value);
                    assert_box(&outcome.value.try_union(&cube).unwrap(), [0; 3], [2; 3]);
                } else {
                    assert_box(&outcome.value, [0; 3], [2; 3]);
                    assert_empty(&outcome.value.try_difference(&cube).unwrap());
                }
            }
        }
    }
}

#[test]
fn accumulation_and_complete_subtraction_can_continue() {
    let a = solid::cube(Real::from(2));
    let b = a.translated(Real::from(2), Real::zero(), Real::zero());
    let mut accumulated = solid::empty();
    for part in [&a, &b] {
        accumulated = accumulated.try_union(part).unwrap();
    }
    assert_box(&accumulated, [0; 3], [4, 2, 2]);
    let remainder = accumulated.try_difference(&a).unwrap();
    assert_box(&remainder, [2, 0, 0], [4, 2, 2]);
    let removed = remainder.try_difference(&b).unwrap();
    assert_empty(&removed);
    assert_empty(&removed.try_difference(&a).unwrap());
    assert_empty(&a.try_intersection(&removed).unwrap());
    assert_box(&removed.try_union(&a).unwrap(), [0; 3], [2; 3]);

    let self_removed = a.try_difference(&a).unwrap();
    assert_empty(&self_removed);
    let transformed =
        solid::try_scale(&self_removed, Real::from(2), Real::one(), Real::one()).unwrap();
    assert_box(&transformed.try_union(&b).unwrap(), [2, 0, 0], [4, 2, 2]);
}

#[test]
fn empty_shortcuts_preserve_invalid_operand_diagnostics_and_indices() {
    let cube = solid::cube(Real::one());
    let mut reversed_face = cube.triangles.to_vec();
    let [a, b, c] = reversed_face[0].indices();
    reversed_face[0] = Triangle::new(a, c, b);
    let invalid = [
        (TriangleMesh::new(vec![Point3::origin()], vec![]), 0),
        (TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]), 0),
        (
            TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 2)]),
            1,
        ),
        (
            TriangleMesh::new(cube.positions.to_vec(), vec![Triangle::new(0, 0, 0)]),
            2,
        ),
        (
            TriangleMesh::new(cube.positions.to_vec(), vec![cube.triangles[0]]),
            3,
        ),
        (TriangleMesh::new(cube.positions.to_vec(), reversed_face), 4),
    ];
    let empty = solid::empty();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for operation in OPERATIONS {
            for (mesh, kind) in &invalid {
                for mesh_index in [0, 1] {
                    let (left, right) = if mesh_index == 0 {
                        (mesh, &empty)
                    } else {
                        (&empty, mesh)
                    };
                    let error = solid::boolean_with_context(left, right, operation, &context)
                        .expect_err("an empty operand must not conceal malformed geometry");
                    match kind {
                        0 => assert_eq!(error, HypermeshError::EmptyMesh { mesh_index }),
                        1 => assert_eq!(
                            error,
                            HypermeshError::VertexIndexOutOfBounds {
                                index: 1,
                                vertex_count: 1,
                            }
                        ),
                        2 => assert_eq!(
                            error,
                            HypermeshError::DegenerateTriangle {
                                mesh_index,
                                triangle_index: 0,
                            }
                        ),
                        3 => assert_eq!(
                            error,
                            HypermeshError::OpenInput {
                                mesh_index,
                                boundary_edges: 3
                            }
                        ),
                        4 => assert!(matches!(error, HypermeshError::NonPwnInput {
                            mesh_index: index, unbalanced_edges: 3,
                        } if index == mesh_index)),
                        _ => unreachable!(),
                    }
                }
            }
        }
    }
}

#[test]
fn identity_regularizes_winding_multiplicity_and_cancellation() {
    let cube = solid::cube(Real::one());
    let doubled = TriangleMesh::new(
        cube.positions.to_vec(),
        cube.triangles
            .iter()
            .chain(cube.triangles.iter())
            .copied()
            .collect(),
    );
    let canceled = TriangleMesh::new(
        cube.positions.to_vec(),
        cube.triangles
            .iter()
            .chain(cube.reversed_winding().triangles.iter())
            .copied()
            .collect(),
    );
    for mesh in [&doubled, &cube.reversed_winding()] {
        let result = mesh.try_union(&solid::empty()).unwrap();
        assert_box(&result, [0; 3], [1; 3]);
        assert_empty(&result.try_difference(&cube).unwrap());
    }
    assert_empty(&canceled.try_union(&solid::empty()).unwrap());
}

#[test]
fn empty_identity_keeps_symbolic_coordinates() {
    let source = solid::cube(Real::one()).translated(Real::pi(), Real::zero(), Real::zero());
    let result = solid::empty().try_union(&source).unwrap();
    let left = Real::pi();
    let right = Real::pi() + Real::one();
    assert!(!result.positions.is_empty());
    assert!(result.positions.iter().all(|p| p.x == left || p.x == right));
    assert_empty(&result.try_difference(&source).unwrap());
}

#[test]
fn empty_operand_does_not_erase_uncertainty_in_input_validation() {
    let cube = solid::cube(Real::one());
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let mut positions = cube.positions.to_vec();
    positions[1].x += unresolved_zero;
    let input = TriangleMesh::new(positions, cube.triangles.to_vec());
    let empty = solid::empty();
    for operation in OPERATIONS {
        for (left, right) in [(&input, &empty), (&empty, &input)] {
            for context in [
                GeometryContext::STRICT,
                GeometryContext::APPROXIMATE_512,
                GeometryContext::STRICT,
            ] {
                let result = solid::boolean_with_context(left, right, operation, &context);
                if context == GeometryContext::STRICT {
                    assert!(
                        matches!(
                            result,
                            Err(HypermeshError::PredicateUndecided { .. }
                                | HypermeshError::UnknownClassification)
                        ),
                        "{result:?}"
                    );
                } else {
                    assert_eq!(
                        result.unwrap().certainty,
                        GeometryCertainty::Approximate512Consumed
                    );
                }
            }
        }
    }
}

#[test]
fn scalar_adapter_reuses_empty_composition_and_visible_errors() {
    let cube = RawTriangleMesh::cube(Real::one()).unwrap();
    let removed = cube.difference(&cube).unwrap();
    assert_empty(removed.native());
    let restored = removed.union(&cube).unwrap();
    assert_box(restored.native(), [0; 3], [1; 3]);
    let invalid = RawTriangleMesh::from_native(TriangleMesh::new(
        vec![Point3::origin()],
        vec![Triangle::new(0, 1, 2)],
    ));
    let error = removed.intersection(&invalid).unwrap_err();
    assert_eq!(
        error.to_string(),
        "vertex index 1 is out of bounds for 1 vertices"
    );
}

#[test]
fn empty_rows_from_a_shared_boolean_batch_can_be_composed() {
    let cube = solid::cube(Real::one());
    let batch = hypermesh::boolean(
        &GeometryContext::STRICT.mesh_context(),
        &[cube.as_ref()],
        BooleanProgram::Expressions {
            nodes: &[BooleanExpression::Operand(0), BooleanExpression::False],
            roots: &[0, 1],
        },
    )
    .unwrap()
    .value;
    assert!(!batch.vertices.is_empty());
    let meshes = batch.into_triangle_meshes().unwrap();
    assert_box(&meshes[0], [0; 3], [1; 3]);
    assert_empty(&meshes[1]);
    assert_box(&meshes[1].try_union(&cube).unwrap(), [0; 3], [1; 3]);
}

#[test]
fn holes_and_disconnected_components_survive_empty_composition() {
    let outer = solid::cube(Real::from(4));
    let inner = solid::cube(Real::from(2)).translated(Real::one(), Real::one(), Real::one());
    let hollow = outer.try_difference(&inner).unwrap();
    let left = solid::cube(Real::from(2));
    let right = left.translated(Real::from(4), Real::zero(), Real::zero());
    let disconnected = left.try_union(&right).unwrap();
    for (source, volume, samples) in [
        (
            &hollow,
            56,
            [([1, 4, 4], true), ([4, 4, 4], false), ([10, 4, 4], false)],
        ),
        (
            &disconnected,
            16,
            [([2, 2, 2], true), ([6, 2, 2], false), ([10, 2, 2], true)],
        ),
    ] {
        let result = solid::empty().try_xor(source).unwrap();
        let mut six_volume = Real::zero();
        for triangle in result.triangles.iter() {
            let [a, b, c] = triangle.indices().map(|i| result.positions[i].to_vector());
            six_volume += a.dot(&b.cross(&c));
        }
        assert_eq!(six_volume, Real::from(6 * volume));
        assert!(
            result
                .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        for (twice_coordinates, inside) in samples {
            // Exact half-integer probes, all away from the boundary.
            let [x, y, z] = twice_coordinates
                .map(|coordinate| (Real::from(coordinate) / Real::from(2)).unwrap());
            assert_eq!(
                solid::contains_point(&result, &Point3::new(x, y, z)).unwrap(),
                inside
            );
        }
        assert_empty(&result.try_difference(source).unwrap());
    }
}

#[test]
fn warming_then_mutating_an_operand_cannot_hide_invalid_topology() {
    let mut source = solid::cube(Real::one());
    let empty = solid::empty();
    assert_box(&source.try_union(&empty).unwrap(), [0; 3], [1; 3]);
    source.triangles = vec![source.triangles[0]].into();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for operation in OPERATIONS {
            assert_eq!(
                solid::boolean_with_context(&empty, &source, operation, &context).unwrap_err(),
                HypermeshError::OpenInput {
                    mesh_index: 1,
                    boundary_edges: 3
                }
            );
        }
    }
}
