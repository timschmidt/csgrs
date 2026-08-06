use std::collections::BTreeMap;
use std::num::NonZeroU32;

use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanMeshResult, BooleanOp, BooleanProgram,
    HypermeshError, MeshCertainty, MeshContext, Point3, PredicatePolicy, Real, Triangle,
    TriangleMesh,
};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn standard_box_triangles() -> Vec<Triangle> {
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
    ]
}

fn exact_box(min: [Real; 3], max: [Real; 3]) -> TriangleMesh {
    TriangleMesh::new(
        vec![
            Point3::new(min[0].clone(), min[1].clone(), min[2].clone()),
            Point3::new(max[0].clone(), min[1].clone(), min[2].clone()),
            Point3::new(max[0].clone(), max[1].clone(), min[2].clone()),
            Point3::new(min[0].clone(), max[1].clone(), min[2].clone()),
            Point3::new(min[0].clone(), min[1].clone(), max[2].clone()),
            Point3::new(max[0].clone(), min[1].clone(), max[2].clone()),
            Point3::new(max[0].clone(), max[1].clone(), max[2].clone()),
            Point3::new(min[0].clone(), max[1].clone(), max[2].clone()),
        ],
        standard_box_triangles(),
    )
}

fn integer_box(min: [i32; 3], max: [i32; 3]) -> TriangleMesh {
    exact_box(min.map(r), max.map(r))
}

fn integer_octahedron(center: [i32; 3], radius: [i32; 3]) -> TriangleMesh {
    let [x, y, z] = center;
    TriangleMesh::new(
        vec![
            Point3::new(r(x + radius[0]), r(y), r(z)),
            Point3::new(r(x - radius[0]), r(y), r(z)),
            Point3::new(r(x), r(y + radius[1]), r(z)),
            Point3::new(r(x), r(y - radius[1]), r(z)),
            Point3::new(r(x), r(y), r(z + radius[2])),
            Point3::new(r(x), r(y), r(z - radius[2])),
        ],
        vec![
            Triangle::new(0, 2, 4),
            Triangle::new(2, 1, 4),
            Triangle::new(1, 3, 4),
            Triangle::new(3, 0, 4),
            Triangle::new(2, 0, 5),
            Triangle::new(1, 2, 5),
            Triangle::new(3, 1, 5),
            Triangle::new(0, 3, 5),
        ],
    )
}

fn integer_tetrahedron(origin: [i32; 3], extent: [i32; 3]) -> TriangleMesh {
    let [x, y, z] = origin;
    TriangleMesh::new(
        vec![
            Point3::new(r(x), r(y), r(z)),
            Point3::new(r(x + extent[0]), r(y), r(z)),
            Point3::new(r(x), r(y + extent[1]), r(z)),
            Point3::new(r(x), r(y), r(z + extent[2])),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn with_rotated_triangle_order(mesh: TriangleMesh, amount: usize) -> TriangleMesh {
    let mut triangles = mesh.triangles.to_vec();
    let count = triangles.len();
    triangles.rotate_left(amount % count);
    TriangleMesh::new(mesh.positions.to_vec(), triangles)
}

fn with_cycled_triangle_vertices(mesh: TriangleMesh) -> TriangleMesh {
    TriangleMesh::new(
        mesh.positions.to_vec(),
        mesh.triangles
            .iter()
            .map(|triangle| Triangle::new(triangle.v1, triangle.v2, triangle.v0))
            .collect(),
    )
}

fn combine_meshes(meshes: &[TriangleMesh]) -> TriangleMesh {
    let position_count = meshes.iter().map(|mesh| mesh.positions.len()).sum();
    let triangle_count = meshes.iter().map(|mesh| mesh.triangles.len()).sum();
    let mut positions = Vec::with_capacity(position_count);
    let mut triangles = Vec::with_capacity(triangle_count);
    for mesh in meshes {
        let base = positions.len();
        positions.extend(mesh.positions.iter().cloned());
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            Triangle::new(base + triangle.v0, base + triangle.v1, base + triangle.v2)
        }));
    }
    TriangleMesh::new(positions, triangles)
}

fn run(
    context: &MeshContext,
    meshes: &[&TriangleMesh],
    program: BooleanProgram<'_>,
) -> Result<hypermesh::MeshOutcome<BooleanMeshBatch>, HypermeshError> {
    let views = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
    hypermesh::boolean(context, &views, program)
}

fn signed_six_volume(vertices: &[Point3], result: &BooleanMeshResult) -> Real {
    let mut volume = Real::zero();
    for &[a, b, c] in &result.triangles {
        let a = &vertices[a as usize];
        let b = &vertices[b as usize];
        let c = &vertices[c as usize];
        volume += &a.x * &(&b.y * &c.z - &b.z * &c.y)
            + &a.y * &(&b.z * &c.x - &b.x * &c.z)
            + &a.z * &(&b.x * &c.y - &b.y * &c.x);
    }
    volume
}

fn assert_certified_boundary(batch: &BooleanMeshBatch, result: &BooleanMeshResult) {
    assert_eq!(result.triangles.len(), result.sources.len());
    let mut directed = BTreeMap::<[u32; 2], [usize; 2]>::new();
    for triangle in &result.triangles {
        assert!(
            triangle
                .iter()
                .all(|&vertex| (vertex as usize) < batch.vertices.len())
        );
        assert!(triangle[0] != triangle[1]);
        assert!(triangle[1] != triangle[2]);
        assert!(triangle[0] != triangle[2]);
        for [start, end] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let edge = if start < end {
                [start, end]
            } else {
                [end, start]
            };
            directed.entry(edge).or_default()[usize::from(start > end)] += 1;
        }
    }
    assert!(directed.values().all(|uses| uses[0] == uses[1]));
    assert!(
        result
            .sources
            .iter()
            .all(|source| matches!(source.orientation, -1 | 1))
    );
}

#[test]
fn overlapping_boxes_materialize_every_operation_under_both_policies() {
    let left = integer_box([0, 0, 0], [2, 2, 2]);
    let right = integer_box([1, 1, 1], [3, 3, 3]);
    let expected = [
        (BooleanOp::Union, 90),
        (BooleanOp::Intersection, 6),
        (BooleanOp::Difference, 42),
        (BooleanOp::SymmetricDifference, 84),
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for (operation, volume) in expected {
            let outcome = run(
                &context,
                &[&left, &right],
                BooleanProgram::Operation(operation),
            )
            .unwrap();
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            assert_eq!(outcome.value.results.len(), 1);
            let result = &outcome.value.results[0];
            assert!(!result.exterior_inside);
            assert_certified_boundary(&outcome.value, result);
            assert_eq!(
                signed_six_volume(&outcome.value.vertices, result),
                r(volume)
            );
        }
    }
}

#[test]
fn one_program_reuses_the_arrangement_for_all_builtin_results() {
    let left = integer_box([0, 0, 0], [2, 2, 2]);
    let right = integer_box([1, 1, 1], [3, 3, 3]);
    let nodes = [
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
        BooleanExpression::Operation(BooleanOp::Difference),
        BooleanExpression::Operation(BooleanOp::SymmetricDifference),
    ];
    let roots = [0, 1, 2, 3];
    let context = MeshContext::new(PredicatePolicy::STRICT);
    let outcome = run(
        &context,
        &[&left, &right],
        BooleanProgram::Expressions {
            nodes: &nodes,
            roots: &roots,
        },
    )
    .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    assert_eq!(outcome.value.results.len(), 4);
    for (result, expected) in outcome.value.results.iter().zip([90, 6, 42, 84]) {
        assert_certified_boundary(&outcome.value, result);
        assert_eq!(
            signed_six_volume(&outcome.value.vertices, result),
            r(expected)
        );
    }
    let mut globally_used = vec![false; outcome.value.vertices.len()];
    for result in &outcome.value.results {
        for &vertex in result.triangles.iter().flatten() {
            globally_used[vertex as usize] = true;
        }
    }
    assert!(globally_used.into_iter().all(|used| used));
}

#[test]
fn arbitrary_truth_dag_handles_reverse_difference_and_unbounded_complement() {
    let left = integer_box([0, 0, 0], [2, 2, 2]);
    let right = integer_box([1, 1, 1], [3, 3, 3]);
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
    let context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let outcome = run(
        &context,
        &[&left, &right],
        BooleanProgram::Expressions {
            nodes: &nodes,
            roots: &roots,
        },
    )
    .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    assert_eq!(outcome.value.results.len(), 3);
    assert_eq!(
        signed_six_volume(&outcome.value.vertices, &outcome.value.results[0]),
        r(42)
    );
    assert_eq!(
        signed_six_volume(&outcome.value.vertices, &outcome.value.results[1]),
        r(42)
    );
    assert_eq!(
        signed_six_volume(&outcome.value.vertices, &outcome.value.results[2]),
        r(-90)
    );
    assert!(!outcome.value.results[0].exterior_inside);
    assert!(!outcome.value.results[1].exterior_inside);
    assert!(outcome.value.results[2].exterior_inside);
    assert_eq!(
        outcome.value.into_triangle_meshes().unwrap_err(),
        HypermeshError::UnboundedBooleanOutput { output: 2 }
    );
}

#[test]
fn unary_disjoint_containment_and_contact_paths_are_total() {
    let outer = integer_box([0, 0, 0], [4, 4, 4]);
    let inner = integer_box([1, 1, 1], [2, 2, 2]);
    let disjoint = integer_box([6, 0, 0], [7, 1, 1]);
    let touching = integer_box([4, 1, 1], [5, 2, 2]);
    let context = MeshContext::new(PredicatePolicy::STRICT);

    let unary = run(
        &context,
        &[&outer],
        BooleanProgram::Operation(BooleanOp::Union),
    )
    .unwrap();
    assert_eq!(
        signed_six_volume(&unary.value.vertices, &unary.value.results[0]),
        r(384)
    );

    for (right, expected) in [
        (&inner, [384, 6, 378, 378]),
        (&disjoint, [390, 0, 384, 390]),
        (&touching, [390, 0, 384, 390]),
    ] {
        for (operation, volume) in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ]
        .into_iter()
        .zip(expected)
        {
            let outcome = run(
                &context,
                &[&outer, right],
                BooleanProgram::Operation(operation),
            )
            .unwrap();
            let result = &outcome.value.results[0];
            assert_certified_boundary(&outcome.value, result);
            assert_eq!(
                signed_six_volume(&outcome.value.vertices, result),
                r(volume)
            );
        }
    }
}

#[test]
fn public_boolean_reports_every_program_and_input_rejection() {
    let context = MeshContext::new(PredicatePolicy::STRICT);
    let valid = integer_box([0, 0, 0], [1, 1, 1]);
    assert_eq!(
        hypermesh::boolean(&context, &[], BooleanProgram::Operation(BooleanOp::Union)).unwrap_err(),
        HypermeshError::EmptyInput
    );
    assert!(matches!(
        run(
            &context,
            &[&valid],
            BooleanProgram::Expressions {
                nodes: &[BooleanExpression::Operand(0)],
                roots: &[],
            },
        ),
        Err(HypermeshError::InvalidBooleanProgram { .. })
    ));
    assert!(matches!(
        run(
            &context,
            &[&valid],
            BooleanProgram::Expressions {
                nodes: &[BooleanExpression::Operand(1)],
                roots: &[0],
            },
        ),
        Err(HypermeshError::InvalidBooleanProgram { .. })
    ));
    assert!(matches!(
        run(
            &context,
            &[&valid],
            BooleanProgram::Expressions {
                nodes: &[BooleanExpression::Not(0)],
                roots: &[0],
            },
        ),
        Err(HypermeshError::InvalidBooleanProgram { .. })
    ));
    assert!(matches!(
        run(
            &context,
            &[&valid],
            BooleanProgram::Expressions {
                nodes: &[BooleanExpression::Operand(0)],
                roots: &[1],
            },
        ),
        Err(HypermeshError::InvalidBooleanProgram { .. })
    ));

    let empty = TriangleMesh::new(Vec::new(), Vec::new());
    assert_eq!(
        run(
            &context,
            &[&empty],
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .unwrap_err(),
        HypermeshError::EmptyMesh { mesh_index: 0 }
    );
    let open = TriangleMesh::new(
        vec![
            Point3::new(r(0), r(0), r(0)),
            Point3::new(r(1), r(0), r(0)),
            Point3::new(r(0), r(1), r(0)),
        ],
        vec![Triangle::new(0, 1, 2)],
    );
    assert_eq!(
        run(
            &context,
            &[&open],
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .unwrap_err(),
        HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3,
        }
    );
}

#[test]
fn boolean_terminal_equality_obeys_strict_and_approximate_512() {
    let left_boundary = Real::pi() + Real::e();
    let right_boundary = Real::e() + Real::pi();
    let left = exact_box(
        [&left_boundary - &Real::one(), Real::zero(), Real::zero()],
        [left_boundary, Real::one(), Real::one()],
    );
    let right = exact_box(
        [right_boundary.clone(), Real::zero(), Real::zero()],
        [&right_boundary + &Real::one(), Real::one(), Real::one()],
    );

    let strict = MeshContext::new(PredicatePolicy::STRICT);
    assert!(matches!(
        run(
            &strict,
            &[&left, &right],
            BooleanProgram::Operation(BooleanOp::Union),
        ),
        Err(HypermeshError::PredicateUndecided { .. })
    ));

    let approximate = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let outcome = run(
        &approximate,
        &[&left, &right],
        BooleanProgram::Operation(BooleanOp::Union),
    )
    .unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
    assert_certified_boundary(&outcome.value, &outcome.value.results[0]);
}

#[test]
fn same_operand_coplanar_shell_overlap_is_order_invariant() {
    let first = with_rotated_triangle_order(integer_box([-5, -4, -3], [-3, -1, -1]), 4);
    let second = with_rotated_triangle_order(integer_box([-5, -5, -5], [-3, -2, -1]), 8);
    let overlapping_shells = combine_meshes(&[first, second]);
    let octahedron = integer_octahedron([-2, -1, 0], [3, 3, 4]);
    // Inclusion-exclusion gives 76 - 125/288: the second box/octahedron
    // intersection and the three-way intersection are the same 1/288
    // tetrahedron. `signed_six_volume` is six times geometric volume.
    let expected_signed_six_volume =
        (r(21_763) / r(48)).expect("the exact volume denominator is nonzero");

    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        let forward = run(
            &context,
            &[&overlapping_shells, &octahedron],
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .unwrap();
        let reverse = run(
            &context,
            &[&octahedron, &overlapping_shells],
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .unwrap();

        assert_eq!(forward.certainty, MeshCertainty::Certified);
        assert_eq!(reverse.certainty, MeshCertainty::Certified);
        assert_certified_boundary(&forward.value, &forward.value.results[0]);
        assert_certified_boundary(&reverse.value, &reverse.value.results[0]);
        assert_eq!(
            signed_six_volume(&forward.value.vertices, &forward.value.results[0]),
            expected_signed_six_volume,
        );
        assert_eq!(
            signed_six_volume(&reverse.value.vertices, &reverse.value.results[0]),
            expected_signed_six_volume,
        );
    }
}

#[test]
fn subdivided_same_operand_overlap_has_empty_disjoint_intersection() {
    let first = with_rotated_triangle_order(integer_box([-5, -4, -3], [-3, -1, -1]), 4)
        .subdivide_triangles(NonZeroU32::MIN)
        .unwrap();
    let second = with_rotated_triangle_order(integer_box([-5, -4, -3], [-2, 0, -2]), 9);
    let overlapping_shells = combine_meshes(&[first, second]);
    let disjoint_octahedron = integer_octahedron([-2, -1, 0], [1, 1, 1]);

    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for operands in [
            [&overlapping_shells, &disjoint_octahedron],
            [&disjoint_octahedron, &overlapping_shells],
        ] {
            let outcome = run(
                &context,
                &operands,
                BooleanProgram::Operation(BooleanOp::Intersection),
            )
            .unwrap();
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            assert!(outcome.value.results[0].triangles.is_empty());
            assert_eq!(
                signed_six_volume(&outcome.value.vertices, &outcome.value.results[0]),
                Real::zero(),
            );
        }
    }
}

#[test]
fn corner_coincident_same_operand_shells_have_consistent_surface_cells() {
    let box_shell = integer_box([3, 4, 5], [7, 6, 6]);
    let tetrahedron = integer_tetrahedron([3, 4, 5], [3, 2, 3]);
    let overlapping_shell_orders = [
        combine_meshes(&[
            box_shell.clone(),
            with_rotated_triangle_order(tetrahedron.clone(), 3),
        ]),
        combine_meshes(&[
            with_rotated_triangle_order(tetrahedron.clone(), 1),
            with_rotated_triangle_order(box_shell.clone(), 7),
        ]),
        combine_meshes(&[
            with_cycled_triangle_vertices(with_rotated_triangle_order(box_shell, 5)),
            with_cycled_triangle_vertices(with_rotated_triangle_order(tetrahedron, 2)),
        ]),
    ];
    let cutter = with_rotated_triangle_order(integer_box([2, 3, 4], [4, 6, 8]), 4);
    let exact_fraction = |numerator, denominator| {
        (r(numerator) / r(denominator)).expect("the exact volume denominator is nonzero")
    };

    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for (shell_order, overlapping_shells) in overlapping_shell_orders.iter().enumerate() {
            for (reverse, operands) in [
                (false, [overlapping_shells, &cutter]),
                (true, [&cutter, overlapping_shells]),
            ] {
                for (operation, expected_signed_six_volume) in [
                    (BooleanOp::Union, exact_fraction(542, 3)),
                    (BooleanOp::Intersection, exact_fraction(50, 3)),
                    (
                        BooleanOp::Difference,
                        exact_fraction(if reverse { 382 } else { 110 }, 3),
                    ),
                    (BooleanOp::SymmetricDifference, r(164)),
                ] {
                    let outcome = run(&context, &operands, BooleanProgram::Operation(operation))
                        .unwrap_or_else(|error| {
                            panic!(
                                "{policy:?} shell_order={shell_order} reverse={reverse} \
                                 operation={operation:?} failed: {error:?}"
                            )
                        });
                    assert_eq!(outcome.certainty, MeshCertainty::Certified);
                    assert_certified_boundary(&outcome.value, &outcome.value.results[0]);
                    assert_eq!(
                        signed_six_volume(&outcome.value.vertices, &outcome.value.results[0]),
                        expected_signed_six_volume,
                    );
                }
            }
        }
    }
}

#[test]
fn subdivided_face_coincident_shell_stack_has_consistent_surface_cells() {
    let wide = integer_box([-4, -5, -5], [0, -1, -4])
        .subdivide_triangles(NonZeroU32::MIN)
        .unwrap();
    let adjoining = with_rotated_triangle_order(integer_box([-5, -5, -5], [-4, -2, -4]), 4);
    let stacked_shell_orders = [
        combine_meshes(&[wide.clone(), adjoining.clone()]),
        combine_meshes(&[
            with_cycled_triangle_vertices(adjoining),
            with_cycled_triangle_vertices(with_rotated_triangle_order(wide, 17)),
        ]),
    ];
    let coincident_subset = with_rotated_triangle_order(integer_box([-5, -5, -5], [-4, -4, -4]), 7);

    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for (shell_order, stacked_shells) in stacked_shell_orders.iter().enumerate() {
            for (reverse, operands) in [
                (false, [stacked_shells, &coincident_subset]),
                (true, [&coincident_subset, stacked_shells]),
            ] {
                for (operation, expected_signed_six_volume) in [
                    (BooleanOp::Union, r(114)),
                    (BooleanOp::Intersection, r(6)),
                    (BooleanOp::Difference, r(if reverse { 0 } else { 108 })),
                    (BooleanOp::SymmetricDifference, r(108)),
                ] {
                    let outcome = run(&context, &operands, BooleanProgram::Operation(operation))
                        .unwrap_or_else(|error| {
                            panic!(
                                "{policy:?} shell_order={shell_order} reverse={reverse} \
                             operation={operation:?} failed: {error:?}"
                            )
                        });
                    assert_eq!(outcome.certainty, MeshCertainty::Certified);
                    assert_certified_boundary(&outcome.value, &outcome.value.results[0]);
                    assert_eq!(
                        signed_six_volume(&outcome.value.vertices, &outcome.value.results[0]),
                        expected_signed_six_volume,
                    );
                }
            }
        }
    }
}
