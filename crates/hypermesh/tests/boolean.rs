use std::collections::BTreeMap;

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
