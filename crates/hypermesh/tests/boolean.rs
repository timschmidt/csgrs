#[path = "../src/test_uncertainty.rs"]
mod test_uncertainty;

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

fn terminally_unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    test_uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
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
    for result in outcome.value.results.iter() {
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
    let left_boundary = Real::e().sin();
    let right_boundary = left_boundary.clone() + terminally_unresolved_zero();
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
fn approximate_results_do_not_certify_their_structure_for_later_operations() {
    // A slot through a block, with its front face 2^-600 behind the block's
    // face x = 0: the offset is u + 2^-600, where u is an exact zero that no
    // proof decides. Strict policy cannot separate the two faces, so it
    // cannot build the difference; APPROXIMATE_512 builds it by consuming
    // its terminal. That result's claim that its triangles meet only in
    // shared vertices or edges rests on the consumed decision. A later strict
    // Boolean must decide the retained result as it decides the same rows
    // rebuilt, and an approximate Boolean that uses the claim must report the
    // consumed terminal.
    let tiny = (0..15).fold(Real::one(), |tiny, _| {
        (tiny / Real::from(1_i64 << 40)).unwrap()
    });
    let offset = &tiny + &terminally_unresolved_zero();
    let block = integer_box([0, 0, 0], [8, 8, 4]);
    let slot = integer_box([0, 2, -1], [5, 6, 5]);
    let slot = TriangleMesh::new(
        slot.positions
            .iter()
            .map(|point| Point3::new(&point.x + &offset, point.y.clone(), point.z.clone()))
            .collect(),
        slot.triangles.to_vec(),
    );
    let strict = MeshContext::new(PredicatePolicy::STRICT);
    let approximate = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let difference = BooleanProgram::Operation(BooleanOp::Difference);
    assert!(matches!(
        run(&strict, &[&block, &slot], difference),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    let outcome = run(&approximate, &[&block, &slot], difference).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
    let slotted = outcome.value.into_triangle_meshes().unwrap().pop().unwrap();
    let rebuilt = TriangleMesh::new(slotted.positions.to_vec(), slotted.triangles.to_vec());
    let far = integer_box([20, 20, 20], [21, 21, 21]);
    let union = BooleanProgram::Operation(BooleanOp::Union);
    for body in [&slotted, &rebuilt] {
        assert!(matches!(
            run(&strict, &[body, &far], union),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
    }
    let outcome = run(&approximate, &[&slotted, &far], union).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Approximate512Consumed);
    assert_eq!(
        outcome.value.results[0].triangles.len(),
        slotted.triangles.len() + 12
    );
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

fn batch_box_cells() -> Vec<([i32; 3], [i32; 3])> {
    let mut cells = Vec::new();
    for (x, a) in [(0, 2), (1, 3), (3, 2)] {
        for (y, b) in [(0, 2), (1, 3), (3, 2)] {
            for (z, c) in [(0, 2), (1, 3), (3, 2)] {
                if (x, y, z) != (1, 1, 1) {
                    cells.push(([x, y, z], [x + a, y + b, z + c]));
                }
            }
        }
    }
    cells
}

fn point_axis(point: &Point3, axis: usize) -> &Real {
    match axis {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => unreachable!(),
    }
}

fn assert_batch_box_union(
    context: &MeshContext,
    batch: BooleanMeshBatch,
    cells: &[([i32; 3], [i32; 3])],
    volume: i32,
) -> TriangleMesh {
    assert_certified_boundary(&batch, &batch.results[0]);
    assert_eq!(
        signed_six_volume(&batch.vertices, &batch.results[0]),
        r(6 * volume)
    );
    assert!(
        batch
            .results
            .iter()
            .skip(1)
            .all(|result| result.triangles.is_empty())
    );
    // All boundary faces lie on the integer coordinate planes. Together with
    // closed topology and exact bounds, one witness in every open unit cell
    // determines this regularized set; no triangle count is a semantic oracle.
    for triangle in &batch.results[0].triangles {
        assert!((0..3).any(|axis| (0..=5).any(|value| {
            triangle
                .iter()
                .all(|&vertex| point_axis(&batch.vertices[vertex as usize], axis) == &r(value))
        })));
    }
    let mut meshes = batch.into_triangle_meshes().unwrap();
    let mesh = meshes.remove(0);
    assert_eq!(
        mesh.validate_closed_pwn(context).unwrap().certainty,
        MeshCertainty::Certified
    );
    let bounds = mesh.exact_bounds(context).unwrap();
    assert_eq!(bounds.certainty, MeshCertainty::Certified);
    let bounds = bounds.value.unwrap();
    for axis in 0..3 {
        let min = cells.iter().map(|(min, _)| min[axis]).min().unwrap();
        let max = cells.iter().map(|(_, max)| max[axis]).max().unwrap();
        assert_eq!(point_axis(&bounds.mins, axis), &r(min));
        assert_eq!(point_axis(&bounds.maxs, axis), &r(max));
    }
    let mut occupied = 0;
    for x in 0..5 {
        for y in 0..5 {
            for z in 0..5 {
                let cell = [x, y, z];
                let expected = cells.iter().any(|(min, max)| {
                    (0..3).all(|axis| min[axis] <= cell[axis] && cell[axis] < max[axis])
                });
                occupied += i32::from(expected);
                let [x, y, z] = cell.map(|value| (r(2 * value + 1) / r(2)).unwrap());
                let outcome = mesh.contains_point(context, &Point3::new(x, y, z)).unwrap();
                assert_eq!(outcome.certainty, MeshCertainty::Certified);
                assert_eq!(outcome.value, expected, "unit cell {cell:?}");
            }
        }
    }
    assert_eq!(occupied, volume);
    mesh
}

fn check_batch_box_common_cells(indices: &[usize], volume: i32) {
    let context = MeshContext::new(PredicatePolicy::STRICT);
    let all_cells = batch_box_cells();
    let cells = indices
        .iter()
        .map(|&index| all_cells[index])
        .collect::<Vec<_>>();
    let original = cells
        .iter()
        .map(|&(min, max)| {
            let mesh = integer_box(min, max);
            // Retain the original CSGRS cuboid face order used by the full failure.
            let triangles = [2, 3, 0, 1, 10, 11, 8, 9, 6, 7, 4, 5]
                .map(|index| mesh.triangles[index])
                .to_vec();
            TriangleMesh::new(mesh.positions.to_vec(), triangles)
        })
        .collect::<Vec<_>>();
    for variant in 0..3 {
        let mut inputs = original.clone();
        if variant == 1 {
            inputs.reverse();
            inputs = inputs
                .into_iter()
                .enumerate()
                .map(|(index, mesh)| with_rotated_triangle_order(mesh, index + 1))
                .collect();
        } else if variant == 2 {
            inputs = inputs.iter().map(TriangleMesh::reversed_winding).collect();
        }
        for merged in [false, true] {
            let operands = if merged {
                vec![TriangleMesh::try_concatenate(&inputs).unwrap()]
            } else {
                inputs.clone()
            };
            for operand in &operands {
                assert_eq!(
                    operand.validate_closed_pwn(&context).unwrap().certainty,
                    MeshCertainty::Certified
                );
            }
            let nodes = [
                BooleanExpression::Operation(BooleanOp::Union),
                BooleanExpression::Operation(BooleanOp::Intersection),
                BooleanExpression::False,
            ];
            let roots: &[u32] = if merged { &[0, 2] } else { &[0, 1, 2] };
            let references = operands.iter().collect::<Vec<_>>();
            let outcome = run(
                &context,
                &references,
                BooleanProgram::Expressions {
                    nodes: &nodes,
                    roots,
                },
            )
            .unwrap_or_else(|error| {
                panic!("indices={indices:?} variant={variant} merged={merged}: {error:?}")
            });
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            let result = assert_batch_box_union(&context, outcome.value, &cells, volume);
            if variant == 0 && !merged {
                let enclosing = integer_box([0, 0, 0], [5, 5, 5]);
                let retained = run(
                    &context,
                    &[&result, &enclosing],
                    BooleanProgram::Operation(BooleanOp::Intersection),
                )
                .unwrap();
                assert_eq!(retained.certainty, MeshCertainty::Certified);
                assert_batch_box_union(&context, retained.value, &cells, volume);
                let removed = run(
                    &context,
                    &[&result, &result],
                    BooleanProgram::Operation(BooleanOp::Difference),
                )
                .unwrap();
                assert_eq!(removed.certainty, MeshCertainty::Certified);
                assert!(removed.value.results[0].triangles.is_empty());
            }
        }
    }
}

#[test]
fn batch_box_common_cells_complete_the_minimized_five_box_union() {
    // Independently, horizontal areas are 14, 16 and 8 over heights 1, 1 and 2.
    check_batch_box_common_cells(&[12, 17, 20, 21, 24], 46);
}

#[test]
fn batch_box_common_cells_complete_the_original_twenty_six_box_cavity() {
    // [0,5]^3 minus the open unit cavity (2,3)^3, with regularized boundaries.
    check_batch_box_common_cells(&(0..26).collect::<Vec<_>>(), 124);
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

/// A square prism `[|x| + |y| <= 8] x [0, 40]` with the triangles of csgrs's
/// `cylinder(8, 40, 4)`, its four samples counterclockwise from `(0, -8)`.
fn square_prism(transform: impl Fn([i32; 3]) -> Point3) -> TriangleMesh {
    let mut positions = vec![transform([0, 0, 0]), transform([0, 0, 40])];
    for z in [0, 40] {
        for [x, y] in [[0, -8], [8, 0], [0, 8], [-8, 0]] {
            positions.push(transform([x, y, z]));
        }
    }
    let mut triangles = Vec::new();
    for index in 0..4 {
        let next = (index + 1) % 4;
        triangles.push(Triangle::new(0, 2 + next, 2 + index));
        triangles.push(Triangle::new(1, 6 + index, 6 + next));
        triangles.push(Triangle::new(2 + index, 2 + next, 6 + next));
        triangles.push(Triangle::new(2 + index, 6 + next, 6 + index));
    }
    TriangleMesh::new(positions, triangles)
}

#[test]
fn construction_incidences_decide_a_near_coincident_rotated_prism() {
    // The prism minus itself rotated by 1/10000 degree about y. A side face of
    // the first meets the rotated top cap's two triangles along one line, and
    // the arrangement puts three points on it: two built on those triangles'
    // planes and one on their shared edge. The cap triangles were decided
    // coplanar, so the points are collinear by construction, but the zero is
    // beyond every proof the scalar layer has. Strict policy used to stop at
    // `surface arrangement point on segment`.
    let angle = (Real::pi() / Real::from(1_800_000)).unwrap();
    let (sine, cosine) = (angle.clone().sin(), angle.cos());
    let prism = square_prism(|[x, y, z]| Point3::new(r(x), r(y), r(z)));
    let rotated = square_prism(|[x, y, z]| {
        Point3::new(
            &cosine * &r(x) + &sine * &r(z),
            r(y),
            &cosine * &r(z) - &sine * &r(x),
        )
    });
    let strict = MeshContext::new(PredicatePolicy::STRICT);
    // Six times the prism's volume, 128 * 40; the rotated prism's is equal.
    let full = r(30720);
    for operation in [
        BooleanOp::Difference,
        BooleanOp::Union,
        BooleanOp::Intersection,
    ] {
        let outcome = run(
            &strict,
            &[&prism, &rotated],
            BooleanProgram::Operation(operation),
        )
        .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
        assert_eq!(outcome.certainty, MeshCertainty::Certified, "{operation:?}");
        let result = &outcome.value.results[0];
        assert_certified_boundary(&outcome.value, result);
        let volume = signed_six_volume(&outcome.value.vertices, result);
        let ordering = |value: &Real| {
            hyperlimit::compare_reals(value, &full, PredicatePolicy::STRICT)
                .value()
                .expect("volume comparison decides")
        };
        let positive = hyperlimit::classify_real_sign(&volume, PredicatePolicy::STRICT)
            .value()
            .expect("volume sign decides");
        assert_eq!(positive, hyperlimit::Sign::Positive, "{operation:?}");
        let expected = match operation {
            BooleanOp::Union => std::cmp::Ordering::Greater,
            _ => std::cmp::Ordering::Less,
        };
        assert_eq!(ordering(&volume), expected, "{operation:?}");
    }
}
