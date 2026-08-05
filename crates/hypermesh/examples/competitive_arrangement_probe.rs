#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::{env, hint::black_box, time::Instant};

use competitive_support::{
    WIDE_RATIONAL_DIVISIONS, clipped_voxel_torus_case, corpus, deep_symbolic_translated_box_case,
    deep_symbolic_translation_depth, dense_coplanar_box_case, exact_mesh_pair, large_boolean_case,
    sparse_multishell_tetrahedra_case, transverse_self_pwn_cluster_case,
    wide_rational_overlapping_box_case, wide_rational_shift,
};
use hypermesh::{
    BooleanExpression, BooleanMeshBatch, BooleanOp, BooleanProgram, MeshContext, MeshOutcome,
    PredicatePolicy, TriangleMesh, boolean, polygon_soup,
};

#[derive(Clone, Copy)]
enum Workload {
    AllFour,
    AllFive,
    Operation(BooleanOp, bool),
}

fn run_operation(
    context: &MeshContext,
    inputs: &[TriangleMesh; 2],
    workload: Workload,
) -> MeshOutcome<BooleanMeshBatch> {
    if matches!(workload, Workload::AllFour | Workload::AllFive) {
        let nodes = [
            BooleanExpression::Operation(BooleanOp::Union),
            BooleanExpression::Operation(BooleanOp::Intersection),
            BooleanExpression::Operation(BooleanOp::Difference),
            BooleanExpression::Operand(0),
            BooleanExpression::Operand(1),
            BooleanExpression::Not(3),
            BooleanExpression::And([4, 5]),
            BooleanExpression::Operation(BooleanOp::SymmetricDifference),
        ];
        let roots: &[u32] = match workload {
            Workload::AllFour => &[0, 1, 2, 6],
            Workload::AllFive => &[0, 1, 2, 6, 7],
            Workload::Operation(_, _) => unreachable!(),
        };
        return boolean(
            context,
            &[inputs[0].as_ref(), inputs[1].as_ref()],
            BooleanProgram::Expressions {
                nodes: &nodes,
                roots,
            },
        )
        .expect("competitive shared arrangement failed");
    }
    let Workload::Operation(operation, reverse) = workload else {
        unreachable!()
    };
    let views = if reverse {
        [inputs[1].as_ref(), inputs[0].as_ref()]
    } else {
        [inputs[0].as_ref(), inputs[1].as_ref()]
    };
    boolean(context, &views, BooleanProgram::Operation(operation))
        .expect("competitive Boolean operation failed")
}

fn main() {
    let mut args = env::args().skip(1);
    let fixture = args
        .next()
        .expect("expected <fixture> <operation> <strict|approximate-512> <repetitions>");
    let operation_name = args
        .next()
        .expect("expected <fixture> <operation> <strict|approximate-512> <repetitions>");
    let policy_name = args
        .next()
        .expect("expected <fixture> <operation> <strict|approximate-512> <repetitions>");
    let repetitions = args
        .next()
        .expect("expected a positive repetition count")
        .parse::<usize>()
        .expect("repetitions must be an integer");
    assert!(
        args.next().is_none() && repetitions != 0,
        "expected <fixture> <operation> <strict|approximate-512> <positive repetitions>"
    );
    let policy = match policy_name.as_str() {
        "strict" => PredicatePolicy::STRICT,
        "approximate-512" => PredicatePolicy::APPROXIMATE_512,
        _ => panic!("policy must be strict or approximate-512"),
    };
    let workload = match operation_name.as_str() {
        "all" => Workload::AllFour,
        "all-five" => Workload::AllFive,
        "union" => Workload::Operation(BooleanOp::Union, false),
        "intersection" => Workload::Operation(BooleanOp::Intersection, false),
        "difference" => Workload::Operation(BooleanOp::Difference, false),
        "reverse-difference" => Workload::Operation(BooleanOp::Difference, true),
        "xor" => Workload::Operation(BooleanOp::SymmetricDifference, false),
        _ => panic!(
            "operation must be union, intersection, difference, reverse-difference, xor, all, or all-five"
        ),
    };

    let case = if let Some(depth) = deep_symbolic_translation_depth(&fixture) {
        deep_symbolic_translated_box_case(depth).0
    } else if let Some(shift) = wide_rational_shift(&fixture) {
        wide_rational_overlapping_box_case(WIDE_RATIONAL_DIVISIONS, shift)
    } else {
        exact_mesh_pair(match fixture.as_str() {
            "clipped_voxel_torus_33" => clipped_voxel_torus_case(33),
            "clipped_voxel_torus_65" => clipped_voxel_torus_case(65),
            "dense_coplanar_boxes_4" => dense_coplanar_box_case(4),
            "dense_coplanar_boxes_16" => dense_coplanar_box_case(16),
            "dense_coplanar_boxes_32" => dense_coplanar_box_case(32),
            "sparse_multishell_tetrahedra_64" => sparse_multishell_tetrahedra_case(64),
            "sparse_multishell_tetrahedra_512" => sparse_multishell_tetrahedra_case(512),
            "transverse_self_pwn_clusters_8" => transverse_self_pwn_cluster_case(8),
            "transverse_self_pwn_clusters_64" => transverse_self_pwn_cluster_case(64),
            "transverse_self_pwn_clusters_512" => transverse_self_pwn_cluster_case(512),
            "subdivided_overlapping_boxes_3072_each" => large_boolean_case(),
            _ => corpus()
                .into_iter()
                .find(|case| case.name == fixture)
                .unwrap_or_else(|| panic!("unknown competitive fixture {fixture}")),
        })
    };
    let name = case.name;
    let inputs = [case.left, case.right];
    let context = MeshContext::new(policy);
    polygon_soup(&context, &[inputs[0].as_ref(), inputs[1].as_ref()])
        .expect("overlapping-box inputs satisfy the PWN contract");

    let start = Instant::now();
    let mut last = None;
    for _ in 0..repetitions {
        last = Some(run_operation(&context, black_box(&inputs), workload));
    }
    let elapsed = start.elapsed();
    let output = last.expect("positive repetitions produce an output");
    let triangles = output
        .value
        .results
        .iter()
        .map(|result| result.triangles.len())
        .collect::<Vec<_>>();
    println!(
        "fixture={} operation={operation_name} policy={policy_name} certainty={:?} repetitions={repetitions} elapsed_ns={} ns_per_iteration={} vertices={} triangles={triangles:?}",
        name,
        output.certainty,
        elapsed.as_nanos(),
        elapsed.as_nanos() / repetitions as u128,
        output.value.vertices.len(),
    );
}
