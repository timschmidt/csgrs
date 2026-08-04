#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::{env, hint::black_box, time::Instant};

use competitive_support::{clipped_voxel_torus_case, corpus, run_hypermesh_all, to_hypermesh};
use hypermesh::{
    BooleanMeshBatch, BooleanOp, BooleanProgram, MeshContext, PredicatePolicy, TriangleMesh,
    boolean, polygon_soup,
};

fn run_operation(
    context: &MeshContext,
    inputs: &[TriangleMesh; 2],
    operation: Option<(BooleanOp, bool)>,
) -> BooleanMeshBatch {
    let Some((operation, reverse)) = operation else {
        return run_hypermesh_all(context, inputs);
    };
    let views = if reverse {
        [inputs[1].as_ref(), inputs[0].as_ref()]
    } else {
        [inputs[0].as_ref(), inputs[1].as_ref()]
    };
    boolean(context, &views, BooleanProgram::Operation(operation))
        .expect("competitive Boolean operation failed")
        .into_value()
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
    let operation = match operation_name.as_str() {
        "all" => None,
        "union" => Some((BooleanOp::Union, false)),
        "intersection" => Some((BooleanOp::Intersection, false)),
        "difference" => Some((BooleanOp::Difference, false)),
        "reverse-difference" => Some((BooleanOp::Difference, true)),
        "xor" => Some((BooleanOp::SymmetricDifference, false)),
        _ => panic!(
            "operation must be union, intersection, difference, reverse-difference, xor, or all"
        ),
    };

    let case = match fixture.as_str() {
        "clipped_voxel_torus_33" => clipped_voxel_torus_case(33),
        "clipped_voxel_torus_65" => clipped_voxel_torus_case(65),
        _ => corpus()
            .into_iter()
            .find(|case| case.name == fixture)
            .unwrap_or_else(|| panic!("unknown competitive fixture {fixture}")),
    };
    let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
    let context = MeshContext::new(policy);
    polygon_soup(&context, &[inputs[0].as_ref(), inputs[1].as_ref()])
        .expect("overlapping-box inputs satisfy the PWN contract");

    let start = Instant::now();
    let mut last = None;
    for _ in 0..repetitions {
        last = Some(run_operation(&context, black_box(&inputs), operation));
    }
    let elapsed = start.elapsed();
    let output = last.expect("positive repetitions produce an output");
    let triangles = output
        .results
        .iter()
        .map(|result| result.triangles.len())
        .collect::<Vec<_>>();
    println!(
        "fixture={} operation={operation_name} policy={policy_name} repetitions={repetitions} elapsed_ns={} ns_per_iteration={} vertices={} triangles={triangles:?}",
        case.name,
        elapsed.as_nanos(),
        elapsed.as_nanos() / repetitions as u128,
        output.vertices.len(),
    );
}
