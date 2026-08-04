#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::{env, hint::black_box, time::Instant};

use competitive_support::{corpus, run_hypermesh_all, to_hypermesh};
use hypermesh::{MeshContext, PredicatePolicy, polygon_soup};

fn main() {
    let mut args = env::args().skip(1);
    let policy_name = args
        .next()
        .expect("expected <strict|approximate-512> <repetitions>");
    let repetitions = args
        .next()
        .expect("expected a positive repetition count")
        .parse::<usize>()
        .expect("repetitions must be an integer");
    assert!(
        args.next().is_none() && repetitions != 0,
        "expected <strict|approximate-512> <positive repetitions>"
    );
    let policy = match policy_name.as_str() {
        "strict" => PredicatePolicy::STRICT,
        "approximate-512" => PredicatePolicy::APPROXIMATE_512,
        _ => panic!("policy must be strict or approximate-512"),
    };

    let case = corpus()
        .into_iter()
        .next()
        .expect("competitive corpus contains overlapping boxes");
    let inputs = [to_hypermesh(&case.left), to_hypermesh(&case.right)];
    let context = MeshContext::new(policy);
    polygon_soup(&context, &[inputs[0].as_ref(), inputs[1].as_ref()])
        .expect("overlapping-box inputs satisfy the PWN contract");

    let start = Instant::now();
    let mut last = None;
    for _ in 0..repetitions {
        last = Some(run_hypermesh_all(&context, black_box(&inputs)));
    }
    let elapsed = start.elapsed();
    let output = last.expect("positive repetitions produce an output");
    let triangles = output
        .results
        .iter()
        .map(|result| result.triangles.len())
        .collect::<Vec<_>>();
    println!(
        "policy={policy_name} repetitions={repetitions} elapsed_ns={} ns_per_iteration={} vertices={} triangles={triangles:?}",
        elapsed.as_nanos(),
        elapsed.as_nanos() / repetitions as u128,
        output.vertices.len(),
    );
}
