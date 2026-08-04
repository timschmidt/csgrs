#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;
#[path = "support/large_mesh_probe.rs"]
mod large_mesh_probe;
#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod mesh_common;

use std::hint::black_box;

use hypermesh::{BooleanProgram, MeshContext, PredicatePolicy, boolean};
use large_mesh_probe::{FIXTURE_HELP, input_views, prepare_large_fixture, prime_input};

fn main() {
    let mut args = std::env::args().skip(1);
    let fixture = args.next().expect(FIXTURE_HELP);
    let (policy_name, policy) = match args.next().as_deref() {
        Some("strict") => ("STRICT", PredicatePolicy::STRICT),
        Some("approximate-512") => ("APPROXIMATE_512", PredicatePolicy::APPROXIMATE_512),
        _ => panic!("expected strict or approximate-512"),
    };
    assert!(
        args.next().is_none(),
        "expected exactly one fixture and one policy"
    );
    let prepared = prepare_large_fixture(&fixture);
    let name = prepared.name;
    let meshes = prepared.meshes;
    let input_triangles = meshes[0].triangles.len() + meshes[1].triangles.len();
    let context = MeshContext::new(policy);
    prime_input(&context, &meshes, prepared.input_path);
    let views = input_views(&meshes, prepared.input_path);

    let result = boolean(
        &context,
        black_box(&views),
        BooleanProgram::Operation(prepared.operation),
    )
    .expect("large fixture Boolean must complete under the selected policy");
    println!(
        "{name}: policy={policy_name}, certainty={:?}, input_triangles={input_triangles}, \
         output_vertices={}, output_triangles={}",
        result.certainty,
        result.value.vertices.len(),
        result.value.results[0].triangles.len()
    );
}
