#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use std::hint::black_box;

use hypermesh::{BooleanOp, EmberConfig, MeshContext, PredicatePolicy, boolean_operation};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn main() {
    let repetitions = std::env::args()
        .nth(1)
        .map_or(256, |value| value.parse().expect("valid repetition count"));
    let meshes = common::octahedron_pair();
    let mut classifications = 0;
    for _ in 0..repetitions {
        let result = boolean_operation(
            &CONTEXT,
            black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
            BooleanOp::Union,
            EmberConfig::default(),
        )
        .expect("octahedron union must remain certified")
        .into_value();
        classifications += black_box(result.classifications().len());
    }
    println!("{classifications}");
}
