#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use std::hint::black_box;

use hypermesh::{BooleanOp, BooleanProgram, MeshContext, PredicatePolicy, boolean};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn main() {
    let repetitions = std::env::args()
        .nth(1)
        .map_or(256, |value| value.parse().expect("valid repetition count"));
    let meshes = common::octahedron_pair();
    let mut triangles = 0;
    for _ in 0..repetitions {
        let result = boolean(
            &CONTEXT,
            black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
            BooleanProgram::Operation(BooleanOp::Union),
        )
        .expect("octahedron union must remain certified")
        .into_value();
        triangles += black_box(result.results[0].triangles.len());
    }
    println!("{triangles}");
}
