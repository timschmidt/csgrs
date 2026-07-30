use std::hint::black_box;

#[path = "../cube.rs"]
mod cube_fixture;

use cube_fixture::cube;
use hypermesh::{
    BooleanOp, EmberConfig, MeshContext, PredicatePolicy, boolean_operation,
};

fn main() -> hypermesh::HypermeshResult<()> {
    let iterations = std::env::args()
        .nth(1)
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(512);
    let context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let left = cube(0, 2);
    let right = cube(1, 3);
    let meshes = [left.as_ref(), right.as_ref()];
    let mut polygons = 0;
    for _ in 0..iterations {
        polygons = black_box(boolean_operation(
            black_box(&context),
            black_box(&meshes),
            BooleanOp::Union,
            EmberConfig::default(),
        )?)
        .value
        .classifications()
        .len();
    }
    println!("{iterations} operations, {polygons} final polygons");
    Ok(())
}
