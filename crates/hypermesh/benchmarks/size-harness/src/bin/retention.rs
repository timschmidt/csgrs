use std::hint::black_box;

#[path = "../cube.rs"]
mod cube_fixture;

use cube_fixture::cube;
use hypermesh::{
    BooleanOp, BooleanProgram, MeshContext, PredicatePolicy, TriangleMeshRef, boolean,
};

fn main() -> hypermesh::HypermeshResult<()> {
    let iterations = std::env::args()
        .nth(1)
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(512);
    let context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let left = cube(0, 2);
    let right = cube(1, 3);
    let meshes = [
        TriangleMeshRef::new(&left.positions, &left.triangles),
        TriangleMeshRef::new(&right.positions, &right.triangles),
    ];
    let mut polygons = 0;
    for _ in 0..iterations {
        polygons = black_box(boolean(
            black_box(&context),
            black_box(&meshes),
            BooleanProgram::Operation(BooleanOp::Union),
        )?)
        .value
        .results[0]
        .triangles
        .len();
    }
    println!("{iterations} operations, {polygons} final triangles");
    Ok(())
}
