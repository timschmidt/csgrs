use std::hint::black_box;

#[path = "../cube.rs"]
mod cube_fixture;

use hypermesh::{BooleanOp, EmberConfig, MeshContext, PredicatePolicy, boolean_mesh};

use cube_fixture::cube;

fn main() -> hypermesh::HypermeshResult<()> {
    let operation = match std::env::args().nth(1).as_deref() {
        Some("intersection") => BooleanOp::Intersection,
        Some("difference") => BooleanOp::Difference,
        Some("symmetric-difference") => BooleanOp::SymmetricDifference,
        _ => BooleanOp::Union,
    };
    let policy = match std::env::args().nth(2).as_deref() {
        Some("strict") => PredicatePolicy::STRICT,
        _ => PredicatePolicy::APPROXIMATE_512,
    };
    let context = MeshContext::new(policy);
    let left = cube(0, 2);
    let right = cube(1, 3);
    let result = boolean_mesh(
        black_box(&context),
        black_box(&[left.as_ref(), right.as_ref()]),
        black_box(operation),
        EmberConfig::default(),
    )?;
    println!(
        "{:?}: {} vertices, {} triangles",
        result.certainty,
        result.value.vertices.len(),
        result.value.triangles.len()
    );
    Ok(())
}
