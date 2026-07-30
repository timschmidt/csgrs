use std::hint::black_box;

use hypermesh::{
    BooleanOp, EmberConfig, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean_mesh,
};

fn cube(min: i32, max: i32) -> TriangleMesh {
    let point = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
    TriangleMesh::new(
        vec![
            point(min, min, min),
            point(max, min, min),
            point(max, max, min),
            point(min, max, min),
            point(min, min, max),
            point(max, min, max),
            point(max, max, max),
            point(min, max, max),
        ],
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
        ],
    )
}

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
