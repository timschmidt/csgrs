use std::hint::black_box;

use hypermesh::{
    BooleanOp, EmberConfig, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean_operation, triangulate_and_resolve_certified,
};

fn tetrahedron(x_offset: i64) -> TriangleMesh {
    let point = |x, y, z| Point3::new(Real::from(x + x_offset), Real::from(y), Real::from(z));
    TriangleMesh::new(
        vec![
            point(0, 0, 0),
            point(3, 0, 0),
            point(0, 3, 0),
            point(0, 0, 3),
        ],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(2, 0, 3),
        ],
    )
}

fn selected_operation() -> BooleanOp {
    match std::env::args().nth(1).as_deref() {
        Some("intersection") => BooleanOp::Intersection,
        Some("difference") => BooleanOp::Difference,
        Some("symmetric-difference") => BooleanOp::SymmetricDifference,
        _ => BooleanOp::Union,
    }
}

fn main() -> hypermesh::HypermeshResult<()> {
    let context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
    let first = tetrahedron(0);
    let second = tetrahedron(1);
    let result = boolean_operation(
        black_box(&context),
        black_box(&[first.as_ref(), second.as_ref()]),
        black_box(selected_operation()),
        EmberConfig::default(),
    )?;
    let triangles =
        triangulate_and_resolve_certified(black_box(&context), black_box(&result.value))?;
    println!(
        "{} polygons, {} vertices, {} triangles",
        result.value.output().polygons.len(),
        triangles.value.vertices.len(),
        triangles.value.triangles.len()
    );
    Ok(())
}
