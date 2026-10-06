use hypermesh::{
    BooleanOp, BooleanProgram, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean,
};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn tetrahedron(offset: i64) -> TriangleMesh {
    let p = |x, y, z| Point3::new(Real::from(x + offset), Real::from(y), Real::from(z));
    TriangleMesh::new(
        vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(2, 0, 3),
        ],
    )
}

fn main() -> hypermesh::HypermeshResult<()> {
    let first = tetrahedron(0);
    let second = tetrahedron(3);
    let result = boolean(
        &CONTEXT,
        &[first.as_ref(), second.as_ref()],
        BooleanProgram::Operation(BooleanOp::Union),
    )?
    .into_value();
    println!(
        "{} exact output triangles",
        result.results[0].triangles.len()
    );
    Ok(())
}
