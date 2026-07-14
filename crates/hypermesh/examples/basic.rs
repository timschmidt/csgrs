use hypermesh::{
    BooleanOp, EmberConfig, InputMesh, Point3, Real, Triangle, boolean_operation,
    triangulate_and_resolve_certified,
};

fn tetrahedron(offset: i64) -> InputMesh {
    let p = |x, y, z| Point3::new(Real::from(x + offset), Real::from(y), Real::from(z));
    InputMesh::new(
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
    let result = boolean_operation(
        &[first.as_ref(), second.as_ref()],
        BooleanOp::Union,
        EmberConfig::default(),
    )?;
    let triangles = triangulate_and_resolve_certified(&result)?;
    println!("{} exact output triangles", triangles.triangles.len());
    Ok(())
}
