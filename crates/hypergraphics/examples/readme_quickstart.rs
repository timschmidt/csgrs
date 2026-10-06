use hypergraphics::{
    Color3, Point2, PredicatePolicy, Real, TriangulationContext, polygon_surface_mesh,
};

fn main() -> hypergraphics::Result<()> {
    let square = [
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];
    let orange = Color3::new(0.8, 0.4, 0.1)?;
    let context = TriangulationContext::new(PredicatePolicy::STRICT);
    let mesh = polygon_surface_mesh(&context, &square, &[], Real::zero(), orange)?.into_value();
    let render_vertices = mesh.to_render_vertices64()?;

    println!(
        "{} exact triangles; {} render vertices",
        mesh.triangle_count(),
        render_vertices.len()
    );
    Ok(())
}
