use csgrs::{Real, csg::CSG, mesh::Mesh, profile::Profile};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body = Mesh::cube(Real::from(12), ());
    let opening = Profile::square(Real::from(4))
        .extrude(Real::from(16), ())
        .translate(Real::from(4), Real::from(4), Real::from(-2));

    let part = body.try_difference(&opening)?;
    assert!(part.is_manifold());
    std::fs::write("drilled_cube.stl", part.to_stl_binary("drilled_cube")?)?;
    Ok(())
}
