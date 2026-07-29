use csgrs::{
    Real, curve,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body = solid::cube(Real::from(12));
    let opening = curve::extrude(&curve::square(Real::from(4)), Real::from(16)).translated(
        Real::from(4),
        Real::from(4),
        Real::from(-2),
    );

    let part = body.try_difference(&opening)?;
    std::fs::write("drilled_cube.stl", to_stl_binary(&part, "drilled_cube")?)?;
    Ok(())
}
