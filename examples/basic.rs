use csgrs::{Csg, GeometryCertainty, io::stl::to_stl_binary};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // One session chooses the predicate policy and keeps the weakest
    // certainty of every operation in the chain.
    let m = Csg::strict();
    let body = m.cube(12)?;
    let opening = m.translate(&m.extrude(&m.square(4)?, 16)?, 4, 4, -2)?;
    let part = m.difference(&body, &opening)?;
    assert_eq!(m.certainty(), GeometryCertainty::Certified);

    std::fs::write("drilled_cube.stl", to_stl_binary(&part, "drilled_cube")?)?;
    Ok(())
}
