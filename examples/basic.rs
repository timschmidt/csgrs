//! Strict-session subtraction of an extruded square opening from a cube.
//! Requires `curve` and `stl-io`. The certainty assertion covers the modeling
//! chain before terminal binary STL projection; it does not certify STL fidelity.
//! Writes or replaces `drilled_cube.stl` in the current working directory.

use csgrs::{Csg, GeometryCertainty, io::stl::to_stl_binary};

/// Builds a 12-unit cube with a through opening spanning X/Y 4..8, checks the
/// session receipt and writes binary STL. Modeling/export/IO errors propagate;
/// an unexpected certainty label panics at the assertion.
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
