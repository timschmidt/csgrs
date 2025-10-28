pub mod intersection;
pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;

pub use crate::intersection::*;
pub use crate::manifold::*;
pub use anyhow::*;
use nalgebra::RowVector3 as Row3;

pub fn create_manifold(
    pos: &[Row3<f64>],
    idx: &[Row3<usize>],
) -> Result<Manifold> {
    panic!()
}

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold
) -> Result<Manifold> {
    panic!()
}

fn is_manifold(
) -> bool {
    false
}
