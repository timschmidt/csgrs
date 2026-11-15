//! A conservative box with its exact source retained.
use hyperlattice::interval::hyperinterval::DyadicBudget;
use hyperreal::Real;
use hypervoxel::{CellBounds, ExactAffineTransform};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let transform = ExactAffineTransform::new(
        [
            [Real::from(2).sqrt()?, 0.into(), 0.into()],
            [0.into(), 1.into(), 0.into()],
            [0.into(), 0.into(), 1.into()],
        ],
        [3.into(), 0.into(), 0.into()],
    );
    let source = CellBounds {
        min: [0.into(), 0.into(), 0.into()],
        max: [1.into(), 1.into(), 1.into()],
    };
    let report =
        transform.enclose_bounds(&source, -32, &mut DyadicBudget::new(4096, 10_000_000))?;
    assert_eq!(report.source(), &source);
    println!("conservative bounds: {:?}", report.bounds());
    Ok(())
}
