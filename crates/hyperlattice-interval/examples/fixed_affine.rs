use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};

fn main() -> Result<(), IntervalLatticeError> {
    type I = FixedInterval<i32, 0>;
    let a = IntervalMatrix::new([[I::point(2)?, I::point(-1)?]])?;
    let x = IntervalVector::new([I::new(1, 3)?, I::new(4, 5)?])?;
    let b = IntervalVector::new([I::point(7)?])?;
    let result = a.affine_enclosure(&x, &b, &mut ())?;
    assert_eq!(result.lanes().unwrap()[0].bounds(), Some((4, 9)));
    Ok(())
}
