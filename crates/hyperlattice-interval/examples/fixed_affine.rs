//! Evaluate an interval affine expression with independent Cartesian input lanes.
//!
//! For x in `[1,3]` and y in `[4,5]`, 2*x-y+7 ranges exactly over `[4,9]`.
//! These integer-grid fixtures need no outward rounding; general fixed-point
//! products may round outward or fail on overflow.

use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};

/// Build singleton coefficients and translation, enclose the expression, and
/// assert the independently derived endpoints. Native errors propagate; unexpected
/// emptiness or bounds panic. A successful example produces no stdout output.
fn main() -> Result<(), IntervalLatticeError> {
    /// Whole-unit signed i32 endpoints for the exactly representable example.
    type I = FixedInterval<i32, 0>;
    let a = IntervalMatrix::new([[I::point(2)?, I::point(-1)?]])?;
    let x = IntervalVector::new([I::new(1, 3)?, I::new(4, 5)?])?;
    let b = IntervalVector::new([I::point(7)?])?;
    let result = a.affine_enclosure(&x, &b, &mut ())?;
    assert_eq!(result.lanes().unwrap()[0].bounds(), Some((4, 9)));
    Ok(())
}
