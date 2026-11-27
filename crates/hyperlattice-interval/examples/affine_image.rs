//! Keep a shared source parameter through two exact affine maps.
//!
//! On the line (t,2t+1), t in `[1,3]`, the relation 2x-y is identically -1.
//! Its Cartesian hull admits independent coordinates and gives `[-5,3]` instead.
//! Both projections are checked on an integer fixed grid with exact endpoints;
//! an enclosure does not replace the retained affine definition.

use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineMap, IntervalLatticeError, IntervalMatrix, IntervalVector};

/// Construct the line image, check its hull, compose the relation, and compare
/// with applying that relation to the hull. Native construction/arithmetic errors
/// propagate; a wrong literal expectation panics. Print the comparison on success.
fn main() -> Result<(), IntervalLatticeError> {
    /// Whole-unit i32 intervals; every coefficient and expected endpoint here fits exactly.
    type I = FixedInterval<i32, 0>;
    let parameter = IntervalVector::new([I::new(1, 3)?])?;
    let line = AffineMap::new(
        IntervalMatrix::new([[I::point(1)?], [I::point(2)?]])?,
        IntervalVector::new([I::point(0)?, I::point(1)?])?,
    )?;
    let image = line.image(parameter);
    let hull = image.enclosure(&mut ())?;
    assert_eq!(hull.lanes(), Some(&[I::new(1, 3)?, I::new(3, 7)?]));

    // Every point on (t, 2t + 1) has 2x - y = -1. Compose on the retained
    // image so both coordinates still refer to the same parameter.
    let linear = IntervalMatrix::new([[I::point(2)?, I::point(-1)?]])?;
    let relation = AffineMap::new(linear, IntervalVector::new([I::point(0)?])?)?;
    let result = relation.image(image).enclosure(&mut ())?;
    assert_eq!(result.lanes(), Some(&[I::point(-1)?]));

    // The Cartesian hull also admits (1, 7) and (3, 3).
    let projected = linear.mul_vector(&hull, &mut ())?;
    assert_eq!(projected.lanes(), Some(&[I::new(-5, 3)?]));
    println!("2x - y is -1 on the affine image and ranges from -5 to 3 on its hull.");
    Ok(())
}
