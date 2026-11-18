//! Keep a shared source parameter through two exact affine maps.

use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineMap, IntervalLatticeError, IntervalMatrix, IntervalVector};

fn main() -> Result<(), IntervalLatticeError> {
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
