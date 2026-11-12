use hyperinterval::{FixedInterval, IntervalError, MidpointRounding, SignSet};

fn main() -> Result<(), IntervalError> {
    type Q30Bounds = FixedInterval<i32, 30>;
    // Exact closed interval [1/2, 1] in Q2.30 numerator units.
    let measurement = Q30Bounds::new(1 << 29, 1 << 30)?;
    let permitted = Q30Bounds::new(0, 1 << 30)?;
    assert!(measurement.subset_of(&permitted));
    assert_eq!(measurement.signs(), SignSet::Positive);
    assert_eq!(measurement.square()?.bounds(), Some((1 << 28, 1 << 30)));
    assert_eq!(measurement.midpoint(MidpointRounding::Floor), Some(3 << 28));
    assert_eq!(
        measurement.intersection(&Q30Bounds::empty()?).bounds(),
        None
    );
    Ok(())
}
