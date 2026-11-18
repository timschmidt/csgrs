//! Use certified dyadic grids to locate the positive square root of two.

use hyperinterval::{DyadicBudget, DyadicInterval, IntervalError};
use hyperreal::{Rational, Real};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = Real::from(2).sqrt()?;
    let threshold = Rational::fraction(181, 128)?;
    let mut exhausted = DyadicBudget::new(256, 0);
    assert_eq!(
        DyadicInterval::from_real(&root, -4, &mut exhausted),
        Err(IntervalError::WorkExhausted)
    );

    // One conversion budget pays for both grid requests and the export of
    // their rational endpoints. Hyperreal owns evaluation of the source.
    let mut budget = DyadicBudget::new(256, 1_000_000);
    for exponent in [-4, -24] {
        let grid = DyadicInterval::from_real(&root, exponent, &mut budget)?;
        let exact = grid.to_rational(&mut budget)?;
        let (lo, hi) = exact.bounds().expect("nonempty root enclosure");
        assert!(lo.is_positive() && lo * lo < Rational::from(2) && hi * hi > Rational::from(2));
        if exact.contains_point(&threshold) {
            println!("Grid 2^{exponent} contains both the root and 181/128.");
            assert_eq!(exponent, -4);
        } else {
            assert!(lo > &threshold);
            println!("Grid 2^{exponent} certifies sqrt(2) > 181/128.");
            assert_eq!(exponent, -24);
        }
    }
    Ok(())
}
