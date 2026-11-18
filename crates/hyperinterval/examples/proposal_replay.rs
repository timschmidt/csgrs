//! Check a floating-point proposal by replaying the authored measurement.

use hyperinterval::{F64ProposalInterval, RationalInterval};
use hyperreal::Rational;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The instrument measures 1/2 with uncertainty 1/4. All three values
    // (nominal, uncertainty and each chosen translation) have exact f64 encodings.
    let nominal = 0.5;
    let uncertainty = 0.25;
    let measurement = RationalInterval::from_nominal_uncertainty(
        &Rational::try_from(nominal)?,
        &Rational::try_from(uncertainty)?,
    )?;
    let estimate = F64ProposalInterval::new(nominal - uncertainty, nominal + uncertainty)?;

    for offset in [1.0, 9_007_199_254_740_992.0] {
        // Translate to another origin and back, using the same source values.
        let shift = F64ProposalInterval::point(offset)?;
        let proposed = estimate.add(&shift)?.sub(&shift)?;
        let (lo, hi) = proposed.estimates().expect("nonempty measurement");

        let shift = RationalInterval::point(Rational::try_from(offset)?);
        let exact = measurement.add(&shift).sub(&shift);
        assert_eq!(exact, measurement);
        let (lower, upper) = exact.bounds().expect("nonempty rational replay");

        // Read the proposed binary endpoints exactly. Only the comparison
        // against the replay establishes whether they contain the result.
        let encloses = Rational::try_from(lo)? <= *lower && *upper <= Rational::try_from(hi)?;
        println!(
            "offset {offset:.0}: proposed [{lo}, {hi}], exact [{lower}, {upper}], encloses = {encloses}"
        );
        assert_eq!(encloses, offset == 1.0);
    }
    Ok(())
}
