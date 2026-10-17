//! Optional interval proof for the existing relative area tolerance.

use crate::Scalar;
use hyperreal::Rational;

/// Decide `|a-b| <= tolerance * max(|a|, |b|, 1)` from certified enclosures.
/// Overlapping bounds leave the original scalar predicate in charge. The
/// rational intervals are proof bounds, not rounded replacements for areas.
pub(super) fn certified_area_tolerance(
    left: &Scalar,
    right: &Scalar,
    tolerance: &Scalar,
) -> Option<bool> {
    let tolerance = tolerance.exact_rational_ref()?;
    if tolerance < &Rational::zero() {
        return None;
    }
    let left = left.certified_rational_interval(-32)?;
    let right = right.certified_rational_interval(-32)?;
    let difference = absolute_interval([&left[0] - &right[1], &left[1] - &right[0]]);
    let left = absolute_interval(left);
    let right = absolute_interval(right);
    let scale_lower = rational_max(
        rational_max(left[0].clone(), right[0].clone()),
        Rational::one(),
    );
    let scale_upper = rational_max(
        rational_max(left[1].clone(), right[1].clone()),
        Rational::one(),
    );
    if difference[1] <= tolerance * &scale_lower {
        Some(true)
    } else if difference[0] > tolerance * &scale_upper {
        Some(false)
    } else {
        None
    }
}

fn absolute_interval([lower, upper]: [Rational; 2]) -> [Rational; 2] {
    if lower >= Rational::zero() {
        [lower, upper]
    } else if upper <= Rational::zero() {
        [-upper, -lower]
    } else {
        [Rational::zero(), rational_max(upper, -lower)]
    }
}

fn rational_max(left: Rational, right: Rational) -> Rational {
    if left >= right { left } else { right }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rational_boundary_inclusion_and_signed_scale_are_exact() {
        let tolerance = crate::scalar::scalar("0.25");
        for (left, right, expected) in [
            ("0", "0.25", true),
            ("0", "0.25000000000000000000001", false),
            ("0", "-0.25", true),
            ("3", "4", true),
            ("3", "4.00000000000000000000001", false),
            ("-3", "-4", true),
            ("-3", "-4.00000000000000000000001", false),
            ("-3", "4", false),
        ] {
            let left = crate::scalar::scalar(left);
            let right = crate::scalar::scalar(right);
            for (left, right) in [(&left, &right), (&right, &left)] {
                assert_eq!(
                    certified_area_tolerance(left, right, &tolerance),
                    Some(expected)
                );
            }
        }
    }

    #[test]
    fn symbolic_enclosures_decide_only_separated_thresholds_cold_and_warm() {
        for scale in [
            Scalar::one(),
            -Scalar::one(),
            crate::scalar::scalar("1e400"),
        ] {
            let left = Scalar::pi() * &scale;
            let close = &left + &scale * crate::scalar::scalar("0.1");
            let far = &left + &scale;
            for _ in 0..2 {
                assert_eq!(
                    certified_area_tolerance(&left, &close, &crate::scalar::scalar("0.1")),
                    Some(true)
                );
                assert_eq!(
                    certified_area_tolerance(&left, &far, &crate::scalar::scalar("0.1")),
                    Some(false)
                );
                assert_eq!(
                    certified_area_tolerance(&left, &left, &Scalar::zero()),
                    None
                );
            }
        }
    }

    #[test]
    fn unavailable_tolerance_retains_scalar_fallback() {
        assert_eq!(
            certified_area_tolerance(&Scalar::one(), &Scalar::one(), &-Scalar::one()),
            None
        );
        assert_eq!(
            certified_area_tolerance(&Scalar::one(), &Scalar::one(), &Scalar::pi()),
            None
        );
    }
}
