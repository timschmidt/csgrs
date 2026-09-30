//! Modular coprimality certificates for exact rational univariate polynomials.
//!
//! A subresultant chain decides whether two polynomials share a factor, but
//! its exact rational coefficients can grow far beyond the size of the inputs.
//! Reduction modulo a prime `p` is a cheap one-sided certificate: when `p`
//! divides no denominator and neither leading numerator, every common factor
//! over the rationals survives reduction with its degree intact (Gauss's
//! lemma), so a constant gcd modulo `p` proves the inputs coprime. A
//! nonconstant modular gcd is only evidence, since an unlucky prime can divide
//! the resultant.

use hyperreal::{Rational, Real};
use num::{BigUint, ToPrimitive};

/// Large primes below `2^61`, chosen independently of any input.
const PRIMES: [u64; 4] = [
    2_305_843_009_213_693_951,
    2_305_843_009_213_693_921,
    2_305_843_009_213_693_907,
    2_305_843_009_213_693_669,
];

/// Outcome of a modular coprimality test.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModularCoprimality {
    /// A good prime certified that the polynomials share no nonconstant factor.
    Coprime,
    /// Every good prime found a nonconstant common factor. The polynomials
    /// very likely share a factor, but this is not a certificate.
    CommonFactorLikely,
    /// The inputs are not all exact rationals, a polynomial is zero, or no
    /// tested prime was good; no conclusion.
    Inconclusive,
}

/// Tests two polynomials, given by coefficients in ascending powers, for a
/// nonconstant common factor modulo several fixed primes.
///
/// Only [`ModularCoprimality::Coprime`] is a certificate.
pub fn univariate_polynomials_modular_coprimality(
    first: &[Real],
    second: &[Real],
) -> ModularCoprimality {
    let (Some(first), Some(second)) = (exact_trimmed(first), exact_trimmed(second)) else {
        return ModularCoprimality::Inconclusive;
    };
    if first.is_empty() || second.is_empty() {
        return ModularCoprimality::Inconclusive;
    }
    if first.len() == 1 || second.len() == 1 {
        // A nonzero constant shares no nonconstant factor.
        return ModularCoprimality::Coprime;
    }
    let mut good_primes = 0;
    for prime in PRIMES {
        let (Some(first), Some(second)) = (reduce(&first, prime), reduce(&second, prime)) else {
            continue;
        };
        good_primes += 1;
        if gcd_degree(first, second, prime) == 0 {
            return ModularCoprimality::Coprime;
        }
    }
    if good_primes == 0 {
        ModularCoprimality::Inconclusive
    } else {
        ModularCoprimality::CommonFactorLikely
    }
}

fn exact_trimmed(coefficients: &[Real]) -> Option<Vec<&Rational>> {
    let mut exact = coefficients
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    while exact
        .last()
        .is_some_and(|coefficient| coefficient.is_zero())
    {
        exact.pop();
    }
    Some(exact)
}

fn residue(value: &BigUint, prime: u64) -> u64 {
    (value % prime)
        .to_u64()
        .expect("a residue is below its u64 modulus")
}

/// Reduces a polynomial modulo `prime`, or declines when the prime divides a
/// denominator or the leading numerator (a bad prime for this input).
fn reduce(coefficients: &[&Rational], prime: u64) -> Option<Vec<u64>> {
    let mut reduced = Vec::with_capacity(coefficients.len());
    for coefficient in coefficients {
        let denominator = residue(coefficient.denominator(), prime);
        if denominator == 0 {
            return None;
        }
        let mut value = mul(
            residue(coefficient.numerator(), prime),
            inverse(denominator, prime),
            prime,
        );
        if coefficient.is_negative() && value != 0 {
            value = prime - value;
        }
        reduced.push(value);
    }
    (*reduced.last()? != 0).then_some(reduced)
}

fn mul(left: u64, right: u64, prime: u64) -> u64 {
    ((u128::from(left) * u128::from(right)) % u128::from(prime)) as u64
}

fn inverse(value: u64, prime: u64) -> u64 {
    // Fermat: value^(p-2) for prime p and nonzero value.
    let (mut base, mut exponent, mut result) = (value, prime - 2, 1);
    while exponent > 0 {
        if exponent & 1 == 1 {
            result = mul(result, base, prime);
        }
        base = mul(base, base, prime);
        exponent >>= 1;
    }
    result
}

fn trim(polynomial: &mut Vec<u64>) {
    while polynomial.last() == Some(&0) {
        polynomial.pop();
    }
}

/// Degree of the monic gcd over the prime field, by the Euclidean algorithm.
fn gcd_degree(mut left: Vec<u64>, mut right: Vec<u64>, prime: u64) -> usize {
    trim(&mut left);
    trim(&mut right);
    while !right.is_empty() {
        // left <- left mod right
        let lead_inverse = inverse(*right.last().expect("nonzero divisor"), prime);
        while left.len() >= right.len() {
            let factor = mul(
                *left.last().expect("nonempty dividend"),
                lead_inverse,
                prime,
            );
            let shift = left.len() - right.len();
            for (index, coefficient) in right.iter().enumerate() {
                let term = mul(factor, *coefficient, prime);
                let slot = &mut left[shift + index];
                *slot = (*slot + prime - term) % prime;
            }
            trim(&mut left);
            if left.is_empty() {
                break;
            }
        }
        std::mem::swap(&mut left, &mut right);
    }
    left.len().saturating_sub(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn polynomial(coefficients: &[i64]) -> Vec<Real> {
        coefficients
            .iter()
            .map(|&value| Real::from(value))
            .collect()
    }

    #[test]
    fn distinct_linear_factors_are_certified_coprime() {
        // (x - 1)(x - 2) and (x - 3)(x + 5)
        assert_eq!(
            univariate_polynomials_modular_coprimality(
                &polynomial(&[2, -3, 1]),
                &polynomial(&[-15, 2, 1]),
            ),
            ModularCoprimality::Coprime
        );
    }

    #[test]
    fn a_shared_factor_is_never_certified_coprime() {
        // (x - 1)(x - 2) and (x - 1)(x + 7), with rational scaling.
        let second = polynomial(&[-7, 6, 1])
            .into_iter()
            .map(|value| (value / Real::from(3)).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(
            univariate_polynomials_modular_coprimality(&polynomial(&[2, -3, 1]), &second),
            ModularCoprimality::CommonFactorLikely
        );
    }

    #[test]
    fn irrational_or_zero_inputs_are_inconclusive() {
        let irrational = vec![Real::from(2).sqrt().unwrap(), Real::one()];
        assert_eq!(
            univariate_polynomials_modular_coprimality(&irrational, &polynomial(&[1, 1])),
            ModularCoprimality::Inconclusive
        );
        assert_eq!(
            univariate_polynomials_modular_coprimality(&polynomial(&[0, 0]), &polynomial(&[1, 1])),
            ModularCoprimality::Inconclusive
        );
    }

    #[test]
    fn nonzero_constants_are_coprime_to_everything() {
        assert_eq!(
            univariate_polynomials_modular_coprimality(&polynomial(&[4]), &polynomial(&[0, 1])),
            ModularCoprimality::Coprime
        );
    }
}
