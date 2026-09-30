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

/// Certifies that two polynomials in `y` over `Q(alpha)` share no nonconstant
/// factor, where `alpha` is a root of `defining` (ascending exact rationals),
/// which need not be irreducible.
///
/// Coefficients are polynomials in `alpha` (ascending exact rationals). Let
/// `R(x) = Res_y(A, B)` and `m1` the minimal polynomial of `alpha`, a primitive
/// factor of `defining`. A common factor forces `R(alpha) = 0`, so `m1 | R`.
/// For a prime `p` dividing no denominator nor the leading coefficient of
/// `defining`, and with neither leading coefficient in `y` vanishing modulo
/// `p`, reduction commutes with the resultant and Gauss's lemma keeps
/// `m1 mod p` a nonconstant common divisor of `R mod p` and `defining mod p`.
/// A constant `gcd(R mod p, defining mod p)` therefore proves coprimality.
/// `R mod p` is interpolated from univariate resultants at sample points
/// where both leading coefficients survive. `false` is not a certificate of
/// a common factor.
pub(crate) fn algebraic_extension_polynomials_certainly_coprime(
    first: &[Vec<Real>],
    second: &[Vec<Real>],
    defining: &[Real],
) -> bool {
    let (Some(first), Some(second), Some(defining)) = (
        exact_nested(first),
        exact_nested(second),
        exact_trimmed(defining),
    ) else {
        return false;
    };
    if defining.len() < 2 || first.len() < 2 || second.len() < 2 {
        return false;
    }
    let x_degree = |polynomial: &[Vec<&Rational>]| {
        polynomial
            .iter()
            .map(|coefficient| coefficient.len().saturating_sub(1))
            .max()
            .unwrap_or(0)
    };
    // deg_x Res_y(A, B) <= deg_y(A) deg_x(B) + deg_y(B) deg_x(A).
    let degree_bound =
        (first.len() - 1) * x_degree(&second) + (second.len() - 1) * x_degree(&first);
    for prime in SMALL_PRIMES {
        let Some(defining) = reduce_all(&defining, prime) else {
            continue;
        };
        if *defining.last().expect("nonconstant defining polynomial") == 0 {
            continue;
        }
        let (Some(first), Some(second)) =
            (reduce_nested(&first, prime), reduce_nested(&second, prime))
        else {
            continue;
        };
        if first.last().is_none_or(|lead| lead.iter().all(|&c| c == 0))
            || second
                .last()
                .is_none_or(|lead| lead.iter().all(|&c| c == 0))
        {
            continue;
        }
        let Some(resultant) = interpolated_resultant(&first, &second, degree_bound, prime) else {
            continue;
        };
        if resultant.iter().all(|&c| c == 0) {
            // Zero modulo p: either a genuine common factor or an unlucky
            // prime; neither certifies coprimality.
            continue;
        }
        if gcd_degree(resultant, defining, prime) == 0 {
            return true;
        }
    }
    false
}

/// `Res_y(A, B) mod p` as a polynomial in `x`, interpolated from exact
/// univariate resultants at points where both leading coefficients survive.
fn interpolated_resultant(
    first: &[Vec<u64>],
    second: &[Vec<u64>],
    degree_bound: usize,
    prime: u64,
) -> Option<Vec<u64>> {
    let first_lead = first.last()?;
    let second_lead = second.last()?;
    let mut points = Vec::with_capacity(degree_bound + 1);
    let mut value = 0_u64;
    while points.len() <= degree_bound {
        if value >= prime {
            return None;
        }
        if evaluate(first_lead, value, prime) != 0 && evaluate(second_lead, value, prime) != 0 {
            let image = |polynomial: &[Vec<u64>]| {
                polynomial
                    .iter()
                    .map(|coefficient| evaluate(coefficient, value, prime))
                    .collect::<Vec<_>>()
            };
            points.push((
                value,
                univariate_resultant(image(first), image(second), prime),
            ));
        }
        value += 1;
    }
    Some(interpolate(&points, prime))
}

/// Resultant of two univariate polynomials with nonzero leading
/// coefficients over the prime field, by the Euclidean recurrence
/// `Res(a, b) = (-1)^(deg a deg b) lc(b)^(deg a - deg r) Res(b, a mod b)`.
fn univariate_resultant(mut first: Vec<u64>, mut second: Vec<u64>, prime: u64) -> u64 {
    let mut result = 1_u64;
    loop {
        trim(&mut first);
        trim(&mut second);
        if first.is_empty() || second.is_empty() {
            return 0;
        }
        let first_degree = first.len() - 1;
        let second_degree = second.len() - 1;
        if second_degree == 0 {
            return mul(result, power(second[0], first_degree as u64, prime), prime);
        }
        let remainder = remainder(&first, &second, prime);
        if remainder.is_empty() {
            return 0;
        }
        let remainder_degree = remainder.len() - 1;
        if (first_degree * second_degree) % 2 == 1 {
            result = (prime - result) % prime;
        }
        result = mul(
            result,
            power(
                *second.last().expect("nonzero divisor"),
                (first_degree - remainder_degree) as u64,
                prime,
            ),
            prime,
        );
        first = second;
        second = remainder;
    }
}

fn remainder(dividend: &[u64], divisor: &[u64], prime: u64) -> Vec<u64> {
    let mut left = dividend.to_vec();
    let lead_inverse = inverse(*divisor.last().expect("nonzero divisor"), prime);
    while left.len() >= divisor.len() {
        let factor = mul(
            *left.last().expect("nonempty dividend"),
            lead_inverse,
            prime,
        );
        let shift = left.len() - divisor.len();
        for (index, coefficient) in divisor.iter().enumerate() {
            let term = mul(factor, *coefficient, prime);
            let slot = &mut left[shift + index];
            *slot = (*slot + prime - term) % prime;
        }
        trim(&mut left);
        if left.is_empty() {
            break;
        }
    }
    left
}

fn power(mut base: u64, mut exponent: u64, prime: u64) -> u64 {
    let mut result = 1;
    while exponent > 0 {
        if exponent & 1 == 1 {
            result = mul(result, base, prime);
        }
        base = mul(base, base, prime);
        exponent >>= 1;
    }
    result
}

/// Newton interpolation over the prime field, ascending coefficients.
fn interpolate(points: &[(u64, u64)], prime: u64) -> Vec<u64> {
    let count = points.len();
    let mut divided: Vec<u64> = points.iter().map(|&(_, value)| value).collect();
    for level in 1..count {
        for index in (level..count).rev() {
            let numerator = (divided[index] + prime - divided[index - 1]) % prime;
            let denominator = (points[index].0 + prime - points[index - level].0) % prime;
            divided[index] = mul(numerator, inverse(denominator, prime), prime);
        }
    }
    // Expand the Newton form into ascending monomial coefficients.
    let mut result = vec![0_u64; count];
    for index in (0..count).rev() {
        // result = result * (x - x_index) + divided[index]
        let root = points[index].0;
        let mut next = vec![0_u64; count];
        for (power_index, &coefficient) in result.iter().enumerate() {
            if coefficient == 0 {
                continue;
            }
            if power_index + 1 < count {
                next[power_index + 1] = (next[power_index + 1] + coefficient) % prime;
            }
            next[power_index] = (next[power_index] + prime - mul(coefficient, root, prime)) % prime;
        }
        next[0] = (next[0] + divided[index]) % prime;
        result = next;
    }
    result
}

/// Primes near `2^15`, small enough to search for roots of a minimal
/// polynomial directly and large enough that unlucky primes are rare.
const SMALL_PRIMES: [u64; 8] = [
    32_749, 32_719, 32_717, 32_713, 32_707, 32_693, 32_687, 32_653,
];

fn exact_nested(polynomial: &[Vec<Real>]) -> Option<Vec<Vec<&Rational>>> {
    polynomial
        .iter()
        .map(|coefficient| exact_trimmed(coefficient))
        .collect()
}

fn reduce_all(coefficients: &[&Rational], prime: u64) -> Option<Vec<u64>> {
    coefficients
        .iter()
        .map(|coefficient| {
            let denominator = residue(coefficient.denominator(), prime);
            if denominator == 0 {
                return None;
            }
            let value = mul(
                residue(coefficient.numerator(), prime),
                inverse(denominator, prime),
                prime,
            );
            Some(if coefficient.is_negative() && value != 0 {
                prime - value
            } else {
                value
            })
        })
        .collect()
}

fn reduce_nested(polynomial: &[Vec<&Rational>], prime: u64) -> Option<Vec<Vec<u64>>> {
    polynomial
        .iter()
        .map(|coefficient| reduce_all(coefficient, prime))
        .collect()
}

fn evaluate(coefficients: &[u64], at: u64, prime: u64) -> u64 {
    coefficients
        .iter()
        .rev()
        .fold(0, |accumulator, &coefficient| {
            (mul(accumulator, at, prime) + coefficient) % prime
        })
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

    fn over_sqrt_two(coefficients: &[&[i64]]) -> Vec<Vec<Real>> {
        coefficients
            .iter()
            .map(|coefficient| polynomial(coefficient))
            .collect()
    }

    #[test]
    fn conjugate_linear_factors_over_sqrt_two_are_certified_coprime() {
        // y - alpha and y + alpha with alpha^2 = 2.
        assert!(algebraic_extension_polynomials_certainly_coprime(
            &over_sqrt_two(&[&[0, -1], &[1]]),
            &over_sqrt_two(&[&[0, 1], &[1]]),
            &polynomial(&[-2, 0, 1]),
        ));
    }

    #[test]
    fn a_shared_extension_factor_is_never_certified_coprime() {
        // y - alpha divides y^2 - 2 over Q(sqrt 2).
        assert!(!algebraic_extension_polynomials_certainly_coprime(
            &over_sqrt_two(&[&[0, -1], &[1]]),
            &over_sqrt_two(&[&[-2], &[0], &[1]]),
            &polynomial(&[-2, 0, 1]),
        ));
    }

    #[test]
    fn a_reducible_defining_polynomial_never_certifies_a_shared_factor() {
        // defining = (x^2 - 2)(x - 3); if alpha = 3 then y - alpha and y - 3
        // share a factor. Specializing alpha to a root of x^2 - 2 modulo p
        // would wrongly certify them coprime; the resultant keeps x - 3.
        assert!(!algebraic_extension_polynomials_certainly_coprime(
            &over_sqrt_two(&[&[0, -1], &[1]]),
            &over_sqrt_two(&[&[-3], &[1]]),
            &polynomial(&[6, -2, -3, 1]),
        ));
        // Conjugate factors stay coprime at every root of that polynomial.
        assert!(algebraic_extension_polynomials_certainly_coprime(
            &over_sqrt_two(&[&[0, -1], &[1]]),
            &over_sqrt_two(&[&[0, 1], &[1]]),
            &polynomial(&[6, -2, -3, 1]),
        ));
    }

    #[test]
    fn nonzero_constants_are_coprime_to_everything() {
        assert_eq!(
            univariate_polynomials_modular_coprimality(&polynomial(&[4]), &polynomial(&[0, 1])),
            ModularCoprimality::Coprime
        );
    }
}
