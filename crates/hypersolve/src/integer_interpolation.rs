use hyperreal::{Rational, Real};
use num::{BigInt, Integer, One, Zero};

pub(crate) fn primitive_integer_polynomial(polynomial: &[Real]) -> Option<Vec<Real>> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    Some(
        Rational::primitive_integer_ratio(&rationals)
            .into_iter()
            .map(Real::from)
            .collect(),
    )
}

pub(crate) fn primitive_integer_polynomial_gcd(left: &[Real], right: &[Real]) -> Option<Vec<Real>> {
    let mut left = primitive_integer_coefficients(left)?;
    let mut right = primitive_integer_coefficients(right)?;
    if left.len() < right.len() {
        std::mem::swap(&mut left, &mut right);
    }
    while !is_zero_integer_polynomial(&right) {
        let remainder = primitive_pseudo_remainder(&left, &right)?;
        left = right;
        right = remainder;
    }
    Some(
        primitive_integer_part(left)
            .into_iter()
            .map(Rational::from_bigint)
            .map(Real::from)
            .collect(),
    )
}

fn primitive_integer_coefficients(polynomial: &[Real]) -> Option<Vec<BigInt>> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    Some(primitive_integer_part(
        Rational::primitive_integer_ratio(&rationals)
            .into_iter()
            .map(|coefficient| coefficient.to_big_integer())
            .collect::<Option<Vec<_>>>()?,
    ))
}

fn primitive_pseudo_remainder(dividend: &[BigInt], divisor: &[BigInt]) -> Option<Vec<BigInt>> {
    let divisor = primitive_integer_part(divisor.to_vec());
    if is_zero_integer_polynomial(&divisor) {
        return None;
    }
    let divisor_degree = divisor.len() - 1;
    let divisor_leading = divisor[divisor_degree].clone();
    let mut remainder = primitive_integer_part(dividend.to_vec());
    while !is_zero_integer_polynomial(&remainder) && remainder.len() > divisor_degree {
        let remainder_degree = remainder.len() - 1;
        let shift = remainder_degree - divisor_degree;
        let remainder_leading = remainder[remainder_degree].clone();
        for coefficient in &mut remainder {
            *coefficient *= &divisor_leading;
        }
        for (index, coefficient) in divisor.iter().enumerate() {
            remainder[index + shift] -= &remainder_leading * coefficient;
        }
        remainder = primitive_integer_part(remainder);
    }
    Some(remainder)
}

fn primitive_integer_part(mut polynomial: Vec<BigInt>) -> Vec<BigInt> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(BigInt::is_zero) {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        return vec![BigInt::zero()];
    }
    let content = polynomial
        .iter()
        .filter(|coefficient| !coefficient.is_zero())
        .fold(BigInt::zero(), |content, coefficient| {
            content.gcd(coefficient)
        });
    if !content.is_zero() && !content.is_one() {
        for coefficient in &mut polynomial {
            *coefficient /= &content;
        }
    }
    if polynomial
        .last()
        .is_some_and(|coefficient| coefficient < &BigInt::zero())
    {
        for coefficient in &mut polynomial {
            *coefficient = -std::mem::take(coefficient);
        }
    }
    polynomial
}

fn is_zero_integer_polynomial(polynomial: &[BigInt]) -> bool {
    polynomial.iter().all(BigInt::is_zero)
}

pub(crate) fn interpolate_integer_samples_up_to_scale(samples: &[Real]) -> Option<Vec<Real>> {
    if samples.is_empty() {
        return Some(Vec::new());
    }
    let mut differences = samples
        .iter()
        .map(|sample| sample.exact_rational_ref()?.to_big_integer())
        .collect::<Option<Vec<_>>>()?;
    let mut forward_differences = Vec::with_capacity(samples.len());
    while !differences.is_empty() {
        forward_differences.push(differences[0].clone());
        for index in 0..differences.len().saturating_sub(1) {
            let (prefix, suffix) = differences.split_at_mut(index + 1);
            prefix[index] = &suffix[0] - &prefix[index];
        }
        differences.pop();
    }

    // Newton interpolation is
    //   p(x) = sum_k Delta^k p(0) * x(x-1)...(x-k+1) / k!.
    // A defining polynomial is invariant under one nonzero scale, so build
    // n! * p(x) using integers throughout and remove its common content once.
    let degree = samples.len() - 1;
    let mut result = vec![BigInt::zero(); samples.len()];
    let mut falling_factorial = vec![BigInt::one()];
    let mut factorial_scale = (1..=degree).map(BigInt::from).product::<BigInt>();
    for (order, difference) in forward_differences.into_iter().enumerate() {
        let scale = difference * &factorial_scale;
        for (coefficient, basis) in result.iter_mut().zip(&falling_factorial) {
            *coefficient += basis * &scale;
        }
        if order < degree {
            let order = BigInt::from(order);
            let old_len = falling_factorial.len();
            falling_factorial.push(BigInt::zero());
            for power in (1..=old_len).rev() {
                falling_factorial[power] =
                    &falling_factorial[power - 1] - &order * &falling_factorial[power];
            }
            falling_factorial[0] *= -&order;
            factorial_scale /= BigInt::from(old_len);
        }
    }
    let content = result
        .iter()
        .filter(|coefficient| !coefficient.is_zero())
        .fold(BigInt::zero(), |content, coefficient| {
            content.gcd(coefficient)
        });
    if !content.is_zero() && !content.is_one() {
        for coefficient in &mut result {
            if !coefficient.is_zero() {
                *coefficient /= &content;
            }
        }
    }
    Some(
        result
            .into_iter()
            .map(Rational::from_bigint)
            .map(Real::from)
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn rational(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
    }

    #[test]
    fn primitive_integer_gcd_avoids_rational_coefficient_growth() {
        // (x - 2)(x + 3) and (x - 2)(x² + 1), with unrelated rational
        // scales, have the primitive gcd x - 2.
        let left = [rational(-3, 1), rational(1, 2), rational(1, 2)];
        let right = [
            rational(-2, 3),
            rational(1, 3),
            rational(-2, 3),
            rational(1, 3),
        ];

        assert_eq!(
            primitive_integer_polynomial_gcd(&left, &right),
            Some(vec![real(-2), real(1)])
        );
    }

    fn multiply_integer_polynomials(left: &[BigInt], right: &[BigInt]) -> Vec<BigInt> {
        let mut product = vec![BigInt::zero(); left.len() + right.len() - 1];
        for (left_index, left_coefficient) in left.iter().enumerate() {
            for (right_index, right_coefficient) in right.iter().enumerate() {
                product[left_index + right_index] += left_coefficient * right_coefficient;
            }
        }
        product
    }

    proptest! {
        #[test]
        fn generated_primitive_integer_gcd_recovers_shared_factor(
            factor in prop::collection::vec(-5_i64..=5, 1..=4),
            left_root in -5_i64..=5,
            right_root in -5_i64..=5,
        ) {
            prop_assume!(factor.last().is_some_and(|coefficient| *coefficient != 0));
            prop_assume!(left_root != right_root);
            let factor = primitive_integer_part(
                factor.into_iter().map(BigInt::from).collect()
            );
            let left = multiply_integer_polynomials(
                &factor,
                &[BigInt::from(-left_root), BigInt::one()],
            );
            let right = multiply_integer_polynomials(
                &factor,
                &[BigInt::from(-right_root), BigInt::one()],
            );
            let as_reals = |polynomial: Vec<BigInt>| {
                polynomial
                    .into_iter()
                    .map(Rational::from_bigint)
                    .map(Real::from)
                    .collect::<Vec<_>>()
            };
            let expected = as_reals(factor);

            prop_assert_eq!(
                primitive_integer_polynomial_gcd(&as_reals(left), &as_reals(right)),
                Some(expected),
            );
        }
    }

    #[test]
    fn scaled_integer_interpolation_preserves_primitive_power_basis() {
        let samples = [real(5), real(4), real(15), real(50)];
        let scaled = interpolate_integer_samples_up_to_scale(&samples).unwrap();
        assert_eq!(scaled, vec![real(5), real(-3), Real::zero(), real(2)]);

        let integer_valued_samples = [Real::zero(), Real::zero(), Real::one()];
        let scaled = interpolate_integer_samples_up_to_scale(&integer_valued_samples).unwrap();
        assert_eq!(scaled, vec![Real::zero(), real(-1), Real::one()]);
    }

    #[test]
    fn scaled_integer_interpolation_supports_factorials_beyond_i64() {
        let samples = vec![Real::one(); 23];
        let scaled = interpolate_integer_samples_up_to_scale(&samples).unwrap();

        assert_eq!(scaled[0], Real::one());
        assert!(
            scaled[1..]
                .iter()
                .all(|coefficient| coefficient == &Real::zero())
        );
    }

    #[test]
    fn scaled_integer_interpolation_rejects_fractional_samples() {
        let fractional = Real::from(Rational::fraction(1, 2).unwrap());
        assert!(interpolate_integer_samples_up_to_scale(&[fractional]).is_none());
    }
}
