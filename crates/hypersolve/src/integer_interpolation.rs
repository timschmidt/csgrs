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

    fn real(value: i64) -> Real {
        Real::from(value)
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
