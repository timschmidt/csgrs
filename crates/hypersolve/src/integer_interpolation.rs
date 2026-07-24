use hyperreal::{Rational, Real};

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
        .map(|sample| {
            sample
                .exact_rational_ref()
                .filter(|sample| sample.is_integer())
                .cloned()
        })
        .collect::<Option<Vec<_>>>()?;
    let mut forward_differences = Vec::with_capacity(samples.len());
    while !differences.is_empty() {
        forward_differences.push(differences[0].clone());
        differences = differences
            .windows(2)
            .map(|pair| &pair[1] - &pair[0])
            .collect();
    }

    // Newton interpolation is
    //   p(x) = sum_k Delta^k p(0) * x(x-1)...(x-k+1) / k!.
    // A defining polynomial is invariant under one nonzero scale, so build
    // n! * p(x) using integers throughout and remove its common content once.
    let degree = samples.len() - 1;
    let mut result = vec![Rational::zero(); samples.len()];
    let mut falling_factorial = vec![Rational::one()];
    for (order, difference) in forward_differences.into_iter().enumerate() {
        let factorial_scale = ((order + 1)..=degree).try_fold(1_i64, |scale, factor| {
            scale.checked_mul(i64::try_from(factor).ok()?)
        })?;
        let scale = difference * Rational::new(factorial_scale);
        for (coefficient, basis) in result.iter_mut().zip(&falling_factorial) {
            *coefficient = &*coefficient + &(basis * &scale);
        }
        if order < degree {
            let mut next = vec![Rational::zero(); falling_factorial.len() + 1];
            let constant = Rational::new(-i64::try_from(order).ok()?);
            for (power, coefficient) in falling_factorial.iter().enumerate() {
                next[power] = &next[power] + &(coefficient * &constant);
                next[power + 1] = &next[power + 1] + coefficient;
            }
            falling_factorial = next;
        }
    }
    Some(result.into_iter().map(Real::from).collect())
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
        assert_eq!(
            primitive_integer_polynomial(&scaled).unwrap(),
            vec![real(5), real(-3), Real::zero(), real(2)]
        );

        let integer_valued_samples = [Real::zero(), Real::zero(), Real::one()];
        let scaled = interpolate_integer_samples_up_to_scale(&integer_valued_samples).unwrap();
        assert_eq!(
            primitive_integer_polynomial(&scaled).unwrap(),
            vec![Real::zero(), real(-1), Real::one()]
        );
    }

    #[test]
    fn scaled_integer_interpolation_rejects_fractional_samples() {
        let fractional = Real::from(Rational::fraction(1, 2).unwrap());
        assert!(interpolate_integer_samples_up_to_scale(&[fractional]).is_none());
    }
}
