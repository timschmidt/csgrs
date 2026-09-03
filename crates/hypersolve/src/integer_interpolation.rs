use hyperreal::{Rational, Real};
use num::{BigInt, Integer, One, Signed, ToPrimitive, Zero};

pub(crate) fn primitive_integer_polynomial(polynomial: &[Real]) -> Option<Vec<Real>> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    Some(
        Rational::primitive_bigint_ratio(&rationals)
            .into_iter()
            .map(Rational::from_bigint)
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
    if let Some(gcd) = modular_integer_polynomial_gcd(&left, &right) {
        return Some(
            gcd.into_iter()
                .map(Rational::from_bigint)
                .map(Real::from)
                .collect(),
        );
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

/// Builds the sign-preserving primitive integer Sturm chain for an exact
/// rational polynomial. Every member differs from the ordinary field chain by
/// one positive rational scale.
pub(crate) fn primitive_integer_sturm_sequence(coefficients: &[Real]) -> Option<Vec<Vec<Real>>> {
    let rationals = coefficients
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let first = primitive_integer_content_part(Rational::primitive_bigint_ratio(&rationals));
    let derivative = primitive_integer_content_part(
        first
            .iter()
            .enumerate()
            .skip(1)
            .map(|(degree, coefficient)| coefficient * BigInt::from(degree))
            .collect(),
    );
    let mut sequence = vec![first];
    if !is_zero_integer_polynomial(&derivative) {
        sequence.push(derivative);
    }
    while sequence.len() >= 2 {
        let previous = sequence[sequence.len() - 2].clone();
        let divisor = &sequence[sequence.len() - 1];
        if divisor.len() == 1 {
            break;
        }
        let mut remainder = primitive_pseudo_remainder(&previous, divisor)?;
        if is_zero_integer_polynomial(&remainder) {
            break;
        }
        for coefficient in &mut remainder {
            *coefficient = -std::mem::take(coefficient);
        }
        sequence.push(remainder);
    }
    Some(
        sequence
            .into_iter()
            .map(|polynomial| {
                polynomial
                    .into_iter()
                    .map(Rational::from_bigint)
                    .map(Real::from)
                    .collect()
            })
            .collect(),
    )
}

fn modular_polynomial_gcd(mut left: Vec<u64>, mut right: Vec<u64>, prime: u64) -> Option<Vec<u64>> {
    modular_trim(&mut left);
    modular_trim(&mut right);
    while !modular_is_zero(&right) {
        let remainder = modular_polynomial_remainder(left, &right, prime)?;
        left = right;
        right = remainder;
    }
    modular_trim(&mut left);
    let leading_inverse = modular_power(*left.last()?, prime.checked_sub(2)?, prime);
    for coefficient in &mut left {
        *coefficient = modular_multiply(*coefficient, leading_inverse, prime);
    }
    Some(left)
}

fn modular_polynomial_remainder(
    mut dividend: Vec<u64>,
    divisor: &[u64],
    prime: u64,
) -> Option<Vec<u64>> {
    if modular_is_zero(divisor) {
        return None;
    }
    modular_trim(&mut dividend);
    let divisor_degree = divisor.len().checked_sub(1)?;
    let inverse = modular_power(*divisor.last()?, prime.checked_sub(2)?, prime);
    while !modular_is_zero(&dividend) && dividend.len() >= divisor.len() {
        let shift = dividend.len() - divisor.len();
        let scale = modular_multiply(*dividend.last()?, inverse, prime);
        for (index, coefficient) in divisor.iter().enumerate().take(divisor_degree + 1) {
            let product = modular_multiply(scale, *coefficient, prime);
            let target = shift + index;
            dividend[target] = (dividend[target] + prime - product) % prime;
        }
        modular_trim(&mut dividend);
    }
    Some(dividend)
}

fn modular_power(mut base: u64, mut exponent: u64, modulus: u64) -> u64 {
    let mut result = 1_u64;
    while exponent != 0 {
        if exponent & 1 != 0 {
            result = modular_multiply(result, base, modulus);
        }
        base = modular_multiply(base, base, modulus);
        exponent >>= 1;
    }
    result
}

fn modular_multiply(left: u64, right: u64, modulus: u64) -> u64 {
    (u128::from(left) * u128::from(right) % u128::from(modulus)) as u64
}

fn modular_trim(polynomial: &mut Vec<u64>) {
    while polynomial.len() > 1 && polynomial.last() == Some(&0) {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(0);
    }
}

fn modular_is_zero(polynomial: &[u64]) -> bool {
    polynomial.iter().all(|coefficient| *coefficient == 0)
}

/// Reconstructs the primitive integer GCD from degree-preserving modular
/// images and accepts it only after exact division of both source polynomials.
/// Unlucky primes can only raise the modular GCD degree; keeping the smallest
/// degree seen and requiring exact divisibility makes every returned result a
/// complete characteristic-zero certificate. Exhausting the reconstruction
/// schedule deliberately falls through to the fraction-free PRS.
fn modular_integer_polynomial_gcd(left: &[BigInt], right: &[BigInt]) -> Option<Vec<BigInt>> {
    if is_zero_integer_polynomial(left) || is_zero_integer_polynomial(right) {
        return None;
    }
    let mut common_leading = None;
    let mut next_prime = 2_147_483_647_u64;
    let mut best_degree = None;
    let mut reconstruction: Option<(BigInt, Vec<BigInt>)> = None;

    // Each image contributes about 31 coefficient bits. This schedule covers
    // factors up to roughly eight thousand bits before the complete PRS
    // fallback; most geometric repeated factors reconstruct after one or two
    // images.
    for _ in 0..256 {
        let prime = previous_prime(next_prime)?;
        next_prime = prime.checked_sub(2)?;
        let modulus = BigInt::from(prime);
        let reduce = |polynomial: &[BigInt]| {
            polynomial
                .iter()
                .map(|coefficient| coefficient.mod_floor(&modulus).to_u64())
                .collect::<Option<Vec<_>>>()
        };
        let (Some(left_modular), Some(right_modular)) = (reduce(left), reduce(right)) else {
            continue;
        };
        if left_modular.last() == Some(&0) || right_modular.last() == Some(&0) {
            continue;
        }
        let mut modular_gcd = modular_polynomial_gcd(left_modular, right_modular, prime)?;
        let degree = modular_gcd.len().saturating_sub(1);
        if degree == 0 {
            return Some(vec![BigInt::one()]);
        }
        let common_leading = common_leading.get_or_insert_with(|| {
            euclidean_bigint_gcd(&left[left.len() - 1], &right[right.len() - 1])
        });
        let common_leading_modular = common_leading.mod_floor(&modulus).to_u64()?;
        if common_leading_modular == 0 {
            continue;
        }
        match best_degree {
            Some(best) if degree > best => continue,
            Some(best) if degree == best => {}
            Some(_) | None => {
                best_degree = Some(degree);
                reconstruction = None;
            }
        }
        for coefficient in &mut modular_gcd {
            *coefficient = modular_multiply(*coefficient, common_leading_modular, prime);
        }
        extend_modular_reconstruction(&mut reconstruction, &modular_gcd, prime)?;
        let (reconstruction_modulus, coefficients) = reconstruction.as_ref()?;
        let half_modulus = reconstruction_modulus >> 1_usize;
        let candidate = primitive_integer_part(
            coefficients
                .iter()
                .map(|coefficient| {
                    if coefficient > &half_modulus {
                        coefficient - reconstruction_modulus
                    } else {
                        coefficient.clone()
                    }
                })
                .collect(),
        );
        if candidate.len().saturating_sub(1) == degree
            && integer_polynomial_divides(left, &candidate)
            && integer_polynomial_divides(right, &candidate)
        {
            return Some(candidate);
        }
    }
    None
}

fn extend_modular_reconstruction(
    reconstruction: &mut Option<(BigInt, Vec<BigInt>)>,
    image: &[u64],
    prime: u64,
) -> Option<()> {
    let Some((modulus, coefficients)) = reconstruction else {
        *reconstruction = Some((
            BigInt::from(prime),
            image.iter().copied().map(BigInt::from).collect(),
        ));
        return Some(());
    };
    if coefficients.len() != image.len() {
        return None;
    }
    let prime_bigint = BigInt::from(prime);
    let modulus_image = modulus.mod_floor(&prime_bigint).to_u64()?;
    let modulus_inverse = modular_power(modulus_image, prime.checked_sub(2)?, prime);
    for (coefficient, image) in coefficients.iter_mut().zip(image) {
        let coefficient_image = coefficient.mod_floor(&prime_bigint).to_u64()?;
        let delta = (*image + prime - coefficient_image) % prime;
        let lift = modular_multiply(delta, modulus_inverse, prime);
        *coefficient += &*modulus * BigInt::from(lift);
    }
    *modulus *= prime;
    Some(())
}

fn integer_polynomial_divides(dividend: &[BigInt], divisor: &[BigInt]) -> bool {
    if is_zero_integer_polynomial(divisor) {
        return false;
    }
    let mut remainder = dividend.to_vec();
    while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
        remainder.pop();
    }
    while !is_zero_integer_polynomial(&remainder) && remainder.len() >= divisor.len() {
        let Some(quotient) = remainder.last().and_then(|coefficient| {
            coefficient
                .is_multiple_of(divisor.last()?)
                .then(|| coefficient / divisor.last().expect("a nonzero divisor has a leader"))
        }) else {
            return false;
        };
        let shift = remainder.len() - divisor.len();
        for (index, coefficient) in divisor[..divisor.len() - 1].iter().enumerate() {
            remainder[shift + index] -= &quotient * coefficient;
        }
        remainder.pop();
        while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
            remainder.pop();
        }
    }
    is_zero_integer_polynomial(&remainder)
}

fn previous_prime(mut candidate: u64) -> Option<u64> {
    if candidate.is_multiple_of(2) {
        candidate = candidate.checked_sub(1)?;
    }
    loop {
        if is_prime(candidate) {
            return Some(candidate);
        }
        candidate = candidate.checked_sub(2)?;
    }
}

fn is_prime(candidate: u64) -> bool {
    if candidate < 2 {
        return false;
    }
    for prime in [2_u64, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37] {
        if candidate.is_multiple_of(prime) {
            return candidate == prime;
        }
    }
    let exponent_twos = (candidate - 1).trailing_zeros();
    let odd_exponent = (candidate - 1) >> exponent_twos;
    'witness: for base in [2_u64, 3, 5, 7, 11] {
        let mut value = modular_power(base % candidate, odd_exponent, candidate);
        if value == 1 || value == candidate - 1 {
            continue;
        }
        for _ in 1..exponent_twos {
            value = modular_multiply(value, value, candidate);
            if value == candidate - 1 {
                continue 'witness;
            }
        }
        return false;
    }
    true
}

fn primitive_integer_coefficients(polynomial: &[Real]) -> Option<Vec<BigInt>> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    Some(primitive_integer_part(Rational::primitive_bigint_ratio(
        &rationals,
    )))
}

fn primitive_pseudo_remainder(dividend: &[BigInt], divisor: &[BigInt]) -> Option<Vec<BigInt>> {
    if is_zero_integer_polynomial(divisor) {
        return None;
    }
    let divisor_degree = divisor.len() - 1;
    let divisor_leading = divisor[divisor_degree].clone();
    let mut remainder = dividend.to_vec();
    let mut steps = 0_usize;
    while !is_zero_integer_polynomial(&remainder) && remainder.len() > divisor_degree {
        let remainder_degree = remainder.len() - 1;
        let shift = remainder_degree - divisor_degree;
        let remainder_leading = remainder[remainder_degree].clone();
        // CORE's fraction-free reduction cancels the leading-coefficient
        // gcd before scaling either polynomial.  The resulting combination
        // is still an exact pseudo-remainder up to a nonzero integer factor,
        // but avoids manufacturing that factor in every coefficient.
        let leading_gcd = euclidean_bigint_gcd(&remainder_leading, &divisor_leading);
        let remainder_scale = &divisor_leading / &leading_gcd;
        let divisor_scale = &remainder_leading / leading_gcd;
        for coefficient in &mut remainder {
            *coefficient *= &remainder_scale;
        }
        for (index, coefficient) in divisor.iter().enumerate() {
            remainder[index + shift] -= &divisor_scale * coefficient;
        }
        while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
            remainder.pop();
        }
        steps += 1;
    }
    if divisor_leading < BigInt::zero() && !steps.is_multiple_of(2) {
        for coefficient in &mut remainder {
            *coefficient = -std::mem::take(coefficient);
        }
    }
    Some(primitive_integer_content_part(remainder))
}

fn primitive_integer_content_part(mut polynomial: Vec<BigInt>) -> Vec<BigInt> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(BigInt::is_zero) {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        return vec![BigInt::zero()];
    }
    let content = integer_polynomial_content(&polynomial);
    if !content.is_zero() && !content.is_one() {
        for coefficient in &mut polynomial {
            *coefficient /= &content;
        }
    }
    polynomial
}

fn primitive_integer_part(polynomial: Vec<BigInt>) -> Vec<BigInt> {
    let mut polynomial = primitive_integer_content_part(polynomial);
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

/// Exact Euclidean GCD for the large, similarly sized coefficients produced
/// by fraction-free polynomial arithmetic.
///
/// `num-integer` uses the binary algorithm for `BigInt::gcd`; on large PRS
/// coefficients its shift/subtract loop can dominate the complete Sturm
/// construction. Quotient/remainder reduction reaches the identical positive
/// gcd in far fewer big-integer operations for this workload.
fn euclidean_bigint_gcd(left: &BigInt, right: &BigInt) -> BigInt {
    let mut left = left.abs();
    let mut right = right.abs();
    if left < right {
        std::mem::swap(&mut left, &mut right);
    }
    while !right.is_zero() {
        let remainder = left % &right;
        left = right;
        right = remainder;
    }
    left
}

fn integer_polynomial_content(polynomial: &[BigInt]) -> BigInt {
    let Some((smallest_index, smallest)) = polynomial
        .iter()
        .enumerate()
        .filter(|(_, coefficient)| !coefficient.is_zero())
        .min_by_key(|(_, coefficient)| coefficient.magnitude().bits())
    else {
        return BigInt::zero();
    };
    let mut content = smallest.abs();
    for (index, coefficient) in polynomial.iter().enumerate() {
        if index == smallest_index || coefficient.is_zero() {
            continue;
        }
        content = euclidean_bigint_gcd(&content, coefficient);
        // One is the terminal content. Continuing through large resultant
        // coefficients can otherwise spend most of a primitive PRS in GCDs
        // whose outcome is already known.
        if content.is_one() {
            break;
        }
    }
    content
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
    let content = integer_polynomial_content(&result);
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

    #[test]
    fn modular_gcd_certifies_coprimality_without_integer_coefficient_growth() {
        let mut left = vec![Real::zero(); 129];
        left[0] = real(1);
        left[128] = real(1);
        let mut right = vec![Real::zero(); 128];
        right[0] = real(3);
        right[127] = real(128);
        assert_eq!(
            primitive_integer_polynomial_gcd(&left, &right),
            Some(vec![Real::one()])
        );
    }

    #[test]
    fn modular_gcd_retains_a_shared_factor() {
        let shared = [real(-2), real(1)];
        let left = [real(-6), real(1), real(1)];
        let right = [real(-2), real(1), real(-2), real(1)];
        assert_eq!(
            primitive_integer_polynomial_gcd(&left, &right),
            Some(shared.to_vec())
        );
    }

    #[test]
    fn modular_reconstruction_certifies_a_wide_primitive_gcd() {
        let wide = BigInt::one() << 192_usize;
        let factor = vec![&wide + 17_u8, -(&wide >> 1_usize) + 5_u8, BigInt::one()];
        let left = multiply_integer_polynomials(&factor, &[BigInt::from(-2_i8), BigInt::one()]);
        let right = multiply_integer_polynomials(&factor, &[BigInt::from(3_i8), BigInt::one()]);

        assert_eq!(modular_integer_polynomial_gcd(&left, &right), Some(factor));
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
