use hyperreal::{Rational, Real};
use num::{BigInt, Integer, One, Signed, Zero};

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

/// Divides exact rational polynomials through their primitive integer parts.
/// Exact integer division certifies the quotient; one final rational scale
/// restores the original leading coefficients. A nonzero remainder declines
/// this path without weakening the caller's general coefficient-field replay.
pub(crate) fn rational_polynomial_exact_quotient(
    dividend: &[Real],
    divisor: &[Real],
) -> Option<Vec<Real>> {
    let first = primitive_integer_coefficients(dividend)?;
    let second = primitive_integer_coefficients(divisor)?;
    let quotient = integer_polynomial_exact_quotient(&first, &second)?;
    if is_zero_integer_polynomial(&quotient) {
        return Some(vec![Real::zero()]);
    }
    let scale = dividend[first.len() - 1].exact_rational_ref()?
        / divisor[second.len() - 1].exact_rational_ref()?
        / Rational::from_bigint(quotient.last()?.clone());
    Some(
        quotient
            .into_iter()
            .map(|coefficient| Real::new(Rational::from_bigint(coefficient) * &scale))
            .collect(),
    )
}

/// Multiplies rational polynomials in one quotient ring without reducing a
/// rational fraction for every coefficient product. Integer pseudo-division
/// retains its exact scale; only the final reduced coefficients return to Real.
pub(crate) fn rational_polynomial_product_modulo(
    left: &[Real],
    right: &[Real],
    modulus: &[Real],
) -> Option<Vec<Real>> {
    if left.is_empty() || right.is_empty() || modulus.is_empty() {
        return None;
    }
    let first = primitive_integer_coefficients(left)?;
    let second = primitive_integer_coefficients(right)?;
    let modulus = primitive_integer_coefficients(modulus)?;
    if modulus.len() < 2 {
        return None;
    }
    if is_zero_integer_polynomial(&first) || is_zero_integer_polynomial(&second) {
        return Some(vec![Real::zero()]);
    }
    let source_scale = |source: &[Real], primitive: &[BigInt]| {
        let pivot = primitive.iter().position(|value| !value.is_zero())?;
        (&source[pivot] / Real::from(Rational::from_bigint(primitive[pivot].clone()))).ok()
    };
    let source_scale = source_scale(left, &first)? * source_scale(right, &second)?;
    let mut remainder = vec![BigInt::zero(); first.len() + second.len() - 1];
    for (first_power, first) in first.iter().enumerate() {
        for (second_power, second) in second.iter().enumerate() {
            remainder[first_power + second_power] += first * second;
        }
    }
    let (remainder, denominator) = integer_polynomial_remainder_modulo(remainder, &modulus)?;
    let scale = (source_scale / Real::from(Rational::from_bigint(denominator))).ok()?;
    Some(
        remainder
            .into_iter()
            .map(|value| Real::from(Rational::from_bigint(value)) * &scale)
            .collect(),
    )
}

/// Reduces a rational polynomial through integer pseudo-division, restoring
/// its exact coefficient scale only after all leading powers are removed.
pub(crate) fn rational_polynomial_remainder_modulo(
    polynomial: &[Real],
    modulus: &[Real],
) -> Option<Vec<Real>> {
    if polynomial.is_empty() {
        return None;
    }
    let primitive = primitive_integer_coefficients(polynomial)?;
    let modulus = primitive_integer_coefficients(modulus)?;
    if modulus.len() < 2 {
        return None;
    }
    let Some(pivot) = primitive.iter().position(|value| !value.is_zero()) else {
        return Some(vec![Real::zero()]);
    };
    let source_scale =
        polynomial[pivot].exact_rational_ref()? / Rational::from_bigint(primitive[pivot].clone());
    let (remainder, denominator) = integer_polynomial_remainder_modulo(primitive, &modulus)?;
    let scale = source_scale / Rational::from_bigint(denominator);
    Some(
        remainder
            .into_iter()
            .map(|value| Real::new(Rational::from_bigint(value) * &scale))
            .collect(),
    )
}

fn integer_polynomial_remainder_modulo(
    mut remainder: Vec<BigInt>,
    modulus: &[BigInt],
) -> Option<(Vec<BigInt>, BigInt)> {
    let degree = modulus.len().checked_sub(1)?;
    let leading = modulus.last()?;
    let mut scale = BigInt::one();
    while remainder.len() > degree && !is_zero_integer_polynomial(&remainder) {
        let shift = remainder.len() - modulus.len();
        let top = remainder.last()?;
        let common = BigInt::from(Rational::gcd_magnitudes(
            top.magnitude(),
            leading.magnitude(),
        ));
        let remainder_scale = leading / &common;
        let modulus_scale = top / common;
        if !remainder_scale.is_one() {
            for coefficient in &mut remainder {
                *coefficient *= &remainder_scale;
            }
        }
        for (power, coefficient) in modulus.iter().enumerate() {
            remainder[shift + power] -= &modulus_scale * coefficient;
        }
        scale *= remainder_scale;
        while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
            remainder.pop();
        }
        // Only remove content shared with the accumulated scale: arbitrary
        // primitive normalization would change the represented field element.
        if !scale.is_one() {
            let mut common = scale.clone();
            for coefficient in &remainder {
                common = BigInt::from(Rational::gcd_magnitudes(
                    common.magnitude(),
                    coefficient.magnitude(),
                ));
                if common.is_one() {
                    break;
                }
            }
            if !common.is_one() {
                for coefficient in &mut remainder {
                    *coefficient /= &common;
                }
                scale /= common;
            }
        }
    }
    Some((remainder, scale))
}

/// Reconstructs an inverse over Q and accepts it only after the exact identity
/// `polynomial * inverse == 1 (mod modulus)` replays over the integers.
/// Failure only declines this accelerator; callers retain their exact-field
/// fallback, including selection of a local factor of a reducible modulus.
pub(crate) fn rational_polynomial_inverse_modulo(
    polynomial: &[Real],
    modulus: &[Real],
) -> Option<Vec<Real>> {
    let source = primitive_integer_coefficients(polynomial)?;
    let modulus = primitive_integer_coefficients(modulus)?;
    let degree = modulus.len().checked_sub(1)?;
    if degree == 0 || is_zero_integer_polynomial(&source) {
        return None;
    }
    let pivot = source
        .iter()
        .position(|coefficient| !coefficient.is_zero())?;
    let source_scale =
        (Real::from(Rational::from_bigint(source[pivot].clone())) / &polynomial[pivot]).ok()?;
    let mut next_prime = 2_147_483_647_u64;
    let mut reconstruction = None;
    let mut images = 0_usize;
    for _ in 0..256 {
        let prime = previous_prime(next_prime)?;
        next_prime = prime.checked_sub(2)?;
        let reduce = |polynomial: &[BigInt]| {
            polynomial
                .iter()
                .map(|value| bigint_modulo_u64(value, prime))
                .collect::<Vec<_>>()
        };
        let modular_modulus = reduce(&modulus);
        if modular_modulus.last() == Some(&0) {
            continue;
        }
        let Some(mut inverse) = modular_polynomial_inverse(reduce(&source), modular_modulus, prime)
        else {
            // A bad prime is harmless: decline instead of inferring anything
            // about characteristic-zero invertibility or the selected root.
            return None;
        };
        inverse.resize(degree, 0);
        extend_modular_reconstruction(&mut reconstruction, &inverse, prime)?;
        images += 1;
        if !images.is_multiple_of(4) {
            continue;
        }
        let (reconstruction_modulus, residues) = reconstruction.as_ref()?;
        let Some((numerator, denominator)) =
            reconstruct_rational_polynomial(reconstruction_modulus, residues)
        else {
            continue;
        };
        let mut residual = vec![BigInt::zero(); source.len() + numerator.len() - 1];
        for (first_power, first) in source.iter().enumerate() {
            for (second_power, second) in numerator.iter().enumerate() {
                residual[first_power + second_power] += first * second;
            }
        }
        residual[0] -= &denominator;
        if integer_polynomial_exact_quotient(&residual, &modulus).is_none() {
            continue;
        }
        let mut inverse = numerator
            .into_iter()
            .map(|value| {
                Rational::from_bigint_fraction(value, denominator.magnitude().clone())
                    .ok()
                    .map(|value| Real::from(value) * &source_scale)
            })
            .collect::<Option<Vec<_>>>()?;
        while inverse.len() > 1 && inverse.last() == Some(&Real::zero()) {
            inverse.pop();
        }
        return Some(inverse);
    }
    None
}

/// Builds the sign-preserving primitive integer Sturm chain for an exact
/// rational polynomial. Every member differs from the ordinary field chain by
/// one positive rational scale.
pub(crate) fn primitive_integer_sturm_sequence(coefficients: &[Real]) -> Option<Vec<Vec<Real>>> {
    let first = primitive_integer_coefficients(coefficients)?;
    let derivative = primitive_integer_content_part(
        first
            .iter()
            .enumerate()
            .skip(1)
            .map(|(degree, coefficient)| coefficient * BigInt::from(degree))
            .collect(),
    );
    primitive_integer_signed_sequence(first, derivative)
}

/// The same positive-scale remainder chain for an arbitrary rational pair.
/// Used by the Sturm-Tarski sign query as well as ordinary root counting.
pub(crate) fn primitive_integer_signed_remainder_sequence(
    first: &[Real],
    second: &[Real],
) -> Option<Vec<Vec<Real>>> {
    primitive_integer_signed_sequence(
        primitive_integer_coefficients(first)?,
        primitive_integer_coefficients(second)?,
    )
}

fn primitive_integer_signed_sequence(
    first: Vec<BigInt>,
    second: Vec<BigInt>,
) -> Option<Vec<Vec<Real>>> {
    let mut sequence = vec![first];
    if !is_zero_integer_polynomial(&second) {
        sequence.push(second);
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
    dividend: Vec<u64>,
    divisor: &[u64],
    prime: u64,
) -> Option<Vec<u64>> {
    Some(modular_polynomial_div_rem(dividend, divisor, prime)?.1)
}

fn modular_polynomial_div_rem(
    mut dividend: Vec<u64>,
    divisor: &[u64],
    prime: u64,
) -> Option<(Vec<u64>, Vec<u64>)> {
    if modular_is_zero(divisor) {
        return None;
    }
    modular_trim(&mut dividend);
    let divisor_degree = divisor.len().checked_sub(1)?;
    let mut quotient = vec![0; dividend.len().saturating_sub(divisor_degree).max(1)];
    let inverse = modular_power(*divisor.last()?, prime.checked_sub(2)?, prime);
    while !modular_is_zero(&dividend) && dividend.len() >= divisor.len() {
        let shift = dividend.len() - divisor.len();
        let scale = modular_multiply(*dividend.last()?, inverse, prime);
        quotient[shift] = scale;
        for (index, coefficient) in divisor.iter().enumerate().take(divisor_degree + 1) {
            let product = modular_multiply(scale, *coefficient, prime);
            let target = shift + index;
            dividend[target] = (dividend[target] + prime - product) % prime;
        }
        modular_trim(&mut dividend);
    }
    modular_trim(&mut quotient);
    Some((quotient, dividend))
}

fn modular_polynomial_inverse(
    polynomial: Vec<u64>,
    modulus: Vec<u64>,
    prime: u64,
) -> Option<Vec<u64>> {
    let mut previous = modulus.clone();
    let mut current = modular_polynomial_remainder(polynomial, &modulus, prime)?;
    let mut previous_coefficient = vec![0];
    let mut current_coefficient = vec![1];
    while !modular_is_zero(&current) {
        let (quotient, remainder) = modular_polynomial_div_rem(previous, &current, prime)?;
        let mut coefficient = previous_coefficient;
        coefficient.resize(
            coefficient
                .len()
                .max(quotient.len() + current_coefficient.len() - 1),
            0,
        );
        for (first_power, first) in quotient.iter().enumerate() {
            for (second_power, second) in current_coefficient.iter().enumerate() {
                let target = &mut coefficient[first_power + second_power];
                *target = (*target + prime - modular_multiply(*first, *second, prime)) % prime;
            }
        }
        previous = current;
        current = remainder;
        previous_coefficient = current_coefficient;
        current_coefficient = modular_polynomial_remainder(coefficient, &modulus, prime)?;
    }
    if previous.len() != 1 || previous[0] == 0 {
        return None;
    }
    let scale = modular_power(previous[0], prime.checked_sub(2)?, prime);
    for coefficient in &mut previous_coefficient {
        *coefficient = modular_multiply(*coefficient, scale, prime);
    }
    Some(previous_coefficient)
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

fn bigint_modulo_u64(value: &BigInt, modulus: u64) -> u64 {
    let modulus_wide = u128::from(modulus);
    let limb_radix = ((u128::from(u64::MAX) + 1) % modulus_wide) as u64;
    let mut remainder = 0_u64;
    for limb in value.iter_u64_digits().rev() {
        remainder = ((u128::from(remainder) * u128::from(limb_radix) + u128::from(limb % modulus))
            % modulus_wide) as u64;
    }
    if value.is_negative() && remainder != 0 {
        modulus - remainder
    } else {
        remainder
    }
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
    let mut next_prime = 2_147_483_647_u64;
    let mut best_degree = None;
    let mut reconstruction: Option<(BigInt, Vec<BigInt>)> = None;
    let mut reconstruction_images = 0_usize;

    // Each image contributes about 31 modulus bits. The rational uniqueness
    // bound therefore covers roughly four-thousand-bit numerator/denominator
    // height before the complete PRS fallback; geometric repeated factors are
    // ordinarily much smaller.
    for _ in 0..256 {
        let prime = previous_prime(next_prime)?;
        next_prime = prime.checked_sub(2)?;
        let reduce = |polynomial: &[BigInt]| {
            polynomial
                .iter()
                .map(|coefficient| bigint_modulo_u64(coefficient, prime))
                .collect::<Vec<_>>()
        };
        let (left_modular, right_modular) = (reduce(left), reduce(right));
        if left_modular.last() == Some(&0) || right_modular.last() == Some(&0) {
            continue;
        }
        let modular_gcd = modular_polynomial_gcd(left_modular, right_modular, prime)?;
        let degree = modular_gcd.len().saturating_sub(1);
        if degree == 0 {
            return Some(vec![BigInt::one()]);
        }
        match best_degree {
            Some(best) if degree > best => continue,
            Some(best) if degree == best => {}
            Some(_) | None => {
                best_degree = Some(degree);
                reconstruction = None;
                reconstruction_images = 0;
            }
        }
        extend_modular_reconstruction(&mut reconstruction, &modular_gcd, prime)?;
        reconstruction_images += 1;
        if !reconstruction_images.is_multiple_of(4) {
            continue;
        }
        let (reconstruction_modulus, coefficients) = reconstruction.as_ref()?;
        let Some(candidate) =
            reconstruct_primitive_rational_polynomial(reconstruction_modulus, coefficients)
        else {
            continue;
        };
        if candidate.len().saturating_sub(1) == degree
            && integer_polynomial_exact_quotient(left, &candidate).is_some()
            && integer_polynomial_exact_quotient(right, &candidate).is_some()
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
    let modulus_image = bigint_modulo_u64(modulus, prime);
    let modulus_inverse = modular_power(modulus_image, prime.checked_sub(2)?, prime);
    for (coefficient, image) in coefficients.iter_mut().zip(image) {
        let coefficient_image = bigint_modulo_u64(coefficient, prime);
        let delta = (*image + prime - coefficient_image) % prime;
        let lift = modular_multiply(delta, modulus_inverse, prime);
        *coefficient += &*modulus * BigInt::from(lift);
    }
    *modulus *= prime;
    Some(())
}

fn reconstruct_primitive_rational_polynomial(
    modulus: &BigInt,
    residues: &[BigInt],
) -> Option<Vec<BigInt>> {
    Some(primitive_integer_part(
        reconstruct_rational_polynomial(modulus, residues)?.0,
    ))
}

fn reconstruct_rational_polynomial(
    modulus: &BigInt,
    residues: &[BigInt],
) -> Option<(Vec<BigInt>, BigInt)> {
    let bound = BigInt::from((modulus.magnitude() >> 1_usize).sqrt());
    if bound.is_zero() {
        return None;
    }
    let rationals = residues
        .iter()
        .map(|residue| rational_reconstruction(residue, modulus, &bound))
        .collect::<Option<Vec<_>>>()?;
    let common_denominator = rationals
        .iter()
        .fold(BigInt::one(), |common, (_, denominator)| {
            let gcd = BigInt::from(Rational::gcd_magnitudes(
                common.magnitude(),
                denominator.magnitude(),
            ));
            common / gcd * denominator
        });
    Some((
        rationals
            .into_iter()
            .map(|(numerator, denominator)| numerator * (&common_denominator / denominator))
            .collect(),
        common_denominator,
    ))
}

fn rational_reconstruction(
    residue: &BigInt,
    modulus: &BigInt,
    bound: &BigInt,
) -> Option<(BigInt, BigInt)> {
    if residue.is_zero() {
        return Some((BigInt::zero(), BigInt::one()));
    }
    let mut previous_remainder = modulus.clone();
    let mut remainder = residue.clone();
    let mut previous_denominator = BigInt::zero();
    let mut denominator = BigInt::one();
    while &remainder.abs() > bound {
        if remainder.is_zero() {
            return None;
        }
        let quotient = &previous_remainder / &remainder;
        let next_remainder = previous_remainder - &quotient * &remainder;
        let next_denominator = previous_denominator - quotient * &denominator;
        previous_remainder = remainder;
        remainder = next_remainder;
        previous_denominator = denominator;
        denominator = next_denominator;
    }
    if denominator.is_zero() || &denominator.abs() > bound {
        return None;
    }
    if denominator.is_negative() {
        remainder = -remainder;
        denominator = -denominator;
    }
    // Extended Euclid maintains
    // `remainder == residue * denominator (mod modulus)` at every step.
    // The bound and coprimality checks are therefore the complete rational
    // reconstruction certificate; the final polynomial candidate is still
    // replayed by exact division against both inputs.
    if !Rational::gcd_magnitudes(remainder.magnitude(), denominator.magnitude()).is_one() {
        return None;
    }
    Some((remainder, denominator))
}

fn integer_polynomial_exact_quotient(
    dividend: &[BigInt],
    divisor: &[BigInt],
) -> Option<Vec<BigInt>> {
    if is_zero_integer_polynomial(divisor) {
        return None;
    }
    let mut remainder = dividend.to_vec();
    while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
        remainder.pop();
    }
    let mut quotient = vec![
        BigInt::zero();
        remainder
            .len()
            .saturating_sub(divisor.len())
            .saturating_add(1)
    ];
    while !is_zero_integer_polynomial(&remainder) && remainder.len() >= divisor.len() {
        let (scale, residual) = remainder.last()?.div_rem(divisor.last()?);
        if !residual.is_zero() {
            return None;
        }
        let shift = remainder.len() - divisor.len();
        for (index, coefficient) in divisor[..divisor.len() - 1].iter().enumerate() {
            remainder[shift + index] -= &scale * coefficient;
        }
        quotient[shift] = scale;
        remainder.pop();
        while remainder.len() > 1 && remainder.last().is_some_and(BigInt::is_zero) {
            remainder.pop();
        }
    }
    is_zero_integer_polynomial(&remainder).then_some(quotient)
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

/// Keeps the polynomial's sign while removing its positive rational scale.
/// Hyperreal already certifies primitive integer content; only canonical
/// degree and zero shape remain to be prepared here. Positive-leading GCD
/// output normalization belongs to `primitive_integer_part` instead.
fn primitive_integer_coefficients(polynomial: &[Real]) -> Option<Vec<BigInt>> {
    let rationals = polynomial
        .iter()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    Some(trim_integer_polynomial(Rational::primitive_bigint_ratio(
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
        let leading_gcd = BigInt::from(Rational::gcd_magnitudes(
            remainder_leading.magnitude(),
            divisor_leading.magnitude(),
        ));
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

fn trim_integer_polynomial(mut polynomial: Vec<BigInt>) -> Vec<BigInt> {
    while polynomial.len() > 1 && polynomial.last().is_some_and(BigInt::is_zero) {
        polynomial.pop();
    }
    if polynomial.is_empty() {
        polynomial.push(BigInt::zero());
    }
    polynomial
}

fn primitive_integer_content_part(polynomial: Vec<BigInt>) -> Vec<BigInt> {
    let mut polynomial = trim_integer_polynomial(polynomial);
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
        content = BigInt::from(Rational::gcd_magnitudes(
            content.magnitude(),
            coefficient.magnitude(),
        ));
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

    fn ordinary_product_remainder(left: &[Real], right: &[Real], modulus: &[Real]) -> Vec<Real> {
        let mut product = vec![Real::zero(); left.len() + right.len() - 1];
        for (i, first) in left.iter().enumerate() {
            for (j, second) in right.iter().enumerate() {
                product[i + j] += first * second;
            }
        }
        crate::root_isolation::polynomial_div_rem(product, modulus, crate::PredicatePolicy::STRICT)
            .unwrap()
            .1
    }

    proptest! {
        #[test]
        fn rational_product_modulo_preserves_the_exact_scale(
            left in prop::collection::vec((-20_i64..=20, 1_u64..=17), 1..10),
            right in prop::collection::vec((-20_i64..=20, 1_u64..=17), 1..10),
            modulus in prop::collection::vec((-20_i64..=20, 1_u64..=17), 2..10),
        ) {
            prop_assume!(modulus.last().unwrap().0 != 0);
            let values = |coefficients: Vec<(i64, u64)>| coefficients.into_iter()
                .map(|(numerator, denominator)| rational(numerator, denominator)).collect::<Vec<_>>();
            let (left, right, modulus) = (values(left), values(right), values(modulus));
            prop_assert_eq!(
                rational_polynomial_product_modulo(&left, &right, &modulus),
                Some(ordinary_product_remainder(&left, &right, &modulus)),
            );
        }
    }

    #[test]
    fn rational_product_modulo_handles_wide_coefficients_and_guards_its_domain() {
        let values = |count: usize, shift: usize| {
            (0..count)
                .map(|index| {
                    let numerator = (BigInt::one() << (4096 + 17 * index)) + (index + 1);
                    let denominator = (BigInt::one() << (shift + index)) + 3_u8;
                    Real::from(
                        Rational::from_bigint_fraction(numerator, denominator.magnitude().clone())
                            .unwrap(),
                    )
                })
                .collect::<Vec<_>>()
        };
        let left = values(10_usize, 192_usize);
        let right = values(9_usize, 256_usize);
        let mut modulus = vec![Real::zero(); 12];
        modulus[0] = rational(-2, 7);
        modulus[3] = rational(3, 11);
        modulus[11] = rational(-5, 13);
        assert_eq!(
            rational_polynomial_product_modulo(&left, &right, &modulus),
            Some(ordinary_product_remainder(&left, &right, &modulus))
        );
        assert_eq!(
            rational_polynomial_product_modulo(&[Real::zero()], &right, &modulus),
            Some(vec![Real::zero()])
        );
        let irrational = Real::from(2).sqrt().unwrap();
        assert!(
            rational_polynomial_product_modulo(&[irrational.clone()], &right, &modulus).is_none()
        );
        assert!(
            rational_polynomial_product_modulo(&left, &[irrational.clone()], &modulus).is_none()
        );
        assert!(
            rational_polynomial_product_modulo(&left, &right, &[irrational, Real::one()]).is_none()
        );
        for invalid in [vec![], vec![Real::zero()], vec![Real::one()]] {
            assert!(rational_polynomial_product_modulo(&left, &right, &invalid).is_none());
        }
        assert!(rational_polynomial_product_modulo(&[], &right, &modulus).is_none());
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
    fn exact_quotient_declines_nondivisibility_and_nonrational_coefficients() {
        assert_eq!(
            rational_polynomial_exact_quotient(&[Real::zero()], &[real(1), real(2)]),
            Some(vec![Real::zero()]),
        );
        assert!(rational_polynomial_exact_quotient(&[Real::one()], &[Real::zero()]).is_none());
        assert!(
            rational_polynomial_exact_quotient(&[real(1), real(1)], &[real(1), real(2)]).is_none()
        );
        assert!(
            rational_polynomial_exact_quotient(&[real(2).sqrt().unwrap()], &[real(1)]).is_none()
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
    fn limb_modulo_matches_exact_signed_remainders() {
        let wide = (BigInt::one() << 521_usize) + (BigInt::one() << 257_usize) + 123_456_789_u64;
        for value in [wide.clone(), -wide, BigInt::zero()] {
            for modulus in [65_521_u64, 2_147_483_647] {
                assert_eq!(
                    BigInt::from(bigint_modulo_u64(&value, modulus)),
                    value.mod_floor(&BigInt::from(modulus))
                );
            }
        }
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

    #[test]
    fn modular_inverse_reconstructs_wide_coefficients_and_rejects_nonunits() {
        let wide = BigInt::one() << 64_usize;
        let degree = 17_usize;
        let mut modulus = vec![Real::zero(); degree + 1];
        modulus[0] = Real::one();
        modulus[degree] = Real::one();
        let source = vec![Real::from(Rational::from_bigint(-&wide)), Real::one()];
        let denominator = wide.pow(degree as u32) + BigInt::one();
        let expected = (0..degree)
            .map(|power| {
                Real::from(
                    Rational::from_bigint_fraction(
                        -wide.pow((degree - 1 - power) as u32),
                        denominator.magnitude().clone(),
                    )
                    .unwrap(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(
            rational_polynomial_inverse_modulo(&source, &modulus),
            Some(expected)
        );
        assert!(
            rational_polynomial_inverse_modulo(&[real(-2), real(1)], &[real(-4), real(0), real(1)])
                .is_none()
        );
        assert!(rational_polynomial_inverse_modulo(&[Real::zero()], &modulus).is_none());
        assert!(rational_polynomial_inverse_modulo(&source, &[Real::one()]).is_none());
        assert!(rational_polynomial_inverse_modulo(&[real(2).sqrt().unwrap()], &modulus).is_none());
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
        fn generated_exact_quotient_preserves_independent_rational_scales(
            divisor in prop::collection::vec(-9_i64..=9, 1..=8),
            quotient in prop::collection::vec(-9_i64..=9, 1..=8),
            dividend_scale in -7_i64..=7,
            divisor_scale in -7_i64..=7,
            denominator in 1_u64..=9,
        ) {
            prop_assume!(divisor.last().is_some_and(|coefficient| *coefficient != 0));
            prop_assume!(quotient.last().is_some_and(|coefficient| *coefficient != 0));
            prop_assume!(dividend_scale != 0 && divisor_scale != 0);
            let divisor = divisor.into_iter().map(BigInt::from).collect::<Vec<_>>();
            let quotient = quotient.into_iter().map(BigInt::from).collect::<Vec<_>>();
            let product = multiply_integer_polynomials(&divisor, &quotient);
            let numerator_scale = rational(dividend_scale, denominator);
            let divisor_scale = real(divisor_scale);
            let scaled = |polynomial: &[BigInt], scale: &Real| polynomial.iter()
                .map(|coefficient| Real::new(Rational::from_bigint(coefficient.clone())) * scale)
                .collect::<Vec<_>>();
            let expected = scaled(&quotient, &(&numerator_scale / &divisor_scale).unwrap());
            prop_assert_eq!(
                rational_polynomial_exact_quotient(
                    &scaled(&product, &numerator_scale),
                    &scaled(&divisor, &divisor_scale),
                ),
                Some(expected),
            );
        }

        #[test]
        fn generated_modular_inverse_preserves_rational_scales(
            constant in -9_i64..=9,
            linear in -9_i64..=9,
            denominator in 1_u64..=9,
            positive in 1_i64..=9,
            modulus_scale in -9_i64..=9,
        ) {
            prop_assume!(constant != 0 || linear != 0);
            prop_assume!(modulus_scale != 0);
            // In Q[x]/(x^2+positive), (b+a*x)^-1 is
            // (b-a*x)/(b^2+positive*a^2), with no real-factor ambiguity.
            let source = [rational(constant, denominator), rational(linear, denominator)];
            let modulus = [real(positive * modulus_scale), Real::zero(), real(modulus_scale)];
            let norm = (constant * constant + positive * linear * linear) as u64;
            let mut expected = vec![rational(constant * denominator as i64, norm), rational(-linear * denominator as i64, norm)];
            if linear == 0 { expected.pop(); }
            prop_assert_eq!(rational_polynomial_inverse_modulo(&source, &modulus), Some(expected));
        }

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
