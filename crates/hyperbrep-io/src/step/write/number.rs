use super::{WriteError, WriteErrorKind, WriteLimits, WriteResource, error::Site};
use hyperbrep::{Real, Vector3};
use hyperreal::Rational;
use num::{BigInt, BigUint, Integer, One, Zero, bigint::Sign};

pub(super) fn exact<'a>(
    value: &'a Real,
    limits: &WriteLimits,
    site: Site,
) -> Result<&'a Rational, WriteError> {
    let rational = value
        .exact_rational_ref()
        .ok_or_else(|| site.error(WriteErrorKind::NonRational))?;
    bits(
        rational
            .numerator()
            .bits()
            .max(rational.denominator().bits()),
        limits,
        site,
    )?;
    Ok(rational)
}
fn bits(count: u64, limits: &WriteLimits, site: Site) -> Result<(), WriteError> {
    let count = usize::try_from(count).map_err(|_| {
        site.error(WriteErrorKind::Limit {
            resource: WriteResource::NumberBits,
            limit: limits.number_bits,
            observed: usize::MAX,
        })
    })?;
    site.check(WriteResource::NumberBits, count, limits.number_bits)
}
fn product(
    left: &BigUint,
    right: &BigUint,
    limits: &WriteLimits,
    site: Site,
) -> Result<BigUint, WriteError> {
    let bound = if left.is_zero() || right.is_zero() {
        0
    } else if left.is_one() {
        right.bits()
    } else if right.is_one() {
        left.bits()
    } else {
        left.bits().saturating_add(right.bits())
    };
    bits(bound, limits, site)?;
    Ok(left * right)
}

pub(super) fn decimal(
    value: &Real,
    limits: &WriteLimits,
    site: Site,
) -> Result<String, WriteError> {
    let rational = exact(value, limits, site)?;
    site.check(WriteResource::DecimalDigits, 1, limits.number.digits)?;
    if rational.is_zero() {
        return Ok(String::from("0."));
    }
    let mut denominator = rational.denominator().clone();
    let twos = usize::try_from(denominator.trailing_zeros().expect("positive denominator"))
        .map_err(|_| {
            site.error(WriteErrorKind::Limit {
                resource: WriteResource::DecimalExponent,
                limit: limits.number.exponent,
                observed: usize::MAX,
            })
        })?;
    site.check(WriteResource::DecimalExponent, twos, limits.number.exponent)?;
    denominator >>= twos;
    let five = BigUint::from(5u8);
    let mut fives = 0usize;
    while !denominator.is_one() {
        let (quotient, remainder) = denominator.div_rem(&five);
        if !remainder.is_zero() {
            return Err(site.error(WriteErrorKind::NonTerminatingDecimal));
        }
        fives = fives.checked_add(1).ok_or_else(|| {
            site.error(WriteErrorKind::Limit {
                resource: WriteResource::DecimalExponent,
                limit: limits.number.exponent,
                observed: usize::MAX,
            })
        })?;
        site.check(
            WriteResource::DecimalExponent,
            fives,
            limits.number.exponent,
        )?;
        denominator = quotient;
    }
    let scale = twos.max(fives);
    let mut coefficient = rational.numerator().clone();
    bits(
        coefficient.bits().saturating_add((scale - twos) as u64),
        limits,
        site,
    )?;
    coefficient <<= scale - twos;
    for _ in 0..scale - fives {
        coefficient = product(&coefficient, &five, limits, site)?;
    }
    let mut digits = coefficient.to_str_radix(10);
    let significant = digits.trim_end_matches('0').len();
    let zeros = digits.len() - significant;
    digits.truncate(significant);
    site.check(
        WriteResource::DecimalDigits,
        digits.len(),
        limits.number.digits,
    )?;
    let exponent = zeros.abs_diff(scale);
    site.check(
        WriteResource::DecimalExponent,
        exponent,
        limits.number.exponent,
    )?;
    let mut output = String::new();
    output
        .try_reserve(digits.len() + 24)
        .map_err(|_| site.error(WriteErrorKind::Allocation))?;
    if rational.is_negative() {
        output.push('-');
    }
    output.push_str(&digits);
    output.push('.');
    if exponent != 0 {
        output.push('E');
        if zeros < scale {
            output.push('-');
        }
        output.push_str(&exponent.to_string());
    }
    Ok(output)
}

/// DIRECTION ratios are homogeneous: primitive integer ratios avoid unnecessary
/// nonterminating coordinates and preserve the exact orientation without roots.
pub(super) fn primitive_direction(
    vector: &Vector3,
    limits: &WriteLimits,
    site: Site,
) -> Result<Vector3, WriteError> {
    let components = [
        exact(&vector[0], limits, site)?,
        exact(&vector[1], limits, site)?,
        exact(&vector[2], limits, site)?,
    ];
    let mut denominator = BigUint::one();
    for value in components {
        let quotient = &denominator / denominator.gcd(value.denominator());
        denominator = product(&quotient, value.denominator(), limits, site)?;
    }
    let mut integers = [BigUint::zero(), BigUint::zero(), BigUint::zero()];
    let mut common = BigUint::zero();
    for (integer, value) in integers.iter_mut().zip(components) {
        *integer = product(
            value.numerator(),
            &(&denominator / value.denominator()),
            limits,
            site,
        )?;
        common = common.gcd(integer);
    }
    if common.is_zero() {
        return Err(site.error(WriteErrorKind::Invalid("zero plane direction")));
    }
    let mut values = [Real::zero(), Real::zero(), Real::zero()];
    for (target, (integer, source)) in values.iter_mut().zip(integers.into_iter().zip(components)) {
        *target = Real::new(Rational::from_bigint(BigInt::from_biguint(
            if source.is_negative() {
                Sign::Minus
            } else {
                Sign::Plus
            },
            integer / &common,
        )));
    }
    Ok(Vector3::new(values))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_decimal_factorization_preserves_reduced_rationals_and_signs() {
        let limits = WriteLimits::default();
        for twos in 0..10 {
            for fives in 0..10 {
                let denominator = Real::from(2i64.pow(twos) * 5i64.pow(fives));
                for numerator in [-101, -1, 0, 1, 101, 1000] {
                    let value = (Real::from(numerator) / &denominator).unwrap();
                    let text = decimal(&value, &limits, Site::default()).unwrap();
                    assert!(text.contains('.'));
                    assert_eq!(
                        Real::new(text.parse::<Rational>().unwrap()),
                        value,
                        "{text}"
                    );
                }
            }
        }
        for (n, d, expected) in [
            (1, 8, "125.E-3"),
            (-3, 40, "-75.E-3"),
            (1000, 1, "1.E3"),
            (0, 1, "0."),
        ] {
            let value = (Real::from(n) / Real::from(d)).unwrap();
            assert_eq!(decimal(&value, &limits, Site::default()).unwrap(), expected);
        }
        for prime in [3, 7, 11, 127] {
            let value = (Real::one() / Real::from(prime)).unwrap();
            assert!(matches!(
                decimal(&value, &limits, Site::default()).unwrap_err().kind,
                WriteErrorKind::NonTerminatingDecimal
            ));
        }
    }
}
