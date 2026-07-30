//! Canonical scalar policy for HyperDRC.
//!
//! Internal measurements, coordinates, rule values, and computed quantities
//! use [`Scalar`]. Primitive floats are reserved for named input/output
//! adapters that must interoperate with finite external formats.

use hyperreal::{Rational, Real};
use serde::{Deserialize, Deserializer};
use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, Sign};

/// HyperDRC's sole internal scalar type.
pub type Scalar = Real;

/// Compare two internal scalars through the workspace predicate policy.
pub(crate) fn compare(left: &Scalar, right: &Scalar) -> Option<Ordering> {
    compare_with_policy(left, right, hyperlimit::PredicatePolicy)
}

/// Compare two internal scalars through an explicit predicate policy.
pub(crate) fn compare_with_policy(
    left: &Scalar,
    right: &Scalar,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    hyperlimit::compare_reals_with_policy(left, right, policy).value()
}

/// Classify an internal scalar sign through the workspace predicate policy.
pub(crate) fn sign(value: &Scalar) -> Option<Sign> {
    sign_with_policy(value, hyperlimit::PredicatePolicy)
}

/// Classify an internal scalar sign through an explicit predicate policy.
pub(crate) fn sign_with_policy(value: &Scalar, policy: PredicatePolicy) -> Option<Sign> {
    hyperlimit::classify_real_sign_with_policy(value, policy).value()
}

/// Whether two internal scalars are certified equal under the workspace policy.
pub(crate) fn eq(left: &Scalar, right: &Scalar) -> bool {
    compare(left, right) == Some(Ordering::Equal)
}

/// Whether two internal scalars are certified unequal under the workspace policy.
pub(crate) fn ne(left: &Scalar, right: &Scalar) -> bool {
    matches!(
        compare(left, right),
        Some(Ordering::Less | Ordering::Greater)
    )
}

/// Whether `left < right` is certified under the workspace policy.
pub(crate) fn lt(left: &Scalar, right: &Scalar) -> bool {
    compare(left, right) == Some(Ordering::Less)
}

/// Whether `left <= right` is certified under the workspace policy.
pub(crate) fn le(left: &Scalar, right: &Scalar) -> bool {
    matches!(compare(left, right), Some(Ordering::Less | Ordering::Equal))
}

/// Whether `left > right` is certified under the workspace policy.
pub(crate) fn gt(left: &Scalar, right: &Scalar) -> bool {
    compare(left, right) == Some(Ordering::Greater)
}

/// Whether `left >= right` is certified under the workspace policy.
pub(crate) fn ge(left: &Scalar, right: &Scalar) -> bool {
    matches!(
        compare(left, right),
        Some(Ordering::Greater | Ordering::Equal)
    )
}

/// Parse a source decimal directly into the exact scalar domain.
///
/// Parsing through [`Rational`] retains the source value rather than first
/// rounding it to an IEEE-754 dyadic. Callers should use this at textual I/O
/// boundaries and carry the returned [`Scalar`] through all internal work.
pub(crate) fn parse_source_scalar(token: &str) -> Option<Scalar> {
    let token = token.trim();
    let (mantissa, exponent) = match token.split_once(['e', 'E']) {
        Some((mantissa, exponent)) => (mantissa, exponent.parse::<i32>().ok()?),
        None => (token, 0_i32),
    };
    let mantissa = mantissa.parse::<Rational>().ok()?;
    let scale = rational_power_of_ten(exponent.unsigned_abs());
    let value = if exponent < 0 {
        mantissa / scale
    } else {
        mantissa * scale
    };
    Some(Scalar::from(value))
}

fn rational_power_of_ten(mut exponent: u32) -> Rational {
    let mut result = Rational::new(1);
    let mut factor = Rational::new(10);
    while exponent != 0 {
        if exponent & 1 == 1 {
            result *= &factor;
        }
        exponent >>= 1;
        if exponent != 0 {
            factor = &factor * &factor;
        }
    }
    result
}

/// Construct an exact scalar from a trusted decimal literal.
pub fn scalar(token: &str) -> Scalar {
    parse_source_scalar(token).expect("trusted HyperDRC scalar literal must be a rational")
}

/// Divide an internal scalar by the exact, nonzero integer two.
///
/// Keeping this invariant operation infallible prevents callers from silently
/// discarding geometry through a `Real` division error branch that cannot be
/// reached for this denominator.
pub(crate) fn half(value: &Scalar) -> Scalar {
    (value.clone() / Scalar::from(2_u8))
        .expect("division by the exact nonzero scalar two cannot fail")
}

/// Deserialize an optional JSON number directly into the exact scalar domain.
///
/// `serde_json::Number` preserves the source decimal spelling. Converting its
/// display form through `Rational` avoids the intermediate IEEE-754 rounding
/// that `deserialize_f64` would introduce at the configuration edge.
pub(crate) fn deserialize_optional<'de, D>(deserializer: D) -> Result<Option<Scalar>, D::Error>
where
    D: Deserializer<'de>,
{
    let number = Option::<serde_json::Number>::deserialize(deserializer)?;
    number
        .map(|number| {
            parse_source_scalar(&number.to_string())
                .ok_or_else(|| serde::de::Error::custom("expected an exact finite JSON number"))
        })
        .transpose()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimal_input_is_retained_exactly() {
        let parsed = parse_source_scalar("0.1").expect("decimal should parse");
        let expected = Scalar::new(Rational::fraction(1, 10).unwrap());

        assert_eq!(parsed, expected);
    }

    #[test]
    fn primitive_float_spellings_are_not_internal_scalars() {
        assert!(parse_source_scalar("NaN").is_none());
        assert!(parse_source_scalar("inf").is_none());
    }

    #[test]
    fn json_number_deserialization_preserves_decimal_value() {
        #[derive(Deserialize)]
        struct Input {
            #[serde(default, deserialize_with = "deserialize_optional")]
            value: Option<Scalar>,
        }

        let input: Input = serde_json::from_str(r#"{"value": 0.1}"#).unwrap();
        assert_eq!(input.value, Some(scalar("0.1")));
    }

    #[test]
    fn scientific_input_is_retained_as_an_exact_power_of_ten() {
        assert_eq!(parse_source_scalar("1e-9"), Some(scalar("0.000000001")));
        assert_eq!(parse_source_scalar("2.5E3"), Some(scalar("2500")));
    }
}
