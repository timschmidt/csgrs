//! Feature-gated Real JSON/CBOR convenience methods and CBOR numeric adapters.
//!
//! Wire decoding delegates certificate/domain admission to RealWire. These
//! convenience methods preserve its Problem category but discard the additional
//! diagnostic detail; direct Serde deserialization retains that detail as text.
//! Encodings contain definitions, not live primitive/evaluation caches or abort
//! signals. Serialization does not clear the source caches or validate that a
//! later import will succeed. Shared graph pointers are serialized by value, so
//! the encoding does not preserve allocation identity or graph sharing.
//! Rational serialization may populate its lazy canonicalization cache.
//!
//! Import is proof work, not merely syntax parsing. Parser recursion limits and
//! individual proof precision limits do not bound total work, output expansion,
//! memory or elapsed time; these wrappers expose no resource/cancellation options.
//! The CBOR Value adapter accepts numeric shorthand that from_bytes does not.

use std::str::FromStr;

use crate::{
    Problem, Rational,
    real::{Real, RealWire},
};
use ciborium::{Value, de, ser};
use num::BigInt;
use num::bigint::Sign as BigIntSign;

/// Encode retained scalar definitions and admit decoded definitions through the scalar owner.
impl Real {
    /// Serialize this retained definition as JSON without clearing its live caches.
    ///
    /// Primitive/evaluation caches and abort signals are omitted, so replay may
    /// need to rediscover facts or recompute approximations. This is not a float
    /// projection or a validation pass. The in-memory serializer is unwrapped;
    /// an encoding error panics rather than returning Problem. Allocation and
    /// traversal have no caller-supplied bound.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap()
    }

    /// Parse one complete JSON Real wire definition and validate its certificates.
    /// Trailing whitespace is allowed; extra values or non-whitespace are errors.
    /// Bare numeric JSON is not the CBOR Value shorthand accepted by TryFrom.
    ///
    /// Invalid named certificates and missing opaque payloads return
    /// [`Problem::ParseError`]. An unproved named certificate or a payload whose
    /// equality is undecided returns [`Problem::UnknownCertificate`]. For a named
    /// class, a supplied payload is retained only after proving that relationship;
    /// an opaque class instead requires a payload whose operand domains are proved.
    /// Known domain and kernel-range violations, including bounded rational
    /// arithmetic arguments and symbolic domain contradictions, return
    /// [`Problem::ParseError`] before an outer scale or identity can hide them.
    /// Conditional algebraic identities are used only to reject invalid inputs,
    /// never to certify their values.
    ///
    /// Every partial operation in a computable payload must also have its
    /// operand domain proved, operands first: a nonzero reciprocal, a
    /// nonnegative root, an inverse-trigonometric or hyperbolic argument in
    /// range, a tangent or cotangent away from its poles, a quantile
    /// probability in (0, 1). Admission uses exact identities, structural signs,
    /// a bounded outward-rounded filter, or the certified sign query that native
    /// `Real` operations use, with precision floor -2048. Selected named-class
    /// positivity checks separately use floor -256. A refuted domain returns
    /// [`Problem::ParseError`];
    /// an undecided one returns [`Problem::UnknownCertificate`]. This bounds
    /// precision, not total work: an operand that is slow to approximate is
    /// approximated during import, as its first sign query would otherwise do.
    /// Syntax/shape failures map to ParseError; validation preserves its Problem
    /// category but not the internal detail string. Primitive caches start empty,
    /// abort signals are absent, and validation may establish fresh node facts.
    pub fn from_json(json: &str) -> Result<Self, Problem> {
        serde_json::from_str::<RealWire>(json)
            .map_err(|_| Problem::ParseError)?
            .validate()
            .map_err(|error| error.problem)
    }

    /// Serialize this definition into an owned CBOR byte vector.
    ///
    /// Like to_json, this omits accelerator state and abort signals without
    /// clearing the source. It allocates a growing vector and unwraps the writer
    /// result, so encoding errors panic. There is no byte/work budget or numeric
    /// shorthand conversion; a successful encoding is not an admission proof.
    ///
    /// Example:
    /// ```
    /// use hyperreal::Real;
    /// let x = Real::new(5.into());
    /// let bytes = x.to_bytes();
    /// let y = Real::from_bytes(&bytes).unwrap();
    /// assert_eq!(x, y);
    /// ```
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        ser::into_writer(self, &mut buf).unwrap();
        buf
    }

    /// Decode the first CBOR Real wire definition and validate it.
    ///
    /// Uses the same representation checks and remaining mathematical-domain
    /// limitations as [`Real::from_json`]. Unlike JSON parsing, this reader does
    /// not require the slice to end after that value; trailing bytes are ignored.
    /// It expects the Real wire shape, not the integer/float/text/pair shorthand
    /// of `TryFrom<&Value>`. Decode errors map to ParseError; validation retains
    /// its Problem category and drops the internal diagnostic detail.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, Problem> {
        de::from_reader::<RealWire, _>(bytes)
            .map_err(|_| Problem::ParseError)?
            .validate()
            .map_err(|error| error.problem)
    }
}

/// Interpret borrowed CBOR numeric shorthand or validate an encoded Real definition.
impl TryFrom<&Value> for Real {
    /// Preserve adapter errors and the scalar admission category without wire detail text.
    type Error = Problem;

    /// Dispatch in order: integer, float, text, two-element rational pair, then wire shape.
    /// Integer values must fit i64, even though CBOR supports wider integers;
    /// otherwise return BadInteger. Floats use exact binary64 ingress and reject
    /// nonfinite values. Text uses the exact Real parser.
    ///
    /// Every length-two array is treated as a numerator/denominator pair. Each
    /// element must be an i64-fitting integer or arbitrary-size integer text.
    /// Malformed integer text gives BadInteger, other element types ParseError,
    /// and a zero denominator DivideByZero. Normalize a negative denominator
    /// into the numerator sign. A malformed pair does not fall back to wire decoding.
    /// Other shapes deserialize to RealWire and run the same admission as from_json;
    /// errors retain Problem categories but lose the extra detail string.
    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        match value {
            Value::Integer(x) => {
                let val: i64 = (*x).try_into().map_err(|_| Problem::BadInteger)?;
                Ok(val.into())
            }
            Value::Float(x) => (*x).try_into(),
            Value::Text(x) => Real::from_str(x),
            Value::Array(x) if x.len() == 2 => {
                /// Decode one rational-pair component without floating conversion.
                /// Numeric CBOR integers are restricted to i64; text parses directly
                /// into BigInt. Report BadInteger for invalid/range-limited integers
                /// and ParseError for unsupported shapes, in caller operand order.
                fn as_big_int(value: &Value) -> Result<BigInt, Problem> {
                    match value {
                        Value::Integer(n) => {
                            let val: i64 = (*n).try_into().map_err(|_| Problem::BadInteger)?;
                            Ok(BigInt::from(val))
                        }
                        Value::Text(x) => {
                            let val: BigInt = x.parse().map_err(|_| Problem::BadInteger)?;
                            Ok(val)
                        }
                        _ => Err(Problem::ParseError),
                    }
                }
                // If we receive an array of length 2, we attempt to interpret it as the numerator and denominator of a Rational.
                let mut numerator = as_big_int(&x[0])?;
                let denominator = as_big_int(&x[1])?;
                let (denominator_sign, denominator_magnitude) = denominator.into_parts();
                if denominator_sign == BigIntSign::NoSign {
                    return Err(Problem::DivideByZero);
                }
                if denominator_sign == BigIntSign::Minus {
                    numerator = -numerator;
                }
                Ok(Real::new(Rational::from_bigint_fraction(
                    numerator,
                    denominator_magnitude,
                )?))
            }
            _ => {
                // In this branch, we should be whatever type ciborium decided to use for `Real`.
                // Try a fallible conversion. If it fails, we're done.
                let wire: RealWire = value.deserialized().map_err(|_| Problem::ParseError)?;
                wire.validate().map_err(|error| error.problem)
            }
        }
    }
}

/// Consume a CBOR value using exactly the borrowed adapter's dispatch and admission.
impl TryFrom<Value> for Real {
    /// The borrowed conversion's typed scalar/shape error, without additional wire detail.
    type Error = Problem;

    /// Borrow the owned input for conversion, then drop it; this does not move a graph out of Value.
    fn try_from(value: Value) -> Result<Self, Self::Error> {
        Self::try_from(&value)
    }
}

/// Wire-format, numeric-adapter and selected replay regression witnesses.
/// Tolerance comparisons below do not establish exact equality of arbitrary expressions.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::rational::Rational;
    use std::cmp::Ordering;

    #[test]
    /// Freeze the exact JSON encoding of +/-5/2 and require rational equality after decoding.
    fn rational_json_serde() {
        let x: Rational = "5/2".parse().unwrap();
        let json = serde_json::to_string(&x).unwrap();
        assert_eq!(json, "{\"sign\":1,\"numerator\":[5],\"denominator\":[2]}");
        let y: Rational = serde_json::from_str(&json).unwrap();
        assert_eq!(x, y);
        let x: Rational = "-5/2".parse().unwrap();
        let json = serde_json::to_string(&x).unwrap();
        assert_eq!(json, "{\"sign\":-1,\"numerator\":[5],\"denominator\":[2]}");
        let y: Rational = serde_json::from_str(&json).unwrap();
        assert_eq!(x, y);
    }

    #[test]
    /// Replay rational five and pi through raw Computable JSON, then compare at binary tolerance 10.
    /// This coarse comparison is not an exact symbolic equality certificate or Real domain-admission test.
    fn computable_json_serde() {
        use crate::computable::Computable;
        let x = Computable::rational(Rational::new(5));
        let json = serde_json::to_string(&x).unwrap();
        let y: Computable = serde_json::from_str(&json).unwrap();
        assert_eq!(x.compare_absolute(&y, 10), (std::cmp::Ordering::Equal));
        let x = Computable::pi();
        let json = serde_json::to_string(&x).unwrap();
        let y: Computable = serde_json::from_str(&json).unwrap();
        assert_eq!(x.compare_absolute(&y, 10), (std::cmp::Ordering::Equal));
    }

    #[test]
    /// Require rational five equality after direct Serde JSON replay; compare 5^e only at tolerance 10.
    fn real_json_serde() {
        let x = Real::new(Rational::new(5));
        let json = serde_json::to_string(&x).unwrap();
        let y: Real = serde_json::from_str(&json).unwrap();
        assert_eq!(x, y);
        // let x = (Real::new(Rational::new(5)) * Real::pi()).pow(Real::e()).unwrap();
        let x = (Real::new(Rational::new(5))).pow(Real::e()).unwrap();
        let json = serde_json::to_string(&x).unwrap();
        let y: Real = serde_json::from_str(&json).unwrap();
        assert_eq!(
            x.fold().compare_absolute(&y.fold(), 10),
            std::cmp::Ordering::Equal
        );
    }

    #[test]
    /// Require rational five equality after direct Serde CBOR replay; compare 5^e only at tolerance 10.
    fn real_cbor_serde() {
        let x = Real::new(Rational::new(5));
        let mut buf = Vec::new();
        ser::into_writer(&x, &mut buf).unwrap();
        let y: Real = de::from_reader(buf.as_slice()).unwrap();
        assert_eq!(x, y);
        let x = (Real::new(Rational::new(5))).pow(Real::e()).unwrap();
        let mut buf = Vec::new();
        ser::into_writer(&x, &mut buf).unwrap();
        let y: Real = de::from_reader(buf.as_slice()).unwrap();
        assert_eq!(
            x.fold().compare_absolute(&y.fold(), 10),
            std::cmp::Ordering::Equal
        );
    }

    #[test]
    /// Replay six selected inverse-function results through both convenience formats.
    /// Require folded comparisons to return Equal at binary tolerance -80, not a universal exact-equality proof.
    fn inverse_function_results_round_trip_serde() {
        let values = [
            Real::new(Rational::fraction(7, 10).unwrap())
                .asin()
                .unwrap(),
            Real::new(Rational::fraction(7, 10).unwrap())
                .acos()
                .unwrap(),
            Real::new(Rational::new(2)).sqrt().unwrap().atan().unwrap(),
            Real::new(Rational::new(2)).sqrt().unwrap().asinh().unwrap(),
            Real::new(Rational::new(2)).sqrt().unwrap().acosh().unwrap(),
            Real::new(Rational::fraction(-1, 2).unwrap())
                .atanh()
                .unwrap(),
        ];

        for value in values {
            let json = value.to_json();
            let from_json = Real::from_json(&json).unwrap();
            assert_eq!(
                value
                    .fold_ref()
                    .compare_absolute(&from_json.fold_ref(), -80),
                Ordering::Equal
            );

            let bytes = value.to_bytes();
            let from_bytes = Real::from_bytes(&bytes).unwrap();
            assert_eq!(
                value
                    .fold_ref()
                    .compare_absolute(&from_bytes.fold_ref(), -80),
                Ordering::Equal
            );
        }
    }

    #[test]
    /// Require exact rational five equality through both public convenience encode/decode pairs.
    fn test_round_trip_functions() {
        let x = Real::new(Rational::new(5));
        let json = x.to_json();
        let y = Real::from_json(&json).unwrap();
        assert_eq!(x, y);

        let y = x.to_bytes();
        let z = Real::from_bytes(&y).unwrap();
        assert_eq!(x, z);
    }

    #[test]
    /// Require seven paths for exact five: wire, integer, text, float, numeric/mixed pairs and borrowed integer.
    /// These cases compare scalar values, not original CBOR byte sequences.
    fn cbor_value_try_into() {
        use ciborium::value::Value;

        // CBOR Real
        let x = Real::new(Rational::new(5));
        let mut buf = Vec::new();
        ser::into_writer(&x, &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // CBOR Integer
        let mut buf = Vec::new();
        ser::into_writer(&5, &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // CBOR String
        let mut buf = Vec::new();
        ser::into_writer(&"5", &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // CBOR Float
        let mut buf = Vec::new();
        ser::into_writer(&5.0, &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // CBOR Pair of Integers
        let mut buf = Vec::new();
        ser::into_writer(&(10, 2), &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // CBOR Pair of Integers and Strings
        let mut buf = Vec::new();
        ser::into_writer(&(10, "2"), &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = value.try_into().unwrap();
        assert_eq!(x, y);

        // Reference to CBOR Integer
        let mut buf = Vec::new();
        ser::into_writer(&5, &mut buf).unwrap();
        let value: Value = de::from_reader(buf.as_slice()).unwrap();
        let y: Real = (&value).try_into().unwrap();
        assert_eq!(x, y);
    }

    #[test]
    /// Require the numeric pair [1, 0] to return DivideByZero rather than a wire-shape error.
    fn cbor_rational_pair_rejects_zero_denominator() {
        let value = Value::Array(vec![Value::Integer(1.into()), Value::Integer(0.into())]);
        assert_eq!(Real::try_from(&value), Err(Problem::DivideByZero));
    }

    #[test]
    /// Require the numeric pair [1, -2] to normalize to the exact rational -1/2.
    fn cbor_rational_pair_normalizes_negative_denominator() {
        let value = Value::Array(vec![Value::Integer(1.into()), Value::Integer((-2).into())]);
        assert_eq!(
            Real::try_from(&value).unwrap(),
            Real::new(Rational::fraction(-1, 2).unwrap())
        );
    }
}
