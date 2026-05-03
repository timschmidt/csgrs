use std::str::FromStr;

use crate::{Problem, Rational, real::Real};
use ciborium::{Value, de, ser};
use num::BigInt;
use serde_json;

impl Real {
    /// Serializes the Real to a JSON string.
    ///
    /// Note: serialization deletes the cache, so serializing and deserializing a Real may come with a performance penalty.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap()
    }

    /// Deserializes a Real from a JSON string.
    pub fn from_json(json: &str) -> Self {
        serde_json::from_str(json).unwrap()
    }

    /// Serializes the Real to a CBOR byte vector.
    ///
    /// Note: serialization deletes the cache, so serializing and deserializing a Real may come with a performance penalty.
    ///
    /// Example:
    /// ```
    /// use hyperreal::Real;
    /// let x = Real::new(5.into());
    /// let bytes = x.to_bytes();
    /// let y = Real::from_bytes(&bytes);
    /// assert_eq!(x, y);
    /// ```
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        ser::into_writer(self, &mut buf).unwrap();
        buf
    }

    /// Deserializes a Real from a CBOR byte slice.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        de::from_reader(bytes).unwrap()
    }
}

impl TryFrom<&Value> for Real {
    type Error = Problem;

    /// Attempt to convert from a CBOR Value. We accept floats, integers, and strings, and serialized Reals.
    fn try_from(value: &Value) -> Result<Self, Self::Error> {
        match value {
            Value::Integer(x) => {
                let val: i64 = (*x).try_into().map_err(|_| Problem::BadInteger)?;
                Ok(val.into())
            }
            Value::Float(x) => (*x).try_into(),
            Value::Text(x) => Real::from_str(x),
            Value::Array(x) if x.len() == 2 => {
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
                let numerator = as_big_int(&x[0])?;
                let denominator = as_big_int(&x[1])?;
                Ok(Real::new(
                    Rational::from_bigint(numerator) / Rational::from_bigint(denominator),
                ))
            }
            _ => {
                // In this branch, we should be whatever type ciborium decided to use for `Real`.
                // Try a fallible conversion. If it fails, we're done.
                let ret: Real = value.deserialized().map_err(|_| Problem::ParseError)?;
                Ok(ret)
            }
        }
    }
}

impl TryFrom<Value> for Real {
    type Error = Problem;

    fn try_from(value: Value) -> Result<Self, Self::Error> {
        Self::try_from(&value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rational::Rational;

    #[test]
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
    fn test_round_trip_functions() {
        let x = Real::new(Rational::new(5));
        let json = x.to_json();
        let y = Real::from_json(&json);
        assert_eq!(x, y);

        let y = x.to_bytes();
        let z = Real::from_bytes(&y);
        assert_eq!(x, z);
    }

    #[test]
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
}
