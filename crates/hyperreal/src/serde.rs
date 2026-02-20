use crate::real::Real;
use ciborium::{de, ser};
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
    /// use realistic::Real;
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
}
