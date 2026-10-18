#![cfg(feature = "serde")]

use hyperlimit::{Point2, PredicatePolicy, Sign, classify_real_sign, orient2};
use hyperreal::Real;

#[test]
fn lazy_scalar_certificates_reach_strict_geometric_predicates() {
    for value in [
        Real::pi(),
        Real::from(2).sqrt().unwrap(),
        Real::from(3).ln().unwrap(),
    ] {
        let mut wire = serde_json::to_value(value).unwrap();
        wire.as_object_mut().unwrap().remove("computable");
        let value = Real::from_json(&wire.to_string()).unwrap();
        assert_eq!(
            classify_real_sign(&value, PredicatePolicy::STRICT).value(),
            Some(Sign::Positive)
        );
        // The determinant of (1, 0) and (0, value) equals value > 0.
        let a = Point2::new(Real::zero(), Real::zero());
        let b = Point2::new(Real::one(), Real::zero());
        let c = Point2::new(Real::zero(), value);
        assert_eq!(
            orient2(&a, &b, &c, PredicatePolicy::STRICT).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            orient2(&a, &c, &b, PredicatePolicy::STRICT).value(),
            Some(Sign::Negative)
        );
    }
}
