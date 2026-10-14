use super::*;

#[test]
fn borrowed_integer_preserves_full_width_sign_and_magnitude() {
    let check = |input: BigInt| {
        let value = Rational::from_bigint_ref(&input);
        assert_eq!(value.sign, input.sign());
        assert_eq!(&value.numerator, input.magnitude());
        assert!(value.denominator.is_one());
    };
    for value in -66_i64..=66 {
        check(BigInt::from(value));
    }
    for bits in [63_usize, 64, 65, 127, 128, 129, 255, 256, 1023, 1024, 4096] {
        for offset in [-1_i8, 0, 1] {
            let value = (BigInt::one() << bits) + offset;
            check(value.clone());
            check(-value);
        }
    }
}

#[test]
fn borrowed_integer_reuses_only_bounded_canonical_storage() {
    for value in -64_i64..=64 {
        let input = BigInt::from(value);
        let borrowed = Rational::from_bigint_ref(&input);
        let canonical = Rational::new(value);
        assert!(Arc::ptr_eq(&borrowed.0, &canonical.0));
    }
    for value in [-66, -65, 65, 66, i64::MIN, i64::MAX] {
        let input = BigInt::from(value);
        let first = Rational::from_bigint_ref(&input);
        let second = Rational::from_bigint_ref(&input);
        assert_eq!(first, second);
        assert!(!Arc::ptr_eq(&first.0, &second.0));
    }
}
