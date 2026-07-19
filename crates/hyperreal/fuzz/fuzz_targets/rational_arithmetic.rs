//! Fuzz exact Rational construction, ownership variants, and ring identities.

#![no_main]

use arbitrary::Arbitrary;
use hyperreal::Rational;
use libfuzzer_sys::fuzz_target;
use num::BigInt;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawRational {
    numerator: i32,
    denominator: u16,
}

impl RawRational {
    fn value(self) -> Rational {
        Rational::fraction(i64::from(self.numerator), u64::from(self.denominator) + 1)
            .expect("the generated denominator is positive")
    }
}

#[derive(Debug, Arbitrary)]
struct Input {
    a: RawRational,
    b: RawRational,
    c: RawRational,
    exponent: i8,
}

fuzz_target!(|input: Input| {
    let a = input.a.value();
    let b = input.b.value();
    let c = input.c.value();

    let add = &a + &b;
    let sub = &a - &b;
    let mul = &a * &b;

    assert_eq!(add, a.clone() + b.clone());
    assert_eq!(sub, a.clone() - b.clone());
    assert_eq!(add, &a + &b);
    assert_eq!(add, &b + &a);
    assert_eq!(sub, &a - &b);
    assert_eq!(mul, a.clone() * b.clone());
    assert_eq!(mul, &b * &a);
    assert_eq!(&a * &c, &c * &a);
    assert_eq!(&a + &Rational::zero(), a);
    assert_eq!(&a * &Rational::one(), a);
    assert_eq!(&a * (&b + &c), (&a * &b) + (&a * &c));
    let same_value = a.clone();
    assert_eq!(&a - &same_value, Rational::zero());

    if !b.is_zero() {
        let quotient = &a / &b;
        assert_eq!(&quotient * &b, a);
        assert_eq!(
            b.clone().inverse().expect("nonzero inverse") * &b,
            Rational::one()
        );
    }

    let exponent = BigInt::from(i32::from(input.exponent.clamp(-12, 12)));
    if !a.is_zero() || exponent >= BigInt::from(0) {
        let powered = a
            .clone()
            .powi(exponent)
            .expect("valid bounded rational power");
        assert!(powered.denominator() > &0_u8.into());
    }

    assert_eq!(a.trunc() + a.fract(), a);
    assert_eq!(a.is_negative(), a.sign() == num::bigint::Sign::Minus);
    assert_eq!(a.is_positive(), !a.is_zero() && !a.is_negative());
    let _ = a.dyadic_to_f64_exact();
    let _ = a.to_string();
});
