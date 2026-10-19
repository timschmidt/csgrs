use super::*;
use std::sync::{Arc, atomic::AtomicBool};

#[test]
fn inverse_legendre_abort_and_coarse_precision() {
    let signal = Some(Arc::new(AtomicBool::new(true)));
    for second in [false, true] {
        let arguments = [Computable::pi(), Computable::one()];
        assert_eq!(
            inverse_legendre(&signal, &arguments, second, -128),
            BigInt::zero()
        );
        for precision in [4, 8, i32::MAX] {
            assert!(inverse_legendre(&None, &arguments, second, precision).abs() <= BigInt::one());
        }
    }
}

#[test]
fn serialized_zero_and_circular_inverses_are_in_domain() {
    for second in [false, true] {
        for integer in [-7, 0, 11] {
            let value = Computable::rational(Rational::new(integer));
            let args = [value.clone(), Computable::zero()];
            for precision in [-32, -128] {
                let actual = inverse_legendre(&None, &args, second, precision);
                assert!((actual - value.approx(precision)).abs() <= BigInt::one());
            }
        }
    }
}
