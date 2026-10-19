use super::*;
use std::sync::{Arc, atomic::AtomicBool};

#[test]
fn legendre_kernel_checks_abort_before_reducing_amplitude() {
    let signal = Some(Arc::new(AtomicBool::new(true)));
    let arguments = [Computable::pi(), Computable::one()];
    for second in [false, true] {
        assert_eq!(
            elliptic_legendre(&signal, &arguments, second, -128),
            BigInt::zero()
        );
    }
}

#[test]
fn legendre_serializable_payloads_cover_zero_and_circular_integrals() {
    for second in [false, true] {
        for amplitude in [0, -7, 11] {
            let value = Computable::rational(Rational::new(amplitude));
            let arguments = [value.clone(), Computable::zero()];
            for precision in [8, -32, -128] {
                let actual = elliptic_legendre(&None, &arguments, second, precision);
                let expected = value.approx(precision);
                assert!((actual - expected).abs() <= BigInt::one());
            }
        }
    }
}
