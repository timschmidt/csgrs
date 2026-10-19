use super::*;
use std::sync::{Arc, atomic::AtomicBool};

#[test]
fn jacobi_kernel_handles_direct_payloads_and_cancellation() {
    let signal = Some(Arc::new(AtomicBool::new(true)));
    for kind in 0..3 {
        assert_eq!(
            jacobi(&signal, &[Computable::pi(), Computable::one()], kind, -128),
            BigInt::zero()
        );
        for log in [-10, 0, 10] {
            let args = [
                Computable::zero(),
                Computable::rational(Rational::from(log)),
            ];
            for p in [-128, 8] {
                let expected = if kind == 0 {
                    Computable::zero()
                } else {
                    Computable::one()
                };
                assert!(
                    (jacobi(&None, &args, kind, p) - expected.approx(p)).abs() <= BigInt::one()
                );
            }
        }
    }
}
