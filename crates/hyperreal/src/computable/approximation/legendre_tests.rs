use super::*;
use std::sync::{Arc, atomic::AtomicBool};

#[test]
/// Require the zero abort placeholder for both direct kernels with a flag already asserted.
/// Use amplitude pi, log complement one and precision -128. This checks entry
/// cancellation, not active-solve latency, internal caches or a phase trace count.
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
/// Exercise 18 direct-kernel circular requests without constructor simplification.
/// For both kinds, use amplitudes 0,-7,11 and precisions 8,-32,-128. Log complement
/// zero means m=0, so compare with the amplitude's approximation within one integer
/// unit. These are payload-shaped arguments, not an actual serialization round trip;
/// the comparison is shared-implementation evidence rather than an independent oracle.
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
