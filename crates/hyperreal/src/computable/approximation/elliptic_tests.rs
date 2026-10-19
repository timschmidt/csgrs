use super::*;

#[test]
fn directed_quotients_and_coarse_requests_keep_their_bounds() {
    for numerator in 0..20_u32 {
        for denominator in 1..12_u32 {
            for shift in -8..8 {
                let [lo, hi] = elliptic_quotient(numerator.into(), denominator.into(), shift);
                let exact = Rational::fraction(numerator.into(), denominator.into()).unwrap()
                    * Rational::new(2).powi(shift.into()).unwrap();
                assert!(Rational::from_bigint(BigInt::from(lo)) <= exact);
                assert!(exact <= Rational::from_bigint(BigInt::from(hi)));
            }
        }
    }
    let arguments = [Computable::one(), Computable::zero(), Computable::zero()];
    for second in [false, true] {
        assert_eq!(
            elliptic_carlson(&None, &arguments, second, i32::MAX),
            BigInt::zero()
        );
        let signal = Some(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
            true,
        )));
        assert_eq!(
            elliptic_carlson(&signal, &arguments, second, -512),
            BigInt::zero()
        );
    }
}
