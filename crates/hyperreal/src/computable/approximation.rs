use crate::Computable;
use crate::Rational;
use crate::computable::{Precision, Signal, scale, shift, should_stop, signed};
use num::bigint::Sign;
use num::{BigInt, BigUint, Signed, ToPrimitive};
use num::{One, Zero};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::ops::Deref;
use std::sync::LazyLock;

include!("approximation/representation.rs");
include!("approximation/dispatch.rs");
include!("approximation/constants.rs");
include!("approximation/arithmetic_kernels.rs");
include!("approximation/exp_sqrt.rs");
include!("approximation/trig.rs");
include!("approximation/logarithms.rs");
include!("approximation/inverse_trig.rs");
include!("approximation/inverse_hyperbolic.rs");
include!("approximation/statistics.rs");

#[cfg(test)]
mod chudnovsky_pi_tests {
    use super::*;
    use rug::{Float, float::Constant, float::Round};
    use std::sync::{Arc, atomic::AtomicBool};

    #[test]
    fn chudnovsky_pi_matches_mpfr_from_one_bit_through_many_digits() {
        for bits in [
            1_u32, 2, 3, 4, 7, 8, 15, 16, 31, 32, 43, 44, 63, 64, 65, 96, 127, 128, 255, 256, 511,
            512, 1_023, 1_024, 2_047, 2_048, 4_095, 4_096, 4_097, 16_384, 65_536, 120_700,
        ] {
            let actual =
                pi_chudnovsky(&None, -(bits as Precision)).expect("supported Chudnovsky precision");

            let mut oracle = Float::with_val(bits + 192, Constant::Pi);
            oracle <<= bits;
            let expected = oracle
                .to_integer_round(Round::Nearest)
                .expect("finite pi oracle")
                .0
                .to_string()
                .parse::<BigInt>()
                .expect("MPFR integer parses as BigInt");
            let error = (&actual - expected).abs();
            assert!(
                error <= BigInt::one(),
                "{bits}-bit pi differs from MPFR by {error} ulps"
            );
            assert_eq!(pi(&None, -(bits as Precision)), actual);
        }
    }

    #[test]
    fn high_precision_pi_honors_a_preexisting_abort() {
        let signal = Some(Arc::new(AtomicBool::new(true)));
        assert!(pi_chudnovsky(&signal, -4_096).is_none());
        assert_eq!(pi(&signal, -4_096), BigInt::zero());
    }
}
