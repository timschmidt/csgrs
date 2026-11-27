//! Retained expression recipes and integer approximation kernels for Computable.
//!
//! Precision p denotes a grid spacing of `2^p`; completed approximations are
//! integers on that grid, while the exact expression remains authoritative.
//! Included sources share this namespace. Constructors and the outer evaluation
//! machinery own simplification, caching and interrupted-result publication;
//! individual kernels document domains, local range checks and resource limits.
//! A returned BigInt alone does not certify successful completion after a stop.

use crate::Computable;
use crate::Rational;
use crate::computable::{Precision, Signal, scale, scale_at_precision, shift, should_stop, signed};
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
include!("approximation/inverse_isometric_latitude.rs");
include!("approximation/certified_interval.rs");
include!("approximation/elliptic_integrals.rs");
include!("approximation/legendre_integrals.rs");
include!("approximation/inverse_legendre.rs");
include!("approximation/jacobi.rs");
include!("approximation/statistics.rs");

#[cfg(test)]
#[path = "approximation/e_plan_tests.rs"]
/// Factorial-planner, directed-enclosure and shared-e recovery/replay regressions.
mod e_plan_tests;

#[cfg(test)]
#[path = "approximation/precision_tests.rs"]
/// Checked sample-exponent boundary regressions, distinct from whole-kernel precision support.
mod precision_tests;

#[cfg(test)]
#[path = "approximation/exp_tail_tests.rs"]
/// Direct exponential-series and coarse negative-tail regression witnesses.
mod exp_tail_tests;

#[cfg(test)]
#[path = "approximation/elliptic_tests.rs"]
/// Carlson-kernel quotient, coarse-request and mean-enclosure regression witnesses.
mod elliptic_tests;

#[cfg(test)]
#[path = "approximation/legendre_tests.rs"]
/// Legendre-kernel abort, zero and circular-payload regression witnesses.
mod legendre_tests;

#[cfg(test)]
#[path = "approximation/inverse_legendre_tests.rs"]
/// Inverse-Legendre coarse, abort and serialized-payload regression witnesses.
mod inverse_legendre_tests;

#[cfg(test)]
#[path = "approximation/jacobi_tests.rs"]
/// Direct Jacobi component payload and cancellation regression witnesses.
mod jacobi_tests;

#[cfg(test)]
/// Direct Chudnovsky precision samples, dispatch agreement and preexisting-stop regression witnesses.
mod chudnovsky_pi_tests {
    use super::*;
    use rug::{Float, float::Constant, float::Round};
    use std::sync::{Arc, atomic::AtomicBool};

    #[test]
    /// Compare 32 direct Chudnovsky requests from one through 120,700 fractional bits with MPFR.
    /// Use a nearest-rounded MPFR integer oracle at 192 extra bits and permit one
    /// integer unit of difference; this is not a strict directed enclosure test.
    /// Also require the pi dispatcher to return the same integer. Register MPFR
    /// thread cleanup; neither comparison is an exhaustive precision proof.
    fn chudnovsky_pi_matches_mpfr_from_one_bit_through_many_digits() {
        crate::mpfr_test::cleanup_on_thread_exit();
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
    /// Require direct Chudnovsky to decline and pi dispatch to return its zero abort placeholder.
    /// The stop is already asserted before the -4096 request. This does not test
    /// mid-computation cancellation latency, cache publication or recovery.
    fn high_precision_pi_honors_a_preexisting_abort() {
        let signal = Some(Arc::new(AtomicBool::new(true)));
        assert!(pi_chudnovsky(&signal, -4_096).is_none());
        assert_eq!(pi(&signal, -4_096), BigInt::zero());
    }
}
