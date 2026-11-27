use hyperreal::{Problem, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/mpfr.rs"]
/// Register thread-local cleanup for development-only MPFR computations.
mod mpfr;
#[path = "support/legendre_oracle.rs"]
/// Supply independent binomial-series and AGM/square-sum Legendre references.
mod oracle;
#[path = "support/representations.rs"]
/// Supply the finite shared inventory of 22 positive scalar construction recipes.
mod representations;

/// Require an eight-guard-bit enclosure of actual-expected inside +/-2^(-bits).
/// Compare exact rational endpoints for absolute-error qualification; this relies
/// on the enclosure API and does not promise relative accuracy or identical rounding.
fn close(actual: &Real, expected: &Real, bits: i32) {
    let [lo, hi] = (actual - expected)
        .certified_rational_interval(-bits - 8)
        .unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(
        lo >= -tolerance.clone() && hi <= tolerance,
        "error [{lo}, {hi}]"
    );
}

#[test]
/// Require exact rational-normal-form zero for 48 signed quarter-period identities.
/// Use three parameters, eight integer counts including 1e30, and both integral
/// kinds. This checks retained endpoint relations without accepting a merely
/// small sampled difference as symbolic cancellation.
fn exact_quarter_amplitudes_retain_complete_integral_relations() {
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for text in ["0.5", "-3", "0"] {
        let m: Real = text.parse().unwrap();
        for count in [-7, -2, -1, 0, 1, 2, 7]
            .map(Real::from)
            .into_iter()
            .chain(["1e30".parse().unwrap()])
        {
            let amplitude = &count * &half_pi;
            for second in [false, true] {
                let (value, complete) = if second {
                    (
                        amplitude.clone().elliptic_e(&m).unwrap(),
                        Real::elliptic_e_complete(&m).unwrap(),
                    )
                } else {
                    (
                        amplitude.clone().elliptic_f(&m).unwrap(),
                        Real::elliptic_k(&m).unwrap(),
                    )
                };
                assert_eq!(
                    (value - &count * complete).exact_rational_normal_form(),
                    Some(Rational::zero()),
                    "{text}, count {count}, second {second}"
                );
            }
        }
    }
}

#[test]
/// Check signal retention through both endpoint shortcuts and distinguish nearby amplitudes.
/// Attach a clear signal to pi/2, construct each integral, then assert the signal
/// before evaluation. Require cancellation, recovery and correct signs for offsets
/// +/-1e-60 at m=1/2. This tests a post-construction pre-evaluation stop, not
/// interruption during arithmetic or tolerance-based endpoint identification.
fn quarter_endpoint_reduction_retains_cancellation_and_distinct_offsets() {
    let parameter: Real = "0.5".parse().unwrap();
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    for second in [false, true] {
        let evaluate = |argument: Real| {
            if second {
                argument.elliptic_e(&parameter)
            } else {
                argument.elliptic_f(&parameter)
            }
        };
        let signal = Arc::new(AtomicBool::new(false));
        let mut argument = quarter.clone();
        argument.abort(signal.clone());
        let value = evaluate(argument).unwrap();
        signal.store(true, Ordering::Relaxed);
        assert!(value.certified_rational_interval(-128).is_none());
        signal.store(false, Ordering::Relaxed);
        close(&value, &evaluate(quarter.clone()).unwrap(), 256);
        let delta: Real = "1e-60".parse().unwrap();
        for direction in [-1, 1] {
            let nearby = evaluate(&quarter + Real::from(direction) * &delta).unwrap();
            // The integral is strictly increasing for m<1. These tiny
            // displacements must not be treated as the symbolic endpoint.
            assert_eq!(
                (nearby - &value).certified_sign_until(-512).sign(),
                Some(if direction < 0 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                }),
            );
        }
    }
}

#[test]
/// Check complete/incomplete circular identities and reject m=1,2 before zero-amplitude shortcuts.
/// Preserve amplitudes zero, e and 1e320 exactly when m=0; both complete circular
/// values equal pi/2. Require typed NotANumber even for the finite second-kind
/// endpoint, which remains outside the common API domain.
fn circular_identities_and_parameter_domain() {
    let zero = Real::zero();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    assert_eq!(Real::elliptic_k(&zero).unwrap(), half_pi);
    assert_eq!(Real::elliptic_e_complete(&zero).unwrap(), half_pi);
    for amplitude in [zero.clone(), Real::e(), "1e320".parse().unwrap()] {
        assert_eq!(amplitude.clone().elliptic_f(&zero).unwrap(), amplitude);
        assert_eq!(amplitude.clone().elliptic_e(&zero).unwrap(), amplitude);
    }
    for parameter in [Real::one(), Real::from(2)] {
        assert_eq!(Real::elliptic_k(&parameter), Err(Problem::NotANumber));
        assert_eq!(
            Real::elliptic_e_complete(&parameter),
            Err(Problem::NotANumber)
        );
        assert_eq!(
            zero.clone().elliptic_f(&parameter),
            Err(Problem::NotANumber)
        );
        assert_eq!(
            zero.clone().elliptic_e(&parameter),
            Err(Problem::NotANumber)
        );
    }
}

#[test]
/// Compare both integrals at 512-bit accuracy against independent series for 44 parameter/amplitude pairs.
/// For each of 22 recipes verify its public structural kind, map v to
/// s=v/(4*(v+1)), and use amplitude pi+s with parameters +/-s. This supplies
/// the oracle's small-parameter domain; it is not exhaustive graph/value coverage.
/// Register MPFR thread cleanup before reference computation.
fn independent_integrand_series_all_scalar_families() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        eprintln!("Legendre independent oracle: {}", case.certificate);
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        let amplitude = Real::pi() + &small;
        for parameter in [small.clone(), -&small] {
            let [f, e] = oracle::series(&amplitude, &parameter).map(oracle::native);
            close(&amplitude.clone().elliptic_f(&parameter).unwrap(), &f, 512);
            close(&amplitude.clone().elliptic_e(&parameter).unwrap(), &e, 512);
        }
    }
}

#[test]
/// Compare six complete-integral complement fixtures with AGM and their incomplete endpoint values.
/// Use c=1/4,1/2,1,4,1e-160,1e160 with m=1-c and 192-bit absolute comparisons.
/// Normalize E by sqrt(c) only in the largest-complement case. AGM supplies an
/// independent reference; endpoint agreement also checks the retained complete
/// relation. Register MPFR cleanup, without claiming a universal oracle error bound.
fn complete_integrals_match_independent_agm_and_incomplete_endpoints() {
    mpfr::cleanup_on_thread_exit();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for text in ["0.25", "0.5", "1", "4", "1e-160", "1e160"] {
        let c: Real = text.parse().unwrap();
        let m = Real::one() - &c;
        let [k, e] = oracle::complete(&c).map(oracle::native);
        let normalization = if text == "1e160" {
            c.clone().sqrt().unwrap()
        } else {
            Real::one()
        };
        close(&Real::elliptic_k(&m).unwrap(), &k, 192);
        close(
            &(Real::elliptic_e_complete(&m).unwrap() / &normalization).unwrap(),
            &(e / &normalization).unwrap(),
            192,
        );
        close(
            &half_pi.clone().elliptic_f(&m).unwrap(),
            &Real::elliptic_k(&m).unwrap(),
            192,
        );
        close(
            &(half_pi.clone().elliptic_e(&m).unwrap() / &normalization).unwrap(),
            &(Real::elliptic_e_complete(&m).unwrap() / normalization).unwrap(),
            192,
        );
    }
}

#[test]
/// Check the m=-3 principal values at phi=5/16 using complementary m=3/4 series.
/// Subtract the complementary remainder from its pi/2 endpoint, scaling F by
/// one half and E by two. Demand 512-bit absolute agreement; this reference
/// uses integrand series rather than Carlson duplication. Register MPFR cleanup.
fn prolate_principal_values_match_complementary_integrand_series() {
    mpfr::cleanup_on_thread_exit();
    let phi: Real = "0.3125".parse().unwrap();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let complementary_parameter: Real = "0.75".parse().unwrap();
    let endpoint = oracle::series(&half_pi, &complementary_parameter);
    let remainder = oracle::series(&(&half_pi - &phi), &complementary_parameter);
    // Substitution t=pi/2-u, c=4: F scales by 1/2 and E by 2.
    let expected_f = oracle::native((endpoint[0].clone() - &remainder[0]) / 2);
    let expected_e = oracle::native((endpoint[1].clone() - &remainder[1]) * 2);
    close(
        &phi.clone().elliptic_f(&Real::from(-3)).unwrap(),
        &expected_f,
        512,
    );
    close(&phi.elliptic_e(&Real::from(-3)).unwrap(), &expected_e, 512);
}

#[test]
/// Exercise four nonzero offsets around pi/2 with parameters +/-3/4 and offsets +/-5/16.
/// Compare both integral kinds at 512 bits to independent direct-amplitude series,
/// covering odd-quarter complement transport on either side. Register MPFR cleanup.
fn nonzero_odd_quarter_reductions_match_independent_integrand_series() {
    mpfr::cleanup_on_thread_exit();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for parameter in ["0.75", "-0.75"] {
        let parameter: Real = parameter.parse().unwrap();
        for offset in ["0.3125", "-0.3125"] {
            let amplitude = &half_pi + offset.parse::<Real>().unwrap();
            let [f, e] = oracle::series(&amplitude, &parameter).map(oracle::native);
            close(&amplitude.clone().elliptic_f(&parameter).unwrap(), &f, 512);
            close(&amplitude.elliptic_e(&parameter).unwrap(), &e, 512);
        }
    }
}

#[test]
/// Check odd symmetry, signed quarter endpoints and full-period shifts for two parameters.
/// Use m=3/4,-3 and counts -7 through 7 for both kinds at 192 bits. Quarter
/// expectations use n complete integrals; pi shifts add 2n complete integrals to
/// the original offset integral, preserving the unwrapped amplitude.
fn odd_symmetry_and_periods_preserve_unwrapped_amplitudes() {
    for parameter in ["0.75", "-3"] {
        let m: Real = parameter.parse().unwrap();
        let half_pi = (Real::pi() / Real::from(2)).unwrap();
        let x: Real = "0.3125".parse().unwrap();
        for second in [false, true] {
            let integral = |a: Real| {
                if second {
                    a.elliptic_e(&m)
                } else {
                    a.elliptic_f(&m)
                }
                .unwrap()
            };
            let complete = if second {
                Real::elliptic_e_complete(&m)
            } else {
                Real::elliptic_k(&m)
            }
            .unwrap();
            close(&integral(-&x), &(-integral(x.clone())), 192);
            for n in -7..=7 {
                close(
                    &integral(Real::from(n) * &half_pi),
                    &(Real::from(n) * &complete),
                    192,
                );
                close(
                    &integral(Real::from(n) * Real::pi() + &x),
                    &(Real::from(2 * n) * &complete + integral(x.clone())),
                    192,
                );
            }
        }
    }
}

#[test]
/// Check normalized increments for +/-1e-320 offsets from pi/2 at m=3/4.
/// For F/E compare the difference divided by the exact offset to endpoint
/// derivatives two and one half at 96-bit absolute accuracy. This finite quotient
/// check would reject snapping these represented displacements to zero.
fn tiny_offsets_at_quarter_periods_remain_distinct() {
    let m: Real = "0.75".parse().unwrap();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for negative in [false, true] {
        let delta: Real = if negative { "-1e-320" } else { "1e-320" }.parse().unwrap();
        let amplitude = &half_pi + &delta;
        for second in [false, true] {
            let complete = if second {
                Real::elliptic_e_complete(&m)
            } else {
                Real::elliptic_k(&m)
            }
            .unwrap();
            let shifted = if second {
                amplitude.clone().elliptic_e(&m)
            } else {
                amplitude.clone().elliptic_f(&m)
            }
            .unwrap();
            let derivative: Real = if second { "0.5" } else { "2" }.parse().unwrap();
            close(&((shifted - complete) / &delta).unwrap(), &derivative, 96);
        }
    }
}

#[test]
/// Check normalized behavior at signed amplitudes 1e320 and 1e-320 for m=1/2.
/// For both kinds compare the large value/amplitude ratio to twice the complete
/// integral divided by pi, and the tiny ratio to one, at 128 bits. These finite
/// asymptotic fixtures do not impose a native work bound for arbitrary magnitudes.
fn enormous_amplitudes_and_tiny_integrals_are_refinable() {
    let m: Real = "0.5".parse().unwrap();
    for sign in [-1, 1] {
        let huge = Real::from(sign) * "1e320".parse::<Real>().unwrap();
        let tiny = Real::from(sign) * "1e-320".parse::<Real>().unwrap();
        for second in [false, true] {
            let complete = if second {
                Real::elliptic_e_complete(&m)
            } else {
                Real::elliptic_k(&m)
            }
            .unwrap();
            let large = if second {
                huge.clone().elliptic_e(&m)
            } else {
                huge.clone().elliptic_f(&m)
            }
            .unwrap();
            let small = if second {
                tiny.clone().elliptic_e(&m)
            } else {
                tiny.clone().elliptic_f(&m)
            }
            .unwrap();
            close(
                &(large / &huge).unwrap(),
                &(Real::from(2) * complete / Real::pi()).unwrap(),
                128,
            );
            close(&(small / &tiny).unwrap(), &Real::one(), 128);
        }
    }
}

#[test]
/// Separate an undecided zero amplitude from an unproved positive parameter complement.
/// The opaque trigonometric zero is a valid amplitude at m=1/2 and agrees with
/// zero at 256 bits. Using it as 1-m must instead return Exhausted for all four
/// APIs, including zero-amplitude calls. Also retain exact equatorial identities
/// for the known valid parameter; uncertainty is not permission to assume a domain.
fn unresolved_parameter_is_not_a_circular_or_zero_identity() {
    let opaque_zero = Real::e().sin().powi(2.into()).unwrap()
        + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    // The amplitude has no domain restriction. An undecided zero is a valid
    // argument even though the same uncertainty cannot admit m=1 as m<1.
    for value in [
        opaque_zero
            .clone()
            .elliptic_f(&"0.5".parse().unwrap())
            .unwrap(),
        opaque_zero
            .clone()
            .elliptic_e(&"0.5".parse().unwrap())
            .unwrap(),
    ] {
        close(&value, &Real::zero(), 256);
    }
    let m = Real::one() - opaque_zero;
    assert_eq!(Real::elliptic_k(&m), Err(Problem::Exhausted));
    assert_eq!(Real::elliptic_e_complete(&m), Err(Problem::Exhausted));
    assert_eq!(Real::zero().elliptic_f(&m), Err(Problem::Exhausted));
    assert_eq!(Real::zero().elliptic_e(&m), Err(Problem::Exhausted));
    let parameter: Real = "0.5".parse().unwrap();
    assert_eq!(Real::zero().elliptic_f(&parameter).unwrap(), Real::zero());
    assert_eq!(Real::zero().elliptic_e(&parameter).unwrap(), Real::zero());
}

#[test]
/// Exercise sign retention, nonmonotone precision requests, abort/retry and concurrent readers.
/// For F/E at amplitude e and m=-3 require positive structural sign and overlap
/// across 32,512,64,256-bit enclosures, not nestedness. An already asserted stop
/// must deny an enclosure; clearing it recovers 256-bit agreement. Four readers
/// require positive lower bounds. This is same-process evidence, not a latency test.
fn precision_order_cancellation_recovery_and_shared_readers() {
    let m: Real = "-3".parse().unwrap();
    for value in [
        Real::e().elliptic_f(&m).unwrap(),
        Real::e().elliptic_e(&m).unwrap(),
    ] {
        assert_eq!(
            value.certified_sign_until(0).sign(),
            Some(RealSign::Positive)
        );
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let current = value.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = value.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-128).is_none());
        signal.store(false, Ordering::Relaxed);
        close(&aborted, &value, 256);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                let value = value.clone();
                scope.spawn(move || {
                    assert!(value.certified_rational_interval(-bits).unwrap()[0] > Rational::zero())
                });
            }
        });
    }
}

#[cfg(feature = "serde")]
#[test]
/// Check JSON/CBOR replay of both kinds at signed amplitudes +/-e and m=3/4.
/// Require structural equality, matching amplitude signs, unchanged JSON after
/// 512-bit warming, and 256-bit replay agreement. Numerical agreement and wire
/// stability are separate assertions; this does not qualify hostile payloads or
/// fresh-process cache isolation.
fn serialization_preserves_amplitude_parameter_sign_and_identity() {
    let m: Real = "0.75".parse().unwrap();
    for amplitude in [Real::e(), -Real::e()] {
        for value in [
            amplitude.clone().elliptic_f(&m).unwrap(),
            amplitude.clone().elliptic_e(&m).unwrap(),
        ] {
            let cold = value.to_json();
            let decoded = Real::from_json(&cold).unwrap();
            assert_eq!(value, decoded);
            assert_eq!(
                decoded.certified_sign_until(0).sign(),
                amplitude.certified_sign_until(0).sign()
            );
            let _ = value.certified_rational_interval(-512).unwrap();
            assert_eq!(cold, value.to_json());
            let mut bytes = Vec::new();
            ciborium::ser::into_writer(&value, &mut bytes).unwrap();
            let decoded: Real = ciborium::de::from_reader(bytes.as_slice()).unwrap();
            assert_eq!(value, decoded);
            close(&decoded, &value, 256);
        }
    }
}
