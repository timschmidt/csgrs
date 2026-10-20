use hyperreal::{Problem, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/mpfr.rs"]
mod mpfr;
#[path = "support/legendre_oracle.rs"]
mod oracle;
#[path = "support/representations.rs"]
mod representations;

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
