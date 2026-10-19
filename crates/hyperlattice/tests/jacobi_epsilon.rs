use hyperlattice::{Complex, Problem, Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr;
#[path = "support/complex_jacobi_oracle.rs"]
mod oracle;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod representations;

fn close(actual: &Complex, expected: &Complex, bits: i32) {
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    for (a, b) in [(&actual.re, &expected.re), (&actual.im, &expected.im)] {
        let [lo, hi] = (a - b).certified_rational_interval(-bits - 8).unwrap();
        assert!(
            lo >= -tolerance.clone() && hi <= tolerance,
            "error [{lo}, {hi}]"
        );
    }
}

fn exactly_equal(actual: Complex, expected: Complex) {
    let difference = actual - expected;
    assert_eq!(
        difference.re.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    assert_eq!(
        difference.im.exact_rational_normal_form(),
        Some(Rational::zero())
    );
}

#[test]
fn real_axis_preserves_the_defining_inverse_composition() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        for text in ["0.5", "-3", "0"] {
            let m: Real = text.parse().unwrap();
            let u = case.value.clone();
            let epsilon = u.clone().jacobi_epsilon(&m).unwrap();
            let complex = Complex::from(u.clone()).jacobi_epsilon(&m).unwrap();
            assert_eq!(complex, Complex::from(epsilon.clone()));
            let recovered = epsilon
                .inverse_elliptic_e(&m)
                .unwrap()
                .elliptic_f(&m)
                .unwrap();
            assert_eq!(
                (recovered - u).exact_rational_normal_form(),
                Some(Rational::zero()),
                "{} / {text}",
                case.certificate
            );
        }
    }
}

#[test]
fn independent_ode_epsilon_enclosures() {
    mpfr::cleanup_on_thread_exit();
    for text in ["0.5", "0.75", "-3", "-0.5", "0"] {
        for (x, y) in [
            ("0.125", "0.0625"),
            ("-0.125", "0.0625"),
            ("0", "-0.125"),
            ("0.125", "0"),
        ] {
            oracle::compare(
                &Complex::new(x.parse().unwrap(), y.parse().unwrap()),
                &text.parse().unwrap(),
                true,
            );
        }
    }
}

#[test]
fn all_scalar_families_reach_independent_epsilon_reference() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        let argument = Complex::new(small.clone(), (&small / Real::from(2)).unwrap());
        for m in [small.clone(), -small, Real::zero()] {
            oracle::compare(&argument, &m, true);
        }
    }
}

#[test]
fn parity_conjugation_and_quasi_addition_hold() {
    let u = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
    let v = Complex::new("-0.0625".parse().unwrap(), "0.125".parse().unwrap());
    for text in ["0.5", "-3", "0"] {
        let m: Real = text.parse().unwrap();
        let epsilon = u.clone().jacobi_epsilon(&m).unwrap();
        close(
            &u.clone().conjugate().jacobi_epsilon(&m).unwrap(),
            &epsilon.clone().conjugate(),
            160,
        );
        close(
            &(-u.clone()).jacobi_epsilon(&m).unwrap(),
            &(-epsilon.clone()),
            160,
        );
        let sum = &u + &v;
        let su = u.clone().jacobi_sncndn(&m).unwrap().0;
        let sv = v.clone().jacobi_sncndn(&m).unwrap().0;
        let ss = sum.clone().jacobi_sncndn(&m).unwrap().0;
        let expected = epsilon + v.clone().jacobi_epsilon(&m).unwrap()
            - Complex::from(m.clone()) * su * sv * ss;
        close(&sum.jacobi_epsilon(&m).unwrap(), &expected, 160);
    }
}

#[test]
fn finite_quarter_periods_and_true_poles_are_distinct() {
    let half: Real = "0.5".parse().unwrap();
    let k = Real::elliptic_k(&half).unwrap();
    let e = Real::elliptic_e_complete(&half).unwrap();
    assert_eq!(
        Complex::new(Real::zero(), k.clone()).jacobi_epsilon(&half),
        Err(Problem::DivideByZero)
    );
    exactly_equal(
        Complex::new(k.clone(), k.clone())
            .jacobi_epsilon(&half)
            .unwrap(),
        Complex::new(e.clone(), &k - &e),
    );
    exactly_equal(
        Complex::new(Real::zero(), Real::from(2) * &k)
            .jacobi_epsilon(&half)
            .unwrap(),
        Complex::new(Real::zero(), Real::from(2) * (&k - &e)),
    );

    let m = Real::from(-3);
    let reciprocal: Real = "0.25".parse().unwrap();
    let kr = Real::elliptic_k(&reciprocal).unwrap();
    let er = Real::elliptic_e_complete(&reciprocal).unwrap();
    let quarter = (&kr / Real::from(2)).unwrap();
    exactly_equal(
        Complex::new(Real::zero(), quarter.clone())
            .jacobi_epsilon(&m)
            .unwrap(),
        Complex::new(Real::zero(), Real::from(2) * (&kr - &er)),
    );
    assert_eq!(
        Complex::new(Real::elliptic_k(&m).unwrap(), quarter).jacobi_epsilon(&m),
        Err(Problem::DivideByZero)
    );
}

#[test]
fn huge_quasi_periods_and_tiny_arguments_retain_their_offsets() {
    let cycles: Real = "1e30".parse().unwrap();
    let small = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
    for m in ["0.5".parse::<Real>().unwrap(), Real::from(-3)] {
        let k = Real::elliptic_k(&m).unwrap();
        let e = Real::elliptic_e_complete(&m).unwrap();
        let h = Real::one() - &m;
        let (ki, ei) = if m == Real::from(-3) {
            let reciprocal = h.clone().inverse().unwrap();
            let r = h.sqrt().unwrap();
            let kr = Real::elliptic_k(&reciprocal).unwrap();
            let er = Real::elliptic_e_complete(&reciprocal).unwrap();
            ((&kr / &r).unwrap(), r * (&kr - er))
        } else {
            let k = Real::elliptic_k(&h).unwrap();
            let e = Real::elliptic_e_complete(&h).unwrap();
            (k.clone(), k - e)
        };
        let twice = Real::from(2) * &cycles;
        let translated = Complex::new(&small.re + &twice * k, &small.im + &twice * ki);
        let expected =
            small.clone().jacobi_epsilon(&m).unwrap() + Complex::new(&twice * e, &twice * ei);
        close(&translated.jacobi_epsilon(&m).unwrap(), &expected, 128);
        let tiny = Complex::new("1e-80".parse().unwrap(), "-1e-80".parse().unwrap());
        close(&tiny.clone().jacobi_epsilon(&m).unwrap(), &tiny, 700);
    }
}

#[test]
fn exact_near_pole_offsets_remain_finite() {
    for text in ["1e-20", "1e-60"] {
        let offset: Real = text.parse().unwrap();
        for m in ["0.5".parse::<Real>().unwrap(), Real::from(-3)] {
            let (x, y) = if m == Real::from(-3) {
                (
                    Real::elliptic_k(&m).unwrap() + &offset,
                    (Real::elliptic_k(&"0.25".parse().unwrap()).unwrap() / Real::from(2)).unwrap(),
                )
            } else {
                (offset.clone(), Real::elliptic_k(&m).unwrap())
            };
            let value = Complex::new(x, y).jacobi_epsilon(&m).unwrap();
            for component in [&value.re, &value.im] {
                assert!(component.certified_rational_interval(-128).is_some());
            }
        }
    }
}

#[test]
fn zero_parameter_is_the_identity_at_huge_arguments() {
    let z = Complex::new("1e320".parse().unwrap(), Real::from(1000));
    assert_eq!(z.clone().jacobi_epsilon(&Real::zero()).unwrap(), z);
}

#[test]
fn invalid_parameters_and_unknown_poles_keep_their_errors() {
    for m in [Real::one(), Real::from(2)] {
        assert_eq!(Real::zero().jacobi_epsilon(&m), Err(Problem::NotANumber));
        assert_eq!(Complex::zero().jacobi_epsilon(&m), Err(Problem::NotANumber));
    }
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(Complex::i().jacobi_epsilon(&zero), Err(Problem::Exhausted));
    assert!(Complex::zero().jacobi_epsilon(&zero).is_ok());
    assert_eq!(
        Complex::zero().jacobi_epsilon(&(Real::one() - &zero)),
        Err(Problem::Exhausted)
    );
    let half: Real = "0.5".parse().unwrap();
    assert_eq!(
        Complex::new(zero, Real::elliptic_k(&half).unwrap()).jacobi_epsilon(&half),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn precision_order_cancellation_and_concurrency_preserve_values() {
    let value = Complex::new("0.25".parse().unwrap(), "0.125".parse().unwrap())
        .jacobi_epsilon(&Real::from(-3))
        .unwrap();
    for component in [&value.re, &value.im] {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let current = component.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = component.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-128).is_none());
        signal.store(false, Ordering::Relaxed);
        assert!(aborted.certified_rational_interval(-256).is_some());
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                scope
                    .spawn(move || assert!(component.certified_rational_interval(-bits).is_some()));
            }
        });
    }
}

#[cfg(feature = "serde")]
#[test]
fn serialized_components_replay_values_and_domain_obligations() {
    for m in [
        "0.5".parse().unwrap(),
        Real::from(-3),
        (Real::pi() / Real::from(8)).unwrap(),
    ] {
        let original = Complex::new(Real::e(), "0.125".parse().unwrap())
            .jacobi_epsilon(&m)
            .unwrap();
        let original = [original.re, original.im];
        let serialized = serde_json::to_string(&original).unwrap();
        let decoded: [Real; 2] = serde_json::from_str(&serialized).unwrap();
        for (a, b) in decoded.iter().zip(&original) {
            let [a, bound_a] = a.certified_rational_interval(-256).unwrap();
            let [b, bound_b] = b.certified_rational_interval(-256).unwrap();
            assert!(a <= bound_b && b <= bound_a);
        }
        assert_eq!(serialized, serde_json::to_string(&decoded).unwrap());
    }
}
