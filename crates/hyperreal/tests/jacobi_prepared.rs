use hyperreal::{Rational, Real};
/// Shared twenty-two scalar recipes for combined-output equivalence and relation checks.
#[path = "support/representations.rs"]
mod representations;

/// Compare four combined outputs with separate calls across 22 scalar recipes and three parameters.
/// Require exact zero residuals and, with serde enabled, identical JSON definitions.
/// This proves the tested value/representation agreement, not identical cancellation
/// attachments, allocation counts or performance.
#[test]
fn combined_outputs_preserve_separate_values_and_serialized_definitions() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        for text in ["0.5", "-3", "0"] {
            let parameter: Real = text.parse().unwrap();
            let argument = case.value.clone();
            let (s, c, d, e) = argument.clone().jacobi_sncndn_epsilon(&parameter).unwrap();
            let (ss, cc, dd) = argument.clone().jacobi_sncndn(&parameter).unwrap();
            let ee = argument.jacobi_epsilon(&parameter).unwrap();
            for (actual, expected) in [s, c, d, e].into_iter().zip([ss, cc, dd, ee]) {
                #[cfg(feature = "serde")]
                assert_eq!(
                    actual.to_json(),
                    expected.to_json(),
                    "{} / {text}",
                    case.certificate
                );
                assert_eq!(
                    (actual - expected).exact_rational_normal_form(),
                    Some(Rational::zero())
                );
            }
        }
    }
}

/// Unwrap a valid fixture's combined result into sn/cn/dn/epsilon order.
/// Admission or construction failure panics rather than substituting partial outputs.
fn outputs(argument: Real, parameter: &Real) -> [Real; 4] {
    let (s, c, d, e) = argument.jacobi_sncndn_epsilon(parameter).unwrap();
    [s, c, d, e]
}

/// Require shared square identities, positive dn and epsilon-to-inverse-E-to-F recovery for 44 cases.
/// Each of 22 scalar recipes supplies two signed symbolic parameters; exact rational
/// normal forms check the retained equations without numerical tolerance.
#[test]
fn shared_outputs_keep_exact_relations_with_symbolic_parameters() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        for m in [small.clone(), -small] {
            let [s, c, d, e] = outputs(case.value.clone(), &m);
            assert_eq!(
                (&s * &s + &c * &c).exact_rational_normal_form(),
                Some(Rational::one())
            );
            assert_eq!(
                (&d * &d + &m * (&s * &s)).exact_rational_normal_form(),
                Some(Rational::one())
            );
            let recovered = e.inverse_elliptic_e(&m).unwrap().elliptic_f(&m).unwrap();
            assert_eq!(
                (recovered - &case.value).exact_rational_normal_form(),
                Some(Rational::zero())
            );
            assert_eq!(
                d.certified_sign_until(0).sign(),
                Some(hyperreal::RealSign::Positive)
            );
        }
    }
}

/// Check eight signed/huge quarter multiples at three parameters against separate calls and n*E_complete.
/// All four residuals must reduce exactly to zero, with JSON agreement under serde.
/// The epsilon relation is checked independently of pairwise wrapper agreement.
#[test]
fn zero_and_quarter_points_preserve_existing_endpoint_forms() {
    for m in ["0.5".parse().unwrap(), Real::from(-3), Real::zero()] {
        let k = Real::elliptic_k(&m).unwrap();
        let complete = Real::elliptic_e_complete(&m).unwrap();
        for n in [
            "0",
            "-3",
            "-2",
            "-1",
            "1",
            "2",
            "3",
            "1000000000000000000000000000001",
        ] {
            let n: Real = n.parse().unwrap();
            let argument = &n * &k;
            let actual = outputs(argument.clone(), &m);
            let (s, c, d) = argument.clone().jacobi_sncndn(&m).unwrap();
            let expected = [s, c, d, argument.jacobi_epsilon(&m).unwrap()];
            for (a, b) in actual.iter().zip(&expected) {
                #[cfg(feature = "serde")]
                assert_eq!(a.to_json(), b.to_json());
                assert_eq!((a - b).exact_rational_normal_form(), Some(Rational::zero()));
            }
            assert_eq!(
                (&actual[3] - &n * &complete).exact_rational_normal_form(),
                Some(Rational::zero())
            );
        }
    }
}

/// Check invalid/unproved parameter failures before zero shortcuts and four opaque-zero outputs.
/// Require NotANumber for m=1,2, Exhausted for an unresolved boundary parameter,
/// and residual enclosures within 2^-128 at m=1/2 for an unrecognized zero input.
#[test]
fn shared_validation_precedes_zero_and_retains_unknown_domain_failures() {
    for parameter in [Real::one(), Real::from(2)] {
        assert_eq!(
            Real::zero().jacobi_sncndn_epsilon(&parameter),
            Err(hyperreal::Problem::NotANumber)
        );
    }
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(
        Real::zero().jacobi_sncndn_epsilon(&(Real::one() - &zero)),
        Err(hyperreal::Problem::Exhausted)
    );
    let values = outputs(zero, &"0.5".parse().unwrap());
    let bound = Rational::fraction(1, 2).unwrap().powi(128.into()).unwrap();
    for (value, expected) in
        values
            .into_iter()
            .zip([Real::zero(), Real::one(), Real::one(), Real::zero()])
    {
        let [lo, hi] = (value - expected)
            .certified_rational_interval(-144)
            .unwrap();
        assert!(lo >= -bound.clone() && hi <= bound);
    }
}

/// Check mixed-precision overlap and an inherited input flag for all four ordinary shared outputs.
/// At sin(e)|-3, asserting the input flag rejects -512 requests; clearing it permits
/// four concurrent readers and later -128 requests. This does not cover exact
/// endpoint carrier recovery, require nested enclosures or measure stop latency.
#[test]
fn shared_amplitude_preserves_precision_order_cancellation_and_concurrency() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let signal = Arc::new(AtomicBool::new(false));
    let mut argument = Real::e().sin();
    argument.abort(signal.clone());
    let values = outputs(argument, &Real::from(-3));
    for value in &values {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 256, 64, 128] {
            let current = value.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
    }
    signal.store(true, Ordering::Relaxed);
    for value in &values {
        assert!(value.certified_rational_interval(-512).is_none());
    }
    signal.store(false, Ordering::Relaxed);
    std::thread::scope(|scope| {
        for value in &values {
            scope.spawn(move || assert!(value.certified_rational_interval(-512).is_some()));
        }
    });
    for value in &values {
        assert!(value.certified_rational_interval(-128).is_some());
    }
}
