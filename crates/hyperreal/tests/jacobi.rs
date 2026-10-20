use hyperreal::{Problem, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/inverse_legendre_oracle.rs"]
mod inverse_oracle;
#[path = "support/jacobi_oracle.rs"]
mod jacobi_oracle;
#[path = "support/legendre_oracle.rs"]
mod legendre_oracle;
#[path = "support/mpfr.rs"]
mod mpfr;
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

fn relations((sn, cn, dn): &(Real, Real, Real), m: &Real) {
    // Public PartialEq compares carriers. Require an exact algebraic
    // certificate here, with no approximation or numerical tolerance.
    assert_eq!(
        (sn * sn + cn * cn).exact_rational_normal_form(),
        Some(Rational::one()),
        "unit norm"
    );
    assert_eq!(
        dn.certified_sign_until(0),
        hyperreal::CertifiedRealSign::Known {
            sign: RealSign::Positive,
            certificate: hyperreal::RealSignCertificate::StructuralFacts,
        }
    );
    assert!(dn.inverse_ref().is_ok());
    let elliptic = cn * cn + (Real::one() - m) * (sn * sn);
    let residual = elliptic - dn * dn;
    assert_eq!(
        residual.exact_rational_normal_form(),
        Some(Rational::zero()),
        "positive elliptic norm"
    );
    assert_eq!(
        (dn * dn + m * (sn * sn)).exact_rational_normal_form(),
        Some(Rational::one()),
        "dn²+m sn²"
    );
    assert_eq!(
        (dn * dn - m * (cn * cn) - (Real::one() - m)).exact_rational_normal_form(),
        Some(Rational::zero()),
        "dn²-m cn²"
    );
}

#[test]
fn independent_amplitudes_and_extreme_complements() {
    mpfr::cleanup_on_thread_exit();
    for text in ["0.25", "4", "1e-160", "1e160"] {
        let c: Real = text.parse().unwrap();
        let m = Real::one() - &c;
        let k = legendre_oracle::native(legendre_oracle::complete(&c)[0].clone());
        let values = (k / Real::from(2)).unwrap().jacobi_sncndn(&m).unwrap();
        // At K/2: sn=1/sqrt(1+sqrt(c)), cn=c^(1/4)/sqrt(1+sqrt(c)), dn=c^(1/4).
        let root = c.sqrt().unwrap();
        let fourth = root.clone().sqrt().unwrap();
        let denominator = (Real::one() + root).sqrt().unwrap();
        for (actual, expected) in [values.0, values.1, values.2].into_iter().zip([
            (Real::one() / &denominator).unwrap(),
            (&fourth / denominator).unwrap(),
            fourth,
        ]) {
            close(&actual, &expected, 160);
        }
    }
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        let phi = Real::pi() + &small;
        for m in [small.clone(), -small] {
            // Fresh rational integral points prevent native inverse cancellation.
            let u = legendre_oracle::native(legendre_oracle::series(&phi, &m)[0].clone());
            let values = u.jacobi_sncndn(&m).unwrap();
            close(&values.0, &phi.clone().sin(), 256);
            close(&values.1, &phi.clone().cos(), 256);
        }
    }
}

#[test]
fn tiny_offsets_and_huge_periods_remain_distinct() {
    let m: Real = "0.75".parse().unwrap();
    let k = Real::elliptic_k(&m).unwrap();
    let tiny: Real = "1e-320".parse().unwrap();
    for sign in [-1, 1] {
        let delta = Real::from(sign) * &tiny;
        let (sn, _, _) = delta.clone().jacobi_sncndn(&m).unwrap();
        close(&(sn / &delta).unwrap(), &Real::one(), 96);
        let (_, cn, dn) = (&k + &delta).jacobi_sncndn(&m).unwrap();
        close(&(cn / &delta).unwrap(), &"-0.5".parse().unwrap(), 96);
        close(&dn, &"0.5".parse().unwrap(), 96);
    }
    let huge: Real = "1e320".parse().unwrap();
    for n in [-1, 0, 1, 2, 3] {
        let periods = &huge + Real::from(n);
        let angle = (Real::pi() * &periods / Real::from(2)).unwrap();
        let endpoints = (&periods * &k).jacobi_sncndn(&m).unwrap();
        assert_eq!(endpoints.0, angle.clone().sin());
        assert_eq!(endpoints.1, angle.cos());
        let residual: Real = "0.125".parse().unwrap();
        let values = (&periods * &k + &residual).jacobi_sncndn(&m).unwrap();
        let expected = (Real::from(n) * &k + residual).jacobi_sncndn(&m).unwrap();
        for (a, b) in [values.0, values.1, values.2]
            .into_iter()
            .zip([expected.0, expected.1, expected.2])
        {
            close(&a, &b, 128);
        }
    }
    for sign in [-1, 1] {
        let angle = Real::from(sign) * &huge * Real::pi() + Real::e();
        close(&angle.clone().sin(), &Real::e().sin(), 128);
        close(&angle.cos(), &Real::e().cos(), 128);
        let input = Real::from(sign) * &huge;
        let (sn, cn, dn) = input.clone().jacobi_sncndn(&Real::zero()).unwrap();
        close(&sn, &input.clone().sin(), 128);
        close(&cn, &input.cos(), 128);
        assert_eq!(dn, Real::one());
    }
}

#[test]
fn mismatched_arguments_parameters_and_coefficients_never_cancel() {
    let m: Real = "0.5".parse().unwrap();
    let (s, c, d) = "0.75".parse::<Real>().unwrap().jacobi_sncndn(&m).unwrap();
    for (u, parameter) in [("0.5", "0.5"), ("0.75", "-0.5")] {
        let (other_s, other_c, _) = u
            .parse::<Real>()
            .unwrap()
            .jacobi_sncndn(&parameter.parse().unwrap())
            .unwrap();
        for residual in [
            &s * &s + &other_c * &other_c - Real::one(),
            &c * &c + &other_s * &other_s - Real::one(),
        ] {
            assert_ne!(
                residual.exact_rational_normal_form(),
                Some(Rational::zero())
            );
            assert!(
                residual
                    .certified_sign_until(-128)
                    .sign()
                    .is_some_and(|s| s != RealSign::Zero)
            );
        }
    }
    for residual in [
        &d * &d + "0.4".parse::<Real>().unwrap() * (&s * &s) - Real::one(),
        &d * &d - "0.4".parse::<Real>().unwrap() * (&c * &c) - (Real::one() - &m),
    ] {
        assert!(
            residual
                .certified_sign_until(-128)
                .sign()
                .is_some_and(|s| s != RealSign::Zero)
        );
    }
}

#[test]
fn precision_order_abort_recovery_and_concurrent_readers() {
    let m: Real = "-3".parse().unwrap();
    let values = Real::e().jacobi_sncndn(&m).unwrap();
    for value in [values.0, values.1, values.2] {
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
                scope.spawn(move || assert!(value.certified_rational_interval(-bits).is_some()));
            }
        });
    }
}

#[test]
fn unresolved_domain_and_unknown_zero_are_honest() {
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(
        Real::zero().jacobi_sncndn(&(Real::one() - &zero)),
        Err(Problem::Exhausted)
    );
    let values = zero.jacobi_sncndn(&"0.5".parse().unwrap()).unwrap();
    for (a, b) in
        [values.0, values.1, values.2]
            .into_iter()
            .zip([Real::zero(), Real::one(), Real::one()])
    {
        close(&a, &b, 128);
    }
}

#[cfg(feature = "serde")]
#[test]
fn json_and_cbor_replay_preserve_values_relations_and_positive_branch() {
    for m in [
        "0.75".parse().unwrap(),
        (Real::pi() / Real::from(8)).unwrap(),
    ] {
        let values = Real::e().jacobi_sncndn(&m).unwrap();
        let cold = serde_json::to_string(&values).unwrap();
        let decoded: (Real, Real, Real) = serde_json::from_str(&cold).unwrap();
        assert_eq!(decoded, values);
        assert_eq!(
            decoded.2.certified_sign_until(0).sign(),
            Some(RealSign::Positive)
        );
        assert_eq!(
            (&decoded.0 * &decoded.0 + &decoded.1 * &decoded.1).exact_rational_normal_form(),
            Some(Rational::one())
        );
        for value in [&decoded.0, &decoded.1, &decoded.2] {
            assert!(value.certified_rational_interval(-256).is_some());
        }
        relations(&decoded, &m);
        assert_eq!(cold, serde_json::to_string(&decoded).unwrap());
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&values, &mut bytes).unwrap();
        let decoded: (Real, Real, Real) = ciborium::de::from_reader(bytes.as_slice()).unwrap();
        for (a, b) in [&decoded.0, &decoded.1, &decoded.2]
            .into_iter()
            .zip([&values.0, &values.1, &values.2])
        {
            assert!(a.certified_rational_interval(-256).is_some());
            close(a, b, 256);
        }
        relations(&decoded, &m);
    }
}

#[test]
fn domain_zero_and_quarter_period_endpoints() {
    for m in [Real::one(), Real::from(2)] {
        assert_eq!(Real::zero().jacobi_sncndn(&m), Err(Problem::NotANumber));
    }
    for m in [Real::zero(), Real::from(-3), "0.75".parse().unwrap()] {
        assert_eq!(
            Real::zero().jacobi_sncndn(&m).unwrap(),
            (Real::zero(), Real::one(), Real::one())
        );
        let k = Real::elliptic_k(&m).unwrap();
        let root = (Real::one() - &m).sqrt().unwrap();
        for n in -8_i32..=8 {
            let expected = match n.rem_euclid(4) {
                0 => (Real::zero(), Real::one(), Real::one()),
                1 => (Real::one(), Real::zero(), root.clone()),
                2 => (Real::zero(), -Real::one(), Real::one()),
                _ => (-Real::one(), Real::zero(), root.clone()),
            };
            assert_eq!((Real::from(n) * &k).jacobi_sncndn(&m).unwrap(), expected);
        }
    }
}

#[test]
fn retained_square_relations_cover_every_scalar_family() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        eprintln!("Jacobi relations: {}", case.certificate);
        let small = (&case.value / (Real::from(2) * (&case.value + Real::one()))).unwrap();
        for m in [Real::zero(), small.clone(), -small] {
            let values = case.value.clone().jacobi_sncndn(&m).unwrap();
            relations(&values, &m);
            let (sn, cn, dn) = values;
            let scale: Real = "-2.5".parse().unwrap();
            assert_eq!(
                (&scale * (&sn * &sn) + &scale * (&cn * &cn)).exact_rational_normal_form(),
                scale.exact_rational()
            );
            close(&(&dn * &dn), &(Real::one() - &m * &sn * &sn), 160);
        }
    }
}

#[test]
fn fresh_rational_arguments_match_independent_mpfr_inverse() {
    mpfr::cleanup_on_thread_exit();
    for m in ["0.5", "-0.75"] {
        let m: Real = m.parse().unwrap();
        for u in ["0.3125", "-1.8", "3.5"] {
            let u: Real = u.parse().unwrap();
            let expected = jacobi_oracle::values(&u, &m);
            let (sn, cn, dn) = u.jacobi_sncndn(&m).unwrap();
            for (actual, expected) in [sn, cn, dn].into_iter().zip(expected) {
                close(&actual, &legendre_oracle::native(expected), 512);
            }
        }
    }
}
