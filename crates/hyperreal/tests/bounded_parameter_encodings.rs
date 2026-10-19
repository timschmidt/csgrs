use hyperreal::{Rational, Real};
#[path = "support/representations.rs"]
mod representations;

#[test]
fn bounded_parameter_encodings_retain_all_scalar_family_values() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let value = &case.value;
        let unit = (value / (value + Real::one())).unwrap();
        let m = (&unit / Real::from(2)).unwrap();
        let h = Real::one() - &m;
        let r = (m.clone() / &h).unwrap().sqrt().unwrap();
        let decoded_h = (Real::one() / (Real::one() + &r * &r)).unwrap();
        assert_eq!(
            (decoded_h - h).exact_rational_normal_form(),
            Some(Rational::zero()),
            "parameter {}",
            case.certificate
        );
        for sign in [-1, 1] {
            let v = Real::from(sign) * &unit;
            let a = (v.clone() / (Real::one() - &v * &v).sqrt().unwrap()).unwrap();
            let decoded_v = (a.clone() / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
            assert_eq!(
                (decoded_v - &v).exact_rational_normal_form(),
                Some(Rational::zero()),
                "meridian {} / {sign}",
                case.certificate
            );
            let decoded_time = v.clone().asin().unwrap().sin();
            assert_eq!(
                (decoded_time - v).exact_rational_normal_form(),
                Some(Rational::zero()),
                "time {} / {sign}",
                case.certificate
            );
        }
    }
    for v in [Real::zero(), Real::one(), -Real::one()] {
        assert_eq!(
            (v.clone().asin().unwrap().sin() - v).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

#[cfg(feature = "serde")]
mod serialization {
    use super::*;
    fn replay(value: &[Real; 5]) -> [Real; 5] {
        serde_json::from_str(&serde_json::to_string(value).unwrap()).unwrap()
    }

    fn encoded(v: Real, m: Real) -> [Real; 5] {
        let r = (&m / (Real::one() - &m)).unwrap().sqrt().unwrap();
        let a = (&v / (Real::one() - &v * &v).sqrt().unwrap()).unwrap();
        let tau = v.clone().asin().unwrap();
        [v, m, r, a, tau]
    }

    fn accept([v, m, r, a, tau]: [Real; 5], context: &str) {
        let decoded_h = (Real::one() / (Real::one() + &r * &r)).unwrap();
        let decoded_v = (&a / (Real::one() + &a * &a).sqrt().unwrap()).unwrap();
        for (label, residual) in [
            ("parameter", decoded_h - (Real::one() - m)),
            ("meridian", decoded_v - &v),
            ("time", tau.sin() - v),
        ] {
            assert_eq!(
                residual.exact_rational_normal_form(),
                Some(Rational::zero()),
                "{context}: {label}"
            );
            assert_eq!(
                residual.sin().exact_rational_normal_form(),
                Some(Rational::zero())
            );
        }
    }

    #[test]
    fn serialized_parameter_encodings_replay_cold_and_after_display_warming() {
        for case in representations::optimized_certificate_representatives() {
            assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
            let unit = (&case.value / (&case.value + Real::one())).unwrap();
            let m = (&unit / Real::from(2)).unwrap();
            for sign in [-1, 1] {
                let source = encoded(Real::from(sign) * &unit, m.clone());
                for warm in [false, true] {
                    let decoded = replay(&source);
                    if warm {
                        for value in &decoded {
                            let _ = value.to_f64_lossy();
                        }
                    }
                    accept(
                        decoded,
                        &format!("{} sign={sign} warm={warm}", case.certificate),
                    );
                }
            }
        }
    }

    #[test]
    fn parameter_zero_and_exact_near_endpoint_offsets_survive_replay() {
        let delta: Real = "1e-100".parse().unwrap();
        for m in [Real::zero(), delta.clone(), Real::one() - &delta] {
            for v in [Real::zero(), -Real::one() + &delta, Real::one() - &delta] {
                accept(replay(&encoded(v, m.clone())), "boundary");
            }
        }
    }

    #[test]
    fn binary_replay_and_concurrent_proofs_preserve_authoritative_values() {
        for case in representations::optimized_certificate_representatives() {
            let unit = (&case.value / (&case.value + Real::one())).unwrap();
            let m = (&unit / Real::from(2)).unwrap();
            let source = encoded(-unit, m);
            let mut cbor = Vec::new();
            ciborium::ser::into_writer(&source, &mut cbor).unwrap();
            let decoded: [Real; 5] = ciborium::de::from_reader(cbor.as_slice()).unwrap();
            let wire = serde_json::to_string(&decoded).unwrap();
            std::thread::scope(|scope| {
                let jobs: Vec<_> = (0..4)
                    .map(|_| {
                        let decoded = &decoded;
                        let label = case.certificate;
                        scope.spawn(move || {
                            for _ in 0..3 {
                                accept(decoded.clone(), label);
                            }
                        })
                    })
                    .collect();
                for job in jobs {
                    job.join().unwrap();
                }
            });
            assert_eq!(serde_json::to_string(&decoded).unwrap(), wire);
            for precision in [-32, -192, -32] {
                for value in &decoded {
                    let [lo, hi] = value.certified_rational_interval(precision).unwrap();
                    assert!(lo <= hi);
                }
                accept(decoded.clone(), case.certificate);
                assert_eq!(serde_json::to_string(&decoded).unwrap(), wire);
            }
            let mut after = Vec::new();
            ciborium::ser::into_writer(&decoded, &mut after).unwrap();
            assert_eq!(after, cbor);
        }
    }
}
