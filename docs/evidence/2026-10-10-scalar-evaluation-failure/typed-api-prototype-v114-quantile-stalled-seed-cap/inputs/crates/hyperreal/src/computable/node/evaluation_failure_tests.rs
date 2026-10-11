#[cfg(test)]
mod evaluation_failure_tests {
    use super::*;
    use crate::Real;
    use std::sync::atomic::{AtomicBool, Ordering};

    type Reducer = fn(Computable) -> Approximation;
    type Constructor = fn(Computable) -> Computable;

    #[test]
    fn quantile_seed_precision_cannot_stall_refinement() {
        let value = Computable::normal_quantile(
            Computable::rational(Rational::fraction(1, 4).unwrap()),
            BigInt::zero(),
            0,
        );
        let sample = value.approx(-16).unwrap();
        // Phi(0)=1/2 and Phi(-1)<1/4, so the quantile is in (-1,0).
        // For the latter bound, pi<4 gives 1/sqrt(2*pi)>1/3, while
        // exp(-t*t/2)>=1-t*t/2 gives integral_0^1 phi(t) dt > 5/18.
        assert!(&sample - 1 < BigInt::zero());
        assert!(&sample + 1 > -(BigInt::one() << 16_usize));
    }

    #[test]
    fn quantile_coarse_public_seed_is_not_a_value_certificate() {
        let mut failures = Vec::new();
        for seed in [-1_000_000_000_i64, -42, 42, 1_000_000_000] {
            for precision in [0, 8, Precision::MAX] {
                let value = Computable::normal_quantile(
                    Computable::rational(Rational::fraction(1, 2).unwrap()),
                    BigInt::from(seed),
                    0,
                );
                let result = value.approx(precision);
                // Symmetry of the normal density proves Phi^-1(1/2) = 0.
                if !matches!(&result, Ok(sample) if sample.abs() <= BigInt::one()) {
                    failures.push(format!("seed={seed}, p={precision}: {result:?}"));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn quantile_coarse_serialized_domains_precede_seed_shortcuts() {
        let mut failures = Vec::new();
        for probability in [-1, 0, 1, 2] {
            for precision in [0, 8, Precision::MAX] {
                let value = restore_raw_node(Approximation::NormalQuantile(Box::new(NormalQuantileData {
                    p: Computable::rational(Rational::new(probability)),
                    seed: BigInt::zero(),
                    seed_prec: -16,
                })));
                let result = value.approx(precision);
                if result != Err(Problem::NotANumber) {
                    failures.push(format!("probability={probability}, p={precision}: {result:?}"));
                }
                if value.internal.cache_snapshot().is_some() {
                    failures.push(format!("probability={probability}, p={precision}: cached invalid result"));
                }
            }
        }
        let value = restore_raw_node(Approximation::NormalQuantile(Box::new(NormalQuantileData {
            p: Computable::rational(Rational::fraction(1, 2).unwrap()),
            seed: BigInt::from(1_000_000_000),
            seed_prec: 0,
        })));
        let result = value.approx(0);
        if !matches!(&result, Ok(sample) if sample.abs() <= BigInt::one()) {
            failures.push(format!("serialized median with false seed: {result:?}"));
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_root_endpoints_keep_exact_certificates() {
        let mut endpoints = vec![
            raw_node(Approximation::Sqrt(Computable::one())),
            raw_node(Approximation::Square(raw_node(Approximation::Sqrt(
                Computable::one(),
            )))),
            Computable::rational(Rational::new(2))
                .sqrt()
                .multiply(Computable::rational(Rational::new(2)).sqrt())
                .multiply_rational(Rational::fraction(1, 2).unwrap()),
        ];
        for degree in 3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE {
            endpoints.push(raw_node(Approximation::NthRoot(Computable::one(), degree)));
        }
        for endpoint in endpoints {
            for negative in [false, true] {
                let child = if negative {
                    raw_node(Approximation::Negate(endpoint.clone()))
                } else {
                    endpoint.clone()
                };
                for reducer in [Approximation::AsinDeferred, Approximation::PrescaledAsin] {
                    let mut value = restore_raw_node(reducer(child.clone()));
                    value.abort(Arc::new(AtomicBool::new(true)));
                    assert_eq!(
                        value.structural_facts().sign,
                        Some(if negative {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                    assert!(value.internal.cache_snapshot().is_none());
                    let sample = value.approx_signal(&None, 0).unwrap();
                    let sample = if negative { -sample } else { sample };
                    // 3 < pi < 4 implies pi/2 lies strictly between 3/2 and 2.
                    assert!((&sample - 1) * 2 < BigInt::from(4));
                    assert!((&sample + 1) * 2 > BigInt::from(3));
                }
                let value = restore_raw_node(Approximation::AcosPositive(child.clone()));
                if negative {
                    let sample = value.approx(0).unwrap();
                    assert!(&sample - 1 < BigInt::from(4) && &sample + 1 > BigInt::from(3));
                } else {
                    assert!(value.approx(-64).unwrap().abs() <= BigInt::one());
                }
                for reducer in [Approximation::AtanhDirect, Approximation::PrescaledAtanh] {
                    let value = restore_raw_node(reducer(child.clone()));
                    assert_eq!(value.approx(0), Err(Problem::NotANumber));
                    assert!(value.internal.cache_snapshot().is_none());
                }
            }
        }
        // A product of two imaginary square roots is not a real rational
        // witness, even though multiplying their radicands would give one.
        let invalid = Computable::rational(Rational::new(-1)).sqrt();
        let child = raw_node(Approximation::Multiply(invalid.clone(), invalid));
        let value = restore_raw_node(Approximation::AsinDeferred(child));
        assert_eq!(value.approx(0), Err(Problem::SqrtNegative));
        assert!(value.internal.cache_snapshot().is_none());
    }

    #[test]
    fn public_inverse_constructors_do_not_cache_invalid_signs() {
        let constructors: [(&str, Constructor); 3] = [
            ("asin", Computable::asin),
            ("acos", Computable::acos),
            ("atanh", Computable::atanh),
        ];
        let mut failures = Vec::new();
        for (name, constructor) in constructors {
            for n in [-2, 2] {
                for api in ["bound", "sign", "facts"] {
                    let value = constructor(Computable::rational(Rational::new(n)));
                    let invalid = match api {
                        "bound" => value.cheap_bound().known_sign().is_some(),
                        "sign" => value.exact_sign().is_some(),
                        _ => {
                            let facts = value.structural_facts();
                            facts.sign.is_some() || facts.zero != ZeroKnowledge::Unknown
                        }
                    };
                    if invalid {
                        failures.push(format!("{name}({n}), {api}: definite invalid-domain fact"));
                    }
                    assert!(value.internal.cache_snapshot().is_none());
                }
            }
        }
        for (name, constructor) in &constructors[..2] {
            let value = constructor(Computable::pi());
            let facts = value.structural_facts();
            if facts.sign.is_some() || facts.zero != ZeroKnowledge::Unknown {
                failures.push(format!("{name}(pi): {facts:?}"));
            }
        }
        for (name, constructor) in constructors {
            for n in if name == "atanh" {
                &[-2, -1, 1, 2][..]
            } else {
                &[-2, 2][..]
            } {
                for precision in [-64, 0, 1, Precision::MAX] {
                    let value = constructor(Computable::rational(Rational::new(*n)));
                    let result = value.approx(precision);
                    if result != Err(Problem::NotANumber)
                        || value.internal.cache_snapshot().is_some()
                    {
                        failures.push(format!("{name}({n}), p={precision}: {result:?}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_facts_require_unit_interval_certificates() {
        let reducers: [(&str, Reducer); 5] = [
            ("asin", Approximation::AsinDeferred),
            ("asin series", Approximation::PrescaledAsin),
            ("acos", Approximation::AcosPositive),
            ("atanh", Approximation::AtanhDirect),
            ("atanh series", Approximation::PrescaledAtanh),
        ];
        let mut cases = Vec::new();
        for (name, reducer) in reducers {
            for child in [Computable::pi(), Computable::pi().negate()] {
                cases.push((name, reducer(child), None));
            }
        }
        for reducer in [Approximation::AtanhDirect, Approximation::PrescaledAtanh] {
            for n in [-1, 1] {
                cases.push((
                    "atanh endpoint",
                    reducer(Computable::rational(Rational::new(n))),
                    None,
                ));
            }
        }
        assert_serialized_fact_cases(cases);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_domains_precede_every_numeric_shortcut() {
        let reducers: [(&str, Reducer); 5] = [
            ("asin", Approximation::AsinDeferred),
            ("asin series", Approximation::PrescaledAsin),
            ("acos", Approximation::AcosPositive),
            ("atanh", Approximation::AtanhDirect),
            ("atanh series", Approximation::PrescaledAtanh),
        ];
        let mut failures = Vec::new();
        for (name, reducer) in reducers {
            for n in [-2, 2] {
                for precision in [1, 0, -64, 8, Precision::MAX] {
                    // An arithmetic graph exercises the generic domain path;
                    // its value is exactly n, independently of approximation.
                    let child = raw_node(Approximation::Add(
                        Computable::rational(Rational::new(n / 2)),
                        Computable::rational(Rational::new(n / 2)),
                    ));
                    let value = restore_raw_node(reducer(child));
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        value.approx(precision)
                    }));
                    if !matches!(result, Ok(Err(Problem::NotANumber))) {
                        failures.push(format!("{name}({n}), p={precision}: {result:?}"));
                    }
                    if value.internal.cache_snapshot().is_some() {
                        failures.push(format!("{name}({n}), p={precision}: cached invalid value"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_boundaries_match_independent_enclosures() {
        use num::rational::BigRational as Q;
        let (pi_lower, pi_upper) = independent_pi_bounds();
        let reducers: [(&str, Reducer); 3] = [
            ("asin", Approximation::AsinDeferred),
            ("asin series", Approximation::PrescaledAsin),
            ("acos", Approximation::AcosPositive),
        ];
        for (name, reducer) in reducers {
            for (n, asin_n, asin_d) in [(-2, -1, 2), (-1, -1, 6), (0, 0, 1), (1, 1, 6), (2, 1, 2)] {
                let coefficient = Q::new(BigInt::from(asin_n), BigInt::from(asin_d));
                let coefficient = if name == "acos" {
                    Q::new(1.into(), 2.into()) - coefficient
                } else {
                    coefficient
                };
                for warm in [false, true] {
                    for precision in [-256_i32, -64, -16, 0, 1, 2, 8] {
                        let quarter = Computable::rational(Rational::fraction(n, 4).unwrap());
                        let child = raw_node(Approximation::Add(quarter.clone(), quarter));
                        let value = restore_raw_node(reducer(child));
                        let sample = prepared_sample(value, warm, precision);
                        let scale = if precision < 0 {
                            Q::from_integer(BigInt::one() << precision.unsigned_abs())
                        } else {
                            Q::new(BigInt::one(), BigInt::one() << precision)
                        };
                        let (lower, upper) = if coefficient < Q::zero() {
                            (
                                &pi_upper * &coefficient * &scale,
                                &pi_lower * &coefficient * scale,
                            )
                        } else {
                            (
                                &pi_lower * &coefficient * &scale,
                                &pi_upper * &coefficient * scale,
                            )
                        };
                        assert!(
                            Q::from_integer(&sample - 1) <= lower
                                && Q::from_integer(&sample + 1) >= upper,
                            "{name}, n={n}, p={precision}, warm={warm}: {sample}"
                        );
                    }
                }
            }
        }
        // sqrt(2)/2 is strictly inside the domain and has asin = pi/4.
        // Its input never becomes a rational witness, so this also exercises
        // the numerical domain certificate and the generic reduction.
        for (name, reducer) in reducers {
            for sign in [-1, 1] {
                let coefficient = Q::new(
                    BigInt::from(if name == "acos" { 2 - sign } else { sign }),
                    4.into(),
                );
                for warm in [false, true] {
                    for precision in [-256_i32, -64, -16, 0, 1, 2, 8] {
                        let child = Computable::rational(Rational::new(2))
                            .sqrt()
                            .multiply(Computable::rational(Rational::fraction(sign, 2).unwrap()));
                        let value = restore_raw_node(reducer(child));
                        let sample = prepared_sample(value, warm, precision);
                        let scale = if precision < 0 {
                            Q::from_integer(BigInt::one() << precision.unsigned_abs())
                        } else {
                            Q::new(BigInt::one(), BigInt::one() << precision)
                        };
                        let (lower, upper) = if coefficient < Q::zero() {
                            (
                                &pi_upper * &coefficient * &scale,
                                &pi_lower * &coefficient * scale,
                            )
                        } else {
                            (
                                &pi_lower * &coefficient * &scale,
                                &pi_upper * &coefficient * scale,
                            )
                        };
                        assert!(
                            Q::from_integer(&sample - 1) <= lower
                                && Q::from_integer(&sample + 1) >= upper,
                            "{name}, sign={sign}, p={precision}, warm={warm}: {sample}"
                        );
                    }
                }
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_atanh_endpoints_and_large_coarse_values() {
        for reducer in [Approximation::AtanhDirect, Approximation::PrescaledAtanh] {
            for n in [-1, 1] {
                for precision in [-64, 0, 1, Precision::MAX] {
                    let half = Computable::rational(Rational::fraction(n, 2).unwrap());
                    let value =
                        restore_raw_node(reducer(raw_node(Approximation::Add(half.clone(), half))));
                    assert_eq!(value.approx(precision), Err(Problem::NotANumber));
                    assert!(value.internal.cache_snapshot().is_none());
                }
                for warm in [false, true] {
                    for precision in [1, 2, 3, Precision::MAX] {
                        let half = Computable::rational(Rational::fraction(63 * n, 128).unwrap());
                        let value = restore_raw_node(reducer(raw_node(Approximation::Add(
                            half.clone(),
                            half,
                        ))));
                        let sample = prepared_sample(value, warm, precision) * n;
                        if precision == Precision::MAX {
                            assert!(sample.is_zero());
                        } else {
                            // atanh(63/64)=ln(127)/2 is in (2,7/2), since 2<e<3.
                            let scale = BigInt::one() << precision;
                            assert!((&sample + 1) * &scale > BigInt::from(2));
                            assert!((&sample - 1) * scale * 2 < BigInt::from(7));
                        }
                    }
                }
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_nonrational_domains_and_invalid_children() {
        let reducers: [(&str, Reducer); 5] = [
            ("asin", Approximation::AsinDeferred),
            ("asin series", Approximation::PrescaledAsin),
            ("acos", Approximation::AcosPositive),
            ("atanh", Approximation::AtanhDirect),
            ("atanh series", Approximation::PrescaledAtanh),
        ];
        for (name, reducer) in reducers {
            for precision in [-64, 1, Precision::MAX] {
                for child in [
                    Computable::pi(),
                    Computable::pi().negate(),
                    Computable::rational(Rational::new(2)).sqrt(),
                ] {
                    let value = restore_raw_node(reducer(child));
                    assert_eq!(
                        value.approx(precision),
                        Err(Problem::NotANumber),
                        "{name}, p={precision}"
                    );
                    assert!(value.internal.cache_snapshot().is_none());
                }
                // Cancelling two undefined operands must not become a rational
                // zero witness. The failure belongs to the child square root.
                let invalid = Computable::rational(Rational::new(-1)).sqrt();
                let child = raw_node(Approximation::Add(
                    invalid.clone(),
                    raw_node(Approximation::Negate(invalid)),
                ));
                let value = restore_raw_node(reducer(child));
                assert_eq!(
                    value.approx(precision),
                    Err(Problem::SqrtNegative),
                    "{name}, p={precision}"
                );
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn generic_inverse_valid_facts_and_domain_cancellation() {
        let reducers: [(&str, Reducer); 4] = [
            ("asin", Approximation::AsinDeferred),
            ("asin series", Approximation::PrescaledAsin),
            ("atanh", Approximation::AtanhDirect),
            ("atanh series", Approximation::PrescaledAtanh),
        ];
        for (name, reducer) in reducers {
            for n in [-1, 0, 1] {
                let quarter = Computable::rational(Rational::fraction(n, 4).unwrap());
                let mut value = restore_raw_node(reducer(raw_node(Approximation::Add(
                    quarter.clone(),
                    quarter,
                ))));
                value.abort(Arc::new(AtomicBool::new(true)));
                let facts = value.structural_facts();
                assert_eq!(
                    facts.sign,
                    Some(public_sign(BigInt::from(n).sign())),
                    "{name}, n={n}"
                );
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        for reducer in [
            Approximation::AsinDeferred,
            Approximation::PrescaledAsin,
            Approximation::AcosPositive,
            Approximation::AtanhDirect,
            Approximation::PrescaledAtanh,
        ] {
            let mut completed = false;
            for stop_after in 0..512 {
                let child = Computable::pi()
                    .multiply(Computable::rational(Rational::fraction(1, 8).unwrap()));
                let value = restore_raw_node(reducer(child));
                let signal = Arc::new(AtomicBool::new(false));
                CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                let result = value.approx_signal(&Some(signal.clone()), -64);
                let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                assert!(!signal.load(Ordering::Relaxed));
                if !observed {
                    assert!(result.is_ok(), "{result:?}");
                    completed = true;
                    break;
                }
                assert_eq!(result, Err(Problem::Exhausted));
                assert!(value.internal.cache_snapshot().is_none());
                assert!(value.approx(-64).is_ok());
            }
            assert!(completed, "inverse-domain poll sweep did not finish");
        }
    }

    fn raw_node(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Valid(BoundInfo::Unknown),
                ExactSignCache::Unknown,
            )),
            signal: None,
        }
    }

    #[test]
    fn explicit_signal_controls_every_deferred_reduction() {
        let reducers: [(&str, Reducer); 6] = [
            ("sin", Approximation::SinDeferred),
            ("cos", Approximation::CosDeferred),
            ("tan", Approximation::TanDeferred),
            ("ln", Approximation::LnDeferred),
            ("atan", Approximation::AtanDeferred),
            ("exp", Approximation::PrescaledExp),
        ];
        let operand = || raw_node(Approximation::Add(Computable::pi(), Computable::e()));
        let mut failures = Vec::new();
        for (name, reduce) in reducers {
            let reference = raw_node(reduce(operand())).approx(-64).unwrap();
            for explicit_signal in [None, Some(Arc::new(AtomicBool::new(false)))] {
                let mut input = operand();
                input.abort(Arc::new(AtomicBool::new(true)));
                let value = raw_node(reduce(input));
                let result = value.approx_signal(&explicit_signal, -64);
                // Independent successful reductions may choose adjacent integers.
                if !matches!(&result, Ok(sample) if (sample - &reference).magnitude() <= &BigUint::from(2_u8))
                {
                    failures.push(format!(
                        "{name}, explicit Some={}: {result:?}",
                        explicit_signal.is_some()
                    ));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn every_observed_reduction_stop_reaches_the_root() {
        let reducers: [(&str, Reducer); 9] = [
            ("sin", Approximation::SinDeferred),
            ("cos", Approximation::CosDeferred),
            ("tan", Approximation::TanDeferred),
            ("ln", Approximation::LnDeferred),
            ("atan", Approximation::AtanDeferred),
            ("exp", Approximation::PrescaledExp),
            ("ln1p", Approximation::PrescaledLn),
            ("asinh", Approximation::AsinhDirect),
            ("acosh", Approximation::AcoshDirect),
        ];
        let mut failures = Vec::new();
        for (name, reduce) in reducers {
            let mut completed = false;
            for stop_after in 0..512 {
                let input = raw_node(Approximation::Add(Computable::pi(), Computable::e()));
                let value = raw_node(reduce(input));
                let signal = Arc::new(AtomicBool::new(false));
                CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(stop_after)));
                let result = value.approx_signal(&Some(signal.clone()), -64);
                let observed = CLEAR_STOP_AFTER_POLLS.with(|polls| polls.take().is_none());
                assert!(!signal.load(Ordering::Relaxed));
                if !observed {
                    assert!(result.is_ok(), "{name}: {result:?}");
                    completed = true;
                    break;
                }
                if result != Err(Problem::Exhausted) || value.internal.cache_snapshot().is_some() {
                    failures.push(format!("{name}, poll {stop_after}: {result:?}"));
                }
                assert!(value.approx(-64).is_ok(), "{name} must remain retryable");
            }
            assert!(completed, "{name} did not finish within the poll sweep");
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[test]
    fn series_domain_probe_preserves_observed_cancellation() {
        let input = raw_node(Approximation::Add(
            Computable::rational(Rational::fraction(1, 16).unwrap()),
            Computable::zero(),
        ));
        let value = raw_node(Approximation::PrescaledAtanh(input));
        let signal = Arc::new(AtomicBool::new(false));
        // Root, dispatcher and kernel entry polls pass. The domain probe
        // observes cancellation, after which the external flag is cleared.
        CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(3)));
        let result = value.approx_signal(&Some(signal.clone()), 1);
        assert!(CLEAR_STOP_AFTER_POLLS.with(|polls| polls.get().is_none()));
        assert!(!signal.load(Ordering::Relaxed));
        assert_eq!(result, Err(Problem::Exhausted));
        assert!(value.internal.cache_snapshot().is_none());
        assert_eq!(value.approx(1), Ok(BigInt::zero()));
    }

    #[test]
    fn coarse_tangent_preserves_the_same_enclosure_with_cold_and_warm_caches() {
        for negative in [false, true] {
            for warm in [false, true] {
                for precision in [1, 2, 3, 4] {
                    let argument = Rational::fraction(if negative { -3 } else { 3 }, 2).unwrap();
                    let value = Computable::rational(argument).tan();
                    let mut sample = prepared_sample(value, warm, precision);
                    if negative {
                        sample = -sample;
                    }
                    // Alternating sine/cosine series give independent bounds:
                    // 120/11 < tan(3/2) < 5124/359. Include the full one-unit
                    // approximation enclosure at each requested scale.
                    let scale = BigInt::one() << precision;
                    assert!(
                        (&sample + 1) * 11 * &scale > BigInt::from(120),
                        "negative={negative}, warm={warm}, p={precision}: {sample}"
                    );
                    assert!((&sample - 1) * 359 * scale < BigInt::from(5124));
                }
            }
        }
    }

    fn prepared_sample(value: Computable, warm: bool, precision: Precision) -> BigInt {
        assert!(value.internal.cache_snapshot().is_none());
        if warm {
            value.approx(-64).unwrap();
        }
        value.approx(precision).unwrap()
    }

    #[cfg(feature = "serde")]
    fn restore_raw_node(kind: Approximation) -> Computable {
        let serialized = serde_json::to_string(&raw_node(kind)).unwrap();
        serde_json::from_str(&serialized).unwrap()
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_private_nodes_require_certificates_for_structural_facts() {
        let cases = [
            (
                "cos(3)",
                Approximation::PrescaledCosRational(Rational::new(3)),
                Some(Sign::Minus),
            ),
            (
                "sin(4)",
                Approximation::PrescaledSinRational(Rational::new(4)),
                Some(Sign::Minus),
            ),
            (
                "sinc(4)",
                Approximation::SincSmall(Computable::rational(Rational::new(4))),
                Some(Sign::Minus),
            ),
            (
                "sinc(pi)",
                Approximation::SincSmall(Computable::pi()),
                Some(Sign::NoSign),
            ),
            (
                "cosc(2*pi)",
                Approximation::CoscSmall(Computable::pi().shift_left(1)),
                Some(Sign::NoSign),
            ),
            (
                "acosh(1)",
                Approximation::AcoshDirect(Computable::one()),
                Some(Sign::NoSign),
            ),
            (
                "atan(1/0)",
                Approximation::IntegralAtan(BigInt::zero()),
                None,
            ),
        ];
        assert_serialized_fact_cases(cases);
    }

    #[cfg(feature = "serde")]
    fn assert_serialized_fact_cases(
        cases: impl IntoIterator<Item = (&'static str, Approximation, Option<Sign>)>,
    ) {
        let mut failures = Vec::new();
        for (name, kind, expected) in cases {
            // Each API starts with fresh facts, so one incorrect inference
            // cannot mask a separate path by warming its cache first.
            let bound_value = restore_raw_node(kind.clone());
            let bound_sign = bound_value.cheap_bound().known_sign();
            if bound_sign.is_some() && bound_sign != expected {
                failures.push(format!("{name}, bound: {bound_sign:?}"));
            }
            let sign_value = restore_raw_node(kind.clone());
            let exact_sign = sign_value.exact_sign();
            if exact_sign.is_some() && exact_sign != expected {
                failures.push(format!("{name}, sign: {exact_sign:?}"));
            }
            let facts_value = restore_raw_node(kind);
            let facts = facts_value.structural_facts();
            let facts_sign = facts.sign.map(private_sign);
            let invalid_zero = match expected {
                None => facts.zero != ZeroKnowledge::Unknown,
                Some(Sign::NoSign) => facts.zero == ZeroKnowledge::NonZero,
                Some(_) => facts.zero == ZeroKnowledge::Zero,
            };
            if (facts_sign.is_some() && facts_sign != expected) || invalid_zero {
                failures.push(format!("{name}, facts: {facts:?}"));
            }
            for value in [bound_value, sign_value, facts_value] {
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_rational_kernel_facts_check_domains_and_endpoints() {
        let negative_half = Rational::fraction(-1, 2).unwrap();
        assert_serialized_fact_cases([
            (
                "cos(pi/2)",
                Approximation::PrescaledCosHalfPiMinusRational(Rational::zero()),
                Some(Sign::NoSign),
            ),
            (
                "sin(-1/2) via complement",
                Approximation::PrescaledCosHalfPiMinusRational(negative_half.clone()),
                Some(Sign::Minus),
            ),
            (
                "sin(4) via complement",
                Approximation::PrescaledCosHalfPiMinusRational(Rational::new(4)),
                Some(Sign::Minus),
            ),
            (
                "cos(2) via complement",
                Approximation::PrescaledSinHalfPiMinusRational(Rational::new(2)),
                Some(Sign::Minus),
            ),
            (
                "cot(pi/2)",
                Approximation::PrescaledCotHalfPiMinusRational(Rational::zero()),
                Some(Sign::NoSign),
            ),
            (
                "tan(-1/2) via complement",
                Approximation::PrescaledCotHalfPiMinusRational(negative_half),
                Some(Sign::Minus),
            ),
            (
                "tan(2) via complement",
                Approximation::PrescaledCotHalfPiMinusRational(Rational::new(2)),
                Some(Sign::Minus),
            ),
            (
                "asin(2)",
                Approximation::AsinRational(Rational::new(2)),
                None,
            ),
            (
                "asin(-2)",
                Approximation::AsinRational(Rational::new(-2)),
                None,
            ),
            (
                "atanh(1)",
                Approximation::AtanhRational(Rational::one()),
                None,
            ),
            (
                "atanh(-1)",
                Approximation::AtanhRational(Rational::new(-1)),
                None,
            ),
            (
                "atanh(2)",
                Approximation::AtanhRational(Rational::new(2)),
                None,
            ),
            (
                "atanh(-2)",
                Approximation::AtanhRational(Rational::new(-2)),
                None,
            ),
            (
                "acos(1)",
                Approximation::AcosPositiveRational(Rational::one()),
                Some(Sign::NoSign),
            ),
            (
                "acos(2)",
                Approximation::AcosPositiveRational(Rational::new(2)),
                None,
            ),
            (
                "acos(-2)",
                Approximation::AcosPositiveRational(Rational::new(-2)),
                None,
            ),
            (
                "pi-acos(-1)",
                Approximation::AcosNegativeRational(Rational::new(-1)),
                Some(Sign::NoSign),
            ),
            (
                "pi-acos(2)",
                Approximation::AcosNegativeRational(Rational::new(2)),
                None,
            ),
            (
                "pi-acos(-2)",
                Approximation::AcosNegativeRational(Rational::new(-2)),
                None,
            ),
        ]);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_rational_inverse_domains_precede_coarse_shortcuts() {
        let mut failures = Vec::new();
        for kind in [
            Approximation::AsinRational(Rational::new(2)),
            Approximation::AsinRational(Rational::new(-2)),
            Approximation::AtanhRational(Rational::new(1)),
            Approximation::AtanhRational(Rational::new(-1)),
            Approximation::AtanhRational(Rational::new(2)),
            Approximation::AtanhRational(Rational::new(-2)),
            Approximation::AcosPositiveRational(Rational::new(2)),
            Approximation::AcosPositiveRational(Rational::new(-2)),
            Approximation::AcosNegativeRational(Rational::new(2)),
            Approximation::AcosNegativeRational(Rational::new(-2)),
        ] {
            let name = format!("{kind:?}");
            for precision in [1, 0, -64, 8, Precision::MAX] {
                let value = restore_raw_node(kind.clone());
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    value.approx(precision)
                }));
                if !matches!(result, Ok(Err(Problem::NotANumber))) {
                    failures.push(format!("{name}, p={precision}: {result:?}"));
                }
                if value.internal.cache_snapshot().is_some() {
                    failures.push(format!(
                        "{name}, p={precision}: cached an invalid-domain result"
                    ));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_rational_boundaries_keep_certificates_and_extreme_coarse_enclosures() {
        let mut cases = Vec::new();
        for n in [-1, 0, 1] {
            let sign = match n {
                -1 => RealSign::Negative,
                0 => RealSign::Zero,
                _ => RealSign::Positive,
            };
            let half = Rational::fraction(n, 2).unwrap();
            cases.push((
                format!("asin({n})"),
                Approximation::AsinRational(Rational::new(n)),
                sign,
            ));
            cases.push((
                format!("atanh({n}/2)"),
                Approximation::AtanhRational(half.clone()),
                sign,
            ));
            cases.push((
                format!("acos({n})"),
                Approximation::AcosPositiveRational(Rational::new(n)),
                if n == 1 {
                    RealSign::Zero
                } else {
                    RealSign::Positive
                },
            ));
            cases.push((
                format!("pi-acos({n})"),
                Approximation::AcosNegativeRational(Rational::new(n)),
                if n == -1 {
                    RealSign::Zero
                } else {
                    RealSign::Positive
                },
            ));
            cases.push((
                format!("sin({n}/2) via complement"),
                Approximation::PrescaledCosHalfPiMinusRational(half.clone()),
                sign,
            ));
            cases.push((
                format!("cos({n}/2) via complement"),
                Approximation::PrescaledSinHalfPiMinusRational(half.clone()),
                RealSign::Positive,
            ));
            cases.push((
                format!("tan({n}/2) via complement"),
                Approximation::PrescaledCotHalfPiMinusRational(half),
                sign,
            ));
        }
        for n in [-63, 63] {
            cases.push((
                format!("atanh({n}/64)"),
                Approximation::AtanhRational(Rational::fraction(n, 64).unwrap()),
                if n < 0 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                },
            ));
        }
        let mut failures = Vec::new();
        for (name, kind, sign) in cases {
            let mut fact_value = restore_raw_node(kind.clone());
            fact_value.abort(Arc::new(AtomicBool::new(true)));
            let facts = fact_value.structural_facts();
            if facts.sign != Some(sign)
                || facts.zero
                    != if sign == RealSign::Zero {
                        ZeroKnowledge::Zero
                    } else {
                        ZeroKnowledge::NonZero
                    }
            {
                failures.push(format!("{name}: {facts:?}"));
            }
            assert!(fact_value.internal.cache_snapshot().is_none());
            let value = restore_raw_node(kind);
            let result = value.approx(Precision::MAX);
            // Every value in this matrix has magnitude below four. At this
            // scale, zero or a one-unit endpoint with the correct sign is valid.
            let valid = match &result {
                Ok(sample) if sample.is_zero() => true,
                Ok(sample) if sample.magnitude().is_one() => match sign {
                    RealSign::Positive => sample.sign() == Sign::Plus,
                    RealSign::Negative => sample.sign() == Sign::Minus,
                    RealSign::Zero => true,
                },
                _ => false,
            };
            if !valid {
                failures.push(format!("{name}, p=MAX: {result:?}"));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    fn independent_pi_bounds() -> (num::rational::BigRational, num::rational::BigRational) {
        use num::rational::BigRational as Q;

        fn atan_inverse_bounds(denominator: u32) -> (Q, Q) {
            let square = BigInt::from(denominator) * denominator;
            let mut power = BigInt::from(denominator);
            let mut lower = Q::zero();
            for index in 0..64_u32 {
                let term = Q::new(BigInt::one(), &power * (2 * index + 1));
                lower += if index % 2 == 0 { term } else { -term };
                power *= &square;
            }
            // The even number of alternating terms ends below atan(1/d).
            let upper = lower.clone() + Q::new(BigInt::one(), power * 129_u32);
            (lower, upper)
        }

        // Machin's identity uses rational arithmetic only: tan(4 atan(1/5)
        // - atan(1/239)) = 1, with the angle in the first quadrant.
        let (lower5, upper5) = atan_inverse_bounds(5);
        let (lower239, upper239) = atan_inverse_bounds(239);
        let pi_lower = lower5 * BigInt::from(16) - upper239 * BigInt::from(4);
        let pi_upper = upper5 * BigInt::from(16) - lower239 * BigInt::from(4);
        assert!(pi_lower > Q::from_integer(BigInt::from(3)));
        assert!(pi_upper < Q::from_integer(BigInt::from(4)));
        assert!(&pi_upper - &pi_lower < Q::new(BigInt::one(), BigInt::one() << 280));
        (pi_lower, pi_upper)
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_acos_rational_enclosures_match_independent_pi_bounds() {
        use num::rational::BigRational as Q;
        let (pi_lower, pi_upper) = independent_pi_bounds();
        let mut failures = Vec::new();
        for (numerator, coefficient_num, coefficient_den) in
            [(-2, 1, 1), (-1, 2, 3), (0, 1, 2), (1, 1, 3), (2, 0, 1)]
        {
            for reflected in [false, true] {
                let coefficient =
                    Q::new(BigInt::from(coefficient_num), BigInt::from(coefficient_den));
                let coefficient = if reflected {
                    Q::one() - coefficient
                } else {
                    coefficient
                };
                for precision in [-256_i32, -64, -16, 0, 1, 2, 8] {
                    let r = Rational::fraction(numerator, 2).unwrap();
                    let kind = if reflected {
                        Approximation::AcosNegativeRational(r)
                    } else {
                        Approximation::AcosPositiveRational(r)
                    };
                    let value = restore_raw_node(kind);
                    let sample = value.approx(precision).unwrap();
                    let scale = if precision < 0 {
                        Q::from_integer(BigInt::one() << precision.unsigned_abs())
                    } else {
                        Q::new(BigInt::one(), BigInt::one() << precision)
                    };
                    let lower = &pi_lower * &coefficient * &scale;
                    let upper = &pi_upper * &coefficient * scale;
                    if Q::from_integer(&sample - 1) > lower || Q::from_integer(&sample + 1) < upper
                    {
                        failures.push(format!(
                            "r={numerator}/2, reflected={reflected}, p={precision}: {sample}"
                        ));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_atanh_large_residual_keeps_coarse_enclosures() {
        let mut failures = Vec::new();
        for negative in [false, true] {
            for warm in [false, true] {
                for precision in [1, 2, 3] {
                    let r = Rational::fraction(if negative { -63 } else { 63 }, 64).unwrap();
                    let value = restore_raw_node(Approximation::AtanhRational(r));
                    let mut sample = prepared_sample(value, warm, precision);
                    if negative {
                        sample = -sample;
                    }
                    // atanh(63/64) = ln(127)/2. The elementary 2 < e < 3
                    // gives 2 < atanh(63/64) < 7/2, independently of this kernel.
                    let scale = BigInt::one() << precision;
                    if (&sample + 1) * &scale <= BigInt::from(2)
                        || (&sample - 1) * &scale * 2 >= BigInt::from(7)
                    {
                        failures.push(format!(
                            "negative={negative}, warm={warm}, p={precision}: {sample}"
                        ));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_private_nodes_recover_valid_domain_certificates() {
        let half = Rational::fraction(1, 2).unwrap();
        for (kind, expected) in [
            (
                Approximation::PrescaledCosRational(half.clone()),
                RealSign::Positive,
            ),
            (
                Approximation::PrescaledSinRational(half.clone().neg()),
                RealSign::Negative,
            ),
            (
                Approximation::PrescaledTanRational(half.clone()),
                RealSign::Positive,
            ),
            (
                Approximation::SincSmall(Computable::rational(half.clone())),
                RealSign::Positive,
            ),
            (
                Approximation::CoscSmall(Computable::rational(half.neg())),
                RealSign::Positive,
            ),
            (
                Approximation::SincSmall(Computable::zero()),
                RealSign::Positive,
            ),
            (
                Approximation::CoscSmall(Computable::zero()),
                RealSign::Positive,
            ),
            (
                Approximation::AcoshDirect(Computable::one()),
                RealSign::Zero,
            ),
            (
                Approximation::AcoshDirect(Computable::rational(Rational::new(2))),
                RealSign::Positive,
            ),
            (
                Approximation::IntegralAtan(BigInt::from(2)),
                RealSign::Positive,
            ),
            (
                Approximation::IntegralAtan(BigInt::from(-2)),
                RealSign::Negative,
            ),
        ] {
            let mut value = restore_raw_node(kind);
            value.abort(Arc::new(AtomicBool::new(true)));
            let facts = value.structural_facts();
            assert_eq!(facts.sign, Some(expected));
            assert_eq!(
                facts.zero,
                if expected == RealSign::Zero {
                    ZeroKnowledge::Zero
                } else {
                    ZeroKnowledge::NonZero
                }
            );
            assert!(value.internal.cache_snapshot().is_none());
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_integral_atan_zero_returns_typed_failure() {
        for precision in [0, -64, 8] {
            let value = restore_raw_node(Approximation::IntegralAtan(BigInt::zero()));
            assert_eq!(value.approx(precision), Err(Problem::DivideByZero));
            assert!(value.internal.cache_snapshot().is_none());
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_unit_integral_atan_has_a_convergent_reduction() {
        for denominator in [1, -1] {
            for precision in [0, -64, -256] {
                let value =
                    restore_raw_node(Approximation::IntegralAtan(BigInt::from(denominator)));
                let mut sample = value.approx(precision).unwrap();
                if denominator < 0 {
                    sample = -sample;
                }
                // atan(1) = pi/4 and the independent geometric bound 3 < pi < 4.
                let scale = BigInt::one() << precision.unsigned_abs();
                assert!((&sample + 1) * 4 > &scale * 3);
                assert!(&sample - 1 < scale);
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_cosine_coarse_shortcut_accepts_negative_results() {
        for kind in [
            Approximation::PrescaledCos(Computable::pi()),
            Approximation::PrescaledCosRational(Rational::new(3)),
        ] {
            for precision in [1, 2, 8, Precision::MAX] {
                let sample = prepared_sample(restore_raw_node(kind.clone()), false, precision);
                // Both cosine arguments lie in (pi/2, 3*pi/2). The scaled
                // negative result's enclosure contains only -1 and 0.
                assert!(
                    sample == BigInt::from(-1) || sample.is_zero(),
                    "p={precision}: {sample}"
                );
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn every_serialized_tangent_quotient_respects_coarse_enclosures() {
        let argument = Rational::fraction(3, 2).unwrap();
        for kind in [
            Approximation::PrescaledTan(Computable::rational(argument.clone())),
            Approximation::PrescaledTanRational(argument.clone()),
            Approximation::TanLargeRational(argument.clone()),
            Approximation::PrescaledCotHalfPiMinusRational(argument),
        ] {
            for warm in [false, true] {
                for precision in [1, 2, 3, 4, Precision::MAX] {
                    let sample = prepared_sample(restore_raw_node(kind.clone()), warm, precision);
                    if precision == Precision::MAX {
                        assert!(sample.is_zero());
                        continue;
                    }
                    let scale = BigInt::one() << precision;
                    assert!((&sample + 1) * 11 * &scale > BigInt::from(120));
                    assert!((&sample - 1) * 359 * scale < BigInt::from(5124));
                }
            }
        }
        for warm in [false, true] {
            for precision in [1, 2, 3, 4, Precision::MAX] {
                let value = restore_raw_node(Approximation::PrescaledCot(Computable::rational(
                    Rational::fraction(1, 16).unwrap(),
                )));
                let sample = prepared_sample(value, warm, precision);
                if precision == Precision::MAX {
                    assert!(sample.is_zero());
                    continue;
                }
                let scale = BigInt::one() << precision;
                // cos(x) > 1-x^2/2 and sin(x) < x give cot(1/16) > 511/32;
                // tan(x) > x gives the upper bound cot(1/16) < 16.
                assert!((&sample + 1) * 32 * &scale > BigInt::from(511));
                assert!((&sample - 1) * scale < BigInt::from(16));
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_rational_logarithms_reject_invalid_residuals() {
        for residual in [-1, -2, -100] {
            for kind in [
                Approximation::PrescaledLnRational(Rational::new(residual)),
                Approximation::BinaryScaledLnRational {
                    residual: Rational::new(residual),
                    shift: 0,
                },
                Approximation::BinaryScaledLnRational {
                    residual: Rational::new(residual),
                    shift: 12,
                },
                Approximation::BinaryScaledLnRational {
                    residual: Rational::new(residual),
                    shift: -12,
                },
            ] {
                let value = restore_raw_node(kind);
                for precision in [0, -64, 8] {
                    assert_eq!(value.approx(precision), Err(Problem::NotANumber));
                    assert!(value.internal.cache_snapshot().is_none());
                }
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serialized_positive_nth_roots_reject_negative_arguments() {
        for degree in [3, 4, 9] {
            let value = restore_raw_node(Approximation::NthRoot(
                Computable::rational(Rational::new(-8)),
                degree,
            ));
            for precision in [0, -64, 8] {
                assert_eq!(value.approx(precision), Err(Problem::NotANumber));
                assert!(value.internal.cache_snapshot().is_none());
            }
        }
    }

    #[test]
    fn observed_cancellation_survives_a_cleared_external_flag() {
        let value = Computable::prescaled_atan(BigInt::from(5));
        let signal = Arc::new(AtomicBool::new(false));
        // Two entry polls pass; the series observes cancellation on its first
        // poll, after which the caller's flag has already been cleared.
        CLEAR_STOP_AFTER_POLLS.with(|polls| polls.set(Some(2)));
        assert_eq!(
            value.approx_signal(&Some(signal.clone()), -128),
            Err(Problem::Exhausted)
        );
        assert!(!signal.load(Ordering::Relaxed));
        assert!(value.internal.cache_snapshot().is_none());
        let retry = value.approx(-128).unwrap();
        // Alternating atan series: 1/5 - 1/(3*5^3) < atan(1/5) < 1/5.
        let scale = BigInt::one() << 128;
        assert!((&retry + 1) * 375 > &scale * 74);
        assert!((&retry - 1) * 5 < scale);
    }

    #[test]
    fn extreme_precision_cannot_poison_the_shared_pi_cache() {
        for warm in [false, true] {
            let pi = Computable::pi();
            if warm {
                pi.approx(-256).unwrap();
            }
            assert_eq!(pi.approx(Precision::MIN), Err(Problem::OutOfRange));
            for precision in [Precision::MIN + 1, Precision::MIN + 2, Precision::MIN + 16] {
                assert_eq!(pi.approx(precision), Err(Problem::Exhausted));
            }
            for precision in [0, -64, -256] {
                let approximation = pi.approx(precision).unwrap();
                let scale = BigInt::one() << precision.unsigned_abs();
                assert!(&approximation + 1 > &scale * 3);
                assert!(&approximation - 1 < &scale * 4);
            }
            let [lower, upper] = Real::pi().certified_rational_interval(-64).unwrap();
            assert!(lower > Rational::new(3));
            assert!(upper < Rational::new(4));
        }
    }

    #[test]
    fn adjacent_precision_failures_remain_retryable_for_nonconstant_nodes() {
        let cosine = Computable::one().cos();
        let product = Computable::pi().multiply(Computable::e());
        for value in [&cosine, &product] {
            for precision in [Precision::MIN + 1, Precision::MIN + 2, Precision::MIN + 16] {
                assert_eq!(value.approx(precision), Err(Problem::Exhausted));
                assert!(value.internal.cache_snapshot().is_none());
            }
            assert!(value.approx(-64).unwrap() > BigInt::one());
        }
    }

    #[test]
    fn exact_zero_and_coarse_requests_do_not_need_large_allocations() {
        assert_eq!(
            Computable::zero().approx(Precision::MIN),
            Ok(BigInt::zero())
        );
        for value in [Computable::one(), Computable::pi(), Computable::e()] {
            assert_eq!(value.approx(Precision::MAX), Ok(BigInt::zero()));
        }
        let square = Approximation::Square(Computable::zero());
        assert_eq!(square.approximate(&None, -64), Ok(BigInt::zero()));
    }

    #[test]
    fn negative_square_roots_do_not_publish_positive_facts() {
        for radicand in [
            Computable::rational(Rational::new(-1)),
            Computable::rational(Rational::new(-4)),
            Computable::pi().negate(),
        ] {
            let root = radicand.sqrt();
            assert_eq!(root.exact_sign(), None);
            assert_eq!(root.zero_status(), ZeroKnowledge::Unknown);
            for precision in [0, -64] {
                assert_eq!(root.approx(precision), Err(Problem::SqrtNegative));
                assert!(root.internal.cache_snapshot().is_none());
                assert_eq!(root.exact_sign(), None);
            }
        }
    }

    #[test]
    fn square_root_rejects_negative_radicands_discovered_during_refinement() {
        // The alternating sine series proves sin(1/2) < 1/2. This expression
        // has no retained structural sign. Magnitude refinement must not send
        // a negative integer sample to BigInt::sqrt.
        for precision in [-16, -64, -256] {
            let half = Computable::rational(Rational::fraction(1, 2).unwrap());
            let radicand = half.clone().sin().add(half.negate());
            assert_eq!(radicand.exact_sign(), None);
            let root = radicand.sqrt();
            assert_eq!(root.approx(precision), Err(Problem::SqrtNegative));
            assert!(root.internal.cache_snapshot().is_none());
        }
    }

    #[test]
    fn square_root_accepts_every_valid_zero_cache_endpoint() {
        for precision in [0, -16, -64] {
            for sample in [-1, 0, 1] {
                // An opaque exact zero with each allowed one-unit rounding.
                // Retain Unknown facts so the kernel must inspect the sample.
                let input = Computable {
                    internal: Arc::new(Node::new(
                        Approximation::Add(Computable::zero(), Computable::zero()),
                        BoundCache::Valid(BoundInfo::Unknown),
                        ExactSignCache::Unknown,
                    )),
                    signal: None,
                };
                input
                    .internal
                    .store_cache_value(2 * precision - 1, BigInt::from(sample));
                let root = Approximation::Sqrt(input);
                assert_eq!(root.approximate(&None, precision), Ok(BigInt::zero()));
            }
        }
    }
}
