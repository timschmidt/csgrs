use hyperreal::{Computable, Rational, RealSign, ZeroKnowledge};
use num::{BigInt, One, Signed};
use rug::Float;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn probability(n: i64, d: u64) -> Computable {
    Computable::rational(Rational::fraction(n, d).unwrap())
}

fn mpfr_cdf(mut x: Float) -> Float {
    x /= Float::with_val(384, 2).sqrt();
    (-x).erfc() / 2
}

fn check_quantile(value: &Computable, n: i64, d: u64, precision: i32) {
    let actual = value.approx(precision);
    let x = Float::with_val(384, Float::parse(actual.to_string()).unwrap());
    let mut lower = Float::with_val(384, &x - 1);
    let mut upper = Float::with_val(384, &x + 1);
    lower <<= precision;
    upper <<= precision;
    let p = Float::with_val(384, n) / Float::with_val(384, d);
    // MPFR's erfc is independent of Hyperreal's erf series and Newton step.
    // The CDF is strictly increasing, so these enclose its unique inverse.
    assert!(
        mpfr_cdf(lower) <= p,
        "lower bound: {n}/{d}, {precision}, {actual}"
    );
    assert!(
        mpfr_cdf(upper) >= p,
        "upper bound: {n}/{d}, {precision}, {actual}"
    );
}

#[test]
fn native_half_probability_is_exact_for_every_seed() {
    for (seed, precision) in [
        (0, -16),
        (6_553_600, -16),
        (-6_553_600, -16),
        (1, i32::MIN),
        (1, i32::MAX),
    ] {
        let value = Computable::normal_quantile(probability(1, 2), seed.into(), precision);
        assert_eq!(value.structural_facts().zero, ZeroKnowledge::Zero);
        for precision in [0, -96, 0] {
            assert_eq!(value.approx(precision), BigInt::from(0));
        }
    }
}

#[test]
fn bad_seeds_cannot_change_interior_or_tail_quantiles() {
    for (n, d) in [
        (1, 4),
        (3, 4),
        (1, 1_000),
        (999, 1_000),
        (1, 1_000_000_000_000_000_000),
        (999_999_999_999_999_999, 1_000_000_000_000_000_000),
    ] {
        for (seed, seed_precision) in [
            (6_553_600, -16),
            (-6_553_600, -16),
            (0, 0),
            (1, i32::MIN),
            (1, i32::MAX),
        ] {
            let value = Computable::normal_quantile(probability(n, d), seed.into(), seed_precision);
            let values = {
                #[cfg(feature = "serde")]
                {
                    let json = serde_json::to_string(&value).unwrap();
                    let mut cbor = Vec::new();
                    ciborium::ser::into_writer(&value, &mut cbor).unwrap();
                    vec![
                        value,
                        serde_json::from_str(&json).unwrap(),
                        ciborium::de::from_reader(cbor.as_slice()).unwrap(),
                    ]
                }
                #[cfg(not(feature = "serde"))]
                {
                    vec![value]
                }
            };
            for value in values {
                #[cfg(feature = "serde")]
                let wire = serde_json::to_vec(&value).unwrap();
                let expected = if (n as u64) * 2 < d {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                for precision in [0, -80, 0] {
                    if let Some(sign) = value.structural_facts().sign {
                        assert_eq!(sign, expected);
                    }
                    check_quantile(&value, n, d, precision);
                }
                assert_eq!(value.structural_facts().sign, Some(expected));
                #[cfg(feature = "serde")]
                assert_eq!(serde_json::to_vec(&value).unwrap(), wire);
            }
        }
    }
}

#[test]
fn concurrent_quantile_queries_reuse_only_certified_values() {
    let value = Arc::new(Computable::normal_quantile(
        probability(3, 4),
        (-6_553_600).into(),
        -16,
    ));
    let threads: Vec<_> = (0..4)
        .map(|offset| {
            let value = value.clone();
            std::thread::spawn(move || {
                let precisions = [0, -96, -16, 0];
                for index in 0..precisions.len() {
                    check_quantile(
                        &value,
                        3,
                        4,
                        precisions[(index + offset) % precisions.len()],
                    );
                }
            })
        })
        .collect();
    for thread in threads {
        thread.join().unwrap();
    }
    assert_eq!(value.structural_facts().sign, Some(RealSign::Positive));
}

#[test]
fn cancelled_quantile_work_does_not_publish_a_seed_or_poison_recovery() {
    let stopped = Arc::new(AtomicBool::new(true));
    let mut value = Computable::normal_quantile(probability(3, 4), 6_553_600.into(), -16);
    value.abort(stopped.clone());
    let _ = value.approx(-80);
    assert_eq!(value.structural_facts().sign, None);
    assert_eq!(value.structural_facts().zero, ZeroKnowledge::Unknown);
    stopped.store(false, Ordering::Relaxed);
    check_quantile(&value, 3, 4, -80);
    assert!(value.approx(0).abs() <= BigInt::one());
}

#[test]
fn rational_cdf_composition_preserves_exact_values_and_tail_completion() {
    // Strict monotonicity gives inverse-CDF(CDF(x)) = x. The expected dyadic
    // values below are exact; evaluating the tiny tail probability is unnecessary.
    for (n, d) in [
        (-1000, 1),
        (1000, 1),
        (-1001, 8),
        (1001, 8),
        (-3, 4),
        (3, 4),
    ] {
        let p = probability(n, d).pnorm();
        for seed in [-6_553_600_i64, 0, 6_553_600] {
            let native = Computable::normal_quantile(p.clone(), seed.into(), -16);
            let values = {
                #[cfg(feature = "serde")]
                {
                    // Preserve an actual imported quantile node even when the
                    // native constructor can now return the exact rational.
                    let graph = serde_json::json!({"internal":{"NormalQuantile":{
                        "p":p, "seed":BigInt::from(seed), "seed_prec":-16
                    }}});
                    let mut cbor = Vec::new();
                    ciborium::ser::into_writer(&graph, &mut cbor).unwrap();
                    vec![
                        native,
                        serde_json::from_value(graph).unwrap(),
                        ciborium::de::from_reader(cbor.as_slice()).unwrap(),
                    ]
                }
                #[cfg(not(feature = "serde"))]
                {
                    vec![native]
                }
            };
            for value in values {
                #[cfg(feature = "serde")]
                let wire = serde_json::to_vec(&value).unwrap();
                let sign = if n < 0 {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                assert_eq!(value.structural_facts().sign, Some(sign));
                for precision in [0_i32, -96, 0] {
                    let actual = value.approx(precision);
                    let expected_numerator = BigInt::from(n) << precision.unsigned_abs();
                    let error = (actual * d - expected_numerator).abs();
                    if precision < 0 {
                        assert_eq!(error, BigInt::from(0));
                    } else {
                        assert!(error <= BigInt::from(d));
                    }
                }
                #[cfg(feature = "serde")]
                {
                    assert_eq!(serde_json::to_vec(&value).unwrap(), wire);
                    let wrapped = serde_json::json!({
                        "rational":Rational::new(-3),
                        "class":"Irrational", "computable":value
                    });
                    let restored = hyperreal::Real::from_json(&wrapped.to_string()).unwrap();
                    assert_eq!(
                        restored.exact_rational_normal_form(),
                        Some(Rational::fraction(-3 * n, d).unwrap())
                    );
                    assert_eq!(
                        restored.inverse_ref().unwrap().exact_rational_normal_form(),
                        Some(Rational::fraction(-3 * n, d).unwrap().inverse().unwrap())
                    );
                }
            }
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn altered_cdf_graph_must_not_borrow_the_original_inverse() {
    let mut graph = serde_json::to_value(probability(3, 4).pnorm()).unwrap();
    let offset = graph
        .pointer_mut("/internal/Offset/0/internal/Add/0")
        .unwrap();
    assert_eq!(*offset, serde_json::to_value(Computable::one()).unwrap());
    // Change (1 + erf(x/sqrt(2)))/2 to (9/8 + erf(x/sqrt(2)))/2.
    // The retained erf argument still suggests x, but it is no longer a proof.
    *offset = serde_json::to_value(probability(9, 8)).unwrap();
    let p: Computable = serde_json::from_value(graph).unwrap();
    let value = Computable::normal_quantile(p, 3.into(), -2);
    let actual = value.approx(-80);
    assert_ne!(actual, BigInt::from(3) << 78);
    let x = Float::with_val(384, Float::parse(actual.to_string()).unwrap());
    let mut lower = Float::with_val(384, &x - 1);
    let mut upper = Float::with_val(384, &x + 1);
    lower <<= -80;
    upper <<= -80;
    let p = mpfr_cdf(Float::with_val(384, 3) / 4) + Float::with_val(384, 1) / 16;
    assert!(mpfr_cdf(lower) <= p);
    assert!(mpfr_cdf(upper) >= p);
}

#[cfg(feature = "serde")]
#[test]
fn imported_half_quantiles_preserve_graphs_and_downstream_zero() {
    use hyperreal::{Problem, Real};
    use serde_json::json;
    for seed in [-6_553_600_i64, 0, 6_553_600] {
        let half = serde_json::to_value(probability(1, 2)).unwrap();
        let graph = json!({"internal":{"NormalQuantile":{
            "p":half, "seed":BigInt::from(seed), "seed_prec":-16
        }}});
        let value: Computable = serde_json::from_value(graph.clone()).unwrap();
        assert_eq!(value.structural_facts().zero, ZeroKnowledge::Zero);
        assert_eq!(serde_json::to_value(&value).unwrap(), graph);
        for scale in [1, -3] {
            let wire =
                json!({"rational":Rational::new(scale),"class":"Irrational","computable":graph});
            let mut bytes = Vec::new();
            ciborium::ser::into_writer(&wire, &mut bytes).unwrap();
            for restored in [
                Real::from_json(&wire.to_string()).unwrap(),
                Real::from_bytes(&bytes).unwrap(),
            ] {
                let json = restored.to_json();
                let cbor = restored.to_bytes();
                for precision in [0, -96, 0] {
                    assert_eq!(restored.structural_facts().zero, ZeroKnowledge::Zero);
                    assert!(restored.certified_rational_interval(precision).is_some());
                }
                assert_eq!(restored.to_json(), json);
                assert_eq!(restored.to_bytes(), cbor);
                assert_eq!(restored.inverse_ref(), Err(Problem::DivideByZero));
                assert_eq!(
                    (Real::one() + restored).exact_rational_normal_form(),
                    Some(Rational::one())
                );
            }
        }
    }
}
