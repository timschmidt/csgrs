#[cfg(all(test, feature = "serde"))]
mod log_range_probe_tests {
    use super::algebraic_leaf_tests::measure;
    use super::*;
    use crate::EvaluationBudget;
    use crate::computable::budget::BudgetScope;
    use rug::{Float, Integer, Rational as Q, float::Round};
    use std::time::Instant;

    fn log_bounds(twos: u32, threes: u32, multiplier: u32) -> (Q, Q) {
        let mut lo2 = Float::with_val(256, 2);
        let mut hi2 = lo2.clone();
        let mut lo3 = Float::with_val(256, 3);
        let mut hi3 = lo3.clone();
        lo2.ln_round(Round::Down);
        hi2.ln_round(Round::Up);
        lo3.ln_round(Round::Down);
        hi3.ln_round(Round::Up);
        let lo = lo2.to_rational().unwrap() * Integer::from(twos * multiplier)
            - hi3.to_rational().unwrap() * Integer::from(threes * multiplier);
        let hi = hi2.to_rational().unwrap() * Integer::from(twos * multiplier)
            - lo3.to_rational().unwrap() * Integer::from(threes * multiplier);
        (lo, hi)
    }

    fn check_bounds(sample: &BigInt, p: Precision, bounds: &(Q, Q)) {
        let unit = if p >= 0 {
            Q::from(Integer::from(1) << p)
        } else {
            Q::from((1, Integer::from(1) << -p))
        };
        let center = Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= bounds.0);
        assert!(Q::from(&center + &unit) >= bounds.1);
    }

    fn shared_setup() {
        let empty = EvaluationBudget::new(0);
        drop(BudgetScope::enter(&empty));
        // A public smooth-log expression warms the shared ln(2)/ln(3)
        // constants. Neither the measured root nor its children are reused.
        Computable::rational(Rational::fraction(4, 3).unwrap())
            .ln()
            .approx(-192)
            .unwrap();
    }

    fn stored_deferred(exponent: u32) -> Computable {
        let child = Computable::rational(
            Rational::from_bigint_fraction(BigInt::one() << exponent, BigUint::from(3_u32))
                .unwrap(),
        );
        assert!(matches!(
            child.internal.approximation,
            Approximation::Ratio(_)
        ));
        // Public deserialization accepts stored LnDeferred expressions. This
        // is distinct from an ordinary call to the public ln constructor.
        let encoded = serde_json::json!({"internal": {"LnDeferred": child}});
        let root: Computable = serde_json::from_value(encoded).unwrap();
        assert!(matches!(
            root.internal.approximation,
            Approximation::LnDeferred(_)
        ));
        root
    }

    fn public_inputs(minimum_exponent: u32) -> (u32, u32, Rational, Rational) {
        for exponent in minimum_exponent..minimum_exponent + 32 {
            let numerator = BigUint::one() << exponent;
            let mut threes = exponent * 63 / 100;
            let mut denominator = BigUint::from(3_u32).pow(threes);
            while &denominator * 3_u32 <= numerator {
                denominator *= 3_u32;
                threes += 1;
            }
            // Select 3/2 <= a = 2^k/3^m <= 2 using exact integers.
            // t=a^2 and x=(t-1/t)/2 then satisfy asinh(x)=ln(t),
            // with 0.9 < x < 2 on the public asinh ln1p branch.
            if &numerator * 2_u32 < &denominator * 3_u32 || numerator > &denominator * 2_u32 {
                continue;
            }
            let n2 = &numerator * &numerator;
            let d2 = &denominator * &denominator;
            let residual =
                Rational::from_bigint_fraction(BigInt::from(&n2 - &d2), d2.clone()).unwrap();
            let asinh_input = Rational::from_bigint_fraction(
                BigInt::from(&n2 * &n2 - &d2 * &d2),
                &n2 * &d2 * 2_u32,
            )
            .unwrap();
            return (exponent, threes, residual, asinh_input);
        }
        panic!("no exact public-range fixture found");
    }

    fn observe(
        route: &str,
        exponent: u32,
        state: &str,
        value: Computable,
        p: Precision,
        units: u64,
        bounds: &(Q, Q),
    ) {
        let shape = match &value.internal.approximation {
            Approximation::LnDeferred(_) => "deferred-ratio",
            Approximation::PrescaledLn(child) => match &child.internal.approximation {
                Approximation::Ratio(_) => "ln1p-ratio",
                Approximation::Int(_) => "ln1p-integer",
                _ => "ln1p-graph",
            },
            other => panic!("unexpected root shape: {other:?}"),
        };
        let preparation = match state {
            "cold" => None,
            "prefix" => Some(format!(
                "{:?}",
                value.approx_with_budget(8, &EvaluationBudget::new(2))
            )),
            "sign" => Some(format!(
                "{:?}",
                value.sign_until_with_budget(0, &EvaluationBudget::new(8))
            )),
            _ => unreachable!(),
        };
        let before = value.internal.cache_snapshot();
        let sign_before = value.immediate_sign();
        let finite_before = value.internal.facts.finite_real();
        let budget = EvaluationBudget::new(units);
        let started = Instant::now();
        let (answer, allocations) = measure(|| value.approx_with_budget(p, &budget));
        let elapsed_ns = started.elapsed().as_nanos();
        assert_eq!(budget.consumed() + budget.remaining(), units);
        let after = value.internal.cache_snapshot();
        eprintln!(
            "log range survey route={route}, exponent={exponent}, state={state}, shape={shape}, preparation={preparation:?}, p={p}, units={units}, answer={answer:?}, consumed={}, remaining={}, finite_before={finite_before}, finite_after={}, before={before:?}, after={after:?}, allocations={allocations:?}, elapsed_ns={elapsed_ns}",
            budget.consumed(),
            budget.remaining(),
            value.internal.facts.finite_real()
        );
        let zero = EvaluationBudget::new(0);
        match &answer {
            Ok(sample) => {
                check_bounds(sample, p, bounds);
                assert_eq!(value.approx_with_budget(p, &zero), answer);
                assert_eq!(value.approx(p), answer);
            }
            Err(problem) => {
                assert_eq!(*problem, Problem::Exhausted);
                assert_eq!(after, before);
            }
        }
        if let Some((precision, _)) = before {
            check_bounds(
                &value.approx_with_budget(precision, &zero).unwrap(),
                precision,
                bounds,
            );
        }
        let sign_after = value.sign_until_with_budget(0, &zero);
        assert!(matches!(sign_after, None | Some(crate::RealSign::Positive)));
        if let Some(sign) = sign_before {
            assert_eq!(sign_after, Some(sign));
        }
    }

    #[test]
    fn log_range_stored_budget_cutpoints() {
        shared_setup();
        for exponent in [16, 65, 4097] {
            let bounds = log_bounds(exponent, 1, 1);
            for state in ["cold", "prefix", "sign"] {
                for p in [0, -32] {
                    for units in [0, 1, 2, 4, 8, 32] {
                        observe(
                            "stored",
                            exponent,
                            state,
                            stored_deferred(exponent),
                            p,
                            units,
                            &bounds,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn log_range_public_construction_routes() {
        shared_setup();
        for minimum in [16, 65, 512, 2048, 8192] {
            let (exponent, threes, residual, asinh_input) = public_inputs(minimum);
            let bounds = log_bounds(exponent, threes, 2);
            for route in ["asinh", "real-json"] {
                for p in [0, -32] {
                    for units in [0, 1, 2, 4, 8, 32] {
                        let value = if route == "asinh" {
                            Computable::rational(asinh_input.clone()).asinh()
                        } else {
                            let real = crate::Real::from(residual.clone()).ln_1p().unwrap();
                            let encoded: serde_json::Value =
                                serde_json::from_str(&real.to_json()).unwrap();
                            assert_eq!(
                                encoded["rational"],
                                serde_json::to_value(Rational::one()).unwrap()
                            );
                            assert_eq!(encoded["class"], "Irrational");
                            // This uses the public serialized payload. It is not
                            // a native Real API accepting EvaluationBudget.
                            serde_json::from_value(encoded["computable"].clone()).unwrap()
                        };
                        observe(route, exponent, "cold", value, p, units, &bounds);
                    }
                }
            }
        }
    }
}
