use hyperreal::{
    Computable, EvaluationBudget, Problem, Rational, RealSign, RealStructuralFacts,
    ZeroKnowledge,
};
use rug::{
    Float, Integer, Rational as Q,
    float::{Constant, Round},
};
use serde_json::{Value, json};

#[derive(Clone)]
struct Oracle {
    lo: Q,
    hi: Q,
    offset: Q,
    sign: RealSign,
    fine: i32,
}

fn power(exponent: i32) -> Q {
    if exponent >= 0 {
        Q::from(Integer::from(1) << exponent as u32)
    } else {
        Q::from((1, Integer::from(1) << exponent.unsigned_abs()))
    }
}

fn oracle(family: &str, bits: u32, negative: bool) -> Oracle {
    let precision = 2560;
    let mut lo;
    let mut hi;
    match family {
        "pi" => {
            lo = Float::with_val_round(precision, Constant::Pi, Round::Down).0;
            hi = Float::with_val_round(precision, Constant::Pi, Round::Up).0;
        },
        "sqrt2" => {
            lo = Float::with_val(precision, 2);
            hi = lo.clone();
            lo.sqrt_round(Round::Down);
            hi.sqrt_round(Round::Up);
        },
        "sin13" => {
            let input = Q::from((1, 3));
            lo = Float::with_val_round(precision, &input, Round::Down).0;
            hi = Float::with_val_round(precision, &input, Round::Up).0;
            // Sine is increasing on this positive interval within (0, 1).
            lo.sin_round(Round::Down);
            hi.sin_round(Round::Up);
        },
        _ => unreachable!(),
    }
    let mut lo = lo.to_rational().unwrap();
    let mut hi = hi.to_rational().unwrap();
    let denominator = Integer::from(1) << bits;
    let low_scaled = lo.clone() * &denominator;
    let high_scaled = hi.clone() * &denominator;
    let floor = Integer::from(low_scaled.numer() / low_scaled.denom());
    assert_eq!(
        floor,
        Integer::from(high_scaled.numer() / high_scaled.denom())
    );
    let offset = Q::from((floor, denominator));
    lo -= &offset;
    hi -= &offset;
    assert!(lo > power(-(bits as i32) - 64));
    assert!(hi < power(-(bits as i32)));
    if negative {
        (lo, hi) = (-hi, -lo);
    }
    Oracle {
        lo,
        hi,
        offset,
        sign: if negative {
            RealSign::Negative
        } else {
            RealSign::Positive
        },
        fine: -(bits as i32) - 128,
    }
}

fn sample(
    value: &Computable,
    precision: i32,
    budget: Option<&EvaluationBudget>,
    oracle: &Oracle,
    required: bool,
) -> Value {
    let answer = match budget {
        Some(budget) => value.approx_with_budget(precision, budget),
        None => value.approx(precision),
    };
    match answer {
        Ok(value) => {
            let integer = Integer::from_str_radix(&value.to_string(), 10).unwrap();
            let unit = power(precision);
            let center = Q::from(integer) * &unit;
            assert!(center.clone() - &unit <= oracle.lo && center + &unit >= oracle.hi);
            json!({"precision": precision, "sample": value.to_string()})
        },
        Err(problem) => {
            assert!(
                !required,
                "required cached/refined sample failed: {problem:?}"
            );
            assert_eq!(problem, Problem::Exhausted);
            json!({"precision": precision, "error": format!("{problem:?}")})
        },
    }
}

fn sign(
    value: &Computable,
    precision: i32,
    budget: &EvaluationBudget,
    oracle: &Oracle,
    required: bool,
) -> Value {
    let answer = value.sign_until_with_budget(precision, budget);
    if let Some(sign) = answer {
        assert_eq!(sign, oracle.sign);
    }
    if required {
        assert_eq!(answer, Some(oracle.sign));
    }
    json!({"precision": precision, "sign": answer.map(|s| format!("{s:?}"))})
}

fn spend(budget: &EvaluationBudget, limit: u64) {
    let integer = Computable::rational(Rational::new(7));
    for _ in 0..=limit {
        if integer.approx_with_budget(0, budget) == Err(Problem::Exhausted) {
            break;
        }
    }
    assert_eq!(budget.remaining(), 0);
    assert_eq!(budget.consumed(), limit);
}

fn roundtrip(value: &Computable) -> Computable {
    serde_json::from_slice(&serde_json::to_vec(value).unwrap()).unwrap()
}

fn rational(value: i64) -> Computable {
    Computable::rational(Rational::new(value))
}

fn zero_oracle(fine: i32) -> Oracle {
    Oracle {
        lo: Q::from(0),
        hi: Q::from(0),
        offset: Q::from(0),
        sign: RealSign::Zero,
        fine,
    }
}

fn child(family: &str, oracle: &Oracle, negative: bool) -> Computable {
    if family == "zero" {
        return rational(0);
    }
    let value = match family {
        "pi" => Computable::pi(),
        "sqrt2" => rational(2).sqrt(),
        "sin13" => Computable::rational(Rational::fraction(1, 3).unwrap()).sin(),
        _ => unreachable!(),
    };
    let offset: Rational = (-oracle.offset.clone()).to_string().parse().unwrap();
    let value = value.add(Computable::rational(offset));
    if negative { value.negate() } else { value }
}

fn compositions(x: &Computable) -> Vec<(&'static str, Computable)> {
    let half = || Computable::rational(Rational::fraction(1, 2).unwrap());
    let positive_exp = x.clone().exp();
    let negative_exp = x.clone().negate().exp();
    let log_exp = positive_exp.clone().ln();
    let log_one_plus_expm1 = x.clone().expm1().add(rational(1)).ln();
    let expm1_log_one_plus = x.clone().add(rational(1)).ln().expm1();
    let inverse_sinh = positive_exp
        .clone()
        .add(negative_exp.clone().negate())
        .multiply(half())
        .asinh();
    let inverse_cosh = positive_exp.add(negative_exp).multiply(half()).acosh();
    let numerator = x.clone().multiply(rational(2)).expm1();
    let inverse_tanh = numerator
        .clone()
        .multiply(numerator.add(rational(2)).inverse())
        .atanh();
    let cancellation = log_exp.clone().add(x.clone().negate());
    vec![
        ("log_exp", log_exp),
        ("log_one_plus_expm1", log_one_plus_expm1),
        ("expm1_log_one_plus", expm1_log_one_plus),
        ("asinh_sinh", inverse_sinh),
        ("acosh_cosh", inverse_cosh),
        ("atanh_tanh", inverse_tanh),
        ("log_exp_cancellation", cancellation),
    ]
}

fn root_oracle(name: &str, base: &Oracle) -> Oracle {
    if name == "log_exp_cancellation" {
        return zero_oracle(base.fine);
    }
    let mut result = base.clone();
    if name == "acosh_cosh" && result.sign == RealSign::Negative {
        (result.lo, result.hi) = (-result.hi, -result.lo);
        result.sign = RealSign::Positive;
    }
    result
}

fn facts_json(facts: RealStructuralFacts) -> Value {
    json!({"sign": facts.sign.map(|s| format!("{s:?}")), "zero": format!("{:?}", facts.zero),
        "exact_rational": facts.exact_rational,
        "magnitude": facts.magnitude.map(|m| json!({"msd": m.msd, "exact_msd": m.exact_msd}))})
}

fn facts(
    value: &Computable,
    oracle: &Oracle,
    previous: Option<RealStructuralFacts>,
) -> RealStructuralFacts {
    let current = value.structural_facts();
    if let Some(sign) = current.sign {
        assert_eq!(sign, oracle.sign);
    }
    if oracle.sign == RealSign::Zero {
        assert_ne!(current.zero, ZeroKnowledge::NonZero);
        assert!(current.magnitude.is_none());
    } else {
        assert_ne!(current.zero, ZeroKnowledge::Zero);
        if let Some(bound) = current.magnitude {
            assert_eq!(current.zero, ZeroKnowledge::NonZero);
            let (lo, hi) = if oracle.sign == RealSign::Positive {
                (oracle.lo.clone(), oracle.hi.clone())
            } else {
                (-oracle.hi.clone(), -oracle.lo.clone())
            };
            assert!(hi < power(bound.msd + 1));
            if bound.exact_msd {
                assert!(lo >= power(bound.msd));
            }
        }
    }
    if let Some(before) = previous {
        if before.sign.is_some() {
            assert_eq!(current.sign, before.sign);
        }
        if before.zero != ZeroKnowledge::Unknown {
            assert_eq!(current.zero, before.zero);
        }
        if before.exact_rational {
            assert!(current.exact_rational);
        }
        if let Some(old) = before.magnitude {
            let new = current.magnitude.expect("available magnitude proof lost");
            assert!(new.msd <= old.msd);
            if old.exact_msd {
                assert_eq!(new, old);
            }
        }
    }
    current
}

fn order(schedule: usize) -> Vec<usize> {
    match schedule {
        0 => vec![0, 1, 2, 3, 4, 5, 6],
        1 => vec![6, 5, 4, 3, 2, 1, 0],
        2 => vec![3, 0, 5, 1, 4, 2, 6],
        _ => unreachable!(),
    }
}

fn oracle_json(oracle: &Oracle) -> Value {
    json!({"lower": oracle.lo.to_string(), "upper": oracle.hi.to_string(),
        "expected_sign": format!("{:?}", oracle.sign), "fine": oracle.fine})
}

fn update_all(
    roots: &[(&str, Computable)],
    oracles: &[Oracle],
    previous: &mut [RealStructuralFacts],
) -> Value {
    let mut output = Vec::new();
    for (index, (_, value)) in roots.iter().enumerate() {
        previous[index] = facts(value, &oracles[index], Some(previous[index]));
        output.push(facts_json(previous[index]));
    }
    json!(output)
}

fn sign_required(facts: RealStructuralFacts, approximation: &Value) -> bool {
    facts.sign.is_some()
        || approximation.get("sample").is_some_and(|value| {
            Integer::from_str_radix(value.as_str().unwrap(), 10)
                .unwrap()
                .abs()
                > 1
        })
}

fn run(
    family: &str,
    bits: u32,
    negative: bool,
    reconstruct: bool,
    schedule: usize,
    units: u64,
) -> Value {
    let base = if family == "zero" {
        zero_oracle(-128)
    } else {
        oracle(family, bits, negative)
    };
    let x = child(family, &base, negative);
    let mut roots = compositions(&x);
    let oracles: Vec<_> = roots
        .iter()
        .map(|(name, _)| root_oracle(name, &base))
        .collect();
    if reconstruct {
        for (_, value) in &mut roots {
            *value = roundtrip(value);
        }
    }
    let wire: Vec<_> = roots
        .iter()
        .map(|(_, v)| serde_json::to_vec(v).unwrap())
        .collect();
    let mut prior: Vec<_> = roots
        .iter()
        .zip(&oracles)
        .map(|((_, v), oracle)| facts(v, oracle, None))
        .collect();
    let initial: Vec<_> = prior.iter().copied().map(facts_json).collect();
    let mut child_prior = facts(&x, &base, None);
    let allowance = EvaluationBudget::new(units);
    let mut events = Vec::new();
    for index in order(schedule) {
        let consumed_before = allowance.consumed();
        let approximation = sample(
            &roots[index].1,
            base.fine,
            Some(&allowance),
            &oracles[index],
            false,
        );
        let consumed_after_sample = allowance.consumed();
        let sign = sign(
            &roots[index].1,
            0,
            &allowance,
            &oracles[index],
            sign_required(prior[index], &approximation),
        );
        let snapshots = update_all(&roots, &oracles, &mut prior);
        child_prior = facts(&x, &base, Some(child_prior));
        assert_eq!(allowance.consumed() + allowance.remaining(), units);
        events.push(json!({"stage": "bounded", "root": index, "approximation": approximation, "sign": sign,
            "consumed_before": consumed_before, "consumed_after_sample": consumed_after_sample,
            "consumed_after_sign": allowance.consumed(), "all_facts": snapshots, "child_facts": facts_json(child_prior)}));
    }
    let before_drain = allowance.consumed();
    spend(&allowance, units);
    let mut retry_order = order(schedule);
    retry_order.reverse();
    for index in retry_order {
        let approximation = sample(&roots[index].1, base.fine, None, &oracles[index], true);
        let sign = sign(
            &roots[index].1,
            0,
            &allowance,
            &oracles[index],
            sign_required(prior[index], &approximation),
        );
        let snapshots = update_all(&roots, &oracles, &mut prior);
        child_prior = facts(&x, &base, Some(child_prior));
        events.push(
            json!({"stage": "unlimited_retry", "root": index, "approximation": approximation,
            "sign": sign, "all_facts": snapshots, "child_facts": facts_json(child_prior)}),
        );
    }
    for index in order(schedule) {
        for precision in [base.fine, -64, 0, 16] {
            let approximation = sample(
                &roots[index].1,
                precision,
                Some(&allowance),
                &oracles[index],
                true,
            );
            let sign = sign(
                &roots[index].1,
                0,
                &allowance,
                &oracles[index],
                sign_required(prior[index], &approximation),
            );
            prior[index] = facts(&roots[index].1, &oracles[index], Some(prior[index]));
            events.push(
                json!({"stage": "cached", "root": index, "approximation": approximation,
                "sign": sign, "facts": facts_json(prior[index])}),
            );
        }
        let finer = sample(
            &roots[index].1,
            base.fine - 64,
            Some(&allowance),
            &oracles[index],
            false,
        );
        let retained = sample(
            &roots[index].1,
            base.fine,
            Some(&allowance),
            &oracles[index],
            true,
        );
        let retry = sample(&roots[index].1, base.fine - 64, None, &oracles[index], true);
        let cached = sample(
            &roots[index].1,
            base.fine - 64,
            Some(&allowance),
            &oracles[index],
            true,
        );
        let sign = sign(
            &roots[index].1,
            0,
            &allowance,
            &oracles[index],
            sign_required(prior[index], &retry),
        );
        let snapshots = update_all(&roots, &oracles, &mut prior);
        child_prior = facts(&x, &base, Some(child_prior));
        events.push(
            json!({"stage": "finer_retry", "root": index, "finer_attempt": finer,
            "retained_sample": retained, "retry": retry, "cached_retry": cached, "sign": sign,
            "all_facts": snapshots, "child_facts": facts_json(child_prior)}),
        );
    }
    for (index, (_, value)) in roots.iter().enumerate() {
        assert_eq!(serde_json::to_vec(value).unwrap(), wire[index]);
    }
    assert_eq!(allowance.remaining(), 0);
    assert_eq!(allowance.consumed(), units);
    json!({"family": family, "bits": bits, "negative": negative, "reconstructed": reconstruct,
        "schedule": schedule, "units": units, "root_names": roots.iter().map(|r| r.0).collect::<Vec<_>>(),
        "oracles": oracles.iter().map(oracle_json).collect::<Vec<_>>(), "initial_facts": initial,
        "events": events, "final_facts": prior.iter().copied().map(facts_json).collect::<Vec<_>>(),
        "consumed_before_drain": before_drain, "consumed_after_drain": allowance.consumed(),
        "base_child_shared_with_roots": !reconstruct, "serialized_expressions_unchanged": true,
        "serialized_root_bytes": wire.iter().map(Vec::len).collect::<Vec<_>>()})
}

fn invalid(kind: &str, reconstruct: bool, schedule: usize, units: u64) -> Value {
    let x = match kind {
        "inverse0" => rational(0).inverse(),
        "sqrtneg" => rational(-1).sqrt(),
        "logneg" => rational(-1).ln(),
        _ => unreachable!(),
    };
    let mut roots = compositions(&x);
    if reconstruct {
        for (_, v) in &mut roots {
            *v = roundtrip(v);
        }
    }
    let wire: Vec<_> = roots
        .iter()
        .map(|(_, v)| serde_json::to_vec(v).unwrap())
        .collect();
    let allowance = EvaluationBudget::new(units);
    let mut rows = Vec::new();
    for index in order(schedule) {
        let value = &roots[index].1;
        let bounded = value.approx_with_budget(-128, &allowance);
        assert!(matches!(
            bounded,
            Err(Problem::Exhausted
                | Problem::DivideByZero
                | Problem::SqrtNegative
                | Problem::NotANumber)
        ));
        assert_eq!(value.sign_until_with_budget(0, &allowance), None);
        assert_eq!(value.structural_facts().sign, None);
        let retry = value.approx(-128);
        assert!(matches!(
            retry,
            Err(Problem::DivideByZero | Problem::SqrtNegative | Problem::NotANumber)
        ));
        assert_eq!(
            value.sign_until_with_budget(0, &EvaluationBudget::new(0)),
            None
        );
        rows.push(json!({"root": index, "name": roots[index].0, "bounded": format!("{bounded:?}"), "retry": format!("{retry:?}")}));
    }
    assert_eq!(allowance.consumed() + allowance.remaining(), units);
    for (index, (_, value)) in roots.iter().enumerate() {
        assert_eq!(serde_json::to_vec(value).unwrap(), wire[index]);
    }
    json!({"kind": kind, "reconstructed": reconstruct, "schedule": schedule, "units": units,
        "rows": rows, "consumed": allowance.consumed(), "serialized_expressions_unchanged": true})
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mode = &args[0];
    let result = if mode == "invalid" {
        invalid(
            &args[1],
            args[2].parse().unwrap(),
            args[3].parse().unwrap(),
            args[4].parse().unwrap(),
        )
    } else {
        assert_eq!(mode, "composed");
        run(
            &args[1],
            args[2].parse().unwrap(),
            args[3].parse().unwrap(),
            args[4].parse().unwrap(),
            args[5].parse().unwrap(),
            args[6].parse().unwrap(),
        )
    };
    println!("{}", json!({"mode": mode, "result": result}));
}
