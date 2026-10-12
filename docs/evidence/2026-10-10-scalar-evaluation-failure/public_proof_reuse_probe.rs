use hyperreal::{
    Computable, EvaluationBudget, Problem, Rational, RealSign, RealStructuralFacts,
    ZeroKnowledge,
};
use rug::{
    Float, Integer, Rational as Q,
    float::{Constant, Round},
};
use serde_json::{Value, json};
use std::sync::{Arc, Barrier};
use std::thread;

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

fn fixture(family: &str, oracle: &Oracle, negative: bool, reconstruct: bool) -> Computable {
    let base = match family {
        "pi" => Computable::pi(),
        "sqrt2" => Computable::rational(Rational::new(2)).sqrt(),
        "sin13" => Computable::rational(Rational::fraction(1, 3).unwrap()).sin(),
        _ => unreachable!(),
    };
    let offset: Rational = (-oracle.offset.clone()).to_string().parse().unwrap();
    let value = base.add(Computable::rational(offset));
    let value = if negative { value.negate() } else { value };
    if reconstruct {
        roundtrip(&value)
    } else {
        value
    }
}

fn roundtrip(value: &Computable) -> Computable {
    serde_json::from_slice(&serde_json::to_vec(value).unwrap()).unwrap()
}

fn facts_json(facts: RealStructuralFacts) -> Value {
    json!({
        "sign": facts.sign.map(|s| format!("{s:?}")),
        "zero": format!("{:?}", facts.zero),
        "exact_rational": facts.exact_rational,
        "magnitude": facts.magnitude.map(|m| json!({"msd": m.msd, "exact_msd": m.exact_msd})),
    })
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
    assert_ne!(current.zero, ZeroKnowledge::Zero);
    if let Some(bound) = current.magnitude {
        assert_eq!(current.zero, ZeroKnowledge::NonZero);
        let (lo, hi) = if oracle.sign == RealSign::Positive {
            (oracle.lo.clone(), oracle.hi.clone())
        } else {
            (-oracle.hi.clone(), -oracle.lo.clone())
        };
        assert!(hi < power(bound.msd + 1), "invalid upper bound {bound:?}");
        if bound.exact_msd {
            assert!(lo >= power(bound.msd), "invalid exact bound {bound:?}");
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
            let new = current.magnitude.expect("available magnitude proof was lost");
            if old.exact_msd {
                assert_eq!(new, old);
            } else {
                assert!(new.msd <= old.msd, "weaker bound: {old:?} -> {new:?}");
            }
        }
    }
    current
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

fn sequential(value: &Computable, oracle: &Oracle) -> Value {
    let wire = serde_json::to_vec(value).unwrap();
    let zero = EvaluationBudget::new(0);
    let cold_sign = sign(value, 0, &zero, oracle, false);
    let cold_sample = sample(value, oracle.fine, Some(&zero), oracle, false);
    let cold_facts = facts(value, oracle, None);
    let warm = sample(value, oracle.fine, None, oracle, true);
    let warm_facts = facts(value, oracle, Some(cold_facts));
    assert_eq!(warm_facts.sign, Some(oracle.sign));
    let spent = EvaluationBudget::new(31);
    spend(&spent, 31);
    let clone = value.clone();
    let mut prior = warm_facts;
    let mut queries = Vec::new();
    for precision in [oracle.fine, oracle.fine + 128, -64, 0, 16] {
        let approximation = sample(&clone, precision, Some(&spent), oracle, true);
        let sign = sign(&clone, 0, &spent, oracle, true);
        prior = facts(&clone, oracle, Some(prior));
        queries.push(
            json!({"approximation": approximation, "sign": sign, "facts": facts_json(prior)}),
        );
    }
    let finer_miss = sample(value, oracle.fine - 64, Some(&spent), oracle, false);
    prior = facts(value, oracle, Some(prior));
    let retained_after_miss = sample(value, oracle.fine, Some(&spent), oracle, true);
    let finer_retry = sample(value, oracle.fine - 64, None, oracle, true);
    let final_facts = facts(value, oracle, Some(prior));
    let after_retry = sample(&clone, oracle.fine - 64, Some(&spent), oracle, true);
    assert_eq!(serde_json::to_vec(value).unwrap(), wire);
    assert_eq!(spent.consumed(), 31);
    assert_eq!(zero.consumed(), 0);
    json!({"cold_sign": cold_sign, "cold_sample": cold_sample, "cold_facts": facts_json(cold_facts),
        "warm": warm, "warm_facts": facts_json(warm_facts), "cached_queries": queries,
        "finer_attempt": finer_miss, "retained_after_finer_attempt": retained_after_miss,
        "finer_retry": finer_retry, "cached_after_retry": after_retry, "final_facts": facts_json(final_facts),
        "spent_units": spent.consumed(), "serialized_expression_unchanged": true})
}

fn concurrent(value: &Computable, oracle: &Oracle, warm: bool, schedule: usize) -> Value {
    let wire = serde_json::to_vec(value).unwrap();
    let initial = facts(value, oracle, None);
    let warmup = warm.then(|| sample(value, oracle.fine, None, oracle, true));
    let before = facts(value, oracle, Some(initial));
    let limit = if warm { 31 } else { [0, 8, 64][schedule] };
    let shared = EvaluationBudget::new(limit);
    if warm {
        spend(&shared, limit);
    }
    let independent = EvaluationBudget::new(1_000_000);
    let barrier = Arc::new(Barrier::new(8));
    let mut handles = Vec::new();
    for worker in 0..8 {
        let value = value.clone();
        let oracle = oracle.clone();
        let barrier = barrier.clone();
        let shared = shared.clone();
        let independent = independent.clone();
        handles.push(thread::Builder::new().stack_size(512 * 1024).spawn(move || {
            barrier.wait();
            let mut previous = before;
            let mut rows = Vec::new();
            for round in 0..8 {
                if (worker + round + schedule) % 3 == 0 { thread::yield_now(); }
                let precision = [oracle.fine, -64, 0, oracle.fine + 128][(worker + round + schedule) % 4];
                let budget = if !warm && worker == 0 { None }
                    else if !warm && worker == 1 { Some(&independent) }
                    else { Some(&shared) };
                let approximation = sample(&value, precision, budget, &oracle, warm || worker < 2);
                let sign = sign(&value, 0, &shared, &oracle, warm);
                previous = facts(&value, &oracle, Some(previous));
                rows.push(json!({"worker": worker, "round": round, "approximation": approximation,
                    "sign": sign, "facts": facts_json(previous)}));
            }
            rows
        }).unwrap());
    }
    let rows: Vec<_> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
    assert_eq!(rows.len(), 64);
    // Accounting is compared after joining; separate concurrent getter calls
    // need not describe one atomic snapshot while other threads spend work.
    assert_eq!(shared.consumed() + shared.remaining(), limit);
    assert_eq!(independent.consumed() + independent.remaining(), 1_000_000);
    let before_drain = shared.consumed();
    spend(&shared, limit);
    let final_facts = facts(value, oracle, Some(before));
    assert_eq!(final_facts.sign, Some(oracle.sign));
    let final_sign = sign(value, 0, &shared, oracle, true);
    let final_sample = sample(value, oracle.fine, Some(&shared), oracle, true);
    assert_eq!(serde_json::to_vec(value).unwrap(), wire);
    json!({"initial_facts": facts_json(initial), "warmup": warmup, "before_facts": facts_json(before),
        "workers": 8, "rounds_per_worker": 8, "rows": rows, "shared_limit": limit,
        "shared_consumed_before_drain": before_drain, "shared_consumed_after_drain": shared.consumed(),
        "independent_consumed": independent.consumed(), "final_facts": facts_json(final_facts),
        "final_sign": final_sign, "final_sample": final_sample, "serialized_expression_unchanged": true})
}

fn invalid(kind: &str, reconstruct: bool) -> Value {
    let (value, expected) = match kind {
        "inverse0" => (
            Computable::rational(Rational::new(0)).inverse(),
            Problem::DivideByZero,
        ),
        "sqrtneg" => (
            Computable::rational(Rational::new(-1)).sqrt(),
            Problem::SqrtNegative,
        ),
        "logneg" => (
            Computable::rational(Rational::new(-1)).ln(),
            Problem::NotANumber,
        ),
        _ => unreachable!(),
    };
    let value = if reconstruct {
        roundtrip(&value)
    } else {
        value
    };
    let wire = serde_json::to_vec(&value).unwrap();
    let shared = EvaluationBudget::new(32);
    let barrier = Arc::new(Barrier::new(4));
    let mut handles = Vec::new();
    for worker in 0..4 {
        let value = value.clone();
        let shared = shared.clone();
        let barrier = barrier.clone();
        let expected = expected.clone();
        handles.push(thread::Builder::new().stack_size(512 * 1024).spawn(move || {
            barrier.wait();
            let mut rows = Vec::new();
            for round in 0..4 {
                let budget = EvaluationBudget::new(if worker % 2 == 0 { 0 } else { 100_000 });
                let active = if round % 2 == 0 { &shared } else { &budget };
                let result = value.approx_with_budget(-128, active);
                assert!(result == Err(Problem::Exhausted) || result == Err(expected.clone()));
                assert_eq!(value.sign_until_with_budget(-128, active), None);
                assert_eq!(value.structural_facts().sign, None);
                rows.push(json!({"worker": worker, "round": round, "answer": format!("{result:?}")}));
            }
            assert_eq!(value.approx(-128), Err(expected));
            rows
        }).unwrap());
    }
    let rows: Vec<_> = handles.into_iter().flat_map(|h| h.join().unwrap()).collect();
    assert_eq!(rows.len(), 16);
    assert_eq!(shared.consumed() + shared.remaining(), 32);
    assert_eq!(serde_json::to_vec(&value).unwrap(), wire);
    json!({"kind": kind, "reconstructed": reconstruct, "expected": format!("{expected:?}"),
        "rows": rows, "unlimited_retries": 4, "shared_consumed": shared.consumed(),
        "serialized_expression_unchanged": true})
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mode = &args[0];
    if mode == "invalid" {
        println!(
            "{}",
            json!({"mode": mode, "result": invalid(&args[1], args[2].parse().unwrap())})
        );
        return;
    }
    let family = &args[1];
    let bits: u32 = args[2].parse().unwrap();
    let negative: bool = args[3].parse().unwrap();
    let reconstruct: bool = args[4].parse().unwrap();
    let schedule: usize = args[5].parse().unwrap();
    let oracle = oracle(family, bits, negative);
    let value = fixture(family, &oracle, negative, reconstruct);
    let result = match mode.as_str() {
        "sequential" => sequential(&value, &oracle),
        "warm" => concurrent(&value, &oracle, true, schedule),
        "race" => concurrent(&value, &oracle, false, schedule),
        _ => unreachable!(),
    };
    println!(
        "{}",
        json!({"mode": mode, "family": family, "bits": bits,
        "negative": negative, "reconstructed": reconstruct, "schedule": schedule,
        "oracle": {"precision_bits": 2560, "lower": oracle.lo.to_string(), "upper": oracle.hi.to_string(),
            "offset": oracle.offset.to_string(), "expected_sign": format!("{:?}", oracle.sign), "fine": oracle.fine},
        "result": result})
    );
}
