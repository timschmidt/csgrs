use hyperreal::{Computable, EvaluationBudget, Rational, RealSign};
use rug::Integer;
use serde_json::{Value, json};
use std::{hint::black_box, time::Instant};

fn raw(name: &str, argument: Value) -> Value {
    let fields: serde_json::Map<String, Value> =
        [(name.to_owned(), argument)].into_iter().collect();
    json!({"internal": fields})
}

fn integer(value: &Integer) -> Value {
    let rational: Rational = value.to_string().parse().unwrap();
    serde_json::to_value(Computable::rational(rational)).unwrap()
}

fn root(value: Value, degree: u32) -> Value {
    if degree == 2 {
        raw("Sqrt", value)
    } else {
        raw("NthRoot", json!([value, degree]))
    }
}

fn fixture(
    degree: u32,
    count: usize,
    bits: u32,
    inverse: bool,
    perturbation: i32,
) -> Computable {
    let mut product = Integer::from(1);
    let mut factors = Vec::new();
    for (index, prime) in [2, 3, 5, 7].into_iter().take(count).enumerate() {
        let factor = if bits == 0 {
            Integer::from(prime)
        } else {
            (Integer::from(1) << bits) + (2 * index + 1)
        };
        product *= &factor;
        factors.push(root(integer(&factor), degree));
    }
    let mut left = root(integer(&product), degree);
    let mut right = factors.remove(0);
    for factor in factors {
        right = raw("Multiply", json!([right, factor]));
    }
    if inverse {
        left = raw("Inverse", left);
        right = raw("Inverse", right);
    }
    // Principal roots of positive rationals multiply, so this is exactly zero.
    let mut difference = raw("Add", json!([left, raw("Negate", right)]));
    if perturbation != 0 {
        let delta = raw("Offset", json!([integer(&Integer::from(perturbation)), -96]));
        difference = raw("Add", json!([difference, delta]));
    }
    serde_json::from_value(difference).unwrap()
}

fn expected(perturbation: i32) -> RealSign {
    match perturbation {
        -1 => RealSign::Negative,
        0 => RealSign::Zero,
        1 => RealSign::Positive,
        _ => unreachable!(),
    }
}

fn sample(
    value: &Computable,
    precision: i32,
    perturbation: i32,
    budget: Option<&EvaluationBudget>,
) -> String {
    let result = match budget {
        Some(budget) => value.approx_with_budget(precision, budget),
        None => value.approx(precision),
    }
    .unwrap();
    let sample = Integer::from_str_radix(&result.to_string(), 10).unwrap();
    let mut expected = rug::Rational::from(perturbation);
    if precision <= -96 {
        expected *= Integer::from(1) << (-96 - precision) as u32;
    } else {
        expected /= Integer::from(1) << (96 + precision) as u32;
    }
    let difference = rug::Rational::from(sample) - expected;
    assert!(difference.abs() <= 1);
    result.to_string()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let degree: u32 = args[1].parse().unwrap();
    let count: usize = args[2].parse().unwrap();
    let bits: u32 = args[3].parse().unwrap();
    let inverse: bool = args[4].parse().unwrap();
    if args[0] == "cost" {
        let iterations: usize = args[5].parse().unwrap();
        let values: Vec<_> = (0..iterations)
            .map(|_| {
                let value = fixture(degree, count, bits, inverse, 0);
                sample(&value, -65536, 0, None);
                value
            })
            .collect();
        let start = Instant::now();
        let mut certified = 0;
        for value in &values {
            assert_eq!(black_box(value.sign_until(-65536)), Some(RealSign::Zero));
            certified += 1;
        }
        println!(
            "{}",
            json!({"stage":"cost", "iterations":iterations, "certified":certified,
            "elapsed_ns":start.elapsed().as_nanos().to_string()})
        );
        return;
    }
    assert_eq!(args[0], "case");
    let perturbation: i32 = args[5].parse().unwrap();
    let units: u64 = args[6].parse().unwrap();
    let value = fixture(degree, count, bits, inverse, perturbation);
    let before = sample(&value, -65536, perturbation, None);
    let budget = EvaluationBudget::new(units);
    let answer = value.sign_until_with_budget(-65536, &budget);
    if let Some(answer) = answer {
        assert_eq!(answer, expected(perturbation));
    }
    assert_eq!(budget.consumed() + budget.remaining(), units);
    assert_eq!(value.sign_until(-65536), Some(expected(perturbation)));
    let zero = EvaluationBudget::new(0);
    assert_eq!(
        value.sign_until_with_budget(0, &zero),
        Some(expected(perturbation))
    );
    let after = sample(&value, -8, perturbation, Some(&zero));
    assert_eq!(zero.consumed(), 0);
    println!(
        "{}",
        json!({"stage":"case", "degree":degree, "factors":count, "coefficient_bits":bits,
        "inverse":inverse, "perturbation":perturbation, "allowance":units,
        "answer":answer.map(|s| format!("{s:?}")), "expected":format!("{:?}",expected(perturbation)),
        "consumed":budget.consumed(), "remaining":budget.remaining(),
        "before_sample":before, "before_precision":-65536, "after_sample":after, "after_precision":-8,
        "zero_budget_proof_retained":true})
    );
}
