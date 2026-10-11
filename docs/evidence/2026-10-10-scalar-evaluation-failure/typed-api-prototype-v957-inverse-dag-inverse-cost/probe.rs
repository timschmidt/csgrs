use hyperreal::{Computable, EvaluationBudget, Rational, RealSign};
use rug::{Integer, Rational as Q};
use serde_json::{Value, json};
use std::{hint::black_box, time::Instant};

fn raw(name: &str, argument: Value) -> Value {
    let fields: serde_json::Map<String, Value> =
        [(name.to_owned(), argument)].into_iter().collect();
    json!({"internal": fields})
}

fn rational(value: &Q) -> Value {
    let value: Rational = value.to_string().parse().unwrap();
    serde_json::to_value(Computable::rational(value)).unwrap()
}

fn integer(value: &Integer) -> Value {
    rational(&Q::from(value.clone()))
}

fn add(a: Value, b: Value) -> Value {
    raw("Add", json!([a, b]))
}

fn mul(a: Value, b: Value) -> Value {
    raw("Multiply", json!([a, b]))
}

fn sub(a: Value, b: Value) -> Value {
    add(a, raw("Negate", b))
}

fn fixture(family: &str, bits: u32, perturbation: i32) -> Computable {
    let b = if bits == 0 {
        Integer::from(5)
    } else {
        Integer::from(1) << bits
    };
    let a: Integer = Integer::from(&b * 3) + 1;
    let pi = || serde_json::to_value(Computable::pi()).unwrap();
    let x = || rational(&Q::from((1, 3)));
    let y = || rational(&Q::from((2, 5)));
    let atan = || raw("AtanDeferred", x());
    let acos = || raw("AcosPositive", x());
    let difference = match family {
        "atan" => sub(
            add(mul(integer(&a), atan()), mul(integer(&b), atan())),
            mul(integer(&Integer::from(&a + &b)), atan()),
        ),
        "acos" => sub(
            mul(integer(&a), add(raw("AsinDeferred", x()), acos())),
            mul(rational(&Q::from((a.clone(), 2))), pi()),
        ),
        "terms" => {
            let other = || raw("AtanDeferred", y());
            sub(
                add(mul(integer(&a), atan()), mul(integer(&b), other())),
                add(mul(integer(&b), other()), mul(integer(&a), atan())),
            )
        },
        "pi" => sub(
            mul(integer(&Integer::from(&a + &b)), pi()),
            add(mul(integer(&a), pi()), mul(integer(&b), pi())),
        ),
        _ => unreachable!(),
    };
    let value = if perturbation == 0 {
        difference
    } else {
        add(
            difference,
            raw("Offset", json!([integer(&Integer::from(perturbation)), -96])),
        )
    };
    serde_json::from_value(value).unwrap()
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
    let value = match budget {
        Some(budget) => value.approx_with_budget(precision, budget),
        None => value.approx(precision),
    }
    .unwrap();
    let parsed = Integer::from_str_radix(&value.to_string(), 10).unwrap();
    let mut exact = Q::from(perturbation);
    if precision <= -96 {
        exact *= Integer::from(1) << (-96 - precision) as u32;
    } else {
        exact /= Integer::from(1) << (96 + precision) as u32;
    }
    assert!((Q::from(parsed) - exact).abs() <= 1);
    value.to_string()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = &args[1];
    let bits: u32 = args[2].parse().unwrap();
    if args[0] == "cost" {
        let iterations: usize = args[3].parse().unwrap();
        let values: Vec<_> = (0..iterations)
            .map(|_| {
                let value = fixture(family, bits, 0);
                sample(&value, -512, 0, None);
                value
            })
            .collect();
        let start = Instant::now();
        let mut certified = 0;
        for value in &values {
            assert_eq!(black_box(value.sign_until(-512)), Some(RealSign::Zero));
            certified += 1;
        }
        println!(
            "{}",
            json!({"stage":"cost", "iterations":iterations, "certified":certified, "elapsed_ns":start.elapsed().as_nanos().to_string()})
        );
        return;
    }
    assert_eq!(args[0], "case");
    let perturbation: i32 = args[3].parse().unwrap();
    let units: u64 = args[4].parse().unwrap();
    let warm: bool = args[5].parse().unwrap();
    let value = fixture(family, bits, perturbation);
    let before = warm.then(|| sample(&value, -512, perturbation, None));
    let budget = EvaluationBudget::new(units);
    let answer = value.sign_until_with_budget(-512, &budget);
    if let Some(answer) = answer {
        assert_eq!(answer, expected(perturbation));
    }
    assert_eq!(budget.consumed() + budget.remaining(), units);
    assert_eq!(value.sign_until(-512), Some(expected(perturbation)));
    let precise = sample(&value, -512, perturbation, None);
    let zero = EvaluationBudget::new(0);
    assert_eq!(
        value.sign_until_with_budget(0, &zero),
        Some(expected(perturbation))
    );
    let after = sample(&value, -8, perturbation, Some(&zero));
    assert_eq!(zero.consumed(), 0);
    println!(
        "{}",
        json!({"stage":"case", "family":family, "coefficient_bits":bits,
        "perturbation":perturbation, "allowance":units, "warm":warm,
        "answer":answer.map(|s| format!("{s:?}")), "expected":format!("{:?}",expected(perturbation)),
        "consumed":budget.consumed(), "remaining":budget.remaining(),
        "before_sample":before, "before_precision":-512, "precise_sample":precise, "precise_precision":-512,
        "after_sample":after, "after_precision":-8, "zero_budget_proof_retained":true})
    );
}
