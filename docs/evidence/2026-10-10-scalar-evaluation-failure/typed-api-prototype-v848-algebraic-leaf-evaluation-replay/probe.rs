use hyperreal::{Computable, EvaluationBudget, Rational, RealSign};
use rug::{Float, Integer, Rational as Q, float::Round};
use serde_json::{Value, json};

fn leaf(value: Computable) -> Value {
    serde_json::to_value(value).unwrap()
}

fn offset(child: Value, shift: i32) -> Value {
    json!({"internal":{"Offset":[child,shift]}})
}

fn add(left: Value, right: Value) -> Value {
    json!({"internal":{"Add":[left,right]}})
}

fn product(left: Value, right: Value) -> Value {
    json!({"internal":{"Multiply":[left,right]}})
}

fn argument(shape: &str, shift: i32, residual: Value) -> Computable {
    let pi = leaf(Computable::pi());
    let third = leaf(Computable::rational(Rational::fraction(1, 3).unwrap()));
    let pi_third = product(pi.clone(), third.clone());
    let periodic = match shape {
        "offset_pi" => offset(pi, shift),
        "offset_third" => offset(pi_third, shift),
        "negative_third" => json!({"internal":{"Negate":offset(pi_third,shift)}}),
        "factored_third" => product(pi, offset(third, shift)),
        "split_third" => product(
            pi,
            add(offset(third.clone(), shift), offset(third, shift + 1)),
        ),
        "shifted_plus_half" => add(offset(pi.clone(), shift), offset(pi, -1)),
        "sixth_via_extremes" => offset(offset(pi_third, i32::MAX), i32::MIN),
        "deep_inverse_offsets" => {
            let mut value = pi_third;
            for _ in 0..32 {
                value = offset(offset(value, shift), -shift);
            }
            value
        },
        "ordinary_third" => pi_third,
        _ => panic!("unknown shape"),
    };
    serde_json::from_value(add(periodic, residual)).unwrap()
}

fn raw(name: &str, value: Value) -> Value {
    let fields: serde_json::Map<String, Value> =
        [(name.to_owned(), value)].into_iter().collect();
    json!({"internal":fields})
}

fn residual_value(name: &str) -> Value {
    let two = leaf(Computable::rational(Rational::new(2)));
    let sqrt2 = raw("Sqrt", two.clone());
    match name {
        "sqrt2" => sqrt2,
        "ln2" => raw("LnDeferred", two),
        "exp1" => raw("PrescaledExp", leaf(Computable::one())),
        "exp100" => raw("PrescaledExp", leaf(Computable::rational(Rational::new(100)))),
        "atan_half" => raw(
            "PrescaledAtan",
            leaf(Computable::rational(Rational::fraction(1, 2).unwrap())),
        ),
        "sqrt2_minus_one" => add(sqrt2, leaf(Computable::rational(Rational::new(-1)))),
        "large_sqrt2" => offset(sqrt2, 10),
        "tiny_sqrt2" => offset(sqrt2, -10),
        "negative_sqrt2" => raw("Negate", sqrt2),
        _ => panic!("unknown residual"),
    }
}

fn residual_enclosure(name: &str) -> (Q, Q) {
    let input = match name {
        "exp1" => Q::from(1),
        "exp100" => Q::from(100),
        "atan_half" => Q::from((1, 2)),
        _ => Q::from(2),
    };
    let mut lo = Float::with_val(1024, &input);
    assert_eq!(lo.to_rational().unwrap(), input);
    let mut hi = lo.clone();
    match name {
        "ln2" => {
            lo.ln_round(Round::Down);
            hi.ln_round(Round::Up);
        },
        "exp1" | "exp100" => {
            lo.exp_round(Round::Down);
            hi.exp_round(Round::Up);
        },
        "atan_half" => {
            lo.atan_round(Round::Down);
            hi.atan_round(Round::Up);
        },
        _ => {
            lo.sqrt_round(Round::Down);
            hi.sqrt_round(Round::Up);
        },
    }
    let (lo, hi) = (lo.to_rational().unwrap(), hi.to_rational().unwrap());
    match name {
        "sqrt2_minus_one" => (lo - 1, hi - 1),
        "large_sqrt2" => (lo * 1024, hi * 1024),
        "tiny_sqrt2" => (lo / 1024, hi / 1024),
        "negative_sqrt2" => (-hi, -lo),
        _ => (lo, hi),
    }
}

fn enclosure(family: &str, q: Q, residual: (Q, Q)) -> (Q, Q) {
    // The harness supplies an independently reduced rational coefficient of pi.
    // MPFR encloses that small angle. A derivative bound of one, rather than
    // endpoint monotonicity, encloses sin/cos across the complete angle interval.
    let (pi_lower, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Down);
    let (pi_upper, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Up);
    let mut lower = pi_lower.to_rational().unwrap() * &q;
    let mut upper = pi_upper.to_rational().unwrap() * &q;
    if q < 0 {
        std::mem::swap(&mut lower, &mut upper);
    }
    lower += residual.0;
    upper += residual.1;
    let (base, _) = Float::with_val_round(1024, &lower, Round::Down);
    let radius = upper - base.to_rational().unwrap();
    assert!(radius >= 0);
    let bounds = |cosine: bool| {
        let mut lo = base.clone();
        let mut hi = base.clone();
        if cosine {
            lo.cos_round(Round::Down);
            hi.cos_round(Round::Up);
        } else {
            lo.sin_round(Round::Down);
            hi.sin_round(Round::Up);
        }
        (
            lo.to_rational().unwrap() - &radius,
            hi.to_rational().unwrap() + &radius,
        )
    };
    let sine = bounds(false);
    let cosine = bounds(true);
    let quotient = |a: (Q, Q), b: (Q, Q)| {
        assert!(b.1 < 0 || b.0 > 0);
        let values = [
            a.0.clone() / &b.0,
            a.0 / &b.1,
            a.1.clone() / &b.0,
            a.1 / b.1,
        ];
        (
            values.iter().min().unwrap().clone(),
            values.iter().max().unwrap().clone(),
        )
    };
    match family {
        "sin" => sine,
        "cos" => cosine,
        "tan" => quotient(sine, cosine),
        "cot" => quotient(cosine, sine),
        _ => panic!("unknown family"),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let mode = args[1].as_str();
    let shape = args[2].as_str();
    let shift: i32 = args[3].parse().unwrap();
    let residual = args[4].as_str();
    let q_num: i64 = args[5].parse().unwrap();
    let q_den: u64 = args[6].parse().unwrap();
    let residual_bounds = residual_enclosure(residual);
    let (lower, upper) = enclosure(family, Q::from((q_num, q_den)), residual_bounds.clone());
    assert!(lower <= upper);
    println!(
        "{}",
        json!({"stage":"oracle", "pi_coefficient":format!("{q_num}/{q_den}"),
        "residual":residual, "residual_lower":residual_bounds.0.to_string(), "residual_upper":residual_bounds.1.to_string(),
        "lower":lower.to_string(), "upper":upper.to_string()})
    );
    let argument = argument(shape, shift, residual_value(residual));
    println!("{}", json!({"stage":"argument"}));
    let value = if mode == "public" {
        match family {
            "sin" => argument.sin(),
            "cos" => argument.cos(),
            "tan" => argument.tan(),
            "cot" => argument.clone().cos().multiply(argument.sin().inverse()),
            _ => unreachable!(),
        }
    } else {
        assert_eq!(mode, "serialized");
        let variant = match family {
            "sin" => "PrescaledSin",
            "cos" => "PrescaledCos",
            "tan" => "PrescaledTan",
            "cot" => "PrescaledCot",
            _ => unreachable!(),
        };
        let object: serde_json::Map<String, Value> =
            [(variant.to_owned(), leaf(argument))].into_iter().collect();
        serde_json::from_value(json!({"internal":object})).unwrap()
    };
    println!("{}", json!({"stage":"constructed"}));
    let empty = EvaluationBudget::new(0);
    let first = value.approx_with_budget(-256, &empty);
    assert!(first.is_ok() || matches!(first, Err(hyperreal::Problem::Exhausted)));
    if let Ok(sample) = &first {
        let unit = Q::from((1, Integer::from(1) << 256));
        let center =
            Q::from(Integer::from_str_radix(&sample.to_string(), 10).unwrap()) * &unit;
        assert!(Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper);
    }
    assert_eq!(empty.consumed(), 0);
    println!(
        "{}",
        json!({"stage":"empty_budget", "numeric":first.is_ok(),
        "error":first.as_ref().err().map(|e|format!("{e:?}")), "consumed":empty.consumed()})
    );
    let budget = EvaluationBudget::new(1_000_000);
    for precision in [0_i32, -8, -256, -8] {
        let result = value.approx_with_budget(precision, &budget);
        let unit = Q::from((1, Integer::from(1) << -precision));
        let sample = result
            .as_ref()
            .ok()
            .filter(|n| n.bits() <= 4096)
            .map(|n| n.to_string());
        let enclosed = sample.as_ref().is_some_and(|sample| {
            let center = Q::from(Integer::from_str_radix(sample, 10).unwrap()) * &unit;
            Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper
        });
        println!(
            "{}",
            json!({"stage":"sample", "precision":precision, "sample":sample,
            "numeric":result.is_ok(), "bits":result.as_ref().ok().map(|n| n.bits()),
            "error":result.as_ref().err().map(|e|format!("{e:?}")), "enclosed":enclosed})
        );
    }
    let before = value.structural_facts().sign;
    let after = value.sign_until_with_budget(0, &empty);
    if before.is_some() {
        assert_eq!(after, before);
    }
    if let Some(sign) = after {
        assert_eq!(
            sign,
            if lower > 0 {
                RealSign::Positive
            } else {
                assert!(upper < 0);
                RealSign::Negative
            }
        );
    }
    let warm = value.approx_with_budget(-8, &empty).unwrap();
    let unit = Q::from((1, 256));
    let center = Q::from(Integer::from_str_radix(&warm.to_string(), 10).unwrap()) * &unit;
    assert!(Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper);
    assert_eq!(empty.consumed(), 0);
    assert!(budget.consumed() <= 1_000_000);
    println!(
        "{}",
        json!({"stage":"warm_budget", "consumed":budget.consumed(),
        "remaining":budget.remaining(), "empty_consumed":empty.consumed(),
        "before_sign":before.map(|s|format!("{s:?}")),"after_sign":after.map(|s|format!("{s:?}")),
        "sample":warm.to_string(),"precision":-8,"enclosed":true})
    );
}
