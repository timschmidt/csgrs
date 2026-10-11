use hyperreal::{Computable, Rational};
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

fn argument(shape: &str, shift: i32, residual: Rational) -> Computable {
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
    serde_json::from_value(add(periodic, leaf(Computable::rational(residual)))).unwrap()
}

fn enclosure(family: &str, q: Q, residual: Q) -> (Q, Q) {
    // The harness supplies an independently reduced rational coefficient of pi.
    // MPFR encloses that small angle. A derivative bound of one, rather than
    // endpoint monotonicity, encloses sin/cos across the complete angle interval.
    let (pi_lower, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Down);
    let (pi_upper, _) = Float::with_val_round(1024, rug::float::Constant::Pi, Round::Up);
    let mut lower = pi_lower.to_rational().unwrap() * &q + &residual;
    let mut upper = pi_upper.to_rational().unwrap() * &q + residual;
    if q < 0 {
        std::mem::swap(&mut lower, &mut upper);
    }
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
    let numerator: i64 = args[4].parse().unwrap();
    let denominator: u64 = args[5].parse().unwrap();
    let q_num: i64 = args[6].parse().unwrap();
    let q_den: u64 = args[7].parse().unwrap();
    let (lower, upper) = enclosure(
        family,
        Q::from((q_num, q_den)),
        Q::from((numerator, denominator)),
    );
    assert!(lower <= upper);
    println!(
        "{}",
        json!({"stage":"oracle", "pi_coefficient":format!("{q_num}/{q_den}"),
        "residual":format!("{numerator}/{denominator}"), "lower":lower.to_string(), "upper":upper.to_string()})
    );
    let argument = argument(
        shape,
        shift,
        Rational::fraction(numerator, denominator).unwrap(),
    );
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
    for precision in [0_i32, -8, -256, -8] {
        let result = value.approx(precision);
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
}
