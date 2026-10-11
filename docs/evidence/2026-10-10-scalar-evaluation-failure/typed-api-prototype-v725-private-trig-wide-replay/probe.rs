use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as Q, float::Round};
use serde_json::{Map, Value, json};

fn enclosure(family: &str, numerator: i64, denominator: u64) -> (Q, Q) {
    let exact = Q::from((numerator, denominator));
    let mut lower = Float::with_val(1024, &exact);
    assert_eq!(lower.to_rational().unwrap(), exact);
    let mut upper = lower.clone();
    if family.contains("Cos") || family == "CoscSmall" {
        lower.cos_round(Round::Down);
        upper.cos_round(Round::Up);
    } else {
        lower.sin_round(Round::Down);
        upper.sin_round(Round::Up);
    }
    let (lower, upper) = (lower.to_rational().unwrap(), upper.to_rational().unwrap());
    match family {
        "SincSmall" => {
            assert_ne!(numerator, 0);
            let (lower, upper) = (lower / &exact, upper / exact);
            if numerator < 0 {
                (upper, lower)
            } else {
                (lower, upper)
            }
        },
        "CoscSmall" => {
            assert_ne!(numerator, 0);
            let squared = exact.clone() * exact;
            (
                (Q::from(1) - upper) / &squared,
                (Q::from(1) - lower) / squared,
            )
        },
        _ => (lower, upper),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let mode = args[1].as_str();
    let numerator: i64 = args[2].parse().unwrap();
    let denominator: u64 = args[3].parse().unwrap();
    let precision: i32 = args[4].parse().unwrap();
    let (lower, upper) = enclosure(family, numerator, denominator);
    assert!(lower <= upper);
    println!(
        "{}",
        json!({"stage":"oracle", "lower":lower.to_string(), "upper":upper.to_string()})
    );
    let rational = Rational::fraction(numerator, denominator).unwrap();
    let value = match mode {
        "serialized" => {
            let child = if family.ends_with("Rational") {
                serde_json::to_value(rational).unwrap()
            } else {
                serde_json::to_value(Computable::rational(rational)).unwrap()
            };
            let variant: Map<String, Value> =
                [(family.to_owned(), child)].into_iter().collect();
            serde_json::from_value(json!({"internal": variant})).unwrap()
        },
        "public" => {
            let child = Computable::rational(rational);
            match family {
                "SincSmall" => child.clone().sin().multiply(child.inverse()),
                "CoscSmall" => Computable::one()
                    .add(child.clone().cos().negate())
                    .multiply(child.clone().multiply(child).inverse()),
                "PrescaledSin" | "PrescaledSinRational" => child.sin(),
                "PrescaledCos" | "PrescaledCosRational" => child.cos(),
                _ => panic!("unknown family"),
            }
        },
        _ => panic!("unknown mode"),
    };
    println!("{}", json!({"stage":"constructed"}));
    for (index, requested) in [precision, 0, precision - 192, precision]
        .into_iter()
        .enumerate()
    {
        let result = value.approx(requested);
        let unit = Q::from((1, Integer::from(1) << -requested));
        let sample = result
            .as_ref()
            .ok()
            .filter(|n| n.bits() <= 4096)
            .map(|n| n.to_string());
        let valid = sample.as_ref().is_some_and(|sample| {
            let center = Q::from(Integer::from_str_radix(sample, 10).unwrap()) * &unit;
            Q::from(&center - &unit) <= lower && Q::from(&center + &unit) >= upper
        });
        // All selected requests are >= -256 and these functions have absolute
        // value <= 1. A >4096-bit sample therefore violates the same contract
        // without converting an arbitrarily large wrong result to decimal.
        assert!((-256..=0).contains(&requested));
        println!(
            "{}",
            json!({"stage":"sample", "index":index, "precision":requested,
            "numeric":result.is_ok(), "sample":sample, "bits":result.as_ref().ok().map(|n| n.bits()),
            "error":result.as_ref().err().map(|e|e.to_string()), "enclosed":valid})
        );
    }
}
