use hyperreal::{Computable, Rational};
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

fn argument(shape: &str, shift: i32) -> Computable {
    let pi = leaf(Computable::pi());
    let half_pi = offset(pi.clone(), -1);
    let value = match shape {
        "offset_pi" => offset(pi, shift),
        "inverse_offsets" => offset(offset(pi, shift), -shift),
        "half_via_extremes" => offset(offset(pi, i32::MAX), i32::MIN),
        "shifted_plus_half" => add(offset(pi, shift), half_pi),
        "factored_plus_half" => {
            add(product(pi, offset(leaf(Computable::one()), shift)), half_pi)
        },
        "rational_split" => {
            let third = leaf(Computable::rational(Rational::fraction(1, 3).unwrap()));
            let scaled = product(pi, third);
            add(offset(scaled.clone(), shift), offset(scaled, shift + 1))
        },
        "deep_inverse_offsets" => {
            let mut value = pi;
            for _ in 0..32 {
                value = offset(offset(value, shift), -shift);
            }
            value
        },
        _ => panic!("unknown shape"),
    };
    serde_json::from_value(value).unwrap()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let mode = args[1].as_str();
    let shape = args[2].as_str();
    let shift: i32 = args[3].parse().unwrap();
    let index: usize = args[4].parse().unwrap();
    let sine = [0_i128, 1, 0, -1][index];
    let cosine = [1_i128, 0, -1, 0][index];
    let expected = match family {
        "sin" => Some(sine),
        "cos" => Some(cosine),
        "tan" => (cosine != 0).then_some(0),
        "cot" => (sine != 0).then_some(0),
        _ => panic!("unknown family"),
    };
    println!(
        "{}",
        json!({"stage":"oracle", "half_pi_index":index, "expected_integer":expected})
    );
    let argument = argument(shape, shift);
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
    for precision in [1_i32, -8, -64, -8] {
        let result = value.approx(precision);
        let sample = result
            .as_ref()
            .ok()
            .and_then(|n| (n.bits() < 127).then(|| n.to_string().parse::<i128>().unwrap()));
        let error = result.as_ref().err().map(|e| format!("{e:?}"));
        let valid = match (expected, sample) {
            (Some(expected), Some(sample)) if precision == 1 => sample
                .checked_mul(2)
                .is_some_and(|center| center.abs_diff(expected) <= 2),
            (Some(expected), Some(sample)) => sample.abs_diff(expected << -precision) <= 1,
            (None, None) => error.as_deref() == Some("DivideByZero"),
            _ => false,
        };
        println!(
            "{}",
            json!({"stage":"sample", "precision":precision,
            "sample":sample.map(|n|n.to_string()), "error":error,
            "numeric":result.is_ok(), "valid":valid})
        );
    }
}
