use hyperreal::{Computable, Rational};
use serde_json::{Map, Value, json};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let mode = args[1].as_str();
    let base = args[2].as_str();
    let numerator: i64 = args[3].parse().unwrap();
    let denominator: u64 = args[4].parse().unwrap();
    let perturbation: i64 = args[5].parse().unwrap();
    let precision: i32 = args[6].parse().unwrap();
    let mut c = Computable::rational(Rational::fraction(numerator, denominator).unwrap());
    if base == "pi" {
        c = Computable::pi().multiply(c);
    } else {
        assert_eq!(base, "rational");
    }
    if perturbation != 0 {
        c = c.add(Computable::rational(
            Rational::fraction(perturbation, 1024).unwrap(),
        ));
    }
    let value = match mode {
        "serialized" => {
            let variant: Map<String, Value> =
                [(family.to_owned(), serde_json::to_value(c).unwrap())]
                    .into_iter()
                    .collect();
            serde_json::from_value(json!({"internal": variant})).unwrap()
        },
        "public" => match family {
            "PrescaledAtan" => c.atan(),
            "PrescaledAsinh" => c.asinh(),
            "PrescaledTan" | "TanDeferred" => c.tan(),
            "PrescaledCot" => c.clone().cos().multiply(c.sin().inverse()),
            _ => panic!("unknown family"),
        },
        _ => panic!("unknown mode"),
    };
    println!("{}", json!({"stage":"constructed"}));
    let result = value.approx(precision);
    println!(
        "{}",
        json!({"stage":"result","family":family,"mode":mode,"base":base,
            "numerator":numerator,"denominator":denominator,"perturbation":perturbation,"precision":precision,
            "sample":result.as_ref().ok().and_then(|n| (n.bits()<=4096).then(|| n.to_string())),
            "bits":result.as_ref().ok().map(|n| n.bits()),"error":result.as_ref().err().map(|e|e.to_string())})
    );
}
