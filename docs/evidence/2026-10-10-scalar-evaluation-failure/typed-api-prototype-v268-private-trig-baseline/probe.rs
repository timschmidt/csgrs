use hyperreal::{Computable, Rational, Real};
use serde_json::{Map, Value, json};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let mode = args[1].as_str();
    let numerator: i64 = args[2].parse().unwrap();
    let denominator: i64 = args[3].parse().unwrap();
    let precision: i32 = args[4].parse().unwrap();
    let r = Rational::fraction(numerator, denominator).unwrap();
    let value = match mode {
        "serialized" => {
            let child = serde_json::to_value(Computable::rational(r)).unwrap();
            let variant: Map<String, Value> = [(family.to_owned(), child)].into_iter().collect();
            serde_json::from_value(json!({"internal": variant})).unwrap()
        }
        "public" => match family {
            "SincSmall" => Real::new(r).sinc().unwrap().fold_ref(),
            "CoscSmall" => Real::new(r).cosc().unwrap().fold_ref(),
            "PrescaledSin" => Computable::rational(r).sin(),
            "PrescaledCos" => Computable::rational(r).cos(),
            _ => panic!("unknown family"),
        },
        _ => panic!("unknown mode"),
    };
    for (index, requested) in [precision, 0, precision - 64, precision]
        .into_iter()
        .enumerate()
    {
        let before = format!("{:?}", value.structural_facts());
        let result = value.approx(requested);
        println!(
            "{}",
            json!({"index":index,"family":family,"mode":mode,"numerator":numerator,"denominator":denominator,
                "precision":requested,"before":before,"after":format!("{:?}",value.structural_facts()),
                "sample":result.as_ref().ok().and_then(|n| (n.bits()<=4096).then(|| n.to_string())),
                "bits":result.as_ref().ok().map(|n| n.bits()),"error":result.as_ref().err().map(|e|e.to_string())})
        );
    }
}
