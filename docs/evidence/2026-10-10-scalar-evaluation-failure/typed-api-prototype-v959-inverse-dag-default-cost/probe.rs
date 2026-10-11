use hyperreal::{Computable, Rational};
use serde_json::{Value, json};
use std::{hint::black_box, time::Instant};

fn raw(name: &str, argument: Value) -> Value {
    let fields: serde_json::Map<String, Value> =
        [(name.to_owned(), argument)].into_iter().collect();
    json!({"internal":fields})
}

fn leaf(n: i64, d: u64) -> Value {
    serde_json::to_value(Computable::rational(Rational::fraction(n, d).unwrap())).unwrap()
}

fn fixture(case: &str) -> Computable {
    let pi = serde_json::to_value(Computable::pi()).unwrap();
    let period = raw("Offset", json!([pi, i32::MAX]));
    let sqrt2 = raw("Sqrt", leaf(2, 1));
    let graph = match case {
        "cold_sin" => return Computable::rational(Rational::fraction(1, 7).unwrap()).sin(),
        "cold_opaque" | "shared_hot" => {
            raw("PrescaledSin", raw("Add", json!([period, sqrt2])))
        },
        "structural_cancel" => {
            let mut atom = sqrt2;
            for _ in 0..64 {
                atom = raw("Offset", json!([atom, 0]));
            }
            atom = raw("PrescaledAtan", atom);
            let cancellation = raw("Add", json!([atom.clone(), raw("Negate", atom)]));
            raw("PrescaledSin", raw("Add", json!([period, cancellation])))
        },
        "domain_tree" => {
            let mut tree = sqrt2;
            for _ in 0..8 {
                tree = raw("Add", json!([tree.clone(), tree]));
            }
            tree
        },
        "nested_exp" => raw(
            "PrescaledExp",
            raw("Add", json!([raw("LnDeferred", leaf(2, 1)), leaf(1, 7)])),
        ),
        _ => panic!("unknown workload"),
    };
    serde_json::from_value(graph).unwrap()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let case = &args[0];
    let count: usize = args[1].parse().unwrap();
    let precision = if case == "shared_hot" { -64 } else { -128 };
    // Warm global constants consistently, keeping expression construction and
    // the warm-up outside the timed interval. Cold roots remain independent.
    black_box(fixture(case).approx(-256).unwrap());
    let values: Vec<_> = if case == "shared_hot" {
        let value = fixture(case);
        black_box(value.approx(-256).unwrap());
        (0..count).map(|_| value.clone()).collect()
    } else {
        (0..count).map(|_| fixture(case)).collect()
    };
    let mut checksum = Computable::zero().approx(0).unwrap();
    let start = Instant::now();
    for value in &values {
        checksum += black_box(value.approx(precision).unwrap());
    }
    let elapsed = start.elapsed().as_nanos();
    println!(
        "{}",
        json!({"case":case,"iterations":count,"precision":precision,
        "elapsed_ns":elapsed.to_string(),"checksum":checksum.to_string()})
    );
}
