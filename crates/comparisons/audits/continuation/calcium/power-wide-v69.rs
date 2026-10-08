// Reuse the previously qualified wire format and the actual public API.
#[allow(dead_code)]
mod public {
    include!("power-sums-public.rs");

    fn input_root(value: &Value, symbol: u32) -> Root {
        let point = Real::from(value["point"].as_str().unwrap().parse::<Rational>().unwrap());
        Root {
            constraint_index: 7,
            symbol: SymbolId(symbol),
            interval_index: 3,
            polynomial_coefficients: value["polynomial"]
                .as_array()
                .unwrap()
                .iter()
                .map(|s| Real::from(s.as_str().unwrap().parse::<Rational>().unwrap()))
                .collect(),
            interval: IsolatedRootInterval {
                lower: point.clone(),
                upper: point.clone(),
                exact_root: Some(point),
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }

    pub fn emit(row: &Value, policy: usize) {
        let left = input_root(&row["left"], 11);
        let right = input_root(&row["right"], 13);
        let before_left = wire_root(&left);
        let before_right = wire_root(&right);
        let report = transform_algebraic_roots_binary(
            &left,
            &right,
            operation(row["op"].as_u64().unwrap() as usize),
            [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512][policy],
        );
        assert_eq!(before_left, wire_root(&left));
        assert_eq!(before_right, wire_root(&right));
        println!(
            "{}",
            json!({"id":row["id"],"policy":policy,"left":before_left,
                "right":before_right,"report":wire_report(&report)})
        );
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("input path");
    let data: Vec<serde_json::Value> =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    for (id, row) in data.iter().enumerate() {
        assert_eq!(row["id"].as_u64().unwrap() as usize, id);
        for policy in 0..2 {
            public::emit(row, policy);
        }
    }
    println!(
        "{}",
        serde_json::json!({"terminal":true,"cases":data.len(),"policies":2,"rows":data.len()*2})
    );
}
