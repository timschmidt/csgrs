#[allow(dead_code)]
mod benchmark {
    include!("power-sums-public.rs");

    fn input_root(value: &Value) -> Root {
        let parse = |v: &Value| Real::from(v.as_str().unwrap().parse::<Rational>().unwrap());
        assert_eq!(value["validation"], "Valid");
        Root {
            constraint_index: value["constraint"].as_u64().unwrap() as usize,
            symbol: SymbolId(value["symbol"].as_u64().unwrap() as u32),
            interval_index: value["intervalIndex"].as_u64().unwrap() as usize,
            polynomial_coefficients: value["polynomial"]
                .as_array()
                .unwrap()
                .iter()
                .map(parse)
                .collect(),
            interval: IsolatedRootInterval {
                lower: parse(&value["lower"]),
                upper: parse(&value["upper"]),
                exact_root: (!value["exact"].is_null()).then(|| parse(&value["exact"])),
                distinct_root_count: value["count"].as_u64().unwrap() as usize,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        }
    }
    fn wall() -> String {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
            .to_string()
    }
    pub fn run_cost(snapshot: Option<fn(bool) -> [usize; 4]>) {
        let args: Vec<String> = std::env::args().collect();
        let mode = &args[1];
        assert!(matches!(mode.as_str(), "check" | "cpu" | "allocation"));
        if mode != "check" {
            assert_eq!(mode == "allocation", snapshot.is_some());
        }
        let cases: Vec<Value> =
            serde_json::from_str(&std::fs::read_to_string(&args[2]).unwrap()).unwrap();
        let groups: Vec<Value> =
            serde_json::from_str(&std::fs::read_to_string(&args[3]).unwrap()).unwrap();
        for group in &groups {
            let id = group["case"].as_u64().unwrap() as usize;
            let case = &cases[id];
            assert_eq!(case["id"].as_u64().unwrap() as usize, id);
            let policy_id = group["policy"].as_u64().unwrap() as usize;
            let policy = [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512][policy_id];
            let lifecycle = group["lifecycle"].as_str().unwrap();
            assert!(matches!(lifecycle, "fresh" | "retained"));
            let iterations = group["iterations"].as_u64().unwrap() as usize;
            assert!(iterations > 0 && iterations <= 20000);
            let op = match case["operation"].as_str().unwrap() {
                "Add" => Op::Add,
                "Subtract" => Op::Subtract,
                "Multiply" => Op::Multiply,
                "Divide" => Op::Divide,
                _ => unreachable!(),
            };
            let (left, right) = (input_root(&case["left"]), input_root(&case["right"]));
            assert_eq!(wire_root(&left), case["left"]);
            assert_eq!(wire_root(&right), case["right"]);
            let invoke = || {
                if lifecycle == "fresh" {
                    let (a, b) = (input_root(&case["left"]), input_root(&case["right"]));
                    transform_algebraic_roots_binary(black_box(&a), black_box(&b), op, policy)
                } else {
                    transform_algebraic_roots_binary(
                        black_box(&left),
                        black_box(&right),
                        op,
                        policy,
                    )
                }
            };
            let expected = wire_report(&invoke());
            for _ in 0..8 {
                black_box(invoke());
            }
            let wall_before = wall();
            let before = snapshot.map_or([0; 4], |s| s(true));
            let start = Instant::now();
            let mut checksum = 0usize;
            for _ in 0..iterations {
                let report = invoke();
                checksum = checksum.wrapping_add(
                    report
                        .representation
                        .as_ref()
                        .map_or(1, |r| r.polynomial_coefficients.len()),
                );
                black_box(report);
            }
            let elapsed_ns = start.elapsed().as_nanos();
            let after = snapshot.map_or([0; 4], |s| s(false));
            let wall_after = wall();
            assert_eq!(wire_report(&invoke()), expected);
            assert_eq!(wire_root(&left), case["left"]);
            assert_eq!(wire_root(&right), case["right"]);
            println!(
                "{}",
                json!({"mode":mode,"case":id,"policy":policy_id,"lifecycle":lifecycle,"iterations":iterations,
                "expected":expected,"checksum":checksum,"elapsed_ns":elapsed_ns,"wall_before":wall_before,"wall_after":wall_after,
                "requests":after[0]-before[0],"requested_bytes":after[1]-before[1],"start_live":before[2],"end_live":after[2],
                "live_delta":after[2] as i128-before[2] as i128,"peak_delta":after[3]-before[2],"final_matches":true})
            );
        }
        println!(
            "{}",
            json!({"terminal":true,"groups":groups.len(),"mode":mode})
        );
    }
}
