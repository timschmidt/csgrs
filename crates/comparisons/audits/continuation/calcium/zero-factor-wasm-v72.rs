#[allow(dead_code)]
mod bridge {
    include!("power-sums-public.rs");
    use std::sync::Mutex;

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

    struct Work {
        which: usize,
        policy: usize,
        fresh: bool,
        left: Root,
        right: Root,
        operation: Op,
        expected: Value,
        iterations: usize,
        checksum: usize,
    }
    impl Work {
        fn perform(&self, cases: &[Value]) -> AlgebraicRootBinaryTransformReport {
            let policy = [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512][self.policy];
            if self.fresh {
                let case = &cases[self.which];
                let (a, b) = (input_root(&case["left"]), input_root(&case["right"]));
                transform_algebraic_roots_binary(
                    black_box(&a),
                    black_box(&b),
                    self.operation,
                    policy,
                )
            } else {
                transform_algebraic_roots_binary(
                    black_box(&self.left),
                    black_box(&self.right),
                    self.operation,
                    policy,
                )
            }
        }
    }
    struct State {
        cases: Vec<Value>,
        work: Option<Work>,
        output: Vec<u8>,
    }
    static INPUT: Mutex<Vec<u8>> = Mutex::new(Vec::new());
    static STATE: Mutex<Option<State>> = Mutex::new(None);

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_input(length: usize) -> *mut u8 {
        assert!(length > 0 && length <= 128 * 1024 * 1024);
        assert!(STATE.lock().unwrap().is_none());
        let mut input = INPUT.lock().unwrap();
        assert!(input.is_empty());
        input.resize(length, 0);
        input.as_mut_ptr()
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_init() -> usize {
        let mut state = STATE.lock().unwrap();
        assert!(state.is_none());
        let input = std::mem::take(&mut *INPUT.lock().unwrap());
        let cases: Vec<Value> = serde_json::from_slice(&input).unwrap();
        assert!(!cases.is_empty());
        for (id, case) in cases.iter().enumerate() {
            assert_eq!(case["id"].as_u64().unwrap() as usize, id);
        }
        let count = cases.len();
        *state = Some(State {
            cases,
            work: None,
            output: Vec::new(),
        });
        count
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_prepare(which: usize, policy: usize, fresh: usize) {
        assert!(policy < 2 && fresh < 2);
        let mut state = STATE.lock().unwrap();
        let state = state.as_mut().unwrap();
        assert!(state.work.is_none());
        state.output.clear();
        let case = &state.cases[which];
        let (left, right) = (input_root(&case["left"]), input_root(&case["right"]));
        assert_eq!(wire_root(&left), case["left"]);
        assert_eq!(wire_root(&right), case["right"]);
        let operation = match case["operation"].as_str().unwrap() {
            "Add" => Op::Add,
            "Subtract" => Op::Subtract,
            "Multiply" => Op::Multiply,
            "Divide" => Op::Divide,
            _ => unreachable!(),
        };
        let mut work = Work {
            which,
            policy,
            fresh: fresh != 0,
            left,
            right,
            operation,
            expected: Value::Null,
            iterations: 0,
            checksum: 0,
        };
        work.expected = wire_report(&work.perform(&state.cases));
        for _ in 0..8 {
            black_box(work.perform(&state.cases));
        }
        state.work = Some(work);
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_batch(iterations: usize) -> usize {
        assert!(iterations > 0 && iterations <= 20000);
        let mut state = STATE.lock().unwrap();
        let State { cases, work, .. } = state.as_mut().unwrap();
        let work = work.as_mut().unwrap();
        assert_eq!(work.iterations, 0);
        let mut checksum = 0usize;
        for _ in 0..iterations {
            let report = work.perform(cases);
            checksum = checksum.wrapping_add(
                report
                    .representation
                    .as_ref()
                    .map_or(1, |r| r.polynomial_coefficients.len()),
            );
            black_box(report);
        }
        work.iterations = iterations;
        work.checksum = checksum;
        checksum
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_finish() -> usize {
        let mut state = STATE.lock().unwrap();
        let state = state.as_mut().unwrap();
        let work = state.work.take().unwrap();
        assert!(work.iterations > 0);
        let report = work.perform(&state.cases);
        assert_eq!(wire_report(&report), work.expected);
        let case = &state.cases[work.which];
        assert_eq!(wire_root(&work.left), case["left"]);
        assert_eq!(wire_root(&work.right), case["right"]);
        assert_eq!(
            work.checksum,
            work.iterations
                * report
                    .representation
                    .as_ref()
                    .map_or(1, |r| r.polynomial_coefficients.len())
        );
        state.output = serde_json::to_vec(&json!({"case":work.which,"policy":work.policy,
            "lifecycle":if work.fresh {"fresh"} else {"retained"},"iterations":work.iterations,
            "checksum":work.checksum,"expected":work.expected,"final_matches":true,"sources_unchanged":true})).unwrap();
        state.output.len()
    }

    #[unsafe(no_mangle)]
    pub extern "C" fn zf_output_ptr() -> *const u8 {
        STATE.lock().unwrap().as_ref().unwrap().output.as_ptr()
    }
}
