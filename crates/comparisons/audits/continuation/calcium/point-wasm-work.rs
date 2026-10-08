struct HistoryWork {
    which: usize,
    policy: usize,
    state: usize,
    fresh: bool,
    left: Root,
    right: Root,
    operation: Op,
    before_left: Value,
    before_right: Value,
    expected: Value,
}

impl HistoryWork {
    fn new(which: usize, policy: usize, state: usize, fresh: bool) -> Self {
        assert!(which < 48 && policy < 2 && state < 4);
        let (mut left, mut right, operation) = case(which);
        history(&mut left, state);
        history(&mut right, state);
        let before_left = wire_root(&left);
        let before_right = wire_root(&right);
        let mut work = Self {
            which,
            policy,
            state,
            fresh,
            left,
            right,
            operation,
            before_left,
            before_right,
            expected: Value::Null,
        };
        for _ in 0..8 {
            black_box(work.perform());
        }
        work.expected = wire_report(&work.perform());
        work
    }

    fn perform(&self) -> Report {
        if self.fresh {
            let (mut a, mut b, operation) = case(self.which);
            history(&mut a, self.state);
            history(&mut b, self.state);
            query(&a, &b, operation, self.policy)
        } else {
            query(&self.left, &self.right, self.operation, self.policy)
        }
    }

    fn batch(&self, iterations: usize) -> (Report, usize) {
        assert!(iterations > 0);
        let mut checksum = 0usize;
        for _ in 1..iterations {
            let report = self.perform();
            checksum = checksum.wrapping_add(
                report
                    .representation
                    .as_ref()
                    .map_or(1, |r| r.polynomial_coefficients.len()),
            );
            black_box(report);
        }
        let report = self.perform();
        checksum = checksum.wrapping_add(
            report
                .representation
                .as_ref()
                .map_or(1, |r| r.polynomial_coefficients.len()),
        );
        (report, checksum)
    }

    fn finish(&self, report: &Report, checksum: usize, iterations: usize) -> Value {
        let actual = wire_report(report);
        assert_eq!(
            actual, self.expected,
            "final measured full result changed after preconditioning"
        );
        assert_eq!(wire_root(&self.left), self.before_left);
        assert_eq!(wire_root(&self.right), self.before_right);
        assert_eq!(
            checksum,
            iterations
                * report
                    .representation
                    .as_ref()
                    .map_or(1, |r| r.polynomial_coefficients.len())
        );
        json!({"case":self.which,"policy":self.policy,"history":self.state,
            "lifecycle":if self.fresh {"fresh"} else {"retained"},
            "iterations":iterations,"checksum":checksum,"left":self.before_left,"right":self.before_right,
            "actual":actual,"final_matches_preconditioned":true,"input_records_unchanged":true})
    }
}
