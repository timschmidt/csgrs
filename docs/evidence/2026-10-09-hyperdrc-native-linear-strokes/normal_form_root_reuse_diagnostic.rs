// Diagnostic-only inclusion in an isolated source tree; never production code.
mod normal_form_root_reuse_diagnostic {
    use super::{Approximation, Computable, Node, PiLaurentPolynomial};
    use std::cell::RefCell;
    use std::collections::HashMap;
    use std::sync::{Arc, Weak};

    const CAPACITY: usize = 16_384;
    struct Seen {
        node: Weak<Node>,
        covered: bool,
    }
    #[derive(Default)]
    struct Counts {
        zero_work_queries: u64,
        covered_to_uncovered: u64,
        uncovered_to_covered: u64,
        budgets: std::collections::BTreeMap<usize, [u64; 4]>,
        resets: u64,
        nodes_inserted: u64,
        // Per root kind: computed, covered, repeated identity, repeated covered,
        // shared covered, repeated shared covered, work, repeated covered work.
        rows: [[u64; 8]; 9],
        nodes: HashMap<(usize, usize), Seen>,
    }
    impl Drop for Counts {
        fn drop(&mut self) {
            eprintln!(
                "NORMAL_FORM_ROOT_REUSE_DIAGNOSTIC {{\"thread\":\"{:?}\",\"capacity\":{},\"zero_work_queries\":{},\"covered_to_uncovered\":{},\"uncovered_to_covered\":{},\"budget_rows\":{:?},\"window_resets\":{},\"nodes_inserted\":{},\"final_window_entries\":{},\"rows\":{:?}}}",
                std::thread::current().id(),
                CAPACITY,
                self.zero_work_queries,
                self.covered_to_uncovered,
                self.uncovered_to_covered,
                self.budgets.iter().map(|(budget, row)| [*budget as u64, row[0], row[1], row[2], row[3]]).collect::<Vec<_>>(),
                self.resets,
                self.nodes_inserted,
                self.nodes.len(),
                self.rows,
            );
        }
    }
    thread_local! {
        static COUNTS: RefCell<Counts> = RefCell::new(Counts::default());
    }
    pub(super) fn computed(
        value: &Computable,
        budget: usize,
        shared: bool,
        work: usize,
        result: Option<&PiLaurentPolynomial>,
    ) {
        let kind = match &value.internal.approximation {
            Approximation::Add(..) => 0,
            Approximation::Multiply(..) => 1,
            Approximation::Negate(..) => 2,
            Approximation::Offset(..) => 3,
            Approximation::Inverse(..) => 4,
            Approximation::Square(..) => 5,
            Approximation::Sqrt(..) => 6,
            Approximation::LinearCombination3(..) => 7,
            _ => 8,
        };
        let covered = result.is_some_and(|polynomial| {
            polynomial
                .terms
                .iter()
                .all(|(_, coefficient)| coefficient.atoms.is_empty())
        });
        let key = (Arc::as_ptr(&value.internal) as usize, budget);
        COUNTS.with(|counts| {
            let mut counts = counts.borrow_mut();
            if counts.nodes.len() == CAPACITY && !counts.nodes.contains_key(&key) {
                counts.nodes.clear();
                counts.resets += 1;
            }
            let previous = counts.nodes.get_mut(&key).map(|seen| {
                // Weak references prevent address reuse within a window, and
                // never increase the strong count used by the real reducer.
                assert_eq!(seen.node.as_ptr(), Arc::as_ptr(&value.internal));
                assert!(seen.node.strong_count() > 0);
                let was_covered = seen.covered;
                seen.covered = covered;
                was_covered
            });
            if previous.is_none() {
                counts.nodes.insert(
                    key,
                    Seen {
                        node: Arc::downgrade(&value.internal),
                        covered,
                    },
                );
                counts.nodes_inserted += 1;
            }
            counts.zero_work_queries += u64::from(work == 0);
            counts.covered_to_uncovered += u64::from(previous == Some(true) && !covered);
            counts.uncovered_to_covered += u64::from(previous == Some(false) && covered);
            let repeated = previous.is_some();
            let repeated_covered = covered && previous == Some(true);
            let budget_row = counts.budgets.entry(budget).or_default();
            budget_row[0] += 1;
            budget_row[1] += u64::from(covered);
            budget_row[2] += u64::from(repeated_covered);
            budget_row[3] += work as u64;
            let row = &mut counts.rows[kind];
            row[0] += 1;
            row[1] += u64::from(covered);
            row[2] += u64::from(repeated);
            row[3] += u64::from(repeated_covered);
            row[4] += u64::from(covered && shared);
            row[5] += u64::from(repeated_covered && shared);
            row[6] += work as u64;
            row[7] += if repeated_covered { work as u64 } else { 0 };
        });
    }
}
