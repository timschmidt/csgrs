// Diagnostic-only module in a frozen native-prototype source tree.
use super::CurveContext;
use std::cell::RefCell;

struct Counts {
    // [actual policy: exact/approximate/preview][bounded flag]
    // [first endpoint: false/true/unknown][radius: false/true/unknown/not run]
    // [repeated endpoint: false/true/unknown/not run].
    rows: [u64; 288],
}

impl Drop for Counts {
    fn drop(&mut self) {
        eprintln!(
            "ARC_ENDPOINT_RETRY_DIAGNOSTIC {{\"thread\":\"{:?}\",\"rows\":{:?}}}",
            std::thread::current().id(),
            self.rows,
        );
    }
}

thread_local! {
    static COUNTS: RefCell<Counts> = const { RefCell::new(Counts { rows: [0; 288] }) };
}

fn result_index(value: Option<bool>) -> usize {
    match value {
        Some(false) => 0,
        Some(true) => 1,
        None => 2,
    }
}

pub(super) fn observe(
    policy: &CurveContext,
    first_endpoint: Option<bool>,
    radius: Option<Option<bool>>,
    repeated_endpoint: Option<Option<bool>>,
) {
    let mode = if policy.is_edge_preview() {
        2
    } else if policy.permits_approximate_512() {
        1
    } else {
        0
    };
    let bounded = usize::from(policy.has_bounded_exact_predicate_budget());
    let index = (mode * 2 + bounded) * 48
        + result_index(first_endpoint) * 16
        + radius.map_or(3, result_index) * 4
        + repeated_endpoint.map_or(3, result_index);
    COUNTS.with(|counts| counts.borrow_mut().rows[index] += 1);
}
