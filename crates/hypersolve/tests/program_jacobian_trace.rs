#![cfg(all(
    feature = "expression-program",
    feature = "serde",
    feature = "dispatch-trace"
))]
#[path = "support/jacobian_trace.rs"]
mod trace;
#[test]
fn derivative_rules_preserve_baseline_outputs_receipts_and_all_dispatch_counts() {
    let (summary, _) = trace::snapshot();
    assert_eq!(summary, include_str!("support/jacobian_trace.txt"));
}
