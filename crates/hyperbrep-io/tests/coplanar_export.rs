#[path = "support/coplanar_export.rs"]
mod acceptance;

#[test]
fn repeated_curved_booleans_report_exchange_blockers_then_recover() {
    acceptance::failures_and_recovery();
}
