#[path = "support/sector_export.rs"]
mod acceptance;

#[test]
fn analytic_sectors_report_exchange_blockers_then_recover() {
    acceptance::failures_and_recovery();
}
