#[path = "support/seam_export.rs"]
mod acceptance;

#[test]
fn seam_cut_exchange_reports_unsupported_surfaces_then_recovers() {
    acceptance::failures_and_recovery();
}
