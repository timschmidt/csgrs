#[path = "support/seam_export.rs"]
mod acceptance;

#[test]
fn seam_cut_exchange_preserves_exact_geometry_and_recovers() {
    acceptance::exchange_and_recovery();
}
