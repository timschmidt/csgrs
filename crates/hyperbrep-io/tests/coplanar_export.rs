#[path = "support/coplanar_export.rs"]
mod acceptance;

#[test]
fn repeated_curved_booleans_complete_exchange_and_recover() {
    acceptance::exchange_and_recovery();
}
