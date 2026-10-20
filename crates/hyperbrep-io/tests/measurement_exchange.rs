#[path = "support/measurement_exchange.rs"]
mod acceptance;

#[test]
fn independent_quintic_step_admits_and_replays_without_measurement() {
    acceptance::independent_quintic_step_admits_and_replays_without_measurement();
}

#[test]
fn malformed_quintic_step_and_budget_failure_cannot_publish_geometry() {
    acceptance::malformed_quintic_step_and_budget_failure_cannot_publish_geometry();
}
