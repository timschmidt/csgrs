#[path = "support/pcurve_evaluation.rs"]
mod acceptance;

#[test]
fn selected_section_pcurves_preserve_images_and_replay() {
    acceptance::selected_section_pcurves_preserve_images_and_replay();
}

#[test]
fn selected_evaluation_policy_budget_and_recovery() {
    acceptance::selected_evaluation_policy_budget_and_recovery();
}
