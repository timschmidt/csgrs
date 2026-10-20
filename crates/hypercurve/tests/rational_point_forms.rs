#[path = "support/rational_point_forms.rs"]
mod support;

#[test]
fn scalar_chart_images_keep_their_joint_source() {
    support::scalar_chart_images_keep_their_joint_source();
}

#[test]
fn joint_identities_and_original_poles() {
    support::joint_identities_and_original_poles();
}

#[test]
fn rational_queries_preserve_similarity_and_cold_sources() {
    support::rational_queries_preserve_similarity_and_cold_sources();
}

#[test]
fn rational_curve_source_and_authored_values() {
    support::rational_curve_source_and_authored_values();
}

#[test]
fn rational_queries_recover_after_budget_refusal() {
    support::rational_queries_recover_after_budget_refusal();
}
#[test]
fn independent_support_fields_keep_rational_identities() {
    support::independent_support_fields_keep_rational_identities();
}
