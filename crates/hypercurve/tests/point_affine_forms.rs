#[path = "support/point_affine_forms.rs"]
mod acceptance;

#[test]
fn authored_and_constant_forms_preserve_exact_values() {
    acceptance::authored_and_constant_forms_preserve_exact_values();
}
#[test]
fn selected_forms_preserve_root_correlation_and_cancellation() {
    acceptance::selected_forms_preserve_root_correlation_and_cancellation();
}
#[test]
fn selected_form_bounds_and_policy_reuse_keep_exact_meaning() {
    acceptance::selected_form_bounds_and_policy_reuse_keep_exact_meaning();
}
#[test]
fn analytic_form_incidence_preserves_the_positive_normal_sheet() {
    acceptance::analytic_form_incidence_preserves_the_positive_normal_sheet();
}
#[test]
fn symbolic_forms_keep_correlated_coefficients() {
    acceptance::symbolic_forms_keep_correlated_coefficients();
}
#[test]
fn parallel_injectivity_requires_the_consumed_range() {
    acceptance::parallel_injectivity_requires_the_consumed_range();
}
