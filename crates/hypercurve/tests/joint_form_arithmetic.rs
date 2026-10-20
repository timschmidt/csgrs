#[path = "support/joint_form_arithmetic.rs"]
mod support;

#[test]
fn polynomial_images_and_original_source_arithmetic() {
    support::polynomial_images_and_original_source_arithmetic();
}
#[test]
fn native_parameter_fields_share_rational_queries() {
    support::native_parameter_fields_share_rational_queries();
}
#[test]
fn constants_divisors_and_authority_guards() {
    support::constants_divisors_and_authority_guards();
}
#[test]
fn different_fiber_selectors_do_not_merge() {
    support::different_fiber_selectors_do_not_merge();
}
#[test]
fn policy_budget_and_finite_composition_recover() {
    support::policy_budget_and_finite_composition_recover();
}

#[test]
fn common_denominator_composition_retains_bounded_degree() {
    support::common_denominator_composition_retains_bounded_degree();
}

#[test]
fn shared_operand_debug_does_not_expand_the_dag() {
    support::shared_operand_debug_does_not_expand_the_dag();
}
