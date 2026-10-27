#[path = "support/polynomial_curves.rs"]
mod support;

#[test]
fn closed_loop_evaluation_split_reverse_and_source_identity() {
    support::closed_loop_evaluation_split_reverse_and_source_identity();
}
#[test]
fn algebraic_slice_keeps_correlated_noncircular_fiber() {
    support::algebraic_slice_keeps_correlated_noncircular_fiber();
}
#[test]
fn similarity_and_point_replay_keep_source_chart() {
    support::similarity_and_point_replay_keep_source_chart();
}
#[test]
fn isolated_empty_and_singular_contacts_remain_distinct() {
    support::isolated_empty_and_singular_contacts_remain_distinct();
}
#[test]
fn forged_selector_rejection_and_budget_recovery() {
    support::forged_selector_rejection_and_budget_recovery();
}

#[test]
fn algebraic_events_and_tiny_loops_preserve_selected_endpoints() {
    support::algebraic_events_and_tiny_loops_preserve_selected_endpoints();
}

#[test]
fn every_similarity_branch_keeps_exact_coordinate_comparisons() {
    support::every_similarity_branch_keeps_exact_coordinate_comparisons();
}

#[test]
fn public_source_restrictions_keep_polynomial_bounds_and_traversal() {
    support::public_source_restrictions_keep_polynomial_bounds_and_traversal();
}

#[path = "support/polynomial_sections.rs"]
mod sections;

#[test]
fn quintic_section_splits_both_operands_and_continues() {
    sections::quintic_section_splits_both_operands_and_continues();
}

#[test]
fn quintic_sections_respect_both_active_domains() {
    sections::quintic_sections_respect_both_active_domains();
}

#[test]
fn vertical_tangent_and_cusp_events_are_not_transverse() {
    sections::vertical_tangent_and_cusp_events_are_not_transverse();
}

#[test]
fn polynomial_pair_sections_and_region_admission() {
    sections::polynomial_pair_sections_and_region_admission();
}

#[test]
fn mapped_quintic_line_sections_keep_source_parameters() {
    sections::mapped_quintic_line_sections_keep_source_parameters();
}

#[test]
fn selected_line_parameters_replay_their_transformed_fiber() {
    sections::selected_line_parameters_replay_their_transformed_fiber();
}

#[test]
fn mapped_sections_preserve_singular_and_tangent_contacts() {
    sections::mapped_sections_preserve_singular_and_tangent_contacts();
}

#[test]
fn mapped_sections_survive_budget_retry_and_policy_changes() {
    sections::mapped_sections_survive_budget_retry_and_policy_changes();
}

#[test]
fn sections_survive_context_drop_budgets_and_policies() {
    sections::sections_survive_context_drop_budgets_and_policies();
}

#[test]
fn sloped_section_splits_both_operands_and_continues() {
    sections::sloped_section_splits_both_operands_and_continues();
}

#[test]
fn horizontal_section_keeps_only_its_own_branch_roots() {
    sections::horizontal_section_keeps_only_its_own_branch_roots();
}

#[test]
fn tangent_lines_and_line_components_keep_their_meaning() {
    sections::tangent_lines_and_line_components_keep_their_meaning();
}
