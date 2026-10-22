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
fn unadmitted_polynomial_pairs_keep_typed_blockers() {
    sections::unadmitted_polynomial_pairs_keep_typed_blockers();
}

#[test]
fn sections_survive_context_drop_budgets_and_policies() {
    sections::sections_survive_context_drop_budgets_and_policies();
}
