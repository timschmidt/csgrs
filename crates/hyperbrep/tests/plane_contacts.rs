#[path = "support/plane_contacts.rs"]
mod acceptance;

#[test]
fn cubic_contacts_retain_original_source_and_certify_independent_equations() {
    acceptance::cubic_contacts_retain_original_source_and_certify_independent_equations();
}

#[test]
fn complete_root_sets_keep_repeated_roots_endpoints_and_distinct_visits() {
    acceptance::complete_root_sets_keep_repeated_roots_endpoints_and_distinct_visits();
}

#[test]
fn high_degree_empty_and_contained_results_are_complete() {
    acceptance::high_degree_empty_and_contained_results_are_complete();
}

#[test]
fn nonunit_nurbs_contacts_use_original_curve_parameters() {
    acceptance::nonunit_nurbs_contacts_use_original_curve_parameters();
}

#[test]
fn nurbs_coincidence_keeps_all_ranges_and_isolated_contacts() {
    acceptance::nurbs_coincidence_keeps_all_ranges_and_isolated_contacts();
}

#[test]
fn nurbs_joint_classification_preserves_each_incident_span() {
    acceptance::nurbs_joint_classification_preserves_each_incident_span();
}

#[test]
fn parallel_plane_contacts_preserve_reversal_frame_and_tangency() {
    acceptance::parallel_plane_contacts_preserve_reversal_frame_and_tangency();
}

#[test]
fn planar_nurbs_overlaps_use_authored_spans_before_spatial_transport() {
    acceptance::planar_nurbs_overlaps_use_authored_spans_before_spatial_transport();
}

#[test]
fn plane_contacts_keep_tiny_exact_separations() {
    acceptance::plane_contacts_keep_tiny_exact_separations();
}

#[test]
fn contact_parameters_split_shared_edges_and_replay_solid_operations() {
    acceptance::contact_parameters_split_shared_edges_and_replay_solid_operations();
}

#[test]
fn plane_contact_policies_exhaustion_and_recovery_preserve_certainty() {
    acceptance::plane_contact_policies_exhaustion_and_recovery_preserve_certainty();
}

#[test]
fn plane_contacts_reject_poles_before_empty_or_contained_shortcuts() {
    acceptance::plane_contacts_reject_poles_before_empty_or_contained_shortcuts();
}

#[test]
fn collapsed_planar_spans_keep_parameter_components() {
    acceptance::collapsed_planar_spans_keep_parameter_components();
}

#[test]
fn finite_cone_boundary_contacts_are_not_claimed_as_equation_roots() {
    acceptance::finite_cone_boundary_contacts_are_not_claimed_as_equation_roots();
}
