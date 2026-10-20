#[path = "support/section_differentials.rs"]
mod support;

#[test]
fn regular_sections_restriction_transform_and_replay() {
    support::regular_sections_restriction_transform_and_replay();
}
#[test]
fn selected_parameters_and_native_spatial_vectors() {
    support::selected_parameters_and_native_spatial_vectors();
}
#[test]
fn cubic_and_projective_section_jets() {
    support::cubic_and_projective_section_jets();
}
#[test]
fn smooth_vertical_events_and_singular_frontiers() {
    support::smooth_vertical_events_and_singular_frontiers();
}
#[test]
fn resource_domain_guards_and_policy_recovery() {
    support::resource_domain_guards_and_policy_recovery();
}

#[test]
fn selected_parallel_vector_signs() {
    support::selected_parallel_vector_signs();
}
