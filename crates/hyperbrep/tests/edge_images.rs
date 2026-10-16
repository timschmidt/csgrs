#[path = "support/edge_image_queries.rs"]
mod queries;
#[path = "support/edge_images.rs"]
mod support;

#[test]
fn complete_circle_images_preserve_frames_trims_and_both_directions() {
    support::parameterizations();
}

#[test]
fn complete_circle_images_reject_wrong_phase_and_plane_under_ambient_approximation() {
    support::faults();
}

#[test]
fn circle_image_radius_and_angular_rate_are_independent_requirements() {
    queries::parameter_and_radius_failures();
}

#[test]
fn image_query_keeps_unresolved_proofs_strict_across_policy_reuse() {
    queries::unresolved_is_not_success();
}

#[test]
fn image_query_distinguishes_unsupported_from_wrong_correspondence() {
    queries::unsupported_is_not_endpoint_agreement();
}
