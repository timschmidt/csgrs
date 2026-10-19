#[path = "support/selected_fiber_images.rs"]
mod support;
#[test]
fn mixed_affine_parameters_replay_and_compose() {
    support::mixed_affine_parameters_replay_and_compose();
}
#[test]
fn conjugate_selection_similarity_and_policy_recovery() {
    support::conjugate_selection_similarity_and_policy_recovery();
}
