#[path = "support/surface_differentials.rs"]
mod support;

#[test]
fn complete_circle_extrusions_and_both_pcurve_consumers() {
    support::complete_circle_extrusions_and_both_pcurve_consumers();
}
#[test]
fn cubic_and_projective_chain_rules() {
    support::cubic_and_projective_chain_rules();
}
#[test]
fn selected_tensors_normals_and_native_knot_charts() {
    support::selected_tensors_normals_and_native_knot_charts();
}
#[test]
fn native_vector_algebra_affine_transport_and_replay() {
    support::native_vector_algebra_affine_transport_and_replay();
}
#[test]
fn selected_parallel_oriented_unit_normals() {
    support::selected_parallel_oriented_unit_normals();
}
#[test]
fn singular_domains_policy_and_budget_recovery() {
    support::singular_domains_policy_and_budget_recovery();
}

#[test]
fn nonsmooth_knot_ownership_and_reflected_surface_normals() {
    support::nonsmooth_knot_ownership_and_reflected_surface_normals();
}
