#[path = "support/plane_tensor_sections.rs"]
mod support;

#[test]
fn original_full_interior_loop() {
    support::original_full_interior_loop();
}
#[test]
fn full_loops_and_nonlinear_weighted_images() {
    support::full_loops_and_nonlinear_weighted_images();
}
#[test]
fn noncircular_singular_and_isolated_sections() {
    support::noncircular_singular_and_isolated_sections();
}
#[test]
fn rational_image_poles_and_replay_guards() {
    support::rational_image_poles_and_replay_guards();
}
#[test]
fn image_refusal_budget_and_recovery() {
    support::image_refusal_budget_and_recovery();
}

#[test]
fn disconnected_mixed_and_near_coincident_sections() {
    support::disconnected_mixed_and_near_coincident_sections();
}

#[test]
fn empty_contained_boundary_and_solver_frontier() {
    support::empty_contained_boundary_and_solver_frontier();
}

#[test]
fn repeated_section_native_replay_is_cold() {
    support::repeated_section_native_replay_is_cold();
}
