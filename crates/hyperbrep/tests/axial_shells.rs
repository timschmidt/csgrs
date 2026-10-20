#[allow(dead_code)]
#[path = "support/closed_cylinder.rs"]
mod closed_cylinder;
#[path = "support/axial_shells.rs"]
mod support;
#[test]
fn axial_shell_shifted_periodic_seams() {
    support::periodic_workflow(
        closed_cylinder::build(hyperbrep::Real::zero(), -100, false).unwrap(),
    );
    support::periodic_workflow(closed_cylinder::build(hyperbrep::Real::zero(), 100, true).unwrap());
}
#[test]
fn axial_shell_scaled_cold_continuation() {
    support::scaled_workflow();
}
#[test]
fn axial_shell_upper_thin_layer() {
    support::workflow(0, false);
}
#[test]
fn axial_shell_lower_thin_layer() {
    support::workflow(1, false);
}
#[test]
fn axial_shell_interior_notch() {
    support::workflow(2, false);
}
#[test]
fn axial_shell_reversed_covering_difference() {
    support::workflow(3, false);
}
#[test]
fn axial_shell_reflected_upper() {
    support::workflow(0, true);
}
#[test]
fn axial_shell_reflected_lower() {
    support::workflow(1, true);
}
#[test]
fn axial_shell_reflected_interior() {
    support::workflow(2, true);
}
#[test]
fn axial_shell_reflected_reverse() {
    support::workflow(3, true);
}
