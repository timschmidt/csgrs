#[path = "support/axial_shells.rs"]
mod acceptance;
#[test]
fn axial_shell_step_0_false() {
    acceptance::round_trip(0, false).unwrap();
}
#[test]
fn axial_shell_step_0_true() {
    acceptance::round_trip(0, true).unwrap();
}
#[test]
fn axial_shell_step_1_false() {
    acceptance::round_trip(1, false).unwrap();
}
#[test]
fn axial_shell_step_1_true() {
    acceptance::round_trip(1, true).unwrap();
}
#[test]
fn axial_shell_step_2_false() {
    acceptance::round_trip(2, false).unwrap();
}
#[test]
fn axial_shell_step_2_true() {
    acceptance::round_trip(2, true).unwrap();
}
#[test]
fn axial_shell_step_3_false() {
    acceptance::round_trip(3, false).unwrap();
}
#[test]
fn axial_shell_step_3_true() {
    acceptance::round_trip(3, true).unwrap();
}
