#[path = "support/axial_cuts.rs"]
mod acceptance;

#[test]
fn touching_axial_tool_result_completes_step_and_continued_modeling() {
    acceptance::round_trip(false, false).unwrap();
}
#[test]
fn disjoint_axial_tool_result_completes_step_and_continued_modeling() {
    acceptance::round_trip(true, false).unwrap();
}
#[test]
fn reflected_touching_result_completes_step_and_continued_modeling() {
    acceptance::round_trip(false, true).unwrap();
}
#[test]
fn reflected_disjoint_result_completes_step_and_continued_modeling() {
    acceptance::round_trip(true, true).unwrap();
}
