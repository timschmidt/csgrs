#[allow(dead_code)]
#[path = "support/axial_cuts.rs"]
mod acceptance;

#[test]
fn partial_depth_cuts_retain_traces_partition_and_select_before_shell_admission() {
    acceptance::partial_stages();
}
#[test]
fn axial_traces_respect_shifted_periodic_charts_reversal_tangency_and_tiny_gaps() {
    acceptance::periodic_traces();
}
#[test]
fn touching_tool_difference_composes_and_replays_exactly() {
    acceptance::unchanged_cut(false);
}
#[test]
fn disjoint_tool_difference_composes_and_replays_exactly() {
    acceptance::unchanged_cut(true);
}
