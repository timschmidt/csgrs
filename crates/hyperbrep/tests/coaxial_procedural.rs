#[path = "support/coaxial_procedural.rs"]
mod support;

#[test]
fn coaxial_parallel_union_and_xor_overlap() {
    support::coaxial_parallel_union_and_xor_overlap();
}

#[test]
fn full_parallel_revolution_boolean_continues() {
    support::full_parallel_revolution_boolean_continues();
}

#[test]
fn selected_parallel_revolution_boolean_continues() {
    support::selected_parallel_revolution_boolean_continues();
}

#[test]
fn coaxial_native_arc_and_nurbs_profiles_continue() {
    support::coaxial_native_arc_and_nurbs_profiles_continue();
}
#[test]
fn coaxial_selected_axes_similarities_and_exact_gaps() {
    support::coaxial_selected_axes_similarities_and_exact_gaps();
}
#[test]
fn coaxial_procedural_cavities_xor_and_empty_recover() {
    support::coaxial_procedural_cavities_xor_and_empty_recover();
}
#[test]
fn coaxial_selected_split_output_continues() {
    support::coaxial_selected_split_output_continues();
}
#[test]
fn coaxial_procedural_certainty_budget_and_axis_rejection() {
    support::coaxial_procedural_certainty_budget_and_axis_rejection();
}
