#[path = "support/body_sets.rs"]
mod cases;

#[test]
fn empty_identities_continue() {
    cases::empty_identities_continue();
}
#[test]
fn disconnected_accumulation_removal_and_bridge() {
    cases::disconnected_accumulation_removal_and_bridge();
}
#[test]
fn overlapping_selections_follow_union_semantics() {
    cases::overlapping_selections_follow_union_semantics();
}
#[test]
fn unselected_bodies_never_reappear() {
    cases::unselected_bodies_never_reappear();
}
#[test]
fn holes_symbolic_coordinates_and_cavities() {
    cases::holes_symbolic_coordinates_and_cavities();
}
#[test]
fn selected_parameters_survive_collection() {
    cases::selected_parameters_survive_collection();
}
#[test]
fn failures_are_atomic_and_empty_does_not_admit_raw_models() {
    cases::failures_are_atomic_and_empty_does_not_admit_raw_models();
}
#[test]
fn edge_and_point_contacts_keep_both_solids() {
    cases::edge_and_point_contacts_keep_both_solids();
}
#[test]
fn exact_gap_and_ambient_policy_survive_composition() {
    cases::exact_gap_and_ambient_policy_survive_composition();
}

#[test]
fn selection_identity_covers_curved_sweeps() {
    cases::selection_identity_covers_curved_sweeps();
}
