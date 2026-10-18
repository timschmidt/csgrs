#[path = "support/selected_procedural_profiles.rs"]
mod acceptance;

#[test]
fn selected_profile_domains_retain_endpoints_and_projection() {
    acceptance::selected_profile_domains_retain_endpoints_and_projection();
}

#[test]
fn selected_profile_domains_reverse_transform_and_replay() {
    acceptance::selected_profile_domains_reverse_transform_and_replay();
}

#[test]
fn full_clipped_parallel_cap_constructs_and_replays() {
    acceptance::full_clipped_parallel_cap_constructs_and_replays();
}

#[test]
fn selected_chord_charts_keep_original_boundary() {
    acceptance::selected_chord_charts_keep_original_boundary();
}

#[test]
fn selected_origin_lines_retain_source_and_replay() {
    acceptance::selected_origin_lines_retain_source_and_replay();
}

#[test]
fn selected_profile_solids_split_both_edge_directions() {
    acceptance::selected_profile_solids_split_both_edge_directions();
}

#[test]
fn selected_profile_solids_reflect_and_replay() {
    acceptance::selected_profile_solids_reflect_and_replay();
}

#[test]
fn selected_profile_solids_continue_slab_booleans() {
    acceptance::selected_profile_solids_continue_slab_booleans();
}

#[test]
fn selected_profile_archives_reject_forged_proofs_and_recover() {
    acceptance::selected_profile_archives_reject_forged_proofs_and_recover();
}

#[test]
fn selected_profiles_keep_holes_components_and_reversal() {
    acceptance::selected_profiles_keep_holes_components_and_reversal();
}

#[test]
fn selected_profiles_preserve_certainty_and_budget_recovery() {
    acceptance::selected_profiles_preserve_certainty_and_budget_recovery();
}
