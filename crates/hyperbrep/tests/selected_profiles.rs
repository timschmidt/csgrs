#[path = "support/selected_profiles.rs"]
mod acceptance;

#[test]
fn selected_boolean_profiles_extrude_and_replay() {
    acceptance::selected_boolean_profiles_extrude_and_replay();
}

#[test]
fn selected_profile_paths_share_native_image_admission() {
    acceptance::selected_profile_paths_share_native_image_admission();
}

#[test]
fn selected_profile_edits_and_slab_booleans_preserve_exact_material() {
    acceptance::selected_profile_edits_and_slab_booleans_preserve_exact_material();
}

#[test]
fn selected_profiles_keep_holes_and_disconnected_components() {
    acceptance::selected_profiles_keep_holes_and_disconnected_components();
}

#[test]
fn selected_profiles_transport_exactly_and_reject_damaged_archives() {
    acceptance::selected_profiles_transport_exactly_and_reject_damaged_archives();
}

#[test]
fn partial_height_profile_contacts_keep_boundary_and_overlap_distinct() {
    acceptance::partial_height_profile_contacts_keep_boundary_and_overlap_distinct();
}
