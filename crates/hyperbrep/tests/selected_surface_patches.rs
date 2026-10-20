#[path = "support/selected_surface_patches.rs"]
mod acceptance;

#[test]
fn selected_extrusion_patch_builds_and_replays() {
    acceptance::selected_extrusion_patch_builds_and_replays();
}

#[test]
fn selected_planar_face_builds_and_replays() {
    acceptance::selected_planar_face_builds_and_replays();
}

#[test]
fn selected_patches_reverse_transform_and_split() {
    acceptance::selected_patches_reverse_transform_and_split();
}

#[test]
fn selected_planar_faces_keep_holes_frames_and_orientation() {
    acceptance::selected_planar_faces_keep_holes_frames_and_orientation();
}

#[test]
fn selected_planar_patches_keep_holes_and_contact_edits() {
    acceptance::selected_planar_patches_keep_holes_and_contact_edits();
}

#[test]
fn selected_patch_archives_reject_wrong_images_and_recover() {
    acceptance::selected_patch_archives_reject_wrong_images_and_recover();
}

#[test]
fn selected_patches_keep_failure_certainty_and_budget_contracts() {
    acceptance::selected_patches_keep_failure_certainty_and_budget_contracts();
}
