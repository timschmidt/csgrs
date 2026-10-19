#[path = "support/selected_revolution.rs"]
mod cases;

#[test]
fn selected_meridian_patch_retains_latitudes_and_replays() {
    cases::selected_meridian_patch_retains_latitudes_and_replays();
}

#[test]
fn selected_rotation_retains_points_and_angular_charts() {
    cases::selected_rotation_retains_points_and_angular_charts();
}

#[test]
fn selected_revolution_supports_reversal_reflection_and_subdivision() {
    cases::selected_revolution_supports_reversal_reflection_and_subdivision();
}

#[test]
fn selected_revolution_rejects_axis_contacts_and_invalid_domains() {
    cases::selected_revolution_rejects_axis_contacts_and_invalid_domains();
}

#[test]
fn selected_revolution_archives_reject_wrong_images_and_recover() {
    cases::selected_revolution_archives_reject_wrong_images_and_recover();
}

#[test]
fn selected_revolution_preserves_certainty_and_budget_recovery() {
    cases::selected_revolution_preserves_certainty_and_budget_recovery();
}

#[test]
fn retained_rotation_closed_edges_split_and_replay() {
    cases::retained_rotation_closed_edges_split_and_replay();
}

#[test]
fn selected_revolution_oblique_axes_tangencies_and_exact_gaps() {
    cases::selected_revolution_oblique_axes_tangencies_and_exact_gaps();
}
