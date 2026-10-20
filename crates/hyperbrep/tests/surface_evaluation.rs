#[path = "support/surface_evaluation.rs"]
mod acceptance;

#[test]
fn joint_plane_images_compose_and_replay() {
    acceptance::joint_plane_images_compose_and_replay();
}

#[test]
fn selected_domains_and_archive_admission() {
    acceptance::selected_domains_and_archive_admission();
}

#[test]
fn selected_surface_policy_and_recovery() {
    acceptance::selected_surface_policy_and_recovery();
}
