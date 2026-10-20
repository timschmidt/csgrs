#[path = "support/selected_planar_images.rs"]
mod acceptance;

#[test]
fn selected_planar_points_keep_frame_and_source_correlation() {
    acceptance::selected_planar_points_keep_frame_and_source_correlation();
}
#[test]
fn selected_planar_points_replay_sources_and_affine_transforms() {
    acceptance::selected_planar_points_replay_sources_and_affine_transforms();
}
#[test]
fn selected_parallel_edge_splits_replay_the_whole_cap() {
    acceptance::selected_parallel_edge_splits_replay_the_whole_cap();
}

#[test]
fn selected_parallel_splits_continue_through_boolean_sets() {
    acceptance::selected_parallel_splits_continue_through_boolean_sets();
}
#[test]
fn selected_parallel_splits_transform_and_preserve_near_separation() {
    acceptance::selected_parallel_splits_transform_and_preserve_near_separation();
}
#[test]
fn selected_planar_archives_reject_forged_sources_and_recover() {
    acceptance::selected_planar_archives_reject_forged_sources_and_recover();
}
#[test]
fn selected_planar_policies_and_work_exhaustion_recover() {
    acceptance::selected_planar_policies_and_work_exhaustion_recover();
}
#[test]
fn selected_planar_poles_remain_typed_failures() {
    acceptance::selected_planar_poles_remain_typed_failures();
}
