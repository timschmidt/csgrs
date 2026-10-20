#[path = "support/parallel_profile_extrusion.rs"]
mod acceptance;

#[test]
fn original_nonlinear_parallel_cap_extrudes_and_replays() {
    acceptance::original_nonlinear_parallel_cap_extrudes_and_replays();
}

#[test]
fn planar_images_preserve_exact_frame_and_chart() {
    acceptance::planar_images_preserve_exact_frame_and_chart();
}

#[test]
fn parallel_cap_classification_and_edge_split() {
    acceptance::parallel_cap_classification_and_edge_split();
}

#[test]
fn parallel_cap_affine_transform_and_cold_replay() {
    acceptance::parallel_cap_affine_transform_and_cold_replay();
}

#[test]
fn parallel_cap_repeated_slab_booleans() {
    acceptance::parallel_cap_repeated_slab_booleans();
}

#[test]
fn parallel_extrusion_point_queries() {
    acceptance::parallel_extrusion_point_queries();
}

#[test]
fn authored_parallel_paths_share_admission() {
    acceptance::authored_parallel_paths_share_admission();
}

#[test]
fn planar_image_archives_reject_forged_interior_and_frames() {
    acceptance::planar_image_archives_reject_forged_interior_and_frames();
}

#[test]
fn planar_image_limits_and_typed_frontiers_recover() {
    acceptance::planar_image_limits_and_typed_frontiers_recover();
}

#[test]
fn parallel_caps_keep_holes_and_components() {
    acceptance::parallel_caps_keep_holes_and_components();
}

#[test]
fn planar_lifts_share_native_source_evaluation() {
    acceptance::planar_lifts_share_native_source_evaluation();
}
