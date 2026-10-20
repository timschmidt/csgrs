#[allow(dead_code)]
#[path = "support/cone_continuation.rs"]
mod acceptance;
#[allow(dead_code)]
#[path = "support/seam_cuts.rs"]
mod seam;

#[test]
fn rationally_rotated_cone_band_continues() {
    acceptance::band();
}

#[test]
fn rationally_rotated_cone_reconstruction_continues() {
    acceptance::reconstruction(false);
}

#[test]
fn swapped_rationally_rotated_cone_reconstruction_continues() {
    acceptance::reconstruction(true);
}

#[test]
fn original_rejected_cone_partition_replays_cold() {
    acceptance::original_partition();
}

#[test]
fn captured_cone_circle_point_query_replays_cold() {
    acceptance::captured_circle_query();
}

#[test]
fn cone_overlap_contacts_keep_boundary_endpoint_images() {
    acceptance::overlap_contacts();
}
