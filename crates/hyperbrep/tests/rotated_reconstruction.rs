#[allow(dead_code)]
#[path = "support/rotated_reconstruction.rs"]
mod acceptance;
#[allow(dead_code)]
#[path = "support/seam_cuts.rs"]
mod seam;

#[test]
fn rationally_rotated_torus_reconstruction_continues() {
    acceptance::workflow(false);
}

#[test]
fn swapped_rationally_rotated_torus_reconstruction_continues() {
    acceptance::workflow(true);
}
