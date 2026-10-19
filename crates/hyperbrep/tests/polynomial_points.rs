#[path = "support/polynomial_points.rs"]
mod support;

#[test]
fn polynomial_point_archive_admission_and_replay() {
    support::polynomial_point_archive_admission_and_replay();
}
