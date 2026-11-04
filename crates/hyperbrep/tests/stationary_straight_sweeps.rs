#[path = "support/stationary_straight_sweeps.rs"]
mod acceptance;

#[test]
#[cfg(feature = "tessellation")]
fn stationary_straight_images_have_exact_planar_triangulations() {
    acceptance::stationary_straight_images_have_exact_planar_triangulations();
}
#[cfg(not(target_arch = "wasm32"))]
use acceptance::{q, shifted_cubic, square};
#[cfg(not(target_arch = "wasm32"))]
use hyperbrep::*;

#[test]
fn stationary_straight_images_construct_the_same_exact_prism() {
    acceptance::stationary_straight_images_construct_the_same_exact_prism();
}

#[test]
fn stationary_straight_images_continue_through_cuts_replay_and_reunion() {
    acceptance::stationary_straight_images_continue_through_cuts_replay_and_reunion();
}

#[test]
fn stationary_straight_images_reject_backtracking_and_lateral_stops() {
    acceptance::stationary_straight_images_reject_backtracking_and_lateral_stops();
}

#[test]
fn stationary_straight_images_preserve_elevation_weights_oblique_frames_and_reversal() {
    acceptance::stationary_straight_images_preserve_elevation_weights_oblique_frames_and_reversal();
}

#[test]
fn stationary_straight_images_keep_holes_and_tiny_material() {
    acceptance::stationary_straight_images_keep_holes_and_tiny_material();
}

#[test]
fn stationary_plane_correspondence_rejects_changed_whole_image_and_backtracking() {
    acceptance::stationary_plane_correspondence_rejects_changed_whole_image_and_backtracking();
}

#[test]
fn stationary_straight_images_continue_through_reflection_and_fixed_frame_entry() {
    acceptance::stationary_straight_images_continue_through_reflection_and_fixed_frame_entry();
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn stationary_straight_archive_replays_in_a_fresh_process() {
    if let Some(path) = std::env::var_os("CSGRS_R21_ARCHIVE_CHILD") {
        let json = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(model.to_json().unwrap(), json);
        let replayed = BodySet::from_solid(model.solids().next().unwrap());
        let canonical =
            builder::cuboid(Point3::origin(), Point3::new(1.into(), 1.into(), 1.into())).unwrap();
        assert!(
            boolean::symmetric_difference(&replayed, &canonical)
                .unwrap()
                .is_empty()
        );
        let slab = builder::cuboid(
            Point3::new((-1).into(), (-1).into(), q(1, 4)),
            Point3::new(2.into(), 2.into(), q(3, 4)),
        )
        .unwrap();
        let cut =
            boolean::difference(&replayed, &slab).expect("fresh replay accepts another cap cut");
        assert_eq!(cut.len(), 2);
        return;
    }
    let swept =
        builder::sweep_curve(&square(), Vector3::x(), Vector3::y(), shifted_cubic()).unwrap();
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-r21-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, swept.model().to_json().unwrap()).unwrap();
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "stationary_straight_archive_replays_in_a_fresh_process",
            "--nocapture",
        ])
        .env("CSGRS_R21_ARCHIVE_CHILD", &path)
        .status()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        status.success(),
        "fresh-process validation and continuation must complete"
    );
}
