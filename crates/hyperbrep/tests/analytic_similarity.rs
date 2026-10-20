#[allow(dead_code)]
#[path = "support/similarity.rs"]
mod similarity;

#[test]
fn analytic_primitives_scale_replay_and_invert_exactly() {
    similarity::primitives();
}

#[test]
fn clipped_and_void_certificates_follow_the_same_metric() {
    similarity::clipped_and_voids();
}

#[test]
fn selected_axial_sources_survive_repeated_scaling_editing_and_replay() {
    similarity::selected_and_repeated();
}

#[test]
fn unsupported_and_invalid_maps_remain_distinct_and_retryable() {
    similarity::rejection_and_recovery();
}

#[test]
fn periodic_seams_and_tiny_distinct_offsets_remain_exact() {
    similarity::periodic_and_near();
}

#[test]
fn extrusion_and_revolution_profiles_preserve_native_families_and_images() {
    similarity::profile_families();
}
