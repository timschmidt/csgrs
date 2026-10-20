#[path = "support/measurement_admission.rs"]
mod acceptance;

#[test]
fn planar_spline_incidence_does_not_require_low_degree_root_solving() {
    acceptance::planar_spline_incidence_does_not_require_low_degree_root_solving();
}

#[test]
fn rational_prism_validity_does_not_require_a_closed_form_measurement() {
    acceptance::rational_prism_validity_does_not_require_a_closed_form_measurement();
}

#[test]
fn holes_components_and_affine_images_keep_optional_measurements() {
    acceptance::holes_components_and_affine_images_keep_optional_measurements();
}

#[test]
fn measurement_absence_survives_empty_identities_and_repeated_slab_cuts() {
    acceptance::measurement_absence_survives_empty_identities_and_repeated_slab_cuts();
}

#[test]
fn inward_and_damaged_archives_remain_invalid_without_volume() {
    acceptance::inward_and_damaged_archives_remain_invalid_without_volume();
}
