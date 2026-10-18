#[path = "support/spline_geometry.rs"]
mod acceptance;

#[test]
fn spline_views_retain_weights_knots_and_reversed_subcurve_charts() {
    acceptance::spline_views_retain_weights_knots_and_reversed_subcurve_charts();
}

#[test]
fn planar_pcurves_prove_whole_image_and_retain_parameter_correspondence() {
    acceptance::planar_pcurves_prove_whole_image_and_retain_parameter_correspondence();
}

#[test]
fn extrusion_location_preserves_oblique_nonunit_direction_and_source_parameters() {
    acceptance::extrusion_location_preserves_oblique_nonunit_direction_and_source_parameters();
}

#[test]
fn extrusion_location_keeps_ambiguity_and_rejects_nontransverse_or_nonplanar_charts() {
    acceptance::extrusion_location_keeps_ambiguity_and_rejects_nontransverse_or_nonplanar_charts();
}
