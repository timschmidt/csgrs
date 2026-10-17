#[allow(dead_code)]
#[path = "support/cone_exchange.rs"]
mod acceptance;

#[test]
fn existing_faceted_plane_cylinder_and_torus_sources_retain_exact_maps() {
    acceptance::inputs::existing_surface_maps();
}

#[test]
fn full_size_cone_exchange_preserves_repeated_modeling_and_empty_recovery() {
    acceptance::complete_workflows();
}
#[test]
fn nonterminating_apex_uses_finite_anchor_and_exact_source_parameter_map() {
    acceptance::shifted_apex_and_units();
}
#[test]
fn independent_manual_revolutions_import_and_reexport_without_fitting() {
    acceptance::manual_line_inputs();
}
#[test]
fn source_branches_axis_reversals_and_declared_units_keep_exact_maps() {
    acceptance::inputs::reversed_generators_axes_units_and_source_maps();
}
#[test]
fn supplied_cone_pcurves_use_generator_parameters_and_keep_source_maps() {
    acceptance::inputs::supplied_pcurve_parameters();
}
#[test]
fn invalid_unsupported_and_resource_failures_remain_typed_and_retryable() {
    acceptance::inputs::failures_and_limits();
}
#[test]
fn similarities_reflections_and_unencodable_splits_preserve_the_native_model() {
    acceptance::transformed_frames_and_explicit_encoding_limits();
}
