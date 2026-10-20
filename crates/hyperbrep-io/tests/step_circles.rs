#[path = "support/analytic_cylinder.rs"]
mod fixture;

#[test]
fn exact_closed_circular_edges_import_as_an_analytic_cylinder() {
    fixture::closed_circle();
}

#[test]
fn circular_source_parameters_and_shared_seam_uses_remain_distinct() {
    fixture::parameters();
}

#[test]
fn partial_arcs_frames_senses_and_loop_order_keep_the_same_exact_solid() {
    fixture::representations();
}

#[test]
fn oblique_and_radical_frames_preserve_exact_geometry_and_native_replay() {
    fixture::oblique_frames();
}

#[test]
fn angular_units_are_independent_of_length_and_names_do_not_infer_pi() {
    fixture::units();
}

#[test]
fn analytic_import_edits_replays_and_composes_through_empty_results() {
    fixture::composition();
}

#[test]
fn invalid_and_unsupported_analytic_sources_never_become_successful_models() {
    fixture::failures();
}

#[test]
fn analytic_import_ownership_accounting_limits_and_recovery_are_explicit() {
    fixture::admission();
}
