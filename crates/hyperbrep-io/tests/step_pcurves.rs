#[allow(dead_code)]
#[path = "support/analytic_cylinder.rs"]
mod cylinder;
#[path = "support/supplied_pcurves.rs"]
mod fixture;

#[test]
fn supplied_circles_and_cylinder_lines_preserve_every_source_map() {
    fixture::source_maps();
}

#[test]
fn supplied_pcurves_preserve_partial_arcs_senses_units_and_loop_order() {
    fixture::representations();
}

#[test]
fn supplied_pcurve_models_edit_replay_reflect_and_compose_through_empty() {
    fixture::composition();
}

#[test]
fn every_alternate_and_seam_constraint_is_checked_before_publication() {
    fixture::failures();
}

#[test]
fn supplied_pcurves_preserve_source_accounting_limits_and_recovery() {
    fixture::admission();
}
