#[allow(dead_code)]
#[path = "support/prism_export.rs"]
mod acceptance;

#[test]
fn exact_prism_extrusions_export_reimport_and_continue() {
    acceptance::same_height();
}

#[test]
fn circular_holes_preserve_inward_orientation_and_exact_replay() {
    acceptance::holes();
}

#[test]
fn malformed_prism_exchange_stays_invalid_and_can_be_retried() {
    acceptance::malformed();
}

#[test]
fn preserved_planar_step_prisms_replay_with_exact_material() {
    acceptance::planar_controls();
}

#[test]
fn inward_analytic_prism_shell_is_rejected() {
    acceptance::inward_shell_is_rejected();
}

#[test]
fn covering_circular_cuts_export_replay_and_recut_exactly() {
    acceptance::covering_circular_controls();
}
