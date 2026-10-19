#[path = "support/bivariate_topology.rs"]
mod support;

#[test]
fn source_bound_slices_limits_and_recovery() {
    support::source_bound_slices_limits_and_recovery();
}

#[test]
fn complete_interior_loop_and_empty_fibers() {
    support::complete_interior_loop_and_empty_fibers();
}

#[test]
fn algebraic_fiber_events_and_quartic_projection_loops() {
    support::algebraic_fiber_events_and_quartic_projection_loops();
}

#[test]
fn irreducible_nodal_quartic_keeps_both_loops() {
    support::irreducible_nodal_quartic_keeps_both_loops();
}

#[test]
fn covertical_critical_points_and_disconnected_loops() {
    support::covertical_critical_points_and_disconnected_loops();
}

#[test]
fn singular_nodes_cusps_and_isolated_contacts_keep_incidence() {
    support::singular_nodes_cusps_and_isolated_contacts_keep_incidence();
}

#[test]
fn trim_boundary_crossings_tangencies_and_corners() {
    support::trim_boundary_crossings_tangencies_and_corners();
}

#[test]
fn degree_drop_and_algebraic_projection_events() {
    support::degree_drop_and_algebraic_projection_events();
}

#[test]
fn original_brep_patch_pullback_and_chart_changes() {
    support::original_brep_patch_pullback_and_chart_changes();
}

#[test]
fn explicit_dimension_degeneracy_budget_and_recovery() {
    support::explicit_dimension_degeneracy_budget_and_recovery();
}
