#[allow(dead_code)]
#[path = "support/seam_cuts.rs"]
mod acceptance;
#[test]
fn torus_axial_half_on_existing_seams() {
    acceptance::original(0);
}
#[test]
fn torus_longitudinal_half_on_existing_seams() {
    acceptance::original(1);
}
#[test]
fn cone_longitudinal_half_on_existing_seams() {
    acceptance::original(2);
}
#[test]
fn seams_are_invariant_under_exact_similarities() {
    assert_eq!(acceptance::transforms(), 54);
}

#[test]
fn existing_seam_proofs_borrow_their_actual_source_revisions() {
    assert_eq!(acceptance::boundary_provenance(), 16);
}

#[test]
fn complete_periodic_boundary_traces_keep_a_single_seam_vertex() {
    assert_eq!(acceptance::closed_boundary_provenance(), 2);
}
