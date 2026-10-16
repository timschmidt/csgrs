#[allow(dead_code)]
#[path = "support/coplanar.rs"]
mod acceptance;

#[test]
fn curved_coplanar_composition_under_exact_similarities() {
    assert_eq!(acceptance::transforms().0, 54);
}

#[test]
fn circular_boundary_proofs_retain_native_charts_and_source_revisions() {
    assert_eq!(acceptance::circular_boundary_provenance(), 8);
}

#[test]
fn torus_duplicate_carrier_replay() {
    acceptance::duplicate_carriers(0);
}

#[test]
fn cone_duplicate_carrier_replay() {
    acceptance::duplicate_carriers(2);
}

#[test]
fn torus_axial_recut() {
    acceptance::original(0, 0);
}
#[test]
fn torus_axial_clear_and_recover() {
    acceptance::original(0, 1);
}
#[test]
fn torus_axial_reconstruct() {
    acceptance::original(0, 2);
}
#[test]
fn torus_longitudinal_recut() {
    acceptance::original(1, 0);
}
#[test]
fn torus_longitudinal_clear_and_recover() {
    acceptance::original(1, 1);
}
#[test]
fn torus_longitudinal_reconstruct() {
    acceptance::original(1, 2);
}
#[test]
fn cone_longitudinal_recut() {
    acceptance::original(2, 0);
}
#[test]
fn cone_longitudinal_clear_and_recover() {
    acceptance::original(2, 1);
}
#[test]
fn cone_longitudinal_reconstruct() {
    acceptance::original(2, 2);
}
