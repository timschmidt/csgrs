#[allow(dead_code)]
#[path = "support/sectors.rs"]
mod acceptance;
#[test]
fn torus_axial_quarter() {
    acceptance::original(0);
}
#[test]
fn torus_longitudinal_quarter() {
    acceptance::original(1);
}
#[test]
fn cone_longitudinal_quarter() {
    acceptance::original(2);
}
#[test]
fn torus_longitudinal_band() {
    acceptance::original(3);
}
#[test]
fn cone_longitudinal_band() {
    acceptance::original(4);
}
#[test]
fn sector_composition_under_exact_similarities() {
    assert_eq!(acceptance::transforms().0, 30);
}

#[test]
fn public_annular_partition_preserves_source_evidence() {
    acceptance::annular_partition();
}
