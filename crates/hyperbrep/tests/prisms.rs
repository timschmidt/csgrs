#[allow(dead_code)]
#[path = "support/prisms.rs"]
mod acceptance;

#[test]
fn circular_region_extrusion_preserves_measurement_and_continuation() {
    acceptance::same_height_cylinder();
}

#[test]
fn covering_slabs_preserve_exact_circular_composition() {
    acceptance::covering_cylinders();
}

#[test]
fn covering_bores_preserve_holes_and_repeated_composition() {
    acceptance::covering_bores();
}

#[test]
fn covering_cut_preserves_annular_material() {
    acceptance::annular_cut();
}

#[test]
fn partial_and_nearly_covering_slabs_keep_their_exact_domain() {
    acceptance::slab_boundaries();
}
