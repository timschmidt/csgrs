#[allow(dead_code)]
#[path = "support/torus_input.rs"]
mod input;
#[allow(dead_code)]
#[path = "support/torus_exchange.rs"]
mod torus_exchange;

#[test]
fn native_ring_torus_survives_exact_step_exchange() {
    torus_exchange::whole();
}

#[test]
fn imported_torus_halves_recut_clear_and_reconstruct() {
    torus_exchange::halves_and_reconstruction();
}

#[test]
fn rational_torus_band_keeps_exact_measures_and_near_samples() {
    torus_exchange::rational_band();
}

#[test]
fn both_torus_angles_keep_supplied_source_maps_and_units() {
    input::source_maps();
}

#[test]
fn malformed_torus_images_and_decimal_seams_fail_without_repair() {
    input::failures_and_recovery();
}

#[test]
fn torus_import_exhaustion_recovers_without_partial_success() {
    input::admission();
}
