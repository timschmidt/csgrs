#[path = "support/pcurve_parameters.rs"]
mod acceptance;

#[test]
fn selected_pcurve_maps_compose_and_replay() {
    acceptance::selected_pcurve_maps_compose_and_replay();
}

#[test]
fn pcurve_maps_check_domains_and_recover_after_exhaustion() {
    acceptance::pcurve_maps_check_domains_and_recover_after_exhaustion();
}
