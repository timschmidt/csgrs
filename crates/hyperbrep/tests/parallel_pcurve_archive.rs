#[path = "support/parallel_pcurve_archive.rs"]
mod acceptance;

#[test]
fn nonlinear_parallel_pcurve_replays_exactly() {
    acceptance::nonlinear_parallel_pcurve_replays_exactly();
}

#[test]
fn polynomial_and_rational_parallel_sources_replay() {
    acceptance::polynomial_and_rational_parallel_sources_replay();
}

#[test]
fn selected_parallel_ranges_survive_replay_and_subdivision() {
    acceptance::selected_parallel_ranges_survive_replay_and_subdivision();
}

#[test]
fn parallel_cusp_endpoints_remain_selected_on_replay() {
    acceptance::parallel_cusp_endpoints_remain_selected_on_replay();
}

#[test]
fn replayed_parallel_pcurves_reenter_planar_modeling() {
    acceptance::replayed_parallel_pcurves_reenter_planar_modeling();
}

#[test]
fn parallel_archives_reject_forged_definitions() {
    acceptance::parallel_archives_reject_forged_definitions();
}

#[test]
fn parallel_archive_limits_and_exhaustion_recover() {
    acceptance::parallel_archive_limits_and_exhaustion_recover();
}
