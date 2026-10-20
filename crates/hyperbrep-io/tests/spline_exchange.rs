#[path = "support/spline_exchange.rs"]
mod acceptance;

#[test]
fn retained_spline_profiles_complete_step_replay_and_repeated_cuts() {
    acceptance::retained_spline_profiles_complete_step_replay_and_repeated_cuts();
}

#[test]
fn independent_splines_retain_charts_weights_trims_units_and_modeling() {
    acceptance::independent_splines_retain_charts_weights_trims_units_and_modeling();
}

#[test]
fn invalid_spline_records_and_exhausted_limits_never_publish_a_body() {
    acceptance::invalid_spline_records_and_exhausted_limits_never_publish_a_body();
}

#[test]
fn spline_whole_images_ambiguous_trims_and_supplied_pcurves_are_checked() {
    acceptance::spline_whole_images_ambiguous_trims_and_supplied_pcurves_are_checked();
}

#[test]
fn spline_write_limits_units_and_exact_numeric_failures_are_explicit() {
    acceptance::spline_write_limits_units_and_exact_numeric_failures_are_explicit();
}
