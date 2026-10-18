#[allow(dead_code)]
#[path = "../../hyperbrep/tests/support/selected_procedural_profiles.rs"]
mod profile;

#[path = "support/selected_procedural_profiles.rs"]
mod acceptance;

#[test]
fn selected_procedural_profiles_fail_step_with_attribution_and_native_recovery() {
    acceptance::selected_procedural_profiles_fail_step_with_attribution_and_native_recovery(
        &profile::clipped_parallel_cap(),
    );
}
