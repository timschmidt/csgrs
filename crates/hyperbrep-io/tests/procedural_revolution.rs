#[path = "support/procedural_revolution.rs"]
mod cases;
#[allow(dead_code)]
#[path = "../../hyperbrep/tests/support/selected_procedural_profiles.rs"]
mod source;
#[test]
fn procedural_revolution_step_rejection_preserves_native_solid() {
    cases::procedural_revolution_step_rejection_preserves_native_solid(
        &source::clipped_parallel_cap(),
    );
}
