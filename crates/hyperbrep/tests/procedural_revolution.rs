#[path = "support/procedural_revolution.rs"]
mod cases;

#[test]
fn full_parallel_cap_revolves_and_replays() {
    cases::full_parallel_cap_revolves_and_replays();
}

#[test]
fn selected_parallel_cap_revolves_and_replays() {
    cases::selected_parallel_cap_revolves_and_replays();
}

#[test]
fn procedural_revolution_reflection_reversal_and_exact_gaps() {
    cases::procedural_revolution_reflection_reversal_and_exact_gaps();
}

#[test]
fn procedural_revolution_subdivision_keeps_shared_incidence() {
    cases::procedural_revolution_subdivision_keeps_shared_incidence();
}

#[test]
fn procedural_revolution_retains_native_and_selected_cavities() {
    cases::procedural_revolution_retains_native_and_selected_cavities();
}

#[test]
fn procedural_revolution_rejects_invalid_profiles_and_cavities() {
    cases::procedural_revolution_rejects_invalid_profiles_and_cavities();
}

#[test]
fn procedural_revolution_certainty_budget_and_archive_recovery() {
    cases::procedural_revolution_certainty_budget_and_archive_recovery();
}

#[test]
fn procedural_revolution_consumes_regions_holes_components_and_empty() {
    cases::procedural_revolution_consumes_regions_holes_components_and_empty();
}

#[test]
fn unified_revolution_preserves_native_images_and_exact_volume() {
    cases::unified_revolution_preserves_native_images_and_exact_volume();
}
