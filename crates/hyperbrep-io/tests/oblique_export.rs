#[path = "support/oblique_export.rs"]
mod support;

#[test]
fn oblique_results_keep_surface_blockers_and_context_recovery() {
    support::failures_and_recovery();
}
