//! MPFR cache teardown for Rug-based test oracles.

/// Zero-sized thread-exit guard for the oracle backend's local caches, not its live values.
struct CacheCleanup;

/// Request local MPFR cache teardown when normal thread-local destruction runs.
/// This does not free shared caches or promise cleanup after process abort/forced exit.
impl Drop for CacheCleanup {
    /// Free only the current worker's backend cache; owned oracle values have their own destructors.
    fn drop(&mut self) {
        rug::float::free_cache(rug::float::FreeCache::Local);
    }
}

thread_local! {
    /// One lazy guard per thread, initialized by the first registration and reused thereafter.
    static CLEANUP: CacheCleanup = const { CacheCleanup };
}

/// Register cleanup before the current test thread first uses MPFR.
///
/// The thread keeps its caches throughout the test. Thread exit releases them
/// after local oracle values have dropped, including when a test unwinds.
/// Shared caches are left alone because other test threads may still use them.
/// Repeated registration neither clears caches immediately nor adds guards.
/// Cleanup depends on thread-local destructors running; process abort/forced
/// termination and other TLS destructor ordering are not covered by this helper.
pub(crate) fn cleanup_on_thread_exit() {
    CLEANUP.with(|_| {});
}
