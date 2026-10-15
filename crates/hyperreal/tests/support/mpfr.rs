//! MPFR cache teardown for Rug-based test oracles.

struct CacheCleanup;

impl Drop for CacheCleanup {
    fn drop(&mut self) {
        rug::float::free_cache(rug::float::FreeCache::Local);
    }
}

thread_local! {
    static CLEANUP: CacheCleanup = const { CacheCleanup };
}

/// Register cleanup before the current test thread first uses MPFR.
///
/// The thread keeps its caches throughout the test. Thread exit releases them
/// after local oracle values have dropped, including when a test unwinds.
/// Shared caches are left alone because other test threads may still use them.
pub(crate) fn cleanup_on_thread_exit() {
    CLEANUP.with(|_| {});
}
