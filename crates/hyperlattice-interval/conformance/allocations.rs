//! Test/probe-only counter of allocation requests through the installed allocator.
//!
//! Callers must arrange a single nonnested measurement and avoid unrelated
//! concurrent allocation. The enable flag is process-global, not thread-local;
//! relaxed atomic counters do not isolate scopes or synchronize callers. Counts
//! include attempted alloc/realloc calls and requested bytes, not live/peak memory,
//! successful allocations alone or allocations bypassing this wrapper.
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// Forward allocations to System while optionally recording requests beforehand.
struct Allocator;
#[global_allocator]
/// Allocator installed for each test/probe executable that includes this module.
static ALLOCATOR: Allocator = Allocator;
/// Process-global switch; a panicking measured closure leaves it enabled.
static ENABLED: AtomicBool = AtomicBool::new(false);
/// Number of alloc/realloc requests since the last reset, wrapping at usize capacity.
static COUNT: AtomicUsize = AtomicUsize::new(0);
/// Sum of requested allocation/reallocation sizes, without subtraction on deallocation.
static BYTES: AtomicUsize = AtomicUsize::new(0);

/// Record one request when enabled; relaxed atomic additions wrap on overflow.
fn record(bytes: usize) {
    if ENABLED.load(Ordering::Relaxed) {
        COUNT.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(bytes, Ordering::Relaxed);
    }
}

/// Preserve System's allocation contract while observing requests without allocating.
/// The unsafe operations forward the caller's pointer/layout requirements unchanged.
unsafe impl GlobalAlloc for Allocator {
    /// Count the requested layout size before delegating, including allocation failures.
    ///
    /// # Safety
    /// The caller must satisfy GlobalAlloc's allocation layout requirements.
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc(layout) }
    }
    /// Forward deallocation without decrementing cumulative request counters.
    ///
    /// # Safety
    /// The pointer and layout must satisfy System's live-allocation deallocation contract.
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }
    /// Count the full requested new size, then forward reallocation, even on failure.
    ///
    /// # Safety
    /// The pointer, old layout and new size must satisfy GlobalAlloc's reallocation rules.
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record(size);
        unsafe { System.realloc(pointer, layout, size) }
    }
}

/// Return the closure result, allocation-request count and cumulative requested bytes.
///
/// Reset counters, enable recording, run the closure once, then disable and read.
/// Destruction inside the closure occurs during the scope; destruction of the
/// returned value occurs later and is not measured. Nesting/concurrent calls are
/// unsupported. There is no unwind guard: panic propagates with recording still
/// enabled. This is an observation helper, not a memory budget or peak-memory meter.
pub fn measure<T>(operation: impl FnOnce() -> T) -> (T, usize, usize) {
    COUNT.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    ENABLED.store(true, Ordering::Relaxed);
    let output = operation();
    ENABLED.store(false, Ordering::Relaxed);
    (
        output,
        COUNT.load(Ordering::Relaxed),
        BYTES.load(Ordering::Relaxed),
    )
}
