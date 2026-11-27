//! Heap accounting for resource atlases.
//!
//! An atlas binary installs [`CountingAllocator`] as its global allocator:
//!
//! ```ignore
//! #[global_allocator]
//! static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;
//! ```
//!
//! It delegates to the system allocator. Counting is dormant (one relaxed
//! load per call) until a resource atlas activates it, so timing atlases are
//! not slowed by the counters. Only atlas examples install it, so production
//! builds of the measured crates are unchanged. Counts cover every thread.
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering::Relaxed};

/// System allocator with allocation, byte, live and peak counters.
pub struct CountingAllocator;

static ACTIVE: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);
static REALLOCATIONS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED: AtomicU64 = AtomicU64::new(0);
// Signed: blocks allocated before activation can be freed after it, so only
// differences of these readings are meaningful.
static LIVE: AtomicI64 = AtomicI64::new(0);
static PEAK: AtomicI64 = AtomicI64::new(0);

fn grow(bytes: u64) {
    let bytes = bytes as i64;
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}
fn shrink(bytes: u64) {
    LIVE.fetch_sub(bytes as i64, Relaxed);
}

// SAFETY: every call delegates to `System` with the caller's arguments, so
// the layout and pointer contract is System's. The counters touch no memory
// handed out by the allocator.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && ACTIVE.load(Relaxed) {
            ALLOCATIONS.fetch_add(1, Relaxed);
            ALLOCATED.fetch_add(layout.size() as u64, Relaxed);
            grow(layout.size() as u64);
        }
        pointer
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() && ACTIVE.load(Relaxed) {
            ALLOCATIONS.fetch_add(1, Relaxed);
            ALLOCATED.fetch_add(layout.size() as u64, Relaxed);
            grow(layout.size() as u64);
        }
        pointer
    }
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let moved = unsafe { System.realloc(pointer, layout, size) };
        if !moved.is_null() && ACTIVE.load(Relaxed) {
            REALLOCATIONS.fetch_add(1, Relaxed);
            let (old, new) = (layout.size() as u64, size as u64);
            if new > old {
                ALLOCATED.fetch_add(new - old, Relaxed);
                grow(new - old);
            } else {
                shrink(old - new);
            }
        }
        moved
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
        if ACTIVE.load(Relaxed) {
            shrink(layout.size() as u64);
        }
    }
}

/// Start counting, and report whether [`CountingAllocator`] is installed.
///
/// Blocks freed after activation but allocated before it are subtracted from
/// the live total, so live figures are only meaningful as differences, which
/// is how every measurement uses them.
pub fn activate() -> bool {
    ACTIVE.store(true, Relaxed);
    let before = ALLOCATIONS.load(Relaxed);
    drop(std::hint::black_box(Box::new(0u64)));
    ALLOCATIONS.load(Relaxed) > before
}

/// Counter readings at the start of a measured call.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Mark {
    allocations: u64,
    reallocations: u64,
    allocated: u64,
    live: i64,
}

/// Heap use of one call, relative to its start.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heap {
    /// New allocations (excluding reallocations).
    pub allocations: u64,
    /// Reallocations that succeeded.
    pub reallocations: u64,
    /// Bytes requested by allocations and by reallocation growth.
    pub allocated: u64,
    /// Highest live bytes above the starting live total.
    pub peak: u64,
    /// Live bytes above the start while the result is still held.
    pub retained: u64,
}

/// Start a measured window: reset the peak to the current live total.
pub(crate) fn mark() -> Mark {
    let live = LIVE.load(Relaxed);
    PEAK.store(live, Relaxed);
    Mark {
        allocations: ALLOCATIONS.load(Relaxed),
        reallocations: REALLOCATIONS.load(Relaxed),
        allocated: ALLOCATED.load(Relaxed),
        live,
    }
}

/// Heap use since `start`, with the result still alive.
pub(crate) fn since(start: Mark) -> Heap {
    Heap {
        allocations: ALLOCATIONS.load(Relaxed) - start.allocations,
        reallocations: REALLOCATIONS.load(Relaxed) - start.reallocations,
        allocated: ALLOCATED.load(Relaxed) - start.allocated,
        peak: delta(PEAK.load(Relaxed), start.live),
        retained: delta(LIVE.load(Relaxed), start.live),
    }
}

/// Live bytes above `start` (after the result and input are dropped: growth
/// of global caches or leaks).
pub(crate) fn residue(start: Mark) -> u64 {
    delta(LIVE.load(Relaxed), start.live)
}

// Nonnegative difference of two live readings.
fn delta(now: i64, start: i64) -> u64 {
    u64::try_from(now - start).unwrap_or(0)
}
