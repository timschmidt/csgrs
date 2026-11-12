//! Test/probe-only scoped allocation counter. Runs in a single-threaded scope.
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

struct Allocator;
#[global_allocator]
static ALLOCATOR: Allocator = Allocator;
static ENABLED: AtomicBool = AtomicBool::new(false);
static COUNT: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);

fn record(bytes: usize) {
    if ENABLED.load(Ordering::Relaxed) {
        COUNT.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(bytes, Ordering::Relaxed);
    }
}

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record(size);
        unsafe { System.realloc(pointer, layout, size) }
    }
}

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
