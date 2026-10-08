use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

struct Counted;
static CALLS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
static LIVE: AtomicU64 = AtomicU64::new(0);
static PEAK: AtomicU64 = AtomicU64::new(0);

fn add(n: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(n as u64, Relaxed);
    let live = LIVE.fetch_add(n as u64, Relaxed) + n as u64;
    PEAK.fetch_max(live, Relaxed);
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() { add(layout.size()); }
        p
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc_zeroed(layout) };
        if !p.is_null() { add(layout.size()); }
        p
    }
    unsafe fn realloc(&self, p: *mut u8, old: Layout, n: usize) -> *mut u8 {
        let next = unsafe { System.realloc(p, old, n) };
        if !next.is_null() {
            LIVE.fetch_sub(old.size() as u64, Relaxed);
            add(n);
        }
        next
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size() as u64, Relaxed);
        unsafe { System.dealloc(p, layout) };
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

pub fn reset() -> u64 {
    let baseline = LIVE.load(Relaxed);
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    PEAK.store(baseline, Relaxed);
    baseline
}

pub fn measure(baseline: u64) -> String {
    let calls = CALLS.load(Relaxed);
    let bytes = BYTES.load(Relaxed);
    let peak = PEAK.load(Relaxed) - baseline;
    let retained = LIVE.load(Relaxed) as i128 - baseline as i128;
    format!("{{\"calls\":{calls},\"bytes\":{bytes},\"peak_extra_requested\":{peak},\"retained_delta\":{retained}}}")
}
