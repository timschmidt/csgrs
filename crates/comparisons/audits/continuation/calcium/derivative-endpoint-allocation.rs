mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering};
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static BYTES: AtomicUsize = AtomicUsize::new(0);
    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);
    struct Count;
    #[global_allocator]
    static GLOBAL: Count = Count;
    fn allocated(size: usize) {
        CALLS.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(size, Ordering::Relaxed);
        let live = LIVE.fetch_add(size, Ordering::Relaxed) + size;
        PEAK.fetch_max(live, Ordering::Relaxed);
    }
    unsafe impl GlobalAlloc for Count {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc(layout) };
            if !p.is_null() {
                allocated(layout.size());
            }
            p
        }
        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc_zeroed(layout) };
            if !p.is_null() {
                allocated(layout.size());
            }
            p
        }
        unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
            LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
            unsafe { System.dealloc(p, layout) }
        }
        unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let out = unsafe { System.realloc(p, layout, size) };
            if !out.is_null() {
                LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
                allocated(size);
            }
            out
        }
    }
    pub fn snapshot(reset_peak: bool) -> [usize; 4] {
        let live = LIVE.load(Ordering::Relaxed);
        if reset_peak {
            PEAK.store(live, Ordering::Relaxed);
        }
        [
            CALLS.load(Ordering::Relaxed),
            BYTES.load(Ordering::Relaxed),
            live,
            PEAK.load(Ordering::Relaxed),
        ]
    }
}
include!("derivative-endpoint-corpus.rs");
fn main() {
    run(Some(allocation::snapshot));
}
