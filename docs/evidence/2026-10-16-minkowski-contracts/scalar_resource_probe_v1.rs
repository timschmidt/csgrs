use csgrs::{GeometryContext, Real};
use hypermesh::{Classification, Plane, Point3};
use std::{hint::black_box, time::Instant};
#[cfg(allocation)]
mod memory {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::hint::black_box;
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    struct CountingAllocator;
    static ALLOCS: AtomicUsize = AtomicUsize::new(0);
    static REALLOCS: AtomicUsize = AtomicUsize::new(0);
    static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
    static REALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);

    fn added(bytes: usize) {
        let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
        PEAK.fetch_max(live, Relaxed);
    }

    fn allocated(pointer: *mut u8, bytes: usize) -> *mut u8 {
        if !pointer.is_null() {
            ALLOCS.fetch_add(1, Relaxed);
            ALLOC_BYTES.fetch_add(bytes, Relaxed);
            added(bytes);
        }
        pointer
    }

    unsafe impl GlobalAlloc for CountingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            allocated(unsafe { System.alloc(layout) }, layout.size())
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            allocated(unsafe { System.alloc_zeroed(layout) }, layout.size())
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
            LIVE.fetch_sub(layout.size(), Relaxed);
        }

        unsafe fn realloc(
            &self,
            pointer: *mut u8,
            layout: Layout,
            new_size: usize,
        ) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, new_size) };
            if !result.is_null() {
                REALLOCS.fetch_add(1, Relaxed);
                REALLOC_BYTES.fetch_add(new_size, Relaxed);
                if new_size >= layout.size() {
                    added(new_size - layout.size());
                } else {
                    LIVE.fetch_sub(layout.size() - new_size, Relaxed);
                }
            }
            result
        }
    }

    #[global_allocator]
    static ALLOCATOR: CountingAllocator = CountingAllocator;

    #[derive(Clone, Copy)]
    pub struct Stats {
        pub allocations: usize,
        pub reallocations: usize,
        pub allocated_bytes: usize,
        pub reallocated_bytes: usize,
        pub live: usize,
    }

    pub fn stats() -> Stats {
        Stats {
            allocations: ALLOCS.load(Relaxed),
            reallocations: REALLOCS.load(Relaxed),
            allocated_bytes: ALLOC_BYTES.load(Relaxed),
            reallocated_bytes: REALLOC_BYTES.load(Relaxed),
            live: LIVE.load(Relaxed),
        }
    }

    pub fn calibrate() {
        let layout = Layout::from_size_align(4096, 8).unwrap();
        let before = stats();
        let pointer = unsafe { std::alloc::alloc(layout) };
        assert!(!pointer.is_null());
        black_box(pointer);
        let during = stats();
        unsafe { std::alloc::dealloc(pointer, layout) };
        let after = stats();
        assert_eq!(during.allocations - before.allocations, 1);
        assert_eq!(during.allocated_bytes - before.allocated_bytes, 4096);
        assert_eq!(during.live - before.live, 4096);
        assert_eq!(after.live, before.live);
    }

    pub fn reset_peak(live: usize) {
        PEAK.store(live, Relaxed);
    }
    pub fn peak() -> usize {
        PEAK.load(Relaxed)
    }
}

fn input(case: &str) -> Real {
    match case {
        "rational" => Real::from(7),
        "unresolved" => {
            let s = Real::e().sin();
            let c = Real::e().cos();
            &s * &s + &c * &c - Real::one()
        },
        "root-square" => Real::from_json(include_str!("fixtures/root-square.json")).unwrap(),
        "eighth-angle" => Real::from_json(include_str!("fixtures/eighth-angle.json")).unwrap(),
        _ => panic!("unknown case"),
    }
}
fn query(value: &Real) -> Option<Classification> {
    let plane = Plane::from_points(
        &Point3::origin(),
        &Point3::new(Real::zero(), Real::one(), Real::zero()),
        &Point3::new(Real::zero(), Real::zero(), Real::one()),
    );
    match hypermesh::classify_point(
        &GeometryContext::STRICT.mesh_context(),
        &Point3::new(value.clone(), Real::zero(), Real::zero()),
        &plane,
    ) {
        Ok(outcome) => {
            assert_eq!(outcome.certainty, hypermesh::MeshCertainty::Certified);
            Some(outcome.value)
        },
        Err(hypermesh::HypermeshError::PredicateUndecided { .. }) => None,
        Err(error) => panic!("unexpected error: {error:?}"),
    }
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let (revision, case, temperature) = (&args[1], &args[2], &args[3]);
    assert!(revision == "baseline" || revision == "candidate");
    assert!(temperature == "cold" || temperature == "warm");
    #[cfg(allocation)]
    memory::calibrate();
    let value = input(case);
    let original = value.to_json();
    if temperature == "warm" {
        black_box(query(&value));
    }
    #[cfg(allocation)]
    let before = memory::stats();
    #[cfg(allocation)]
    memory::reset_peak(before.live);
    let started = Instant::now();
    let result = black_box(query(black_box(&value)));
    let nanos = started.elapsed().as_nanos();
    #[cfg(allocation)]
    let after = memory::stats();
    #[cfg(allocation)]
    let metrics = format!(
        "{},{},{},{},{},{}",
        after.allocations - before.allocations,
        after.reallocations - before.reallocations,
        after.allocated_bytes - before.allocated_bytes,
        after.reallocated_bytes - before.reallocated_bytes,
        after.live as i128 - before.live as i128,
        memory::peak() - before.live
    );
    #[cfg(not(allocation))]
    let metrics = "0,0,0,0,0,0";
    let expected = match case.as_str() {
        "rational" => Some(Classification::Positive),
        "unresolved" => None,
        _ if revision == "baseline" => None,
        _ => Some(Classification::On),
    };
    assert_eq!(result, expected);
    assert_eq!(
        value.to_json(),
        original,
        "proof must preserve symbolic source"
    );
    std::fs::write(&args[4], original).unwrap();
    let completion = if result.is_some() {
        "certified"
    } else {
        "undecided"
    };
    println!(
        "revision,case,temperature,nanos,completion,allocations,reallocations,allocated_bytes,reallocated_bytes,live_query_delta_bytes,peak_additional_requested_bytes"
    );
    println!("{revision},{case},{temperature},{nanos},{completion},{metrics}");
}
