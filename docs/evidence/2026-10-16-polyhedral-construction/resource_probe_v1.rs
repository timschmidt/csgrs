use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hypermesh::Point3;
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

fn r(n: i32) -> Real { n.into() }
fn build(case: &str) -> TriangleMesh {
    let outcome = match case {
        "tetrahedron" => solid::polyhedron_with_context(
            &[[r(0),r(0),r(0)],[r(2),r(0),r(0)],[r(0),r(2),r(0)],[r(0),r(0),r(2)]],
            &[&[0,2,1],&[0,1,3],&[0,3,2],&[1,2,3]], &GeometryContext::STRICT),
        "loft-square" => solid::loft_with_context(&[section(2,0),section(4,8)], &GeometryContext::STRICT),
        _ => panic!("unknown fixture"),
    }.unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    outcome.value
}
fn section(half: i32, z: i32) -> Vec<Point3> {
    [(-half,-half),(half,-half),(half,half),(-half,half)].into_iter()
        .map(|(x,y)| Point3::new(r(x),r(y),r(z))).collect()
}
fn record(mesh: &TriangleMesh) -> String {
    let points = mesh
        .positions
        .iter()
        .map(|p| format!("[{},{},{}]", p.x.to_json(), p.y.to_json(), p.z.to_json()))
        .collect::<Vec<_>>();
    let triangles = mesh
        .triangles
        .iter()
        .map(|t| format!("[{},{},{}]", t.v0, t.v1, t.v2))
        .collect::<Vec<_>>();
    format!(
        "{{\"positions\":[{}],\"triangles\":[{}]}}\n",
        points.join(","),
        triangles.join(",")
    )
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let (case, temperature) = (&args[1], &args[2]);
    assert!(temperature == "cold" || temperature == "warm");
    #[cfg(allocation)]
    memory::calibrate();
    if temperature == "warm" { drop(black_box(build(case))); }
    #[cfg(allocation)]
    let before = memory::stats();
    #[cfg(allocation)]
    memory::reset_peak(before.live);
    let started = Instant::now();
    let value = black_box(build(case));
    let nanos = started.elapsed().as_nanos();
    #[cfg(allocation)]
    let after = memory::stats();
    #[cfg(allocation)]
    let peak = memory::peak();
    drop(value);
    #[cfg(allocation)]
    let retained = memory::stats().live as i128 - before.live as i128;
    #[cfg(allocation)]
    let metrics = format!("{},{},{},{},{},{},{}", after.allocations - before.allocations,
        after.reallocations - before.reallocations, after.allocated_bytes - before.allocated_bytes,
        after.reallocated_bytes - before.reallocated_bytes, after.live as i128 - before.live as i128,
        peak - before.live, retained);
    #[cfg(not(allocation))]
    let metrics = "0,0,0,0,0,0,0";
    // Serialization replay starts after construction/drop snapshots.
    let replay = build(case);
    let serialized = record(&replay);
    let (positions, triangles, bytes) = (replay.positions.len(), replay.triangles.len(), serialized.len());
    assert!(positions > 0 && triangles > 0);
    std::fs::write(&args[3], serialized).unwrap();
    let certainty = "certified_local";
    println!("case,temperature,nanos,positions,triangles,serialized_bytes,certainty,allocations,reallocations,allocated_bytes,reallocated_bytes,live_delta_bytes,peak_additional_requested_bytes,retained_delta_bytes");
    println!("{case},{temperature},{nanos},{positions},{triangles},{bytes},{certainty},{metrics}");
}
