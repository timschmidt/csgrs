//! Focused source-validation cost probe, independently checked tetrahedron queries.
use hypermesh::{
    MeshCertainty as GeometryCertainty, MeshContext as GeometryContext, PredicatePolicy, Real,
    TriangleMesh,
};
use hypermesh::{Point3, Triangle};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, AtomicUsize, Ordering::Relaxed};
use std::time::Instant;
struct Counting;
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicIsize = AtomicIsize::new(0);
static PEAK: AtomicIsize = AtomicIsize::new(0);
fn allocated(n: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(n, Relaxed);
    let live = LIVE.fetch_add(n as isize, Relaxed) + n as isize;
    PEAK.fetch_max(live, Relaxed);
}
unsafe impl GlobalAlloc for Counting {
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
        unsafe { System.dealloc(p, layout) };
        LIVE.fetch_sub(layout.size() as isize, Relaxed);
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, n: usize) -> *mut u8 {
        let q = unsafe { System.realloc(p, layout, n) };
        if !q.is_null() {
            LIVE.fetch_sub(layout.size() as isize, Relaxed);
            allocated(n);
        }
        q
    }
}
#[global_allocator]
static ALLOC: Counting = Counting;
fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}
fn source() -> TriangleMesh {
    TriangleMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}
fn queries(
    mesh: &TriangleMesh,
    context: &GeometryContext,
    cases: &[(Point3, bool)],
) -> Vec<bool> {
    cases
        .iter()
        .map(|(point, expected)| {
            let outcome = mesh.contains_point(context, point).unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert_eq!(outcome.value, *expected);
            outcome.value
        })
        .collect()
}
fn main() {
    let eighth = (Real::one() / Real::from(8)).unwrap();
    let cases = [
        (Point3::new(eighth.clone(), eighth.clone(), eighth), true),
        (p(0, 0, 0), false),
        (p(1, 1, 1), false),
        (p(10, 10, 10), false),
    ];
    let mut rows = Vec::new();
    for (policy, context) in [
        ("strict", GeometryContext::new(PredicatePolicy::STRICT)),
        (
            "approximate",
            GeometryContext::new(PredicatePolicy::APPROXIMATE_512),
        ),
    ] {
        let mut meshes = Vec::new();
        for phase in ["cold_construct_query", "warm_query"] {
            let before_live = LIVE.load(Relaxed);
            PEAK.store(before_live, Relaxed);
            let before_calls = CALLS.load(Relaxed);
            let before_bytes = BYTES.load(Relaxed);
            let start = Instant::now();
            let mut values = Vec::new();
            if phase == "cold_construct_query" {
                for _ in 0..16 {
                    let mesh = source();
                    values.extend(queries(&mesh, &context, &cases));
                    meshes.push(mesh);
                }
            } else {
                for mesh in &meshes {
                    values.extend(queries(mesh, &context, &cases));
                }
            }
            let elapsed = start.elapsed().as_nanos();
            let calls = CALLS.load(Relaxed) - before_calls;
            let bytes = BYTES.load(Relaxed) - before_bytes;
            let live = LIVE.load(Relaxed) - before_live;
            let peak = PEAK.load(Relaxed) - before_live;
            rows.push(format!("{{\"policy\":\"{policy}\",\"phase\":\"{phase}\",\"completed\":64,\"certified\":64,\"elapsed_ns\":{elapsed},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_requested_bytes_delta\":{live},\"peak_extra_requested_bytes\":{peak},\"values\":{values:?}}}"));
        }
        let before = LIVE.load(Relaxed);
        drop(meshes);
        let after = LIVE.load(Relaxed);
        rows.push(format!("{{\"policy\":\"{policy}\",\"phase\":\"drop_meshes\",\"released_requested_bytes\":{}}}",before-after));
    }
    println!("[{}]", rows.join(","));
}
