//! Completed projection/section costs with independent exact set checks.
use csgrs::{Real, curve, solid};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hash::Hasher;
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use std::time::Instant;

struct Counted;
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static REQUESTED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn allocated(bytes: usize) {
    ALLOCATIONS.fetch_add(1, Relaxed);
    REQUESTED.fetch_add(bytes, Relaxed);
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    PEAK.fetch_max(live, Relaxed);
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, old: Layout, size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, old, size) };
        if !result.is_null() {
            LIVE.fetch_sub(old.size(), Relaxed);
            allocated(size);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

macro_rules! construct {
    ($value:expr) => {{
        #[cfg(candidate_api)]
        {
            $value.unwrap()
        }
        #[cfg(not(candidate_api))]
        {
            $value
        }
    }};
}

fn complete(shape: &str, offset: u32) -> String {
    let width = Real::from(2 + offset);
    let mesh = solid::cube(width.clone()).unwrap();
    let region = if shape == "flatten" {
        construct!(solid::flatten(&mesh))
    } else {
        let z = match shape {
            "slice_middle" => (&width / Real::from(2)).unwrap(),
            "slice_bottom" => Real::zero(),
            "slice_top" => width.clone(),
            _ => panic!("unknown workload"),
        };
        let (region, strings, paths) = construct!(solid::slice_z(&mesh, z));
        assert!(strings.is_empty() && paths.is_empty());
        region
    };
    assert_eq!(region.filled_area().unwrap(), Some(&width * &width));
    let half_width = (&width / Real::from(2)).unwrap();
    assert_eq!(
        curve::contains_xy(&region, half_width.clone(), half_width).unwrap(),
        Some(true)
    );
    assert_eq!(
        curve::contains_xy(&region, (-1).into(), 1.into()).unwrap(),
        Some(false)
    );
    let profiles = region.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
    for edge in material.segments() {
        assert!(matches!(edge, hypercurve::Segment2::Line(_)));
        let [a, b] = [edge.start(), edge.end()];
        for p in [a, b] {
            assert!(p.x() >= &Real::zero() && p.x() <= &width);
            assert!(p.y() >= &Real::zero() && p.y() <= &width);
        }
        // Every straight boundary edge lies on the independently known square.
        assert!(
            (a.x() == b.x() && (a.x() == &Real::zero() || a.x() == &width))
                || (a.y() == b.y() && (a.y() == &Real::zero() || a.y() == &width))
        );
    }
    // Keep the actual materialized boundary rows: topology fixes may change
    // traversal or collinear subdivision. They are not a semantic oracle.
    let rows = material
        .segments()
        .iter()
        .map(|edge| {
            format!(
                "[{},{}]",
                edge.start().x().to_json(),
                edge.start().y().to_json()
            )
        })
        .collect::<Vec<_>>();
    format!("[{}]", rows.join(","))
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let shape = &args[1];
    let cache = &args[2];
    let iterations: usize = args[3].parse().unwrap();
    if cache == "disabled" {
        solid::cache::with_policy(solid::cache::CachePolicy::DISABLED, || {
            run(shape, cache, iterations)
        });
    } else {
        run(shape, cache, iterations);
    }
}

fn run(shape: &str, cache: &str, iterations: usize) {
    if cache == "dump" {
        let rows = (0..iterations)
            .map(|i| complete(shape, i as u32))
            .collect::<Vec<_>>();
        println!("[{}]", rows.join(",\n"));
        return;
    }
    assert!(cache == "warm" || cache == "varied" || cache == "disabled");
    let offset = |i: usize| if cache == "warm" { 0 } else { (i % 128) as u32 };
    for i in 0..128 {
        black_box(complete(shape, offset(i)));
    }
    let initial_live = LIVE.load(Relaxed);
    let initial_allocations = ALLOCATIONS.load(Relaxed);
    let initial_requested = REQUESTED.load(Relaxed);
    PEAK.store(initial_live, Relaxed);
    let mut checksum = std::collections::hash_map::DefaultHasher::new();
    let mut output_bytes = 0;
    let start = Instant::now();
    for i in 0..iterations {
        let rows = complete(shape, offset(i));
        output_bytes += rows.len();
        checksum.write(rows.as_bytes());
        black_box(rows);
    }
    let nanos = start.elapsed().as_nanos();
    let allocations = ALLOCATIONS.load(Relaxed) - initial_allocations;
    let requested = REQUESTED.load(Relaxed) - initial_requested;
    let retained = LIVE.load(Relaxed) as i128 - initial_live as i128;
    let peak = PEAK.load(Relaxed) - initial_live;
    let checksum = checksum.finish();
    println!(
        "{{\"shape\":\"{shape}\",\"cache\":\"{cache}\",\"iterations\":{iterations},\"completed\":{iterations},\"policy\":\"strict\",\"certainty\":\"Certified\",\"oracle\":\"independent exact area, membership and all boundary edges\",\"nanos\":{nanos},\"allocations\":{allocations},\"requested_bytes\":{requested},\"retained_byte_delta\":{retained},\"peak_live_bytes_above_start\":{peak},\"output_bytes\":{output_bytes},\"checksum\":{checksum}}}"
    );
}
