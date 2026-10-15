//! Matched successful planar operations, with independent coordinate and area oracles.
use csgrs::{Real, curve};
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
    let base = curve::rectangle(width.clone(), Real::from(3)).unwrap();
    let region = match shape {
        "translate" => construct!(curve::translated(&base, 2.into(), (-1).into())),
        "scale" => construct!(curve::scaled(&base, (-2).into(), 3.into())),
        "rotate" => construct!(curve::rotated(&base, 90.into())),
        "matrix" => {
            let mut matrix = hyperlattice::Matrix4::identity();
            matrix.0[0][1] = 2.into();
            matrix.0[0][3] = 1.into();
            matrix.0[1][3] = 2.into();
            construct!(curve::transformed(&base, &matrix))
        },
        "query" => {
            let half_width = (&width / Real::from(2)).unwrap();
            assert_eq!(
                construct!(curve::contains_xy(&base, half_width, 1.into())),
                Some(true)
            );
            assert_eq!(
                construct!(curve::contains_xy(&base, 0.into(), 1.into())),
                None
            );
            assert_eq!(
                construct!(curve::contains_xy(&base, (-1).into(), 1.into())),
                Some(false)
            );
            base
        },
        "adapter_translate" => csgrs::adapter::CurveRegionF64::from_native(base)
            .translate(2.0, -1.0, 0.0)
            .unwrap()
            .into_native(),
        "adapter_scale" => csgrs::adapter::CurveRegionF64::from_native(base)
            .scale(-2.0, 3.0, 1.0)
            .unwrap()
            .into_native(),
        "adapter_rotate" => csgrs::adapter::CurveRegionF64::from_native(base)
            .rotate(0.0, 0.0, 90.0)
            .unwrap()
            .into_native(),
        _ => panic!("unknown workload"),
    };
    let scale = matches!(shape, "scale" | "adapter_scale");
    let expected_area = &width * Real::from(if scale { 18 } else { 3 });
    assert_eq!(
        region
            .filled_area()
            .unwrap()
            .unwrap()
            .certified_eq_until(&expected_area, -1024)
            .as_bool(),
        Some(true)
    );
    let expected = [
        (Real::zero(), Real::zero()),
        (width.clone(), Real::zero()),
        (width, Real::from(3)),
        (Real::zero(), Real::from(3)),
    ]
    .map(|(x, y)| match shape {
        "translate" | "adapter_translate" => (x + Real::from(2), y - Real::one()),
        "scale" | "adapter_scale" => (-Real::from(2) * x, Real::from(3) * y),
        "rotate" | "adapter_rotate" => (-y, x),
        "matrix" => (x + Real::from(2) * &y + Real::one(), y + Real::from(2)),
        "query" => (x, y),
        _ => unreachable!(),
    });
    let profiles = region.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
    assert_eq!(material.segments().len(), 4);
    for (x, y) in expected {
        assert!(material.segments().iter().any(|edge| {
            edge.start().x().certified_eq_until(&x, -1024).as_bool() == Some(true)
                && edge.start().y().certified_eq_until(&y, -1024).as_bool() == Some(true)
        }));
    }
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
    if cache == "dump" {
        let rows = (0..iterations)
            .map(|i| complete(shape, i as u32))
            .collect::<Vec<_>>();
        println!("[{}]", rows.join(",\n"));
        return;
    }
    assert!(cache == "warm" || cache == "varied");
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
        "{{\"shape\":\"{shape}\",\"cache\":\"{cache}\",\"iterations\":{iterations},\"completed\":{iterations},\"policy\":\"strict\",\"certainty\":\"Certified\",\"oracle\":\"independent exact coordinates and area\",\"nanos\":{nanos},\"allocations\":{allocations},\"requested_bytes\":{requested},\"retained_byte_delta\":{retained},\"peak_live_bytes_above_start\":{peak},\"output_bytes\":{output_bytes},\"checksum\":{checksum}}}"
    );
}
