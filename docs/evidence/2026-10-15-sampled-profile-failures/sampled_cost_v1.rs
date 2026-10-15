//! Matched successful sampled-profile workloads, including exact boundary materialization.
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
    let i = Real::from(offset);
    let sqrt2 = Real::from(2).sqrt().unwrap();
    let (region, expected) = match shape {
        "arrow" => {
            let length = Real::from(2) + i;
            (
                construct!(curve::arrow(length.clone(), 1.into(), 1.into(), 2.into())),
                length + Real::one(),
            )
        },
        "trapezoid" => {
            let width = Real::one() + i;
            (
                construct!(curve::trapezoid(width.clone(), 3.into(), 2.into(), 5.into())),
                width + Real::from(3),
            )
        },
        "star" => {
            let radius = Real::from(2) + i;
            (
                construct!(curve::star(4, radius.clone(), 1.into())),
                Real::from(2) * radius * sqrt2,
            )
        },
        "teardrop" => {
            let length = Real::from(3) + i;
            (
                construct!(curve::teardrop(2.into(), length.clone(), 2)),
                length,
            )
        },
        "egg" => {
            let width = Real::from(2) + i;
            (
                construct!(curve::egg(width.clone(), 4.into(), 4)),
                Real::from(2) * width,
            )
        },
        "squircle" => {
            let width = Real::from(4) + i;
            (construct!(curve::squircle(width.clone(), 2.into(), 4)), width)
        },
        "sector" => {
            let radius = Real::one() + i;
            (
                construct!(curve::pie_slice(radius.clone(), 0.into(), 90.into(), 1)),
                (&radius * radius / Real::from(2)).unwrap(),
            )
        },
        "heart" => {
            let width = Real::from(2) + i;
            let expected = (Real::from(656) * &width * &sqrt2
                / (Real::from(32)
                    * (Real::from(18) + (Real::from(15) * sqrt2 / Real::from(2)).unwrap())))
            .unwrap();
            (construct!(curve::heart(width, 2.into(), 8)), expected)
        },
        "supershape" => {
            let radius = Real::one() + i;
            (
                construct!(curve::supershape(
                    radius.clone(),
                    radius.clone(),
                    4.into(),
                    2.into(),
                    2.into(),
                    2.into(),
                    4
                )),
                Real::from(2) * &radius * radius,
            )
        },
        _ => panic!("unknown workload"),
    };
    let actual = region.filled_area().unwrap().unwrap();
    assert_eq!(
        actual.certified_eq_until(&expected, -1024).as_bool(),
        Some(true)
    );
    let profiles = region.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1);
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty());
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
    assert!(cache == "warm" || cache == "miss");
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
        "{{\"shape\":\"{shape}\",\"cache\":\"{cache}\",\"iterations\":{iterations},\"completed\":{iterations},\"policy\":\"strict\",\"certainty\":\"Certified\",\"oracle\":\"independent exact area\",\"nanos\":{nanos},\"allocations\":{allocations},\"requested_bytes\":{requested},\"retained_byte_delta\":{retained},\"peak_live_bytes_above_start\":{peak},\"output_bytes\":{output_bytes},\"checksum\":{checksum}}}"
    );
}
