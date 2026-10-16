//! Absolute candidate costs of corrected fractional text, including extreme fields.
use csgrs::{
    Real, TriangleMesh,
    io::{obj, stl},
};
use hyperlattice::Point3;
use hypermesh::Triangle;
use std::{
    alloc::{GlobalAlloc, Layout, System},
    hint::black_box,
    path::PathBuf,
    sync::atomic::{AtomicUsize, Ordering},
    time::Instant,
};

struct Counted;
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn allocated(size: usize) {
    CALLS.fetch_add(1, Ordering::Relaxed);
    BYTES.fetch_add(size, Ordering::Relaxed);
    let live = LIVE.fetch_add(size, Ordering::Relaxed) + size;
    PEAK.fetch_max(live, Ordering::Relaxed);
}

// Every allocation is delegated to System with its original layout. Counters
// observe successful requests without allocating or changing pointer ownership.
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            allocated(layout.size());
        }
        ptr
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc_zeroed(layout) };
        if !ptr.is_null() {
            allocated(layout.size());
        }
        ptr
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) };
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let next = unsafe { System.realloc(ptr, layout, size) };
        if !next.is_null() {
            LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
            allocated(size);
        }
        next
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let output = PathBuf::from(&args[1]);
    let real = |value| Real::try_from(value).unwrap();
    let triangle = |points: [[f64; 3]; 3]| {
        TriangleMesh::new(
            points
                .map(|[x, y, z]| Point3::new(real(x), real(y), real(z)))
                .to_vec(),
            vec![Triangle::new(0, 1, 2)],
        )
    };
    let tiny = 2_f64.powi(-70);
    let cloud = TriangleMesh::new(
        [1.1, tiny, f64::from_bits(1), f64::MAX]
            .map(|value| Point3::new(real(value), real(-value), Real::zero()))
            .to_vec(),
        Vec::new(),
    );
    let fixtures = [
        ("obj-fractional-extremes", cloud, true),
        (
            "obj-ordinary-cube",
            csgrs::solid::cube(real(1.1)).unwrap(),
            true,
        ),
        (
            "ascii-fractional",
            triangle([[0.; 3], [1.1, 0., 0.], [0., 1., tiny]]),
            false,
        ),
        (
            "ascii-minimum-f32",
            triangle([
                [0.; 3],
                [1., 0., 0.],
                [1., f64::from(f32::from_bits(1)), 0.],
            ]),
            false,
        ),
    ];
    for (name, mesh, is_obj) in fixtures {
        let mut outputs = Vec::with_capacity(65);
        for (phase, count) in [("first", 1), ("warm", 64)] {
            let initial_live = LIVE.load(Ordering::Relaxed);
            PEAK.store(initial_live, Ordering::Relaxed);
            CALLS.store(0, Ordering::Relaxed);
            BYTES.store(0, Ordering::Relaxed);
            let start = Instant::now();
            for _ in 0..count {
                outputs.push(if is_obj {
                    obj::to_obj(black_box(&mesh), "cost").unwrap()
                } else {
                    stl::to_stl_ascii(black_box(&mesh), "cost").unwrap()
                });
            }
            let elapsed = start.elapsed();
            let calls = CALLS.load(Ordering::Relaxed);
            let bytes = BYTES.load(Ordering::Relaxed);
            let peak = PEAK.load(Ordering::Relaxed) - initial_live;
            let live_delta = LIVE.load(Ordering::Relaxed) as i128 - initial_live as i128;
            let last = outputs.last().unwrap();
            println!(
                "{{\"fixture\":\"{name}\",\"phase\":\"{phase}\",\"iterations\":{count},\"elapsed_ns\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"peak_additional_live_bytes\":{peak},\"retained_delta_bytes\":{live_delta},\"output_bytes\":{},\"output_capacity\":{},\"completion\":\"verified_exact_binary64_coordinate_projection\"}}",
                elapsed.as_nanos(),
                last.len(),
                last.capacity()
            );
        }
        for text in &outputs[1..] {
            assert_eq!(text, &outputs[0]);
        }
        if is_obj {
            let restored = obj::from_obj(outputs[0].as_bytes()).unwrap();
            assert_eq!(restored.positions.as_ref(), mesh.positions.as_ref());
            assert_eq!(restored.triangles.len(), mesh.triangles.len());
        } else {
            let rows: Vec<Vec<Real>> = outputs[0]
                .lines()
                .filter_map(|line| line.trim().strip_prefix("vertex "))
                .map(|line| {
                    line.split_whitespace()
                        .map(|value| value.parse().unwrap())
                        .collect()
                })
                .collect();
            assert_eq!(rows.len(), 3);
            for (row, point) in rows.iter().zip(mesh.positions.iter()) {
                assert_eq!(row, &vec![point.x.clone(), point.y.clone(), point.z.clone()]);
            }
        }
        std::fs::write(output.join(format!("{name}.txt")), &outputs[0]).unwrap();
    }
}
