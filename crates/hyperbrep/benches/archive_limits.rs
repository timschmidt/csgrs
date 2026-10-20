//! Archive admission costs and Rust allocator calls, with fixtures outside timing.

use hyperbrep::{
    ArchiveLimits, ArchiveResource, PersistenceError, Point3, RawModel, Real, builder,
};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Instant;

struct CountingAllocator;
static COUNTING: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);

fn record(size: usize) {
    if COUNTING.load(Ordering::Relaxed) {
        CALLS.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(size, Ordering::Relaxed);
    }
}

// SAFETY: every allocation operation delegates unchanged to System; the extra
// atomic counters neither inspect nor alter allocated memory or pointer ownership.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record(size);
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn measure(
    name: &str,
    iterations: usize,
    no_allocations: bool,
    mut operation: impl FnMut() -> usize,
) {
    CALLS.store(0, Ordering::Relaxed);
    BYTES.store(0, Ordering::Relaxed);
    COUNTING.store(true, Ordering::Relaxed);
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        checksum += black_box(operation());
    }
    let elapsed = started.elapsed();
    COUNTING.store(false, Ordering::Relaxed);
    let calls = CALLS.load(Ordering::Relaxed);
    let bytes = BYTES.load(Ordering::Relaxed);
    if no_allocations {
        assert_eq!((calls, bytes), (0, 0));
    }
    println!("allocation_metadata name={name} calls={calls} requested_bytes={bytes}");
    println!(
        "{name} {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations as u32
    );
}

fn rejected(
    json: &str,
    limits: &ArchiveLimits,
    resource: ArchiveResource,
    observed: usize,
) -> usize {
    let Err(PersistenceError::ResourceLimit(error)) =
        RawModel::from_json(black_box(json), black_box(limits))
    else {
        panic!("input must fail admission");
    };
    assert_eq!(error.resource, resource);
    assert_eq!(error.observed, observed);
    1
}

fn main() {
    let model = builder::cuboid(
        Point3::origin(),
        Point3::new(Real::from(2), Real::from(3), Real::from(4)),
    )
    .unwrap();
    let json = model.model().to_json().unwrap();
    let limits = ArchiveLimits::default();
    println!("archive_metadata bytes={}", json.len());
    measure("archive_limits/native_decode_encode:", 256, false, || {
        let raw = RawModel::from_json(black_box(&json), &limits).unwrap();
        let encoded = raw.to_json().unwrap();
        assert_eq!(encoded, json);
        encoded.len()
    });
    let oversized = "[".repeat(1_048_576);
    let limited = ArchiveLimits {
        input_bytes: oversized.len() - 1,
        ..limits
    };
    measure("archive_limits/reject_bytes:", 100_000, true, || {
        rejected(
            &oversized,
            &limited,
            ArchiveResource::InputBytes,
            oversized.len(),
        )
    });
    let deep = format!(
        "{{\"version\":8,\"ignored\":{}0{}}}",
        "[".repeat(50_000),
        "]".repeat(50_000)
    );
    let limited = ArchiveLimits {
        nesting_depth: 16,
        ..limits
    };
    measure("archive_limits/reject_depth:", 100_000, true, || {
        rejected(&deep, &limited, ArchiveResource::NestingDepth, 17)
    });
    let wide = format!("{{\"version\":8,\"ignored\":[{}0]}}", "0,".repeat(20_000));
    let limited = ArchiveLimits {
        tokens: 32,
        ..limits
    };
    measure("archive_limits/reject_tokens:", 100_000, true, || {
        rejected(&wide, &limited, ArchiveResource::Tokens, 33)
    });
    let limited = ArchiveLimits {
        records: 99,
        ..limits
    };
    measure("archive_limits/reject_records:", 10_000, false, || {
        rejected(&json, &limited, ArchiveResource::Records, 100)
    });
    let high_degree = r#"{"version":9,"curves":[{"Nurbs":{"degree":300,"control_points":[],"weights":[],"knots":[]}}]}"#;
    measure("archive_limits/reject_degree:", 10_000, false, || {
        rejected(high_degree, &limits, ArchiveResource::Degree, 300)
    });
    let bad = serde_json::json!(vec!["not an exact scalar"; 128]);
    let large_controls = serde_json::json!({"version":9,"pcurves":[{"RationalBezier":{"homogeneous_controls":bad}}]}).to_string();
    let limited = ArchiveLimits {
        geometry_elements: 64,
        ..limits
    };
    measure("archive_limits/reject_geometry:", 10_000, false, || {
        rejected(
            &large_controls,
            &limited,
            ArchiveResource::GeometryElements,
            128,
        )
    });
    let wide_wire = serde_json::json!({"version":9,"wires":[{"edge_uses":bad}]}).to_string();
    let limited = ArchiveLimits {
        topology_references: 64,
        ..limits
    };
    measure("archive_limits/reject_topology:", 10_000, false, || {
        rejected(
            &wide_wire,
            &limited,
            ArchiveResource::TopologyReferences,
            128,
        )
    });
}
