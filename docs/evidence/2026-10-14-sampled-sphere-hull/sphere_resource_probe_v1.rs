//! Allocation traffic and retained requested bytes for original native spheres.
use csgrs::{GeometryCertainty, GeometryContext, Real, solid};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use std::time::Instant;

struct Counted;
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);

fn allocated(bytes: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(bytes, Relaxed);
    PEAK.fetch_max(LIVE.fetch_add(bytes, Relaxed) + bytes, Relaxed);
}

unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let result = unsafe { System.alloc(layout) };
        if !result.is_null() {
            allocated(layout.size());
        }
        result
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, layout, size) };
        if !result.is_null() {
            LIVE.fetch_sub(layout.size(), Relaxed);
            allocated(size);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn measure<T>(name: &str, body: impl FnOnce() -> T) -> T {
    let initial = LIVE.load(Relaxed);
    PEAK.store(initial, Relaxed);
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    let started = Instant::now();
    let result = body();
    let nanos = started.elapsed().as_nanos();
    let calls = CALLS.load(Relaxed);
    let bytes = BYTES.load(Relaxed);
    let retained = LIVE.load(Relaxed) as i128 - initial as i128;
    let peak = PEAK.load(Relaxed) - initial;
    println!(
        "{{\"phase\":\"{name}\",\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak},\"nanos_with_allocator_accounting\":{nanos}}}"
    );
    black_box(result)
}

fn main() {
    let case = std::env::args().nth(1).expect("frozen case name");
    let (radius, segments, stacks) = match case.as_str() {
        "fifteen_degrees" => (8_u8, 24, 12),
        "original_medium" => (10, 32, 16),
        "fifth_turn" => (8, 10, 7),
        "seventh_turn" => (8, 7, 5),
        _ => panic!("unknown frozen case"),
    };
    solid::cache::clear();
    println!("resource_case={case} repetitions=3 grammar_policy=default");
    let initial = LIVE.load(Relaxed);
    let mesh = measure("construct", || {
        solid::sphere(Real::from(radius), segments, stacks)
    });
    // Three calls share the original input graph. All output meshes remain
    // owned until after the third call; no validation or serialization occurs
    // between queries. The grammar cache does not retain convex-hull results.
    let cold = measure("cold_hull", || {
        solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT)
    });
    let warm = measure("warm_hull", || {
        solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT)
    });
    let replay = measure("replay_hull", || {
        solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT)
    });
    assert_eq!(mesh.positions.len(), segments * (stacks - 1) + 2);
    assert_eq!(mesh.triangles.len(), 2 * segments * (stacks - 1));
    assert!(mesh.is_closed_manifold());
    for (phase, result) in [("cold", &cold), ("warm", &warm), ("replay", &replay)] {
        println!("witness_phase={phase}");
        println!(
            "input case={case} radius={radius} segments={segments} stacks={stacks} positions={} triangles={}",
            mesh.positions.len(),
            mesh.triangles.len()
        );
        match result {
            Ok(outcome) => {
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                let hull = &outcome.value;
                assert_eq!(hull.positions.len(), mesh.positions.len());
                assert_eq!(hull.triangles.len(), 2 * hull.positions.len() - 4);
                assert!(hull.is_closed_manifold());
                assert!(mesh.positions.iter().all(|p| hull.positions.contains(p)));
                assert!(hull.positions.iter().all(|p| mesh.positions.contains(p)));
                let order: Vec<_> = hull
                    .positions
                    .iter()
                    .map(|point| {
                        mesh.positions
                            .iter()
                            .position(|source| source == point)
                            .unwrap()
                    })
                    .collect();
                println!("position_order={order:?}");
                println!("triangle_rows={:?}", hull.triangles);
                println!(
                    "result case={case} certified=true original_positions_preserved=true positions={} triangles={}",
                    hull.positions.len(),
                    hull.triangles.len()
                );
            },
            Err(error) => println!("result case={case} certified=false error={error:?}"),
        }
    }
    measure("drop_outputs_and_input", || drop((cold, warm, replay, mesh)));
    measure("clear_grammar_cache", solid::cache::clear);
    let retained = LIVE.load(Relaxed) as i128 - initial as i128;
    println!("{{\"lifecycle_retained_requested_bytes\":{retained}}}");
}
