//! Per-step cost of successive-operation chains: time, allocation and sizes.
//!
//! Runs the shared corpus chains at their long lengths, one step at a time on
//! one thread, and prints a Markdown table. Each row reports the step's P39
//! outcome, its certified manifold validation, triangle and vertex counts, the
//! stored coordinate sizes, wall time, bytes allocated and peak live heap
//! growth during the step. These are focused timer-scale probes, not matched
//! benchmarks: compare rows of one run, and repeat runs before drawing any
//! performance conclusion.
//!
//! ```text
//! cargo bench --bench successive_operations [-- <chain-name-substring>]
//! ```

#[allow(dead_code)]
#[path = "../../tests/support/successive_operations.rs"]
mod chains;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicUsize, Ordering};

use chains::Chain;
use csgrs::solid::{self, SolidExt};

struct CountingAllocator;
static ALLOCATED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

fn grow(size: usize) {
    ALLOCATED.fetch_add(size, Ordering::Relaxed);
    let live = LIVE.fetch_add(size, Ordering::Relaxed) + size;
    PEAK.fetch_max(live, Ordering::Relaxed);
}

// SAFETY: The original pointers and layouts pass unchanged to System; the
// counters neither inspect allocations nor change their lifetime.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        grow(layout.size());
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        grow(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        if size >= layout.size() {
            grow(size - layout.size());
        } else {
            LIVE.fetch_sub(layout.size() - size, Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: CountingAllocator = CountingAllocator;

/// Allocation during one step: total bytes and peak live growth.
type StepAllocation = (usize, usize);

fn measured(chain: Chain, allocations: Rc<RefCell<Vec<StepAllocation>>>) -> Chain {
    let step = chain.step;
    Chain {
        name: chain.name,
        steps: chain.steps,
        initial: chain.initial,
        step: Box::new(move |index, body| {
            let live = LIVE.load(Ordering::Relaxed);
            let allocated = ALLOCATED.load(Ordering::Relaxed);
            PEAK.store(live, Ordering::Relaxed);
            let result = step(index, body);
            allocations.borrow_mut().push((
                ALLOCATED.load(Ordering::Relaxed) - allocated,
                PEAK.load(Ordering::Relaxed).saturating_sub(live),
            ));
            result
        }),
    }
}

/// The sampled 32 x 16 sphere drilled by a box. It decides under `STRICT`
/// through hyperreal's cyclotomic zero proof.
fn sampled_sphere_cut() -> Chain {
    Chain {
        name: "sampled 32x16 sphere cut",
        steps: 1,
        initial: Box::new(|| solid::sphere(chains::r(10), 32, 16).unwrap()),
        step: Box::new(|_, body| {
            let tool = solid::cuboid(chains::r(4), chains::r(4), chains::r(30))?.translated(
                chains::r(-2),
                chains::r(-2),
                chains::r(-15),
            )?;
            Ok(body.try_difference_with_context(&tool, &chains::STRICT)?)
        }),
    }
}

fn corpus() -> Vec<Chain> {
    vec![
        chains::square_drills(32).0,
        chains::polygon_drills(4, 16),
        chains::polygon_drills(6, 8),
        chains::polygon_drills(8, 6),
        chains::polygon_drills(16, 4),
        chains::polygon_drills(32, 2),
        chains::tilted_polygon_drills(32, 10, 4),
        chains::cross_bars(20).0,
        chains::rational_rotation_cuts(16),
        chains::degree_rotation_cuts(30, 8),
        chains::degree_rotation_cuts(45, 8),
        chains::degree_rotation_cuts(10, 3),
        chains::cube_unions(16).0,
        chains::polyhedron_unions(false, 16),
        chains::polyhedron_unions(true, 6),
        chains::feedback_boxes(6).0,
        chains::round_trip_drills("stl", 8),
        chains::round_trip_drills("obj", 8),
        chains::extruded_profile_cuts(6).0,
        chains::revolved_ring_cuts(6),
        sampled_sphere_cut(),
    ]
}

fn main() {
    // Cargo passes `--bench`; any other argument filters chain names.
    let filter = std::env::args()
        .skip(1)
        .find(|argument| !argument.starts_with("--"));
    let mut header = chains::TABLE_HEADER.lines();
    println!(
        "{} MB allocated | MB peak growth |",
        header.next().unwrap_or_default()
    );
    println!("{} ---: | ---: |", header.next().unwrap_or_default());
    for chain in corpus() {
        if filter
            .as_deref()
            .is_some_and(|filter| !chain.name.contains(filter))
        {
            continue;
        }
        let allocations = Rc::new(RefCell::new(Vec::new()));
        let chain = measured(chain, Rc::clone(&allocations));
        let records = chains::run(&chain, |_, _| {});
        let allocations = allocations.borrow();
        for (row, (allocated, peak)) in chains::table_rows(chain.name, &records)
            .into_iter()
            .zip(allocations.iter())
        {
            println!(
                "{row} {:.1} | {:.1} |",
                *allocated as f64 / 1e6,
                *peak as f64 / 1e6
            );
        }
    }
}
