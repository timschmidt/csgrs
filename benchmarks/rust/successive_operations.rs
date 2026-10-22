//! Per-step cost of successive-operation chains: time, allocation and sizes.
//!
//! Runs the shared corpus chains at their long lengths, one step at a time on
//! one thread, and prints a Markdown table. Each row reports the step's P39
//! outcome, its certified manifold validation, triangle and vertex counts, the
//! stored coordinate sizes, and the wall time of the step, of the validation
//! after it and of dropping the previous result. The counting allocator adds
//! the bytes allocated during the step, the peak live heap growth during the
//! step, the live heap when the step starts and the allocator calls during the
//! step. These are focused timer-scale probes, not matched benchmarks: compare
//! rows of one run, and repeat runs before drawing any performance conclusion.
//!
//! "Allocated" sums the size of every `alloc` and `alloc_zeroed` request and
//! the growth of every `realloc` that grows; a `realloc` that shrinks adds
//! nothing. It is allocation traffic, not memory in use. Live bytes add each
//! allocation, subtract each deallocation and change by a `realloc`'s net
//! size; "peak growth" is the step's highest live count minus its count at
//! the start. Every `alloc`, `alloc_zeroed` and `realloc` is one call.
//!
//! ```text
//! cargo bench --bench successive_operations [-- [--untouched] [<chain-name-substring>]]
//! ```
//!
//! `--untouched` feeds each result to the next step without validating it or
//! measuring its sizes, so no later step can reuse facts that validation
//! cached; the default validates between steps, as the corpus test does.
//! Setting `SUCCESSIVE_OPERATIONS_RAW=<path>` also writes every row in exact
//! units (bytes, nanoseconds) to that file as tab-separated values, after
//! header lines that record the revision, toolchain, profile and arguments.

#[allow(dead_code)]
#[path = "../../tests/support/successive_operations.rs"]
mod chains;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::RefCell;
use std::fmt::Write as _;
use std::rc::Rc;
use std::sync::atomic::{AtomicUsize, Ordering};

use chains::{Between, Chain, StepRecord};
use csgrs::solid::{self, SolidExt};

struct CountingAllocator;
static ALLOCATED: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static CALLS: AtomicUsize = AtomicUsize::new(0);

fn grow(size: usize) {
    CALLS.fetch_add(1, Ordering::Relaxed);
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
            CALLS.fetch_add(1, Ordering::Relaxed);
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

/// Allocation during one step, in bytes and calls.
#[derive(Clone, Copy)]
struct StepAllocation {
    allocated: usize,
    peak_growth: usize,
    live_at_start: usize,
    calls: usize,
}

fn measured(chain: Chain, allocations: Rc<RefCell<Vec<StepAllocation>>>) -> Chain {
    let step = chain.step;
    Chain {
        name: chain.name,
        steps: chain.steps,
        initial: chain.initial,
        step: Box::new(move |index, body| {
            let live = LIVE.load(Ordering::Relaxed);
            let allocated = ALLOCATED.load(Ordering::Relaxed);
            let calls = CALLS.load(Ordering::Relaxed);
            PEAK.store(live, Ordering::Relaxed);
            let result = step(index, body);
            allocations.borrow_mut().push(StepAllocation {
                allocated: ALLOCATED.load(Ordering::Relaxed) - allocated,
                peak_growth: PEAK.load(Ordering::Relaxed).saturating_sub(live),
                live_at_start: live,
                calls: CALLS.load(Ordering::Relaxed) - calls,
            });
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

/// First line of a command's output, or why it is unavailable.
fn command_line(program: &str, arguments: &[&str]) -> String {
    match std::process::Command::new(program).args(arguments).output() {
        Ok(output) if output.status.success() => String::from_utf8_lossy(&output.stdout)
            .lines()
            .next()
            .unwrap_or_default()
            .to_owned(),
        Ok(output) => format!("unavailable ({})", output.status),
        Err(error) => format!("unavailable ({error})"),
    }
}

fn raw_header(arguments: &[String]) -> String {
    let dirty = command_line("git", &["status", "--porcelain", "--untracked-files=no"]);
    format!(
        "# successive_operations raw rows\n# revision\t{}{}\n# rustc on PATH at run time\t{}\n# package\tcsgrs {}\n# profile\t{}\n# target\t{}-{}\n# arguments\t{}\n",
        command_line("git", &["rev-parse", "HEAD"]),
        if dirty.is_empty() {
            ""
        } else {
            " (modified tracked files)"
        },
        command_line("rustc", &["-V"]),
        env!("CARGO_PKG_VERSION"),
        if cfg!(debug_assertions) {
            "debug assertions"
        } else {
            "release"
        },
        std::env::consts::ARCH,
        std::env::consts::OS,
        arguments.join(" "),
    )
}

const RAW_COLUMNS: &str = "chain\tstep\toutcome\tmanifold\ttriangles\tvertices\trational\tsymbolic\tmax_denominator_bits\tmax_graph_nodes\tmax_graph_depth\tstep_ns\tvalidation_ns\tdrop_ns\tallocated_bytes\tpeak_growth_bytes\tlive_bytes_at_start\tallocator_calls\n";

fn raw_row(name: &str, record: &StepRecord, allocation: &StepAllocation) -> String {
    let outcome = match &record.outcome {
        chains::Outcome::Certified => "certified".to_owned(),
        chains::Outcome::Approximate => "approximate".to_owned(),
        chains::Outcome::Failed(kind, _) => kind.as_str().to_owned(),
    };
    let size = record.size;
    format!(
        "{name}\t{}\t{outcome}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n",
        record.step,
        record
            .closed_manifold
            .map_or("-", |closed| if closed { "yes" } else { "no" }),
        record.triangles,
        record.vertices,
        size.rational,
        size.symbolic,
        size.max_denominator_bits,
        size.max_nodes,
        size.max_depth,
        record.elapsed.as_nanos(),
        record
            .validation
            .map_or_else(|| "-".to_owned(), |duration| duration.as_nanos().to_string()),
        record.drop.as_nanos(),
        allocation.allocated,
        allocation.peak_growth,
        allocation.live_at_start,
        allocation.calls,
    )
}

fn main() {
    // Cargo passes `--bench`; `--untouched` selects the mode, and any other
    // argument filters chain names.
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    let between = if arguments.iter().any(|argument| argument == "--untouched") {
        Between::Untouched
    } else {
        Between::Validate
    };
    let filter = arguments.iter().find(|argument| !argument.starts_with("--"));
    let raw_path = std::env::var_os("SUCCESSIVE_OPERATIONS_RAW");
    let mut raw = raw_path
        .as_ref()
        .map(|_| raw_header(&arguments) + RAW_COLUMNS);
    let mut header = chains::TABLE_HEADER.lines();
    println!(
        "{} MB allocated | MB peak growth | MB live at start | allocator calls |",
        header.next().unwrap_or_default()
    );
    println!(
        "{} ---: | ---: | ---: | ---: |",
        header.next().unwrap_or_default()
    );
    for chain in corpus() {
        if filter.is_some_and(|filter| !chain.name.contains(filter.as_str())) {
            continue;
        }
        let allocations = Rc::new(RefCell::new(Vec::new()));
        let chain = measured(chain, Rc::clone(&allocations));
        let records = chains::run_with(&chain, between, |_, _| {});
        let allocations = allocations.borrow();
        for ((row, record), allocation) in chains::table_rows(chain.name, &records)
            .into_iter()
            .zip(&records)
            .zip(allocations.iter())
        {
            println!(
                "{row} {:.1} | {:.1} | {:.1} | {} |",
                allocation.allocated as f64 / 1e6,
                allocation.peak_growth as f64 / 1e6,
                allocation.live_at_start as f64 / 1e6,
                allocation.calls,
            );
            if let Some(raw) = raw.as_mut() {
                let _ = write!(raw, "{}", raw_row(chain.name, record, allocation));
            }
        }
    }
    if let (Some(path), Some(raw)) = (raw_path, raw) {
        std::fs::write(&path, raw).unwrap_or_else(|error| {
            panic!("cannot write {}: {error}", path.to_string_lossy())
        });
    }
}
