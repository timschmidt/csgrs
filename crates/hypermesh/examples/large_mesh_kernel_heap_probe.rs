#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;
#[path = "support/large_mesh_probe.rs"]
mod large_mesh_probe;
#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod mesh_common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering};

use hypermesh::{BooleanProgram, MeshContext, PredicatePolicy, boolean};
use large_mesh_probe::{FIXTURE_HELP, input_views, prepare_large_fixture, prime_input};

struct TrackingAllocator;

static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE_BLOCKS: AtomicUsize = AtomicUsize::new(0);
static ALLOCATION_CALLS: AtomicUsize = AtomicUsize::new(0);
static DEALLOCATION_CALLS: AtomicUsize = AtomicUsize::new(0);
static REALLOCATION_CALLS: AtomicUsize = AtomicUsize::new(0);
static ADDED_BYTES: AtomicUsize = AtomicUsize::new(0);
static REMOVED_BYTES: AtomicUsize = AtomicUsize::new(0);

#[global_allocator]
static ALLOCATOR: TrackingAllocator = TrackingAllocator;

unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            record_allocation(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        record_deallocation(layout.size());
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let resized = unsafe { System.realloc(pointer, layout, new_size) };
        if !resized.is_null() {
            record_reallocation(layout.size(), new_size);
        }
        resized
    }
}

fn record_allocation(bytes: usize) {
    ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
    LIVE_BLOCKS.fetch_add(1, Ordering::Relaxed);
    ADDED_BYTES.fetch_add(bytes, Ordering::Relaxed);
    let live = LIVE_BYTES.fetch_add(bytes, Ordering::Relaxed) + bytes;
    PEAK_BYTES.fetch_max(live, Ordering::Relaxed);
}

fn record_deallocation(bytes: usize) {
    DEALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
    LIVE_BLOCKS.fetch_sub(1, Ordering::Relaxed);
    REMOVED_BYTES.fetch_add(bytes, Ordering::Relaxed);
    LIVE_BYTES.fetch_sub(bytes, Ordering::Relaxed);
}

fn record_reallocation(old_bytes: usize, new_bytes: usize) {
    REALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
    match new_bytes.cmp(&old_bytes) {
        std::cmp::Ordering::Greater => {
            let growth = new_bytes - old_bytes;
            ADDED_BYTES.fetch_add(growth, Ordering::Relaxed);
            let live = LIVE_BYTES.fetch_add(growth, Ordering::Relaxed) + growth;
            PEAK_BYTES.fetch_max(live, Ordering::Relaxed);
        }
        std::cmp::Ordering::Less => {
            let reduction = old_bytes - new_bytes;
            REMOVED_BYTES.fetch_add(reduction, Ordering::Relaxed);
            LIVE_BYTES.fetch_sub(reduction, Ordering::Relaxed);
        }
        std::cmp::Ordering::Equal => {}
    }
}

#[derive(Clone, Copy)]
struct AllocationSnapshot {
    live_bytes: usize,
    live_blocks: usize,
    allocation_calls: usize,
    deallocation_calls: usize,
    reallocation_calls: usize,
    added_bytes: usize,
    removed_bytes: usize,
}

impl AllocationSnapshot {
    fn capture() -> Self {
        Self {
            live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
            live_blocks: LIVE_BLOCKS.load(Ordering::Relaxed),
            allocation_calls: ALLOCATION_CALLS.load(Ordering::Relaxed),
            deallocation_calls: DEALLOCATION_CALLS.load(Ordering::Relaxed),
            reallocation_calls: REALLOCATION_CALLS.load(Ordering::Relaxed),
            added_bytes: ADDED_BYTES.load(Ordering::Relaxed),
            removed_bytes: REMOVED_BYTES.load(Ordering::Relaxed),
        }
    }
}

struct AllocationInterval {
    start: AllocationSnapshot,
}

struct AllocationIntervalResult {
    peak_live_bytes: usize,
    end: AllocationSnapshot,
    allocation_calls: usize,
    deallocation_calls: usize,
    reallocation_calls: usize,
    added_bytes: usize,
    removed_bytes: usize,
}

impl AllocationInterval {
    fn begin() -> Self {
        let start = AllocationSnapshot::capture();
        PEAK_BYTES.store(start.live_bytes, Ordering::Relaxed);
        Self { start }
    }

    fn finish(self) -> AllocationIntervalResult {
        let end = AllocationSnapshot::capture();
        let peak_live_bytes = PEAK_BYTES.load(Ordering::Relaxed);
        let allocation_calls = end.allocation_calls - self.start.allocation_calls;
        let deallocation_calls = end.deallocation_calls - self.start.deallocation_calls;
        let reallocation_calls = end.reallocation_calls - self.start.reallocation_calls;
        let added_bytes = end.added_bytes - self.start.added_bytes;
        let removed_bytes = end.removed_bytes - self.start.removed_bytes;
        assert!(peak_live_bytes >= self.start.live_bytes.max(end.live_bytes));
        assert_eq!(
            signed_delta(end.live_bytes, self.start.live_bytes),
            signed_delta(added_bytes, removed_bytes),
            "allocator byte accounting is inconsistent"
        );
        assert_eq!(
            signed_delta(end.live_blocks, self.start.live_blocks),
            signed_delta(allocation_calls, deallocation_calls),
            "allocator block accounting is inconsistent"
        );
        AllocationIntervalResult {
            peak_live_bytes,
            end,
            allocation_calls,
            deallocation_calls,
            reallocation_calls,
            added_bytes,
            removed_bytes,
        }
    }
}

fn signed_delta(after: usize, before: usize) -> i128 {
    after as i128 - before as i128
}

fn main() {
    let mut args = std::env::args().skip(1);
    let fixture = args.next().expect(FIXTURE_HELP);
    let (policy_name, policy) = match args.next().as_deref() {
        Some("strict") => ("STRICT", PredicatePolicy::STRICT),
        Some("approximate-512") => ("APPROXIMATE_512", PredicatePolicy::APPROXIMATE_512),
        _ => panic!("expected strict or approximate-512"),
    };
    assert!(
        args.next().is_none(),
        "expected exactly one fixture and one policy"
    );

    let process_baseline = AllocationSnapshot::capture();
    let prepared = prepare_large_fixture(&fixture);
    let name = prepared.name;
    let meshes = prepared.meshes;
    let input_triangles = meshes[0].triangles.len() + meshes[1].triangles.len();
    let context = MeshContext::new(policy);

    let (result, input_retained, kernel) = {
        prime_input(&context, &meshes, prepared.input_path);
        let views = input_views(&meshes, prepared.input_path);
        let input_retained = AllocationSnapshot::capture();
        let interval = AllocationInterval::begin();
        let result = boolean(
            &context,
            black_box(&views),
            BooleanProgram::Operation(prepared.operation),
        )
        .expect("large fixture Boolean must complete under the selected policy");
        (result, input_retained, interval.finish())
    };

    let certainty = result.certainty;
    let output_vertices = result.value.vertices.len();
    let output_triangles = result.value.results[0].triangles.len();
    let post_boolean = kernel.end;
    drop(result);
    let after_output_drop = AllocationSnapshot::capture();
    drop(meshes);
    let after_input_drop = AllocationSnapshot::capture();

    println!(
        "{name}: policy={policy_name}, certainty={certainty:?}, input_triangles={input_triangles}, \
         output_vertices={output_vertices}, output_triangles={output_triangles}"
    );
    println!(
        "kernel_heap: metric=global_allocator_requested_payload, \
         process_baseline_live_bytes={}, process_baseline_live_blocks={}, \
         input_retained_live_bytes={}, input_retained_live_blocks={}, \
         input_payload_bytes={}, input_payload_blocks={}, \
         boolean_peak_live_bytes={}, kernel_peak_incremental_bytes={}, \
         post_boolean_retained_incremental_bytes={}, output_live_payload_bytes={}, \
         input_fact_growth_bytes={}, \
         post_input_drop_incremental_bytes={}, allocation_calls={}, deallocation_calls={}, \
         reallocation_calls={}, added_bytes={}, removed_bytes={}",
        process_baseline.live_bytes,
        process_baseline.live_blocks,
        input_retained.live_bytes,
        input_retained.live_blocks,
        signed_delta(input_retained.live_bytes, process_baseline.live_bytes),
        signed_delta(input_retained.live_blocks, process_baseline.live_blocks),
        kernel.peak_live_bytes,
        kernel.peak_live_bytes - input_retained.live_bytes,
        signed_delta(post_boolean.live_bytes, input_retained.live_bytes),
        signed_delta(post_boolean.live_bytes, after_output_drop.live_bytes),
        signed_delta(after_output_drop.live_bytes, input_retained.live_bytes),
        signed_delta(after_input_drop.live_bytes, process_baseline.live_bytes),
        kernel.allocation_calls,
        kernel.deallocation_calls,
        kernel.reallocation_calls,
        kernel.added_bytes,
        kernel.removed_bytes,
    );
}
