//! Isolated allocator instrumentation for every comparable operation plus
//! retained-geometry size sweeps.

use std::alloc::{GlobalAlloc, Layout, System};
use std::env;
use std::ffi::c_int;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use exactcore_hyper_comparison::{
    COMPARABLE_OPERATION_IDS, HyperMemoryOperation, PreparedBenchmark,
};
use hyperlimit::{
    Point2, Point3, PredicateOutcome, PredicatePolicy, classify_point_line,
    classify_point_triangle3, classify_triangle_triangle3,
};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const ALLOCATION_MAGIC: u64 = 0x4859_5045_524d_454d;
const DEFAULT_OUTPUT: &str = "target/memory-sweep/results.tsv";

static TRACKING_ENABLED: AtomicBool = AtomicBool::new(false);
static ALLOCATION_COUNT: AtomicU64 = AtomicU64::new(0);
static REALLOCATION_COUNT: AtomicU64 = AtomicU64::new(0);
static DEALLOCATION_COUNT: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static REALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static DEALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static LIVE_BYTES: AtomicU64 = AtomicU64::new(0);
static PEAK_LIVE_BYTES: AtomicU64 = AtomicU64::new(0);

#[repr(C, align(16))]
#[derive(Clone, Copy)]
struct AllocationHeader {
    requested_size: usize,
    magic: u64,
    counted: bool,
}

struct TrackingAllocator;

fn allocation_layout(layout: Layout) -> Option<(Layout, usize)> {
    let (combined, offset) = Layout::new::<AllocationHeader>().extend(layout).ok()?;
    Some((combined.pad_to_align(), offset))
}

fn update_peak(candidate: u64) {
    let mut peak = PEAK_LIVE_BYTES.load(Ordering::Relaxed);
    while candidate > peak {
        match PEAK_LIVE_BYTES.compare_exchange_weak(
            peak,
            candidate,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(observed) => peak = observed,
        }
    }
}

fn record_allocation(size: usize) {
    ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
    ALLOCATED_BYTES.fetch_add(size as u64, Ordering::Relaxed);
    let live = LIVE_BYTES.fetch_add(size as u64, Ordering::Relaxed) + size as u64;
    update_peak(live);
}

fn record_reallocation(old_size: usize, new_size: usize, was_counted: bool) {
    REALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
    REALLOCATED_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
    if was_counted {
        resize_live_bytes(old_size, new_size);
    } else {
        let live = LIVE_BYTES.fetch_add(new_size as u64, Ordering::Relaxed) + new_size as u64;
        update_peak(live);
    }
}

fn resize_live_bytes(old_size: usize, new_size: usize) {
    if new_size >= old_size {
        let added = (new_size - old_size) as u64;
        let live = LIVE_BYTES.fetch_add(added, Ordering::Relaxed) + added;
        update_peak(live);
    } else {
        LIVE_BYTES.fetch_sub((old_size - new_size) as u64, Ordering::Relaxed);
    }
}

fn record_deallocation(size: usize, count_event: bool) {
    LIVE_BYTES.fetch_sub(size as u64, Ordering::Relaxed);
    if count_event {
        DEALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
        DEALLOCATED_BYTES.fetch_add(size as u64, Ordering::Relaxed);
    }
}

// SAFETY: every returned pointer comes from `System` with a layout extended by
// a private header. Deallocation and reallocation reconstruct exactly that
// layout from the caller-supplied original layout.
unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let Some((combined, offset)) = allocation_layout(layout) else {
            return std::ptr::null_mut();
        };
        // SAFETY: `combined` is a valid layout.
        let base = unsafe { System.alloc(combined) };
        if base.is_null() {
            return base;
        }
        let counted = TRACKING_ENABLED.load(Ordering::Relaxed);
        let header = AllocationHeader {
            requested_size: layout.size(),
            magic: ALLOCATION_MAGIC,
            counted,
        };
        // SAFETY: `base` has room and alignment for the header and payload.
        unsafe { base.cast::<AllocationHeader>().write(header) };
        if counted {
            record_allocation(layout.size());
        }
        // SAFETY: `offset` is the payload offset returned by `Layout::extend`.
        unsafe { base.add(offset) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let Some((combined, offset)) = allocation_layout(layout) else {
            return std::ptr::null_mut();
        };
        // SAFETY: `combined` is a valid layout.
        let base = unsafe { System.alloc_zeroed(combined) };
        if base.is_null() {
            return base;
        }
        let counted = TRACKING_ENABLED.load(Ordering::Relaxed);
        let header = AllocationHeader {
            requested_size: layout.size(),
            magic: ALLOCATION_MAGIC,
            counted,
        };
        // SAFETY: `base` has room and alignment for the header and payload.
        unsafe { base.cast::<AllocationHeader>().write(header) };
        if counted {
            record_allocation(layout.size());
        }
        // SAFETY: `offset` is the payload offset returned by `Layout::extend`.
        unsafe { base.add(offset) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        let Some((combined, offset)) = allocation_layout(layout) else {
            std::process::abort();
        };
        // SAFETY: the pointer was returned at this deterministic offset.
        let base = unsafe { pointer.sub(offset) };
        // SAFETY: the header was initialized by this allocator.
        let header = unsafe { base.cast::<AllocationHeader>().read() };
        if header.magic != ALLOCATION_MAGIC || header.requested_size != layout.size() {
            std::process::abort();
        }
        if header.counted {
            record_deallocation(
                header.requested_size,
                TRACKING_ENABLED.load(Ordering::Relaxed),
            );
        }
        // SAFETY: `base` and `combined` match the allocation call.
        unsafe { System.dealloc(base, combined) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let Some((old_combined, old_offset)) = allocation_layout(layout) else {
            return std::ptr::null_mut();
        };
        let Ok(new_payload) = Layout::from_size_align(new_size, layout.align()) else {
            return std::ptr::null_mut();
        };
        let Some((new_combined, new_offset)) = allocation_layout(new_payload) else {
            return std::ptr::null_mut();
        };
        if old_offset != new_offset || old_combined.align() != new_combined.align() {
            return std::ptr::null_mut();
        }
        // SAFETY: the pointer was returned at `old_offset` by this allocator.
        let base = unsafe { pointer.sub(old_offset) };
        // SAFETY: the existing header remains initialized until realloc succeeds.
        let old_header = unsafe { base.cast::<AllocationHeader>().read() };
        if old_header.magic != ALLOCATION_MAGIC || old_header.requested_size != layout.size() {
            std::process::abort();
        }
        // SAFETY: `base` and `old_combined` identify the old allocation; the
        // new combined layout has the same alignment.
        let new_base = unsafe { System.realloc(base, old_combined, new_combined.size()) };
        if new_base.is_null() {
            // Restore the header because `read` moved it logically.
            // SAFETY: failed realloc leaves the old allocation valid.
            unsafe { base.cast::<AllocationHeader>().write(old_header) };
            return new_base;
        }
        let enabled = TRACKING_ENABLED.load(Ordering::Relaxed);
        let counted = old_header.counted || enabled;
        let new_header = AllocationHeader {
            requested_size: new_size,
            magic: ALLOCATION_MAGIC,
            counted,
        };
        // SAFETY: successful realloc returned space for the new header.
        unsafe { new_base.cast::<AllocationHeader>().write(new_header) };
        if enabled {
            record_reallocation(old_header.requested_size, new_size, old_header.counted);
        } else if old_header.counted {
            resize_live_bytes(old_header.requested_size, new_size);
        }
        // SAFETY: `new_offset` is the extended payload offset.
        unsafe { new_base.add(new_offset) }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: TrackingAllocator = TrackingAllocator;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct AllocationStats {
    allocation_count: u64,
    reallocation_count: u64,
    deallocation_count: u64,
    allocated_bytes: u64,
    reallocated_bytes: u64,
    deallocated_bytes: u64,
    live_bytes: u64,
    peak_live_bytes: u64,
}

impl AllocationStats {
    fn rust_snapshot() -> Self {
        Self {
            allocation_count: ALLOCATION_COUNT.load(Ordering::Relaxed),
            reallocation_count: REALLOCATION_COUNT.load(Ordering::Relaxed),
            deallocation_count: DEALLOCATION_COUNT.load(Ordering::Relaxed),
            allocated_bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
            reallocated_bytes: REALLOCATED_BYTES.load(Ordering::Relaxed),
            deallocated_bytes: DEALLOCATED_BYTES.load(Ordering::Relaxed),
            live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
            peak_live_bytes: PEAK_LIVE_BYTES.load(Ordering::Relaxed),
        }
    }
}

fn rust_tracking_reset() {
    ALLOCATION_COUNT.store(0, Ordering::Relaxed);
    REALLOCATION_COUNT.store(0, Ordering::Relaxed);
    DEALLOCATION_COUNT.store(0, Ordering::Relaxed);
    ALLOCATED_BYTES.store(0, Ordering::Relaxed);
    REALLOCATED_BYTES.store(0, Ordering::Relaxed);
    DEALLOCATED_BYTES.store(0, Ordering::Relaxed);
    PEAK_LIVE_BYTES.store(LIVE_BYTES.load(Ordering::Relaxed), Ordering::Relaxed);
}

fn rust_tracking_enable(enabled: bool) {
    TRACKING_ENABLED.store(enabled, Ordering::SeqCst);
}

#[repr(C)]
struct ExactMemoryFixture {
    _private: [u8; 0],
}

unsafe extern "C" {
    fn ec_memory_tracking_init() -> c_int;
    fn ec_memory_tracking_enable(enabled: c_int);
    fn ec_memory_tracking_reset();
    fn ec_memory_tracking_snapshot(output: *mut AllocationStats);
    fn ec_memory_release_caches();
    fn ec_memory_fixture_new(
        workload: c_int,
        coordinates: *const i64,
        case_count: usize,
    ) -> *mut ExactMemoryFixture;
    fn ec_memory_fixture_run(
        fixture: *mut ExactMemoryFixture,
        repetitions: u64,
        materialize_outputs: c_int,
    ) -> u64;
    fn ec_memory_fixture_free(fixture: *mut ExactMemoryFixture);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Library {
    ExactCore,
    Hyper,
}

impl Library {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "exactCorelib" | "exact" => Ok(Self::ExactCore),
            "Hyper" | "hyper" => Ok(Self::Hyper),
            _ => Err(format!("unknown library {value}")),
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::ExactCore => "exactCorelib",
            Self::Hyper => "Hyper",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Workload {
    LinePoint,
    TrianglePoint,
    TrianglePair,
}

impl Workload {
    const ALL: [Self; 3] = [Self::LinePoint, Self::TrianglePoint, Self::TrianglePair];

    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "line-point" => Ok(Self::LinePoint),
            "triangle-point" => Ok(Self::TrianglePoint),
            "triangle-pair" => Ok(Self::TrianglePair),
            _ => Err(format!("unknown workload {value}")),
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::LinePoint => "line-point",
            Self::TrianglePoint => "triangle-point",
            Self::TrianglePair => "triangle-pair",
        }
    }

    const fn exact_id(self) -> c_int {
        match self {
            Self::LinePoint => 0,
            Self::TrianglePoint => 1,
            Self::TrianglePair => 2,
        }
    }

    const fn coordinates_per_case(self) -> usize {
        match self {
            Self::LinePoint => 6,
            Self::TrianglePoint => 12,
            Self::TrianglePair => 18,
        }
    }

    const fn default_sizes(self) -> &'static [usize] {
        match self {
            Self::LinePoint => &[8, 32, 128, 512, 2_048, 8_192],
            Self::TrianglePoint => &[4, 16, 64, 256, 1_024, 4_096],
            Self::TrianglePair => &[1, 4, 16, 64, 256, 1_024],
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OutputMode {
    Streaming,
    Materialized,
}

impl OutputMode {
    const ALL: [Self; 2] = [Self::Streaming, Self::Materialized];

    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "streaming" => Ok(Self::Streaming),
            "materialized" => Ok(Self::Materialized),
            _ => Err(format!("unknown output mode {value}")),
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Streaming => "streaming",
            Self::Materialized => "materialized",
        }
    }

    const fn materializes(self) -> bool {
        matches!(self, Self::Materialized)
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ProcessMemory {
    rss_bytes: u64,
    pss_bytes: u64,
    private_bytes: u64,
    vm_hwm_bytes: u64,
}

fn parse_kib_field(text: &str, field: &str) -> Option<u64> {
    text.lines().find_map(|line| {
        let rest = line.strip_prefix(field)?;
        let kib = rest.split_whitespace().next()?.parse::<u64>().ok()?;
        Some(kib * 1_024)
    })
}

fn process_memory() -> Result<ProcessMemory, String> {
    let status = fs::read_to_string("/proc/self/status")
        .map_err(|error| format!("read /proc/self/status: {error}"))?;
    let rollup = fs::read_to_string("/proc/self/smaps_rollup")
        .map_err(|error| format!("read /proc/self/smaps_rollup: {error}"))?;
    let private_clean = parse_kib_field(&rollup, "Private_Clean:").unwrap_or(0);
    let private_dirty = parse_kib_field(&rollup, "Private_Dirty:").unwrap_or(0);
    Ok(ProcessMemory {
        rss_bytes: parse_kib_field(&rollup, "Rss:").unwrap_or(0),
        pss_bytes: parse_kib_field(&rollup, "Pss:").unwrap_or(0),
        private_bytes: private_clean + private_dirty,
        vm_hwm_bytes: parse_kib_field(&status, "VmHWM:").unwrap_or(0),
    })
}

fn coordinate_origin(index: usize) -> (i64, i64, i64) {
    let x = ((index % 256) * 37) as i64;
    let y = (((index / 256) % 256) * 41) as i64;
    let z = ((index % 17) as i64 - 8) * 3;
    (x, y, z)
}

fn generate_coordinates(workload: Workload, count: usize) -> Vec<i64> {
    let mut coordinates = Vec::with_capacity(count * workload.coordinates_per_case());
    for index in 0..count {
        let (x, y, z) = coordinate_origin(index);
        match workload {
            Workload::LinePoint => {
                coordinates.extend_from_slice(&[x, y, x + 13, y + 5]);
                match index % 3 {
                    0 => coordinates.extend_from_slice(&[x + 26, y + 10]),
                    1 => coordinates.extend_from_slice(&[x + 3, y + 11]),
                    _ => coordinates.extend_from_slice(&[x + 7, y - 9]),
                }
            }
            Workload::TrianglePoint => {
                coordinates.extend_from_slice(&[x, y, z, x + 20, y, z, x, y + 20, z]);
                match index % 3 {
                    0 => coordinates.extend_from_slice(&[x + 4, y + 5, z]),
                    1 => coordinates.extend_from_slice(&[x + 10, y + 10, z]),
                    _ => coordinates.extend_from_slice(&[x + 18, y + 18, z]),
                }
            }
            Workload::TrianglePair => {
                coordinates.extend_from_slice(&[x, y, z, x + 20, y, z, x, y + 20, z]);
                if index % 2 == 0 {
                    coordinates.extend_from_slice(&[
                        x + 4,
                        y + 4,
                        z - 6,
                        x + 4,
                        y + 4,
                        z + 6,
                        x + 10,
                        y + 4,
                        z,
                    ]);
                } else {
                    coordinates.extend_from_slice(&[
                        x + 30,
                        y + 30,
                        z - 6,
                        x + 30,
                        y + 30,
                        z + 6,
                        x + 36,
                        y + 30,
                        z,
                    ]);
                }
            }
        }
    }
    debug_assert_eq!(coordinates.len(), count * workload.coordinates_per_case());
    coordinates
}

fn real(value: i64) -> Real {
    Real::from(value)
}

fn point2(values: &[i64]) -> Point2 {
    Point2::new(real(values[0]), real(values[1]))
}

fn point3(values: &[i64]) -> Point3 {
    Point3::new(real(values[0]), real(values[1]), real(values[2]))
}

struct LinePointCase {
    from: Point2,
    to: Point2,
    query: Point2,
}

struct TrianglePointCase {
    triangle: [Point3; 3],
    query: Point3,
}

struct TrianglePairCase {
    left: [Point3; 3],
    right: [Point3; 3],
}

enum HyperFixture {
    LinePoint(Vec<LinePointCase>),
    TrianglePoint(Vec<TrianglePointCase>),
    TrianglePair(Vec<TrianglePairCase>),
}

fn outcome_code<T>(outcome: &PredicateOutcome<T>) -> u64 {
    match outcome {
        PredicateOutcome::Decided { .. } => 1,
        PredicateOutcome::Unknown { .. } => 2,
    }
}

fn output_capacity(cases: usize, repetitions: u64) -> Result<usize, String> {
    let repetitions = usize::try_from(repetitions)
        .map_err(|_| "repetition count does not fit usize".to_owned())?;
    cases
        .checked_mul(repetitions)
        .ok_or_else(|| "materialized output capacity overflow".to_owned())
}

impl HyperFixture {
    fn new(workload: Workload, coordinates: &[i64], count: usize) -> Self {
        match workload {
            Workload::LinePoint => {
                let mut cases = Vec::with_capacity(count);
                for values in coordinates.as_chunks::<6>().0 {
                    cases.push(LinePointCase {
                        from: point2(values),
                        to: point2(&values[2..]),
                        query: point2(&values[4..]),
                    });
                }
                Self::LinePoint(cases)
            }
            Workload::TrianglePoint => {
                let mut cases = Vec::with_capacity(count);
                for values in coordinates.as_chunks::<12>().0 {
                    cases.push(TrianglePointCase {
                        triangle: [point3(values), point3(&values[3..]), point3(&values[6..])],
                        query: point3(&values[9..]),
                    });
                }
                Self::TrianglePoint(cases)
            }
            Workload::TrianglePair => {
                let mut cases = Vec::with_capacity(count);
                for values in coordinates.as_chunks::<18>().0 {
                    cases.push(TrianglePairCase {
                        left: [point3(values), point3(&values[3..]), point3(&values[6..])],
                        right: [
                            point3(&values[9..]),
                            point3(&values[12..]),
                            point3(&values[15..]),
                        ],
                    });
                }
                Self::TrianglePair(cases)
            }
        }
    }

    fn run(&self, repetitions: u64, mode: OutputMode) -> Result<u64, String> {
        match self {
            Self::LinePoint(cases) => {
                let mut checksum = 0_u64;
                if mode.materializes() {
                    let mut outputs =
                        Vec::with_capacity(output_capacity(cases.len(), repetitions)?);
                    for _ in 0..repetitions {
                        for case in cases {
                            outputs.push(classify_point_line(
                                &case.from,
                                &case.to,
                                &case.query,
                                POLICY,
                            ));
                        }
                    }
                    checksum = outputs.iter().map(outcome_code).sum();
                    black_box(&outputs);
                } else {
                    for _ in 0..repetitions {
                        for case in cases {
                            let output =
                                classify_point_line(&case.from, &case.to, &case.query, POLICY);
                            checksum += outcome_code(&output);
                            black_box(output);
                        }
                    }
                }
                Ok(black_box(checksum))
            }
            Self::TrianglePoint(cases) => {
                let mut checksum = 0_u64;
                if mode.materializes() {
                    let mut outputs =
                        Vec::with_capacity(output_capacity(cases.len(), repetitions)?);
                    for _ in 0..repetitions {
                        for case in cases {
                            outputs.push(classify_point_triangle3(
                                &case.triangle[0],
                                &case.triangle[1],
                                &case.triangle[2],
                                &case.query,
                                POLICY,
                            ));
                        }
                    }
                    checksum = outputs.iter().map(outcome_code).sum();
                    black_box(&outputs);
                } else {
                    for _ in 0..repetitions {
                        for case in cases {
                            let output = classify_point_triangle3(
                                &case.triangle[0],
                                &case.triangle[1],
                                &case.triangle[2],
                                &case.query,
                                POLICY,
                            );
                            checksum += outcome_code(&output);
                            black_box(output);
                        }
                    }
                }
                Ok(black_box(checksum))
            }
            Self::TrianglePair(cases) => {
                let mut checksum = 0_u64;
                if mode.materializes() {
                    let mut outputs =
                        Vec::with_capacity(output_capacity(cases.len(), repetitions)?);
                    for _ in 0..repetitions {
                        for case in cases {
                            outputs.push(classify_triangle_triangle3(
                                &case.left[0],
                                &case.left[1],
                                &case.left[2],
                                &case.right[0],
                                &case.right[1],
                                &case.right[2],
                                POLICY,
                            ));
                        }
                    }
                    checksum = outputs.iter().map(outcome_code).sum();
                    black_box(&outputs);
                } else {
                    for _ in 0..repetitions {
                        for case in cases {
                            let output = classify_triangle_triangle3(
                                &case.left[0],
                                &case.left[1],
                                &case.left[2],
                                &case.right[0],
                                &case.right[1],
                                &case.right[2],
                                POLICY,
                            );
                            checksum += outcome_code(&output);
                            black_box(output);
                        }
                    }
                }
                Ok(black_box(checksum))
            }
        }
    }
}

#[derive(Clone, Debug)]
struct MeasurementRow {
    library: Library,
    workload: String,
    mode: OutputMode,
    cases: usize,
    repetitions: u64,
    fixture: AllocationStats,
    operation: AllocationStats,
    drop_phase: AllocationStats,
    baseline_memory: ProcessMemory,
    fixture_memory: ProcessMemory,
    operation_memory: ProcessMemory,
    final_memory: ProcessMemory,
    checksum: u64,
}

const TSV_HEADER: &str = "library\tworkload\tmode\tcases\trepetitions\twork_items\t\
fixture_allocation_count\tfixture_reallocation_count\tfixture_deallocation_count\tfixture_allocated_bytes\tfixture_reallocated_bytes\tfixture_deallocated_bytes\tfixture_live_bytes\tfixture_peak_live_bytes\t\
operation_allocation_count\toperation_reallocation_count\toperation_deallocation_count\toperation_allocated_bytes\toperation_reallocated_bytes\toperation_deallocated_bytes\toperation_live_bytes\toperation_peak_live_bytes\toperation_peak_growth_bytes\toperation_ephemeral_peak_bytes\toperation_live_delta_bytes\t\
drop_allocation_count\tdrop_reallocation_count\tdrop_deallocation_count\tdrop_allocated_bytes\tdrop_reallocated_bytes\tdrop_deallocated_bytes\tdrop_live_bytes\tdrop_peak_live_bytes\tresidual_live_bytes\t\
baseline_rss_bytes\tfixture_rss_bytes\toperation_rss_bytes\tfinal_rss_bytes\tbaseline_pss_bytes\tfixture_pss_bytes\toperation_pss_bytes\tfinal_pss_bytes\tbaseline_private_bytes\tfixture_private_bytes\toperation_private_bytes\tfinal_private_bytes\tbaseline_vm_hwm_bytes\tfinal_vm_hwm_bytes\tchecksum";

impl MeasurementRow {
    fn tsv(&self) -> String {
        let work_items = self.cases as u64 * self.repetitions;
        let peak_growth = self
            .operation
            .peak_live_bytes
            .saturating_sub(self.fixture.live_bytes);
        let ephemeral_peak = self
            .operation
            .peak_live_bytes
            .saturating_sub(self.fixture.live_bytes.max(self.operation.live_bytes));
        let live_delta =
            i128::from(self.operation.live_bytes) - i128::from(self.fixture.live_bytes);
        let fields = [
            self.library.name().to_owned(),
            self.workload.clone(),
            self.mode.name().to_owned(),
            self.cases.to_string(),
            self.repetitions.to_string(),
            work_items.to_string(),
            self.fixture.allocation_count.to_string(),
            self.fixture.reallocation_count.to_string(),
            self.fixture.deallocation_count.to_string(),
            self.fixture.allocated_bytes.to_string(),
            self.fixture.reallocated_bytes.to_string(),
            self.fixture.deallocated_bytes.to_string(),
            self.fixture.live_bytes.to_string(),
            self.fixture.peak_live_bytes.to_string(),
            self.operation.allocation_count.to_string(),
            self.operation.reallocation_count.to_string(),
            self.operation.deallocation_count.to_string(),
            self.operation.allocated_bytes.to_string(),
            self.operation.reallocated_bytes.to_string(),
            self.operation.deallocated_bytes.to_string(),
            self.operation.live_bytes.to_string(),
            self.operation.peak_live_bytes.to_string(),
            peak_growth.to_string(),
            ephemeral_peak.to_string(),
            live_delta.to_string(),
            self.drop_phase.allocation_count.to_string(),
            self.drop_phase.reallocation_count.to_string(),
            self.drop_phase.deallocation_count.to_string(),
            self.drop_phase.allocated_bytes.to_string(),
            self.drop_phase.reallocated_bytes.to_string(),
            self.drop_phase.deallocated_bytes.to_string(),
            self.drop_phase.live_bytes.to_string(),
            self.drop_phase.peak_live_bytes.to_string(),
            self.drop_phase.live_bytes.to_string(),
            self.baseline_memory.rss_bytes.to_string(),
            self.fixture_memory.rss_bytes.to_string(),
            self.operation_memory.rss_bytes.to_string(),
            self.final_memory.rss_bytes.to_string(),
            self.baseline_memory.pss_bytes.to_string(),
            self.fixture_memory.pss_bytes.to_string(),
            self.operation_memory.pss_bytes.to_string(),
            self.final_memory.pss_bytes.to_string(),
            self.baseline_memory.private_bytes.to_string(),
            self.fixture_memory.private_bytes.to_string(),
            self.operation_memory.private_bytes.to_string(),
            self.final_memory.private_bytes.to_string(),
            self.baseline_memory.vm_hwm_bytes.to_string(),
            self.final_memory.vm_hwm_bytes.to_string(),
            self.checksum.to_string(),
        ];
        fields.join("\t")
    }
}

fn exact_snapshot() -> AllocationStats {
    let mut stats = AllocationStats::default();
    // SAFETY: `stats` is a valid writable C-layout output record.
    unsafe { ec_memory_tracking_snapshot(&mut stats) };
    stats
}

fn measure_exact(
    workload: Workload,
    mode: OutputMode,
    cases: usize,
    repetitions: u64,
    coordinates: &[i64],
) -> Result<MeasurementRow, String> {
    // SAFETY: initialization has no pointer arguments and precedes fixtures.
    if unsafe { ec_memory_tracking_init() } != 0 {
        return Err("could not initialize exactCore memory tracking".to_owned());
    }
    // SAFETY: cache cleanup is valid with no live fixture.
    unsafe { ec_memory_release_caches() };
    let baseline_memory = process_memory()?;

    // SAFETY: phase controls have no pointer arguments.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
    }
    // SAFETY: the packed coordinate slice remains live for construction.
    let native = unsafe { ec_memory_fixture_new(workload.exact_id(), coordinates.as_ptr(), cases) };
    // SAFETY: phase control is independent of fixture validity.
    unsafe { ec_memory_tracking_enable(0) };
    let native =
        NonNull::new(native).ok_or_else(|| "exactCore fixture construction failed".to_owned())?;
    let fixture = exact_snapshot();
    let fixture_memory = process_memory()?;

    // SAFETY: the retained fixture is valid and uniquely owned by this worker.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
    }
    // SAFETY: the fixture remains alive and exactCore calls are serialized.
    let checksum = unsafe {
        ec_memory_fixture_run(
            native.as_ptr(),
            repetitions,
            c_int::from(mode.materializes()),
        )
    };
    // SAFETY: phase control has no pointer arguments.
    unsafe { ec_memory_tracking_enable(0) };
    let operation = exact_snapshot();
    let operation_memory = process_memory()?;

    // SAFETY: this is the fixture's sole owning pointer; cache cleanup follows
    // destruction and both actions remain inside the teardown phase.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
        ec_memory_fixture_free(native.as_ptr());
        ec_memory_release_caches();
        ec_memory_tracking_enable(0);
    }
    let drop_phase = exact_snapshot();
    let final_memory = process_memory()?;

    Ok(MeasurementRow {
        library: Library::ExactCore,
        workload: workload.name().to_owned(),
        mode,
        cases,
        repetitions,
        fixture,
        operation,
        drop_phase,
        baseline_memory,
        fixture_memory,
        operation_memory,
        final_memory,
        checksum,
    })
}

fn measure_hyper(
    workload: Workload,
    mode: OutputMode,
    cases: usize,
    repetitions: u64,
    coordinates: &[i64],
) -> Result<MeasurementRow, String> {
    let baseline_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    let fixture = HyperFixture::new(workload, coordinates, cases);
    rust_tracking_enable(false);
    let fixture_stats = AllocationStats::rust_snapshot();
    let fixture_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    let checksum_result = fixture.run(repetitions, mode);
    rust_tracking_enable(false);
    let checksum = checksum_result?;
    let operation = AllocationStats::rust_snapshot();
    let operation_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    drop(fixture);
    rust_tracking_enable(false);
    let drop_phase = AllocationStats::rust_snapshot();
    let final_memory = process_memory()?;

    Ok(MeasurementRow {
        library: Library::Hyper,
        workload: workload.name().to_owned(),
        mode,
        cases,
        repetitions,
        fixture: fixture_stats,
        operation,
        drop_phase,
        baseline_memory,
        fixture_memory,
        operation_memory,
        final_memory,
        checksum,
    })
}

fn measure_exact_operation(operation_id: &str, repetitions: u64) -> Result<MeasurementRow, String> {
    // SAFETY: initialization has no pointer arguments and precedes fixtures.
    if unsafe { ec_memory_tracking_init() } != 0 {
        return Err("could not initialize exactCore memory tracking".to_owned());
    }
    // SAFETY: cache cleanup is valid with no live fixture.
    unsafe { ec_memory_release_caches() };
    let baseline_memory = process_memory()?;

    // SAFETY: phase controls have no pointer arguments.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
    }
    let prepared = PreparedBenchmark::new(operation_id);
    // SAFETY: phase control is independent of fixture validity.
    unsafe { ec_memory_tracking_enable(0) };
    let mut prepared = prepared.map_err(|error| error.to_string())?;
    let fixture = exact_snapshot();
    let fixture_memory = process_memory()?;

    // SAFETY: phase controls have no pointer arguments.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
    }
    let run_result = prepared.run_iterations(repetitions);
    // SAFETY: phase control is independent of the operation result.
    unsafe { ec_memory_tracking_enable(0) };
    run_result.map_err(|error| error.to_string())?;
    let operation = exact_snapshot();
    let operation_memory = process_memory()?;

    // SAFETY: the fixture is uniquely owned, and cache cleanup follows its
    // destruction while teardown tracking remains enabled.
    unsafe {
        ec_memory_tracking_reset();
        ec_memory_tracking_enable(1);
    }
    drop(prepared);
    // SAFETY: no exactCore fixture remains live.
    unsafe {
        ec_memory_release_caches();
        ec_memory_tracking_enable(0);
    }
    let drop_phase = exact_snapshot();
    let final_memory = process_memory()?;

    Ok(MeasurementRow {
        library: Library::ExactCore,
        workload: operation_id.to_owned(),
        mode: OutputMode::Streaming,
        cases: 1,
        repetitions,
        fixture,
        operation,
        drop_phase,
        baseline_memory,
        fixture_memory,
        operation_memory,
        final_memory,
        checksum: repetitions,
    })
}

fn measure_hyper_operation(operation_id: &str, repetitions: u64) -> Result<MeasurementRow, String> {
    let baseline_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    let prepared = HyperMemoryOperation::new(operation_id);
    rust_tracking_enable(false);
    let mut prepared = prepared?;
    let fixture = AllocationStats::rust_snapshot();
    let fixture_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    prepared.run_iterations(repetitions);
    rust_tracking_enable(false);
    let operation = AllocationStats::rust_snapshot();
    let operation_memory = process_memory()?;

    rust_tracking_reset();
    rust_tracking_enable(true);
    drop(prepared);
    rust_tracking_enable(false);
    let drop_phase = AllocationStats::rust_snapshot();
    let final_memory = process_memory()?;

    Ok(MeasurementRow {
        library: Library::Hyper,
        workload: operation_id.to_owned(),
        mode: OutputMode::Streaming,
        cases: 1,
        repetitions,
        fixture,
        operation,
        drop_phase,
        baseline_memory,
        fixture_memory,
        operation_memory,
        final_memory,
        checksum: repetitions,
    })
}

fn run_worker(arguments: &[String]) -> Result<(), String> {
    if arguments.len() != 5 {
        return Err(
            "worker usage: memory-sweep --worker LIBRARY WORKLOAD MODE CASES REPETITIONS"
                .to_owned(),
        );
    }
    let library = Library::parse(&arguments[0])?;
    let workload = Workload::parse(&arguments[1])?;
    let mode = OutputMode::parse(&arguments[2])?;
    let cases = arguments[3]
        .parse::<usize>()
        .map_err(|error| format!("invalid case count: {error}"))?;
    let repetitions = arguments[4]
        .parse::<u64>()
        .map_err(|error| format!("invalid repetition count: {error}"))?;
    if cases == 0 || repetitions == 0 {
        return Err("case and repetition counts must be positive".to_owned());
    }
    let coordinates = generate_coordinates(workload, cases);
    let row = match library {
        Library::ExactCore => measure_exact(workload, mode, cases, repetitions, &coordinates)?,
        Library::Hyper => measure_hyper(workload, mode, cases, repetitions, &coordinates)?,
    };
    println!("{}", row.tsv());
    Ok(())
}

fn run_operation_worker(arguments: &[String]) -> Result<(), String> {
    if arguments.len() != 3 {
        return Err(
            "worker usage: memory-sweep --operation-worker LIBRARY OPERATION REPETITIONS"
                .to_owned(),
        );
    }
    let library = Library::parse(&arguments[0])?;
    let operation_id = &arguments[1];
    if !COMPARABLE_OPERATION_IDS.contains(&operation_id.as_str()) {
        return Err(format!("unknown comparable operation {operation_id}"));
    }
    let repetitions = arguments[2]
        .parse::<u64>()
        .map_err(|error| format!("invalid repetition count: {error}"))?;
    if repetitions == 0 {
        return Err("repetition count must be positive".to_owned());
    }
    let row = match library {
        Library::ExactCore => measure_exact_operation(operation_id, repetitions)?,
        Library::Hyper => measure_hyper_operation(operation_id, repetitions)?,
    };
    println!("{}", row.tsv());
    Ok(())
}

#[derive(Debug)]
struct SweepOptions {
    workloads: Vec<Workload>,
    operations: Vec<String>,
    modes: Vec<OutputMode>,
    sizes: Option<Vec<usize>>,
    repetitions: u64,
    output: PathBuf,
    quick: bool,
    include_scaling: bool,
    include_operations: bool,
}

const QUICK_OPERATION_IDS: &[&str] = &[
    "rational.multiply",
    "real.sqrt",
    "complex.multiply",
    "vector3.cross",
    "matrix4.inverse",
    "geometry2.line_intersection",
    "geometry3.triangle_relation_6",
    "polynomial.resultant",
    "bivariate.resultant",
    "triangulation.delaunay_complex",
    "curve.segment_dispatch",
    "mesh.triangle_triangle_intersection",
    "path.circle_segment_intersection",
];

fn comma_values<T>(
    text: &str,
    mut parse: impl FnMut(&str) -> Result<T, String>,
) -> Result<Vec<T>, String> {
    text.split(',').map(|value| parse(value.trim())).collect()
}

fn parse_options(arguments: &[String]) -> Result<SweepOptions, String> {
    let mut options = SweepOptions {
        workloads: Workload::ALL.to_vec(),
        operations: COMPARABLE_OPERATION_IDS
            .iter()
            .map(|id| (*id).to_owned())
            .collect(),
        modes: OutputMode::ALL.to_vec(),
        sizes: None,
        repetitions: 2,
        output: PathBuf::from(DEFAULT_OUTPUT),
        quick: false,
        include_scaling: true,
        include_operations: true,
    };
    let mut operations_explicit = false;
    let mut index = 0;
    while index < arguments.len() {
        match arguments[index].as_str() {
            "--quick" => options.quick = true,
            "--workloads" => {
                index += 1;
                let value = arguments.get(index).ok_or("--workloads needs a value")?;
                options.workloads = comma_values(value, Workload::parse)?;
            }
            "--operations" => {
                index += 1;
                let value = arguments.get(index).ok_or("--operations needs a value")?;
                options.operations = if value == "all" {
                    COMPARABLE_OPERATION_IDS
                        .iter()
                        .map(|id| (*id).to_owned())
                        .collect()
                } else {
                    comma_values(value, |entry| {
                        COMPARABLE_OPERATION_IDS
                            .contains(&entry)
                            .then(|| entry.to_owned())
                            .ok_or_else(|| format!("unknown comparable operation {entry}"))
                    })?
                };
                operations_explicit = true;
            }
            "--modes" => {
                index += 1;
                let value = arguments.get(index).ok_or("--modes needs a value")?;
                options.modes = comma_values(value, OutputMode::parse)?;
            }
            "--sizes" => {
                index += 1;
                let value = arguments.get(index).ok_or("--sizes needs a value")?;
                let sizes = comma_values(value, |entry| {
                    entry
                        .parse::<usize>()
                        .map_err(|error| format!("invalid size {entry}: {error}"))
                })?;
                if sizes.contains(&0) {
                    return Err("sizes must be positive".to_owned());
                }
                options.sizes = Some(sizes);
            }
            "--repetitions" => {
                index += 1;
                let value = arguments.get(index).ok_or("--repetitions needs a value")?;
                options.repetitions = value
                    .parse::<u64>()
                    .map_err(|error| format!("invalid repetitions: {error}"))?;
                if options.repetitions == 0 {
                    return Err("repetitions must be positive".to_owned());
                }
            }
            "--output" => {
                index += 1;
                options.output =
                    PathBuf::from(arguments.get(index).ok_or("--output needs a path")?);
            }
            "--scaling-only" => options.include_operations = false,
            "--operations-only" => options.include_scaling = false,
            "--list-operations" => {
                for id in COMPARABLE_OPERATION_IDS {
                    println!("{id}");
                }
                std::process::exit(0);
            }
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other}")),
        }
        index += 1;
    }
    if !options.include_scaling && !options.include_operations {
        return Err("--scaling-only and --operations-only cannot be combined".to_owned());
    }
    if options.include_scaling && (options.workloads.is_empty() || options.modes.is_empty()) {
        return Err("at least one scaling workload and mode are required".to_owned());
    }
    if options.include_operations && options.operations.is_empty() {
        return Err("at least one comparable operation is required".to_owned());
    }
    if options.quick && !operations_explicit {
        options.operations = QUICK_OPERATION_IDS
            .iter()
            .map(|id| (*id).to_owned())
            .collect();
    }
    Ok(options)
}

fn print_help() {
    println!(
        "memory-sweep [OPTIONS]\n\n\
         Options:\n  \
         --quick                      Small scaling grid plus 13-operation smoke set\n  \
         --workloads LIST             line-point,triangle-point,triangle-pair\n  \
         --operations LIST            Comparable IDs or all (default: all 160)\n  \
         --modes LIST                 streaming,materialized\n  \
         --sizes LIST                 Override workload-specific size grids\n  \
         --repetitions N              Traversals per fixture (default: 2)\n  \
         --scaling-only               Skip the fixed comparable-operation sweep\n  \
         --operations-only            Skip the three scaling workloads\n  \
         --list-operations            Print all comparable operation IDs\n  \
         --output PATH                TSV destination (default: {DEFAULT_OUTPUT})"
    );
}

fn sizes_for(workload: Workload, options: &SweepOptions) -> Vec<usize> {
    if let Some(sizes) = &options.sizes {
        return sizes.clone();
    }
    if options.quick {
        return match workload {
            Workload::LinePoint => vec![8, 128],
            Workload::TrianglePoint => vec![4, 64],
            Workload::TrianglePair => vec![1, 16],
        };
    }
    workload.default_sizes().to_vec()
}

fn child_row(
    executable: &Path,
    library: Library,
    workload: Workload,
    mode: OutputMode,
    cases: usize,
    repetitions: u64,
) -> Result<String, String> {
    eprintln!(
        "measuring {} / {} / {} / n={cases}",
        library.name(),
        workload.name(),
        mode.name()
    );
    let output = Command::new(executable)
        .arg("--worker")
        .arg(library.name())
        .arg(workload.name())
        .arg(mode.name())
        .arg(cases.to_string())
        .arg(repetitions.to_string())
        .output()
        .map_err(|error| format!("start worker: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "worker failed ({})\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    let stdout = String::from_utf8(output.stdout)
        .map_err(|error| format!("worker output was not UTF-8: {error}"))?;
    let row = stdout.trim();
    if row.lines().count() != 1 || row.split('\t').count() != TSV_HEADER.split('\t').count() {
        return Err(format!("malformed worker row: {row:?}"));
    }
    Ok(row.to_owned())
}

fn child_operation_row(
    executable: &Path,
    library: Library,
    operation_id: &str,
    repetitions: u64,
) -> Result<String, String> {
    eprintln!("measuring {} / {operation_id}", library.name());
    let output = Command::new(executable)
        .arg("--operation-worker")
        .arg(library.name())
        .arg(operation_id)
        .arg(repetitions.to_string())
        .output()
        .map_err(|error| format!("start worker: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "worker failed ({})\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    let stdout = String::from_utf8(output.stdout)
        .map_err(|error| format!("worker output was not UTF-8: {error}"))?;
    let row = stdout.trim();
    if row.lines().count() != 1 || row.split('\t').count() != TSV_HEADER.split('\t').count() {
        return Err(format!("malformed worker row: {row:?}"));
    }
    Ok(row.to_owned())
}

fn run_parent(arguments: &[String]) -> Result<(), String> {
    let options = parse_options(arguments)?;
    let executable =
        env::current_exe().map_err(|error| format!("find current executable: {error}"))?;
    let mut report = String::new();
    writeln!(&mut report, "{TSV_HEADER}").expect("write to String");
    if options.include_scaling {
        for workload in &options.workloads {
            for cases in sizes_for(*workload, &options) {
                for mode in &options.modes {
                    for library in [Library::ExactCore, Library::Hyper] {
                        let row = child_row(
                            &executable,
                            library,
                            *workload,
                            *mode,
                            cases,
                            options.repetitions,
                        )?;
                        writeln!(&mut report, "{row}").expect("write to String");
                    }
                }
            }
        }
    }
    if options.include_operations {
        for operation_id in &options.operations {
            for library in [Library::ExactCore, Library::Hyper] {
                let row =
                    child_operation_row(&executable, library, operation_id, options.repetitions)?;
                writeln!(&mut report, "{row}").expect("write to String");
            }
        }
    }
    if let Some(parent) = options.output.parent() {
        fs::create_dir_all(parent)
            .map_err(|error| format!("create {}: {error}", parent.display()))?;
    }
    fs::write(&options.output, &report)
        .map_err(|error| format!("write {}: {error}", options.output.display()))?;
    print!("{report}");
    eprintln!("wrote {}", options.output.display());
    Ok(())
}

fn real_main() -> Result<(), String> {
    exactcore_hyper_comparison::memory_profile_link_anchor();
    let arguments: Vec<String> = env::args().skip(1).collect();
    if arguments.first().is_some_and(|value| value == "--worker") {
        run_worker(&arguments[1..])
    } else if arguments
        .first()
        .is_some_and(|value| value == "--operation-worker")
    {
        run_operation_worker(&arguments[1..])
    } else {
        run_parent(&arguments)
    }
}

fn main() -> ExitCode {
    match real_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("memory-sweep: {error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_records_have_the_documented_width() {
        for workload in Workload::ALL {
            assert_eq!(
                generate_coordinates(workload, 7).len(),
                7 * workload.coordinates_per_case()
            );
        }
    }

    #[test]
    fn allocation_layout_preserves_payload_alignment() {
        for alignment in [1, 8, 16, 64, 4_096] {
            let payload = Layout::from_size_align(137, alignment).unwrap();
            let (combined, offset) = allocation_layout(payload).unwrap();
            assert_eq!(combined.align().max(alignment), combined.align());
            assert_eq!(offset % alignment, 0);
        }
    }

    #[test]
    fn default_and_quick_options_select_the_documented_operation_sets() {
        let default = parse_options(&[]).unwrap();
        assert!(default.include_scaling);
        assert!(default.include_operations);
        assert_eq!(default.operations.len(), 160);

        let quick = parse_options(&["--quick".to_owned()]).unwrap();
        assert_eq!(quick.operations, QUICK_OPERATION_IDS);

        let operations_only = parse_options(&["--operations-only".to_owned()]).unwrap();
        assert!(!operations_only.include_scaling);
        assert!(operations_only.include_operations);
    }

    #[test]
    fn every_comparable_operation_has_an_executable_exactcore_fixture() {
        // SAFETY: initialization has no pointer arguments and tracking remains
        // disabled throughout this catalog contract test.
        assert_eq!(unsafe { ec_memory_tracking_init() }, 0);
        for id in COMPARABLE_OPERATION_IDS {
            let mut operation = PreparedBenchmark::new(id)
                .unwrap_or_else(|error| panic!("could not construct {id}: {error}"));
            operation
                .run_iterations(1)
                .unwrap_or_else(|error| panic!("could not execute {id}: {error}"));
            drop(operation);
            // SAFETY: the operation fixture has been destroyed.
            unsafe { ec_memory_release_caches() };
        }
    }
}
