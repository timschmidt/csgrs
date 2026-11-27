//! Resource atlases measure heap, stack, shape and counters per call.
use perf_atlas::{Atlas, Config, CountingAllocator, Shape, Shaped};
use std::sync::atomic::{AtomicU64, Ordering};

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

static EVENTS: AtomicU64 = AtomicU64::new(0);
// Heap counters are process-wide, so the tests must not measure concurrently.
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());
fn events() -> u64 {
    EVENTS.load(Ordering::Relaxed)
}

// Recursion with a 1 KiB frame buffer, so stack use grows with depth.
#[inline(never)]
fn recurse(depth: u64) -> u64 {
    let mut buffer = [0u8; 1024];
    buffer[(depth % 1024) as usize] = depth as u8;
    let buffer = std::hint::black_box(buffer);
    if depth == 0 {
        u64::from(buffer[0])
    } else {
        recurse(depth - 1).wrapping_add(u64::from(buffer[(depth % 1024) as usize]))
    }
}

fn resources(json: &str, key: &str) -> Vec<f64> {
    // Values of `"key":N` in order of appearance.
    json.split(&format!("\"{key}\":"))
        .skip(1)
        .map(|rest| {
            let end = rest.find([',', '}']).unwrap();
            rest[..end].parse().unwrap_or(f64::NAN)
        })
        .collect()
}

#[test]
fn resource_points_record_heap_stack_shape_counters_and_failures() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let path =
        std::env::temp_dir().join(format!("perf-atlas-resources-{}.json", std::process::id()));
    let mut atlas = Atlas::new(
        "test",
        Config {
            resources: true,
            stack_reserve: 64 << 20,
            peak_ceiling: 48 << 20,
            ..Config::quick()
        },
        Some(path.to_string_lossy().into_owned()),
    );
    atlas.counter("events", events);
    // Heap: a buffer of 2^m KiB, returned (retained) or dropped inside.
    atlas.operation("g", "buffer", "KiB", "allocate a buffer");
    atlas.sweep(
        "returned",
        &[1.0, 1024.0, 65536.0],
        |m, _| m as usize * 1024,
        |&bytes| std::hint::black_box(vec![1u8; bytes]).len(),
    );
    atlas.sweep(
        "kept",
        &[1024.0],
        |m, _| m as usize * 1024,
        |&bytes| {
            EVENTS.fetch_add(3, Ordering::Relaxed);
            Shaped::new(vec![1u8; bytes], |v| {
                Some(Shape {
                    nodes: v.len() as u64,
                    depth: 1,
                    bits: 8,
                })
            })
        },
    );
    // Stack: recursion depth 16 to 1024 frames of at least 1 KiB.
    atlas.operation("g", "recursion", "frames", "recursive frames");
    atlas.sweep(
        "frames",
        &[16.0, 256.0, 1024.0],
        |m, _| m as u64,
        |&d| recurse(d),
    );
    // A panicking call is an error point, not a crash.
    atlas.operation("g", "panics", "m", "panics");
    atlas.sweep("k", &[1.0], |m, _| m, |_: &f64| -> u64 { panic!("boom") });
    atlas.finish();
    let json = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).ok();
    assert!(json.contains("\"metric\":\"resources\""));
    assert!(json.contains("\"counters\":[\"events\"]"));

    let peaks = resources(&json, "peak");
    let retained = resources(&json, "retained");
    let stacks = resources(&json, "stack");
    // Returned buffers: 1 KiB, 1 MiB and 64 MiB, the last over the 48 MiB
    // ceiling. A returned `len` retains nothing.
    assert!(peaks[0] >= 1024.0 && peaks[0] < 64.0 * 1024.0, "{peaks:?}");
    assert!(peaks[1] >= 1048576.0, "{peaks:?}");
    assert!(json.contains("\"memory-ceiling\""));
    assert_eq!(
        retained[0],
        0.0,
        "{retained:?} {peaks:?} {:?}",
        resources(&json, "allocations")
    );
    // The kept buffer is still held when measured, with its shape and counter.
    assert!(retained[3] >= 1048576.0, "{retained:?}");
    assert!(json.contains("\"shape\":{\"nodes\":1048576,\"depth\":1,\"bits\":8}"));
    assert!(json.contains("\"counters\":[3]"));
    // Stack grows with recursion: 1024 frames of >= 1 KiB use >= 1 MiB.
    let (s16, s256, s1024) = (stacks[4], stacks[5], stacks[6]);
    assert!(s16 < s256 && s256 < s1024, "{stacks:?}");
    assert!(s1024 >= 1048576.0, "{stacks:?}");
    assert!(json.contains("\"panic: boom\""));
}

#[test]
fn residue_counts_bytes_left_after_the_result_is_dropped() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    use std::cell::RefCell;
    thread_local! {
        static CACHE: RefCell<Vec<Vec<u8>>> = const { RefCell::new(Vec::new()) };
    }
    let path = std::env::temp_dir().join(format!("perf-atlas-residue-{}.json", std::process::id()));
    let mut atlas = Atlas::new(
        "test",
        Config {
            resources: true,
            stack_reserve: 16 << 20,
            ..Config::quick()
        },
        Some(path.to_string_lossy().into_owned()),
    );
    atlas.operation("g", "cache", "KiB", "grows a cache each call");
    atlas.sweep(
        "k",
        &[64.0],
        |m, _| m as usize * 1024,
        |&bytes| CACHE.with(|c| c.borrow_mut().push(vec![0u8; bytes])),
    );
    atlas.finish();
    let json = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).ok();
    let residue = resources(&json, "residue");
    assert!(residue[0] >= 65536.0, "{residue:?}");
}
