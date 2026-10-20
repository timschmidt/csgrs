use hyperbrep_io::part21::{ErrorKind, ReadLimits, Resource, parse};
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

fn run(name: &str, fixture: &(String, String), iterations: usize, expected: usize) {
    let (source, canonical) = fixture;
    let limits = ReadLimits {
        nesting_depth: 30_000,
        ..ReadLimits::default()
    };
    println!(
        "fixture name={name} input_bytes={} output_bytes={} entities={expected}",
        source.len(),
        canonical.len()
    );
    measure(name, iterations, false, || {
        let exchange = parse(black_box(source.as_bytes()), &limits).unwrap();
        assert_eq!(exchange.entities().len(), expected);
        assert_eq!(exchange.unresolved_references().count(), 0);
        let output = exchange.to_step(10_000_000).unwrap();
        assert_eq!(&output, canonical);
        black_box(exchange);
        output.len()
    });
}
fn main() {
    // Expected output is authored independently, never obtained from the parser.
    let envelope = |body: &str| {
        let source = format!(
            "ISO-10303-21;HEADER;FILE_DESCRIPTION(('bench'),'2;1');FILE_NAME('','',(),(),'','','');FILE_SCHEMA(('TEST'));ENDSEC;DATA;{body}ENDSEC;END-ISO-10303-21;"
        );
        let canonical = format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('bench'),'2;1');\nFILE_NAME('','',(),(),'','','');\nFILE_SCHEMA(('TEST'));\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n",
            body.trim_end()
        );
        (source, canonical)
    };
    run(
        "part21/small:",
        &envelope("#1=POINT('x''y',(9007199254740993.0,1.E400,1.E-400));"),
        1000,
        1,
    );
    let wide = (1..=10_000)
        .map(|id| format!("#{id}=POINT('',({id}.0,0.,1.E-6),#1);\n"))
        .collect::<String>();
    run("part21/wide:", &envelope(&wide), 20, 10_000);
    let deep = format!("#1=A({}1{});", "(".repeat(20_000), ")".repeat(20_000));
    run("part21/deep:", &envelope(&deep), 20, 1);
    let rejected = "0".repeat(1024 * 1024);
    let limits = ReadLimits {
        input_bytes: 0,
        ..ReadLimits::default()
    };
    measure("part21/reject_bytes:", 100_000, true, || {
        assert!(matches!(
            parse(black_box(rejected.as_bytes()), &limits)
                .unwrap_err()
                .kind,
            ErrorKind::Limit {
                resource: Resource::InputBytes,
                ..
            }
        ));
        1
    });
}
