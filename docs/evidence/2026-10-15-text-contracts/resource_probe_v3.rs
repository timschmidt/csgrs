use csgrs::{Real, curve, errors::ValidationError};
use hypercurve::{CurveGeometry2, CurveRegion2, CurveString2, Point2};
use std::{hint::black_box, time::Instant};
#[cfg(allocation)]
mod memory {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::hint::black_box;
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    struct CountingAllocator;
    static ALLOCS: AtomicUsize = AtomicUsize::new(0);
    static REALLOCS: AtomicUsize = AtomicUsize::new(0);
    static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
    static REALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);

    fn added(bytes: usize) {
        let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
        PEAK.fetch_max(live, Relaxed);
    }

    fn allocated(pointer: *mut u8, bytes: usize) -> *mut u8 {
        if !pointer.is_null() {
            ALLOCS.fetch_add(1, Relaxed);
            ALLOC_BYTES.fetch_add(bytes, Relaxed);
            added(bytes);
        }
        pointer
    }

    unsafe impl GlobalAlloc for CountingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            allocated(unsafe { System.alloc(layout) }, layout.size())
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            allocated(unsafe { System.alloc_zeroed(layout) }, layout.size())
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
            LIVE.fetch_sub(layout.size(), Relaxed);
        }

        unsafe fn realloc(
            &self,
            pointer: *mut u8,
            layout: Layout,
            new_size: usize,
        ) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, new_size) };
            if !result.is_null() {
                REALLOCS.fetch_add(1, Relaxed);
                REALLOC_BYTES.fetch_add(new_size, Relaxed);
                if new_size >= layout.size() {
                    added(new_size - layout.size());
                } else {
                    LIVE.fetch_sub(layout.size() - new_size, Relaxed);
                }
            }
            result
        }
    }

    #[global_allocator]
    static ALLOCATOR: CountingAllocator = CountingAllocator;

    #[derive(Clone, Copy)]
    pub struct Stats {
        pub allocations: usize,
        pub reallocations: usize,
        pub allocated_bytes: usize,
        pub reallocated_bytes: usize,
        pub live: usize,
    }

    pub fn stats() -> Stats {
        Stats {
            allocations: ALLOCS.load(Relaxed),
            reallocations: REALLOCS.load(Relaxed),
            allocated_bytes: ALLOC_BYTES.load(Relaxed),
            reallocated_bytes: REALLOC_BYTES.load(Relaxed),
            live: LIVE.load(Relaxed),
        }
    }

    pub fn calibrate() {
        let layout = Layout::from_size_align(4096, 8).unwrap();
        let before = stats();
        let pointer = unsafe { std::alloc::alloc(layout) };
        assert!(!pointer.is_null());
        black_box(pointer);
        let during = stats();
        unsafe { std::alloc::dealloc(pointer, layout) };
        let after = stats();
        assert_eq!(during.allocations - before.allocations, 1);
        assert_eq!(during.allocated_bytes - before.allocated_bytes, 4096);
        assert_eq!(during.live - before.live, 4096);
        assert_eq!(after.live, before.live);
    }

    pub fn reset_peak(live: usize) {
        PEAK.store(live, Relaxed);
    }
    pub fn peak() -> usize {
        PEAK.load(Relaxed)
    }
}

trait Complete<T> {
    fn complete(self) -> T;
}
impl Complete<CurveRegion2> for CurveRegion2 {
    fn complete(self) -> CurveRegion2 {
        self
    }
}
impl Complete<CurveRegion2> for Result<CurveRegion2, ValidationError> {
    fn complete(self) -> CurveRegion2 {
        self.unwrap()
    }
}
impl Complete<Vec<CurveString2>> for Vec<CurveString2> {
    fn complete(self) -> Vec<CurveString2> {
        self
    }
}
impl Complete<Vec<CurveString2>> for Result<Vec<CurveString2>, ValidationError> {
    fn complete(self) -> Vec<CurveString2> {
        self.unwrap()
    }
}

enum TextGeometry {
    Region(CurveRegion2),
    Strokes(Vec<CurveString2>),
}
fn build(case: &str) -> TextGeometry {
    match case {
        "readme-truetype" => TextGeometry::Region(
            curve::truetype_text("CSG", include_bytes!("../../../asar.ttf"), Real::from(72))
                .complete(),
        ),
        "bench-truetype" => TextGeometry::Region(
            curve::truetype_text(
                "csgrs benchmark",
                include_bytes!("../../../asar.ttf"),
                Real::from(24),
            )
            .complete(),
        ),
        "readme-hershey" => TextGeometry::Strokes(
            curve::hershey_strings("CSG", &hypercurve::hershey::fonts::ROWMANS, Real::one())
                .complete(),
        ),
        "bench-hershey" => TextGeometry::Strokes(
            curve::hershey_strings(
                "CSGRS",
                &hypercurve::hershey::fonts::FUTURAL,
                Real::from(2),
            )
            .complete(),
        ),
        "custom-hershey" => TextGeometry::Strokes(
            curve::hershey_strings(
                "AA",
                &hypercurve::hershey::Font::new(&["MWRMNV RRMVV"], 'A'),
                Real::one(),
            )
            .complete(),
        ),
        "negative-hershey" => TextGeometry::Strokes(
            curve::hershey_strings(
                "AA",
                &hypercurve::hershey::Font::new(&["MWRMNV RRMVV"], 'A'),
                -Real::one(),
            )
            .complete(),
        ),
        _ => panic!("unknown fixture"),
    }
}
fn point(point: &Point2) -> String {
    format!("[{},{}]", point.x().to_json(), point.y().to_json())
}
fn record(value: &TextGeometry) -> (usize, usize, String) {
    match value {
        TextGeometry::Strokes(strings) => {
            assert!(!strings.is_empty());
            let rows = strings
                .iter()
                .map(|string| {
                    let edges = string
                        .segments()
                        .iter()
                        .map(|edge| format!("[{},{}]", point(edge.start()), point(edge.end())))
                        .collect::<Vec<_>>();
                    format!("[{}]", edges.join(","))
                })
                .collect::<Vec<_>>();
            (
                strings.len(),
                strings.iter().map(|s| s.segments().len()).sum(),
                format!("[{}]\n", rows.join(",\n")),
            )
        },
        TextGeometry::Region(region) => {
            assert!(!region.is_empty());
            let paths = region.boundary_paths().unwrap();
            let rows = paths
                .iter()
                .map(|path| {
                    let curves = path
                        .curves()
                        .iter()
                        .map(|curve| {
                            let controls = match curve
                                .geometry()
                                .expect("original fixture has native geometry")
                            {
                                CurveGeometry2::Line(line) => vec![line.start(), line.end()],
                                CurveGeometry2::QuadraticBezier(bezier) => {
                                    bezier.control_points().to_vec()
                                },
                                CurveGeometry2::CubicBezier(bezier) => {
                                    bezier.control_points().to_vec()
                                },
                                other => panic!("unexpected font curve: {other:?}"),
                            };
                            format!(
                                "[{}]",
                                controls.into_iter().map(point).collect::<Vec<_>>().join(",")
                            )
                        })
                        .collect::<Vec<_>>();
                    format!("[{}]", curves.join(","))
                })
                .collect::<Vec<_>>();
            (
                region.len(),
                paths.iter().map(|p| p.curves().len()).sum(),
                format!("[{}]\n", rows.join(",\n")),
            )
        },
    }
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let (case, temperature) = (&args[1], &args[2]);
    assert!(temperature == "cold" || temperature == "warm");
    #[cfg(allocation)]
    memory::calibrate();
    if temperature == "warm" {
        drop(black_box(build(case)));
    }
    #[cfg(allocation)]
    let before = memory::stats();
    #[cfg(allocation)]
    memory::reset_peak(before.live);
    let started = Instant::now();
    let receipt = hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || {
        black_box(build(case))
    });
    let nanos = started.elapsed().as_nanos();
    assert!(
        receipt.is_certified(),
        "strict construction must not consume approximate facts"
    );
    let value = receipt.into_unverified();
    #[cfg(allocation)]
    let after = memory::stats();
    #[cfg(allocation)]
    let peak = memory::peak();
    // Capture post-drop retention before output inspection allocates anything.
    drop(value);
    #[cfg(allocation)]
    let retained = memory::stats().live as i128 - before.live as i128;
    #[cfg(allocation)]
    let metrics = format!(
        "{},{},{},{},{},{},{}",
        after.allocations - before.allocations,
        after.reallocations - before.reallocations,
        after.allocated_bytes - before.allocated_bytes,
        after.reallocated_bytes - before.reallocated_bytes,
        after.live as i128 - before.live as i128,
        peak - before.live,
        retained
    );
    #[cfg(not(allocation))]
    let metrics = "0,0,0,0,0,0,0";
    // A separate deterministic replay supplies serialization after all cost
    // snapshots; its work and allocations are outside this construction sample.
    let replay =
        hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || build(case));
    assert!(replay.is_certified());
    let (carriers, edges, serialized) = record(&replay.into_unverified());
    let carrier_kind = if case.ends_with("truetype") {
        "boundary_loops"
    } else {
        "strings"
    };
    let bytes = serialized.len();
    std::fs::write(&args[3], serialized).unwrap();
    println!(
        "case,temperature,nanos,carrier_kind,carriers,edges,serialized_bytes,certainty,allocations,reallocations,allocated_bytes,reallocated_bytes,live_delta_bytes,peak_additional_requested_bytes,retained_delta_bytes"
    );
    println!(
        "{case},{temperature},{nanos},{carrier_kind},{carriers},{edges},{bytes},certified_local,{metrics}"
    );
}
