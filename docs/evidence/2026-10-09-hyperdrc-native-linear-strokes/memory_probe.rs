use hypercurve::{BezierLineImageFitRelation, CircularArc2, LineSeg2, Point2, Real, Segment2};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

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
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, bytes: usize) -> *mut u8 {
        let result = unsafe { System.realloc(pointer, layout, bytes) };
        if !result.is_null() {
            LIVE.fetch_sub(layout.size(), Relaxed);
            allocated(bytes);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn measure<T>(name: &str, body: impl FnOnce() -> T, witness: impl FnOnce(&T) -> bool) {
    let initial = LIVE.load(Relaxed);
    PEAK.store(initial, Relaxed);
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    let result = body();
    let calls = CALLS.load(Relaxed);
    let bytes = BYTES.load(Relaxed);
    let retained = LIVE.load(Relaxed) as i128 - initial as i128;
    let peak = PEAK.load(Relaxed) - initial;
    // Verification is deliberately outside the counted operation.
    let valid = witness(&result);
    println!(
        "{{\"name\":\"{name}\",\"iterations\":1000,\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak},\"witness\":{valid}}}"
    );
    assert!(valid);
    black_box(result);
}

fn main() {
    println!(
        "{{\"line_seg2_bytes\":{},\"segment2_bytes\":{}}}",
        std::mem::size_of::<LineSeg2>(),
        std::mem::size_of::<Segment2>(),
    );
    let start = Point2::from_values(0, 0);
    let end = Point2::from_values(3, 4);
    measure(
        "line_storage",
        || {
            (0..1000)
                .map(|_| LineSeg2::try_new(start.clone(), end.clone()).unwrap())
                .collect::<Vec<_>>()
        },
        |lines| {
            lines.len() == 1000
                && lines
                    .iter()
                    .all(|line| line.delta() == (Real::from(3), Real::from(4)))
        },
    );
    for (name, dx, dy) in [
        ("rational_parallel_offset", 3, 4),
        ("surd_parallel_offset", 1, 2),
    ] {
        let line = LineSeg2::try_new(start.clone(), Point2::from_values(dx, dy)).unwrap();
        measure(
            name,
            || {
                (0..1000)
                    .map(|_| line.offset_left(Real::one()).unwrap())
                    .collect::<Vec<_>>()
            },
            |lines| {
                lines.len() == 1000
                    && lines.iter().all(|line| {
                        let (actual_dx, actual_dy) = line.delta();
                        hyperlimit::compare_reals(
                            &actual_dx,
                            &Real::from(dx),
                            hyperlimit::PredicatePolicy::STRICT,
                        )
                        .value()
                            == Some(std::cmp::Ordering::Equal)
                            && hyperlimit::compare_reals(
                                &actual_dy,
                                &Real::from(dy),
                                hyperlimit::PredicatePolicy::STRICT,
                            )
                            .value()
                                == Some(std::cmp::Ordering::Equal)
                    })
            },
        );
    }
    let center = Point2::new(Real::pi(), Real::e());
    let radius = Real::from(2).sqrt().unwrap();
    let arc = CircularArc2::try_from_center(
        Point2::new(center.x() + &radius, center.y().clone()),
        Point2::new(center.x().clone(), center.y() + &radius),
        center,
        false,
    )
    .unwrap();
    let circle = arc.rational_bezier_decomposition().unwrap().spans()[0].curve();
    measure(
        "retained_circle_line_fit",
        || {
            (0..1000)
                .filter(|_| {
                    matches!(
                        black_box(circle).fit_exact_line_image().unwrap(),
                        BezierLineImageFitRelation::NotLine
                    )
                })
                .count()
        },
        |count| *count == 1000,
    );
}
