//! Requested Rust allocator bytes, excluding allocator metadata and native C heaps.
//! Contours have independent caches; their immutable segment/scalar inputs are shared.

use hypercurve::{
    CircularArc2, Contour2, CurveCertainty, LineSeg2, Point2, PredicatePolicy, Real, Segment2,
    evaluate_under,
};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

struct Counted;
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
const COUNT: usize = 1000;

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

fn measure<T>(name: &str, body: impl FnOnce() -> T, witness: impl FnOnce(&T) -> bool) -> T {
    let initial = LIVE.load(Relaxed);
    PEAK.store(initial, Relaxed);
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    let result = body();
    let calls = CALLS.load(Relaxed);
    let bytes = BYTES.load(Relaxed);
    let retained = LIVE.load(Relaxed) as i128 - initial as i128;
    let peak = PEAK.load(Relaxed) - initial;
    // Verification and output are outside the counted operation.
    let valid = witness(&result);
    println!(
        "{{\"name\":\"{name}\",\"iterations\":{COUNT},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak},\"witness\":{valid}}}"
    );
    assert!(valid);
    black_box(result)
}

fn query(contours: &[Contour2], point: &Point2) -> i64 {
    contours
        .iter()
        .map(|contour| i64::from(black_box(contour).winding_number(black_box(point)).unwrap()))
        .sum()
}

fn run() {
    println!(
        "{{\"contour2_bytes\":{},\"line_seg2_bytes\":{},\"segment2_bytes\":{}}}",
        std::mem::size_of::<Contour2>(),
        std::mem::size_of::<LineSeg2>(),
        std::mem::size_of::<Segment2>(),
    );
    for (location, center) in [
        ("origin", Point2::from_values(0, 0)),
        ("translated", Point2::new(Real::pi(), Real::e())),
    ] {
        let shifted = |x, y| center.translated(Real::from(x), Real::from(y));
        let corners = [
            shifted(-2, -2),
            shifted(2, -2),
            shifted(2, 2),
            shifted(-2, 2),
        ];
        let square = (0..4)
            .map(|index| {
                Segment2::Line(
                    LineSeg2::try_new(
                        corners[index].clone(),
                        corners[(index + 1) % 4].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect::<Vec<_>>();
        let cardinals = [shifted(2, 0), shifted(0, 2), shifted(-2, 0), shifted(0, -2)];
        let circle = (0..4)
            .map(|index| {
                Segment2::Arc(
                    CircularArc2::try_from_center(
                        cardinals[index].clone(),
                        cardinals[(index + 1) % 4].clone(),
                        center.clone(),
                        false,
                    )
                    .unwrap(),
                )
            })
            .collect::<Vec<_>>();
        let repeated = (0..8)
            .flat_map(|_| circle.iter().cloned())
            .collect::<Vec<_>>();
        let quarter = (Real::one() / Real::from(4)).unwrap();
        let inside = center.translated(quarter.clone(), quarter.clone());
        let outside = center.translated(Real::from(3), quarter);
        for (geometry, segments, winding) in [
            ("square", square, 1),
            ("circle", circle, 1),
            ("circle_repeated_eight", repeated, 8),
        ] {
            let name = format!("{location}_{geometry}");
            let contours = measure(
                &format!("{name}_construction"),
                || {
                    (0..COUNT)
                        .map(|_| Contour2::try_new(segments.clone()).unwrap())
                        .collect::<Vec<_>>()
                },
                |contours| {
                    contours.len() == COUNT
                        && contours.iter().all(|c| c.segments().len() == segments.len())
                },
            );
            for (phase, point, expected) in [
                ("first_inside", &inside, winding),
                ("warm_inside", &inside, winding),
                ("warm_outside", &outside, 0),
            ] {
                measure(
                    &format!("{name}_{phase}"),
                    || query(&contours, point),
                    |sum| *sum == expected * COUNT as i64,
                );
            }
            let clones = measure(
                &format!("{name}_warm_clone"),
                || contours.clone(),
                |clones| clones.len() == COUNT,
            );
            measure(
                &format!("{name}_cloned_inside"),
                || query(&clones, &inside),
                |sum| *sum == winding * COUNT as i64,
            );
        }
    }
}

fn main() {
    let result = evaluate_under(PredicatePolicy::STRICT, run);
    assert_eq!(result.certainty(), CurveCertainty::Certified);
}
