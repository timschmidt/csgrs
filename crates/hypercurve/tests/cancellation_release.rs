//! Heap retained after a cancelled stalled operation, measured by a counting
//! allocator independently of the operation's own bookkeeping.
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, Ordering};
use std::time::Duration;

use hypercurve::{
    CancellationToken, CurveCornerMode2, CurveFillet2, CurvePath2, CurveRegion2, OffsetCap,
    OffsetCornerStyle2, Point2, PredicatePolicy, RationalQuadraticBezier2, Real, WorkBudget,
    WorkKind, evaluate_with_cancellable_budget,
};

struct CountingAllocator;
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;
static LIVE_BYTES: AtomicIsize = AtomicIsize::new(0);
static PEAK_BYTES: AtomicIsize = AtomicIsize::new(0);
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            let live = LIVE_BYTES.fetch_add(layout.size() as isize, Ordering::Relaxed);
            PEAK_BYTES.fetch_max(live + layout.size() as isize, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        LIVE_BYTES.fetch_sub(layout.size() as isize, Ordering::Relaxed);
        unsafe { System.dealloc(p, layout) };
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let new = unsafe { System.realloc(p, layout, size) };
        if !new.is_null() {
            let delta = size as isize - layout.size() as isize;
            let live = LIVE_BYTES.fetch_add(delta, Ordering::Relaxed);
            PEAK_BYTES.fetch_max(live + delta, Ordering::Relaxed);
        }
        new
    }
}

fn fraction(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

/// A weight-2/3 rational quadratic from `start` to `end`, its middle control
/// at the chord midpoint moved vertically by `outward`.
fn hyperbola(start: Point2, end: Point2, outward: i32) -> hypercurve::Curve2 {
    let middle = Point2::new(
        (start.x() + end.x()) * fraction(1, 2),
        (start.y() + end.y()) * fraction(1, 2) + Real::from(outward),
    );
    RationalQuadraticBezier2::try_new(start, middle, end, Real::one(), fraction(2, 3), Real::one())
        .unwrap()
        .into()
}

/// The stalled bevel stroke of a filleted hyperbola pair from the open-path
/// composition corpus. It does not complete, so only cancellation ends it.
fn filleted_hyperbola_bevel_stroke() -> hypercurve::ExactCurveResult<CurveRegion2> {
    let path = CurvePath2::try_new(vec![
        hyperbola(point(3, 8), point(9, 7), -1),
        hyperbola(point(9, 7), point(15, 7), -2),
    ])
    .unwrap();
    let filleted = path
        .fillet_vertex(
            1,
            &CurveFillet2::new(fraction(1, 2)),
            CurveCornerMode2::TrimOnly,
        )?
        .into_solutions()
        .into_iter()
        .next()
        .expect("the hyperbola corner admits a fillet");
    CurveRegion2::stroke_path(
        &filleted,
        fraction(3, 4),
        &OffsetCornerStyle2::Bevel,
        OffsetCap::Round,
    )
}

/// One cancelled attempt: heap live while it runs, its peak, and the heap it
/// leaves behind, each relative to the heap before it started.
fn cancelled_attempt(running_for: Duration) -> (isize, isize, isize) {
    let baseline = LIVE_BYTES.load(Ordering::Relaxed);
    PEAK_BYTES.store(baseline, Ordering::Relaxed);
    let token = CancellationToken::new();
    let worker = {
        let token = token.clone();
        std::thread::Builder::new()
            .stack_size(256 << 20)
            .spawn(move || {
                evaluate_with_cancellable_budget(
                    PredicatePolicy::STRICT,
                    WorkBudget::UNLIMITED,
                    &token,
                    || filleted_hyperbola_bevel_stroke().is_ok(),
                )
                .result
                .map(|value| value.into_unverified())
            })
            .unwrap()
    };
    std::thread::sleep(running_for);
    let running = LIVE_BYTES.load(Ordering::Relaxed) - baseline;
    token.cancel();
    let result = worker.join().unwrap();
    assert_eq!(result.unwrap_err().kind, WorkKind::Cancellation);
    let peak = PEAK_BYTES.load(Ordering::Relaxed) - baseline;
    let retained = LIVE_BYTES.load(Ordering::Relaxed) - baseline;
    (running, peak, retained)
}

/// The cancelled worker's temporary arithmetic is released when it returns.
/// The first attempts warm bounded process-wide tables (about 1.2 MiB, then a
/// geometrically shrinking remainder); once warm, a cancelled attempt leaves
/// tens to hundreds of bytes of a peak near 4.7 MB. Retention that grew with
/// each cancelled attempt would fail the last two.
#[test]
fn cancelled_stalled_stroke_releases_its_temporary_heap() {
    let attempts = [1; 6].map(|seconds| cancelled_attempt(Duration::from_secs(seconds)));
    eprintln!("(live while running, peak, retained) bytes per attempt: {attempts:?}");
    for (running, peak, _) in attempts {
        assert!(running > 0 && peak >= running);
    }
    for (_, peak, retained) in &attempts[4..] {
        assert!(
            retained * 100 < *peak,
            "a warm cancelled attempt retained {retained} of its {peak} peak bytes"
        );
    }
}
