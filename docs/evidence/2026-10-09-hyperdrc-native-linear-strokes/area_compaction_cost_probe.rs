//! Exact loop area: construction, first query, and repeated query costs.
use hypercurve::{
    CircularArc2, Contour2, CurveRegion2, LineSeg2, Point2, PredicatePolicy, Real, Segment2,
};
use std::hint::black_box;
use std::time::Instant;

#[cfg(feature = "count-alloc")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    pub struct Counted;
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
            let result = unsafe { System.alloc(layout) };
            if !result.is_null() {
                allocated(layout.size());
            }
            result
        }
        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            LIVE.fetch_sub(layout.size(), Relaxed);
            unsafe { System.dealloc(pointer, layout) };
        }
        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, size) };
            if !result.is_null() {
                LIVE.fetch_sub(layout.size(), Relaxed);
                allocated(size);
            }
            result
        }
    }
    pub fn start() -> usize {
        let live = LIVE.load(Relaxed);
        PEAK.store(live, Relaxed);
        CALLS.store(0, Relaxed);
        BYTES.store(0, Relaxed);
        live
    }
    pub fn finish(initial: usize) -> (usize, usize, i128, usize) {
        (
            CALLS.load(Relaxed),
            BYTES.load(Relaxed),
            LIVE.load(Relaxed) as i128 - initial as i128,
            PEAK.load(Relaxed) - initial,
        )
    }
}
#[cfg(feature = "count-alloc")]
#[global_allocator]
static ALLOCATOR: allocation::Counted = allocation::Counted;

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    #[cfg(feature = "count-alloc")]
    let initial = allocation::start();
    let start = Instant::now();
    let result = body();
    let nanos = start.elapsed().as_nanos();
    #[cfg(feature = "count-alloc")]
    let (calls, bytes, retained, peak) = allocation::finish(initial);
    #[cfg(not(feature = "count-alloc"))]
    let (calls, bytes, retained, peak) = (0, 0, 0, 0);
    println!(
        "{{\"name\":\"{name}\",\"iterations\":{count},\"nanos\":{nanos},\"allocation_accounting\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak}}}",
        cfg!(feature = "count-alloc")
    );
    black_box(result)
}

fn fixture(kind: usize) -> Contour2 {
    let origin = match kind {
        1 | 3 | 5 | 7 => Real::pi(),
        8 => "1e400".parse().unwrap(),
        _ => Real::zero(),
    };
    let point = |x: Real, y: Real| Point2::new(&origin + x, Real::from(17) + y);
    if kind < 2 {
        let points = [(0, 0), (4, 0), (4, 3), (0, 3)];
        return Contour2::try_new(
            (0..4)
                .map(|i| {
                    let a = points[i];
                    let b = points[(i + 1) % 4];
                    Segment2::Line(
                        LineSeg2::try_new(
                            point(Real::from(a.0), Real::from(a.1)),
                            point(Real::from(b.0), Real::from(b.1)),
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap();
    }
    if kind < 4 {
        let points = [(1, 0), (0, 1), (-1, 0), (0, -1)];
        let center = point(Real::zero(), Real::zero());
        return Contour2::try_new(
            (0..4)
                .map(|i| {
                    let a = points[i];
                    let b = points[(i + 1) % 4];
                    Segment2::Arc(
                        CircularArc2::try_from_center(
                            point(Real::from(a.0), Real::from(a.1)),
                            point(Real::from(b.0), Real::from(b.1)),
                            center.clone(),
                            false,
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap();
    }
    let (dx, dy) = if kind < 6 {
        (3, 4)
    } else if kind < 8 {
        (1, 2)
    } else {
        (2, 3)
    };
    let length = Real::from(dx * dx + dy * dy).sqrt().unwrap();
    let nx = (Real::from(-dy) / &length).unwrap();
    let ny = (Real::from(dx) / &length).unwrap();
    let start = point(Real::zero(), Real::zero());
    let end = point(Real::from(dx), Real::from(dy));
    let ls = Point2::new(start.x() + &nx, start.y() + &ny);
    let rs = Point2::new(start.x() - &nx, start.y() - &ny);
    let le = Point2::new(end.x() + &nx, end.y() + &ny);
    let re = Point2::new(end.x() - &nx, end.y() - &ny);
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(rs.clone(), re.clone()).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(re, le.clone(), end, false).unwrap()),
        Segment2::Line(LineSeg2::try_new(le, ls.clone()).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(ls, rs, start, false).unwrap()),
    ])
    .unwrap()
}

fn verify(values: &[Real], expected: &Real) {
    for value in values {
        assert_eq!(
            hyperlimit::compare_reals(value, expected, PredicatePolicy::STRICT).value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!(count > 0);
    let names = [
        "rectangle-rational",
        "rectangle-pi",
        "circle-rational",
        "circle-pi",
        "capsule-rational-length",
        "capsule-rational-length-pi",
        "capsule-quadratic-length",
        "capsule-quadratic-length-pi",
        "capsule-huge-translation",
    ];
    for (kind, name) in names.into_iter().enumerate() {
        let expected = match kind {
            0 | 1 => Real::from(12),
            2 | 3 => Real::pi(),
            4 | 5 => Real::from(10) + Real::pi(),
            6 | 7 => Real::from(2) * Real::from(5).sqrt().unwrap() + Real::pi(),
            _ => Real::from(2) * Real::from(13).sqrt().unwrap() + Real::pi(),
        };
        let contours = measure(&format!("{name}-contour-construction"), count, || {
            (0..count).map(|_| fixture(kind)).collect::<Vec<_>>()
        });
        let first = measure(&format!("{name}-contour-first"), count, || {
            contours
                .iter()
                .map(|c| black_box(c).signed_area().unwrap().unwrap())
                .collect::<Vec<_>>()
        });
        verify(&first, &expected);
        let warm = measure(&format!("{name}-contour-warm"), count, || {
            (0..count)
                .map(|_| black_box(&contours[0]).signed_area().unwrap().unwrap())
                .collect::<Vec<_>>()
        });
        verify(&warm, &expected);
        let regions = measure(&format!("{name}-region-construction"), count, || {
            contours
                .into_iter()
                .map(|c| CurveRegion2::try_from_native_contours(vec![c], vec![]).unwrap())
                .collect::<Vec<_>>()
        });
        let first = measure(&format!("{name}-region-first"), count, || {
            regions
                .iter()
                .map(|r| black_box(r).filled_area().unwrap().unwrap())
                .collect::<Vec<_>>()
        });
        verify(&first, &expected);
        let warm = measure(&format!("{name}-region-warm"), count, || {
            (0..count)
                .map(|_| black_box(&regions[0]).filled_area().unwrap().unwrap())
                .collect::<Vec<_>>()
        });
        verify(&warm, &expected);
        println!("{{\"witness\":\"{name}\",\"count\":{count},\"all_areas_exact\":true}}");
    }
}
