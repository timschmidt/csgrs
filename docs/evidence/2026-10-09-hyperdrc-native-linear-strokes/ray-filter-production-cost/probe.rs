//! Public exact-normal-form workloads, with optional Rust allocator accounting.
use hypercurve::{
    Curve2, CurveGeometry2, CurveParameterRange2, CurvePath2, CurvePoint2, CurveRegion2,
    ExactCurveError, FillRule, Point2, PredicatePolicy, QuadraticBezier2, Real,
    RegionPointLocation, evaluate_under,
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


fn lens(x: &Real, y: &Real, scale: &Real, retained: bool) -> Result<CurveRegion2, ExactCurveError> {
    let map = |dx: i32, dy: i32| Point2::new(x + scale * Real::from(dx), y + scale * Real::from(dy));
    let curves = if retained {
        let range = CurveParameterRange2::try_new(Real::one().into(), Real::from(2).into())?;
        vec![
            Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(-3, 4), map(-2, 1), map(-1, 0))), range.clone())?,
            Curve2::try_from_bezier_range(CurveGeometry2::QuadraticBezier(QuadraticBezier2::new(map(3, -4), map(2, -1), map(1, 0))), range)?,
        ]
    } else {
        vec![QuadraticBezier2::new(map(-1, 0), map(0, -1), map(1, 0)).into(), QuadraticBezier2::new(map(1, 0), map(0, 1), map(-1, 0)).into()]
    };
    let path = CurvePath2::try_new(curves)?;
    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)
}

struct Fixture {
    region: Option<CurveRegion2>,
    point: CurvePoint2,
    expected: u8,
    area: Real,
}

fn fixture(kind: usize, retained: bool, index: usize) -> Fixture {
    let (x, y, scale) = [
        ("0", "0", "1"),
        ("0", "0", "1e200"),
        ("1e308", "1e308", "1e307"),
        ("-1e308", "-1e308", "1e307"),
        ("1e308", "-1e308", "1e307"),
        ("0", "0", "1e-400"),
        ("1e308", "-1e308", "1e-400"),
        ("1e400", "-1e400", "1"),
    ][kind];
    let x: Real = x.parse().unwrap();
    let y: Real = y.parse().unwrap();
    let scale: Real = scale.parse().unwrap();
    // Quarter-units at x=0 in the independently defined lens.
    let (dy, expected) = [(0, 2), (2, 1), (3, 0), (1, 2)][index % 4];
    let point = Point2::new(x.clone(), &y + &scale * (Real::from(dy) / Real::from(4)).unwrap()).into();
    let area = (Real::from(4) * &scale * &scale / Real::from(3)).unwrap();
    Fixture { region: lens(&x, &y, &scale, retained).ok(), point, expected, area }
}

fn query(region: Option<&CurveRegion2>, point: &CurvePoint2) -> u8 {
    match region.map(|region| region.classify_point(point)) {
        Some(Ok(RegionPointLocation::Outside)) => 0,
        Some(Ok(RegionPointLocation::Boundary)) => 1,
        Some(Ok(RegionPointLocation::Inside)) => 2,
        Some(Err(_)) | None => 3,
    }
}

fn verify_construction(name: &str, fixtures: &[Fixture]) {
    let mut constructed = 0;
    let mut nonempty = 0;
    let mut loops = 0;
    let mut exact_area_matches = 0;
    for fixture in fixtures {
        if let Some(region) = &fixture.region {
            constructed += 1;
            nonempty += usize::from(!region.is_empty());
            loops += region.boundary_loops().len();
            if let Ok(Some(area)) = region.filled_area() {
                exact_area_matches += usize::from((&area - &fixture.area).exact_rational_normal_form() == Real::zero().exact_rational());
            }
        }
    }
    println!("{{\"witness\":\"{name}\",\"kind\":\"construction\",\"count\":{},\"constructed\":{constructed},\"nonempty\":{nonempty},\"loops\":{loops},\"exact_area_matches\":{exact_area_matches}}}", fixtures.len());
}

fn verify_queries(name: &str, fixtures: &[Fixture], values: &[u8]) {
    let mut counts = [0; 4];
    let mut correct = 0;
    let mut hash = 0xcbf29ce484222325_u64;
    for (fixture, value) in fixtures.iter().zip(values) {
        counts[usize::from(*value)] += 1;
        correct += usize::from(fixture.expected == *value);
        hash = (hash ^ u64::from(*value)).wrapping_mul(0x100000001b3);
    }
    println!("{{\"witness\":\"{name}\",\"kind\":\"query\",\"count\":{},\"counts\":{counts:?},\"correct\":{correct},\"output_fnv1a64\":\"{hash:016x}\"}}", fixtures.len());
}

fn main() {
    let until = Instant::now() + std::time::Duration::from_millis(100);
    let mut state = 1_u64;
    while Instant::now() < until { state = black_box(state).wrapping_mul(6364136223846793005).wrapping_add(1); }
    black_box(state);
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!(count >= 4 && count.is_multiple_of(4));
    evaluate_under(PredicatePolicy::STRICT, || {
        for (kind, family) in ["ordinary", "large_projection", "positive_midpoint", "negative_midpoint", "mixed_midpoint", "subnormal", "huge_plus_tiny", "outside_f64"].into_iter().enumerate() {
            for retained in [false, true] {
                let chart = if retained { "retained" } else { "unit" };
                let name = format!("{family}_{chart}");
                let fixtures = measure(&format!("{name}_construction"), count, || (0..count).map(|index| fixture(kind, retained, index)).collect::<Vec<_>>());
                for phase in ["first", "warm"] {
                    let query_name = format!("{name}_{phase}");
                    let values = measure(&query_name, count, || fixtures.iter().map(|fixture| query(fixture.region.as_ref(), black_box(&fixture.point))).collect::<Vec<_>>());
                    verify_queries(&query_name, &fixtures, &values);
                }
                let clones = fixtures.iter().map(|fixture| fixture.region.clone()).collect::<Vec<_>>();
                let query_name = format!("{name}_cloned");
                let values = measure(&query_name, count, || fixtures.iter().zip(&clones).map(|(fixture, region)| query(region.as_ref(), black_box(&fixture.point))).collect::<Vec<_>>());
                verify_queries(&query_name, &fixtures, &values);
                // Area verification follows every timed query, so it cannot
                // prepare a first-query cache before that measurement.
                verify_construction(&format!("{name}_construction"), &fixtures);
            }
        }
    }).certified().unwrap();
}
