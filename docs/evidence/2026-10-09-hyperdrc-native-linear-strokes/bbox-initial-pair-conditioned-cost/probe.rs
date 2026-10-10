//! Public exact-normal-form workloads, with optional Rust allocator accounting.
use hypercurve::{Aabb2, Classification, Point2, Real};
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

struct Fixture {
    points: Vec<Point2>,
    expected: [Real; 4],
    codes: [i32; 4],
}

fn fixture(kind: usize, size: usize, index: usize) -> Fixture {
    let (origin, step) = match kind {
        0 => (Real::from(17), Real::one()),
        1 => (Real::pi() + Real::from(17), Real::one()),
        2 => (Real::pi() + Real::from(53).sqrt().unwrap(), Real::from(5).sqrt().unwrap()),
        3 => ("1e400".parse().unwrap(), "1e-400".parse().unwrap()),
        4 => (Real::pi() * "1e-400".parse::<Real>().unwrap(), Real::from(5).sqrt().unwrap() * "1e-400".parse::<Real>().unwrap()),
        5 => (Real::from(7).sin() + Real::from(53).sqrt().unwrap(), Real::one()),
        _ => unreachable!(),
    };
    let pattern = [3, -2, 0, 1, -1, 2, -3, 4];
    let coordinates = (0..size).map(|i| (pattern[(i + index) % 8], pattern[(3 * i + index + 1) % 8])).collect::<Vec<_>>();
    let map_x = |v| &origin + &step * Real::from(v);
    let map_y = |v| -&origin + &step * Real::from(v);
    let lo_x = coordinates.iter().map(|p| p.0).min().unwrap();
    let hi_x = coordinates.iter().map(|p| p.0).max().unwrap();
    let lo_y = coordinates.iter().map(|p| p.1).min().unwrap();
    let hi_y = coordinates.iter().map(|p| p.1).max().unwrap();
    Fixture {
        points: coordinates.into_iter().map(|(x, y)| Point2::new(map_x(x), map_y(y))).collect(),
        expected: [map_x(lo_x), map_x(hi_x), map_y(lo_y), map_y(hi_y)],
        codes: [lo_x, hi_x, lo_y, hi_y],
    }
}

fn verify(name: &str, fixtures: &[Fixture], answers: &[Classification<Aabb2>]) {
    assert_eq!(fixtures.len(), answers.len());
    let mut hash = 0xcbf29ce484222325_u64;
    for (fixture, answer) in fixtures.iter().zip(answers) {
        let Classification::Decided(bounds) = answer else { panic!("expected exact bounds: {name}") };
        for (actual, expected) in [bounds.min_x(), bounds.max_x(), bounds.min_y(), bounds.max_y()].into_iter().zip(&fixture.expected) {
            assert_eq!((actual - expected).exact_rational_normal_form(), Real::zero().exact_rational(), "{name}");
        }
        for value in fixture.codes {
            for byte in value.to_le_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
            }
        }
    }
    println!("{{\"witness\":\"{name}\",\"decided\":{},\"integer_extrema_fnv1a64\":\"{hash:016x}\"}}", fixtures.len());
}

fn main() {
    // Fixed unmeasured CPU activity before constructing any geometry. This
    // changes processor startup conditions, without priming point allocations.
    let conditioner_end = Instant::now() + std::time::Duration::from_millis(100);
    let mut conditioner_state = 1_u64;
    while Instant::now() < conditioner_end {
        conditioner_state = black_box(conditioner_state).wrapping_mul(6364136223846793005).wrapping_add(1);
    }
    black_box(conditioner_state);
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    let selected_family: usize = std::env::args().nth(2).unwrap().parse().unwrap();
    let selected_size: usize = std::env::args().nth(3).unwrap().parse().unwrap();
    for (kind, family) in ["rational", "pi_translation", "quadratic_translation", "huge_plus_tiny", "tiny_quadratic", "opaque_translation"].into_iter().enumerate() {
        if kind != selected_family { continue; }
        for size in [1, 2, 3, 8] {
            if size != selected_size { continue; }
            let name = format!("{family}_{size}_points");
            let fixtures = measure(&format!("{name}_construction"), count, || (0..count).map(|index| fixture(kind, size, index)).collect::<Vec<_>>());
            for phase in ["first", "warm"] {
                let query = format!("{name}_{phase}");
                let answers = measure(&query, count, || fixtures.iter().map(|value| Aabb2::from_points(black_box(&value.points))).collect::<Vec<_>>());
                verify(&query, &fixtures, &answers);
            }
            let cloned = fixtures.iter().map(|value| value.points.clone()).collect::<Vec<_>>();
            let query = format!("{name}_cloned");
            let answers = measure(&query, count, || cloned.iter().map(|value| Aabb2::from_points(black_box(value))).collect::<Vec<_>>());
            verify(&query, &fixtures, &answers);
        }
    }
}
