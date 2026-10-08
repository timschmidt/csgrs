//! Allocation profile for every finite optimized `Real` certificate class.
//!
//! Fixtures and a warm-up operation are created before counting starts. The
//! reported rows therefore measure steady-state carrier operations, including
//! result destruction, rather than fixture construction.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use hyperlattice::{Complex, Matrix3, Matrix4, Point3, Rational, Real, Vector3};

const DEFAULT_ITERATIONS: usize = 128;

struct CountingAllocator;

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

static ENABLED: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static DEALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static REALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && ENABLED.load(Ordering::Relaxed) {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            add_live_bytes(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if ENABLED.load(Ordering::Relaxed) {
            DEALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            subtract_live_bytes(layout.size());
        }
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let replacement = unsafe { System.realloc(pointer, layout, new_size) };
        if !replacement.is_null() && ENABLED.load(Ordering::Relaxed) {
            REALLOCATIONS.fetch_add(1, Ordering::Relaxed);
            ALLOCATED_BYTES.fetch_add(new_size, Ordering::Relaxed);
            if new_size >= layout.size() {
                add_live_bytes(new_size - layout.size());
            } else {
                subtract_live_bytes(layout.size() - new_size);
            }
        }
        replacement
    }
}

fn add_live_bytes(bytes: usize) {
    let live = LIVE_BYTES.fetch_add(bytes, Ordering::Relaxed) + bytes;
    let mut peak = PEAK_LIVE_BYTES.load(Ordering::Relaxed);
    while live > peak {
        match PEAK_LIVE_BYTES.compare_exchange_weak(
            peak,
            live,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(observed) => peak = observed,
        }
    }
}

fn subtract_live_bytes(bytes: usize) {
    let _ = LIVE_BYTES.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |live| {
        Some(live.saturating_sub(bytes))
    });
}

#[derive(Clone, Copy)]
struct AllocationStats {
    allocations: usize,
    deallocations: usize,
    reallocations: usize,
    allocated_bytes: usize,
    peak_live_bytes: usize,
    live_bytes: usize,
}

impl AllocationStats {
    fn snapshot() -> Self {
        Self {
            allocations: ALLOCATIONS.load(Ordering::Relaxed),
            deallocations: DEALLOCATIONS.load(Ordering::Relaxed),
            reallocations: REALLOCATIONS.load(Ordering::Relaxed),
            allocated_bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
            peak_live_bytes: PEAK_LIVE_BYTES.load(Ordering::Relaxed),
            live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
        }
    }
}

struct CountingGuard;

impl CountingGuard {
    fn start() -> Self {
        for counter in [
            &ALLOCATIONS,
            &DEALLOCATIONS,
            &REALLOCATIONS,
            &ALLOCATED_BYTES,
            &LIVE_BYTES,
            &PEAK_LIVE_BYTES,
        ] {
            counter.store(0, Ordering::Relaxed);
        }
        ENABLED.store(true, Ordering::SeqCst);
        Self
    }
}

impl Drop for CountingGuard {
    fn drop(&mut self) {
        ENABLED.store(false, Ordering::SeqCst);
    }
}

fn measure<T>(iterations: usize, mut operation: impl FnMut() -> T) -> AllocationStats {
    drop(black_box(operation()));
    let guard = CountingGuard::start();
    for _ in 0..iterations {
        drop(black_box(operation()));
    }
    let stats = AllocationStats::snapshot();
    drop(guard);
    stats
}

fn print_row(name: &str, operation: &str, iterations: usize, stats: AllocationStats) {
    let divisor = iterations as f64;
    println!(
        "| {name} | {operation} | {:.2} | {:.2} | {:.1} | {:.2} | {} | {} |",
        stats.allocations as f64 / divisor,
        stats.deallocations as f64 / divisor,
        stats.allocated_bytes as f64 / divisor,
        stats.reallocations as f64 / divisor,
        stats.peak_live_bytes,
        stats.live_bytes,
    );
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn representation_values() -> Vec<(&'static str, Real)> {
    let pi = Real::pi();
    let e = Real::e();
    let pi_squared = &pi * &pi;
    let sqrt_two = Real::from(2).sqrt().expect("positive radicand");
    let ln_two = Real::from(2).ln().expect("positive logarithm input");
    let ln_three = Real::from(3).ln().expect("positive logarithm input");

    vec![
        ("One", fraction(3, 2)),
        ("Pi", pi.clone()),
        ("PiPow", pi_squared.clone()),
        ("PiInv", pi.clone().inverse().expect("pi is nonzero")),
        ("PiExp", &pi * &e),
        ("PiInvExp", (&e / &pi).expect("pi is nonzero")),
        ("PiSqrt", &pi * &sqrt_two),
        ("ConstProduct", &pi_squared * &e),
        ("ConstOffset", &pi - Real::from(3)),
        ("ConstProductSqrt", &(&pi_squared * &e) * &sqrt_two),
        ("Sqrt", sqrt_two),
        ("Exp", Real::from(2).exp().expect("finite exponential")),
        ("Ln", ln_three.clone()),
        (
            "LnAffine",
            (Real::from(2) * &e).ln().expect("positive logarithm input"),
        ),
        ("LnProduct", &ln_two * &ln_three),
        ("Log10", Real::from(2).log10().expect("positive input")),
        ("Log2", Real::from(3).log2().expect("positive input")),
        (
            "Pow10",
            fraction(1, 7)
                .exp10()
                .expect("finite rational base-ten power"),
        ),
        (
            "Pow2",
            fraction(1, 7)
                .exp2()
                .expect("finite rational base-two power"),
        ),
        ("SinPi", fraction(1, 5).sin_pi()),
        (
            "TanPi",
            fraction(1, 5)
                .tan_pi()
                .expect("one fifth of a turn is not a tangent pole"),
        ),
        ("Irrational", Real::one().sin()),
    ]
}

fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|value| value.parse().expect("iteration count must be an integer"))
        .unwrap_or(DEFAULT_ITERATIONS);
    assert!(iterations > 0, "iteration count must be positive");

    let values = representation_values();
    assert_eq!(values.len(), 22, "update the Real allocation corpus");
    println!("Hyperlattice allocation profile ({iterations} iterations per row)\n");
    println!(
        "Type sizes: Real={} Complex={} Vector3={} Matrix3={} Matrix4={} bytes\n",
        size_of::<Real>(),
        size_of::<Complex>(),
        size_of::<Vector3>(),
        size_of::<Matrix3>(),
        size_of::<Matrix4>(),
    );
    println!(
        "| Certificate | Operation | allocs/op | deallocs/op | bytes/op | reallocs/op | peak live bytes | end live delta |"
    );
    println!("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |");

    for (name, value) in values {
        let complex = Complex::new(value.clone(), Real::one());
        print_row(
            name,
            "complex multiply",
            iterations,
            measure(iterations, || &complex * &complex),
        );

        let vector = Vector3::new([value.clone(), Real::one(), Real::from(2)]);
        print_row(
            name,
            "vector dot/cross",
            iterations,
            measure(iterations, || (vector.dot(&vector), vector.cross(&vector))),
        );

        let matrix3 = Matrix3::new([
            [value.clone(), Real::one(), Real::zero()],
            [Real::zero(), Real::from(2), Real::one()],
            [Real::one(), Real::zero(), Real::from(3)],
        ]);
        print_row(
            name,
            "matrix3 determinant",
            iterations,
            measure(iterations, || matrix3.determinant()),
        );

        let matrix4 = Matrix4::affine_translation([value.clone(), Real::from(2), Real::from(3)]);
        let point = Point3::new(value, Real::one(), Real::from(2));
        print_row(
            name,
            "matrix4 point transform",
            iterations,
            measure(iterations, || {
                matrix4
                    .transform_point3(&point)
                    .expect("affine transform remains finite")
            }),
        );
    }
}
