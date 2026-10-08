//! Steady-state allocation profile for every finite optimized `Real` class.
//!
//! Fixtures and eight warm-up passes are created before each counting epoch.
//! Rows include result construction/destruction but exclude fixture
//! construction and first-use scalar cache materialization.

use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicIsize, AtomicUsize, Ordering};

use hyperlimit::PredicatePolicy;
use hyperreal::{Rational, Real};
use hypersolve::{
    Constraint, Expr, Problem, SparseResidualTerm, SymbolId, certify_candidate,
    context_from_problem, replay_sparse_linear_residuals, solve_dense_linear_system_bareiss,
};

const DEFAULT_ITERATIONS: usize = 32;
const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const MIN_PRECISION: i32 = -256;

struct CountingAllocator;

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

static ENABLED: AtomicBool = AtomicBool::new(false);
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static DEALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static REALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static ALLOCATED_BYTES: AtomicUsize = AtomicUsize::new(0);
// Signed deltas expose any pre-epoch allocation freed during measurement
// instead of letting a saturating subtraction hide it as a clean zero.
static LIVE_BYTES: AtomicIsize = AtomicIsize::new(0);
static PEAK_LIVE_BYTES: AtomicIsize = AtomicIsize::new(0);

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
    let bytes = bytes as isize;
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
    LIVE_BYTES.fetch_sub(bytes as isize, Ordering::Relaxed);
}

#[derive(Clone, Copy)]
struct AllocationStats {
    allocations: usize,
    deallocations: usize,
    reallocations: usize,
    allocated_bytes: usize,
    peak_live_bytes: isize,
    live_bytes: isize,
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
        ] {
            counter.store(0, Ordering::Relaxed);
        }
        LIVE_BYTES.store(0, Ordering::Relaxed);
        PEAK_LIVE_BYTES.store(0, Ordering::Relaxed);
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
    // Exact-real approximation nodes refine lazily. Several warm-up passes
    // ensure the counting epoch measures a stable cache depth rather than
    // charging retained refinement state as an apparent live-byte leak.
    for _ in 0..8 {
        drop(black_box(operation()));
    }
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
    assert_eq!(
        stats.live_bytes, 0,
        "{name}/{operation} retained or released foreign bytes during the counting epoch"
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

fn representation_problem(value: &Real) -> Problem {
    let mut problem = Problem::default();
    let variable = problem.add_variable("x", value.clone());
    let symbol = SymbolId(variable.0);
    let delta = || Expr::symbol(symbol, "x") - Expr::real(value.clone());
    problem.add_constraint(Constraint::equality("affine", delta()));
    problem.add_constraint(Constraint::equality("quadratic", delta() * delta()));
    problem.add_constraint(Constraint::equality("non-polynomial", delta().sin()));
    problem
}

fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|value| value.parse().expect("iteration count must be an integer"))
        .unwrap_or(DEFAULT_ITERATIONS);
    assert!(iterations > 0, "iteration count must be positive");

    let values = representation_values();
    assert_eq!(values.len(), 22, "update the Real allocation corpus");
    println!("Hypersolve allocation profile ({iterations} iterations per row)\n");
    println!(
        "Type sizes: Real={} Problem={} Expr={} SparseResidualTerm={} bytes\n",
        size_of::<Real>(),
        size_of::<Problem>(),
        size_of::<Expr>(),
        size_of::<SparseResidualTerm>(),
    );
    println!(
        "| Certificate | Operation | allocs/op | deallocs/op | bytes/op | reallocs/op | peak live bytes | end live delta |"
    );
    println!("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |");

    for (name, value) in values {
        let problem = representation_problem(&value);
        let context = context_from_problem(&problem);
        print_row(
            name,
            "analysis + certification",
            iterations,
            measure(iterations, || {
                let analysis = problem.analyze();
                certify_candidate(&analysis, &context)
            }),
        );

        let matrix = vec![
            vec![value.clone(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        let rhs = vec![&value * Real::from(2), Real::from(3)];
        print_row(
            name,
            "dense Bareiss + proof",
            iterations,
            measure(iterations, || {
                solve_dense_linear_system_bareiss(&matrix, &rhs, MIN_PRECISION, APPROX)
                    .expect("bounded exact dense solve")
            }),
        );

        let sparse_terms = [
            SparseResidualTerm {
                row: 0,
                column: 0,
                coefficient: value.clone(),
            },
            SparseResidualTerm {
                row: 1,
                column: 1,
                coefficient: Real::one(),
            },
        ];
        let candidate = [Real::from(2), Real::from(3)];
        print_row(
            name,
            "sparse exact replay",
            iterations,
            measure(iterations, || {
                replay_sparse_linear_residuals(2, 2, &sparse_terms, &rhs, &candidate, MIN_PRECISION)
                    .expect("identity-preserving sparse replay")
            }),
        );
    }
}
