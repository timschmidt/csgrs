use hyperlattice::{Matrix3, Matrix4, Problem, Real};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicIsize, AtomicUsize, Ordering::Relaxed};

struct Counting;
static ENABLED: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicIsize = AtomicIsize::new(0);
static PEAK: AtomicIsize = AtomicIsize::new(0);

fn allocated(size: usize) {
    let current = LIVE.fetch_add(size as isize, Relaxed) + size as isize;
    if ENABLED.load(Relaxed) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(size, Relaxed);
        PEAK.fetch_max(current, Relaxed);
    }
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() { allocated(layout.size()); }
        ptr
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc_zeroed(layout) };
        if !ptr.is_null() { allocated(layout.size()); }
        ptr
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size() as isize, Relaxed);
        unsafe { System.dealloc(ptr, layout) };
    }
    unsafe fn realloc(&self, ptr: *mut u8, old: Layout, size: usize) -> *mut u8 {
        let next = unsafe { System.realloc(ptr, old, size) };
        if !next.is_null() {
            LIVE.fetch_sub(old.size() as isize, Relaxed);
            allocated(size);
        }
        next
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

fn measure(name: &str, iterations: usize, mut operation: impl FnMut() -> u64) {
    // Warm immutable input certificates equally before counting steady-state work.
    black_box(operation());
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    let initial = LIVE.load(Relaxed);
    PEAK.store(initial, Relaxed);
    ENABLED.store(true, Relaxed);
    let mut checksum = 0_u64;
    for _ in 0..iterations { checksum = checksum.wrapping_add(black_box(operation())); }
    ENABLED.store(false, Relaxed);
    let peak = PEAK.load(Relaxed) - initial;
    let retained = LIVE.load(Relaxed) - initial;
    println!("{name},{iterations},{},{},{peak},{retained},{checksum}",
        CALLS.load(Relaxed), BYTES.load(Relaxed));
}

fn main() {
    let iterations: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    let export = std::env::args().nth(2).is_some_and(|arg| arg == "export");
    #[cfg(baseline)]
    let functions: [fn(Real) -> Result<Real, Problem>; 4] = [
        hyperlattice::ln, hyperlattice::log10, hyperlattice::asin, hyperlattice::acos,
    ];
    #[cfg(not(baseline))]
    let functions: [fn(Real) -> Result<Real, Problem>; 4] = [Real::ln, Real::log10, Real::asin, Real::acos];
    for (index, name) in ["ln", "log10", "asin", "acos"].into_iter().enumerate() {
        let floats = if index < 2 { [9.0, 1.0e-12, 1.0e12, std::f64::consts::E] }
            else { [0.5, -0.999_999, 0.999_999, 1.0e-12] };
        let values = floats.map(|x| Real::try_from(x).unwrap());
        measure(name, iterations, || {
            let mut sum = 0_u64;
            for value in &values {
                let result = black_box(functions[index](black_box(value.clone())).unwrap());
                if export {
                    sum = sum.wrapping_add(result.to_f64_lossy().unwrap().to_bits());
                } else {
                    sum += result.zero_status() as u64 + 1;
                }
            }
            sum
        });
    }
    let signal = Arc::new(AtomicBool::new(false));
    let matrix3 = Matrix3::new([[2.into(), 3.into(), 5.into()], [0.into(), 7.into(), 11.into()], [0.into(), 0.into(), 13.into()]]);
    let matrix4 = Matrix4::new([[2.into(), 3.into(), 5.into(), 7.into()], [0.into(), 11.into(), 13.into(), 17.into()], [0.into(), 0.into(), 19.into(), 23.into()], [0.into(), 0.into(), 0.into(), 29.into()]]);
    measure("matrix3_checked_abort", iterations, || {
        let result = black_box(matrix3.clone()).inverse_checked_with_abort(&signal).unwrap();
        assert_eq!(black_box(&matrix3) * &result, Matrix3::identity());
        result[0][0].to_f64_lossy().unwrap().to_bits()
    });
    measure("matrix4_checked_abort", iterations, || {
        let result = black_box(matrix4.clone()).inverse_checked_with_abort(&signal).unwrap();
        assert_eq!(black_box(&matrix4) * &result, Matrix4::identity());
        result[0][0].to_f64_lossy().unwrap().to_bits()
    });
}
