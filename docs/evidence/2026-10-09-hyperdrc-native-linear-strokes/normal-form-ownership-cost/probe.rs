//! Public exact-normal-form workloads, with optional Rust allocator accounting.
use hyperreal::{Rational, Real};
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

fn fixture(kind: usize, index: usize) -> Real {
    let root2 = Real::from(2).sqrt().unwrap();
    let root3 = Real::from(3).sqrt().unwrap();
    let pi = Real::pi();
    let k = Real::from((index % 7 + 1) as i32);
    let zero = match kind {
        0 => {
            // (sqrt(2) + k sqrt(3))^2 = 2 + 3k^2 + 2k sqrt(6).
            let sum = &root2 + &k * &root3;
            &sum * &sum
                - Real::from(2)
                - Real::from(3) * &k * &k
                - Real::from(2) * &k * Real::from(6).sqrt().unwrap()
        },
        1 => {
            // 1/(1 + sqrt(2)) = sqrt(2) - 1.
            ((Real::one() + &root2).inverse().unwrap() + Real::one() - &root2) * k
        },
        2 => {
            // -pi * ((pi-1)-pi/3) * pi / pi^2 + (pi-1) = pi/3.
            let third = (&pi / Real::from(3)).unwrap();
            let shifted = &pi - Real::one();
            let transported = (&pi * (&shifted - &third) * &pi / (&pi * &pi)).unwrap();
            (-transported + shifted - third) * k
        },
        3 => {
            // Laurent multiplication and reciprocal scaling preserve r+s.
            let sum = &root2 + &root3;
            let transported = ((&sum * &pi) / (&k * &pi)).unwrap() * &k;
            transported - sum
        },
        4 => {
            // Linear opaque atan atoms cancel through the same pi chart.
            let angle = Real::new(Rational::fraction(3, 4).unwrap()).atan().unwrap();
            let shifted = &angle + &pi;
            let transported = (&pi * (&pi - &shifted) * &pi / (&pi * &pi)).unwrap();
            (-transported - angle) * k
        },
        5 => {
            // (k+sqrt(2))(k-sqrt(2)) = k^2-2.
            (&k + &root2) * (&k - &root2) - &k * &k + Real::from(2)
        },
        _ => unreachable!(),
    };
    zero + Real::from((index % 3) as i32 - 1)
}

fn verify(name: &str, answers: &[Option<Rational>]) {
    let mut counts = [0_usize; 3];
    let mut hash = 0xcbf29ce484222325_u64;
    for (index, answer) in answers.iter().enumerate() {
        let expected = Rational::new((index % 3) as i32 - 1);
        assert_eq!(answer.as_ref(), Some(&expected), "{name}, item {index}");
        counts[index % 3] += 1;
        for byte in answer.as_ref().unwrap().to_string().bytes().chain([b'\n']) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"negative\":{},\"zero\":{},\"positive\":{},\"canonical_rational_fnv1a64\":\"{hash:016x}\"}}",
        counts[0], counts[1], counts[2]
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for (kind, name) in [
        "multiradical_square",
        "quadratic_reciprocal",
        "pi_parameter_transport",
        "radical_laurent_transport",
        "opaque_linear_atom",
        "conjugate_product",
    ]
    .into_iter()
    .enumerate()
    {
        let values = measure(&format!("{name}_construction"), count, || {
            (0..count).map(|i| fixture(kind, i)).collect::<Vec<_>>()
        });
        let non_leaf = values.iter().filter(|v| v.exact_rational().is_none()).count();
        println!("{{\"fixture\":\"{name}\",\"non_leaf_inputs\":{non_leaf}}}");
        for phase in ["first", "warm"] {
            let query = format!("{name}_{phase}");
            let answers = measure(&query, count, || {
                values
                    .iter()
                    .map(|value| black_box(value).exact_rational_normal_form())
                    .collect::<Vec<_>>()
            });
            verify(&query, &answers);
        }
        let cloned = values.clone();
        let query = format!("{name}_cloned");
        let answers = measure(&query, count, || {
            cloned
                .iter()
                .map(|value| black_box(value).exact_rational_normal_form())
                .collect::<Vec<_>>()
        });
        verify(&query, &answers);
    }
}
