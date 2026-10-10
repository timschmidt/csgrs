//! Cost of checked scientific output versus the existing approximate formatter.
#[cfg(feature = "checked-api")]
use hyperreal::ScientificCase;
use hyperreal::{Problem, Real};
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

const COMMON: [&str; 11] = [
    "zero",
    "rational",
    "pi",
    "sqrt_two",
    "tiny",
    "negative_tiny",
    "huge",
    "pi_scaled_huge",
    "pi_scaled_tiny",
    "positive_offset",
    "negative_offset",
];

fn scalar(text: &str) -> Real {
    text.parse().unwrap()
}

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn value(family: &str) -> Real {
    match family {
        "zero" => Real::zero(),
        "rational" => scalar("7/8"),
        "pi" => Real::pi(),
        "sqrt_two" => Real::from(2).sqrt().unwrap(),
        "tiny" => scalar("1e-400"),
        "negative_tiny" => scalar("-1e-400"),
        "huge" => scalar("1e400"),
        "pi_scaled_huge" => Real::pi() * scalar("1e400"),
        "pi_scaled_tiny" => Real::pi() * scalar("1e-400"),
        "positive_offset" => unresolved_zero() + scalar("1e-40"),
        "negative_offset" => unresolved_zero() - scalar("1e-40"),
        "unresolved_zero" => unresolved_zero(),
        "original_hypot" => {
            let delta = (Real::one() + unresolved_zero()) - Real::one();
            Real::hypot2(&delta, &Real::zero()).unwrap()
        },
        "rounding_tie" => unresolved_zero() + scalar("1.234565"),
        _ => panic!("unknown family"),
    }
}

fn expected(family: &str) -> Result<String, Problem> {
    Ok(match family {
        "zero" => "0.000000e0",
        "rational" => "8.750000e-1",
        "pi" => "3.141593e0",
        "sqrt_two" => "1.414214e0",
        "tiny" => "1.000000e-400",
        "negative_tiny" => "-1.000000e-400",
        "huge" => "1.000000e400",
        "pi_scaled_huge" => "3.141593e400",
        "pi_scaled_tiny" => "3.141593e-400",
        "positive_offset" => "1.000000e-40",
        "negative_offset" => "-1.000000e-40",
        "unresolved_zero" | "original_hypot" => return Err(Problem::UnknownZero),
        "rounding_tie" => return Err(Problem::Exhausted),
        _ => panic!("unknown family"),
    }
    .into())
}

fn query(value: &Real, places: usize) -> Result<String, Problem> {
    #[cfg(feature = "checked-api")]
    {
        value.try_scientific(places, -512, ScientificCase::Lower)
    }
    #[cfg(not(feature = "checked-api"))]
    {
        Ok(format!("{value:.places$e}"))
    }
}

fn witness(result: &Result<String, Problem>) -> u64 {
    match result {
        Ok(text) => text.bytes().fold(0xcbf29ce484222325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
        }),
        Err(Problem::UnknownZero) => 1,
        Err(Problem::Exhausted) => 2,
        Err(problem) => panic!("unexpected typed result: {problem:?}"),
    }
}

fn measure(family: &str, stage: &str, values: &[Real], expected: &Result<String, Problem>) {
    let places = if family == "rounding_tie" { 5 } else { 6 };
    let count = values.len();
    #[cfg(feature = "count-alloc")]
    let initial = allocation::start();
    let start = Instant::now();
    let mut checksum = 0_u64;
    for value in values {
        let result = query(black_box(value), places);
        // Every returned value is checked, in both timing and allocation runs.
        // The shared comparison and checksum overhead is part of this harness.
        assert_eq!(&result, expected);
        checksum = checksum.wrapping_add(witness(black_box(&result)));
    }
    let nanos = start.elapsed().as_nanos();
    #[cfg(feature = "count-alloc")]
    let (calls, bytes, retained, peak) = allocation::finish(initial);
    #[cfg(not(feature = "count-alloc"))]
    let (calls, bytes, retained, peak) = (0, 0, 0, 0);
    assert_eq!(checksum, witness(expected).wrapping_mul(count as u64));
    println!(
        "{{\"family\":\"{family}\",\"stage\":\"{stage}\",\"iterations\":{count},\"places\":{places},\"floor\":-512,\"nanos\":{nanos},\"checksum\":{checksum},\"allocation_accounting\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak}}}",
        cfg!(feature = "count-alloc")
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    let mut families = COMMON.to_vec();
    #[cfg(feature = "checked-api")]
    families.extend(["unresolved_zero", "original_hypot", "rounding_tie"]);
    #[cfg(not(feature = "checked-api"))]
    families.shrink_to_fit();
    for family in families {
        let expected = expected(family);
        // Objects are constructed before timing; global constants and caches
        // used internally by constructors can already be warm.
        let values: Vec<_> = (0..count).map(|_| value(family)).collect();
        measure(family, "first", &values, &expected);
        measure(family, "warm", &values, &expected);
    }
}
