use num::{BigInt, BigUint, One};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};

struct CountingAllocator;
static ACTIVE: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static REALLOCATIONS: AtomicUsize = AtomicUsize::new(0);

fn record(bytes: usize, reallocation: bool) {
    if ACTIVE.load(Relaxed) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(bytes, Relaxed);
        if reallocation {
            REALLOCATIONS.fetch_add(1, Relaxed);
        }
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            record(layout.size(), false);
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            record(layout.size(), false);
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let pointer = unsafe { System.realloc(pointer, layout, new_size) };
        if !pointer.is_null() {
            record(new_size, true);
        }
        pointer
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

// INSERT_BASELINE

// INSERT_CANDIDATE

fn candidate(approximation: &BigInt, degree: u32) -> (BigUint, BigUint) {
    integer_nth_root_bounds(approximation.magnitude(), degree)
}

type Kernel = fn(&BigInt, u32) -> (BigUint, BigUint);

fn measure(kernel: Kernel, sample: &BigInt, degree: u32) -> ((BigUint, BigUint), [usize; 3]) {
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    REALLOCATIONS.store(0, Relaxed);
    ACTIVE.store(true, Relaxed);
    let result = black_box(kernel)(black_box(sample), black_box(degree));
    ACTIVE.store(false, Relaxed);
    (
        result,
        [
            CALLS.load(Relaxed),
            BYTES.load(Relaxed),
            REALLOCATIONS.load(Relaxed),
        ],
    )
}

fn main() {
    for degree in 3..=9_u32 {
        for root_bits in [16_usize, 128, 1_024, 65_568_u32.div_ceil(degree) as usize] {
            let root = (BigUint::one() << root_bits) + BigUint::from(7_u32);
            let power = BigInt::from(root.pow(degree));
            for delta in -1..=1_i32 {
                let sample = &power + BigInt::from(delta);
                for repeat in 0..3 {
                    let (before, after) = if repeat % 2 == 0 {
                        (
                            measure(baseline, &sample, degree),
                            measure(candidate, &sample, degree),
                        )
                    } else {
                        let after = measure(candidate, &sample, degree);
                        (measure(baseline, &sample, degree), after)
                    };
                    assert_eq!(before.0, after.0);
                    println!(
                        "{{\"degree\":{degree},\"root_bits\":{root_bits},\"sample_bits\":{},\"delta\":{delta},\"repeat\":{repeat},\"baseline\":{:?},\"candidate\":{:?},\"endpoints_equal\":true}}",
                        sample.magnitude().bits(),
                        before.1,
                        after.1,
                    );
                }
            }
        }
    }
}
