use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
struct Counter;
fn account(size: usize) {
    CALLS.fetch_add(1, Relaxed);
    BYTES.fetch_add(size, Relaxed);
    let live = LIVE.fetch_add(size, Relaxed) + size;
    PEAK.fetch_max(live, Relaxed);
}
unsafe impl GlobalAlloc for Counter {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            account(l.size());
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        LIVE.fetch_sub(l.size(), Relaxed);
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        let q = unsafe { System.realloc(p, l, n) };
        if !q.is_null() {
            LIVE.fetch_sub(l.size(), Relaxed);
            account(n);
        }
        q
    }
}
#[global_allocator]
static ALLOCATOR: Counter = Counter;
#[allow(dead_code)]
#[path = "../snapshot/hypersolve/src/policy_division.rs"]
mod policy_division;
#[allow(dead_code)]
mod pilot {
    include!("../main.rs");
    pub fn profile_all() {
        use crate::{BYTES, CALLS, LIVE, PEAK};
        use std::sync::atomic::Ordering::Relaxed;
        let _ = Real::zero();
        let _ = Real::one();
        let mut count = 0;
        for degree in [0, 1, 2, 4, 8, 16, 32, 64] {
            for bits in [32, 256] {
                for kind in ["dense", "sparse", "fraction"] {
                    for lifetime in ["fresh", "reused"] {
                        let root = coefficients(degree, bits, kind, 137);
                        let p = square(&root);
                        verify_rational(&p, Some(&root)); // Numerical oracle outside allocation windows.
                        println!("INPUT\t{degree}\t{bits}\t{kind}\t{lifetime}");
                        let mut control = None;
                        for (name, f) in ALGORITHMS {
                            let outside = LIVE.load(Relaxed);
                            let input: Vec<_> = p.iter().map(as_real).collect();
                            if lifetime == "reused" {
                                for _ in 0..3 {
                                    drop(f(input.clone()));
                                }
                            }
                            let start = LIVE.load(Relaxed);
                            let calls = CALLS.load(Relaxed);
                            let bytes = BYTES.load(Relaxed);
                            PEAK.store(start, Relaxed);
                            let result = f(input.clone());
                            let before = LIVE.load(Relaxed) as i128 - start as i128;
                            drop(result);
                            let cached = LIVE.load(Relaxed) as i128 - start as i128;
                            let metrics = (
                                CALLS.load(Relaxed) - calls,
                                BYTES.load(Relaxed) - bytes,
                                PEAK.load(Relaxed) - start,
                                before,
                                cached,
                            );
                            drop(input);
                            let escaped = LIVE.load(Relaxed) as i128 - outside as i128;
                            assert_eq!(escaped, 0, "allocation survived all owners");
                            if name == "baseline" {
                                control = Some(metrics);
                            } else if name == "control" {
                                assert_eq!(
                                    Some(metrics),
                                    control,
                                    "identical code allocation control"
                                );
                            }
                            println!(
                                "MEMORY\t{name}\t{}\t{}\t{}\t{}\t{}\t{escaped}",
                                metrics.0, metrics.1, metrics.2, metrics.3, metrics.4
                            );
                            count += 1;
                        }
                    }
                }
            }
        }
        println!("SUMMARY\t{count}");
    }
}
fn main() {
    pilot::profile_all();
}
