// Second-generation measurement only. All numerical code is the unchanged
// correctness-qualified pilot, included below rather than copied or repaired.
#[cfg(feature = "allocation-profile")]
compile_error!("use live-allocations, not the original windowed allocator");

#[cfg(feature = "live-allocations")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    pub static CALLS: AtomicUsize = AtomicUsize::new(0);
    pub static BYTES: AtomicUsize = AtomicUsize::new(0);
    pub static LIVE: AtomicUsize = AtomicUsize::new(0);
    pub static PEAK: AtomicUsize = AtomicUsize::new(0);
    pub struct Counter;
    fn add(size: usize) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(size, Relaxed);
        let live = LIVE.fetch_add(size, Relaxed) + size;
        PEAK.fetch_max(live, Relaxed);
    }
    unsafe impl GlobalAlloc for Counter {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let p = unsafe { System.alloc(layout) };
            if !p.is_null() {
                add(layout.size());
            }
            p
        }
        unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
            LIVE.fetch_sub(layout.size(), Relaxed);
            unsafe { System.dealloc(p, layout) }
        }
        unsafe fn realloc(&self, p: *mut u8, layout: Layout, new: usize) -> *mut u8 {
            let q = unsafe { System.realloc(p, layout, new) };
            if !q.is_null() {
                LIVE.fetch_sub(layout.size(), Relaxed);
                add(new);
            }
            q
        }
    }
}
#[cfg(feature = "live-allocations")]
#[global_allocator]
static ALLOCATOR: allocation::Counter = allocation::Counter;

#[allow(dead_code)]
mod pilot {
    include!("../main.rs");

    // The duplicate pointer is an identical-code timing control.
    const SELECTED: [(&str, Multiply); 4] = [
        ("baseline", multiply_exact_polynomials),
        ("control", multiply_exact_polynomials),
        ("k8", k8),
        ("gated", gated),
    ];

    // Rebuilding from canonical BigRationals creates new RationalData ownership;
    // cloning Real would share the exact caches whose effects we are separating.
    fn fresh(a: &[BigRational], b: &[BigRational]) -> (Vec<Real>, Vec<Real>) {
        (
            a.iter().map(as_real).collect(),
            b.iter().map(as_real).collect(),
        )
    }
    fn fresh_batch(
        f: Multiply,
        iterations: usize,
        a: &[BigRational],
        b: &[BigRational],
        e: &[Real],
    ) -> (f64, f64) {
        let mut measured_cpu = 0.0;
        let mut measured_wall = 0.0;
        // Pools of eight amortize timer calls without preparing a calibration-
        // sized heap. Input creation/destruction is outside the measured interval;
        // result destruction and exact verification remain inside, as in v1.
        let mut remaining = iterations;
        while remaining > 0 {
            let count = remaining.min(8);
            let pool: Vec<_> = (0..count).map(|_| fresh(a, b)).collect();
            let wall = Instant::now();
            let started = cpu();
            for (a, b) in &pool {
                let result = black_box(f(black_box(a), black_box(b)));
                assert_eq!(result, e);
            }
            measured_cpu += cpu() - started;
            measured_wall += wall.elapsed().as_secs_f64();
            drop(pool);
            remaining -= count;
        }
        (measured_cpu, measured_wall)
    }

    pub fn bench_v2(
        n: usize,
        m: usize,
        bits: usize,
        density: &str,
        lifetime: &str,
        seed: u64,
        seconds: f64,
        rounds: usize,
    ) {
        assert!(!cfg!(feature = "live-allocations"));
        assert!(lifetime == "fresh" || lifetime == "reused");
        let a = values(n, bits, density, seed);
        let b = values(m, bits, density, seed + 391);
        let e = oracle(&a, &b);
        let owned: Vec<_> = SELECTED.iter().map(|_| fresh(&a, &b)).collect();
        println!("INPUT\t{n}\t{m}\t{bits}\t{density}\t{lifetime}\t{seed}");
        let measure = |i: usize, count: usize| {
            let f = SELECTED[i].1;
            if lifetime == "fresh" {
                fresh_batch(f, count, &a, &b, &e)
            } else {
                batch(f, count, &owned[i].0, &owned[i].1, &e)
            }
        };
        let mut batches = vec![0; SELECTED.len()];
        for position in 0..SELECTED.len() {
            let i = (position + seed as usize) % SELECTED.len();
            let mut warm = 0.0;
            while warm < seconds {
                warm += measure(i, 1).0;
            }
            let mut count = 8;
            loop {
                let (c, w) = measure(i, count);
                if c >= seconds {
                    println!("CALIBRATE\t{}\t{count}\t{c:.9}\t{w:.9}", SELECTED[i].0);
                    break;
                }
                count *= 2;
                assert!(count <= 16_777_216);
            }
            batches[i] = count;
        }
        for round in 0..rounds + 2 {
            for position in 0..SELECTED.len() {
                let i = (round + position + seed as usize) % SELECTED.len();
                let (c, w) = measure(i, batches[i]);
                println!(
                    "{}\t{round}\t{position}\t{}\t{}\t{c:.9}\t{w:.9}",
                    if round < 2 { "WARMUP" } else { "MEASURE" },
                    SELECTED[i].0,
                    batches[i]
                );
                if round >= 2 {
                    assert!(c >= seconds * 0.5, "measured batch too short");
                }
            }
        }
        println!("PASS\tbench-v2");
    }

    #[cfg(feature = "live-allocations")]
    pub fn memory_v2(n: usize, m: usize, bits: usize, density: &str, lifetime: &str) {
        use crate::allocation::*;
        use std::sync::atomic::Ordering::Relaxed;
        assert!(lifetime == "fresh" || lifetime == "reused");
        let a = values(n, bits, density, 137);
        let b = values(m, bits, density, 528);
        let e = oracle(&a, &b);
        // Initialize common globals and formatting before scoped measurements.
        let _ = Real::zero();
        let _ = Real::one();
        println!("INPUT\t{n}\t{m}\t{bits}\t{density}\t{lifetime}");
        for (name, f) in SELECTED {
            // Each algorithm owns disjoint input caches, so measurement order
            // cannot warm another algorithm's retained coefficient relations.
            let outside = LIVE.load(Relaxed);
            let (owned_a, owned_b) = fresh(&a, &b);
            if lifetime == "reused" {
                for _ in 0..3 {
                    assert_eq!(f(&owned_a, &owned_b), e);
                }
            }
            let start = LIVE.load(Relaxed);
            let calls = CALLS.load(Relaxed);
            let bytes = BYTES.load(Relaxed);
            PEAK.store(start, Relaxed);
            let product = f(&owned_a, &owned_b);
            let before_drop = LIVE.load(Relaxed) as i128 - start as i128;
            // Product destruction can leave caches owned by either input.
            drop(product);
            let cached = LIVE.load(Relaxed) as i128 - start as i128;
            let peak = PEAK.load(Relaxed) - start;
            let calls = CALLS.load(Relaxed) - calls;
            let bytes = BYTES.load(Relaxed) - bytes;
            drop(owned_a);
            drop(owned_b);
            let escaped = LIVE.load(Relaxed) as i128 - outside as i128;
            assert_eq!(
                escaped, 0,
                "allocations survived all measured input/output owners"
            );
            println!(
                "MEMORY\t{name}\t{calls}\t{bytes}\t{peak}\t{before_drop}\t{cached}\t{escaped}"
            );
        }
    }
}

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let n = args[1].parse().unwrap();
    let m = args[2].parse().unwrap();
    let bits = args[3].parse().unwrap();
    match args[0].as_str() {
        "bench" => pilot::bench_v2(
            n,
            m,
            bits,
            &args[4],
            &args[5],
            args[6].parse().unwrap(),
            args[7].parse().unwrap(),
            args[8].parse().unwrap(),
        ),
        #[cfg(feature = "live-allocations")]
        "memory" => pilot::memory_v2(n, m, bits, &args[4], &args[5]),
        _ => panic!("unknown mode"),
    }
}
