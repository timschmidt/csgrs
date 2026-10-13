//! Compaction costs with exact equality checks and separately reported sign facts.
use hyperreal::{Rational, Real, RealSign};
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

fn positive_root(kind: usize) -> Real {
    match kind {
        0 => (Real::from(3) + Real::from(5).sqrt().unwrap())
            .sqrt()
            .unwrap(),
        1 => (Real::from(3) + Real::from(2).sqrt().unwrap())
            .sqrt()
            .unwrap(),
        2 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        _ => unreachable!(),
    }
}

fn expected_sign(kind: usize, index: usize) -> RealSign {
    if kind == 11 {
        RealSign::Zero
    } else if (kind == 3) ^ (index % 2 == 1) {
        RealSign::Negative
    } else {
        RealSign::Positive
    }
}

fn fixture(kind: usize, index: usize) -> Real {
    let value = match kind {
        0 => Real::one() + Real::from(2).sqrt().unwrap(),
        1 => positive_root(0),
        2 | 14 => positive_root(1),
        3 => Real::one() - (positive_root(1) + Real::one()),
        4..=9 => {
            let root = positive_root((kind - 4) / 2);
            let lower = root.certified_rational_interval(-1024).unwrap()[0].clone();
            let delta = root - Real::from(lower);
            assert_eq!(
                delta.certified_sign_until(-1100).sign(),
                Some(RealSign::Positive)
            );
            if kind % 2 == 0 {
                delta
            } else {
                &delta * &delta
            }
        }
        10 | 11 => {
            let root = positive_root(2);
            let rational = &root * &root - Real::from(2) * Real::from(6).sqrt().unwrap();
            if kind == 11 {
                rational - Real::from(5)
            } else {
                rational
            }
        }
        12 => Real::pi() + Real::from(2).sqrt().unwrap(),
        13 => positive_root(2) + Real::from(5).sqrt().unwrap(),
        _ => unreachable!(),
    };
    let numerator = (index % 7 + 1) as i64 * if index % 2 == 0 { 1 } else { -1 };
    let mut value = value * Real::from(Rational::fraction(numerator, 13).unwrap());
    assert_eq!(
        value.certified_sign_until(-1100).sign(),
        Some(expected_sign(kind, index))
    );
    if kind == 14 {
        value.abort(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
            true,
        )));
    }
    value
}

fn witness(name: &str, kind: usize, originals: &[Real], answers: &[Option<Real>]) {
    assert_eq!(originals.len(), answers.len());
    let mut hash = 0xcbf29ce484222325_u64;
    let mut covered = 0_usize;
    let mut immediate_known = 0_usize;
    let mut precision_zero_known = 0_usize;
    for (index, (original, compact)) in originals.iter().zip(answers).enumerate() {
        assert_eq!(compact.is_some(), !matches!(kind, 12 | 13));
        let expected = expected_sign(kind, index);
        let fact = if let Some(compact) = compact {
            covered += 1;
            // Record facts before equality reduction can populate any caches.
            if let Some(sign) = compact.immediate_sign() {
                assert_eq!(sign, expected);
                immediate_known += 1;
            }
            if let Some(sign) = compact.certified_sign_until(0).sign() {
                assert_eq!(sign, expected);
                precision_zero_known += 1;
            }
            assert_eq!(
                (compact - original).quadratic_tower_sign(),
                Some(RealSign::Zero)
            );
            match expected {
                RealSign::Negative => "equal-negative",
                RealSign::Zero => "equal-zero",
                RealSign::Positive => "equal-positive",
            }
        } else {
            "uncovered"
        };
        for byte in fact.bytes().chain([b'\n']) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"inputs\":{},\"covered\":{covered},\"value_equality_fnv1a64\":\"{hash:016x}\",\"immediate_known\":{immediate_known},\"precision_zero_known\":{precision_zero_known}}}",
        originals.len()
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!(count > 0);
    for (kind, name) in [
        "quadratic",
        "denested_tower",
        "nested_tower",
        "negative_normalized_tower",
        "denested_tiny_linear",
        "denested_tiny_square",
        "nested_tiny_linear",
        "nested_tiny_square",
        "biquadratic_tiny_linear",
        "biquadratic_tiny_square",
        "rational_cancellation",
        "zero_cancellation",
        "opaque_decline",
        "outside_tower_decline",
        "aborted_known_tower",
    ]
    .into_iter()
    .enumerate()
    {
        let values = measure(&format!("{name}_construct_and_prove_source"), count, || {
            (0..count)
                .map(|index| fixture(kind, index))
                .collect::<Vec<_>>()
        });
        for phase in ["first_compaction", "warm_compaction", "cloned_compaction"] {
            let row_name = format!("{name}_{phase}");
            let answers = measure(&row_name, count, || {
                if phase == "cloned_compaction" {
                    values
                        .iter()
                        .cloned()
                        .map(|value| black_box(value).compact_quadratic_tower())
                        .collect::<Vec<_>>()
                } else {
                    values
                        .iter()
                        .map(|value| black_box(value).compact_quadratic_tower())
                        .collect::<Vec<_>>()
                }
            });
            witness(&row_name, kind, &values, &answers);
            drop(answers);
        }
        drop(values);
    }
}
