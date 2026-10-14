//! Allocation mechanism diagnostic, using unchanged public constructors.
use hyperreal::Rational;
use num::{BigInt, BigUint, ToPrimitive};
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};

mod inputs;

const BINS: usize = 513;
static ACTIVE: AtomicBool = AtomicBool::new(false);
static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static CALLS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
static HISTOGRAM: [AtomicUsize; BINS] = [const { AtomicUsize::new(0) }; BINS];

struct Counted;

fn allocated(bytes: usize) {
    let live = LIVE.fetch_add(bytes, Relaxed) + bytes;
    if ACTIVE.load(Relaxed) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(bytes, Relaxed);
        PEAK.fetch_max(live, Relaxed);
        HISTOGRAM[bytes.min(BINS - 1)].fetch_add(1, Relaxed);
    }
}

// SAFETY: every request delegates unchanged to System; bookkeeping owns no
// heap storage, cannot recurse through allocation and never touches payloads.
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        // SAFETY: the caller provides the GlobalAlloc layout contract.
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        // SAFETY: this forwards the caller's valid layout unchanged.
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() {
            allocated(layout.size());
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Relaxed);
        // SAFETY: the caller provides the matching pointer/layout pair.
        unsafe { System.dealloc(pointer, layout) };
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        // SAFETY: the caller provides the original allocation and valid size.
        let result = unsafe { System.realloc(pointer, layout, size) };
        if !result.is_null() {
            LIVE.fetch_sub(layout.size(), Relaxed);
            allocated(size);
        }
        result
    }
}

#[global_allocator]
static ALLOCATOR: Counted = Counted;

struct Scope;
impl Drop for Scope {
    fn drop(&mut self) {
        ACTIVE.store(false, Relaxed);
    }
}

fn measure<T>(body: impl FnOnce() -> T) -> (T, serde_json::Value) {
    assert!(!ACTIVE.load(Relaxed));
    let initial = LIVE.load(Relaxed);
    PEAK.store(initial, Relaxed);
    CALLS.store(0, Relaxed);
    BYTES.store(0, Relaxed);
    for bin in &HISTOGRAM {
        bin.store(0, Relaxed);
    }
    ACTIVE.store(true, Relaxed);
    let scope = Scope;
    let value = black_box(body());
    drop(scope);
    let calls = CALLS.load(Relaxed);
    let bytes = BYTES.load(Relaxed);
    let retained = LIVE.load(Relaxed) as i128 - initial as i128;
    let peak = PEAK.load(Relaxed) - initial;
    // Collect and format only after counters stop. LIVE keeps tracking System
    // requests outside scopes so each retained/peak delta has its own baseline.
    let histogram = HISTOGRAM
        .iter()
        .enumerate()
        .filter_map(|(size, bin)| {
            let count = bin.load(Relaxed);
            if count == 0 {
                None
            } else {
                Some((size.to_string(), count))
            }
        })
        .collect::<std::collections::BTreeMap<_, _>>();
    (
        value,
        serde_json::json!({"calls": calls, "requested_bytes": bytes,
            "retained_bytes": retained, "peak_bytes": peak,
            "request_size_histogram": histogram, "last_bin_means_512_or_more": true}),
    )
}

fn original(value: &BigInt) -> Rational {
    Rational::from_bigint(value.clone())
}

fn signed_word(value: &BigInt) -> Rational {
    value
        .to_i64()
        .map_or_else(|| Rational::from_bigint(value.clone()), Rational::new)
}

fn main() {
    assert_eq!(usize::BITS, 64);
    let reverse = std::env::args().nth(1).as_deref() == Some("reverse");
    let (_, empty) = measure(|| black_box(()));
    assert_eq!(empty["calls"], 0);
    let (control, control_counts) = measure(|| black_box(vec![0_u8; black_box(17)]));
    assert_eq!(control.len(), 17);
    assert_eq!(control_counts["calls"], 1);
    assert_eq!(control_counts["requested_bytes"], 17);
    assert_eq!(control_counts["retained_bytes"], 17);
    assert_eq!(control_counts["request_size_histogram"]["17"], 1);
    drop(control);
    let unwind =
        std::panic::catch_unwind(|| measure(|| panic!("intentional allocation scope unwind")));
    assert!(unwind.is_err() && !ACTIVE.load(Relaxed));
    println!(
        "{}",
        serde_json::json!({"control": "allocation", "passed": true,
        "empty": empty, "allocated": control_counts, "unwind_disabled": true})
    );

    let cases = inputs::VALUES
        .iter()
        .map(|s| s.parse::<BigInt>().unwrap())
        .collect::<Vec<_>>();
    let expected = inputs::VALUES
        .iter()
        .map(|s| s.parse::<Rational>().unwrap())
        .collect::<Vec<_>>();
    // Canonical constructors and lazy constants are warmed equally outside all
    // observations; this is an allocation mechanism probe, not a cold cost trial.
    for value in &cases {
        drop(black_box(original(value)));
        drop(black_box(signed_word(value)));
    }
    let methods = if reverse { [3, 2, 1, 0] } else { [0, 1, 2, 3] };
    for (index, value) in cases.iter().enumerate() {
        for method in methods {
            let (name, counts) = match method {
                0 => {
                    let (result, counts) = measure(|| black_box(value).magnitude().clone());
                    assert_eq!(&result, value.magnitude());
                    ("clone-magnitude", counts)
                },
                1 => {
                    let Some(word) = value.magnitude().to_u128() else {
                        continue;
                    };
                    let (result, counts) = measure(|| BigUint::from(black_box(word)));
                    assert_eq!(&result, value.magnitude());
                    ("primitive-magnitude", counts)
                },
                2 => {
                    let (result, counts) = measure(|| original(black_box(value)));
                    assert_eq!(result, expected[index]);
                    ("original-leaf", counts)
                },
                3 => {
                    let (result, counts) = measure(|| signed_word(black_box(value)));
                    assert_eq!(result, expected[index]);
                    ("signed-word-leaf", counts)
                },
                _ => unreachable!(),
            };
            println!(
                "{}",
                serde_json::json!({"case": inputs::VALUES[index],
                "method": name, "counts": counts, "exact_value_verified": true})
            );
        }
    }
    let boundary = inputs::SIGNED_WORD_VALUES
        .iter()
        .map(|s| s.parse::<BigInt>().unwrap())
        .collect::<Vec<_>>();
    let fit = (0..1000)
        .filter(|index| boundary[index % boundary.len()].to_i64().is_some())
        .count();
    assert_eq!(fit, 501);
    for method in if reverse {
        [true, false]
    } else {
        [false, true]
    } {
        let (_, counts) = measure(|| {
            for index in 0..1000 {
                let value = black_box(&boundary[index % boundary.len()]);
                drop(black_box(if method {
                    signed_word(value)
                } else {
                    original(value)
                }));
            }
        });
        println!(
            "{}",
            serde_json::json!({"batch": "signed-word-cycle", "iterations": 1000,
            "signed_word_fits": fit, "method": if method { "signed-word-leaf" } else { "original-leaf" },
            "counts": counts})
        );
    }
}
