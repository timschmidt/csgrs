//! Public arc membership; production libraries and separate resource modes.
use hyperreal::endpoint_cost_diagnostic as diagnostic;
mod trace_probe;
use hypercurve::{
    CircularArc2, CurveCertainty, ExactCurveResult, Point2, PredicatePolicy, evaluate_under,
};
use hyperreal::Real;
use std::hint::black_box;
use std::sync::{Arc, atomic::AtomicBool};
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
        hyperreal::endpoint_cost_diagnostic::allocated(bytes);
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
            hyperreal::endpoint_cost_diagnostic::deallocated(layout.size());
            unsafe { System.dealloc(pointer, layout) };
        }
        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, size) };
            if !result.is_null() {
                LIVE.fetch_sub(layout.size(), Relaxed);
                hyperreal::endpoint_cost_diagnostic::deallocated(layout.size());
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
    let between = diagnostic::finish();
    diagnostic::begin();
    #[cfg(feature = "count-alloc")]
    let initial = allocation::start();
    let start = Instant::now();
    let result = body();
    let nanos = start.elapsed().as_nanos();
    #[cfg(feature = "count-alloc")]
    let (calls, bytes, retained, peak) = allocation::finish(initial);
    #[cfg(not(feature = "count-alloc"))]
    let (calls, bytes, retained, peak) = (0, 0, 0, 0);
    let measured = diagnostic::finish();
    println!(
        "{{\"name\":\"{name}\",\"iterations\":{count},\"nanos\":{nanos},\"allocation_accounting\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak}}}",
        cfg!(feature = "count-alloc")
    );
    trace_probe::dump(name, "between", &between);
    trace_probe::dump(name, "measured", &measured);
    diagnostic::begin();
    black_box(result)
}

#[derive(Clone)]
struct Fixture {
    arc: CircularArc2,
    point: Point2,
    expected: Option<bool>,
    approximate: bool,
    terminal: bool,
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn fixture(kind: usize, index: usize) -> Fixture {
    let offset = Real::from((index % 7) as i32);
    let (center, scale) = match kind {
        2 => (
            Point2::new(Real::pi() + &offset, Real::e() - &offset),
            Real::pi(),
        ),
        3 => (
            Point2::new(Real::e().sin() + &offset, Real::e().cos() - &offset),
            Real::one(),
        ),
        _ => (Point2::new(offset.clone(), -offset), Real::one()),
    };
    let transform =
        |x: Real, y: Real| Point2::new(center.x() + &scale * x, center.y() + &scale * y);
    let mut arc = CircularArc2::try_from_center(
        transform(Real::one(), Real::zero()),
        transform(Real::zero(), Real::one()),
        center.clone(),
        false,
    )
    .unwrap();
    if index % 2 == 1 {
        arc = arc.reversed();
    }
    let (x, y) = match kind {
        1 => {
            let half_root = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
            (half_root.clone(), half_root)
        },
        6 => (Real::from(2), Real::zero()),
        7 => (rational(-3, 5), rational(4, 5)),
        8 | 9 => {
            let sine = Real::e().sin();
            let cosine = Real::e().cos();
            let mut zero = &sine * &sine + &cosine * &cosine - Real::one();
            if kind == 8 {
                zero.abort(Arc::new(AtomicBool::new(true)));
            }
            (Real::one() + zero, Real::zero())
        },
        11 => (Real::one() - "1e-40".parse::<Real>().unwrap(), Real::zero()),
        _ => (rational(3, 5), rational(4, 5)),
    };
    let point = match kind {
        4 => arc.start().clone(),
        5 => Point2::new(arc.start().x().clone(), arc.start().y().clone()),
        _ => transform(x, y),
    };
    Fixture {
        arc,
        point,
        expected: match kind {
            6 | 7 | 11 => Some(false),
            8 => None,
            _ => Some(true),
        },
        approximate: kind == 9 || kind == 10,
        terminal: kind == 9,
    }
}

fn query(name: &str, cases: &[Fixture]) {
    let answers: Vec<(ExactCurveResult<bool>, CurveCertainty)> =
        measure(name, cases.len(), || {
            cases
                .iter()
                .map(|case| {
                    let policy = if case.approximate {
                        PredicatePolicy::APPROXIMATE_512
                    } else {
                        PredicatePolicy::STRICT
                    };
                    let result = evaluate_under(policy, || {
                        black_box(&case.arc).contains_point(black_box(&case.point))
                    });
                    let certainty = result.certainty();
                    (result.into_unverified(), certainty)
                })
                .collect()
        });
    let mut hash = 0xcbf29ce484222325_u64;
    let mut decided_true = 0;
    let mut decided_false = 0;
    let mut uncertain = 0;
    let mut terminals = 0;
    for (index, (case, answer)) in cases.iter().zip(&answers).enumerate() {
        assert_eq!(
            answer.0.as_ref().ok().copied(),
            case.expected,
            "membership {name}/{index}"
        );
        assert_eq!(
            answer.1 == CurveCertainty::Approximate512Consumed,
            case.terminal,
            "certainty {name}/{index}"
        );
        match answer.0 {
            Ok(true) => decided_true += 1,
            Ok(false) => decided_false += 1,
            Err(_) => uncertain += 1,
        }
        terminals += usize::from(answer.1 == CurveCertainty::Approximate512Consumed);
        for byte in format!("{answer:?}").bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    let sample = format!("{:?}", answers[0]);
    println!(
        "{{\"witness\":{name:?},\"decided_true\":{decided_true},\"decided_false\":{decided_false},\"uncertain\":{uncertain},\"approximate_terminal_consumed\":{terminals},\"receipt_fnv1a64\":\"{hash:016x}\",\"first_receipt\":{sample:?}}}"
    );
}

fn main() {
    diagnostic::begin();
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    assert!(count > 0);
    for (kind, family) in [
        "rational_non_endpoint",
        "radical_non_endpoint",
        "pi_scaled_non_endpoint",
        "opaque_translated_non_endpoint",
        "stored_endpoint",
        "copied_endpoint_coordinates",
        "off_circle",
        "other_sweep",
        "aborted_unknown",
        "approximate_endpoint",
        "approximate_non_endpoint",
        "near_endpoint_off_circle",
    ]
    .into_iter()
    .enumerate()
    {
        let cases = measure(&format!("{family}_construction"), count, || {
            (0..count)
                .map(|index| fixture(kind, index))
                .collect::<Vec<_>>()
        });
        let non_rational = cases
            .iter()
            .flat_map(|case| {
                [
                    case.arc.start(),
                    case.arc.end(),
                    case.arc.center(),
                    &case.point,
                ]
            })
            .flat_map(|point| [point.x(), point.y()])
            .filter(|coordinate| coordinate.exact_rational_ref().is_none())
            .count();
        println!(
            "{{\"fixture\":{family:?},\"count\":{count},\"non_rational_coordinates\":{non_rational}}}"
        );
        query(&format!("{family}_first"), &cases);
        query(&format!("{family}_warm"), &cases);
        let cloned = cases.clone();
        query(&format!("{family}_cloned"), &cloned);
    }
    trace_probe::dump("process", "tail", &diagnostic::finish());
}
