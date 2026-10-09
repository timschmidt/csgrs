//! Public point equality, with separate construction, preparation, and query costs.
use hypercurve::Point2;
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

struct Case {
    left: Point2,
    right: Point2,
    expected: bool,
}

fn fixture(kind: usize, index: usize) -> Case {
    let integer = (index % 7 + 1) as i32;
    let k = Real::from(integer);
    let root2 = Real::from(2).sqrt().unwrap();
    let root3 = Real::from(3).sqrt().unwrap();
    let pi = Real::pi();
    let (left_x, left_y, right_x, right_y) = match kind {
        0 => {
            let x = Real::new(Rational::fraction(i64::from(integer), 7).unwrap());
            let y = Real::new(Rational::fraction(i64::from(integer * 3), 11).unwrap());
            (x.clone(), y.clone(), x, y)
        },
        1 => {
            let x = &pi + &k;
            let y = Real::e() - &k;
            (x.clone(), y.clone(), x, y)
        },
        2 => {
            // Same exact value through two quadratic-field expressions.
            let sum = &root2 + &k * &root3;
            let square = &sum * &sum;
            let expanded = Real::from(2)
                + Real::from(3) * &k * &k
                + Real::from(2) * &k * Real::from(6).sqrt().unwrap();
            (square, root3.clone(), expanded, root3)
        },
        3 => {
            let third = (&pi / Real::from(3)).unwrap();
            let shifted = &pi - Real::one();
            let transported = (&pi * (&shifted - &third) * &pi / (&pi * &pi)).unwrap();
            (-transported + shifted, root2.clone(), third, root2)
        },
        4 => {
            let angle = Real::new(Rational::fraction(3, 4).unwrap()).atan().unwrap();
            let shifted = &angle + &pi;
            let transported = (&pi * (&pi - &shifted) * &pi / (&pi * &pi)).unwrap();
            (-transported, root3.clone(), angle, root3)
        },
        5..=7 => {
            let x = Real::from(7).sin() + Real::from(53).sqrt().unwrap() + &k;
            let y = Real::from(5).cos() + root3;
            (x.clone(), y.clone(), x, y)
        },
        _ => unreachable!(),
    };
    let left = Point2::new(left_x, left_y);
    if kind == 6 {
        return Case {
            right: left.clone(),
            left,
            expected: true,
        };
    }
    if kind == 7 {
        return Case {
            right: Point2::new(left.x().clone(), left.y().clone()),
            left,
            expected: true,
        };
    }
    let (dx, dy) = [(0, 0), (1, 0), (0, 1), (-1, 1), (1, -1)][index % 5];
    Case {
        left,
        right: Point2::new(right_x + Real::from(dx), right_y + Real::from(dy)),
        expected: dx == 0 && dy == 0,
    }
}

fn verify(name: &str, cases: &[Case], answers: &[bool]) {
    assert_eq!(answers.len(), cases.len());
    let mut yes = 0;
    let mut hash = 0xcbf29ce484222325_u64;
    for (index, (case, &answer)) in cases.iter().zip(answers).enumerate() {
        assert_eq!(answer, case.expected, "{name}, item {index}");
        yes += usize::from(answer);
        hash = (hash ^ u64::from(answer)).wrapping_mul(0x100000001b3);
    }
    println!(
        "{{\"witness\":\"{name}\",\"equal\":{yes},\"unequal\":{},\"boolean_fnv1a64\":\"{hash:016x}\"}}",
        answers.len() - yes
    );
}

fn query(name: &str, cases: &[Case]) {
    let answers = measure(name, cases.len(), || {
        cases
            .iter()
            .map(|case| black_box(&case.left) == black_box(&case.right))
            .collect::<Vec<_>>()
    });
    verify(name, cases, &answers);
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for (kind, name) in [
        "rational",
        "constant_offsets",
        "multiradical_identity",
        "pi_transport_identity",
        "opaque_atom_identity",
        "opaque_offsets",
        "shared_point",
        "copied_coordinates",
    ]
    .into_iter()
    .enumerate()
    {
        let cases = measure(&format!("{name}_construction"), count, || {
            (0..count).map(|i| fixture(kind, i)).collect::<Vec<_>>()
        });
        let non_rational = cases
            .iter()
            .flat_map(|c| [c.left.x(), c.left.y(), c.right.x(), c.right.y()])
            .filter(|v| v.exact_rational_ref().is_none())
            .count();
        println!("{{\"fixture\":\"{name}\",\"non_rational_coordinates\":{non_rational}}}");
        query(&format!("{name}_first"), &cases);
        query(&format!("{name}_repeat"), &cases);
        measure(&format!("{name}_prepare"), count, || {
            for case in &cases {
                for value in [case.left.x(), case.left.y(), case.right.x(), case.right.y()] {
                    assert!(black_box(value).certified_rational_interval(-128).is_some());
                }
            }
        });
        let cached = cases
            .iter()
            .flat_map(|c| [c.left.x(), c.left.y(), c.right.x(), c.right.y()])
            .filter(|v| v.cached_rational_interval(-32).is_some())
            .count();
        assert_eq!(cached, 4 * count);
        println!("{{\"fixture\":\"{name}_prepared\",\"cached_coordinates\":{cached}}}");
        query(&format!("{name}_prepared_first"), &cases);
        query(&format!("{name}_prepared_repeat"), &cases);
        let cloned = cases
            .iter()
            .map(|c| Case {
                left: c.left.clone(),
                right: c.right.clone(),
                expected: c.expected,
            })
            .collect::<Vec<_>>();
        query(&format!("{name}_cloned"), &cloned);
    }
}
