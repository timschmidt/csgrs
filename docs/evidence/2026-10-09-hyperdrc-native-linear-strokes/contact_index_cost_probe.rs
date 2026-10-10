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

struct Fixture {
    points: Vec<CurvePoint2>,
    query: CurvePoint2,
    expected: Option<Vec<usize>>,
    selected: bool,
}

fn selected_point(
    lower: Real,
    polynomial: &BezierParameterPolynomial,
    scale: &Real,
) -> CurvePoint2 {
    let interval = BezierParameterInterval::try_new(lower, Real::one()).unwrap();
    let parameter =
        BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval).unwrap();
    let line = RationalBezier2::try_new(
        vec![
            Point2::from_values(0, 0),
            Point2::new(scale.clone(), Real::zero()),
        ],
        vec![Real::one(); 2],
    )
    .unwrap();
    let range = CurveParameterRange2::try_new(
        BezierParameter2::Algebraic(parameter).into(),
        Real::from(2).into(),
    )
    .unwrap();
    Curve2::try_from_bezier_range(CurveGeometry2::RationalBezier(line), range)
        .unwrap()
        .start()
}

fn fixture(kind: usize, sequence: usize) -> Fixture {
    if kind <= 1 {
        let group = if kind == 0 { 1 } else { 8 };
        let selected = sequence % (64 / group);
        let points = (0..64)
            .map(|i| Point2::from_values(u32::try_from(i / group).unwrap(), 0).into())
            .collect();
        return Fixture {
            points,
            query: Point2::from_values(u32::try_from(selected).unwrap(), 0).into(),
            expected: Some((selected * group..(selected + 1) * group).collect()),
            selected: false,
        };
    }
    if kind == 7 {
        let point = CurvePoint2::from(Point2::new("1e400".parse().unwrap(), Real::zero()));
        return Fixture {
            points: vec![point.clone(); 64],
            query: point,
            expected: None,
            selected: false,
        };
    }
    if kind == 8 {
        let ulp = (Real::one() / Real::from(1_u64 << 52)).unwrap();
        let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
            (&ulp / Real::from(4)).unwrap() - Real::from(2),
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
        .unwrap();
        let scale = Real::try_from(f64::MAX).unwrap();
        let wide = selected_point(Real::zero(), &polynomial, &scale);
        assert_eq!(exact_contact_point_box(&wide), Some([0.0, f64::MAX, 0.0, 0.0]));
        // The exact width MAX is representable; rounding upward broadens
        // the candidate search to negative keys as well. Keep this adverse
        // control to expose the cost of that conservative saturation.
        let mut points: Vec<CurvePoint2> = (1..=64)
            .map(|i| {
                Point2::new(
                    (-&scale * Real::from(i) / Real::from(64)).unwrap(),
                    Real::zero(),
                )
                .into()
            })
            .collect();
        points.push(wide);
        return Fixture {
            points,
            query: Point2::new(scale, Real::zero()).into(),
            expected: Some(vec![64]),
            selected: false,
        };
    }
    let scale = Real::try_from(match kind {
        2 => f64::from_bits((1023_u64 - 600) << 52),
        3 | 5 => 1.0,
        4 => f64::from_bits((1023_u64 + 600) << 52),
        6 => 1e308,
        _ => unreachable!(),
    })
    .unwrap();
    let ulp = (Real::one() / Real::from(1_u64 << 52)).unwrap();
    let delta = (&ulp / Real::from(4)).unwrap();
    let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
        delta - Real::from(2),
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ])
    .unwrap();
    let point = selected_point(-Real::one() - &ulp, &polynomial, &scale);
    let query = selected_point(
        Real::one() - (&ulp / Real::from(2)).unwrap(),
        &polynomial,
        &scale,
    );
    let contacts = if kind == 5 { 64 } else { 1 };
    Fixture {
        points: vec![point; contacts],
        query,
        expected: Some((0..contacts).collect()),
        selected: true,
    }
}

fn build(fixture: &Fixture) -> ContactPointIndex {
    let mut result = ContactPointIndex::default();
    for (i, point) in fixture.points.iter().enumerate() {
        result.push(i, Some(point), i);
    }
    // This no-op merge keeps the exact production lifecycle method linked.
    result.merge_vertex(0, 0);
    result
}

fn verify_construction(name: &str, fixtures: &[Fixture], indices: &[ContactPointIndex]) {
    let correct = fixtures
        .iter()
        .zip(indices)
        .filter(|(fixture, index)| {
            index.boxes.len() == fixture.points.len()
                && (0..fixture.points.len()).all(|i| index.incidences(i) == [i])
        })
        .count();
    println!(
        "{{\"witness\":\"{name}\",\"kind\":\"construction\",\"count\":{},\"correct\":{correct}}}",
        fixtures.len()
    );
}

fn verify_queries(name: &str, fixtures: &[Fixture], values: &[Option<Vec<usize>>]) {
    let correct = fixtures
        .iter()
        .zip(values)
        .filter(|(fixture, value)| fixture.expected == **value)
        .count();
    let mut hash = 0xcbf29ce484222325_u64;
    let mut candidates = 0_usize;
    let mut fallbacks = 0_usize;
    for value in values {
        let label = value.as_ref().map_or(u64::MAX, |v| v.len() as u64);
        hash = (hash ^ label).wrapping_mul(0x100000001b3);
        if let Some(value) = value {
            candidates += value.len();
            for &index in value {
                hash = (hash ^ index as u64).wrapping_mul(0x100000001b3);
            }
        } else {
            fallbacks += 1;
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"kind\":\"query\",\"count\":{},\"correct\":{correct},\"candidates\":{candidates},\"fallbacks\":{fallbacks},\"output_fnv1a64\":\"{hash:016x}\"}}",
        fixtures.len()
    );
}

fn main() {
    let until = Instant::now() + std::time::Duration::from_millis(100);
    let mut state = 1_u64;
    while Instant::now() < until {
        state = black_box(state)
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1);
    }
    black_box(state);
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    evaluate_under(PredicatePolicy::STRICT, || {
        for (kind, family) in [
            "exact_sparse",
            "exact_crowded",
            "selected_tiny",
            "selected_unit",
            "selected_huge",
            "selected_crowded",
            "selected_overflow",
            "unboxed",
            "max_width_saturation",
        ]
        .into_iter()
        .enumerate()
        {
            // Point producers run before index construction measurement.
            let fixtures = (0..count).map(|i| fixture(kind, i)).collect::<Vec<_>>();
            let indices = measure(&format!("{family}_construction"), count, || {
                fixtures.iter().map(build).collect::<Vec<_>>()
            });
            verify_construction(&format!("{family}_construction"), &fixtures, &indices);
            for phase in ["first", "warm", "cloned"] {
                let name = format!("{family}_{phase}");
                let clones = (phase == "cloned").then(|| {
                    fixtures
                        .iter()
                        .map(|fixture| fixture.query.clone())
                        .collect::<Vec<_>>()
                });
                let values = measure(&name, count, || {
                    fixtures
                        .iter()
                        .zip(&indices)
                        .enumerate()
                        .map(|(i, (fixture, index))| {
                            index.candidates(Some(black_box(
                                clones.as_ref().map_or(&fixture.query, |points| &points[i]),
                            )))
                        })
                        .collect::<Vec<_>>()
                });
                verify_queries(&name, &fixtures, &values);
            }
            // Keep equality verification after first-query measurement.
            for fixture in &fixtures {
                if fixture.selected {
                    assert!(fixture.points[0].coincides_with(&fixture.query).unwrap());
                }
            }
        }
    })
    .certified()
    .unwrap();
}
