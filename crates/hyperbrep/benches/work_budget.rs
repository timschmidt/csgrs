use std::hint::black_box;
use std::time::{Duration, Instant};

use hyperbrep::{Curve3, CurveParameterLocation, OperationContext, Point3, Real, WorkBudget};

const LINE_ITERATIONS: usize = 10_000;
const ROOT_ITERATIONS: usize = 500;

type Sample = (Duration, usize);

fn measure(iterations: usize, mut operation: impl FnMut() -> usize) -> Sample {
    let mut checksum = 0;
    let start = Instant::now();
    for _ in 0..iterations {
        checksum += black_box(operation());
    }
    (start.elapsed(), checksum)
}

fn measure_pair(
    iterations: usize,
    scoped_first: bool,
    direct: impl FnMut() -> usize,
    scoped: impl FnMut() -> usize,
) -> (Sample, Sample) {
    if scoped_first {
        let scoped = measure(iterations, scoped);
        (measure(iterations, direct), scoped)
    } else {
        let direct = measure(iterations, direct);
        (direct, measure(iterations, scoped))
    }
}

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn main() {
    let scoped_first = std::env::args().any(|arg| arg == "--scoped-first");
    let line = Curve3::line(
        Point3::origin(),
        Point3::new(Real::one(), Real::one(), Real::one()),
    )
    .unwrap();
    let half = rational(1, 2);
    let middle = Point3::new(half.clone(), half.clone(), half.clone());
    let half_parameter = hyperbrep::Parameter::from_real(half.clone());
    let line_query = || {
        let point = black_box(&line)
            .point_at(black_box(&half_parameter))
            .unwrap();
        assert_eq!(black_box(point).try_coordinates(), Some(&middle));
        1
    };
    let zero = OperationContext::strict(WorkBudget::ZERO);
    let (direct, scoped) = measure_pair(LINE_ITERATIONS, scoped_first, line_query, || {
        let attempted = black_box(&zero).evaluate(line_query);
        assert_eq!(attempted.usage.refinements, 0);
        assert_eq!(attempted.usage.promotions, 0);
        attempted.result.unwrap()
    });
    let (elapsed, checksum) = direct;
    println!(
        "work_budget/direct_line_query: {LINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / LINE_ITERATIONS as u32,
    );
    let (elapsed, checksum) = scoped;
    println!(
        "work_budget/scoped_line_query: {LINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / LINE_ITERATIONS as u32,
    );

    let curve = Curve3::rational_bezier(
        [-105, 241, -381, 381, -241, 105]
            .map(|n| Point3::new(rational(n, 10240), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let origin = Point3::origin();
    let expected = [1, 2, 4, 6, 7].map(|n| rational(n, 8));
    let roots = || {
        let CurveParameterLocation::Parameters(parameters) =
            black_box(&curve).parameters_of(black_box(&origin)).unwrap()
        else {
            panic!("expected five exact roots");
        };
        assert_eq!(parameters.len(), expected.len());
        for (parameter, expected) in black_box(&parameters).iter().zip(&expected) {
            assert_eq!(parameter.try_real(), Some(expected));
        }
        parameters.len()
    };
    let bounded = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 0,
    });
    let (direct, scoped) = measure_pair(ROOT_ITERATIONS, scoped_first, roots, || {
        let attempted = black_box(&bounded).evaluate(roots);
        assert!(attempted.usage.refinements > 0);
        assert!(attempted.usage.refinements <= bounded.work_budget().refinements);
        assert_eq!(attempted.usage.promotions, 0);
        attempted.result.unwrap()
    });
    let (elapsed, checksum) = direct;
    println!(
        "work_budget/direct_five_root_query: {ROOT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ROOT_ITERATIONS as u32,
    );
    let (elapsed, checksum) = scoped;
    println!(
        "work_budget/scoped_five_root_query: {ROOT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ROOT_ITERATIONS as u32,
    );
}
