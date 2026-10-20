use std::cmp::Ordering;
use std::hint::black_box;
use std::time::{Duration, Instant};

use hyperbrep::{Curve3, CurveParameterLocation, Parameter, Point3, Real};

const ITERATIONS: usize = 250;

fn measure(mut operation: impl FnMut() -> usize) -> (Duration, usize) {
    let mut checksum = 0;
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        checksum += black_box(operation());
    }
    (start.elapsed(), checksum)
}

fn controls() -> Vec<Real> {
    [-5, -7, -9, -11, -13, 145]
        .map(|n| (Real::from(n) / Real::from(5)).unwrap())
        .to_vec()
}

fn check_roots(curve: &Curve3, ranges: &[(Parameter, Parameter)]) -> usize {
    let CurveParameterLocation::Parameters(parameters) =
        black_box(curve).parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("expected selected roots");
    };
    assert_eq!(parameters.len(), ranges.len());
    for (parameter, (lower, upper)) in parameters.iter().zip(ranges) {
        assert!(parameter.try_real().is_none());
        assert_eq!(parameter.compare(lower).unwrap(), Ordering::Greater);
        assert_eq!(parameter.compare(upper).unwrap(), Ordering::Less);
    }
    parameters.len()
}

fn main() {
    let unit = [(Real::zero().into(), Real::one().into())];
    let unweighted = Curve3::rational_bezier(
        controls()
            .into_iter()
            .map(|x| Point3::new(x, Real::zero(), Real::zero()))
            .collect(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let (elapsed, checksum) = measure(|| check_roots(&unweighted, &unit));
    println!(
        "retained_parameters/quintic: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    let weights = (1..=6).map(Real::from).collect::<Vec<_>>();
    let weighted = Curve3::rational_bezier(
        controls()
            .into_iter()
            .zip(&weights)
            .map(|(x, weight)| {
                let x = (x / weight).unwrap();
                Point3::new(x.clone(), Real::zero(), Real::from(2) * x)
            })
            .collect(),
        weights,
    )
    .unwrap();
    let (elapsed, checksum) = measure(|| check_roots(&weighted, &unit));
    println!(
        "retained_parameters/weighted_spatial_quintic: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    let separated = Curve3::rational_bezier(
        controls()
            .into_iter()
            .map(|x| Point3::new(x.clone(), Real::zero(), Real::from(2) * x + Real::one()))
            .collect(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let (elapsed, checksum) = measure(|| {
        assert!(matches!(
            black_box(&separated)
                .parameters_of(&Point3::origin())
                .unwrap(),
            CurveParameterLocation::None
        ));
        0
    });
    println!(
        "retained_parameters/rejected_planar_projection: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    let values = controls();
    let nurbs = Curve3::nurbs(
        5,
        values
            .iter()
            .chain(values.iter().rev().skip(1))
            .map(|x| Point3::new(x.clone(), Real::zero(), Real::zero()))
            .collect(),
        vec![Real::one(); 11],
        std::iter::repeat_n(Real::from(2), 6)
            .chain(std::iter::repeat_n(Real::from(5), 5))
            .chain(std::iter::repeat_n(Real::from(9), 6))
            .collect(),
    )
    .unwrap();
    let spans = [
        (Real::from(2).into(), Real::from(5).into()),
        (Real::from(5).into(), Real::from(9).into()),
    ];
    let (elapsed, checksum) = measure(|| check_roots(&nurbs, &spans));
    println!(
        "retained_parameters/nurbs_two_spans: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );
}
