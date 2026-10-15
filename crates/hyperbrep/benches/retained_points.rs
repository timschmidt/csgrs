use std::cmp::Ordering;
use std::hint::black_box;
use std::time::{Duration, Instant};

use hyperbrep::{
    Axis, Curve3, CurveParameterLocation, GeometricPoint3, Matrix4, Parameter, Point3, Real,
};

const ITERATIONS: usize = 1000;

fn measure(mut operation: impl FnMut() -> usize) -> (Duration, usize) {
    let mut checksum = 0;
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        checksum += black_box(operation());
    }
    (start.elapsed(), checksum)
}

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn controls() -> Vec<Point3> {
    [-5, -7, -9, -11, -13, 145]
        .map(|n| {
            Point3::new(
                (Real::from(n) / Real::from(5)).unwrap(),
                Real::zero(),
                Real::zero(),
            )
        })
        .to_vec()
}

fn root(curve: &Curve3) -> Parameter {
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("expected a selected root");
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn weighted_curve() -> Curve3 {
    let weights = (1..=6).map(Real::from).collect::<Vec<_>>();
    let controls = controls()
        .into_iter()
        .zip(&weights)
        .map(|(point, weight)| {
            let x = (point.x / weight).unwrap();
            Point3::new(x.clone(), Real::one(), Real::from(2) * x + Real::from(3))
        })
        .collect();
    Curve3::rational_bezier(controls, weights).unwrap()
}

fn main() {
    let root_curve = Curve3::rational_bezier(controls(), vec![Real::one(); 6]).unwrap();
    let parameter = root(&root_curve);
    let line = Curve3::line(point(0, 0, 0), point(2, 4, 6)).unwrap();
    let half: Parameter = (Real::one() / Real::from(2)).unwrap().into();
    let expected = point(1, 2, 3);
    let (elapsed, checksum) = measure(|| {
        let value = black_box(&line).point_at(black_box(&half)).unwrap();
        assert_eq!(value.try_coordinates(), Some(&expected));
        1
    });
    println!(
        "retained_points/authored_line: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    let lower = GeometricPoint3::from_coordinates(point(1, 2, 3));
    let upper = GeometricPoint3::from_coordinates(point(2, 4, 6));
    let (elapsed, checksum) = measure(|| {
        // A fresh curve and point make both projection caches cold. The shared
        // selected root is prepared outside timing and may already be refined.
        let curve = Curve3::line(point(0, 0, 0), point(2, 4, 6)).unwrap();
        let value = curve.point_at(black_box(&parameter)).unwrap();
        assert!(value.try_coordinates().is_none());
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            assert_eq!(
                value.compare_coordinate(&lower, axis).unwrap(),
                Ordering::Greater
            );
            assert_eq!(
                value.compare_coordinate(&upper, axis).unwrap(),
                Ordering::Less
            );
        }
        1
    });
    println!(
        "retained_points/selected_line_cold_carrier: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    let selected = line.point_at(&parameter).unwrap();
    let alias = selected.clone();
    let (elapsed, checksum) = measure(|| {
        assert!(
            black_box(&selected)
                .coincides_with(black_box(&alias))
                .unwrap()
        );
        1
    });
    println!(
        "retained_points/shared_image_identity: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    let first = weighted_curve().point_at(&parameter).unwrap();
    let second = weighted_curve().point_at(&root(&root_curve)).unwrap();
    let weighted_expected = GeometricPoint3::from_coordinates(point(0, 1, 3));
    assert!(first.coincides_with(&weighted_expected).unwrap());
    assert!(first.coincides_with(&second).unwrap());
    let (elapsed, checksum) = measure(|| {
        assert!(
            black_box(&first)
                .coincides_with(black_box(&second))
                .unwrap()
        );
        1
    });
    println!(
        "retained_points/independent_weighted_images_warm: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    let nurbs = Curve3::nurbs(
        5,
        controls(),
        vec![Real::one(); 6],
        std::iter::repeat_n(Real::from(2), 6)
            .chain(std::iter::repeat_n(Real::from(5), 6))
            .collect(),
    )
    .unwrap();
    let knot_parameter = root(&nurbs);
    let origin = GeometricPoint3::from_coordinates(Point3::origin());
    assert!(
        nurbs
            .point_at(&knot_parameter)
            .unwrap()
            .coincides_with(&origin)
            .unwrap()
    );
    let (elapsed, checksum) = measure(|| {
        let value = black_box(&nurbs)
            .point_at(black_box(&knot_parameter))
            .unwrap();
        assert!(value.try_coordinates().is_none());
        assert!(value.coincides_with(&origin).unwrap());
        1
    });
    println!(
        "retained_points/nurbs_fresh_image_warm_carrier: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    selected.bounds().unwrap();
    let (elapsed, checksum) = measure(|| {
        let bounds = black_box(&selected).bounds().unwrap();
        let lower = GeometricPoint3::from_coordinates(bounds.mins);
        let upper = GeometricPoint3::from_coordinates(bounds.maxs);
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            assert_ne!(
                selected.compare_coordinate(&lower, axis).unwrap(),
                Ordering::Less
            );
            assert_ne!(
                selected.compare_coordinate(&upper, axis).unwrap(),
                Ordering::Greater
            );
        }
        1
    });
    println!(
        "retained_points/bounds_and_enclosure_checks_warm: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );

    let transform =
        Matrix4::from_row_major([1, 2, 0, 3, 0, -1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from));
    let inverse = transform.clone().inverse().unwrap();
    let (elapsed, checksum) = measure(|| {
        let restored = black_box(&selected)
            .transformed(&transform)
            .unwrap()
            .transformed(&inverse)
            .unwrap();
        assert!(restored.try_coordinates().is_none());
        assert!(restored.coincides_with(&selected).unwrap());
        1
    });
    println!(
        "retained_points/affine_roundtrip: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32
    );
}
