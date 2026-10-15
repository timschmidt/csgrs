use std::cmp::Ordering;

use hyperbrep::{
    Axis, Curve3, CurveParameterLocation, GeometricPoint3, GeometryError, Matrix4,
    OperationContext, Parameter, Point3, Real, Vector3, WorkBudget, WorkUsage,
};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn quintic_controls() -> Vec<Point3> {
    [-5, -7, -9, -11, -13, 145]
        .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
        .to_vec()
}

fn parameters(curve: &Curve3) -> Vec<Parameter> {
    let CurveParameterLocation::Parameters(parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("expected finite selected parameters");
    };
    assert!(
        parameters
            .iter()
            .all(|parameter| parameter.try_real().is_none())
    );
    parameters
}

fn root() -> Parameter {
    parameters(&Curve3::rational_bezier(quintic_controls(), vec![Real::one(); 6]).unwrap())
        .remove(0)
}

#[test]
fn public_evaluation_retains_selected_images_and_authored_coordinate_views() {
    let parameter = root();
    let curve = Curve3::line(point(1, 2, 3), point(3, 5, 7)).unwrap();
    let selected = curve.point_at(&parameter).unwrap();
    assert!(selected.try_coordinates().is_none());
    let (source, retained) = selected.curve_source().unwrap();
    assert_eq!(source.kind(), curve.kind());
    assert_eq!(retained.compare(&parameter).unwrap(), Ordering::Equal);
    let lower = GeometricPoint3::from_coordinates(point(2, 3, 5));
    let upper = GeometricPoint3::from_coordinates(point(3, 5, 6));
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        assert_eq!(
            selected.compare_coordinate(&lower, axis).unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            selected.compare_coordinate(&upper, axis).unwrap(),
            Ordering::Less
        );
    }
    let authored = curve
        .point_at(&Parameter::from_real(rational(1, 2)))
        .unwrap();
    assert_eq!(
        authored.try_coordinates(),
        Some(&Point3::new(Real::from(2), rational(7, 2), Real::from(5)))
    );
    assert!(authored.curve_source().is_none());
}

#[test]
fn independent_curves_compare_selected_images_and_preserve_tiny_separation() {
    let parameter = root();
    let first = Curve3::line(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .point_at(&parameter)
        .unwrap();
    let second = Curve3::line(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .point_at(&root())
        .unwrap();
    assert!(first.coincides_with(&second).unwrap());
    let tiny: Real = "1e-1000".parse().unwrap();
    let shifted = Curve3::line(
        Point3::new(Real::zero(), Real::zero(), tiny.clone()),
        Point3::new(Real::from(2), Real::from(3), Real::from(4) + tiny),
    )
    .unwrap()
    .point_at(&parameter)
    .unwrap();
    assert!(!first.coincides_with(&shifted).unwrap());
    assert_eq!(
        first.compare_coordinate(&shifted, Axis::X).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        first.compare_coordinate(&shifted, Axis::Y).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        first.compare_coordinate(&shifted, Axis::Z).unwrap(),
        Ordering::Less
    );
}

#[test]
fn distinct_selected_parameters_can_reach_the_same_spatial_point() {
    let curve = Curve3::rational_bezier(
        [3, 3, -2, -12, 21]
            .map(|n| Point3::new(rational(n, 48), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 5],
    )
    .unwrap();
    let parameters = parameters(&curve);
    assert_eq!(parameters.len(), 2);
    assert_eq!(
        parameters[0].compare(&parameters[1]).unwrap(),
        Ordering::Less
    );
    let first = curve.point_at(&parameters[0]).unwrap();
    let second = curve.point_at(&parameters[1]).unwrap();
    assert!(first.coincides_with(&second).unwrap());
    assert!(
        first
            .coincides_with(&GeometricPoint3::from_coordinates(Point3::origin()))
            .unwrap()
    );
}

#[test]
fn selected_nurbs_evaluation_uses_the_authored_knot_chart() {
    let curve = Curve3::nurbs(
        5,
        quintic_controls(),
        vec![Real::one(); 6],
        std::iter::repeat_n(Real::from(2), 6)
            .chain(std::iter::repeat_n(Real::from(5), 6))
            .collect(),
    )
    .unwrap();
    let selected = curve.point_at(&parameters(&curve)[0]).unwrap();
    assert!(selected.try_coordinates().is_none());
    assert!(
        selected
            .coincides_with(&GeometricPoint3::from_coordinates(Point3::origin()))
            .unwrap()
    );
}

#[test]
fn retained_point_bounds_enclose_all_coordinates_including_constant_projections() {
    let parameter = root();
    for curve in [
        Curve3::line(point(1, 2, 3), point(3, 5, 7)).unwrap(),
        Curve3::line(point(0, 0, 0), point(0, 0, 1)).unwrap(),
    ] {
        let selected = curve.point_at(&parameter).unwrap();
        let bounds = selected.bounds().unwrap();
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
        assert!(selected.try_coordinates().is_none());
    }
}

#[test]
fn affine_transform_composition_keeps_selected_points_exact() {
    let parameter = root();
    let start = point(1, 2, 3);
    let end = point(3, 5, 7);
    let original = Curve3::line(start.clone(), end.clone())
        .unwrap()
        .point_at(&parameter)
        .unwrap();
    let transform =
        Matrix4::from_row_major([1, 2, 0, 3, 0, -1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from));
    let inverse = transform.clone().inverse().unwrap();
    let expected = Curve3::line(
        transform.transform_point3(&start).unwrap(),
        transform.transform_point3(&end).unwrap(),
    )
    .unwrap()
    .point_at(&parameter)
    .unwrap();
    let mut replayed = original.clone();
    for _ in 0..64 {
        let moved = replayed.transformed(&transform).unwrap();
        assert!(moved.try_coordinates().is_none());
        assert!(moved.coincides_with(&expected).unwrap());
        replayed = moved.transformed(&inverse).unwrap();
        assert!(replayed.coincides_with(&original).unwrap());
        assert_eq!(
            replayed
                .curve_source()
                .unwrap()
                .1
                .compare(&parameter)
                .unwrap(),
            Ordering::Equal
        );
    }
}

#[test]
fn selected_domain_errors_and_unsupported_angular_evaluation_remain_distinct() {
    let line = Curve3::line(point(0, 0, 0), point(1, 1, 1)).unwrap();
    let outside_source = Curve3::nurbs(
        5,
        quintic_controls(),
        vec![Real::one(); 6],
        std::iter::repeat_n(Real::from(2), 6)
            .chain(std::iter::repeat_n(Real::from(5), 6))
            .collect(),
    )
    .unwrap();
    assert_eq!(
        line.point_at(&parameters(&outside_source)[0]).unwrap_err(),
        GeometryError::ParameterOutsideDomain
    );
    let circle = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        Real::zero(),
        Real::tau(),
    )
    .unwrap();
    assert_eq!(
        circle.point_at(&root()).unwrap_err(),
        GeometryError::UnsupportedSelectedEvaluation
    );
    assert_eq!(
        circle
            .point_at(&Parameter::from_real(Real::zero()))
            .unwrap()
            .try_coordinates(),
        Some(&point(1, 0, 0))
    );
    let mut singular = Matrix4::identity();
    singular.0[0][0] = Real::zero();
    let selected = line.point_at(&root()).unwrap();
    assert_eq!(
        selected.transformed(&singular).unwrap_err(),
        GeometryError::SingularTransform
    );
}

#[test]
fn shared_point_and_source_identity_do_not_require_solver_work() {
    let curve = Curve3::line(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let parameter = root();
    let first = curve.point_at(&parameter).unwrap();
    let same_source = curve.point_at(&parameter).unwrap();
    let outcome = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        assert!(first.coincides_with(&first.clone()).unwrap());
        assert!(first.coincides_with(&same_source).unwrap());
    });
    outcome.result.unwrap();
    assert_eq!(outcome.usage, WorkUsage::default());
}

#[test]
fn point_comparison_cannot_consume_ambient_approximate_equality() {
    let tiny: Real = "1e-2000".parse().unwrap();
    let delta = (Real::pi() + &tiny + &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    let first = GeometricPoint3::from_coordinates(Point3::origin());
    let second = GeometricPoint3::from_coordinates(Point3::new(delta, Real::zero(), Real::zero()));
    assert!(first.coincides_with(&second).is_err());
    let approximate = hypercurve::provisional(|| first.coincides_with(&second));
    assert!(approximate.into_unverified().is_err());
}

#[test]
fn selected_comparison_retries_after_stronger_scalar_evidence() {
    let tiny: Real = "1e-2000".parse().unwrap();
    let delta = (Real::pi() + &tiny + &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    let curve = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(delta.clone(), Real::one(), Real::zero()),
        ],
        vec![Real::one(); 2],
    )
    .unwrap();
    let selected = curve.point_at(&root()).unwrap();
    let origin = GeometricPoint3::from_coordinates(Point3::origin());
    assert!(selected.compare_coordinate(&origin, Axis::X).is_err());
    let provisional = hypercurve::provisional(|| selected.compare_coordinate(&origin, Axis::X));
    assert!(provisional.into_unverified().is_err());
    // An unresolved coordinate does not hide a certified separation in another.
    assert!(!selected.coincides_with(&origin).unwrap());
    assert_eq!(
        delta.refine_sign_until(-8192),
        Some(hyperreal::RealSign::Positive)
    );
    assert_eq!(
        selected.compare_coordinate(&origin, Axis::X).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        selected
            .clone()
            .compare_coordinate(&origin, Axis::X)
            .unwrap(),
        Ordering::Greater
    );
    assert!(selected.try_coordinates().is_none());
}

#[test]
fn weighted_selected_image_preserves_the_shared_denominator_in_every_coordinate() {
    let weights = (1..=6).map(Real::from).collect::<Vec<_>>();
    let controls = quintic_controls()
        .into_iter()
        .zip(&weights)
        .map(|(point, weight)| {
            let x = (point.x / weight).unwrap();
            Point3::new(x.clone(), Real::one(), Real::from(2) * x + Real::from(3))
        })
        .collect();
    let curve = Curve3::rational_bezier(controls, weights).unwrap();
    let selected = curve.point_at(&root()).unwrap();
    assert!(selected.try_coordinates().is_none());
    assert!(selected.coincides_with(&point(0, 1, 3).into()).unwrap());
    let bounds = selected.bounds().unwrap();
    for axis in [Axis::X, Axis::Y, Axis::Z] {
        assert_ne!(
            selected
                .compare_coordinate(&bounds.mins.clone().into(), axis)
                .unwrap(),
            Ordering::Less
        );
        assert_ne!(
            selected
                .compare_coordinate(&bounds.maxs.clone().into(), axis)
                .unwrap(),
            Ordering::Greater
        );
    }
}

#[test]
fn concurrent_selected_queries_share_successful_evidence() {
    let parameter = root();
    let first = Curve3::line(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .point_at(&parameter)
        .unwrap();
    let second = Curve3::line(point(0, 0, 0), point(2, 3, 4))
        .unwrap()
        .point_at(&root())
        .unwrap();
    std::thread::scope(|scope| {
        for _ in 0..8 {
            scope.spawn(|| {
                assert!(first.coincides_with(&second).unwrap());
                for axis in [Axis::X, Axis::Y, Axis::Z] {
                    assert_eq!(
                        first.compare_coordinate(&second, axis).unwrap(),
                        Ordering::Equal
                    );
                }
                assert!(first.bounds().is_ok());
                assert!(second.bounds().is_ok());
            });
        }
    });
}
