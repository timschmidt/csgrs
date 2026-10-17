use hyperbrep::{
    ConeBranch, ConicalRevolution, GeometryError, NonConicalGeneratorReason, Point2, Point3, Real,
    STRICT_PREDICATES, Surface, SurfaceKind, Vector3,
};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::cmp::Ordering;

#[test]
fn unresolved_incidence_stays_typed_under_ambient_preview_and_recovery() {
    let tiny: Real = "1e-2000".parse().unwrap();
    let gap = (Real::pi() + &tiny + &tiny).exp().unwrap() - (Real::pi() + tiny).exp().unwrap();
    assert_eq!(
        compare_reals(&gap, &Real::zero(), STRICT_PREDICATES).value(),
        None
    );
    let attempt = || {
        ConicalRevolution::from_line(
            Point3::new(Real::zero(), gap.clone(), Real::zero()),
            Vector3::new([1.into(), 0.into(), 1.into()]),
            Point3::origin(),
            Vector3::z(),
        )
    };
    assert!(matches!(
        attempt(),
        Err(GeometryError::PredicateUnresolved { .. })
    ));
    let preview = hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, attempt);
    assert!(matches!(
        preview.certified().unwrap(),
        Err(GeometryError::PredicateUnresolved { .. })
    ));
    let zero = hyperbrep::OperationContext::strict(hyperbrep::WorkBudget::ZERO);
    let retry = zero.evaluate(|| {
        ConicalRevolution::from_line(
            Point3::new(1.into(), 0.into(), 1.into()),
            Vector3::new([1.into(), 0.into(), 1.into()]),
            Point3::origin(),
            Vector3::z(),
        )
    });
    let source = retry.result.unwrap().unwrap();
    assert_eq!(retry.usage, hyperbrep::WorkUsage::default());
    for branch in [
        ConeBranch::IncreasingParameter,
        ConeBranch::DecreasingParameter,
    ] {
        let chart = source.chart(branch).unwrap();
        let point = chart
            .surface()
            .point_at(&Point2::new(Real::zero(), Real::one()))
            .unwrap();
        assert_eq!(source.branch_at(&point).unwrap(), branch);
    }
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn equal(a: &Real, b: &Real) {
    assert!(
        matches!(
            compare_reals(a, b, STRICT_PREDICATES),
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                ..
            }
        ),
        "{a:?} != {b:?}"
    );
}
fn equal_vector(a: &Vector3, b: &Vector3) {
    for i in 0..3 {
        equal(&a[i], &b[i]);
    }
}
fn equal_point(a: &Point3, b: &Point3) {
    equal_vector(&(a - &Point3::origin()), &(b - &Point3::origin()));
}
fn rotate(v: &Vector3, axis: &Vector3, u: &Real) -> Vector3 {
    let axial = axis.clone() * v.dot(axis);
    let radial = v - &axial;
    axial + radial.clone() * u.clone().cos() + axis.cross(&radial) * u.clone().sin()
}

#[test]
fn conical_revolution_maps_both_branches_and_oriented_partials() {
    let center = Point3::new(Real::pi(), Real::e(), 2.into());
    let original_axis = Vector3::new([0.into(), q(3, 5), q(4, 5)]);
    let radial = Vector3::x();
    let origin =
        center.clone() + radial.clone() * Real::from(3) + original_axis.clone() * Real::from(7);
    let apex = center.clone() + original_axis.clone();
    for axis_sign in [-1, 1] {
        let axis = original_axis.clone() * Real::from(axis_sign);
        for generator_scale in [q(-7, 3), Real::one(), q(1, 1000)] {
            let velocity =
                (radial.clone() + original_axis.clone() * Real::from(2)) * &generator_scale;
            let source = ConicalRevolution::from_line(
                origin.clone(),
                velocity.clone(),
                center.clone(),
                axis.clone(),
            )
            .unwrap();
            let apex_parameter = (Real::from(-3) / &generator_scale).unwrap();
            equal(source.apex_parameter(), &apex_parameter);
            equal_point(source.apex(), &apex);
            equal_point(source.generator().0, &origin);
            equal_vector(source.generator().1, &velocity);
            equal_point(source.axis().0, &center);
            equal_vector(source.axis().1, &axis);
            for (branch, sign) in [
                (ConeBranch::IncreasingParameter, 1),
                (ConeBranch::DecreasingParameter, -1),
            ] {
                let chart = source.chart(branch).unwrap();
                assert_eq!(chart.branch(), branch);
                assert_eq!(chart.surface().kind(), SurfaceKind::Cone);
                let cone = chart.surface().conical_geometry().unwrap();
                equal_point(cone.apex, &apex);
                let expected_reversed =
                    compare_reals(&velocity.dot(&axis), &Real::zero(), STRICT_PREDICATES);
                assert_eq!(
                    chart.reverses_orientation(),
                    matches!(
                        expected_reversed,
                        PredicateOutcome::Decided {
                            value: Ordering::Less,
                            ..
                        }
                    )
                );
                for distance in [q(1, 7), Real::from(4)] {
                    let t = &apex_parameter + Real::from(sign) * distance;
                    for angle in [Real::zero(), Real::pi() * q(1, 2), q(3, 4).atan().unwrap()] {
                        let uv = Point2::new(angle.clone(), t.clone());
                        let native = chart.native_parameters(&uv);
                        let line_point = origin.clone() + velocity.clone() * &t;
                        let expected =
                            center.clone() + rotate(&(&line_point - &center), &axis, &angle);
                        assert_eq!(source.branch_at(&expected).unwrap(), branch);
                        equal_point(&chart.surface().point_at(&native).unwrap(), &expected);
                        let canonical = chart.canonical_parameters_of(&expected).unwrap();
                        equal_point(&chart.surface().point_at(&canonical).unwrap(), &expected);
                        equal(&canonical.y, &native.y);
                        let recovered = chart.source_parameters(&native).unwrap();
                        equal(&recovered.x, &angle);
                        equal(&recovered.y, &t);
                        let partials = chart.surface().partials_at(&native).unwrap();
                        let u_partial = axis.cross(&(&expected - &center));
                        let v_partial = rotate(&velocity, &axis, &angle);
                        equal_vector(&(partials.u().clone() * chart.angular_scale()), &u_partial);
                        equal_vector(&(partials.v().clone() * chart.slant_scale()), &v_partial);
                        let normal = chart.surface().normal_at(&native).unwrap();
                        let source_normal = u_partial.cross(&v_partial);
                        equal(&normal.cross(&source_normal).norm_squared(), &Real::zero());
                        assert!(
                            matches!(compare_reals(&normal.dot(&source_normal), &Real::zero(), STRICT_PREDICATES),
                            PredicateOutcome::Decided { value, .. }
                            if value == if chart.reverses_orientation() { Ordering::Less } else { Ordering::Greater })
                        );
                    }
                }
                let wrong = chart.native_parameters(&Point2::new(
                    Real::zero(),
                    &apex_parameter - Real::from(sign),
                ));
                assert_eq!(
                    chart.surface().point_at(&wrong).unwrap_err(),
                    GeometryError::SurfaceParameterOutsideDomain
                );
                let at_apex =
                    chart.native_parameters(&Point2::new(Real::zero(), apex_parameter.clone()));
                equal_point(&chart.surface().point_at(&at_apex).unwrap(), &apex);
                assert_eq!(
                    chart.surface().normal_at(&at_apex).unwrap_err(),
                    GeometryError::SingularSurfaceParameter
                );
                assert_eq!(
                    chart.canonical_parameters_of(&apex).unwrap_err(),
                    GeometryError::SingularSurfaceParameter
                );
            }
            assert_eq!(
                source.branch_at(&apex).unwrap_err(),
                GeometryError::SingularSurfaceParameter
            );
        }
    }
}

#[test]
fn shifted_generator_keeps_negative_parameters_on_a_native_cone() {
    let source = ConicalRevolution::from_line(
        Point3::new(5.into(), 0.into(), 0.into()),
        Vector3::new([3.into(), 0.into(), (-7).into()]),
        Point3::origin(),
        Vector3::z(),
    )
    .unwrap();
    equal(source.apex_parameter(), &q(-5, 3));
    equal_point(source.apex(), &Point3::new(0.into(), 0.into(), q(35, 3)));
    let chart = source.chart(ConeBranch::IncreasingParameter).unwrap();
    assert_eq!(
        source
            .branch_at(&Point3::new(2.into(), 0.into(), 7.into()))
            .unwrap(),
        ConeBranch::IncreasingParameter
    );
    let uv = chart.native_parameters(&Point2::new(0.into(), (-1).into()));
    equal_point(
        &chart.surface().point_at(&uv).unwrap(),
        &Point3::new(2.into(), 0.into(), 7.into()),
    );
    assert!(matches!(
        compare_reals(&uv.y, &Real::zero(), STRICT_PREDICATES),
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        }
    ));
    equal(&chart.source_parameters(&uv).unwrap().y, &Real::from(-1));
}

#[test]
fn invalid_nonconical_and_nearby_sources_remain_distinct() {
    let make = |origin, velocity, axis| {
        ConicalRevolution::from_line(origin, velocity, Point3::origin(), axis)
    };
    assert_eq!(
        make(Point3::origin(), Vector3::zero(), Vector3::z()).unwrap_err(),
        GeometryError::DegenerateLine
    );
    assert_eq!(
        make(Point3::origin(), Vector3::x(), Vector3::z() * Real::from(2)).unwrap_err(),
        GeometryError::InvalidRevolutionAxis
    );
    for (origin, direction, reason) in [
        (
            Point3::new(1.into(), 0.into(), 0.into()),
            Vector3::z(),
            NonConicalGeneratorReason::Parallel,
        ),
        (
            Point3::origin(),
            Vector3::x(),
            NonConicalGeneratorReason::Perpendicular,
        ),
        (
            Point3::new(0.into(), q(1, 1_000_000) * q(1, 1_000_000), 0.into()),
            Vector3::new([1.into(), 0.into(), 2.into()]),
            NonConicalGeneratorReason::Skew,
        ),
    ] {
        assert_eq!(
            make(origin, direction, Vector3::z()).unwrap_err(),
            GeometryError::NonConicalGenerator(reason)
        );
    }
    let source = make(
        Point3::origin(),
        Vector3::new([1.into(), 0.into(), 2.into()]),
        Vector3::z(),
    )
    .unwrap();
    let gap = q(1, 1_000_000) * q(1, 1_000_000);
    for radial in [Real::one() - &gap, Real::one() + &gap] {
        assert_eq!(
            source
                .branch_at(&Point3::new(radial, 0.into(), 2.into()))
                .unwrap_err(),
            GeometryError::PointNotOnSurface
        );
    }
    assert!(
        Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())
            .unwrap()
            .conical_geometry()
            .is_none()
    );
}
