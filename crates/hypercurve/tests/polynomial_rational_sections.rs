//! Polynomial sections retain authored rational parameter charts.

use hypercurve::{
    Curve2, CurveParameterRange2, CurvePoint2, Point2, PolynomialCurveSet2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn curves(equation: BivariatePolynomial) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn on(point: &CurvePoint2, equation: BivariatePolynomial) {
    assert_eq!(
        point
            .rational_form(equation, polynomial(&[&[1]]))
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

fn quintic() -> BivariatePolynomial {
    polynomial(&[&[5, 5, 0, 0, 0, 1], &[-1]])
}

fn circle_equation() -> BivariatePolynomial {
    polynomial(&[&[-1, 0, 1], &[], &[1]])
}

fn quarter_circle() -> Curve2 {
    // x=(1-t^2)/(1+t^2), y=-2t/(1+t^2), with rational Bernstein weights.
    RationalQuadraticBezier2::try_new(
        Point2::from_values(1, 0),
        Point2::from_values(1, -1),
        Point2::from_values(0, -1),
        1.into(),
        1.into(),
        2.into(),
    )
    .unwrap()
    .into()
}

fn finite_contact(branch: &Curve2, curve: &Curve2, other_equation: &BivariatePolynomial) {
    for (a, b, swapped) in [(branch, curve, false), (curve, branch, true)] {
        let result = a.intersect_curve(b).unwrap();
        assert!(result.is_complete() && result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        let topology = a.intersection_topology(b).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        assert_eq!(topology.result().contacts().len(), 1);
        let contact = &topology.result().contacts()[0];
        on(contact.point(), quintic());
        on(contact.point(), other_equation.clone());
        assert_eq!(
            contact
                .point()
                .affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            contact
                .point()
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Less
        );
        assert_eq!(
            contact.tangent_cross_sign(),
            Some(if swapped {
                hyperreal::RealSign::Positive
            } else {
                hyperreal::RealSign::Negative
            })
        );
        assert!(contact.is_certified_transverse());
        for (curve, location) in [(a, contact.first()), (b, contact.second())] {
            assert!(
                curve
                    .point_at(&location.parameter().unwrap())
                    .unwrap()
                    .coincides_with(contact.point())
                    .unwrap()
            );
        }
        let replay = contact.point().definition().unwrap().into_point().unwrap();
        on(&replay, quintic());
        on(&replay, other_equation.clone());
        let next = topology.first()[0]
            .intersection_topology(&topology.second()[0])
            .unwrap();
        assert_eq!((next.first().len(), next.second().len()), (1, 1));
        assert_eq!(next.result().contacts().len(), 1);
    }
}

#[test]
fn polynomial_spans_keep_finite_polynomial_contacts() {
    let set = curves(quintic());
    let negative = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    finite_contact(
        &set.curves()[0],
        &negative,
        &polynomial(&[&[0, 0, 1], &[-1]]),
    );
    let positive = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(1, 2)),
        Point2::from_values(1, 1),
    ));
    for (a, b) in [(&set.curves()[0], &positive), (&positive, &set.curves()[0])] {
        assert!(a.intersection_topology(b).unwrap().result().is_disjoint());
    }
}

#[test]
fn rational_spans_keep_finite_polynomial_contacts() {
    let set = curves(quintic());
    finite_contact(&set.curves()[0], &quarter_circle(), &circle_equation());
}

#[test]
fn polynomial_spans_keep_common_component_correspondence() {
    let set = curves(polynomial(&[&[0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 1);
    let branch = &set.curves()[0];
    let quadratic = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(-1, 1),
        Point2::from_values(0, -1),
        Point2::from_values(1, 1),
    ));
    for (a, b, swapped) in [(branch, &quadratic, false), (&quadratic, branch, true)] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().contacts().is_empty());
        assert_eq!(topology.result().overlaps().len(), 1);
        let overlap = &topology.result().overlaps()[0];
        let branch_range = [q(-3, 4).into(), q(-1, 4).into()];
        let curve_range = [q(1, 4).into(), q(1, 2).into()];
        let clipped = if swapped {
            overlap.restrict(curve_range, branch_range)
        } else {
            overlap.restrict(branch_range, curve_range)
        }
        .unwrap()
        .unwrap();
        let mut points = Vec::new();
        for (s, t) in [
            (
                clipped.first_range().start(),
                clipped.second_range().start(),
            ),
            (clipped.first_range().end(), clipped.second_range().end()),
        ] {
            let point = a.point_at(s).unwrap();
            assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
            on(&point, polynomial(&[&[0, 1], &[], &[-1]]));
            points.push(point);
        }
        for expected in [q(-1, 2), q(-1, 4)] {
            assert!(points.iter().any(|point| {
                point
                    .affine_form(1.into(), 0.into(), 0.into())
                    .compare_to_real(&expected)
                    .unwrap()
                    == Ordering::Equal
            }));
        }
        let first = a
            .restrict_source(
                CurveParameterRange2::try_new(
                    clipped.first_range().start().clone(),
                    clipped.first_range().end().clone(),
                )
                .unwrap(),
            )
            .unwrap();
        let second = b
            .restrict_source(
                CurveParameterRange2::try_new(
                    clipped.second_range().start().clone(),
                    clipped.second_range().end().clone(),
                )
                .unwrap(),
            )
            .unwrap();
        let next = first.intersection_topology(&second).unwrap();
        assert_eq!(next.result().overlaps().len(), 1);
        assert_eq!((next.first().len(), next.second().len()), (1, 1));
    }
}

#[test]
fn rational_spans_keep_common_component_correspondence() {
    let set = curves(circle_equation());
    assert_eq!(set.curves().len(), 2);
    let negative = &set.curves()[0];
    let circle = quarter_circle();
    for (a, b) in [(negative, &circle), (&circle, negative)] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().contacts().is_empty());
        assert_eq!(topology.result().overlaps().len(), 1);
        let overlap = &topology.result().overlaps()[0];
        assert_eq!(
            overlap.orientation(),
            hypercurve::CurveOverlapOrientation2::Reversed
        );
        for (s, t) in [
            (
                overlap.first_range().start(),
                overlap.second_range().start(),
            ),
            (overlap.first_range().end(), overlap.second_range().end()),
        ] {
            let point = a.point_at(s).unwrap();
            on(&point, circle_equation());
            assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
        }
    }
}

#[test]
fn polynomial_branch_consumes_retained_rational_abscissa() {
    // C(t)=(t^3,t^2); at y=1/2 the positive t and x=t^3 remain correlated.
    let cubic = Curve2::from(hypercurve::CubicBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(1, 3)),
        Point2::from_values(1, 1),
    ));
    let line = Curve2::from(
        hypercurve::LineSeg2::try_new(
            Point2::new(0.into(), q(1, 2)),
            Point2::new(1.into(), q(1, 2)),
        )
        .unwrap(),
    );
    let contact = cubic.intersect_curve(&line).unwrap();
    assert_eq!(contact.contacts().len(), 1);
    let point = contact.contacts()[0].point().clone();
    assert!(point.coordinates().is_none());
    on(&point, polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    drop(contact);
    drop(cubic);
    drop(line);
    let set = curves(polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 2);
    let branch = &set.curves()[1];
    assert_eq!(
        branch
            .parameter_domain()
            .start()
            .compare(&Real::zero().into())
            .unwrap(),
        Ordering::Equal
    );
    let replay = branch.point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, polynomial(&[&[0, 0, 0, 1], &[], &[-1]]));
    assert_eq!(
        replay
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn polynomial_branch_consumes_fifth_degree_rational_abscissa() {
    // C(t)=(t^2,t^5), so at y=1/2 the source x solves 4x^5=1.
    let controls = (0..=5)
        .map(|i| {
            Point2::new(
                q(i * (i - 1), 20),
                if i == 5 { Real::one() } else { Real::zero() },
            )
        })
        .collect();
    let source =
        Curve2::from(hypercurve::RationalBezier2::try_new(controls, vec![Real::one(); 6]).unwrap());
    let line = Curve2::from(
        hypercurve::LineSeg2::try_new(
            Point2::new(0.into(), q(1, 2)),
            Point2::new(1.into(), q(1, 2)),
        )
        .unwrap(),
    );
    let contact = source.intersect_curve(&line).unwrap();
    assert_eq!(contact.contacts().len(), 1);
    let point = contact.contacts()[0].point().clone();
    assert!(point.coordinates().is_none());
    let equation = polynomial(&[&[0, 0, 1], &[], &[], &[], &[], &[-1]]);
    on(&point, equation.clone());
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    drop(contact);
    drop(source);
    drop(line);
    let set = curves(equation.clone());
    assert_eq!(set.curves().len(), 2);
    let replay = set.curves()[1].point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, equation);
    on(&replay, polynomial(&[&[-1], &[], &[], &[], &[], &[4]]));
    assert_eq!(
        replay
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn polynomial_branch_consumes_generic_quintic_abscissa_without_projection() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let roots = polynomial.isolate_unit_interval_roots().unwrap();
    assert_eq!(roots.len(), 1);
    let parameter = roots[0].clone().into();
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let point = source.point_at(&parameter).unwrap();
    assert!(point.coordinates().is_none());
    on(&point, quintic());
    on(&point, polynomial_from_parabola());
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(x.as_bezier_parameter().is_none());
    drop(source);
    drop(roots);
    let set = curves(quintic());
    let replay = set.curves()[0].point_at(&x).unwrap();
    assert!(replay.coincides_with(&point).unwrap());
    on(&replay, quintic());
    on(&replay, polynomial_from_parabola());
}

fn polynomial_from_parabola() -> BivariatePolynomial {
    polynomial(&[&[0, 0, 1], &[-1]])
}

fn ascending(
    a: hypercurve::CurveParameter2,
    b: hypercurve::CurveParameter2,
) -> CurveParameterRange2 {
    if a.compare(&b).unwrap() == Ordering::Greater {
        CurveParameterRange2::try_new(b, a).unwrap()
    } else {
        CurveParameterRange2::try_new(a, b).unwrap()
    }
}

#[test]
fn rational_components_keep_monotone_visits_and_algebraic_clips() {
    let equation = polynomial(&[&[0, 1], &[], &[-1]]);
    let set = curves(equation.clone());
    // C(t)=((2t-1)^2,(2t-1)^4) visits the same parabola twice.
    let curve = Curve2::from(
        hypercurve::RationalBezier2::try_new(
            vec![
                Point2::from_values(1, 1),
                Point2::from_values(0, -1),
                Point2::new(q(-1, 3), 1.into()),
                Point2::from_values(0, -1),
                Point2::from_values(1, 1),
            ],
            vec![Real::one(); 5],
        )
        .unwrap(),
    );
    for (a, b, swapped) in [
        (&set.curves()[0], &curve, false),
        (&curve, &set.curves()[0], true),
    ] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().contacts().is_empty());
        assert_eq!(topology.result().overlaps().len(), 2);
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        let mut orientations = Vec::new();
        for overlap in topology.result().overlaps() {
            orientations.push(overlap.orientation());
            let x = [q(1, 4).into(), q(1, 2).into()];
            let t = [Real::zero().into(), Real::one().into()];
            let clipped = if swapped {
                overlap.restrict(t, x)
            } else {
                overlap.restrict(x, t)
            }
            .unwrap()
            .unwrap();
            for (s, t) in [
                (
                    clipped.first_range().start(),
                    clipped.second_range().start(),
                ),
                (clipped.first_range().end(), clipped.second_range().end()),
            ] {
                let point = a.point_at(s).unwrap();
                on(&point, equation.clone());
                assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
            }
            let first = a
                .restrict_source(ascending(
                    clipped.first_range().start().clone(),
                    clipped.first_range().end().clone(),
                ))
                .unwrap();
            let second = b
                .restrict_source(ascending(
                    clipped.second_range().start().clone(),
                    clipped.second_range().end().clone(),
                ))
                .unwrap();
            let next = first.intersection_topology(&second).unwrap();
            assert_eq!(next.result().overlaps().len(), 1);
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            assert!(
                clipped
                    .restrict(
                        [Real::zero().into(), Real::one().into()],
                        [Real::zero().into(), Real::one().into()]
                    )
                    .unwrap()
                    .is_some()
            );
        }
        assert!(orientations.contains(&hypercurve::CurveOverlapOrientation2::Same));
        assert!(orientations.contains(&hypercurve::CurveOverlapOrientation2::Reversed));
    }
}

#[test]
fn rational_common_support_keeps_other_branch_endpoint_contacts() {
    let set = curves(circle_equation());
    let positive = &set.curves()[1];
    let rational = quarter_circle();
    for (a, b) in [(positive, &rational), (&rational, positive)] {
        let result = a.intersect_curve(b).unwrap();
        assert!(result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 1);
        let hit = &result.contacts()[0];
        assert!(
            hit.point()
                .coincides_with(&Point2::from_values(1, 0).into())
                .unwrap()
        );
        assert!(!hit.is_certified_transverse());
    }
    let set = curves(polynomial(&[&[0, 0, 1], &[], &[-1]]));
    assert_eq!(set.curves().len(), 4);
    let other = &set.curves()[2];
    assert_eq!(
        other
            .point_at(&q(1, 2).into())
            .unwrap()
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Less
    );
    let diagonal = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(-1, -1),
        Point2::from_values(0, 0),
        Point2::from_values(1, 1),
    ));
    let result = other.intersect_curve(&diagonal).unwrap();
    assert!(result.overlaps().is_empty());
    assert_eq!(result.contacts().len(), 1);
    assert!(
        result.contacts()[0]
            .point()
            .coincides_with(&Point2::from_values(0, 0).into())
            .unwrap()
    );
    assert_eq!(result.contacts()[0].tangent_cross_sign(), None);
}

#[test]
fn polynomial_rational_sections_keep_finite_domains_and_tangency() {
    let equation = polynomial(&[&[0, 1], &[], &[-1]]);
    let set = curves(equation.clone());
    for (y, count, tangent) in [
        (Real::zero(), 1, true),
        (q(1, 4), 2, false),
        (-q(1, 1 << 20), 0, false),
    ] {
        let line = Curve2::from(QuadraticBezier2::new(
            Point2::new((-1).into(), y.clone()),
            Point2::new(0.into(), y.clone()),
            Point2::new(1.into(), y.clone()),
        ));
        for (a, b) in [(&set.curves()[0], &line), (&line, &set.curves()[0])] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().overlaps().is_empty());
            assert_eq!(topology.result().contacts().len(), count);
            for contact in topology.result().contacts() {
                on(contact.point(), equation.clone());
                assert_eq!(
                    contact
                        .point()
                        .affine_form(0.into(), 1.into(), 0.into())
                        .compare_to_real(&y)
                        .unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    contact.tangent_cross_sign() == Some(hyperreal::RealSign::Zero),
                    tangent
                );
            }
        }
    }
    let horizontal = Curve2::from(QuadraticBezier2::new(
        Point2::new((-1).into(), q(1, 4)),
        Point2::new(0.into(), q(1, 4)),
        Point2::new(1.into(), q(1, 4)),
    ));
    let restricted = horizontal
        .restrict_source(CurveParameterRange2::try_new(q(1, 2).into(), Real::one().into()).unwrap())
        .unwrap();
    let result = set.curves()[0].intersect_curve(&restricted).unwrap();
    assert_eq!(result.contacts().len(), 1);
    assert_eq!(
        result.contacts()[0]
            .point()
            .affine_form(1.into(), 0.into(), 0.into())
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
}

#[test]
fn polynomial_rational_sections_consume_native_circular_charts() {
    let set = curves(quintic());
    let arc = Curve2::from(
        hypercurve::CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, -1),
            Point2::from_values(0, 0),
            true,
        )
        .unwrap(),
    );
    finite_contact(&set.curves()[0], &arc, &circle_equation());
}

#[test]
fn polynomial_rational_sections_keep_spline_source_charts() {
    let controls = vec![
        Point2::from_values(-1, 1),
        Point2::new(q(-1, 2), 0.into()),
        Point2::new(q(1, 2), 0.into()),
        Point2::from_values(1, 1),
    ];
    let knots: Vec<_> = [-2, -2, -2, 0, 2, 2, 2]
        .into_iter()
        .map(Real::from)
        .collect();
    let polynomial_spline = Curve2::from(
        hypercurve::PolynomialSplineCurve2::try_new(2, controls.clone(), knots.clone()).unwrap(),
    );
    let rational_spline = Curve2::from(
        hypercurve::NurbsCurve2::try_new(2, controls, vec![Real::one(); 4], knots).unwrap(),
    );
    let equation = polynomial(&[&[0, 1], &[], &[-1]]);
    let set = curves(equation.clone());
    for spline in [polynomial_spline, rational_spline] {
        for (a, b) in [(&set.curves()[0], &spline), (&spline, &set.curves()[0])] {
            let topology = a.intersection_topology(b).unwrap();
            assert!(topology.result().contacts().is_empty());
            assert_eq!(topology.result().overlaps().len(), 2);
            for overlap in topology.result().overlaps() {
                for (s, t) in [
                    (
                        overlap.first_range().start(),
                        overlap.second_range().start(),
                    ),
                    (overlap.first_range().end(), overlap.second_range().end()),
                ] {
                    let s = a
                        .parameter_from_span(overlap.first_span_index(), s)
                        .unwrap();
                    let t = b
                        .parameter_from_span(overlap.second_span_index(), t)
                        .unwrap();
                    let point = a.point_at(&s).unwrap();
                    on(&point, equation.clone());
                    assert!(point.coincides_with(&b.point_at(&t).unwrap()).unwrap());
                }
            }
        }
        let restricted = spline
            .restrict_source(
                CurveParameterRange2::try_new(Real::from(-1).into(), Real::one().into()).unwrap(),
            )
            .unwrap();
        let topology = set.curves()[0].intersection_topology(&restricted).unwrap();
        assert_eq!(topology.result().overlaps().len(), 2);
        assert_eq!(topology.second().len(), 2);
    }
}

#[test]
fn polynomial_rational_sections_keep_transformed_charts() {
    let set = curves(quintic());
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    for (transform, world, reflected) in [
        (
            hypercurve::Similarity2::try_from_real_affine(
                0.into(),
                (-1).into(),
                1.into(),
                0.into(),
                3.into(),
                4.into(),
            )
            .unwrap(),
            polynomial(&[&[267, -1], &[-410], &[270], &[-90], &[15], &[-1]]),
            false,
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                2.into(),
                (-3).into(),
            )
            .unwrap(),
            polynomial(&[&[261, 410, 270, 90, 15, 1], &[1]]),
            true,
        ),
    ] {
        let branch = set.curves()[0].transform_similarity(&transform).unwrap();
        let rational = source.transform_similarity(&transform).unwrap();
        for (a, b, swapped) in [(&branch, &rational, false), (&rational, &branch, true)] {
            let topology = a.intersection_topology(b).unwrap();
            assert_eq!(topology.result().contacts().len(), 1);
            let hit = &topology.result().contacts()[0];
            on(hit.point(), world.clone());
            assert_eq!(
                hit.tangent_cross_sign(),
                Some(if reflected ^ swapped {
                    hyperreal::RealSign::Positive
                } else {
                    hyperreal::RealSign::Negative
                })
            );
            for (curve, location) in [(a, hit.first()), (b, hit.second())] {
                assert!(
                    curve
                        .point_at(&location.parameter().unwrap())
                        .unwrap()
                        .coincides_with(hit.point())
                        .unwrap()
                );
            }
            let next = topology.first()[0]
                .intersection_topology(&topology.second()[0])
                .unwrap();
            assert_eq!(next.result().contacts().len(), 1);
        }
    }
}

#[test]
fn rational_cusp_component_keeps_one_overlap_without_duplicate_contact() {
    let equation = polynomial(&[&[0, 0, 1], &[], &[], &[-1]]);
    let set = curves(equation.clone());
    assert_eq!(set.curves().len(), 2);
    // With t=2u-1, C(u)=(t^2,t^3); the two visits meet at the cusp.
    let cusp = Curve2::from(hypercurve::CubicBezier2::new(
        Point2::from_values(1, -1),
        Point2::new(q(-1, 3), 1.into()),
        Point2::new(q(-1, 3), (-1).into()),
        Point2::from_values(1, 1),
    ));
    for branch in set.curves() {
        for (a, b) in [(branch, &cusp), (&cusp, branch)] {
            let topology = a.intersection_topology(b).unwrap();
            assert_eq!(topology.result().overlaps().len(), 1);
            assert!(topology.result().contacts().is_empty());
            let overlap = &topology.result().overlaps()[0];
            for (s, t) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                let s = a
                    .parameter_from_span(overlap.first_span_index(), s)
                    .unwrap();
                let t = b
                    .parameter_from_span(overlap.second_span_index(), t)
                    .unwrap();
                let point = a.point_at(&s).unwrap();
                on(&point, equation.clone());
                assert!(point.coincides_with(&b.point_at(&t).unwrap()).unwrap());
            }
        }
    }
}

#[test]
fn rational_common_components_keep_transforms_and_traversal() {
    let set = curves(polynomial(&[&[0, 1], &[], &[-1]]));
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(-1, 1),
        Point2::from_values(0, -1),
        Point2::from_values(1, 1),
    ));
    for (transform, equation) in [
        (
            hypercurve::Similarity2::try_from_real_affine(
                0.into(),
                (-1).into(),
                1.into(),
                0.into(),
                3.into(),
                4.into(),
            )
            .unwrap(),
            polynomial(&[&[13, -8, 1], &[1]]),
        ),
        (
            hypercurve::Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                2.into(),
                (-3).into(),
            )
            .unwrap(),
            polynomial(&[&[-1, 1], &[4], &[-1]]),
        ),
    ] {
        for ar in [false, true] {
            for br in [false, true] {
                let mut branch = set.curves()[0].transform_similarity(&transform).unwrap();
                let mut rational = source.transform_similarity(&transform).unwrap();
                if ar {
                    branch = branch.reversed().unwrap();
                }
                if br {
                    rational = rational.reversed().unwrap();
                }
                for (a, b) in [(&branch, &rational), (&rational, &branch)] {
                    let topology = a.intersection_topology(b).unwrap();
                    assert!(topology.result().contacts().is_empty());
                    assert_eq!(topology.result().overlaps().len(), 1);
                    let overlap = &topology.result().overlaps()[0];
                    assert_eq!(
                        overlap.orientation(),
                        if ar ^ br {
                            hypercurve::CurveOverlapOrientation2::Reversed
                        } else {
                            hypercurve::CurveOverlapOrientation2::Same
                        }
                    );
                    for (s, t) in [
                        (
                            overlap.first_range().start(),
                            overlap.second_range().start(),
                        ),
                        (overlap.first_range().end(), overlap.second_range().end()),
                    ] {
                        let point = a.point_at(s).unwrap();
                        on(&point, equation.clone());
                        assert!(point.coincides_with(&b.point_at(t).unwrap()).unwrap());
                    }
                }
            }
        }
    }
}

#[test]
fn polynomial_overlap_parameters_use_original_source_chart() {
    let set = curves(quintic());
    let branch = &set.curves()[0];
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let point = source.point_at(&roots[0].clone().into()).unwrap();
    let x = point
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(x.as_bezier_parameter().is_none());
    let mapped = branch.parameter_from_span(0, &x).unwrap();
    assert_eq!(mapped.compare(&x).unwrap(), Ordering::Equal);
    assert!(
        branch
            .point_at(&mapped)
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
    assert!(matches!(
        branch.parameter_from_span(1, &x),
        Err(hypercurve::ExactCurveError::Invalid { .. })
    ));
}

#[test]
fn polynomial_rational_sections_keep_degree_seven_source_relations() {
    // C(t)=(t^7,-t); the contact parameter solves t^7+t^5+5t-5=0.
    let source = Curve2::from(
        hypercurve::RationalBezier2::try_new(
            (0..=7)
                .map(|i| Point2::new(if i == 7 { Real::one() } else { Real::zero() }, q(-i, 7)))
                .collect(),
            vec![Real::one(); 8],
        )
        .unwrap(),
    );
    let set = curves(quintic());
    finite_contact(
        &set.curves()[0],
        &source,
        &polynomial(&[&[0, 0, 0, 0, 0, 0, 0, 1], &[1]]),
    );
}

#[test]
fn polynomial_rational_sections_retry_and_replay() {
    let (a, b, point) = {
        let set = curves(quintic());
        let source = Curve2::from(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(0.into(), q(-1, 2)),
            Point2::from_values(1, -1),
        ));
        let denied = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget::ZERO,
            || {
                assert!(
                    matches!(set.curves()[0].intersect_curve(&source),Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason()==hypercurve::UncertaintyReason::BudgetExhausted)
                );
            },
        );
        let reached = denied.result.unwrap_err();
        assert_eq!((reached.limit, reached.used), (0, 0));
        assert!(reached.requested > 0);
        let retry = hypersolve::work_budget::with_budget(
            hypersolve::work_budget::WorkBudget {
                refinements: 100_000,
                promotions: 100_000,
            },
            || set.curves()[0].intersection_topology(&source),
        );
        let topology = retry.result.unwrap().unwrap();
        assert_eq!(topology.result().contacts().len(), 1);
        (
            topology.first()[0].clone(),
            topology.second()[0].clone(),
            topology.result().contacts()[0].point().clone(),
        )
    };
    let replay = point.definition().unwrap().into_point().unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let next = a.intersection_topology(&b).unwrap();
            assert_eq!(next.result().contacts().len(), 1);
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
            let hit = &next.result().contacts()[0];
            on(hit.point(), quintic());
            on(hit.point(), polynomial(&[&[0, 0, 1], &[-1]]));
            assert!(hit.point().coincides_with(&replay).unwrap());
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}

#[test]
fn rational_overlaps_keep_selected_trims_across_policies() {
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    let source = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::new(0.into(), q(-1, 2)),
        Point2::from_values(1, -1),
    ));
    let selected = source
        .point_at(&roots[0].clone().into())
        .unwrap()
        .affine_form(1.into(), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(selected.as_bezier_parameter().is_none());
    let equation = polynomial(&[&[0, 1], &[], &[-1]]);
    let set = curves(equation.clone());
    let rational = Curve2::from(QuadraticBezier2::new(
        Point2::from_values(-1, 1),
        Point2::from_values(0, -1),
        Point2::from_values(1, 1),
    ));
    let overlap = set.curves()[0]
        .intersect_curve(&rational)
        .unwrap()
        .overlaps()[0]
        .clone();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let clipped = overlap
                .restrict(
                    [selected.clone(), q(3, 4).into()],
                    [Real::zero().into(), Real::one().into()],
                )
                .unwrap()
                .unwrap();
            assert_eq!(
                clipped.first_range().start().compare(&selected).unwrap(),
                Ordering::Equal
            );
            let again = clipped
                .restrict(
                    [Real::from(-1).into(), Real::one().into()],
                    [Real::zero().into(), Real::one().into()],
                )
                .unwrap()
                .unwrap();
            assert_eq!(
                again.first_range().start().compare(&selected).unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                again.first_range().end().compare(&q(3, 4).into()).unwrap(),
                Ordering::Equal
            );
            for (s, t) in [
                (again.first_range().start(), again.second_range().start()),
                (again.first_range().end(), again.second_range().end()),
            ] {
                let point = set.curves()[0].point_at(s).unwrap();
                on(&point, equation.clone());
                assert!(
                    point
                        .coincides_with(&rational.point_at(t).unwrap())
                        .unwrap()
                );
            }
            let a = set.curves()[0]
                .restrict_source(ascending(
                    again.first_range().start().clone(),
                    again.first_range().end().clone(),
                ))
                .unwrap();
            let b = rational
                .restrict_source(ascending(
                    again.second_range().start().clone(),
                    again.second_range().end().clone(),
                ))
                .unwrap();
            let next = a.intersection_topology(&b).unwrap();
            assert_eq!(next.result().overlaps().len(), 1);
            assert_eq!((next.first().len(), next.second().len()), (1, 1));
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}

#[test]
fn polynomial_rational_sections_keep_degree_thirty_seven_source_relations() {
    // C(t)=(t^37,-t), retaining the base selection t^37+t^5+5t-5=0.
    let source = Curve2::from(
        hypercurve::RationalBezier2::try_new(
            (0..=37)
                .map(|i| Point2::new(if i == 37 { Real::one() } else { Real::zero() }, q(-i, 37)))
                .collect(),
            vec![Real::one(); 38],
        )
        .unwrap(),
    );
    let set = curves(quintic());
    let mut y = vec![Real::zero(); 38];
    y[37] = Real::one();
    finite_contact(
        &set.curves()[0],
        &source,
        &BivariatePolynomial::new(vec![y, vec![Real::one()]]),
    );
}
