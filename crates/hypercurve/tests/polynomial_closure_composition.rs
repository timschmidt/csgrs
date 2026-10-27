//! Composition witnesses for independently qualified selected-branch features.

use hypercurve::{
    BooleanOp, Curve2, CurveLoopOrientation2, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2,
    PolynomialCurveSet2, Real, RegionPointLocation, Similarity2,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn selected_face_probes_keep_quintic_ordinates_and_policy_receipts() {
    // p(y)=y^5+5y+5 has one real root alpha in (-1,0), since p'>0,
    // p(-1)=-1 and p(0)=5. The strip is [-1,1] x [alpha,0].
    let equation = BivariatePolynomial::new(vec![vec![
        5.into(),
        5.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let source = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    assert_eq!(source.curves().len(), 1);
    let bottom = &source.curves()[0];
    let point = bottom.point_at(&Real::zero().into()).unwrap();
    assert_eq!(
        point
            .affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap()
            .scalar(),
        Some(&Real::zero())
    );
    assert_eq!(
        point
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Less
    );
    assert_eq!(
        point
            .affine_form(0.into(), 1.into(), 1.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        point
            .rational_form(
                equation.clone(),
                BivariatePolynomial::new(vec![vec![Real::one()]])
            )
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    let path = CurvePath2::try_new(vec![
        bottom.clone(),
        Curve2::try_line(bottom.end(), Point2::from_values(1, 0).into()).unwrap(),
        line(Point2::from_values(1, 0), Point2::from_values(-1, 0)),
        Curve2::try_line(Point2::from_values(-1, 0).into(), bottom.start()).unwrap(),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    let middle = region
        .boolean_region(
            &rectangle(q(-1, 2), q(1, 2), (-2).into(), 1.into()),
            BooleanOp::Intersection,
        )
        .unwrap();
    let sides = region
        .boolean_region(&middle, BooleanOp::Difference)
        .unwrap();
    assert_eq!(sides.boundary_paths().unwrap().len(), 2);
    let replay = point.definition().unwrap().into_point().unwrap();
    let turn = Similarity2::try_from_real_affine(
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        3.into(),
        4.into(),
    )
    .unwrap();
    let rotated = |region: &CurveRegion2| {
        region
            .transform_affine(
                &0.into(),
                &(-1).into(),
                &1.into(),
                &0.into(),
                &3.into(),
                &4.into(),
            )
            .unwrap()
    };
    let rotated_point = hypercurve::CurvePointDefinition2::Similarity {
        source: replay.clone(),
        transform: turn,
    }
    .into_point()
    .unwrap();
    // The quarter turn makes the known source abscissa the world ordinate.
    // Its other coordinate remains a selected quintic value.
    for (sides, middle, replay) in [
        (sides.clone(), middle.clone(), replay),
        (rotated(&sides), rotated(&middle), rotated_point),
    ] {
        for policy in [
            hypercurve::PredicatePolicy::STRICT,
            hypercurve::PredicatePolicy::APPROXIMATE_512,
            hypercurve::PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                assert_eq!(
                    sides.classify_point(&replay).unwrap(),
                    RegionPointLocation::Outside
                );
                assert_eq!(
                    middle.classify_point(&replay).unwrap(),
                    RegionPointLocation::Boundary
                );
                let empty = sides
                    .boolean_region(&middle, BooleanOp::Intersection)
                    .unwrap();
                assert!(empty.is_empty());
                let restored = empty.boolean_region(&middle, BooleanOp::Union).unwrap();
                let paths = restored.boundary_paths().unwrap();
                assert_eq!(paths.len(), 1);
                assert_eq!(paths[0].curves().len(), 4);
                let retained: Vec<_> = paths[0]
                    .curves()
                    .iter()
                    .filter_map(Curve2::polynomial_branch)
                    .collect();
                assert_eq!(retained.len(), 1);
                assert_eq!(retained[0].topology().equation(), &equation);
                assert!(retained[0].shares_source(bottom.polynomial_branch().unwrap()));
                assert_eq!(retained[0].root_index(), 0);
            });
            assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
            outcome.certified().unwrap();
        }
    }
}

#[test]
fn guarded_rational_maps_compose_with_algebraic_similarity_sections() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    let equation = polynomial(&[&[2, 1]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-2).into(), 2.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let source = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    assert_eq!(source.curves().len(), 1);
    // Both coordinates retain the original guard 3+x, which is positive here.
    let map = hypercurve::RationalMap2::new(
        [
            polynomial(&[&[], &[3], &[1]]),
            polynomial(&[&[0, 3], &[0, 1]]),
        ],
        polynomial(&[&[3], &[1]]),
    );
    let mapped = source.curves()[0].rational_image(&map).unwrap();
    let s = Real::from(2).sqrt().unwrap();
    let r = Real::from(3).sqrt().unwrap();
    for reflected in [false, true] {
        let image = Similarity2::try_from_real_affine(
            if reflected { -s.clone() } else { s.clone() },
            0.into(),
            0.into(),
            s.clone(),
            r.clone(),
            0.into(),
        )
        .unwrap();
        let branch = mapped.transform_similarity(&image).unwrap();
        let cut = line(Point2::from_values(0, -3), Point2::from_values(0, -1))
            .transform_similarity(&image)
            .unwrap();
        let overlap_line = line(Point2::from_values(-1, -2), Point2::from_values(1, -2))
            .transform_similarity(&image)
            .unwrap();
        let expected = Point2::new(r.clone(), Real::from(-2) * &s);
        for reverse in [false, true] {
            let curve = if reverse {
                branch.reversed().unwrap()
            } else {
                branch.clone()
            };
            let retained = curve.polynomial_branch().unwrap();
            assert_eq!(retained.topology().equation(), &equation);
            assert_eq!(retained.rational_maps(), std::slice::from_ref(&map));
            assert!(retained.image_transform().is_some());
            for (first, second, swapped) in [(&curve, &cut, false), (&cut, &curve, true)] {
                let result = first.intersect_curve(second).unwrap();
                assert!(result.is_complete() && result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), 1);
                let contact = &result.contacts()[0];
                assert!(
                    contact
                        .point()
                        .coincides_with(&expected.clone().into())
                        .unwrap()
                );
                assert_eq!(
                    contact.tangent_cross_sign(),
                    Some(if reflected ^ reverse ^ swapped {
                        hyperreal::RealSign::Negative
                    } else {
                        hyperreal::RealSign::Positive
                    })
                );
                let parameter = if swapped {
                    contact.second()
                } else {
                    contact.first()
                }
                .parameter()
                .unwrap();
                assert_eq!(
                    parameter.compare(&Real::zero().into()).unwrap(),
                    Ordering::Equal
                );
                let replay = contact.point().definition().unwrap().into_point().unwrap();
                assert!(replay.coincides_with(&expected.clone().into()).unwrap());
            }
            let split = curve.intersection_topology(&overlap_line).unwrap();
            assert_eq!(split.result().overlaps().len(), 1);
            assert_eq!(split.first().len(), 3);
            let mut contacts = 0;
            for piece in split.first() {
                let branch = piece.polynomial_branch().unwrap();
                assert_eq!(branch.rational_maps(), std::slice::from_ref(&map));
                assert_eq!(branch.topology().equation(), &equation);
                let result = piece.intersect_curve(&cut).unwrap();
                assert!(result.is_complete());
                contacts += result.contacts().len();
            }
            assert_eq!(contacts, 1);
        }
    }
}

fn line(a: Point2, b: Point2) -> Curve2 {
    LineSeg2::try_new(a, b).unwrap().into()
}

fn rectangle(x0: Real, x1: Real, y0: Real, y1: Real) -> CurveRegion2 {
    let vertices = [
        Point2::new(x0.clone(), y0.clone()),
        Point2::new(x1.clone(), y0),
        Point2::new(x1, y1.clone()),
        Point2::new(x0, y1),
    ];
    let curves = (0..4)
        .map(|index| line(vertices[index].clone(), vertices[(index + 1) % 4].clone()))
        .collect();
    CurveRegion2::try_from_boundary_paths(
        &[CurvePath2::try_new(curves).unwrap()],
        FillRule::NonZero,
    )
    .unwrap()
}

#[test]
fn closed_domain_components_remain_filled_boundaries_after_boolean_reconstruction() {
    // The two selected components y=0 and y=1 lie on the closed source bounds.
    // Together with x=-1 and x=1 they enclose exactly [-1,1] x [0,1].
    let equation = BivariatePolynomial::new(vec![vec![0.into(), (-1).into(), 1.into()]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [0.into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let source = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    assert_eq!(source.curves().len(), 2);
    let path = CurvePath2::try_new(vec![
        source.curves()[0].clone(),
        line(Point2::from_values(1, 0), Point2::from_values(1, 1)),
        source.curves()[1].reversed().unwrap(),
        line(Point2::from_values(-1, 1), Point2::from_values(-1, 0)),
    ])
    .unwrap();
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
    assert_eq!(
        path.reversed().unwrap().simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Clockwise
    );
    let clip = rectangle(q(-1, 2), q(1, 2), (-1).into(), 2.into());
    for fill in [FillRule::NonZero, FillRule::EvenOdd] {
        for boundary in [path.clone(), path.reversed().unwrap()] {
            let region = CurveRegion2::try_from_boundary_paths(&[boundary], fill).unwrap();
            let selected = region
                .boolean_region(&clip, BooleanOp::Intersection)
                .unwrap();
            let paths = selected.boundary_paths().unwrap();
            assert_eq!(paths.len(), 1);
            assert_eq!(paths[0].curves().len(), 4);
            let mut roots = Vec::new();
            let mut verticals = Vec::new();
            for curve in paths[0].curves() {
                let endpoints = curve.parameter_endpoints();
                if let Some(branch) = curve.polynomial_branch() {
                    assert_eq!(branch.topology().equation(), &equation);
                    let index = branch.root_index();
                    assert!(
                        branch.shares_source(source.curves()[index].polynomial_branch().unwrap())
                    );
                    assert!(
                        branch.rational_maps().is_empty() && branch.image_transform().is_none()
                    );
                    roots.push(index);
                    for (parameter, x) in [
                        (curve.parameter_domain().start(), q(-1, 2)),
                        (curve.parameter_domain().end(), q(1, 2)),
                    ] {
                        assert_eq!(
                            parameter.compare(&x.clone().into()).unwrap(),
                            Ordering::Equal
                        );
                        let point = curve.point_at(parameter).unwrap();
                        assert!(
                            point
                                .coincides_with(&Point2::new(x, [0, 1][index].into()).into())
                                .unwrap()
                        );
                        let replay = point.definition().unwrap().into_point().unwrap();
                        assert!(replay.coincides_with(&point).unwrap());
                        assert_eq!(
                            selected.classify_point(&replay).unwrap(),
                            RegionPointLocation::Boundary
                        );
                    }
                } else {
                    let start = curve.point_at(endpoints.0).unwrap();
                    let end = curve.point_at(endpoints.1).unwrap();
                    let x = start.affine_form(1.into(), 0.into(), 0.into());
                    let side = if x.compare_to_real(&q(-1, 2)).unwrap() == Ordering::Equal {
                        -1
                    } else {
                        assert_eq!(x.compare_to_real(&q(1, 2)).unwrap(), Ordering::Equal);
                        1
                    };
                    let x = q(side, 2);
                    assert_eq!(
                        end.affine_form(1.into(), 0.into(), -x.clone())
                            .compare_to_real(&Real::zero())
                            .unwrap(),
                        Ordering::Equal
                    );
                    assert!(
                        (start
                            .coincides_with(&Point2::new(x.clone(), 0.into()).into())
                            .unwrap()
                            && end
                                .coincides_with(&Point2::new(x.clone(), 1.into()).into())
                                .unwrap())
                            || (start
                                .coincides_with(&Point2::new(x.clone(), 1.into()).into())
                                .unwrap()
                                && end
                                    .coincides_with(&Point2::new(x, 0.into()).into())
                                    .unwrap())
                    );
                    verticals.push(side);
                }
            }
            roots.sort_unstable();
            verticals.sort_unstable();
            assert_eq!(roots, vec![0, 1]);
            assert_eq!(verticals, vec![-1, 1]);
            let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, fill).unwrap();
            let empty = rebuilt
                .boolean_region(&selected, BooleanOp::Difference)
                .unwrap();
            assert!(empty.is_empty());
            let restored = empty.boolean_region(&rebuilt, BooleanOp::Union).unwrap();
            assert_eq!(restored.boundary_paths().unwrap()[0].curves().len(), 4);
            let remainder = region
                .boolean_region(&restored, BooleanOp::Difference)
                .unwrap();
            assert_eq!(remainder.boundary_paths().unwrap().len(), 2);
            assert!(
                remainder
                    .boolean_region(&restored, BooleanOp::Intersection)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}
