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

/// A branch mapped by a rational map that is an exact similarity, here the
/// scaled reflection (x, y) -> (2y, 2x), keeps that map as its authoritative
/// definition and trims inside the square [-1, 1]^2 through the map's proved
/// affine view, exactly like the same similarity applied directly: the image
/// of y = x^2 is X = Y^2 / 2 with Y = 2x, so the retained source parameters
/// are x in [-1/2, 1/2].
#[test]
fn similarity_certified_rational_images_trim_inside_regions() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &polynomial(&[&[0, 1], &[0], &[-1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let branch = &set.curves()[0];
    let map = hypercurve::RationalMap2::new(
        [polynomial(&[&[0, 2]]), polynomial(&[&[0], &[2]])],
        polynomial(&[&[1]]),
    );
    let mapped = branch.rational_image(&map).unwrap();
    assert_eq!(
        mapped.polynomial_branch().unwrap().rational_maps(),
        std::slice::from_ref(&map)
    );
    let similarity = Similarity2::try_from_real_affine(
        0.into(),
        2.into(),
        2.into(),
        0.into(),
        0.into(),
        0.into(),
    )
    .unwrap();
    let direct = branch.transform_similarity(&similarity).unwrap();
    let square = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(-1, -1), Point2::from_values(1, -1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(1, -1), Point2::from_values(1, 1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(-1, 1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(-1, 1), Point2::from_values(-1, -1)).unwrap(),
        ),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[square], FillRule::NonZero).unwrap();
    for curve in [&mapped, &direct] {
        let fragments = curve.trim_inside_region_with_parameters(&region).unwrap();
        assert_eq!(fragments.len(), 1);
        let (start, end) = fragments[0].represented_parameter_range().unwrap();
        assert_eq!(start.partial_cmp(&q(-1, 2)), Some(Ordering::Equal));
        assert_eq!(end.partial_cmp(&q(1, 2)), Some(Ordering::Equal));
    }
}

/// Exact affine maps that are not similarities: a stretch and a reflecting
/// stretch of the parabola branch y = x² keep their retained rational maps
/// and trim inside the square [-1, 1]² to the source abscissas [-1/2, 1/2],
/// where the mapped abscissa ±2x reaches the square's sides.
#[test]
fn affine_rational_images_trim_inside_regions() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &polynomial(&[&[0, 1], &[0], &[-1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let branch = &set.curves()[0];
    let square = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(-1, -1), Point2::from_values(1, -1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(1, -1), Point2::from_values(1, 1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(1, 1), Point2::from_values(-1, 1)).unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(-1, 1), Point2::from_values(-1, -1)).unwrap(),
        ),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[square], FillRule::NonZero).unwrap();
    let mut crossings = Vec::new();
    for scale in [2, -2] {
        // (u, v) -> (scale u, v).
        let map = hypercurve::RationalMap2::new(
            [polynomial(&[&[0], &[scale]]), polynomial(&[&[0, 1]])],
            polynomial(&[&[1]]),
        );
        let mapped = branch.rational_image(&map).unwrap();
        assert_eq!(
            mapped.polynomial_branch().unwrap().rational_maps(),
            std::slice::from_ref(&map)
        );
        let fragments = mapped.trim_inside_region_with_parameters(&region).unwrap();
        assert_eq!(fragments.len(), 1, "scale {scale}");
        let (start, end) = fragments[0].represented_parameter_range().unwrap();
        assert_eq!(start.partial_cmp(&q(-1, 2)), Some(Ordering::Equal));
        assert_eq!(end.partial_cmp(&q(1, 2)), Some(Ordering::Equal));
        // The vertical line through the image of x = 1/4, traversed upward.
        let side = Curve2::from(
            LineSeg2::try_new(
                Point2::new(q(scale, 4), (-1).into()),
                Point2::new(q(scale, 4), 1.into()),
            )
            .unwrap(),
        );
        let relation = mapped.intersect_curve(&side).unwrap();
        assert_eq!(relation.contacts().len(), 1, "scale {scale}");
        crossings.push(relation.contacts()[0].tangent_cross_sign());
    }
    // A reflecting map reverses the image's orientation against the line.
    let [stretched, reflected] = [crossings[0], crossings[1]];
    assert!(stretched.is_some_and(|sign| sign != hyperreal::RealSign::Zero));
    assert_eq!(
        reflected,
        stretched.map(|sign| match sign {
            hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
            hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
            hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
        })
    );
}

/// A nonlinear polynomial map (x, y) -> (x, y + x²) carries the parabola
/// branch y = x² to Y = 2X². Lines meet that image through the exact pullback
/// of the line into the source chart: Y = 1/2 at (+-1/2, 1/2) and X = 1/4 at
/// (1/4, 1/8), with the original map retained and every contact parameter
/// evaluating to its contact point.
#[test]
fn nonlinear_rational_images_intersect_lines_through_pullbacks() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &polynomial(&[&[0, 1], &[0], &[-1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let branch = &set.curves()[0];
    let map = hypercurve::RationalMap2::new(
        [
            polynomial(&[&[0], &[1]]),
            polynomial(&[&[0, 1], &[0], &[1]]),
        ],
        polynomial(&[&[1]]),
    );
    let mapped = branch.rational_image(&map).unwrap();
    assert_eq!(
        mapped.polynomial_branch().unwrap().rational_maps(),
        std::slice::from_ref(&map)
    );
    let line = |a: (Real, Real), b: (Real, Real)| {
        Curve2::from(LineSeg2::try_new(Point2::new(a.0, a.1), Point2::new(b.0, b.1)).unwrap())
    };
    // The image derivative is the map's Jacobian times the source tangent:
    // d/dx (x, 2x²) = (1, 4x), which is (1, 1) at x = 1/4.
    let derivative = mapped
        .derivatives_at(&hypercurve::CurveParameter2::from(q(1, 4)), 1)
        .unwrap()
        .remove(0)
        .coordinate_image()
        .unwrap();
    assert!(
        derivative
            .coincides_with(&hypercurve::CurvePoint2::from(Point2::from_values(1, 1)))
            .unwrap()
    );
    // Inside [-1, 1]^2 the image Y = 2X^2 keeps |X| <= sqrt(2)/2: one
    // fragment whose source abscissas are exactly -+sqrt(2)/2.
    let square = CurvePath2::try_new(vec![
        line(
            (Real::from(-1), Real::from(-1)),
            (Real::from(1), Real::from(-1)),
        ),
        line(
            (Real::from(1), Real::from(-1)),
            (Real::from(1), Real::from(1)),
        ),
        line(
            (Real::from(1), Real::from(1)),
            (Real::from(-1), Real::from(1)),
        ),
        line(
            (Real::from(-1), Real::from(1)),
            (Real::from(-1), Real::from(-1)),
        ),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths(&[square], FillRule::NonZero).unwrap();
    let fragments = mapped.trim_inside_region_with_parameters(&region).unwrap();
    assert_eq!(fragments.len(), 1);
    // The ends are selected roots without a scalar parameter view; their
    // replayed parameters evaluate to the exact boundary points.
    let root = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
    let range = fragments[0].parameter_range().unwrap();
    for (parameter, x) in [(range.start(), -root.clone()), (range.end(), root)] {
        let target = hypercurve::CurvePoint2::from(Point2::new(x, Real::one()));
        assert!(
            mapped
                .point_at(parameter)
                .unwrap()
                .coincides_with(&target)
                .unwrap()
        );
    }
    let horizontal = line((Real::from(-2), q(1, 2)), (Real::from(2), q(1, 2)));
    let vertical = line((q(1, 4), Real::from(-1)), (q(1, 4), Real::from(3)));
    // The image tangent toward increasing source abscissa is (1, 4X); its
    // cross product with the line direction gives the expected signs.
    use hyperreal::RealSign::{Negative, Positive};
    for (cut, expected) in [
        (
            &horizontal,
            vec![(q(-1, 2), q(1, 2), Positive), (q(1, 2), q(1, 2), Negative)],
        ),
        (&vertical, vec![(q(1, 4), q(1, 8), Positive)]),
    ] {
        let relation = mapped.intersect_curve(cut).unwrap();
        assert!(relation.is_complete());
        assert_eq!(relation.contacts().len(), expected.len());
        for (x, y, sign) in expected {
            let target = hypercurve::CurvePoint2::from(Point2::new(x, y));
            let contact = relation
                .contacts()
                .iter()
                .find(|contact| contact.point().coincides_with(&target).unwrap())
                .expect("expected contact");
            assert_eq!(contact.tangent_cross_sign(), Some(sign));
            assert!(
                mapped
                    .point_at(&contact.first().parameter().unwrap())
                    .unwrap()
                    .coincides_with(&target)
                    .unwrap()
            );
        }
    }
}

/// Two different graphs, y = x² and y = 1/2 - x², meet transversally at
/// (+-1/2, 1/4). Under one shared nonlinear map certified injective on both
/// source domains they meet exactly at the images of those contacts, with
/// the source tangent signs; a reflecting injective map negates the signs.
/// Distinct nonlinear maps keep the typed frontier.
#[test]
fn injective_nonlinear_images_intersect_in_their_shared_source_chart() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    let domain = BivariateCurveDomain {
        first: [(-1).into(), 1.into()],
        second: [(-1).into(), 1.into()],
    };
    let branch = |equation: BivariatePolynomial| {
        let topology = decompose_bivariate_curve(&equation, &domain, Default::default()).unwrap();
        let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
        assert_eq!(set.curves().len(), 1);
        set.curves()[0].clone()
    };
    // y - x² and 2y + 2x² - 1.
    let up = branch(polynomial(&[&[0, 1], &[0], &[-1]]));
    let down = branch(polynomial(&[&[-1, 2], &[0], &[2]]));
    // (x, y) -> (s x, y + x²/4): J = [[s, 0], [x/2, 1]] is injective on
    // [-1, 1]² for s = 1 and s = -1 (which reverses orientation).
    let map = |s: i32| {
        hypercurve::RationalMap2::new(
            [
                polynomial(&[&[0], &[4 * s]]),
                polynomial(&[&[0, 4], &[0], &[1]]),
            ],
            polynomial(&[&[4]]),
        )
    };
    let source = up.intersect_curve(&down).unwrap();
    assert!(source.is_complete());
    assert_eq!(source.contacts().len(), 2);
    let signs = |relation: &hypercurve::CurveIntersectionResult2, s: i32| {
        [-1, 1].map(|x: i32| {
            let target = hypercurve::CurvePoint2::from(Point2::new(
                q(if s == 0 { x } else { s * x }, 2),
                if s == 0 { q(1, 4) } else { q(5, 16) },
            ));
            relation
                .contacts()
                .iter()
                .find(|contact| contact.point().coincides_with(&target).unwrap())
                .expect("expected contact")
                .tangent_cross_sign()
        })
    };
    let expected = signs(&source, 0);
    assert!(
        expected
            .iter()
            .all(|sign| sign.is_some_and(|sign| sign != hyperreal::RealSign::Zero))
    );
    for s in [1, -1] {
        let (first, second) = (
            up.rational_image(&map(s)).unwrap(),
            down.rational_image(&map(s)).unwrap(),
        );
        let relation = first.intersect_curve(&second).unwrap();
        assert!(relation.is_complete());
        assert_eq!(relation.contacts().len(), 2);
        let found = signs(&relation, s);
        let negate = |sign: hyperreal::RealSign| match sign {
            hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
            hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
            hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
        };
        for (found, expected) in found.into_iter().zip(expected) {
            assert_eq!(
                found,
                if s < 0 {
                    expected.map(negate)
                } else {
                    expected
                }
            );
        }
        for contact in relation.contacts() {
            for (curve, location) in [(&first, contact.first()), (&second, contact.second())] {
                let hypercurve::CurvePointLocations2::Locations(locations) =
                    curve.point_locations(contact.point()).unwrap()
                else {
                    panic!("injective image");
                };
                assert_eq!(locations.len(), 1);
                assert!(
                    curve
                        .point_at(&location.parameter().unwrap())
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
            }
        }
    }
    // Different nonlinear maps share no certified source chart.
    let error = up
        .rational_image(&map(1))
        .unwrap()
        .intersect_curve(&down.rational_image(&map(-1)).unwrap())
        .unwrap_err();
    assert!(matches!(
        error,
        hypercurve::ExactCurveError::Blocked(blocker)
            if blocker.reason() == hypercurve::UncertaintyReason::Unsupported
    ));
}

/// The circle x² + y² = 1/2 as two graph branches forms a closed loop. Under
/// an injective nonlinear map its image loop keeps the source orientation,
/// and under a reflecting one reverses it; each image point's derivative is
/// the map's Jacobian times the source tangent.
#[test]
fn injective_nonlinear_images_keep_loop_orientation() {
    let polynomial = |rows: &[&[i32]]| {
        BivariatePolynomial::new(
            rows.iter()
                .map(|r| r.iter().copied().map(Real::from).collect())
                .collect(),
        )
    };
    // 2x² + 2y² - 1.
    let topology = decompose_bivariate_curve(
        &polynomial(&[&[-1, 0, 2], &[0], &[2]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    assert_eq!(set.curves().len(), 2);
    let loop_of = |curves: [Curve2; 2]| {
        // Chain the two branches end to end.
        let [first, second] = curves;
        let second = if second.start().coincides_with(&first.end()).unwrap() {
            second
        } else {
            second.reversed().unwrap()
        };
        CurvePath2::try_new(vec![first, second]).unwrap()
    };
    let source = loop_of([set.curves()[0].clone(), set.curves()[1].clone()]);
    let expected = source.simple_loop_orientation().unwrap();
    // (x, y) -> (s (x + x²/4), y).
    let map = |s: i32| {
        hypercurve::RationalMap2::new(
            [polynomial(&[&[0], &[4 * s], &[s]]), polynomial(&[&[0, 4]])],
            polynomial(&[&[4]]),
        )
    };
    for s in [1, -1] {
        let image = loop_of([0, 1].map(|i| set.curves()[i].rational_image(&map(s)).unwrap()));
        let orientation = image.simple_loop_orientation().unwrap();
        if s > 0 {
            assert_eq!(orientation, expected);
        } else {
            assert_ne!(orientation, expected);
        }
    }
}

#[test]
fn nonlinear_line_pullback_keeps_isolated_contacts() {
    // Y=x²+y² pulls the line Y=0 back to the isolated source point (0,0).
    // On the source branch y=0 the image is the parabola (x,x²).
    let polynomial = |rows: Vec<Vec<i32>>| {
        BivariatePolynomial::new(
            rows.into_iter()
                .map(|r| r.into_iter().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &polynomial(vec![vec![0, 1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let map = hypercurve::RationalMap2::new(
        [
            polynomial(vec![vec![0], vec![1]]),
            polynomial(vec![vec![0, 0, 1], vec![0], vec![1]]),
        ],
        polynomial(vec![vec![1]]),
    );
    let curve = set.curves()[0].rational_image(&map).unwrap();
    let cut = line(Point2::from_values(-2, 0), Point2::from_values(2, 0));
    let relation = curve.intersect_curve(&cut).unwrap();
    assert!(relation.is_complete());
    assert_eq!(relation.contacts().len(), 1);
    let contact = &relation.contacts()[0];
    assert!(
        contact
            .point()
            .coincides_with(&Point2::from_values(0, 0).into())
            .unwrap()
    );
    assert_eq!(
        contact.tangent_cross_sign(),
        Some(hyperreal::RealSign::Zero)
    );
}

#[test]
fn nonlinear_shared_chart_requires_joint_injectivity() {
    // Complex squaring identifies (x,0) and (-x,0). It is injective on each
    // source rectangle separately, but not on their union: the image spans
    // coincide on X in [1,9]. Separate local certificates cannot prove that
    // their source intersections enumerate all image intersections.
    let polynomial = |rows: Vec<Vec<Real>>| BivariatePolynomial::new(rows);
    let map = hypercurve::RationalMap2::new(
        [
            polynomial(vec![
                vec![0.into(), 0.into(), (-1).into()],
                vec![0.into()],
                vec![1.into()],
            ]),
            polynomial(vec![vec![0.into()], vec![0.into(), 2.into()]]),
        ],
        polynomial(vec![vec![1.into()]]),
    );
    let image = |low: i32, high: i32| {
        let domain = BivariateCurveDomain {
            first: [low.into(), high.into()],
            second: [q(-1, 10), q(1, 10)],
        };
        let hypercurve::Classification::Decided(bounds) = hypercurve::Aabb2::from_points(&[
            Point2::new(low.into(), q(-1, 10)),
            Point2::new(high.into(), q(1, 10)),
        ]) else {
            panic!("exact rectangle");
        };
        map.certify_injective_on(&bounds).unwrap();
        let topology = decompose_bivariate_curve(
            &polynomial(vec![vec![0.into(), 1.into()]]),
            &domain,
            Default::default(),
        )
        .unwrap();
        PolynomialCurveSet2::from_topology(topology, Default::default())
            .unwrap()
            .curves()[0]
            .rational_image(&map)
            .unwrap()
    };
    let relation = image(-3, -1).intersect_curve(&image(1, 3));
    assert!(
        matches!(relation, Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::Unsupported)
    );
}

#[test]
fn nonlinear_map_chains_keep_sections_after_restriction_reversal_and_reflection() {
    let poly = |rows: Vec<Vec<i32>>| {
        BivariatePolynomial::new(
            rows.into_iter()
                .map(|row| row.into_iter().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &poly(vec![vec![0, 1], vec![0], vec![-1]]),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let shear = |sign: i32| {
        hypercurve::RationalMap2::new(
            [
                poly(vec![vec![0], vec![sign]]),
                poly(vec![vec![0, sign], vec![0], vec![sign]]),
            ],
            poly(vec![vec![sign]]),
        )
    };
    // y=x² -> Y=3X², retaining both shear definitions and a negative weight.
    let image = set.curves()[0]
        .rational_image(&shear(1))
        .unwrap()
        .rational_image(&shear(-1))
        .unwrap();
    let reflection = Similarity2::try_from_real_affine(
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        2.into(),
        3.into(),
    )
    .unwrap();
    let curve = image
        .transform_similarity(&reflection)
        .unwrap()
        .subcurve(q(-3, 4).into(), q(3, 4).into())
        .unwrap();
    let cut = line(
        Point2::new(q(11, 4), 1.into()),
        Point2::new(q(11, 4), 5.into()),
    );
    for (curve, reversed) in [(curve.clone(), false), (curve.reversed().unwrap(), true)] {
        let relation = curve.intersect_curve(&cut).unwrap();
        assert!(relation.is_complete());
        assert_eq!(relation.contacts().len(), 2);
        for x in [-1, 1] {
            let target =
                hypercurve::CurvePoint2::from(Point2::new(q(11, 4), Real::from(3) + q(x, 2)));
            let contact = relation
                .contacts()
                .iter()
                .find(|contact| contact.point().coincides_with(&target).unwrap())
                .unwrap();
            assert_eq!(
                contact.tangent_cross_sign(),
                Some(if (x > 0) != reversed {
                    hyperreal::RealSign::Positive
                } else {
                    hyperreal::RealSign::Negative
                })
            );
            let parameter = contact.first().parameter().unwrap();
            assert_eq!(parameter.compare(&q(x, 2).into()).unwrap(), Ordering::Equal);
            assert!(
                curve
                    .point_at(&parameter)
                    .unwrap()
                    .coincides_with(&target)
                    .unwrap()
            );
            let replay = contact.point().definition().unwrap().into_point().unwrap();
            assert!(replay.coincides_with(&target).unwrap());
        }
    }
}

#[test]
fn distinct_affine_charts_intersect_with_each_original_parameter() {
    // y=x² and its reflected/translated image Y=1/2-X² meet exactly at
    // (+-1/2,1/4), even though their retained affine maps differ.
    let equation = BivariatePolynomial::new(vec![
        vec![0.into(), 1.into()],
        vec![0.into()],
        vec![(-1).into()],
    ]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let first = &set.curves()[0];
    let transform = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        (-1).into(),
        0.into(),
        q(1, 2),
    )
    .unwrap();
    let second = first
        .transform_similarity(&transform)
        .unwrap()
        .reversed()
        .unwrap();
    for (a, b) in [(first, &second), (&second, first)] {
        let relation = a.intersect_curve(b).unwrap();
        assert!(relation.is_complete());
        assert_eq!(relation.contacts().len(), 2);
        for x in [-1, 1] {
            let target = hypercurve::CurvePoint2::from(Point2::new(q(x, 2), q(1, 4)));
            let contact = relation
                .contacts()
                .iter()
                .find(|contact| contact.point().coincides_with(&target).unwrap())
                .unwrap();
            for (curve, location) in [(a, contact.first()), (b, contact.second())] {
                assert_eq!(
                    location
                        .parameter()
                        .unwrap()
                        .compare(&q(x, 2).into())
                        .unwrap(),
                    Ordering::Equal
                );
                assert!(
                    curve
                        .point_at(&location.parameter().unwrap())
                        .unwrap()
                        .coincides_with(&target)
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn nonlinear_point_location_never_publishes_an_incomplete_preimage_set() {
    // (x,y)->(x²,y) visits (1/4,0) at both x=-1/2 and x=1/2 on y=0.
    // Knowing one construction path is not a completeness certificate.
    let equation = BivariatePolynomial::new(vec![vec![0.into(), 1.into()]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let map = hypercurve::RationalMap2::new(
        [
            BivariatePolynomial::new(vec![vec![0.into()], vec![0.into()], vec![1.into()]]),
            equation,
        ],
        BivariatePolynomial::new(vec![vec![1.into()]]),
    );
    let curve = set.curves()[0].rational_image(&map).unwrap();
    let point = curve.point_at(&q(1, 2).into()).unwrap();
    match curve.point_locations(&point) {
        Ok(hypercurve::CurvePointLocations2::Locations(locations)) => {
            assert_eq!(locations.len(), 2);
            for expected in [q(-1, 2), q(1, 2)] {
                assert!(locations.iter().any(|location| {
                    location
                        .parameter()
                        .unwrap()
                        .compare(&expected.clone().into())
                        .unwrap()
                        == Ordering::Equal
                }));
            }
        }
        Err(hypercurve::ExactCurveError::Blocked(blocker)) => {
            assert_eq!(blocker.reason(), hypercurve::UncertaintyReason::Unsupported)
        }
        other => panic!(
            "expected the complete preimage set or an explicit implementation blocker: {other:?}"
        ),
    }
}

#[test]
fn nonlinear_images_intersect_chords_with_selected_coefficients() {
    // a is the unique root of a^5+a=1 in (0,1). The image of y=x²
    // under (x,y)->(x,y+x²) meets the chord (0,a)--(1,a) exactly once,
    // at x=sqrt(a/2). Neither a nor this contact needs a Real view.
    let alpha = hypercurve::CurvePointDefinition2::PolynomialFiber {
        equation: BivariatePolynomial::new(vec![vec![
            (-1).into(),
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]]),
        parameter: Real::zero().into(),
        root: hypersolve::IsolatedRootInterval {
            lower: 0.into(),
            upper: 1.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let end = hypercurve::CurvePointDefinition2::Similarity {
        source: alpha.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let chord = Curve2::try_line(alpha.clone(), end).unwrap();
    let polynomial = |rows: Vec<Vec<i32>>| {
        BivariatePolynomial::new(
            rows.into_iter()
                .map(|row| row.into_iter().map(Real::from).collect())
                .collect(),
        )
    };
    let topology = decompose_bivariate_curve(
        &polynomial(vec![vec![0, 1], vec![0], vec![-1]]),
        &BivariateCurveDomain {
            first: [0.into(), 1.into()],
            second: [(-1).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let map = hypercurve::RationalMap2::new(
        [
            polynomial(vec![vec![0], vec![1]]),
            polynomial(vec![vec![0, 1], vec![0], vec![1]]),
        ],
        polynomial(vec![vec![1]]),
    );
    let curve = set.curves()[0].rational_image(&map).unwrap();
    let denied =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
            let result = curve.intersect_curve(&chord);
            match result {
                Err(hypercurve::ExactCurveError::Blocked(blocker)) => assert_eq!(
                    blocker.reason(),
                    hypercurve::UncertaintyReason::BudgetExhausted
                ),
                Ok(result) => assert!(!result.is_complete()),
                other => panic!("expected typed exhaustion before a completed result: {other:?}"),
            }
        });
    let exhausted = denied.result.unwrap_err();
    assert_eq!((exhausted.limit, exhausted.used), (0, 0));
    assert!(exhausted.requested > 0);
    let retry = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || curve.intersect_curve(&chord),
    );
    let completed = retry.result.unwrap().unwrap();
    assert!(completed.is_complete());
    assert_eq!(completed.contacts().len(), 1);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let relation = curve.intersect_curve(&chord).unwrap();
            assert!(relation.is_complete());
            assert_eq!(relation.contacts().len(), 1);
            assert!(
                relation.contacts()[0]
                    .point()
                    .coincides_with(completed.contacts()[0].point())
                    .unwrap()
            );
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
    let reflection = Similarity2::try_from_real_affine(
        (-1).into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
    )
    .unwrap();
    for reflected in [false, true] {
        for curve_reversed in [false, true] {
            for chord_reversed in [false, true] {
                let a = if reflected {
                    curve.transform_similarity(&reflection).unwrap()
                } else {
                    curve.clone()
                };
                let b = if reflected {
                    chord.transform_similarity(&reflection).unwrap()
                } else {
                    chord.clone()
                };
                let a = if curve_reversed {
                    a.reversed().unwrap()
                } else {
                    a
                };
                let b = if chord_reversed {
                    b.reversed().unwrap()
                } else {
                    b
                };
                for swapped in [false, true] {
                    let (first, second) = if swapped { (&b, &a) } else { (&a, &b) };
                    let relation = first.intersect_curve(second).unwrap();
                    assert!(relation.is_complete());
                    assert_eq!(relation.contacts().len(), 1);
                    let contact = &relation.contacts()[0];
                    assert_eq!(
                        contact
                            .point()
                            .affine_form(0.into(), 1.into(), 0.into())
                            .compare(&alpha.affine_form(0.into(), 1.into(), 0.into()))
                            .unwrap(),
                        Ordering::Equal
                    );
                    assert_eq!(
                        contact
                            .point()
                            .rational_form(
                                polynomial(vec![vec![0, 1], vec![0], vec![-2]]),
                                polynomial(vec![vec![1]])
                            )
                            .unwrap()
                            .compare_to_real(&Real::zero())
                            .unwrap(),
                        Ordering::Equal
                    );
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if reflected ^ curve_reversed ^ chord_reversed ^ swapped {
                            hyperreal::RealSign::Positive
                        } else {
                            hyperreal::RealSign::Negative
                        })
                    );
                    assert!(contact.is_certified_transverse());
                    for (curve, location) in [(first, contact.first()), (second, contact.second())]
                    {
                        assert!(
                            curve
                                .point_at(&location.parameter().unwrap())
                                .unwrap()
                                .coincides_with(contact.point())
                                .unwrap()
                        );
                    }
                    let replay = contact.point().definition().unwrap().into_point().unwrap();
                    assert!(replay.coincides_with(contact.point()).unwrap());
                    let hypercurve::CurvePointLocations2::Locations(locations) =
                        a.point_locations(&replay).unwrap()
                    else {
                        panic!("injective mapped branch")
                    };
                    assert_eq!(locations.len(), 1);
                }
            }
        }
    }
    // The supporting line still meets the curve, but this short finite chord
    // ends before the contact: a > 1/2 implies sqrt(a/2) > 1/2 > 1/4.
    let short_end = hypercurve::CurvePointDefinition2::Similarity {
        source: alpha.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            q(1, 4),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let short = Curve2::try_line(alpha.clone(), short_end).unwrap();
    let empty = curve.intersect_curve(&short).unwrap();
    assert!(empty.is_complete());
    assert!(empty.contacts().is_empty() && empty.overlaps().is_empty());
    // A genuinely rational chart with a negative original denominator:
    // (X,Y)=(x/(1+x),y/(1+x)), D=-(1+x), gives (1-X)Y=X².
    let height = hypercurve::CurvePointDefinition2::Similarity {
        source: alpha.clone(),
        transform: Similarity2::try_from_real_affine(
            q(1, 4),
            0.into(),
            0.into(),
            q(1, 4),
            0.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let end = hypercurve::CurvePointDefinition2::Similarity {
        source: height.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let rational_chord = Curve2::try_line(height, end).unwrap();
    let rational = set.curves()[0]
        .rational_image(&hypercurve::RationalMap2::new(
            [
                polynomial(vec![vec![0], vec![-1]]),
                polynomial(vec![vec![0, -1]]),
            ],
            polynomial(vec![vec![-1], vec![-1]]),
        ))
        .unwrap();
    let relation = rational.intersect_curve(&rational_chord).unwrap();
    assert!(relation.is_complete());
    assert_eq!(relation.contacts().len(), 1);
    let contact = &relation.contacts()[0];
    assert_eq!(
        contact
            .point()
            .rational_form(
                polynomial(vec![vec![0, 1], vec![0, -1], vec![-1]]),
                polynomial(vec![vec![1]])
            )
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        contact
            .point()
            .affine_form(0.into(), 1.into(), 0.into())
            .compare(&alpha.affine_form(0.into(), q(1, 4), 0.into()))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        contact.tangent_cross_sign(),
        Some(hyperreal::RealSign::Negative)
    );
    for (curve, location) in [
        (&rational, contact.first()),
        (&rational_chord, contact.second()),
    ] {
        assert!(
            curve
                .point_at(&location.parameter().unwrap())
                .unwrap()
                .coincides_with(contact.point())
                .unwrap()
        );
    }
}

#[test]
fn mapped_selected_line_components_never_become_empty_successes() {
    let equation = BivariatePolynomial::new(vec![vec![
        (-1).into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]]);
    let alpha = hypercurve::CurvePointDefinition2::PolynomialFiber {
        equation: equation.clone(),
        parameter: Real::zero().into(),
        root: hypersolve::IsolatedRootInterval {
            lower: 0.into(),
            upper: 1.into(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let endpoint = hypercurve::CurvePointDefinition2::Similarity {
        source: alpha.clone(),
        transform: Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            1.into(),
            0.into(),
        )
        .unwrap(),
    }
    .into_point()
    .unwrap();
    let chord = Curve2::try_line(alpha, endpoint).unwrap();
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [0.into(), 1.into()],
            second: [0.into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
    let map = hypercurve::RationalMap2::new(
        [
            BivariatePolynomial::new(vec![vec![0.into()], vec![4.into()], vec![1.into()]]),
            BivariatePolynomial::new(vec![vec![0.into(), 4.into()]]),
        ],
        BivariatePolynomial::new(vec![vec![4.into()]]),
    );
    let curve = set.curves()[0].rational_image(&map).unwrap();
    // Both finite supports contain the entire segment (0,a)--(1,a).
    // The current zero-resultant route is an implementation gap, not empty.
    match curve.intersect_curve(&chord) {
        Ok(result) => assert!(result.is_complete() && !result.overlaps().is_empty()),
        Err(hypercurve::ExactCurveError::Blocked(blocker)) => {
            assert_eq!(blocker.reason(), hypercurve::UncertaintyReason::Unsupported)
        }
        other => {
            panic!("expected a retained overlap or its explicit implementation blocker: {other:?}")
        }
    }
}
