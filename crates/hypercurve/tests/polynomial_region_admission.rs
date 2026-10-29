//! The original polynomial-boundary region witness must reach one arrangement.

use hypercurve::{
    BooleanOp, Curve2, CurveFamily2, CurveLoopOrientation2, CurveParameterRange2, CurvePath2,
    CurveRegion2, FillRule, LineSeg2, Point2, PolynomialCurveSet2, Real, RegionPointLocation,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn parabola_path() -> CurvePath2 {
    let equation = BivariatePolynomial::new(vec![
        vec![Real::zero(), Real::zero(), Real::one()],
        vec![(-1).into()],
    ]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    assert_eq!(set.curves().len(), 2);
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    CurvePath2::try_new(vec![
        set.curves()[0].restrict_source(range.clone()).unwrap(),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(q(1, 4), q(-1, 2)),
                Point2::new(q(1, 4), q(1, 2)),
            )
            .unwrap(),
        ),
        set.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap()
}

#[test]
fn original_parabola_path_has_exact_loop_orientation() {
    let path = parabola_path();
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
    assert_eq!(
        path.reversed().unwrap().simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Clockwise
    );
}

#[test]
fn original_parabola_path_is_admitted_to_the_shared_region() {
    let path = parabola_path();
    for fill in [FillRule::NonZero, FillRule::EvenOdd] {
        let region =
            CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), fill).unwrap();
        assert_eq!(region.boundary_loops().len(), 1);
        assert_eq!(region.boundary_loops()[0].curves().len(), 3);
    }
}

#[test]
fn polynomial_region_classifies_native_and_retained_points() {
    let path = parabola_path();
    let region =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), FillRule::NonZero)
            .unwrap();
    // Independent filled-set contract: 0 <= x <= 1/4 and y² <= x.
    for (x, y, expected) in [
        (q(1, 8), Real::zero(), RegionPointLocation::Inside),
        (q(1, 8), q(1, 2), RegionPointLocation::Outside),
        (q(1, 4), Real::zero(), RegionPointLocation::Boundary),
        (Real::zero(), Real::zero(), RegionPointLocation::Boundary),
        (q(-1, 8), Real::zero(), RegionPointLocation::Outside),
    ] {
        assert_eq!(
            region.classify_point(&Point2::new(x, y).into()).unwrap(),
            expected
        );
    }
    for index in [0, 2] {
        let point = path.curves()[index].point_at(&q(1, 8).into()).unwrap();
        assert_eq!(
            region.classify_point(&point).unwrap(),
            RegionPointLocation::Boundary
        );
    }
}

fn rectangle(x0: Real, x1: Real, y0: Real, y1: Real) -> CurveRegion2 {
    let points = [
        Point2::new(x0.clone(), y0.clone()),
        Point2::new(x1.clone(), y0),
        Point2::new(x1, y1.clone()),
        Point2::new(x0, y1),
    ];
    let curves = (0..4)
        .map(|i| {
            Curve2::from(LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap())
        })
        .collect();
    CurveRegion2::try_from_boundary_paths(
        &[CurvePath2::try_new(curves).unwrap()],
        FillRule::NonZero,
    )
    .unwrap()
}

#[test]
fn polynomial_region_boolean_reconstruction_keeps_selected_supports() {
    let path = parabola_path();
    let region =
        CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), FillRule::NonZero)
            .unwrap();
    let clip = rectangle(q(1, 16), q(1, 8), (-1).into(), 1.into());
    let selected = region
        .boolean_region(&clip, BooleanOp::Intersection)
        .unwrap();
    let paths = selected.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 4);
    let mut roots = Vec::new();
    for curve in paths[0].curves() {
        if let Some(branch) = curve.polynomial_branch() {
            let source = path.curves()[0].polynomial_branch().unwrap();
            assert_eq!(branch.topology().equation(), source.topology().equation());
            assert_eq!(branch.slab_index(), source.slab_index());
            assert!(branch.rational_maps().is_empty() && branch.image_transform().is_none());
            roots.push(branch.root_index());
            assert_eq!(
                curve
                    .parameter_domain()
                    .start()
                    .compare(&q(1, 16).into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            assert_eq!(
                curve
                    .parameter_domain()
                    .end()
                    .compare(&q(1, 8).into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
        } else {
            assert_eq!(curve.family(), CurveFamily2::Line);
        }
    }
    roots.sort_unstable();
    assert_eq!(roots, vec![0, 1]);
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    let empty = rebuilt
        .boolean_region(&selected, BooleanOp::Difference)
        .unwrap();
    assert!(empty.is_empty());
    let restored = empty.boolean_region(&rebuilt, BooleanOp::Union).unwrap();
    assert_eq!(restored.boundary_paths().unwrap()[0].curves().len(), 4);
}

fn parabola_region() -> CurveRegion2 {
    CurveRegion2::try_from_boundary_paths(&[parabola_path()], FillRule::NonZero).unwrap()
}

#[test]
fn reconstructed_polynomial_path_classifies_native_and_selected_points() {
    let region = parabola_region();
    let path = &region.boundary_paths().unwrap()[0];
    for (x, y, expected) in [
        (
            q(1, 8),
            Real::zero(),
            hypercurve::ContourPointLocation::Inside,
        ),
        (q(1, 8), q(1, 2), hypercurve::ContourPointLocation::Outside),
        (
            Real::zero(),
            Real::zero(),
            hypercurve::ContourPointLocation::Boundary,
        ),
    ] {
        assert_eq!(
            path.classify_point(&Point2::new(x, y).into()).unwrap(),
            expected
        );
    }
    for curve in path
        .curves()
        .iter()
        .filter(|curve| curve.polynomial_branch().is_some())
    {
        let point = curve.point_at(&q(1, 8).into()).unwrap();
        assert_eq!(
            path.classify_point(&point).unwrap(),
            hypercurve::ContourPointLocation::Boundary
        );
    }
}

#[test]
fn polynomial_region_holes_follow_both_fill_rules_and_boolean_operations() {
    let region = parabola_region();
    // This entire rectangle is strictly inside y² <= x: y² <= 1/64 < 1/16 <= x.
    let hole = rectangle(q(1, 16), q(1, 8), q(-1, 8), q(1, 8));
    let punctured = region.boolean_region(&hole, BooleanOp::Difference).unwrap();
    let paths = punctured.boundary_paths().unwrap();
    assert_eq!(paths.len(), 2);
    let orientations: Vec<_> = paths
        .iter()
        .map(|path| path.simple_loop_orientation().unwrap())
        .collect();
    assert!(orientations.contains(&CurveLoopOrientation2::Clockwise));
    assert!(orientations.contains(&CurveLoopOrientation2::Counterclockwise));
    let xor = region.boolean_region(&hole, BooleanOp::Xor).unwrap();
    assert!(
        xor.boolean_region(&punctured, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let reversed: Vec<_> = paths.iter().map(|path| path.reversed().unwrap()).collect();
    for fill in [FillRule::NonZero, FillRule::EvenOdd] {
        for authored in [&paths, &reversed] {
            let rebuilt = CurveRegion2::try_from_boundary_paths(authored, fill).unwrap();
            assert_eq!(rebuilt.boundary_loops().len(), 2);
            for (x, y, expected) in [
                (q(3, 32), Real::zero(), RegionPointLocation::Outside),
                (q(3, 16), Real::zero(), RegionPointLocation::Inside),
                (q(1, 16), Real::zero(), RegionPointLocation::Boundary),
                (q(1, 8), q(3, 8), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    rebuilt.classify_point(&Point2::new(x, y).into()).unwrap(),
                    expected
                );
            }
            assert!(
                rebuilt
                    .boolean_region(&punctured, BooleanOp::Xor)
                    .unwrap()
                    .is_empty()
            );
        }
    }
    let restored = punctured.boolean_region(&hole, BooleanOp::Union).unwrap();
    assert!(
        restored
            .boolean_region(&region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let common = region
        .boolean_region(&hole, BooleanOp::Intersection)
        .unwrap();
    assert!(
        common
            .boolean_region(&hole, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn polynomial_regions_keep_disconnected_components_under_similarity() {
    let region = parabola_region();
    let translated = region
        .transform_affine(
            &1.into(),
            &0.into(),
            &0.into(),
            &1.into(),
            &3.into(),
            &0.into(),
        )
        .unwrap();
    let union = region
        .boolean_region(&translated, BooleanOp::Union)
        .unwrap();
    assert_eq!(union.boundary_loops().len(), 2);
    assert!(
        region
            .boolean_region(&translated, BooleanOp::Intersection)
            .unwrap()
            .is_empty()
    );
    // T(x,y) = (2-3y, -1+3x) is an invertible similarity; image membership
    // follows directly by applying its inverse to y² <= x and 0 <= x <= 1/4.
    let transform = |r: &CurveRegion2| {
        r.transform_affine(
            &0.into(),
            &(-3).into(),
            &3.into(),
            &0.into(),
            &2.into(),
            &(-1).into(),
        )
        .unwrap()
    };
    let image = transform(&union);
    let assembled = transform(&region)
        .boolean_region(&transform(&translated), BooleanOp::Union)
        .unwrap();
    assert_eq!(image.boundary_loops().len(), 2);
    assert!(
        image
            .boolean_region(&assembled, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    for (x, y, expected) in [
        (q(1, 8), Real::zero(), RegionPointLocation::Inside),
        (q(25, 8), Real::zero(), RegionPointLocation::Inside),
        (Real::one(), Real::zero(), RegionPointLocation::Outside),
        (q(1, 8), q(1, 2), RegionPointLocation::Outside),
    ] {
        let point = Point2::new(
            Real::from(2) - Real::from(3) * y,
            Real::from(3) * x - Real::one(),
        );
        assert_eq!(image.classify_point(&point.into()).unwrap(), expected);
    }
    let paths = image.boundary_paths().unwrap();
    let branches: Vec<_> = paths
        .iter()
        .flat_map(|path| path.curves())
        .filter_map(Curve2::polynomial_branch)
        .collect();
    assert_eq!(branches.len(), 4);
    assert!(
        branches
            .iter()
            .all(|branch| branch.rational_maps().is_empty() && branch.image_transform().is_some())
    );
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(
        rebuilt
            .boolean_region(&image, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn polynomial_region_regularizes_contact_but_keeps_nearby_distinct_boundaries() {
    let region = parabola_region();
    let tangent = rectangle(q(1, 4), q(1, 2), (-1).into(), 1.into());
    let empty = region
        .boolean_region(&tangent, BooleanOp::Intersection)
        .unwrap();
    assert!(empty.is_empty());
    let small = q(1, 1 << 20);
    let small = &small * &small;
    let small = &small * &small; // Exactly 2^-80, never a tolerance.
    let start = q(1, 4) - &small;
    let clip = rectangle(start.clone(), q(1, 2), (-1).into(), 1.into());
    let thin = region
        .boolean_region(&clip, BooleanOp::Intersection)
        .unwrap();
    let paths = thin.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 4);
    let mut count = 0;
    for curve in paths[0].curves() {
        if curve.polynomial_branch().is_some() {
            count += 1;
            assert_eq!(
                curve
                    .parameter_domain()
                    .start()
                    .compare(&start.clone().into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            assert_eq!(
                curve
                    .parameter_domain()
                    .end()
                    .compare(&q(1, 4).into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
        }
    }
    assert_eq!(count, 2);
    let middle = q(1, 4) - small * q(1, 2);
    assert_eq!(
        thin.classify_point(&Point2::new(middle, 0.into()).into())
            .unwrap(),
        RegionPointLocation::Inside
    );
    assert!(
        thin.boolean_region(&empty, BooleanOp::Xor)
            .unwrap()
            .boolean_region(&thin, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn polynomial_region_boolean_reuses_generic_quintic_trim_and_replay() {
    // y=f(x)=x^5+x²+5x-5 is strictly increasing on [0,1]. Its zero alpha
    // has degree five. Clipping f(x)<=y<=3 by y>=0 must retain f on [alpha,1].
    let equation = BivariatePolynomial::new(vec![
        vec![5.into(), 1.into()],
        vec![(-5).into()],
        vec![(-1).into()],
        vec![],
        vec![],
        vec![(-1).into()],
    ]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [0.into(), 1.into()],
            second: [(-6).into(), 4.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    assert_eq!(set.curves().len(), 1);
    let points = [
        Point2::from_values(1, 2),
        Point2::from_values(1, 3),
        Point2::from_values(0, 3),
        Point2::from_values(0, -5),
    ];
    let mut curves = vec![set.curves()[0].clone()];
    curves.extend((0..3).map(|i| {
        Curve2::from(LineSeg2::try_new(points[i].clone(), points[i + 1].clone()).unwrap())
    }));
    let region = CurveRegion2::try_from_boundary_paths(
        &[CurvePath2::try_new(curves).unwrap()],
        FillRule::NonZero,
    )
    .unwrap();
    let clip = rectangle(0.into(), 1.into(), 0.into(), 3.into());
    let selected = region
        .boolean_region(&clip, BooleanOp::Intersection)
        .unwrap();
    let roots = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    assert_eq!(roots.len(), 1);
    let alpha: hypercurve::CurveParameter2 = roots[0].clone().into();
    let paths = selected.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 5);
    let branches: Vec<_> = paths[0]
        .curves()
        .iter()
        .filter(|curve| curve.polynomial_branch().is_some())
        .collect();
    assert_eq!(branches.len(), 1);
    let curve = branches[0];
    assert_eq!(
        curve.polynomial_branch().unwrap().topology().equation(),
        &equation
    );
    assert_eq!(
        curve.parameter_domain().start().compare(&alpha).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        curve
            .parameter_domain()
            .end()
            .compare(&Real::one().into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    let point = curve.point_at(&alpha).unwrap();
    let replay = point.definition().unwrap().into_point().unwrap();
    assert_eq!(
        selected.classify_point(&replay).unwrap(),
        RegionPointLocation::Boundary
    );
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    // f(3/4)=-461/1024 and f(7/8)=21415/32768. The first cut is
    // below alpha and retains the horizontal segment; the second is above it.
    assert_eq!(
        hypercurve::CurveParameter2::from(q(3, 4))
            .compare(&alpha)
            .unwrap(),
        std::cmp::Ordering::Less
    );
    assert_eq!(
        alpha.compare(&q(7, 8).into()).unwrap(),
        std::cmp::Ordering::Less
    );
    let further = rebuilt
        .boolean_region(
            &rectangle(q(3, 4), 1.into(), 0.into(), 3.into()),
            BooleanOp::Intersection,
        )
        .unwrap();
    let paths = further.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 5);
    let curve = paths[0]
        .curves()
        .iter()
        .find(|curve| curve.polynomial_branch().is_some())
        .unwrap();
    assert_eq!(
        curve.parameter_domain().start().compare(&alpha).unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        curve.polynomial_branch().unwrap().topology().equation(),
        &equation
    );
    let beyond = further
        .boolean_region(
            &rectangle(q(7, 8), 1.into(), 0.into(), 3.into()),
            BooleanOp::Intersection,
        )
        .unwrap();
    let paths = beyond.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 4);
    let curve = paths[0]
        .curves()
        .iter()
        .find(|curve| curve.polynomial_branch().is_some())
        .unwrap();
    assert_eq!(
        curve
            .parameter_domain()
            .start()
            .compare(&q(7, 8).into())
            .unwrap(),
        std::cmp::Ordering::Equal
    );
    assert_eq!(
        curve.polynomial_branch().unwrap().topology().equation(),
        &equation
    );
}

#[test]
fn polynomial_region_clips_open_authored_curves() {
    let region = parabola_region();
    let line = Curve2::from(
        LineSeg2::try_new(Point2::from_values(-1, 0), Point2::from_values(1, 0)).unwrap(),
    );
    let pieces = line.trim_inside_region(&region).unwrap();
    assert_eq!(pieces.len(), 1);
    assert!(
        pieces[0]
            .start()
            .coincides_with(&Point2::from_values(0, 0).into())
            .unwrap()
    );
    assert!(
        pieces[0]
            .end()
            .coincides_with(&Point2::new(q(1, 4), 0.into()).into())
            .unwrap()
    );
}

#[test]
fn polynomial_region_clips_open_generated_curves() {
    let region = parabola_region();
    let clip = rectangle(q(1, 16), q(1, 8), (-1).into(), 1.into());
    for source in parabola_path()
        .curves()
        .iter()
        .filter(|curve| curve.polynomial_branch().is_some())
    {
        let pieces = source.trim_inside_region_with_parameters(&clip).unwrap();
        assert_eq!(pieces.len(), 1);
        let curve = pieces[0].curve();
        assert_eq!(
            curve
                .parameter_domain()
                .start()
                .compare(&q(1, 16).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            curve
                .parameter_domain()
                .end()
                .compare(&q(1, 8).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            curve.polynomial_branch().unwrap().root_index(),
            source.polynomial_branch().unwrap().root_index()
        );
        assert!(
            curve.trim_inside_region(&region).unwrap()[0]
                .polynomial_branch()
                .is_some()
        );
    }
}

#[test]
fn polynomial_region_mixed_circular_boolean_retains_original_supports() {
    let source = parabola_region();
    let right = Point2::new(q(1, 2), 0.into());
    let left = Point2::new(q(-1, 2), 0.into());
    let center = Point2::from_values(0, 0);
    let circle = CurvePath2::try_new(vec![
        hypercurve::CircularArc2::try_from_center(
            right.clone(),
            left.clone(),
            center.clone(),
            false,
        )
        .unwrap()
        .into(),
        hypercurve::CircularArc2::try_from_center(left, right, center, false)
            .unwrap()
            .into(),
    ])
    .unwrap();
    let disk = CurveRegion2::try_from_boundary_paths(&[circle], FillRule::NonZero).unwrap();
    let common = source
        .boolean_region(&disk, BooleanOp::Intersection)
        .unwrap();
    // The two parabola/circle contacts obey x²+x=1/4; the positive root is alpha.
    let alpha = (Real::from(2).sqrt().unwrap() - Real::one()) * q(1, 2);
    let paths = common.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 5);
    let mut branches = 0;
    let mut circular = 0;
    let mut lines = 0;
    for curve in paths[0].curves() {
        if curve.polynomial_branch().is_some() {
            branches += 1;
            assert_eq!(
                curve
                    .parameter_domain()
                    .start()
                    .compare(&Real::zero().into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            assert_eq!(
                curve
                    .parameter_domain()
                    .end()
                    .compare(&alpha.clone().into())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
        } else if curve.family() == CurveFamily2::Line {
            lines += 1;
            for point in [curve.start(), curve.end()] {
                assert_eq!(
                    point
                        .affine_form(1.into(), 0.into(), 0.into())
                        .compare_to_real(&q(1, 4))
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
            }
        } else {
            circular += 1;
            for point in [curve.start(), curve.end()] {
                assert_eq!(
                    point
                        .rational_form(
                            BivariatePolynomial::new(vec![
                                vec![q(-1, 4), 0.into(), 1.into()],
                                vec![],
                                vec![1.into()]
                            ]),
                            BivariatePolynomial::new(vec![vec![1.into()]])
                        )
                        .unwrap()
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
            }
        }
    }
    assert_eq!((branches, circular, lines), (2, 2, 1));
    for (x, y, expected) in [
        (q(1, 8), Real::zero(), RegionPointLocation::Inside),
        (q(1, 4), q(7, 16), RegionPointLocation::Outside),
        (q(1, 16), q(3, 8), RegionPointLocation::Outside),
        (q(1, 4), q(3, 8), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            common.classify_point(&Point2::new(x, y).into()).unwrap(),
            expected
        );
    }
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(
        rebuilt
            .boolean_region(&common, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn polynomial_region_zero_offset_keeps_sources_and_remaining_gaps_are_typed() {
    let source = parabola_region();
    let zero = source
        .offset(Real::zero(), &hypercurve::OffsetCornerStyle2::Round)
        .unwrap();
    assert!(
        zero.boolean_region(&source, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let paths = zero.boundary_paths().unwrap();
    assert_eq!(
        paths[0]
            .curves()
            .iter()
            .filter(|curve| curve.polynomial_branch().is_some())
            .count(),
        2
    );
    // This same original corner now has a unique exact trim-only chamfer.
    // Its supports and chord setbacks determine the convex retained cap;
    // successful construction alone is not the acceptance criterion.
    use hypercurve::{CurveCornerSolutions2, CurvePoint2, CurvePointRationalForm2};
    use std::cmp::Ordering;
    fn coordinate(point: &CurvePoint2, x: bool) -> CurvePointRationalForm2 {
        point
            .rational_form(
                BivariatePolynomial::new(if x {
                    vec![vec![], vec![Real::one()]]
                } else {
                    vec![vec![Real::zero(), Real::one()]]
                }),
                BivariatePolynomial::new(vec![vec![Real::one()]]),
            )
            .unwrap()
    }
    let original = source.boundary_paths().unwrap();
    let original = original[0].curves();
    let CurveCornerSolutions2::Unique(chamfer) = source
        .chamfer_loop_vertex_by_setbacks(
            0,
            1,
            q(1, 64),
            q(1, 64),
            hypercurve::CurveCornerMode2::TrimOnly,
        )
        .unwrap()
    else {
        panic!("the original convex parabola corner has one trim-only chamfer");
    };
    let edited = chamfer.boundary_paths().unwrap();
    assert_eq!(edited.len(), 1);
    let curves = edited[0].curves();
    assert_eq!(curves.len(), 4);
    let lower = curves
        .iter()
        .position(|curve| {
            curve.polynomial_branch().is_some_and(|branch| {
                branch.has_same_parameterization(original[0].polynomial_branch().unwrap())
            })
        })
        .unwrap();
    let curves: Vec<_> = (0..4).map(|offset| &curves[(lower + offset) % 4]).collect();
    assert!(
        curves[0]
            .start()
            .coincides_with(&original[0].start())
            .unwrap()
    );
    let cut = curves[0].end();
    assert_eq!(curves[1].family(), CurveFamily2::Line);
    assert!(curves[1].start().coincides_with(&cut).unwrap());
    let line_cut = CurvePoint2::from(Point2::new(q(1, 4), q(-31, 64)));
    assert!(curves[1].end().coincides_with(&line_cut).unwrap());
    assert_eq!(curves[2].family(), CurveFamily2::Line);
    assert!(curves[2].start().coincides_with(&line_cut).unwrap());
    assert!(curves[2].end().coincides_with(&original[1].end()).unwrap());
    assert!(
        curves[3]
            .polynomial_branch()
            .unwrap()
            .has_same_parameterization(original[2].polynomial_branch().unwrap())
    );
    assert!(
        curves[3]
            .start()
            .coincides_with(&original[2].start())
            .unwrap()
    );
    assert!(curves[3].end().coincides_with(&original[2].end()).unwrap());
    assert_eq!(
        curves[0]
            .parameter_domain()
            .start()
            .compare(original[0].parameter_domain().start())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        curves[0]
            .parameter_domain()
            .end()
            .compare(original[0].parameter_domain().start())
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        curves[0]
            .parameter_domain()
            .end()
            .compare(original[0].parameter_domain().end())
            .unwrap(),
        Ordering::Less
    );
    let x = coordinate(&cut, true);
    let y = coordinate(&cut, false);
    assert_eq!(
        y.multiply(&y).unwrap().compare(&x).unwrap(),
        Ordering::Equal
    );
    assert_eq!(y.compare_to_real(&Real::zero()).unwrap(), Ordering::Less);
    let corner = original[1].start();
    for point in [&cut, &line_cut] {
        let dx = coordinate(point, true)
            .subtract(&coordinate(&corner, true))
            .unwrap();
        let dy = coordinate(point, false)
            .subtract(&coordinate(&corner, false))
            .unwrap();
        assert_eq!(
            dx.multiply(&dx)
                .unwrap()
                .add(&dy.multiply(&dy).unwrap())
                .unwrap()
                .compare_to_real(&q(1, 4096))
                .unwrap(),
            Ordering::Equal
        );
    }
    assert!(
        chamfer
            .boolean_region(&source, BooleanOp::Difference)
            .unwrap()
            .is_empty()
    );
    let rebuilt =
        CurveRegion2::try_from_boundary_paths(&[edited[0].reversed().unwrap()], FillRule::EvenOdd)
            .unwrap();
    assert!(
        rebuilt
            .boolean_region(&chamfer, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );

    let options = hypercurve::FiniteProjectionOptions::try_new(0.001).unwrap();
    let profiles = source.project_to_finite_profiles(&options).unwrap();
    assert_eq!(profiles.len(), 1);
    assert!(profiles[0].holes().is_empty());
    let ring = profiles[0].material();
    assert!(ring.is_closed());
    assert_eq!(ring.chord_error(), options.chord_error());
    assert_eq!(ring.points().first(), Some(&[0.0, 0.0]));
    assert_eq!(ring.points().last(), ring.points().first());
    let error = Real::try_from(options.chord_error()).unwrap();
    let points: Vec<_> = ring
        .points()
        .iter()
        .map(|point| point.map(|value| Real::try_from(value).unwrap()))
        .collect();
    let mut caps = 0;
    for pair in points.windows(2) {
        let [[ax, ay], [bx, by]] = pair else {
            unreachable!()
        };
        if ax == &q(1, 4) && bx == &q(1, 4) {
            assert_eq!(ay, &q(-1, 2));
            assert_eq!(by, &q(1, 2));
            caps += 1;
            continue;
        }
        assert_eq!(
            hyperlimit::compare_reals(by, ay, hyperlimit::PredicatePolicy::STRICT).value(),
            Some(Ordering::Less)
        );
        // Independently parametrize the exact boundary as (y^2, y). The
        // secant's horizontal deviation is at most (by-ay)^2/4 over its
        // whole interval. Add the actual binary64 endpoints' horizontal
        // residuals to bound the emitted chord without sampling the arc.
        let dy = by - ay;
        let bound =
            (&dy * &dy / Real::from(4)).unwrap() + (ax - ay * ay).abs() + (bx - by * by).abs();
        assert!(matches!(
            hyperlimit::compare_reals(&bound, &error, hyperlimit::PredicatePolicy::STRICT).value(),
            Some(Ordering::Less | Ordering::Equal)
        ));
    }
    assert_eq!(caps, 1);
    let retained = source.boundary_paths().unwrap();
    for index in [0, 2] {
        assert!(
            retained[0].curves()[index]
                .polynomial_branch()
                .unwrap()
                .has_same_parameterization(original[index].polynomial_branch().unwrap())
        );
    }
    assert!(
        source
            .boolean_region(&zero, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );

    for error in [
        source
            .offset(q(1, 64), &hypercurve::OffsetCornerStyle2::Round)
            .unwrap_err(),
        source
            .fillet_loop_vertex(
                0,
                1,
                &hypercurve::CurveFillet2::new(q(1, 64)),
                hypercurve::CurveCornerMode2::TrimOnly,
            )
            .unwrap_err(),
    ] {
        assert!(
            matches!(&error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason() == hypercurve::UncertaintyReason::Unsupported),
            "{error:?}"
        );
    }
}

#[test]
fn polynomial_region_budget_retry_and_policy_transitions_keep_exact_results() {
    use hypersolve::work_budget::{self, WorkBudget};
    let source = parabola_region();
    let clip = rectangle(q(1, 16), q(1, 8), (-1).into(), 1.into());
    let denied = work_budget::with_budget(WorkBudget::ZERO, || {
        assert!(
            matches!(source.boolean_region(&clip, BooleanOp::Intersection), Err(hypercurve::ExactCurveError::Blocked(blocker)) if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted)
        );
    });
    let reached = denied.result.unwrap_err();
    assert_eq!((reached.limit, reached.used), (0, 0));
    assert!(reached.requested > 0);
    let retry = work_budget::with_budget(
        WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || {
            source
                .boolean_region(&clip, BooleanOp::Intersection)
                .unwrap()
        },
    );
    let expected = retry.result.unwrap();
    assert_eq!(expected.boundary_paths().unwrap()[0].curves().len(), 4);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            source
                .boolean_region(&clip, BooleanOp::Intersection)
                .unwrap()
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        let result = outcome.certified().unwrap();
        assert!(
            result
                .boolean_region(&expected, BooleanOp::Xor)
                .unwrap()
                .is_empty()
        );
    }
}

fn singular_branches(x_degree: usize) -> PolynomialCurveSet2 {
    let mut rows = vec![vec![Real::zero(), Real::zero(), Real::one()]];
    rows.resize(x_degree + 1, Vec::new());
    rows[x_degree] = vec![(-1).into()];
    let topology = decompose_bivariate_curve(
        &BivariatePolynomial::new(rows),
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

fn right_cap(lower: Curve2, upper: Curve2) -> CurvePath2 {
    CurvePath2::try_new(vec![
        lower,
        LineSeg2::try_new(Point2::from_values(1, -1), Point2::from_values(1, 1))
            .unwrap()
            .into(),
        upper.reversed().unwrap(),
    ])
    .unwrap()
}

#[test]
fn polynomial_region_cusp_keeps_one_sided_branch_incidence() {
    // The cusp bounds the exact set 0 <= x <= 1, y² <= x³.
    let set = singular_branches(3);
    assert_eq!(set.curves().len(), 2);
    let path = right_cap(set.curves()[0].clone(), set.curves()[1].clone());
    let region = CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap();
    assert_eq!(region.boundary_loops().len(), 1);
    for (x, y, expected) in [
        (Real::zero(), Real::zero(), RegionPointLocation::Boundary),
        (q(1, 2), Real::zero(), RegionPointLocation::Inside),
        (q(1, 4), q(1, 4), RegionPointLocation::Outside),
        (Real::one(), Real::zero(), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            region.classify_point(&Point2::new(x, y).into()).unwrap(),
            expected
        );
    }
    let clipped = region
        .boolean_region(
            &rectangle(q(1, 4), q(1, 2), (-2).into(), 2.into()),
            BooleanOp::Intersection,
        )
        .unwrap();
    let paths = clipped.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 4);
    for curve in paths[0]
        .curves()
        .iter()
        .filter(|curve| curve.polynomial_branch().is_some())
    {
        assert_eq!(
            curve
                .parameter_domain()
                .start()
                .compare(&q(1, 4).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            curve
                .parameter_domain()
                .end()
                .compare(&q(1, 2).into())
                .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            curve.polynomial_branch().unwrap().topology().equation(),
            set.topology().equation()
        );
    }
    let rebuilt = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(
        rebuilt
            .boolean_region(&clipped, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn polynomial_region_node_keeps_two_material_lobes_at_shared_vertex() {
    let set = singular_branches(2);
    assert_eq!(set.curves().len(), 4);
    let mut left = Vec::new();
    let mut right = Vec::new();
    for curve in set.curves() {
        if curve
            .parameter_domain()
            .start()
            .compare(&Real::zero().into())
            .unwrap()
            .is_lt()
        {
            left.push(curve.clone());
        } else {
            right.push(curve.clone());
        }
    }
    left.sort_by_key(|curve| curve.polynomial_branch().unwrap().root_index());
    right.sort_by_key(|curve| curve.polynomial_branch().unwrap().root_index());
    let left = CurvePath2::try_new(vec![
        left[0].clone(),
        left[1].reversed().unwrap(),
        LineSeg2::try_new(Point2::from_values(-1, 1), Point2::from_values(-1, -1))
            .unwrap()
            .into(),
    ])
    .unwrap();
    let right = right_cap(right[0].clone(), right[1].clone());
    for fill in [FillRule::NonZero, FillRule::EvenOdd] {
        let region =
            CurveRegion2::try_from_boundary_paths(&[left.clone(), right.clone()], fill).unwrap();
        assert_eq!(region.boundary_loops().len(), 2);
        for (x, y, expected) in [
            (Real::zero(), Real::zero(), RegionPointLocation::Boundary),
            (q(-1, 2), Real::zero(), RegionPointLocation::Inside),
            (q(1, 2), Real::zero(), RegionPointLocation::Inside),
            (Real::zero(), q(1, 2), RegionPointLocation::Outside),
        ] {
            assert_eq!(
                region.classify_point(&Point2::new(x, y).into()).unwrap(),
                expected
            );
        }
        let clipped = region
            .boolean_region(
                &rectangle(q(-1, 2), q(1, 2), (-2).into(), 2.into()),
                BooleanOp::Intersection,
            )
            .unwrap();
        assert_eq!(clipped.boundary_loops().len(), 2);
        assert!(
            clipped
                .boolean_region(&region, BooleanOp::Difference)
                .unwrap()
                .is_empty()
        );
    }
}

// An elliptic support prevents the frontier from being reduced to a catalog
// of rationally parameterizable curves. On this selected cap the filled set
// is -1 <= x <= 0 and y^2 <= 1 + x^3, with rational corner coordinates.
fn elliptic_chamfer_admission_source() -> CurveRegion2 {
    let equation = BivariatePolynomial::new(vec![
        vec![(-1).into(), 0.into(), 1.into()],
        vec![],
        vec![],
        vec![(-1).into()],
    ]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-2).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    assert_eq!(set.curves().len(), 2);
    let range = CurveParameterRange2::try_new(Real::from(-1).into(), Real::zero().into()).unwrap();
    let path = CurvePath2::try_new(vec![
        set.curves()[0].restrict_source(range.clone()).unwrap(),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(0.into(), (-1).into()),
                Point2::new(0.into(), 1.into()),
            )
            .unwrap(),
        ),
        set.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap();
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap()
}

#[test]
fn algebraic_corner_chamfer_preserves_the_region_through_reconstruction_and_boolean() {
    use hypercurve::{
        CurveCornerMode2, CurveCornerSolutions2, CurvePoint2, CurvePointRationalForm2,
    };
    use std::cmp::Ordering;
    fn coordinate(point: &CurvePoint2, x: bool) -> CurvePointRationalForm2 {
        point
            .rational_form(
                BivariatePolynomial::new(if x {
                    vec![vec![], vec![Real::one()]]
                } else {
                    vec![vec![Real::zero(), Real::one()]]
                }),
                BivariatePolynomial::new(vec![vec![Real::one()]]),
            )
            .unwrap()
    }
    fn setback(point: &CurvePoint2, corner: &CurvePoint2) {
        let dx = coordinate(point, true)
            .subtract(&coordinate(corner, true))
            .unwrap();
        let dy = coordinate(point, false)
            .subtract(&coordinate(corner, false))
            .unwrap();
        assert_eq!(
            dx.multiply(&dx)
                .unwrap()
                .add(&dy.multiply(&dy).unwrap())
                .unwrap()
                .compare_to_real(&q(1, 4096))
                .unwrap(),
            Ordering::Equal
        );
    }
    let source = elliptic_chamfer_admission_source()
        .boolean_region(
            &rectangle((-2).into(), 1.into(), (-2).into(), q(1, 2)),
            BooleanOp::Intersection,
        )
        .unwrap();
    let original = source.boundary_paths().unwrap();
    assert_eq!(original.len(), 1);
    let vertices: Vec<_> = original[0]
        .curves()
        .iter()
        .enumerate()
        .filter_map(|(index, curve)| {
            (curve.polynomial_branch().is_some()
                && coordinate(&curve.start(), false)
                    .compare_to_real(&q(1, 2))
                    .unwrap()
                    == Ordering::Equal)
                .then_some(index)
        })
        .collect();
    assert_eq!(vertices.len(), 1);
    let original_upper = &original[0].curves()[vertices[0]];
    let corner = original_upper.start();
    let x = coordinate(&corner, true);
    assert_eq!(x.compare_to_real(&Real::zero()).unwrap(), Ordering::Less);
    assert_eq!(
        x.multiply(&x)
            .unwrap()
            .multiply(&x)
            .unwrap()
            .compare_to_real(&q(-3, 4))
            .unwrap(),
        Ordering::Equal
    );
    let CurveCornerSolutions2::Unique(result) = source
        .chamfer_loop_vertex_by_setbacks(
            0,
            vertices[0],
            q(1, 64),
            q(1, 64),
            CurveCornerMode2::TrimOnly,
        )
        .unwrap()
    else {
        panic!("one exact finite trim solution")
    };
    let paths = result.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    let path = &paths[0];
    assert_eq!(path.curves().len(), 5);
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        CurveLoopOrientation2::Counterclockwise
    );
    for (index, curve) in path.curves().iter().enumerate() {
        assert!(
            curve
                .end()
                .coincides_with(&path.curves()[(index + 1) % 5].start())
                .unwrap()
        );
    }
    let branches: Vec<_> = path
        .curves()
        .iter()
        .enumerate()
        .filter(|(_, curve)| curve.polynomial_branch().is_some())
        .collect();
    assert_eq!(branches.len(), 2);
    let (upper_index, upper) = *branches
        .iter()
        .find(|(_, curve)| curve.polynomial_branch().unwrap().root_index() == 1)
        .unwrap();
    let (_, lower) = *branches
        .iter()
        .find(|(_, curve)| curve.polynomial_branch().unwrap().root_index() == 0)
        .unwrap();
    for curve in [upper, lower] {
        assert!(
            curve.polynomial_branch().unwrap().topology()
                == original_upper.polynomial_branch().unwrap().topology()
        );
    }
    assert!(
        upper
            .polynomial_branch()
            .unwrap()
            .has_same_parameterization(original_upper.polynomial_branch().unwrap())
    );
    assert_eq!(
        upper
            .parameter_domain()
            .start()
            .compare(&Real::from(-1).into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        upper
            .parameter_domain()
            .end()
            .compare(original_upper.parameter_domain().end())
            .unwrap(),
        Ordering::Less
    );
    assert_eq!(
        lower
            .parameter_domain()
            .start()
            .compare(&Real::from(-1).into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        lower
            .parameter_domain()
            .end()
            .compare(&Real::zero().into())
            .unwrap(),
        Ordering::Equal
    );
    let upper_cut = upper.start();
    assert_eq!(
        coordinate(&upper_cut, true)
            .compare(&coordinate(&corner, true))
            .unwrap(),
        Ordering::Less
    );
    assert_eq!(
        coordinate(&upper_cut, false)
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Less
    );
    setback(&upper_cut, &corner);
    let chord = &path.curves()[(upper_index + 4) % 5];
    assert_eq!(chord.family(), CurveFamily2::Line);
    assert!(chord.end().coincides_with(&upper_cut).unwrap());
    let horizontal_cut = chord.start();
    assert_eq!(
        coordinate(&horizontal_cut, false)
            .compare_to_real(&q(1, 2))
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        coordinate(&horizontal_cut, true)
            .compare(&coordinate(&corner, true))
            .unwrap(),
        Ordering::Greater
    );
    setback(&horizontal_cut, &corner);
    let top = &path.curves()[(upper_index + 3) % 5];
    let right = &path.curves()[(upper_index + 2) % 5];
    assert_eq!(top.family(), CurveFamily2::Line);
    assert_eq!(right.family(), CurveFamily2::Line);
    assert!(
        top.start()
            .coincides_with(&Point2::new(0.into(), q(1, 2)).into())
            .unwrap()
    );
    assert!(top.end().coincides_with(&horizontal_cut).unwrap());
    assert!(
        right
            .start()
            .coincides_with(&Point2::from_values(0, -1).into())
            .unwrap()
    );
    assert!(right.end().coincides_with(&top.start()).unwrap());
    // These five exact oriented pieces bound the original convex set
    // {-1<=x<=0, y^2<=1+x^3, y<=1/2} clipped by the new chord half-plane.
    // The upper graph is strictly concave on (-1,0), so the inward chord
    // removes precisely the corner between the two certified setback cuts.
    assert_eq!(
        result.classify_point(&corner).unwrap(),
        RegionPointLocation::Outside
    );
    assert_eq!(
        result.classify_point(&upper_cut).unwrap(),
        RegionPointLocation::Boundary
    );
    assert_eq!(
        result
            .classify_point(&Point2::new(q(-1, 2), 0.into()).into())
            .unwrap(),
        RegionPointLocation::Inside
    );
    let rebuilt =
        CurveRegion2::try_from_boundary_paths(&[path.reversed().unwrap()], FillRule::EvenOdd)
            .unwrap();
    // Both chamfer cuts are strictly left of -3/4. This independent later
    // clip removes that whole edit and leaves exactly one lower polynomial
    // graph over [-3/4,0], a top at y=1/2 and two vertical closing lines.
    for point in [&upper_cut, &horizontal_cut] {
        assert_eq!(
            coordinate(point, true).compare_to_real(&q(-3, 4)).unwrap(),
            Ordering::Less
        );
    }
    let further = rebuilt
        .boolean_region(
            &rectangle(q(-3, 4), 0.into(), (-2).into(), q(1, 2)),
            BooleanOp::Intersection,
        )
        .unwrap();
    let paths = further.boundary_paths().unwrap();
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 4);
    let branches: Vec<_> = paths[0]
        .curves()
        .iter()
        .filter(|curve| curve.polynomial_branch().is_some())
        .collect();
    assert_eq!(branches.len(), 1);
    let branch = branches[0];
    assert_eq!(branch.polynomial_branch().unwrap().root_index(), 0);
    assert!(
        branch
            .polynomial_branch()
            .unwrap()
            .has_same_parameterization(lower.polynomial_branch().unwrap())
    );
    assert_eq!(
        branch
            .parameter_domain()
            .start()
            .compare(&q(-3, 4).into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        branch
            .parameter_domain()
            .end()
            .compare(&Real::zero().into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        further
            .classify_point(&branch.point_at(&q(-1, 2).into()).unwrap())
            .unwrap(),
        RegionPointLocation::Boundary
    );
    assert_eq!(
        further
            .classify_point(&Point2::new(q(-1, 2), 0.into()).into())
            .unwrap(),
        RegionPointLocation::Inside
    );
}
