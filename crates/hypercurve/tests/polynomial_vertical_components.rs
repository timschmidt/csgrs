//! Native consumption of the solver's whole vertical intervals.
use hypercurve::{CurveParameter2, CurvePoint2, Point2, PolynomialCurveSet2, Real};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

#[test]
fn vertical_line_remains_native_through_point_queries() {
    let equation = BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    let set = PolynomialCurveSet2::from_topology(topology, Default::default())
        .expect("a certified vertical interval must be a native exact curve");
    assert_eq!(set.curves().len(), 1);
    assert!(set.isolated_points().is_empty());
    let curve = &set.curves()[0];
    for y in [-2, -1, 0, 1, 2] {
        let parameter = CurveParameter2::from(Real::from(y));
        let expected = CurvePoint2::from(Point2::new(Real::zero(), Real::from(y)));
        assert!(
            curve
                .point_at(&parameter)
                .unwrap()
                .coincides_with(&expected)
                .unwrap()
        );
    }
}

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn set(equation: &BivariatePolynomial) -> PolynomialCurveSet2 {
    PolynomialCurveSet2::from_topology(
        decompose_bivariate_curve(
            equation,
            &BivariateCurveDomain {
                first: [(-2).into(), 2.into()],
                second: [(-2).into(), 2.into()],
            },
            hypersolve::bivariate_topology::BivariateTopologyConfig {
                max_axis_degree: 16,
                ..Default::default()
            },
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap()
}
fn power(p: &BivariatePolynomial, exponent: usize) -> BivariatePolynomial {
    (0..exponent).fold(polynomial(&[&[1]]), |a, _| {
        hypersolve::bivariate_arithmetic::bivariate_multiply(&a, p)
    })
}

#[test]
fn mixed_axes_keep_both_charts_and_defining_powers() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let equation = bivariate_multiply(
        &power(&polynomial(&[&[], &[1]]), 2),
        &power(&polynomial(&[&[0, 1]]), 3),
    );
    let curves = set(&equation);
    assert_eq!(curves.curves().len(), 4); // two half axes in each chart
    assert!(curves.isolated_points().is_empty());
    assert!(curves.graph_branch(0, 0).is_some());
    assert!(curves.graph_branch(0, 1).is_none());
    for curve in curves.curves() {
        let branch = curve.polynomial_branch().unwrap();
        let vertical = branch.image_transform().is_some();
        assert_eq!(branch.defining_exponent(), if vertical { 2 } else { 3 });
        let t = if branch.slab_index() == 0 { -1 } else { 1 };
        let point = curve.point_at(&Real::from(t).into()).unwrap();
        let expected = if vertical {
            Point2::from_values(0, t)
        } else {
            Point2::from_values(t, 0)
        };
        assert!(point.coincides_with(&expected.into()).unwrap());
        assert!(
            point
                .rational_form(equation.clone(), polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap()
                .is_eq()
        );
        assert!(branch.certifies_polynomial_identity(&equation));
        let axis = if vertical {
            polynomial(&[&[], &[1]])
        } else {
            polynomial(&[&[0, 1]])
        };
        assert!(branch.certifies_polynomial_identity(&axis));
        let other = if vertical {
            polynomial(&[&[0, 1]])
        } else {
            polynomial(&[&[], &[1]])
        };
        assert!(!branch.certifies_polynomial_identity(&other));
    }
}

#[test]
fn residual_isolated_point_on_a_column_is_not_isolated_in_the_union() {
    // x(x²+y²): the only residual point is already on the column.
    let equation = polynomial(&[&[], &[0, 0, 1], &[], &[1]]);
    let curves = set(&equation);
    assert_eq!(curves.curves().len(), 2); // exchanged chart splits at the origin
    assert!(curves.isolated_points().is_empty());
    assert!(curves.graph_branch(0, 0).is_none());
    for curve in curves.curves() {
        for endpoint in [curve.start(), curve.end()] {
            assert!(
                endpoint
                    .affine_form(1.into(), 0.into(), 0.into())
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
        }
    }
}

#[test]
fn selected_quintic_columns_keep_points_jets_and_sections_exact() {
    use hypercurve::LineSeg2;
    // Strictly increasing quintic, with exactly one real root alpha in (-1,0).
    let p = polynomial(&[&[5], &[5], &[], &[], &[], &[1]]);
    let curves = set(&power(&p, 3));
    assert_eq!(curves.curves().len(), 1);
    let curve = &curves.curves()[0];
    assert_eq!(curve.polynomial_branch().unwrap().defining_exponent(), 3);
    assert!(curves.graph_branch(0, 0).is_none());
    for y in [-2, -1, 0, 1, 2] {
        let point = curve.point_at(&Real::from(y).into()).unwrap();
        assert!(
            point
                .rational_form(p.clone(), polynomial(&[&[1]]))
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap()
                .is_eq()
        );
        assert!(
            point
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::from(y))
                .unwrap()
                .is_eq()
        );
    }
    for (jet, expected) in curve
        .derivatives_at(&Real::zero().into(), 2)
        .unwrap()
        .into_iter()
        .zip([Point2::from_values(0, 1), Point2::from_values(0, 0)])
    {
        assert!(
            jet.coordinate_image()
                .unwrap()
                .coincides_with(&expected.into())
                .unwrap()
        );
    }
    let line: hypercurve::Curve2 =
        LineSeg2::try_new(Point2::from_values(-2, 0), Point2::from_values(2, 0))
            .unwrap()
            .into();
    let intersection = curve.intersect_curve(&line).unwrap();
    assert!(intersection.is_complete());
    assert_eq!(intersection.contacts().len(), 1);
    assert!(intersection.overlaps().is_empty());
    assert!(intersection.contacts()[0].is_certified_transverse());
    let split = curve.intersection_topology(&line).unwrap();
    assert_eq!(split.first().len(), 2);
    for piece in split.first() {
        assert!(piece.intersect_curve(&line).unwrap().is_complete());
        assert_eq!(piece.polynomial_branch().unwrap().defining_exponent(), 3);
    }
}

fn rectangle() -> hypercurve::CurveRegion2 {
    use hypercurve::{CurvePath2, CurveRegion2, FillRule, LineSeg2};
    let columns = set(&polynomial(&[&[-1], &[], &[1]]));
    assert_eq!(columns.curves().len(), 2);
    let line = |a, b, c, d| {
        LineSeg2::try_new(Point2::from_values(a, b), Point2::from_values(c, d))
            .unwrap()
            .into()
    };
    let path = CurvePath2::try_new(vec![
        line(-1, -1, 1, -1),
        columns.curves()[1]
            .subcurve(Real::from(-1).into(), Real::one().into())
            .unwrap(),
        line(1, 1, -1, 1),
        columns.curves()[0]
            .subcurve(Real::from(-1).into(), Real::one().into())
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap()
}
fn check_rectangle(region: &hypercurve::CurveRegion2) {
    use hypercurve::{BooleanOp, CurveRegion2, FillRule, RegionPointLocation};
    assert_eq!(region.boundary_paths().unwrap().len(), 1);
    let original = region.boundary_paths().unwrap();
    assert_eq!(
        original[0]
            .curves()
            .iter()
            .filter(|c| c.polynomial_branch().is_some())
            .count(),
        2
    );
    // Independent whole-image expectation for both retained vertical sides,
    // including after cold replay; membership samples alone are insufficient.
    for curve in original[0].curves() {
        if let Some(branch) = curve.polynomial_branch() {
            assert!(branch.certifies_polynomial_identity(&polynomial(&[&[-1], &[], &[1]])));
        }
    }
    for (x, y, expected) in [
        (0, 0, RegionPointLocation::Inside),
        (1, 0, RegionPointLocation::Boundary),
        (0, 2, RegionPointLocation::Outside),
    ] {
        assert_eq!(
            region
                .classify_point(&Point2::from_values(x, y).into())
                .unwrap(),
            expected
        );
    }
    let rebuilt = CurveRegion2::try_from_boundary_paths(&original, FillRule::EvenOdd).unwrap();
    assert!(
        rebuilt
            .boolean_region(region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let empty = rebuilt
        .boolean_region(region, BooleanOp::Difference)
        .unwrap();
    assert!(empty.is_empty());
    assert!(
        empty
            .boolean_region(&rebuilt, BooleanOp::Union)
            .unwrap()
            .boolean_region(region, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
}
#[test]
fn vertical_charts_reenter_region_arrangement_and_reconstruction() {
    check_rectangle(&rectangle());
}

#[cfg(feature = "svg")]
#[test]
fn exchanged_charts_replay_cold_and_compose() {
    use hypercurve::{
        CurvePath2, Similarity2, SvgGeometry2, export_svg_document, import_svg_document,
    };
    const INPUT: &str = "HYPERCURVE_VERTICAL_REPLAY_INPUT";
    fn consume(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        check_rectangle(geometry.region());
        let curve = &geometry.paths()[0].curves()[0];
        assert_eq!(curve.polynomial_branch().unwrap().defining_exponent(), 3);
        for t in [-1, 0, 1] {
            let point = curve.point_at(&Real::from(t).into()).unwrap();
            // Similarity maps (alpha,t) to (2t+3,2alpha-1).
            assert!(
                point
                    .affine_form(1.into(), 0.into(), 0.into())
                    .compare_to_real(&Real::from(2 * t + 3))
                    .unwrap()
                    .is_eq()
            );
            // 32 p((y+1)/2) = (y+1)^5 + 80(y+1) + 160.
            let relation = polynomial(&[&[241, 85, 10, 10, 5, 1]]);
            // Use the retained chart instead of materializing alpha's image.
            let branch = curve.polynomial_branch().unwrap();
            assert!(branch.certifies_polynomial_identity(&relation));
            assert!(
                point
                    .rational_form(relation, polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
            assert!(!branch.certifies_polynomial_identity(&polynomial(&[&[-1, 2]])));
        }
        let (left, right) = curve.split_at(Real::zero().into()).unwrap();
        assert!(left.end().coincides_with(&right.start()).unwrap());
        assert_eq!(left.polynomial_branch().unwrap().defining_exponent(), 3);
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        consume(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let p = polynomial(&[&[5], &[5], &[], &[], &[], &[1]]);
    let columns = set(&power(&p, 3));
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        2.into(),
        2.into(),
        0.into(),
        3.into(),
        (-1).into(),
    )
    .unwrap();
    let curve = columns.curves()[0]
        .subcurve(Real::from(-1).into(), Real::one().into())
        .unwrap()
        .reversed()
        .unwrap()
        .transform_similarity(&transform)
        .unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        rectangle(),
        vec![],
        vec![CurvePath2::try_new(vec![curve]).unwrap()],
    ))
    .unwrap();
    consume(&text);
    let path = std::env::temp_dir().join(format!("hypercurve-vertical-{}.svg", std::process::id()));
    std::fs::write(&path, &text).unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "exchanged_charts_replay_cold_and_compose",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        result.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}

#[test]
fn admission_exhaustion_and_cancellation_do_not_poison_retry() {
    use hypersolve::work_budget::{self, CancellationToken, WorkBudget, WorkKind};
    let equation = polynomial(&[&[], &[1]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-1).into(), 1.into()],
        },
        Default::default(),
    )
    .unwrap();
    let admit = || PolynomialCurveSet2::from_topology(topology.clone(), Default::default());
    let denied = work_budget::with_budget(WorkBudget::ZERO, || {
        let error = admit().unwrap_err();
        assert!(
            matches!(error,hypercurve::ExactCurveError::Blocked(b) if b.operation()==hypercurve::CurveOperation2::Construction && b.reason()==hypercurve::UncertaintyReason::BudgetExhausted)
        );
    });
    let error = denied.result.unwrap_err();
    assert_eq!(error.kind, WorkKind::AlgebraicPromotion);
    assert_eq!(error.limit, 0);
    assert_eq!(error.used, 0);
    assert_eq!(error.requested, 1);
    let cancellation = CancellationToken::new();
    cancellation.cancel();
    let cancelled =
        work_budget::with_cancellable_budget(WorkBudget::UNLIMITED, &cancellation, || {
            assert!(admit().is_err())
        });
    assert_eq!(cancelled.result.unwrap_err().kind, WorkKind::Cancellation);
    let retry = admit().unwrap();
    assert_eq!(retry.curves().len(), 1);
    assert!(retry.isolated_points().is_empty());
}

#[test]
fn singular_contacts_boundary_columns_and_close_roots_keep_complete_native_sets() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let x = polynomial(&[&[], &[1]]);
    for (residual, count) in [
        (polynomial(&[&[0, 0, 1], &[], &[-1]]), 6),      // node
        (polynomial(&[&[0, 0, 1], &[], &[], &[-1]]), 4), // cusp
        (polynomial(&[&[0, 1], &[], &[-1]]), 4),         // parabola crossing
        (polynomial(&[&[0, 0, 1], &[-1]]), 6),           // tangent to the column
    ] {
        let equation = bivariate_multiply(&x, &residual);
        let curves = set(&equation);
        assert_eq!(curves.curves().len(), count, "{residual:?}");
        assert!(curves.isolated_points().is_empty());
        for curve in curves.curves() {
            let branch = curve.polynomial_branch().unwrap();
            assert!(branch.certifies_polynomial_identity(&equation));
            for point in [curve.start(), curve.end()] {
                assert!(
                    point
                        .rational_form(equation.clone(), polynomial(&[&[1]]))
                        .unwrap()
                        .compare_to_real(&Real::zero())
                        .unwrap()
                        .is_eq()
                );
            }
        }
        // The exact column remains consumable on both sides of the singularity.
        for y in [-2, -1, 0, 1, 2] {
            let point = Point2::from_values(0, y).into();
            assert!(curves.curves().iter().any(|curve| matches!(curve.point_locations(&point).unwrap(),hypercurve::CurvePointLocations2::Locations(locations) if !locations.is_empty())));
        }
    }
    for boundary in [-2, 2] {
        let equation = power(&polynomial(&[&[-boundary], &[1]]), 2);
        let curves = set(&equation);
        assert_eq!(curves.curves().len(), 1);
        let curve = &curves.curves()[0];
        assert_eq!(curve.polynomial_branch().unwrap().defining_exponent(), 2);
        assert!(
            curve
                .point_at(&Real::zero().into())
                .unwrap()
                .coincides_with(&Point2::from_values(boundary, 0).into())
                .unwrap()
        );
    }
    let tiny = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let equation = BivariatePolynomial::new(vec![vec![], vec![-tiny.clone()], vec![Real::one()]]);
    let curves = set(&equation);
    assert_eq!(curves.curves().len(), 2);
    let a = curves.curves()[0].point_at(&Real::zero().into()).unwrap();
    let b = curves.curves()[1].point_at(&Real::zero().into()).unwrap();
    assert!(!a.coincides_with(&b).unwrap());
    assert!(
        a.affine_form(1.into(), 0.into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap()
            .is_eq()
    );
    assert!(
        b.affine_form(1.into(), 0.into(), 0.into())
            .compare_to_real(&tiny)
            .unwrap()
            .is_eq()
    );
}
