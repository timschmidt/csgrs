use super::*;

fn source(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

// Reproduce the native finite-API inputs of CPArti J1 A5/A6 at their original
// scale. The complete KiCad board remains part of the downstream replay.
fn cparti_pad(center: [f64; 2]) -> CurveRegion2 {
    let x = (source(0.3) / Real::from(2)).unwrap();
    let y = (source(1.15) / Real::from(2)).unwrap();
    let radius = source(0.075);
    let corners = [
        (
            Point2::new(&x - &radius, -&y + &radius),
            Point2::new(&x - &radius, -y.clone()),
            Point2::new(x.clone(), -&y + &radius),
        ),
        (
            Point2::new(&x - &radius, &y - &radius),
            Point2::new(x.clone(), &y - &radius),
            Point2::new(&x - &radius, y.clone()),
        ),
        (
            Point2::new(-&x + &radius, &y - &radius),
            Point2::new(-&x + &radius, y.clone()),
            Point2::new(-x.clone(), &y - &radius),
        ),
        (
            Point2::new(-&x + &radius, -&y + &radius),
            Point2::new(-x.clone(), -&y + &radius),
            Point2::new(-x + &radius, -y),
        ),
    ];
    let mut segments = Vec::new();
    for (i, (center, start, end)) in corners.iter().enumerate() {
        segments.push(Segment2::Arc(
            CircularArc2::try_from_center(start.clone(), end.clone(), center.clone(), false)
                .unwrap(),
        ));
        segments.push(Segment2::Line(
            LineSeg2::try_new(end.clone(), corners[(i + 1) % 4].1.clone()).unwrap(),
        ));
    }
    CurveRegion2::try_from_native_material_contours(vec![Contour2::try_new(segments).unwrap()])
        .unwrap()
        .transform_affine(
            &(-Real::one()),
            &Real::zero(),
            &Real::zero(),
            &(-Real::one()),
            &source(center[0]),
            &source(center[1]),
        )
        .unwrap()
}

#[test]
fn original_cparti_pad_overlap_has_an_exact_area() {
    let left = cparti_pad([141.995001, 98.5]);
    let right = cparti_pad([141.995, 97.5]);
    let overlap = left
        .boolean_region(&right, BooleanOp::Intersection)
        .unwrap();
    assert_eq!(overlap.boundary_loops().len(), 1);
    assert!(overlap.boundary_loops()[0].has_algebraic_fragments());
    let area = overlap
        .filled_area()
        .unwrap()
        .expect("retained circular area");
    assert_eq!(
        real_sign(&area, &CurveContext::STRICT),
        Some(RealSign::Positive)
    );
    assert_eq!(
        compare_reals(&area, &(source(0.3) * source(0.15)), &CurveContext::STRICT),
        Some(std::cmp::Ordering::Less)
    );
    let Classification::Decided(contour) = retained_native_loop_to_contour(
        &overlap.boundary_loops()[0],
        FillRule::NonZero,
        &CurveContext::STRICT,
    )
    .unwrap() else {
        panic!("the same retained arcs must admit a native contour");
    };
    assert!(contour.signed_area().unwrap().is_some());
    assert_eq!(overlap.clone().filled_area().unwrap(), Some(area));
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn selected(coefficients: [i32; 3]) -> BezierParameter2 {
    BezierParameter2::Algebraic(
        crate::BezierAlgebraicParameter2::try_isolate(
            crate::BezierParameterPolynomial::try_new_power_basis(
                coefficients.into_iter().map(Real::from).collect(),
            )
            .unwrap(),
            crate::BezierParameterInterval::try_new(Real::zero(), Real::one()).unwrap(),
        )
        .unwrap(),
    )
}

fn semicircle_source(degree: usize) -> BezierSubcurve2 {
    // x=(1-2t)/(1-2t+2t²), y=2t(1-t)/(1-2t+2t²).
    // The middle homogeneous control is at infinity, while the curve is finite.
    BezierSubcurve2::Rational(
        RationalBezier2::from_homogeneous_controls(
            [(1, 0, 1), (0, 1, 0), (-1, 0, 1)]
                .into_iter()
                .map(|(x, y, w)| {
                    crate::HomogeneousControl2::new(Real::from(x), Real::from(y), Real::from(w))
                })
                .collect(),
        )
        .unwrap()
        .elevated_to_degree(degree)
        .unwrap(),
    )
}

fn close_with_chord(
    fragment: BezierSplitFragment2,
    start: Point2,
    end: Point2,
) -> CurveRegionBoundaryLoop2 {
    let chord = BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            end.clone(),
            Point2::new(
                ((start.x() + end.x()) / Real::from(2)).unwrap(),
                ((start.y() + end.y()) / Real::from(2)).unwrap(),
            ),
            start,
        )),
    };
    CurveRegionBoundaryLoop2::new(vec![fragment, chord], &CurveContext::STRICT).unwrap()
}

#[test]
fn selected_circular_cap_has_closed_form_area_after_elevation_and_reversal() {
    let height = (Real::from(3).sqrt().unwrap() / Real::from(2)).unwrap();
    let expected = (Real::pi() / Real::from(6)).unwrap() - (&height / Real::from(2)).unwrap();
    for degree in [2, 5] {
        let source = semicircle_source(degree);
        let Classification::Decided(split) = source
            .split_at_parameters_refined(
                &BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                &[selected([-1, 2, 2]), selected([3, -6, 2])],
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("exact selected circle cuts");
        };
        let fragment = split.fragments()[1].clone();
        assert!(matches!(
            fragment,
            BezierSplitFragment2::RetainedBezier { .. }
        ));
        let start = Point2::new(rational(1, 2), height.clone());
        let end = Point2::new(rational(-1, 2), height.clone());
        for reversed in [false, true] {
            let boundary = if reversed {
                close_with_chord(fragment.reversed().unwrap(), end.clone(), start.clone())
            } else {
                close_with_chord(fragment.clone(), start.clone(), end.clone())
            };
            for policy in [
                CurveContext::STRICT,
                CurveContext::APPROXIMATE_512,
                CurveContext::STRICT,
            ] {
                let Classification::Decided(Some(area)) =
                    boundary.signed_area_raw(&policy).unwrap()
                else {
                    panic!("selected circular cap integral");
                };
                let expected = if reversed {
                    -&expected
                } else {
                    expected.clone()
                };
                assert_eq!(
                    real_sign(&(area - &expected), &CurveContext::STRICT),
                    Some(RealSign::Zero)
                );
                let Classification::Decided(contour) =
                    retained_native_loop_to_contour(&boundary, FillRule::NonZero, &policy).unwrap()
                else {
                    panic!("native cap view");
                };
                assert_eq!(
                    real_sign(
                        &(contour.signed_area().unwrap().unwrap() - &expected),
                        &CurveContext::STRICT
                    ),
                    Some(RealSign::Zero)
                );
            }
        }
    }
}

#[test]
fn retained_semicircle_and_major_arc_keep_their_sweep() {
    let half = rational(1, 2);
    let major = BezierSubcurve2::RationalQuadratic(
        RationalQuadraticBezier2::try_new(
            Point2::from_values(1, 0),
            Point2::from_values(1, -1),
            Point2::from_values(0, -1),
            Real::one(),
            -half.clone().sqrt().unwrap(),
            Real::one(),
        )
        .unwrap(),
    );
    for (source, start, end, expected) in [
        (
            semicircle_source(2),
            Point2::from_values(1, 0),
            Point2::from_values(-1, 0),
            &half * Real::pi(),
        ),
        (
            major,
            Point2::from_values(1, 0),
            Point2::from_values(0, -1),
            rational(3, 4) * Real::pi() + half,
        ),
    ] {
        let fragment = BezierSplitFragment2::RetainedBezier {
            reversed: false,
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            source_curve: source,
            start_image: None,
            end_image: None,
        };
        for reversed in [false, true] {
            let boundary = if reversed {
                close_with_chord(fragment.reversed().unwrap(), end.clone(), start.clone())
            } else {
                close_with_chord(fragment.clone(), start.clone(), end.clone())
            };
            let area = boundary.signed_area().unwrap().unwrap();
            let expected = if reversed {
                -&expected
            } else {
                expected.clone()
            };
            assert_eq!(
                real_sign(&(area - expected), &CurveContext::STRICT),
                Some(RealSign::Zero)
            );
        }
    }
}

#[test]
fn native_fragment_view_rejects_unproved_exterior_traversal_and_general_curves() {
    let fragment = |source_curve, start, end| BezierSplitFragment2::RetainedBezier {
        reversed: false,
        start,
        end,
        source_curve,
        start_image: None,
        end_image: None,
    };
    let exterior = fragment(
        semicircle_source(2),
        BezierParameter2::Exact(Real::from(-1)),
        BezierParameter2::Exact(Real::zero()),
    );
    let general = fragment(
        BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 2),
            Point2::from_values(2, 0),
        )),
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    );
    for fragment in [exterior, general] {
        assert!(matches!(
            retained_native_fragment_segment(&fragment, &CurveContext::STRICT).unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        ));
    }
}
