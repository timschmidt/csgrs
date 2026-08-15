use hypercurve::{
    BezierSplitFragment2, BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2,
    CurveCertainty, CurveContext, CurveError, CurvePath2, CurveRegion2, ExactCurveError, LineSeg2,
    OffsetCap, OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2, Real,
    RegionPointLocation,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (s(numerator) / s(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Curve2 {
    Curve2::from(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn location(region: &CurveRegion2, point: Point2) -> RegionPointLocation {
    match region
        .classify_point(&point, &CurveContext::STRICT)
        .unwrap()
        .value
    {
        Classification::Decided(location) => location,
        Classification::Uncertain(reason) => panic!("point classification blocked: {reason:?}"),
    }
}

#[test]
fn exact_path_stroke_cap_styles_have_their_defined_extent() {
    let path = CurvePath2::try_new(vec![line(0, 0, 4, 0)]).unwrap();
    let stroke = |cap| {
        CurveRegion2::stroke_path(
            &path,
            s(1),
            &OffsetCornerStyle2::Round,
            cap,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    };

    let butt = stroke(OffsetCap::Butt);
    assert_eq!(
        location(&butt, Point2::new(q(-1, 2), s(0))),
        RegionPointLocation::Outside
    );
    assert_eq!(
        location(&butt, Point2::new(s(2), q(1, 2))),
        RegionPointLocation::Inside
    );

    let round = stroke(OffsetCap::Round);
    assert_eq!(
        location(&round, Point2::new(q(-1, 2), s(0))),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&round, Point2::new(s(-1), s(-1))),
        RegionPointLocation::Outside
    );

    let square = stroke(OffsetCap::Square);
    assert_eq!(
        location(&square, Point2::new(q(-1, 2), q(3, 4))),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&square, Point2::new(q(-3, 2), s(0))),
        RegionPointLocation::Outside
    );
}

#[test]
fn exact_path_stroke_corner_styles_share_the_region_offset_solver() {
    let path = CurvePath2::try_new(vec![line(0, 0, 4, 0), line(4, 0, 4, 4)]).unwrap();
    let stroke = |style: &OffsetCornerStyle2| {
        CurveRegion2::stroke_path(&path, s(1), style, OffsetCap::Butt, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    };

    let bevel = stroke(&OffsetCornerStyle2::Bevel);
    let round = stroke(&OffsetCornerStyle2::Round);
    let miter = stroke(&OffsetCornerStyle2::Miter { limit: s(4) });
    let round_only = Point2::new(q(9, 2), q(-3, 4));
    let miter_only = Point2::new(q(19, 4), q(-3, 4));
    assert_eq!(
        location(&bevel, round_only.clone()),
        RegionPointLocation::Outside
    );
    assert_eq!(location(&round, round_only), RegionPointLocation::Inside);
    assert_eq!(
        location(&round, miter_only.clone()),
        RegionPointLocation::Outside
    );
    assert_eq!(location(&miter, miter_only), RegionPointLocation::Inside);
}

#[test]
fn exact_path_stroke_regularizes_a_parallel_reversal_corner() {
    let path = CurvePath2::try_new(vec![line(0, 0, 2, 0), line(2, 0, 1, 0)]).unwrap();
    let stroke = |style: &OffsetCornerStyle2| {
        CurveRegion2::stroke_path(&path, s(1), style, OffsetCap::Butt, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    };
    let round = stroke(&OffsetCornerStyle2::Round);

    assert_eq!(
        location(&round, Point2::new(q(5, 2), s(0))),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&round, Point2::new(q(7, 2), s(0))),
        RegionPointLocation::Outside
    );
    for nonround in [
        OffsetCornerStyle2::Bevel,
        OffsetCornerStyle2::Miter { limit: s(4) },
    ] {
        let stroke = stroke(&nonround);
        assert_eq!(
            location(&stroke, Point2::new(q(5, 2), s(0))),
            RegionPointLocation::Outside
        );
        assert_eq!(
            location(&stroke, Point2::new(q(3, 2), s(0))),
            RegionPointLocation::Inside
        );
    }

    let closed = CurvePath2::try_new(vec![line(0, 0, 2, 0), line(2, 0, 0, 0)]).unwrap();
    let capsule = CurveRegion2::stroke_path(
        &closed,
        s(1),
        &OffsetCornerStyle2::Round,
        OffsetCap::Butt,
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    assert_eq!(
        location(&capsule, Point2::new(q(-1, 2), s(0))),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&capsule, Point2::new(q(5, 2), s(0))),
        RegionPointLocation::Inside
    );
}

#[test]
fn exact_path_stroke_is_invariant_to_a_collinear_partition() {
    let stroke = |path: CurvePath2| {
        CurveRegion2::stroke_path(
            &path,
            s(1),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    };
    let whole = stroke(CurvePath2::try_new(vec![line(0, 0, 4, 0)]).unwrap());
    let partitioned =
        stroke(CurvePath2::try_new(vec![line(0, 0, 2, 0), line(2, 0, 4, 0)]).unwrap());
    let symmetric_difference = partitioned
        .boolean_region(&whole, BooleanOp::Xor, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(symmetric_difference.is_empty());
}

#[test]
fn path_stroke_requires_a_policy_positive_half_width() {
    let path = CurvePath2::try_new(vec![line(0, 0, 4, 0)]).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for half_width in [s(0), s(-1)] {
            assert!(matches!(
                CurveRegion2::stroke_path(
                    &path,
                    half_width,
                    &OffsetCornerStyle2::Round,
                    OffsetCap::Butt,
                    &policy,
                ),
                Err(ExactCurveError::Invalid {
                    cause: CurveError::InvalidOffsetOptions,
                    ..
                })
            ));
        }
    }

    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    assert!(matches!(
        CurveRegion2::stroke_path(
            &path,
            undecidable_zero.clone(),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::RealSign
    ));
    assert!(matches!(
        CurveRegion2::stroke_path(
            &path,
            undecidable_zero,
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::APPROXIMATE_512,
        ),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidOffsetOptions,
            ..
        })
    ));
}

#[test]
fn self_crossing_source_is_regularized_instead_of_rejected() {
    let path =
        CurvePath2::try_new(vec![line(0, 0, 4, 4), line(4, 4, 0, 4), line(0, 4, 4, 0)]).unwrap();
    let stroke = CurveRegion2::stroke_path(
        &path,
        q(1, 4),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
        &CurveContext::STRICT,
    )
    .unwrap();
    assert_eq!(stroke.certainty, CurveCertainty::Certified);
    assert!(!stroke.value.is_empty());
    assert_eq!(
        location(&stroke.value, p(2, 2)),
        RegionPointLocation::Inside
    );
}

#[test]
fn closed_path_stroke_is_cyclic_and_does_not_apply_caps() {
    let path = CurvePath2::try_new(vec![
        line(0, 0, 4, 0),
        line(4, 0, 4, 4),
        line(4, 4, 0, 4),
        line(0, 4, 0, 0),
    ])
    .unwrap();
    let stroke = |cap| {
        CurveRegion2::stroke_path(
            &path,
            s(1),
            &OffsetCornerStyle2::Round,
            cap,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    };
    let butt = stroke(OffsetCap::Butt);
    assert_eq!(butt, stroke(OffsetCap::Round));
    assert_eq!(butt, stroke(OffsetCap::Square));
    assert_eq!(location(&butt, p(2, 2)), RegionPointLocation::Outside);
    assert_eq!(
        location(&butt, Point2::new(s(2), q(1, 2))),
        RegionPointLocation::Inside
    );
}

#[test]
fn exact_path_stroke_handles_an_arc_parallel_radius_collapse() {
    let path = CurvePath2::try_new(vec![Curve2::from(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let stroke = CurveRegion2::stroke_path(
        &path,
        s(1),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
        &CurveContext::STRICT,
    )
    .unwrap();
    assert_eq!(stroke.certainty, CurveCertainty::Certified);
    assert_eq!(
        location(&stroke.value, Point2::new(s(1), q(-1, 2))),
        RegionPointLocation::Inside
    );
    assert_eq!(
        location(&stroke.value, p(1, 3)),
        RegionPointLocation::Outside
    );
}

#[test]
fn curved_endpoint_caps_use_the_exact_one_sided_tangent() {
    let path = CurvePath2::try_new(vec![Curve2::from(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap(),
    )])
    .unwrap();
    let stroke = |cap| {
        CurveRegion2::stroke_path(
            &path,
            q(1, 2),
            &OffsetCornerStyle2::Round,
            cap,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value()
    };
    let behind_start = Point2::new(q(-1, 4), q(-1, 4));
    assert_eq!(
        location(&stroke(OffsetCap::Butt), behind_start.clone()),
        RegionPointLocation::Outside
    );
    assert_eq!(
        location(&stroke(OffsetCap::Round), behind_start),
        RegionPointLocation::Inside
    );

    let square_only = Point2::new(q(-9, 20), q(-9, 20));
    assert_eq!(
        location(&stroke(OffsetCap::Round), square_only.clone()),
        RegionPointLocation::Outside
    );
    assert_eq!(
        location(&stroke(OffsetCap::Square), square_only),
        RegionPointLocation::Inside
    );
}

#[test]
fn square_cap_uses_the_first_nonzero_endpoint_derivative() {
    let start_singular = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        p(0, 0),
        p(0, 0),
        p(4, 0),
    ))])
    .unwrap();
    let start_stroke = CurveRegion2::stroke_path(
        &start_singular,
        s(1),
        &OffsetCornerStyle2::Round,
        OffsetCap::Square,
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    assert_eq!(
        location(&start_stroke, Point2::new(q(-1, 2), q(1, 2))),
        RegionPointLocation::Inside
    );

    let end_singular = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        p(0, 0),
        p(4, 0),
        p(4, 0),
    ))])
    .unwrap();
    let end_stroke = CurveRegion2::stroke_path(
        &end_singular,
        s(1),
        &OffsetCornerStyle2::Round,
        OffsetCap::Square,
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    assert_eq!(
        location(&end_stroke, Point2::new(q(9, 2), q(1, 2))),
        RegionPointLocation::Inside
    );
}

#[test]
fn nonlinear_source_cusp_endpoint_uses_its_exact_one_sided_frame() {
    // P(u)=(u^2,u^3) has a nonlinear source cusp at its first endpoint.
    // Reversing its controls moves the same cusp to the final endpoint and
    // requires the even-order endpoint derivative to be orientation-corrected.
    let third = q(1, 3);
    let curves = [
        CubicBezier2::new(
            p(0, 0),
            p(0, 0),
            Point2::new(third.clone(), Real::zero()),
            p(1, 1),
        ),
        CubicBezier2::new(p(1, 1), Point2::new(third, Real::zero()), p(0, 0), p(0, 0)),
    ];
    for curve in curves {
        let path = CurvePath2::try_new(vec![Curve2::from(curve)]).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::reset();
            let stroke_call = || {
                CurveRegion2::stroke_path(
                    &path,
                    q(1, 4),
                    &OffsetCornerStyle2::Round,
                    OffsetCap::Square,
                    &policy,
                )
            };
            #[cfg(feature = "dispatch-trace")]
            let stroke = hyperreal::dispatch_trace::with_recording(stroke_call);
            #[cfg(not(feature = "dispatch-trace"))]
            let stroke = stroke_call();
            let stroke = stroke.unwrap_or_else(|error| {
                #[cfg(feature = "dispatch-trace")]
                let trace = hyperreal::dispatch_trace::take_trace()
                    .dispatch
                    .into_iter()
                    .filter(|entry| entry.layer == "hypercurve")
                    .collect::<Vec<_>>();
                #[cfg(not(feature = "dispatch-trace"))]
                let trace = ();
                panic!("a nonlinear endpoint source cusp must remain strokeable: policy={policy:?}, error={error:?}; {trace:?}")
            });
            assert_eq!(stroke.certainty, CurveCertainty::Certified);
            assert_eq!(
                location(&stroke.value, Point2::new(q(-1, 8), Real::zero())),
                RegionPointLocation::Inside,
            );
            assert_eq!(
                location(&stroke.value, Point2::new(q(-3, 8), Real::zero())),
                RegionPointLocation::Outside,
            );
        }
    }
}

#[test]
fn nonlinear_path_stroke_retains_exact_parallels_under_both_policies() {
    let path = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        p(0, 0),
        p(1, 2),
        p(3, 2),
        p(4, 0),
    ))])
    .unwrap();
    let run = |policy| {
        CurveRegion2::stroke_path(
            &path,
            q(1, 10),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            policy,
        )
    };
    let strict = run(&CurveContext::STRICT).unwrap();
    let approximate = run(&CurveContext::APPROXIMATE_512).unwrap();
    assert_eq!(strict.value, approximate.value);
    assert!(strict.value.boundary_loops().iter().any(|loop_| {
        loop_
            .fragments()
            .iter()
            .any(|fragment| matches!(fragment, BezierSplitFragment2::AnalyticParallel(_)))
    }));
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert!(matches!(
        CurveRegion2::stroke_path(
            &path,
            Real::zero(),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Invalid { .. })
    ));
}

#[test]
#[ignore = "long exact end-to-end source-cusp regression; run explicitly in release mode"]
fn exact_path_stroke_splits_an_interior_source_cusp() {
    // P(u) = ((2u - 1)^2, (2u - 1)^3).  Both first derivatives vanish at
    // u=1/2, while the two regular incident branches have exact opposite
    // one-sided tangents.  A stroke must partition at that source singularity
    // and regularize the two branch bands instead of rejecting the whole
    // authored cubic.
    let path = CurvePath2::try_new(vec![Curve2::from(CubicBezier2::new(
        p(1, -1),
        Point2::new(q(-1, 3), s(1)),
        Point2::new(q(-1, 3), s(-1)),
        p(1, 1),
    ))])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let stroke_call = || {
            CurveRegion2::stroke_path(
                &path,
                q(1, 4),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let stroke = hyperreal::dispatch_trace::with_recording(stroke_call);
        #[cfg(not(feature = "dispatch-trace"))]
        let stroke = stroke_call();
        let stroke = stroke.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace()
                .dispatch
                .into_iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            #[cfg(not(feature = "dispatch-trace"))]
            let trace = ();
            panic!("an exact source cusp must remain strokeable: policy={policy:?}, error={error:?}; {trace:?}")
        });
        assert_eq!(stroke.certainty, CurveCertainty::Certified);
        assert!(!stroke.value.is_empty());
        assert_eq!(
            location(&stroke.value, p(0, 0)),
            RegionPointLocation::Inside
        );
    }
}

#[test]
#[ignore = "long exact degree-four end-to-end regression; about 24 seconds in release mode"]
fn exact_path_stroke_preserves_an_even_multiplicity_stationary_source() {
    // With s=2u-1, P(u)=(s^3,s^4) has P'(u)=s^2(6,8s).
    // The common hodograph factor has even multiplicity, so the two regular
    // branches meet on the same normal sheet and must not acquire a reversal
    // cap at u=1/2. Degree four is represented by an exact unit-weight
    // RationalBezier2 because the public polynomial primitives stop at cubic.
    let curve = RationalBezier2::try_new(
        vec![
            p(-1, 1),
            Point2::new(q(1, 2), s(-1)),
            p(0, 1),
            Point2::new(q(-1, 2), s(-1)),
            p(1, 1),
        ],
        vec![Real::one(); 5],
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![Curve2::from(curve)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let stroke_call = || {
            CurveRegion2::stroke_path(
                &path,
                q(1, 4),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let stroke = hyperreal::dispatch_trace::with_recording(stroke_call);
        #[cfg(not(feature = "dispatch-trace"))]
        let stroke = stroke_call();
        let stroke = stroke.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace()
                .dispatch
                .into_iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            #[cfg(not(feature = "dispatch-trace"))]
            let trace = ();
            panic!("an even-multiplicity stationary source must remain strokeable: policy={policy:?}, error={error:?}; {trace:?}")
        });
        assert_eq!(stroke.certainty, CurveCertainty::Certified);
        assert!(!stroke.value.is_empty());
        assert_eq!(
            location(&stroke.value, p(0, 0)),
            RegionPointLocation::Inside
        );
    }
}

#[test]
#[ignore = "long exact rational-cusp end-to-end regression; run explicitly in release mode"]
fn exact_path_stroke_splits_a_nonuniform_rational_source_cusp() {
    // These controls and weights are the projective reparameterization
    // t=u/(2-u) of P(t)=((2t-1)^2,(2t-1)^3). The geometric cusp is unchanged,
    // but its source parameter moves from t=1/2 to u=2/3 and its homogeneous
    // hodograph is genuinely rational.
    let curve = RationalBezier2::try_new(
        vec![
            p(1, -1),
            Point2::new(q(-1, 3), s(1)),
            Point2::new(q(-1, 3), s(-1)),
            p(1, 1),
        ],
        vec![s(8), s(4), s(2), s(1)],
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![Curve2::from(curve)]).unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let stroke_call = || {
            CurveRegion2::stroke_path(
                &path,
                q(1, 4),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
                &policy,
            )
        };
        #[cfg(feature = "dispatch-trace")]
        let stroke = hyperreal::dispatch_trace::with_recording(stroke_call);
        #[cfg(not(feature = "dispatch-trace"))]
        let stroke = stroke_call();
        let stroke = stroke.unwrap_or_else(|error| {
            #[cfg(feature = "dispatch-trace")]
            let trace = hyperreal::dispatch_trace::take_trace()
                .dispatch
                .into_iter()
                .filter(|entry| entry.layer == "hypercurve")
                .collect::<Vec<_>>();
            #[cfg(not(feature = "dispatch-trace"))]
            let trace = ();
            panic!("a nonuniform rational source cusp must remain strokeable: policy={policy:?}, error={error:?}; {trace:?}")
        });
        assert_eq!(stroke.certainty, CurveCertainty::Certified);
        assert!(!stroke.value.is_empty());
        assert_eq!(
            location(&stroke.value, p(0, 0)),
            RegionPointLocation::Inside
        );
    }
}

#[test]
fn rational_nurbs_path_stroke_retains_exact_parallels_under_both_policies() {
    let curve = Curve2::try_nurbs(
        2,
        vec![p(0, 0), p(2, 3), p(4, 0)],
        vec![s(1), s(2), s(1)],
        vec![s(0), s(0), s(0), s(1), s(1), s(1)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let path = CurvePath2::try_new(vec![curve]).unwrap();
    let run = |policy| {
        CurveRegion2::stroke_path(
            &path,
            q(1, 10),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            policy,
        )
        .unwrap()
    };

    let strict = run(&CurveContext::STRICT);
    let approximate = run(&CurveContext::APPROXIMATE_512);
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(strict.value, approximate.value);
    assert!(strict.value.boundary_loops().iter().any(|loop_| {
        loop_
            .fragments()
            .iter()
            .any(|fragment| matches!(fragment, BezierSplitFragment2::AnalyticParallel(_)))
    }));
}

#[test]
fn path_stroke_obeys_the_approximate_512_connectivity_terminal() {
    let left_form = Real::pi() + Real::e();
    let right_form = Real::e() + Real::pi();
    let curves = vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), Point2::new(left_form, Real::zero())).unwrap()),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(right_form, Real::zero()),
                Point2::new(s(8), s(0)),
            )
            .unwrap(),
        ),
    ];
    assert!(matches!(
        CurvePath2::try_new_with_policy(curves.clone(), &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(_))
    ));
    let path = CurvePath2::try_new_with_policy(curves, &CurveContext::APPROXIMATE_512)
        .unwrap()
        .into_value();
    assert!(matches!(
        CurveRegion2::stroke_path(
            &path,
            q(1, 2),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(_))
    ));
    let stroke = CurveRegion2::stroke_path(
        &path,
        q(1, 2),
        &OffsetCornerStyle2::Round,
        OffsetCap::Butt,
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(stroke.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(
        location(&stroke.value, p(1, 0)),
        RegionPointLocation::Inside
    );
}

#[test]
fn path_stroke_obeys_the_approximate_512_closure_terminal() {
    let left_form = Real::pi() + Real::e();
    let right_form = Real::e() + Real::pi();
    let path = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(left_form, Real::zero()),
                Point2::new(s(8), s(0)),
            )
            .unwrap(),
        ),
        Curve2::from(
            LineSeg2::try_new(
                Point2::new(s(8), s(0)),
                Point2::new(right_form, Real::zero()),
            )
            .unwrap(),
        ),
    ])
    .unwrap();
    assert!(matches!(
        CurveRegion2::stroke_path(
            &path,
            q(1, 2),
            &OffsetCornerStyle2::Round,
            OffsetCap::Butt,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(_))
    ));
    let stroke = CurveRegion2::stroke_path(
        &path,
        q(1, 2),
        &OffsetCornerStyle2::Round,
        OffsetCap::Butt,
        &CurveContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(stroke.certainty, CurveCertainty::Approximate512Consumed);
    assert!(!stroke.value.is_empty());
}
