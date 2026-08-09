#[cfg(feature = "predicates")]
use hypercurve::{
    BezierAlgebraicChord2, BezierAlgebraicParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CurveBoundaryInteriorSide2, CurveRegionBoundaryLoop2,
    RationalBezierIntersectionPointEvidence2,
};
use hypercurve::{
    BezierFlatteningOptions, BezierSplitFragment2, CircularArc2, Classification, Contour2,
    CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveCornerMode2, CurveCornerNoSolution2,
    CurveCornerSolutions2, CurveError, CurveFamily2, CurveOutcome, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, ExactCurveError, FillRule, FiniteProjectionOptions, LineSeg2,
    OffsetCornerStyle2, Point2, QuadraticBezier2, RationalBezier2, Real, RegionPointLocation,
    Segment2, Similarity2,
};
use hyperreal::SymbolicDependencyMask;

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn sharp_offset() -> OffsetCornerStyle2 {
    OffsetCornerStyle2::Miter {
        limit: Real::from(1_000),
    }
}

fn square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> Contour2 {
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(min_x, min_y), p(max_x, min_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, min_y), p(max_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, max_y), p(min_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(min_x, max_y), p(min_x, min_y)).unwrap()),
    ])
    .unwrap()
}

fn reversed(contour: &Contour2) -> Contour2 {
    Contour2::try_new_with_fill_rule(
        contour
            .segments()
            .iter()
            .rev()
            .map(Segment2::reversed)
            .collect(),
        contour.fill_rule(),
    )
    .unwrap()
}

fn square_with_redundant_edge() -> Contour2 {
    let points = [p(0, 0), p(2, 0), p(4, 0), p(4, 4), p(0, 4), p(0, 0)];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn right_isosceles_triangle() -> Contour2 {
    let points = [p(0, 0), p(4, 0), p(0, 4), p(0, 0)];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn double_wound_square(fill_rule: FillRule) -> Contour2 {
    let corners = [p(0, 0), p(10, 0), p(10, 10), p(0, 10), p(0, 0)];
    let segments = corners
        .windows(2)
        .chain(corners.windows(2))
        .map(|edge| Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap()))
        .collect();
    Contour2::try_new_with_fill_rule(segments, fill_rule).unwrap()
}

fn path_from_contour(contour: &Contour2) -> CurvePath2 {
    CurvePath2::try_new(
        contour
            .segments()
            .iter()
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line.clone()),
                Segment2::Arc(arc) => Curve2::from(arc.clone()),
            })
            .collect(),
    )
    .unwrap()
}

fn full_circle_path(radius: i64) -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(
            CircularArc2::try_from_center(p(radius, 0), p(-radius, 0), p(0, 0), false).unwrap(),
        ),
        Curve2::from(
            CircularArc2::try_from_center(p(-radius, 0), p(radius, 0), p(0, 0), false).unwrap(),
        ),
    ])
    .unwrap()
}

fn double_wound_quadratic_cap() -> CurvePath2 {
    let curve = Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4)));
    let close = Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap());
    CurvePath2::try_new(vec![curve.clone(), close.clone(), curve, close]).unwrap()
}

fn rational_cap_path() -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![p(-2, 0), p(0, 4), p(2, 0)],
                vec![Real::one(), Real::from(2), Real::one()],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(-2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-2, -2), p(-2, 0)).unwrap()),
    ])
    .unwrap()
}

fn quadratic_fillet_path() -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(0, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -2), p(0, 0)).unwrap()),
    ])
    .unwrap()
}

fn bow_tie_path() -> CurvePath2 {
    let points = [p(0, 0), p(4, 4), p(0, 4), p(4, 0), p(0, 0)];
    CurvePath2::try_new(
        points
            .windows(2)
            .map(|edge| Curve2::from(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap()))
            .collect(),
    )
    .unwrap()
}

fn self_crossing_cubic_path(rational_reparameterization: bool) -> CurvePath2 {
    // The two self-contact parameters are the irrational roots of
    // `t^2 - t + 1/8`, so region traversal exercises retained algebraic
    // endpoints rather than only represented rational witnesses.
    let controls = vec![p(3, 0), p(-5, 1), p(-5, -6), p(3, 3)];
    let curve = if rational_reparameterization {
        Curve2::from(
            RationalBezier2::try_new(
                controls,
                vec![Real::one(), Real::from(2), Real::from(4), Real::from(8)],
            )
            .unwrap(),
        )
    } else {
        Curve2::from(CubicBezier2::new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            controls[3].clone(),
        ))
    };
    CurvePath2::try_new(vec![
        curve,
        Curve2::from(LineSeg2::try_new(p(3, 3), p(3, 0)).unwrap()),
    ])
    .unwrap()
}

fn bow_tie_contour(fill_rule: FillRule) -> Contour2 {
    let points = [p(0, 0), p(4, 4), p(0, 4), p(4, 0), p(0, 0)];
    Contour2::try_new_with_fill_rule(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
        fill_rule,
    )
    .unwrap()
}

fn u_shape() -> Contour2 {
    let points = [
        p(0, 0),
        p(10, 0),
        p(10, 10),
        p(7, 10),
        p(7, 3),
        p(3, 3),
        p(3, 10),
        p(0, 10),
        p(0, 0),
    ];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

fn dumbbell_shape() -> Contour2 {
    let points = [
        p(0, 0),
        p(4, 0),
        p(4, 1),
        p(8, 1),
        p(8, 0),
        p(12, 0),
        p(12, 4),
        p(8, 4),
        p(8, 3),
        p(4, 3),
        p(4, 4),
        p(0, 4),
        p(0, 0),
    ];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}

trait IntoCertifiedClassification<T> {
    fn into_certified_classification(self) -> Classification<T>;
}

impl<T> IntoCertifiedClassification<T> for Classification<T> {
    fn into_certified_classification(self) -> Classification<T> {
        self
    }
}

impl<T> IntoCertifiedClassification<T> for CurveOutcome<Classification<T>> {
    fn into_certified_classification(self) -> Classification<T> {
        assert_eq!(self.certainty, CurveCertainty::Certified);
        self.value
    }
}

fn decided<T>(classification: impl IntoCertifiedClassification<T>) -> T {
    match classification.into_certified_classification() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected decided result, got {reason:?}"),
    }
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}

#[cfg(feature = "predicates")]
fn axis_aligned_algebraic_rectangle(policy: &CurveContext) -> CurveRegion2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            vec![-q(1, 2), Real::zero(), Real::one()],
            policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
    let horizontal = |height: Real| {
        RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), height.clone()),
                Point2::new(Real::one(), height),
            ],
            vec![Real::one(); 2],
        )
        .unwrap()
    };
    let bottom_right = RationalBezierIntersectionPointEvidence2::Algebraic(
        horizontal(Real::zero())
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
    );
    let top_right = RationalBezierIntersectionPointEvidence2::Algebraic(
        horizontal(Real::one())
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
    );
    let bottom_left = RationalBezierIntersectionPointEvidence2::Exact(p(0, 0));
    let top_left = RationalBezierIntersectionPointEvidence2::Exact(p(0, 1));
    let chord = |start, end| {
        BezierSplitFragment2::AlgebraicChord(decided(
            BezierAlgebraicChord2::try_new(start, end, policy).unwrap(),
        ))
    };
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            chord(bottom_left.clone(), bottom_right.clone()),
            chord(bottom_right, top_right.clone()),
            chord(top_right, top_left.clone()),
            chord(top_left, bottom_left),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

#[cfg(feature = "predicates")]
#[test]
fn correlated_chord_pair_endpoints_survive_transform_and_offset() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = axis_aligned_algebraic_rectangle(&policy);
        let second = first
            .transform_affine(
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &Real::one(),
                &q(1, 4),
                &q(1, 4),
                &policy,
            )
            .expect("the translated selected-field rectangle must remain exact")
            .into_value();
        let evidence = first
            .intersect_region(&second, &policy)
            .expect("the two retained chord regions must intersect exactly");
        assert_eq!(evidence.certainty, CurveCertainty::Certified);
        assert!(
            evidence.value.is_complete(),
            "{:?}",
            evidence.value.blockers()
        );
        assert!(
            evidence
                .value
                .contacts()
                .iter()
                .filter(|contact| matches!(
                    contact.point(),
                    Some(RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_))
                ))
                .count()
                >= 1,
            "the strict-interior line crossings must retain their two-support point evidence: {evidence:?}",
        );

        let batch = first
            .boolean_regions(&second, &policy)
            .expect("the two retained chord regions must Boolean exactly");
        assert_eq!(batch.certainty, CurveCertainty::Certified);
        let intersection = batch.value.intersection().clone();
        let retained_pair_endpoints = |region: &CurveRegion2| {
            region
                .boundary_loops()
                .iter()
                .flat_map(|boundary| boundary.fragments())
                .filter_map(|fragment| match fragment {
                    BezierSplitFragment2::AlgebraicChord(chord) => Some(
                        usize::from(matches!(
                            chord.start(),
                            RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
                        )) + usize::from(matches!(
                            chord.end(),
                            RationalBezierIntersectionPointEvidence2::AlgebraicChordPair(_)
                        )),
                    ),
                    _ => None,
                })
                .sum::<usize>()
        };
        assert!(retained_pair_endpoints(&intersection) >= 2);

        let transformed = intersection
            .transform_affine(
                &Real::zero(),
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::from(2),
                &Real::from(-3),
                &policy,
            )
            .expect("correlated chord-pair endpoints must survive an exact affine transform");
        assert_eq!(transformed.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                transformed
                    .value
                    .classify_point(&Point2::new(q(5, 2), q(-5, 2)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Inside),
        );
        assert!(retained_pair_endpoints(&transformed.value) >= 2);

        let expanded = transformed
            .value
            .offset(q(1, 20), &sharp_offset(), &policy)
            .expect("transformed chord-pair endpoints must survive an exact offset");
        assert_eq!(expanded.certainty, CurveCertainty::Certified);
        assert!(!expanded.value.is_empty());
        assert!(retained_pair_endpoints(&expanded.value) >= 2);
    }
}

#[cfg(feature = "predicates")]
fn axis_aligned_algebraic_l_region(policy: &CurveContext) -> CurveRegion2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            vec![-q(1, 2), Real::zero(), Real::one()],
            policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
    let selected = |height: Real| {
        RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezier2::try_new(
                vec![
                    Point2::new(Real::zero(), height.clone()),
                    Point2::new(Real::one(), height),
                ],
                vec![Real::one(); 2],
            )
            .unwrap()
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
        )
    };
    let exact = |x, y| RationalBezierIntersectionPointEvidence2::Exact(Point2::new(x, y));
    let points = [
        exact(Real::zero(), Real::zero()),
        selected(Real::zero()),
        selected(Real::one()),
        exact(q(1, 2), Real::one()),
        exact(q(1, 2), q(1, 2)),
        exact(Real::zero(), q(1, 2)),
    ];
    let fragments = (0..points.len())
        .map(|index| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                    policy,
                )
                .unwrap(),
            ))
        })
        .collect();
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

#[cfg(feature = "predicates")]
fn axis_aligned_algebraic_dumbbell_region(
    policy: &CurveContext,
    fill_rule: FillRule,
    reverse: bool,
) -> CurveRegion2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(
            vec![-q(1, 2), Real::zero(), Real::one()],
            policy,
        )
        .unwrap(),
    );
    let interval =
        decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
    let parameter =
        decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
    let selected = |height: Real| {
        RationalBezierIntersectionPointEvidence2::Algebraic(
            RationalBezier2::try_new(
                vec![
                    Point2::new(Real::from(12), height.clone()),
                    Point2::new(Real::from(13), height),
                ],
                vec![Real::one(); 2],
            )
            .unwrap()
            .point_at_algebraic_parameter(&parameter, policy)
            .unwrap(),
        )
    };
    let exact = |x, y| RationalBezierIntersectionPointEvidence2::Exact(p(x, y));
    let points = [
        exact(0, 0),
        exact(4, 0),
        exact(4, 1),
        exact(8, 1),
        exact(8, 0),
        selected(Real::zero()),
        selected(Real::from(4)),
        exact(8, 4),
        exact(8, 3),
        exact(4, 3),
        exact(4, 4),
        exact(0, 4),
    ];
    let fragments = (0..points.len())
        .map(|index| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                    policy,
                )
                .unwrap(),
            ))
        })
        .collect::<Vec<_>>();
    let fragments = if reverse {
        fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect()
    } else {
        fragments
    };
    let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![fill_rule],
        vec![if reverse {
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        }],
    )
    .unwrap()
}

#[test]
fn unified_native_constructor_retains_zero_signed_area_boundary_for_diagnostics() {
    let policy = CurveContext::STRICT;
    let contour = bow_tie_contour(FillRule::EvenOdd);

    let region = CurveRegion2::try_from_native_material_contours(vec![contour.clone()], &policy)
        .unwrap()
        .into_value();
    let native = decided(region.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours(), std::slice::from_ref(&contour));
    assert!(native.hole_contours().is_empty());
}

#[test]
fn unified_region_offsets_quadratic_boundary_through_exact_parallel_arrangement() {
    let policy = CurveContext::STRICT;
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 0), p(0, 4), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(-2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-2, -2), p(-2, 0)).unwrap()),
    ])
    .unwrap();
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    let exact = source
        .offset(Real::one(), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    assert!(!exact.is_empty());
    assert!(exact.has_algebraic_fragments());
    assert_eq!(
        exact
            .classify_point(&p(0, 0), &policy)
            .unwrap()
            .into_value(),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        exact
            .classify_point(&p(0, 5), &policy)
            .unwrap()
            .into_value(),
        Classification::Decided(RegionPointLocation::Outside)
    );
    let options = BezierFlatteningOptions::try_new(q(1, 32), 12, &policy).unwrap();

    let segmented = decided(
        source
            .segment_certified(&options, &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(segmented.evidence().max_source_chord_error(), &q(1, 32));
    assert!(segmented.evidence().lossy_boundary());
    assert_eq!(segmented.evidence().loop_evidence().len(), 1);
    assert_eq!(
        segmented.evidence().loop_evidence()[0].role(),
        CurveRegionLoopRole::Material
    );
    assert_eq!(
        segmented.evidence().loop_evidence()[0].fill_rule(),
        FillRule::NonZero
    );
    assert!(segmented.evidence().loop_evidence()[0].output_segment_count() > 4);
    assert!(matches!(
        certified(
            segmented
                .region()
                .native_contours_fast_path(&policy)
                .unwrap()
        ),
        Classification::Decided(_)
    ));

    let offset = decided(
        source
            .offset_with_certified_segmentation(Real::one(), &sharp_offset(), &options, &policy)
            .unwrap()
            .into_value(),
    );

    assert!(!offset.region().is_empty());
    assert!(offset.evidence().used_exact_authoritative_path());
    assert!(!offset.evidence().lossy_boundary());
    assert_eq!(offset.evidence().max_source_chord_error(), &q(1, 32));
    assert!(offset.evidence().loop_evidence().is_empty());
    assert!(offset.region().has_algebraic_fragments());
}

#[test]
fn repeated_region_offsets_compose_retained_exact_parallels_under_both_policies() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1))),
        Curve2::from(QuadraticBezier2::new(p(0, 1), p(-1, 1), p(-1, 0))),
        Curve2::from(QuadraticBezier2::new(p(-1, 0), p(-1, -1), p(0, -1))),
        Curve2::from(QuadraticBezier2::new(p(0, -1), p(1, -1), p(1, 0))),
    ])
    .unwrap();
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();

    let strict_first = source
        .offset(q(1, 10), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(
        strict_first.boundary_loops()[0]
            .fragments()
            .iter()
            .all(|fragment| matches!(fragment, BezierSplitFragment2::AnalyticParallel(_)))
    );
    assert_eq!(
        decided(strict_first.loop_roles(&CurveContext::STRICT).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    let strict_repeated = strict_first
        .offset(q(1, 5), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap();
    let strict_direct = source
        .offset(q(3, 10), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict_repeated.certainty, CurveCertainty::Certified);
    assert_eq!(strict_repeated.value, strict_direct.value);
    let strict_partially_reversed = strict_first
        .offset(-q(1, 20), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap();
    let strict_smaller_direct = source
        .offset(q(1, 20), &OffsetCornerStyle2::Round, &CurveContext::STRICT)
        .unwrap();
    assert_eq!(strict_partially_reversed.value, strict_smaller_direct.value);

    let approximate_first = source
        .offset(
            q(1, 10),
            &OffsetCornerStyle2::Round,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap()
        .into_value();
    let approximate_repeated = approximate_first
        .offset(
            q(1, 5),
            &OffsetCornerStyle2::Round,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(approximate_repeated.certainty, CurveCertainty::Certified);
    assert_eq!(approximate_repeated.value, strict_direct.value);
    for fragment in approximate_repeated.value.boundary_loops()[0].fragments() {
        let BezierSplitFragment2::AnalyticParallel(fragment) = fragment else {
            panic!("the composed non-PH quadratic parallel must stay analytic");
        };
        assert_eq!(fragment.parallel().distance(), &-q(3, 10));
    }
}

#[test]
fn unified_region_offsets_general_rational_boundary_identically_under_both_policies() {
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[rational_cap_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let strict = source
        .offset(
            Real::one(),
            &OffsetCornerStyle2::Round,
            &CurveContext::STRICT,
        )
        .unwrap();
    let approximate = source
        .offset(
            Real::one(),
            &OffsetCornerStyle2::Round,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();

    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(strict.value, approximate.value);
    assert!(strict.value.has_algebraic_fragments());
    assert_eq!(
        certified(
            strict
                .value
                .classify_point(&p(0, 0), &CurveContext::STRICT)
                .unwrap()
        ),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(
            strict
                .value
                .classify_point(&p(0, 5), &CurveContext::STRICT)
                .unwrap()
        ),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_offset_corner_styles_have_exact_area_and_miter_fallback() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy)
        .unwrap()
        .into_value();
    let round = source
        .offset(Real::one(), &OffsetCornerStyle2::Round, &policy)
        .unwrap()
        .into_value();
    let bevel = source
        .offset(Real::one(), &OffsetCornerStyle2::Bevel, &policy)
        .unwrap()
        .into_value();
    let limited_miter = source
        .offset(
            Real::one(),
            &OffsetCornerStyle2::Miter { limit: Real::one() },
            &policy,
        )
        .unwrap()
        .into_value();
    let miter = source
        .offset(Real::one(), &sharp_offset(), &policy)
        .unwrap()
        .into_value();

    let round_native = decided(round.native_contours_fast_path(&policy).unwrap());
    let round_area = round_native.material_contours()[0]
        .signed_area()
        .unwrap()
        .unwrap();
    assert_eq!(
        round_area
            .certified_eq_until(&(Real::from(32) + Real::pi()), -512)
            .as_bool(),
        Some(true)
    );
    assert_eq!(
        decided(bevel.filled_area(&policy).unwrap()),
        Some(Real::from(34))
    );
    assert_eq!(limited_miter, bevel);
    assert_eq!(
        decided(miter.filled_area(&policy).unwrap()),
        Some(Real::from(36))
    );
    assert_eq!(
        certified(miter.classify_point(&p(-1, -1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        certified(round.classify_point(&p(-1, -1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_reuses_design_parameter_corner_solvers() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy)
        .unwrap()
        .into_value();

    let CurveCornerSolutions2::Unique(chamfer) = source
        .chamfer_loop_vertex_by_setbacks(
            0,
            1,
            Real::one(),
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &policy,
        )
        .unwrap()
        .into_value()
    else {
        panic!("a square vertex must have one trim-only chamfer");
    };
    assert_eq!(
        decided(chamfer.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    let chamfer_paths = decided(chamfer.materialized_boundary_paths(&policy).unwrap());
    assert_eq!(chamfer_paths[0].curves().len(), 5);
    assert_eq!(chamfer_paths[0].curves()[0].end(), &p(3, 0));
    assert_eq!(chamfer_paths[0].curves()[1].end(), &p(4, 1));

    let CurveCornerSolutions2::Unique(fillet) = source
        .fillet_loop_vertex_by_radius(0, 1, Real::one(), CurveCornerMode2::TrimOnly, &policy)
        .unwrap()
        .into_value()
    else {
        panic!("a square vertex must have one trim-only fillet");
    };
    assert_eq!(
        decided(fillet.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    let fillet_native = decided(fillet.native_contours_fast_path(&policy).unwrap());
    assert_eq!(fillet_native.material_contours()[0].segments().len(), 5);
    let Segment2::Arc(arc) = &fillet_native.material_contours()[0].segments()[1] else {
        panic!("the region fillet must recover its exact circular carrier");
    };
    assert_eq!(arc.start(), &p(3, 0));
    assert_eq!(arc.end(), &p(4, 1));
    assert_eq!(arc.center(), &p(3, 1));
    assert_eq!(arc.radius_squared(), Real::one());
    assert!(!arc.is_clockwise());

    let CurveCornerSolutions2::Multiple(extended) = source
        .fillet_loop_vertex_by_radius(0, 1, Real::one(), CurveCornerMode2::TrimOrExtend, &policy)
        .unwrap()
        .into_value()
    else {
        panic!("the region must preserve both exact trim-or-extend candidates");
    };
    assert_eq!(extended.len(), 2);

    let CurveCornerSolutions2::Unique(one_sided) = source
        .chamfer_loop_vertex_by_setbacks(
            0,
            1,
            Real::zero(),
            Real::one(),
            CurveCornerMode2::TrimOnly,
            &policy,
        )
        .unwrap()
        .into_value()
    else {
        panic!("a one-sided zero setback must retain its nondegenerate chamfer");
    };
    assert_eq!(
        decided(one_sided.native_contours_fast_path(&policy).unwrap()).material_contours()[0]
            .segments()
            .len(),
        5
    );
    assert_eq!(
        source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                Real::zero(),
                Real::zero(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );
    assert_eq!(
        source
            .fillet_loop_vertex_by_radius(0, 1, Real::zero(), CurveCornerMode2::TrimOnly, &policy,)
            .unwrap()
            .into_value(),
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );
}

#[test]
fn unified_region_native_chamfer_uses_arc_sweep_evidence() {
    let rounded = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(p(4, 0), p(5, 1), p(4, 1), false).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(5, 1), p(5, 4)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(5, 4), p(0, 4)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(0, 4), p(0, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source =
            CurveRegion2::try_from_native_material_contours(vec![rounded.clone()], &policy)
                .unwrap()
                .into_value();
        let CurveCornerSolutions2::Unique(chamfered) = source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                q(1, 2),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the native line-arc vertex must have one exact chamfer");
        };
        let native = decided(chamfered.native_contours_fast_path(&policy).unwrap());
        let segments = native.material_contours()[0].segments();
        assert_eq!(segments.len(), 6);
        let Segment2::Line(previous) = &segments[0] else {
            panic!("the previous native line must remain a line");
        };
        let Segment2::Line(chamfer) = &segments[1] else {
            panic!("the inserted native chamfer must be a line");
        };
        let Segment2::Arc(next) = &segments[2] else {
            panic!("the next native arc must remain an arc");
        };
        let previous_cut = Point2::new(q(7, 2), Real::zero());
        assert_eq!(previous.end(), &previous_cut);
        assert_eq!(chamfer.start(), &previous_cut);
        assert_eq!(chamfer.end(), next.start());
        assert_eq!(next.center(), &p(4, 1));
        assert_eq!(next.end(), &p(5, 1));
        assert_eq!(
            next.start()
                .distance_squared(&p(4, 0))
                .certified_eq_until(&Real::one(), -4096)
                .as_bool(),
            Some(true)
        );
    }
}

#[test]
fn unified_region_native_fillet_retains_certified_arc_contacts() {
    let curved = Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(1, 1), p(-2, 1)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(-2, 1), p(-2, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurveRegion2::try_from_native_material_contours(vec![curved.clone()], &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Unique(filleted) = source
            .fillet_loop_vertex_by_radius(0, 1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the native line/arc vertex must have one exact fillet");
        };
        let native = decided(filleted.native_contours_fast_path(&policy).unwrap());
        let segments = native.material_contours()[0].segments();
        assert_eq!(segments.len(), 5);
        let Segment2::Line(previous) = &segments[0] else {
            panic!("the previous native line must remain a line");
        };
        let Segment2::Arc(fillet) = &segments[1] else {
            panic!("the inserted fillet must remain a circular arc");
        };
        let Segment2::Arc(next) = &segments[2] else {
            panic!("the next native arc must remain a circular arc");
        };
        assert_eq!(previous.end(), fillet.start());
        assert_eq!(fillet.end(), next.start());
        assert_eq!(next.center(), &p(1, 0));
        assert_eq!(next.end(), &p(1, 1));
        assert_eq!(
            fillet
                .radius_squared()
                .certified_eq_until(&q(1, 4), -4096)
                .as_bool(),
            Some(true)
        );
        let expected_center = Point2::new(Real::one() - Real::from(2).sqrt().unwrap(), q(1, 2));
        assert_eq!(
            fillet
                .center()
                .distance_squared(&expected_center)
                .certified_eq_until(&Real::zero(), -4096)
                .as_bool(),
            Some(true)
        );
    }
}

#[test]
fn unified_region_corners_use_rational_circular_carriers() {
    let native_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap();
    let conic = native_arc
        .rational_bezier_decomposition(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .spans()[0]
        .curve()
        .clone();
    let elevated = RationalBezier2::from(conic.clone())
        .elevated_to_degree(5)
        .unwrap();
    let carriers = [Curve2::from(conic), Curve2::from(elevated)];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for carrier in &carriers {
            let path = CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(-2, 0), p(0, 0)).unwrap()),
                carrier.clone(),
                Curve2::from(LineSeg2::try_new(p(1, 1), p(-2, 1)).unwrap()),
                Curve2::from(LineSeg2::try_new(p(-2, 1), p(-2, 0)).unwrap()),
            ])
            .unwrap();
            let source = CurveRegion2::try_from_boundary_paths(&[path], &policy)
                .unwrap()
                .into_value();

            let CurveCornerSolutions2::Unique(chamfered) = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    q(1, 2),
                    q(1, 2),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the retained circular region corner must have one chamfer");
            };
            let chamfer_paths = decided(chamfered.materialized_boundary_paths(&policy).unwrap());
            assert!(
                chamfer_paths[0]
                    .curves()
                    .iter()
                    .any(|curve| curve.family() == CurveFamily2::RationalQuadraticBezier)
            );

            let CurveCornerSolutions2::Unique(filleted) = source
                .fillet_loop_vertex_by_radius(0, 1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value()
            else {
                panic!("the retained circular region corner must have one fillet");
            };
            let fillet_paths = decided(filleted.materialized_boundary_paths(&policy).unwrap());
            assert_eq!(fillet_paths[0].curves().len(), 5);
            assert!(
                fillet_paths[0]
                    .curves()
                    .iter()
                    .any(|curve| curve.family() == CurveFamily2::RationalQuadraticBezier)
            );
        }
    }
}

#[test]
fn unified_region_corners_use_represented_bezier_incidence() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Unique(filleted) = source
            .fillet_loop_vertex_by_radius(0, 1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
            .unwrap()
            .into_value()
        else {
            panic!("the represented line/Bezier region corner must have one fillet");
        };
        let fillet_paths = decided(filleted.materialized_boundary_paths(&policy).unwrap());
        assert_eq!(fillet_paths[0].curves().len(), 5);
        assert_eq!(
            fillet_paths[0].curves()[2].family(),
            CurveFamily2::QuadraticBezier
        );

        let next_setback = (Real::from(657).sqrt().unwrap() / Real::from(16)).unwrap();
        let CurveCornerSolutions2::Unique(chamfered) = source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                Real::one(),
                next_setback,
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the represented line/Bezier region corner must have one chamfer");
        };
        let chamfer_paths = decided(chamfered.materialized_boundary_paths(&policy).unwrap());
        assert_eq!(chamfer_paths[0].curves().len(), 5);
        assert_eq!(
            chamfer_paths[0].curves()[2].family(),
            CurveFamily2::QuadraticBezier
        );
    }
}

#[test]
fn unified_region_chamfer_retains_algebraic_bezier_cut() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Unique(chamfered) = source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the algebraic line/Bezier setback must have one retained chamfer");
        };
        let fragments = chamfered.boundary_loops()[0].fragments();
        assert_eq!(fragments.len(), 5);
        assert!(matches!(
            fragments[1],
            BezierSplitFragment2::AlgebraicChord(_)
        ));
        assert!(matches!(
            fragments[2],
            BezierSplitFragment2::AlgebraicEndpointImages { .. }
        ));
        assert_eq!(
            decided(chamfered.loop_roles(&policy).unwrap()),
            vec![CurveRegionLoopRole::Material]
        );
        #[cfg(feature = "predicates")]
        {
            assert_eq!(
                certified(chamfered.classify_point(&p(-2, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Inside)
            );
            assert_eq!(
                certified(chamfered.classify_point(&p(0, 0), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside)
            );
            assert_eq!(
                certified(chamfered.classify_point(&p(-1, 0), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Boundary)
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn unified_region_chamfer_reenters_general_algebraic_chords() {
    let bottom = Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap());
    let curved = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)));
    let top = Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap());
    let left = Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap());
    let paths = [
        (
            CurvePath2::try_new(vec![
                bottom.clone(),
                curved.clone(),
                top.clone(),
                left.clone(),
            ])
            .unwrap(),
            [1, 1, 2],
            false,
        ),
        (
            CurvePath2::try_new(vec![curved, top, left, bottom]).unwrap(),
            [0, 0, 1],
            false,
        ),
        (
            CurvePath2::try_new(vec![
                Curve2::from(LineSeg2::try_new(p(-4, 0), p(-4, 2)).unwrap()),
                Curve2::from(LineSeg2::try_new(p(-4, 2), p(1, 2)).unwrap()),
                Curve2::from(QuadraticBezier2::new(p(1, 2), p(0, 1), p(0, 0))),
                Curve2::from(LineSeg2::try_new(p(0, 0), p(-4, 0)).unwrap()),
            ])
            .unwrap(),
            [3, 4, 4],
            true,
        ),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (path, vertices, chord_is_previous) in &paths {
            let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(path), &policy)
                .unwrap()
                .into_value();
            let first = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertices[0],
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(first.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(first) = first.into_value() else {
                panic!("the first Bezier setback must retain one algebraic chord");
            };

            let one_sided = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertices[1],
                    Real::zero(),
                    q(1, 4),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(one_sided.certainty, CurveCertainty::Certified);
            assert!(matches!(
                one_sided.into_value(),
                CurveCornerSolutions2::Unique(_)
            ));
            let (over_previous, over_next) = if *chord_is_previous {
                (Real::from(2), q(1, 4))
            } else {
                (q(1, 4), Real::from(2))
            };
            let over = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertices[1],
                    over_previous,
                    over_next,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(over.certainty, CurveCertainty::Certified);
            assert_eq!(
                over.into_value(),
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
            );

            // The next edit meets a represented line and the retained general
            // chord. Its chord-side cut is a lazy exact unit-tangent displacement.
            let second = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertices[1],
                    q(1, 4),
                    q(1, 4),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(second.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(second) = second.into_value() else {
                panic!("the materialized/algebraic-chord corner must have one chamfer");
            };
            assert_eq!(second.boundary_loops()[0].fragments().len(), 6);
            assert_eq!(
                second.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                    .count(),
                2
            );

            // The inserted chord and retained source chord now meet directly.
            // Both have independently selected endpoints and neither requires a
            // represented unit tangent.
            let third = second
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertices[2],
                    q(1, 10),
                    q(1, 10),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap();
            assert_eq!(third.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(third) = third.into_value() else {
                panic!("the algebraic-chord/algebraic-chord corner must have one chamfer");
            };
            assert_eq!(third.boundary_loops()[0].fragments().len(), 7);
            assert_eq!(
                third.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                    .count(),
                3
            );
            assert_eq!(
                decided(third.loop_roles(&policy).unwrap()),
                vec![CurveRegionLoopRole::Material]
            );
            assert_eq!(
                certified(third.classify_point(&p(-2, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Inside)
            );
            assert_eq!(
                certified(third.classify_point(&p(0, 0), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside)
            );

            let distant = CurveRegion2::try_from_native_material_contours(
                vec![square(10, 10, 12, 12)],
                &policy,
            )
            .unwrap()
            .into_value();
            let batch = third
                .boolean_regions(&distant, &policy)
                .unwrap()
                .into_value();
            assert!(batch.intersection().is_empty());
            assert_eq!(batch.union().boundary_loops().len(), 2);
            assert_eq!(batch.difference().boundary_loops().len(), 1);
            assert_eq!(batch.xor().boundary_loops().len(), 2);
        }
    }
}

#[test]
fn unified_region_chamfer_joins_two_algebraic_bezier_cuts() {
    let previous = Curve2::from(QuadraticBezier2::new(p(-1, 2), p(0, 1), p(0, 0)));
    let next = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)));
    let close = Curve2::from(LineSeg2::try_new(p(1, 2), p(-1, 2)).unwrap());
    let paths = [
        (
            CurvePath2::try_new(vec![previous.clone(), next.clone(), close.clone()]).unwrap(),
            1,
        ),
        (CurvePath2::try_new(vec![next, close, previous]).unwrap(), 0),
    ];

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for (path, vertex_index) in &paths {
            let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(path), &policy)
                .unwrap()
                .into_value();
            let CurveCornerSolutions2::Unique(chamfered) = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    *vertex_index,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("two algebraic Bezier setbacks must define one retained chamfer");
            };
            let fragments = chamfered.boundary_loops()[0].fragments();
            assert_eq!(fragments.len(), 4);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                    .count(),
                1
            );
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| matches!(
                        fragment,
                        BezierSplitFragment2::AlgebraicEndpointImages { .. }
                    ))
                    .count(),
                2
            );
            let distant = CurveRegion2::try_from_native_material_contours(
                vec![square(10, 10, 12, 12)],
                &policy,
            )
            .unwrap()
            .into_value();
            let evidence = chamfered
                .intersect_region(&distant, &policy)
                .unwrap()
                .into_value();
            assert!(evidence.is_disjoint());
            assert_eq!(evidence.candidate_carrier_pair_count(), 0);
            let batch = chamfered
                .boolean_regions(&distant, &policy)
                .unwrap()
                .into_value();
            assert!(batch.intersection().is_empty());
            assert_eq!(batch.union().boundary_loops().len(), 2);
            assert_eq!(batch.difference().boundary_loops().len(), 1);
            #[cfg(feature = "predicates")]
            {
                assert_eq!(
                    certified(chamfered.classify_point(&p(0, 1), &policy).unwrap()),
                    Classification::Decided(RegionPointLocation::Inside)
                );
                assert_eq!(
                    certified(chamfered.classify_point(&p(0, 0), &policy).unwrap()),
                    Classification::Decided(RegionPointLocation::Outside)
                );
                assert_eq!(
                    certified(chamfered.classify_point(&p(-1, 2), &policy).unwrap()),
                    Classification::Decided(RegionPointLocation::Boundary)
                );
            }
        }
    }
}

#[test]
fn algebraic_chamfer_participates_in_a_disjoint_boolean_batch() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Unique(chamfered) = source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the algebraic line/Bezier setback must have one retained chamfer");
        };
        let distant =
            CurveRegion2::try_from_native_material_contours(vec![square(10, 10, 12, 12)], &policy)
                .unwrap()
                .into_value();

        let evidence = chamfered
            .intersect_region(&distant, &policy)
            .unwrap()
            .into_value();
        assert!(evidence.is_disjoint());
        assert_eq!(evidence.candidate_carrier_pair_count(), 0);

        let batch = chamfered
            .boolean_regions(&distant, &policy)
            .unwrap()
            .into_value();
        assert!(batch.intersection().is_empty());
        assert_eq!(batch.union().boundary_loops().len(), 2);
        assert_eq!(batch.difference().boundary_loops().len(), 1);
        assert_eq!(batch.xor().boundary_loops().len(), 2);
        assert!(
            batch.difference().boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn one_field_algebraic_chamfer_regularizes_without_rebuilding_its_solver() {
    let path = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), &policy)
            .unwrap()
            .into_value();
        let CurveCornerSolutions2::Unique(chamfered) = source
            .chamfer_loop_vertex_by_setbacks(
                0,
                1,
                Real::one(),
                Real::one(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .unwrap()
            .into_value()
        else {
            panic!("the algebraic line/Bezier setback must have one retained chamfer");
        };
        let regularized = chamfered.regularized_region(&policy).unwrap().into_value();
        assert_eq!(regularized.boundary_loops().len(), 1);
        assert!(
            regularized.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
        );
        assert_eq!(
            certified(regularized.classify_point(&p(-2, 1), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn unified_region_corners_use_canonical_spline_bezier_spans() {
    let controls = vec![p(0, 0), p(0, 1), p(1, 2)];
    let knots = vec![
        Real::from(2),
        Real::from(2),
        Real::from(2),
        Real::from(5),
        Real::from(5),
        Real::from(5),
    ];
    let carriers = [
        (
            CurveFamily2::PolynomialBSpline,
            Curve2::try_polynomial_bspline(
                2,
                controls.clone(),
                knots.clone(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
        (
            CurveFamily2::Nurbs,
            Curve2::try_nurbs(
                2,
                controls,
                vec![Real::one(); 3],
                knots,
                &CurveContext::STRICT,
            )
            .unwrap()
            .into_value(),
        ),
    ];
    let expected_cut = Point2::new(q(9, 16), q(3, 2));
    let expected_line_cut = Point2::new(-q(39, 16), Real::zero());
    let next_setback = (Real::from(657).sqrt().unwrap() / Real::from(16)).unwrap();

    for (family, carrier) in carriers {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap()),
            carrier,
            Curve2::from(LineSeg2::try_new(p(1, 2), p(-4, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-4, 2), p(-4, 0)).unwrap()),
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source =
                CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path), &policy)
                    .unwrap()
                    .into_value();
            let source_paths = decided(source.materialized_boundary_paths(&policy).unwrap());
            let canonical_family = source_paths[0].curves()[1].family();
            assert_eq!(
                canonical_family,
                match family {
                    CurveFamily2::PolynomialBSpline => CurveFamily2::QuadraticBezier,
                    CurveFamily2::Nurbs => CurveFamily2::RationalQuadraticBezier,
                    _ => unreachable!(),
                }
            );
            let CurveCornerSolutions2::Unique(chamfered) = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    Real::one(),
                    next_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap()
                .into_value()
            else {
                panic!("the {family:?} region span must define one exact chamfer");
            };
            let chamfer_paths = decided(chamfered.materialized_boundary_paths(&policy).unwrap());
            assert_eq!(chamfer_paths[0].curves()[2].family(), canonical_family);
            assert_eq!(chamfer_paths[0].curves()[2].start(), &expected_cut);

            let fillets = source
                .fillet_loop_vertex_by_radius(0, 1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
                .unwrap()
                .into_value();
            let has_expected = |candidate: &CurveRegion2| {
                let paths = decided(candidate.materialized_boundary_paths(&policy).unwrap());
                paths[0].curves()[2].family() == canonical_family
                    && paths[0].curves()[1].family() == CurveFamily2::RationalQuadraticBezier
                    && paths[0].curves()[0]
                        .end()
                        .distance_squared(&expected_line_cut)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
                    && paths[0].curves()[2]
                        .start()
                        .distance_squared(&expected_cut)
                        .certified_eq_until(&Real::zero(), -4096)
                        .as_bool()
                        == Some(true)
            };
            match &fillets {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the {family:?} region span lost its exact fillet: {reason:?}")
                }
            }
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn unified_region_corner_solver_obeys_terminal_policy_once() {
    let source = CurveRegion2::try_from_native_material_contours(
        vec![square(0, 0, 4, 4)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    assert!(matches!(
        source.fillet_loop_vertex_by_radius(
            0,
            1,
            undecidable_zero.clone(),
            CurveCornerMode2::TrimOnly,
            &CurveContext::STRICT,
        ),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::RealSign
    ));
    let approximate = source
        .fillet_loop_vertex_by_radius(
            0,
            1,
            undecidable_zero,
            CurveCornerMode2::TrimOnly,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        approximate.value,
        CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::ZeroDesignValue)
    );
}

#[test]
fn unified_region_offset_corner_options_obey_the_terminal_policy() {
    let source = CurveRegion2::try_from_native_material_contours(
        vec![square(0, 0, 4, 4)],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert!(matches!(
            source.offset(
                Real::one(),
                &OffsetCornerStyle2::Miter {
                    limit: -Real::one(),
                },
                &policy,
            ),
            Err(ExactCurveError::Invalid {
                cause: CurveError::InvalidOffsetOptions,
                ..
            })
        ));
    }

    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let style = OffsetCornerStyle2::Miter {
        limit: undecidable_zero,
    };
    assert!(matches!(
        source.offset(Real::one(), &style, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::RealSign
    ));
    let approximate = source
        .offset(Real::one(), &style, &CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        decided(
            approximate
                .value
                .filled_area(&CurveContext::STRICT)
                .unwrap()
        ),
        Some(Real::from(34))
    );
}

#[cfg(feature = "predicates")]
#[test]
fn axis_aligned_algebraic_chords_reenter_exact_region_offsets() {
    let distance = q(1, 10);
    let miter = OffsetCornerStyle2::Miter {
        limit: Real::from(2),
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let expanded = source
            .offset(distance.clone(), &miter, &policy)
            .expect("axis-aligned algebraic expansion must remain exact");
        assert_eq!(expanded.certainty, CurveCertainty::Certified);
        let expanded = expanded.value;
        assert_eq!(expanded.boundary_loops().len(), 1);
        assert!(
            expanded.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
        );
        assert_eq!(
            certified(
                expanded
                    .classify_point(&Point2::new(-q(1, 20), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(
                expanded
                    .classify_point(&Point2::new(-q(1, 5), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            certified(
                expanded
                    .classify_point(&Point2::new(Real::zero(), -distance.clone()), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Boundary)
        );

        let repeated = expanded
            .offset(distance.clone(), &miter, &policy)
            .expect("translated algebraic endpoint expressions must compose exactly");
        assert_eq!(repeated.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                repeated
                    .value
                    .classify_point(&Point2::new(-q(3, 20), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );

        let contracted = source
            .offset(-distance.clone(), &miter, &policy)
            .expect("axis-aligned algebraic contraction must remain exact");
        assert_eq!(contracted.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                contracted
                    .value
                    .classify_point(&Point2::new(q(1, 20), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            certified(
                contracted
                    .value
                    .classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );

        let beveled = source
            .offset(distance.clone(), &OffsetCornerStyle2::Bevel, &policy)
            .expect("algebraic bevel joins must remain exact");
        assert_eq!(beveled.certainty, CurveCertainty::Certified);
        assert!(beveled.value.boundary_loops()[0].fragments().len() >= 8);

        let limited_miter = source
            .offset(
                distance.clone(),
                &OffsetCornerStyle2::Miter { limit: Real::one() },
                &policy,
            )
            .expect("an exceeded algebraic miter limit must fall back to exact bevels");
        assert_eq!(limited_miter.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                limited_miter
                    .value
                    .classify_point(&Point2::new(-q(9, 100), -q(9, 100)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Outside)
        );

        let collapsed = source
            .offset(-q(2, 5), &miter, &policy)
            .expect("an algebraic offset past the first collapse must regularize exactly");
        assert_eq!(collapsed.certainty, CurveCertainty::Certified);
        assert!(collapsed.value.is_empty());

        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let round_offset = || source.offset(distance.clone(), &OffsetCornerStyle2::Round, &policy);
        #[cfg(feature = "dispatch-trace")]
        let rounded = hyperreal::dispatch_trace::with_recording(round_offset);
        #[cfg(not(feature = "dispatch-trace"))]
        let rounded = round_offset();
        let rounded = rounded.expect("selected-field algebraic round joins must remain exact");
        #[cfg(feature = "dispatch-trace")]
        {
            let trace = hyperreal::dispatch_trace::take_trace();
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "adjacent-circular-endpoint-only",
                ),
                4
            );
            assert_eq!(
                trace.path_count("hypercurve", "algebraic-chord-pair", "general-rational",),
                0
            );
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-circle-rational-pair",
                    "bounds-disjoint",
                ),
                2
            );
            for path in [
                "nonadjacent-line",
                "nonadjacent-circle",
                "nonadjacent-general",
            ] {
                assert_eq!(
                    trace.path_count("hypercurve", "algebraic-circle-rational-pair", path),
                    0
                );
            }
        }
        assert_eq!(rounded.certainty, CurveCertainty::Certified);
        assert_eq!(rounded.value.boundary_loops().len(), 1);
        assert!(
            rounded.value.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        for (point, expected) in [
            (
                Point2::new(-q(1, 20), -q(1, 20)),
                RegionPointLocation::Inside,
            ),
            (
                Point2::new(-q(9, 100), -q(9, 100)),
                RegionPointLocation::Outside,
            ),
            (
                Point2::new(Real::zero(), -distance.clone()),
                RegionPointLocation::Boundary,
            ),
        ] {
            assert_eq!(
                certified(rounded.value.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected)
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_algebraic_round_joins_reenter_exact_region_offsets() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let rounded = source
            .offset(q(1, 10), &OffsetCornerStyle2::Round, &policy)
            .expect("the first selected-field round offset must remain exact")
            .into_value();

        let expanded = rounded
            .offset(q(1, 20), &OffsetCornerStyle2::Round, &policy)
            .expect("retained selected circles must support a second exact offset");
        assert_eq!(expanded.certainty, CurveCertainty::Certified);
        assert!(
            expanded.value.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        assert_eq!(
            certified(
                expanded
                    .value
                    .classify_point(&Point2::new(Real::zero(), -q(3, 20)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );
        assert!(
            expanded.value.boundary_loops()[0]
                .arrangement_sources()
                .is_some()
        );
        let expanded_again = expanded
            .value
            .offset(q(1, 100), &OffsetCornerStyle2::Round, &policy)
            .expect("a certified convex selected-circle parallel must remain reusable");
        assert_eq!(expanded_again.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                expanded_again
                    .value
                    .classify_point(&Point2::new(q(1, 4), -q(4, 25)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );

        let contracted = rounded
            .offset(-q(1, 20), &OffsetCornerStyle2::Round, &policy)
            .expect("a retained selected circle must contract before its radius collapses");
        assert_eq!(contracted.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                contracted
                    .value
                    .classify_point(&Point2::new(Real::zero(), -q(1, 20)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );

        let collapsed_round = rounded
            .offset(-q(1, 10), &OffsetCornerStyle2::Round, &policy)
            .expect("an exact selected-circle radius collapse must remove only the arc")
            .into_value();
        assert!(
            collapsed_round.boundary_loops()[0]
                .fragments()
                .iter()
                .all(|fragment| !matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        assert_eq!(
            certified(
                collapsed_round
                    .classify_point(&Point2::new(q(1, 4), Real::zero()), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );

        let past_collapse = rounded
            .offset(-q(3, 20), &OffsetCornerStyle2::Round, &policy)
            .expect("a selected-circle parallel past its local collapse must regularize exactly");
        assert_eq!(
            past_collapse.certainty,
            if policy == CurveContext::STRICT {
                CurveCertainty::Certified
            } else {
                CurveCertainty::Approximate512Consumed
            },
        );
        for (point, expected) in [
            (
                Point2::new(q(1, 4), q(1, 20)),
                RegionPointLocation::Boundary,
            ),
            (Point2::new(q(1, 4), q(1, 40)), RegionPointLocation::Outside),
            (Point2::new(q(1, 4), q(1, 4)), RegionPointLocation::Inside),
        ] {
            assert_eq!(
                certified(past_collapse.value.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected),
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn algebraic_chords_and_round_centers_survive_exact_similarities() {
    let quarter_turn = Similarity2::try_from_real_affine(
        Real::zero(),
        Real::from(-1),
        Real::one(),
        Real::zero(),
        Real::from(2),
        Real::from(3),
    )
    .unwrap();
    let distance = q(1, 20);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let transformed = source
            .transform_similarity(&quarter_turn, &policy)
            .expect("a nonsingular exact affine map must retain selected chord fields");
        assert_eq!(transformed.certainty, CurveCertainty::Certified);
        assert_eq!(
            transformed.value.boundary_loops()[0]
                .fragments()
                .iter()
                .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                .count(),
            4
        );
        for (point, expected) in [
            (Point2::new(q(3, 2), q(13, 4)), RegionPointLocation::Inside),
            (Point2::new(q(3, 2), q(15, 4)), RegionPointLocation::Outside),
        ] {
            assert_eq!(
                certified(transformed.value.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected),
            );
        }

        let transformed_round = transformed
            .value
            .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("axis certificates must survive a cardinal similarity");
        assert_eq!(transformed_round.certainty, CurveCertainty::Certified);
        assert!(
            transformed_round.value.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        let transformed_boundary = Point2::new(Real::from(2) + &distance, q(13, 4));
        assert_eq!(
            certified(
                transformed_round
                    .value
                    .classify_point(&transformed_boundary, &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );

        let rounded = source
            .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the selected-field round source must complete");
        let rotated_round = rounded
            .value
            .transform_similarity(&quarter_turn, &policy)
            .expect("direct selected circle centers must transform in their retained field");
        assert_eq!(rotated_round.certainty, CurveCertainty::Certified);
        assert!(
            rotated_round.value.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        assert_eq!(
            certified(
                rotated_round
                    .value
                    .classify_point(&transformed_boundary, &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn translated_algebraic_round_regions_boolean_through_cusp_chord_contacts() {
    let radius = q(1, 20);
    let translation = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        radius.clone(),
        q(1, 40),
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let first = source
            .offset(radius.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("first selected round region must remain exact")
            .into_value();
        let second = first
            .transform_similarity(&translation, &policy)
            .expect("translated selected round region must remain exact")
            .into_value();
        let evidence = first
            .intersect_region(&second, &policy)
            .expect("translated round boundaries must intersect exactly")
            .into_value();
        assert!(evidence.is_complete(), "{evidence:?}");
        assert!(!evidence.contacts().is_empty(), "{evidence:?}");
        assert!(
            evidence.contacts().iter().any(|contact| matches!(
                contact.point(),
                Some(RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_))
            )),
            "the translated round regions must exercise retained cusp/chord contacts: {evidence:?}",
        );

        let batch = first
            .boolean_regions(&second, &policy)
            .expect("translated selected round regions must Boolean exactly");
        assert_eq!(batch.certainty, CurveCertainty::Certified);
        assert!(!batch.value.intersection().is_empty());
        assert!(!batch.value.union().is_empty());
        assert!(!batch.value.difference().is_empty());
        assert_eq!(
            certified(
                batch
                    .value
                    .union()
                    .classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside),
        );

        let third = second
            .transform_similarity(&translation, &policy)
            .expect("a second exact translation must retain selected round evidence")
            .into_value();
        let replay = batch
            .value
            .intersection()
            .boolean_regions(&third, &policy)
            .expect("a split cusp/chord contact must re-enter a later Boolean exactly");
        assert_eq!(replay.certainty, CurveCertainty::Certified);
        assert!(!replay.value.intersection().is_empty());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn rotated_algebraic_round_regions_boolean_through_oblique_three_field_contacts() {
    let radius = q(1, 20);
    let rotation = Similarity2::try_from_real_affine(
        q(3, 5),
        -q(4, 5),
        q(4, 5),
        q(3, 5),
        Real::zero(),
        Real::zero(),
    )
    .unwrap();
    // Rotate the cardinal translation `(radius, radius / 2)` with the source.
    let translated_in_rotated_frame = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        q(1, 100),
        q(11, 200),
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let rounded = axis_aligned_algebraic_rectangle(&policy)
            .offset(radius.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the selected round region must remain exact")
            .into_value();
        let first = rounded
            .transform_similarity(&rotation, &policy)
            .expect("a rational rotation must retain all selected fields")
            .into_value();
        let second = first
            .transform_similarity(&translated_in_rotated_frame, &policy)
            .expect("the rotated selected fields must survive translation")
            .into_value();

        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let intersection_work = || first.intersect_region(&second, &policy);
        #[cfg(feature = "dispatch-trace")]
        let evidence = hyperreal::dispatch_trace::with_recording(intersection_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let evidence = intersection_work();
        let evidence = evidence
            .expect("rotated round boundaries must intersect through the exact oblique kernel");
        assert_eq!(evidence.certainty, CurveCertainty::Certified);
        let evidence = evidence.into_value();
        assert!(evidence.is_complete(), "{evidence:?}");
        assert!(
            evidence.contacts().iter().any(|contact| matches!(
                contact.point(),
                Some(RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_))
            )),
            "the rotated round regions must retain an oblique cusp/chord contact: {evidence:?}",
        );
        #[cfg(feature = "dispatch-trace")]
        {
            let trace = hyperreal::dispatch_trace::take_trace();
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-circle-chord-kernel",
                    "general-algebraic-oblique",
                ) > 0,
                "the public rotated-region path must enter the general algebraic kernel: {trace:?}",
            );
        }

        let batch = first
            .boolean_regions(&second, &policy)
            .expect("rotated selected round regions must Boolean exactly");
        assert_eq!(batch.certainty, CurveCertainty::Certified);
        assert!(!batch.value.union().is_empty());
        assert!(!batch.value.intersection().is_empty());
        assert!(!batch.value.difference().is_empty());
        assert!(!batch.value.xor().is_empty());
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let reoffset_work = || {
            batch
                .value
                .intersection()
                .offset(q(1, 500), &OffsetCornerStyle2::Bevel, &policy)
        };
        #[cfg(feature = "dispatch-trace")]
        let reoffset = hyperreal::dispatch_trace::with_recording(reoffset_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let reoffset = reoffset_work();
        let reoffset =
            reoffset.expect("an oblique retained cusp/chord boundary must re-offset exactly");
        assert_eq!(reoffset.certainty, CurveCertainty::Certified);
        assert!(!reoffset.value.is_empty());
        #[cfg(feature = "dispatch-trace")]
        {
            let trace = hyperreal::dispatch_trace::take_trace();
            assert!(
                trace.path_count(
                    "hypercurve",
                    "curve-region-exact-offset-span",
                    "certified-oblique-algebraic-chord",
                ) > 0,
                "the reoffset must retain its oblique chord fast path: {trace:?}",
            );
            assert!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-circle-chord-pair",
                    "adjacent-endpoint-only",
                ) > 0,
                "the derived bevel endpoint proof must survive regularization: {trace:?}",
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn cusp_chord_boolean_boundary_reoffsets_with_exact_bevels() {
    let radius = q(1, 20);
    let translation = Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        radius.clone(),
        q(1, 40),
    )
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let first = axis_aligned_algebraic_rectangle(&policy)
            .offset(radius.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the first selected round region must remain exact")
            .into_value();
        let second = first
            .transform_similarity(&translation, &policy)
            .expect("the translated selected round region must remain exact")
            .into_value();
        let intersection = first
            .boolean_regions(&second, &policy)
            .expect("the selected round regions must Boolean exactly")
            .into_value()
            .intersection()
            .clone();
        assert!(intersection.boundary_loops().iter().any(|boundary| {
            boundary.fragments().windows(2).any(|pair| {
                matches!(
                    (&pair[0], &pair[1]),
                    (
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                        BezierSplitFragment2::AlgebraicChord(_)
                    ) | (
                        BezierSplitFragment2::AlgebraicChord(_),
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    )
                )
            })
        }));
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::reset();
        let reoffset_work = || intersection.offset(q(1, 100), &OffsetCornerStyle2::Bevel, &policy);
        #[cfg(feature = "dispatch-trace")]
        let reoffset = hyperreal::dispatch_trace::with_recording(reoffset_work);
        #[cfg(not(feature = "dispatch-trace"))]
        let reoffset = reoffset_work();
        let reoffset = reoffset.expect("a retained cusp/chord boundary must re-offset exactly");
        #[cfg(feature = "dispatch-trace")]
        {
            let trace = hyperreal::dispatch_trace::take_trace();
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-chord-source-incidence",
                    "diagonal-deflated",
                ),
                1
            );
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-selected-fiber-projection",
                    "quotient-ring",
                ),
                1
            );
            assert_eq!(
                trace.path_count(
                    "hypercurve",
                    "algebraic-selected-fiber-projection",
                    "general-resultant-fallback",
                ),
                0
            );
        }
        assert_eq!(reoffset.certainty, CurveCertainty::Certified);
        assert!(!reoffset.value.is_empty());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn one_chord_orders_contacts_from_two_selected_round_corners() {
    let radius = q(1, 20);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let rounded = source
            .offset(radius.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the selected-field round source must complete")
            .into_value();
        let tall = source
            .transform_affine(
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &Real::from(3),
                &Real::zero(),
                &Real::from(-1),
                &policy,
            )
            .expect("the tall selected-field cutter source must remain exact")
            .into_value();
        let cutter = tall
            .offset(
                q(1, 40),
                &OffsetCornerStyle2::Miter {
                    limit: Real::from(2),
                },
                &policy,
            )
            .expect("the cutter offset must retain certified axis chords")
            .into_value();

        let evidence = rounded
            .intersect_region(&cutter, &policy)
            .expect("both selected round corners must meet one chord exactly");
        assert_eq!(evidence.certainty, CurveCertainty::Certified);
        let evidence = evidence.into_value();
        let blockers = evidence
            .blockers()
            .iter()
            .map(|blocker| {
                (
                    blocker.first().fragment_index(),
                    blocker.first().family(),
                    blocker.second().fragment_index(),
                    blocker.second().family(),
                    blocker.uncertainty_reason(),
                )
            })
            .collect::<Vec<_>>();
        assert!(evidence.is_complete(), "{blockers:?}");
        let correlated_contacts = evidence
            .contacts()
            .iter()
            .filter(|contact| {
                matches!(
                    contact.point(),
                    Some(RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_))
                )
            })
            .count();
        assert_eq!(correlated_contacts, 2);

        let batch = rounded
            .boolean_regions(&cutter, &policy)
            .expect("the shared chord contacts must enter all four Booleans");
        assert_eq!(batch.certainty, CurveCertainty::Certified);
        assert!(!batch.value.union().is_empty());
        assert!(!batch.value.intersection().is_empty());
        assert!(!batch.value.difference().is_empty());
        assert!(!batch.value.xor().is_empty());
        let retained_correlated_chord_endpoints = batch
            .value
            .intersection()
            .boundary_loops()
            .iter()
            .flat_map(|boundary| boundary.fragments())
            .filter_map(|fragment| match fragment {
                BezierSplitFragment2::AlgebraicChord(chord) => Some(
                    usize::from(matches!(
                        chord.start(),
                        RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                    )) + usize::from(matches!(
                        chord.end(),
                        RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                    )),
                ),
                _ => None,
            })
            .sum::<usize>();
        assert!(retained_correlated_chord_endpoints >= 2);
        let replay_clip =
            CurveRegion2::try_from_native_material_contours(vec![square(-1, 0, 2, 2)], &policy)
                .unwrap()
                .into_value();
        let replay_evidence = batch
            .value
            .intersection()
            .intersect_region(&replay_clip, &policy)
            .expect("the retained strict-interior contacts must enter a later intersection");
        let replay_certainty = if policy == CurveContext::STRICT {
            CurveCertainty::Certified
        } else {
            CurveCertainty::Approximate512Consumed
        };
        assert_eq!(replay_evidence.certainty, replay_certainty);
        let replay_blockers = replay_evidence
            .value
            .blockers()
            .iter()
            .map(|blocker| {
                (
                    blocker.first().fragment_index(),
                    blocker.first().family(),
                    blocker.second().fragment_index(),
                    blocker.second().family(),
                    blocker.uncertainty_reason(),
                )
            })
            .collect::<Vec<_>>();
        assert!(replay_evidence.value.is_complete(), "{replay_blockers:?}");
        let replay = batch
            .value
            .intersection()
            .boolean_regions(&replay_clip, &policy)
            .expect("the retained strict-interior contacts must enter a later Boolean");
        assert_eq!(replay.certainty, replay_certainty);
        assert!(!replay.value.intersection().is_empty());

        let collinear_min_x = -q(1, 40);
        let collinear_corners = [
            Point2::new(collinear_min_x.clone(), Real::zero()),
            Point2::new(Real::from(2), Real::zero()),
            Point2::new(Real::from(2), Real::one()),
            Point2::new(collinear_min_x, Real::one()),
        ];
        let collinear_clip = CurveRegion2::try_from_native_material_contours(
            vec![
                Contour2::try_new(
                    (0..4)
                        .map(|index| {
                            Segment2::Line(
                                LineSeg2::try_new(
                                    collinear_corners[index].clone(),
                                    collinear_corners[(index + 1) % 4].clone(),
                                )
                                .unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap(),
            ],
            &policy,
        )
        .unwrap()
        .into_value();
        let collinear_evidence = batch
            .value
            .intersection()
            .intersect_region(&collinear_clip, &policy)
            .expect("the retained correlated chord must overlap a later exact line");
        assert_eq!(collinear_evidence.certainty, replay_certainty);
        let collinear_blockers = collinear_evidence
            .value
            .blockers()
            .iter()
            .map(|blocker| {
                (
                    blocker.first().fragment_index(),
                    blocker.first().family(),
                    blocker.second().fragment_index(),
                    blocker.second().family(),
                    blocker.uncertainty_reason(),
                )
            })
            .collect::<Vec<_>>();
        assert!(
            collinear_evidence.value.is_complete(),
            "{collinear_blockers:?}"
        );
        assert!(!collinear_evidence.value.overlaps().is_empty());
        let collinear_replay = batch
            .value
            .intersection()
            .boolean_regions(&collinear_clip, &policy)
            .expect("the retained correlated overlap must enter all four later Booleans");
        assert_eq!(collinear_replay.certainty, replay_certainty);
        assert!(!collinear_replay.value.union().is_empty());
        assert!(!collinear_replay.value.intersection().is_empty());
        if policy == CurveContext::STRICT {
            assert_eq!(
                certified(
                    batch
                        .value
                        .intersection()
                        .classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy)
                        .unwrap(),
                ),
                Classification::Decided(RegionPointLocation::Inside),
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_algebraic_cusp_chamfers_use_the_unified_retained_kernel() {
    let setback = q(1, 100);
    let repeated_setback = q(1, 200);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let rounded = || {
            axis_aligned_algebraic_rectangle(&policy)
                .offset(q(1, 10), &OffsetCornerStyle2::Round, &policy)
                .expect("selected-field round joins must remain exact")
                .into_value()
        };
        for cusp_is_next in [true, false] {
            let source = rounded();
            let fragments = source.boundary_loops()[0].fragments();
            let cusp_index = fragments
                .iter()
                .enumerate()
                .find_map(|(index, fragment)| {
                    (index > 0
                        && index + 1 < fragments.len()
                        && matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_)))
                    .then_some(index)
                })
                .expect("the round offset must retain a non-seam cusp fragment");
            let vertex = if cusp_is_next {
                cusp_index
            } else {
                cusp_index + 1
            };
            let first = source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    setback.clone(),
                    setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("both retained cusp endpoint orientations must chamfer exactly");
            assert_eq!(first.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(first) = first.value else {
                panic!("a retained cusp endpoint must have one interior setback cut");
            };
            assert_eq!(
                first.boundary_loops()[0].fragments().len(),
                fragments.len() + 1
            );
            assert!(
                first.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .any(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
            );

            // Re-enter at the newly created cusp/chord junction. The first
            // exact angular cut is now the corner parameter for the second.
            let repeated = first
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    cusp_index + 1,
                    repeated_setback.clone(),
                    repeated_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a retained cusp chamfer endpoint must remain reusable");
            assert_eq!(repeated.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(repeated) = repeated.value else {
                panic!("the repeated retained cusp chamfer must be unique");
            };
            assert_eq!(
                repeated.boundary_loops()[0].fragments().len(),
                fragments.len() + 2
            );
            for (point, expected) in [
                (Point2::new(q(1, 2), q(1, 2)), RegionPointLocation::Inside),
                (
                    Point2::new(-Real::one(), -Real::one()),
                    RegionPointLocation::Outside,
                ),
            ] {
                assert_eq!(
                    certified(repeated.classify_point(&point, &policy).unwrap()),
                    Classification::Decided(expected),
                );
            }

            let zero_source = rounded();
            let (previous_setback, next_setback) = if cusp_is_next {
                (setback.clone(), Real::zero())
            } else {
                (Real::zero(), setback.clone())
            };
            let zero = zero_source
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    previous_setback,
                    next_setback,
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a zero cusp-side setback must retain the exact corner");
            assert_eq!(zero.certainty, CurveCertainty::Certified);
            assert!(matches!(zero.value, CurveCornerSolutions2::Unique(_)));

            let over = rounded()
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    vertex,
                    Real::one(),
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a cusp over-setback must terminate exactly");
            assert_eq!(over.certainty, CurveCertainty::Certified);
            assert!(matches!(
                over.value,
                CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::OutsideTrimDomain)
            ));
        }

        let source = rounded();
        let mut seam_fragments = source.boundary_loops()[0].fragments().to_vec();
        let cusp_index = seam_fragments
            .iter()
            .position(|fragment| {
                matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
            })
            .expect("the round offset must retain a cusp");
        seam_fragments.rotate_left(cusp_index);
        let seam_fragment_count = seam_fragments.len();
        let seam_boundary = CurveRegionBoundaryLoop2::new(seam_fragments.clone(), &policy).unwrap();
        let seam = CurveRegion2::try_new_with_loop_topology(
            vec![seam_boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();
        let seam_cut = seam
            .chamfer_loop_vertex_by_setbacks(
                0,
                0,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("the loop seam must not alter a retained cusp chamfer");
        assert_eq!(seam_cut.certainty, CurveCertainty::Certified);
        assert!(matches!(seam_cut.value, CurveCornerSolutions2::Unique(_)));

        let reversed_fragments = seam_fragments
            .iter()
            .rev()
            .map(|fragment| fragment.reversed().unwrap())
            .collect();
        let reversed_boundary = CurveRegionBoundaryLoop2::new(reversed_fragments, &policy).unwrap();
        let reversed = CurveRegion2::try_new_with_loop_topology(
            vec![reversed_boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Right],
        )
        .unwrap();
        let reversed_cut = reversed
            .chamfer_loop_vertex_by_setbacks(
                0,
                0,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("reversed retained cusp traversal must chamfer exactly");
        assert_eq!(reversed_cut.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(reversed_cut) = reversed_cut.value else {
            panic!("the reversed seam cusp must have one exact chamfer");
        };
        assert_eq!(
            reversed_cut.boundary_loops()[0].fragments().len(),
            seam_fragment_count + 1,
        );
        assert_eq!(
            certified(
                reversed_cut
                    .classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Inside),
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn canonical_exact_chord_regions_fillet_without_line_demotion() {
    let exact_chord_rectangle = |policy: &CurveContext, x_offset: i64| {
        let points = [
            p(x_offset, 0),
            p(x_offset + 4, 0),
            p(x_offset + 4, 4),
            p(x_offset, 4),
            p(x_offset, 0),
        ];
        let fragments = points
            .windows(2)
            .map(|edge| {
                let Classification::Decided(chord) = BezierAlgebraicChord2::try_new(
                    RationalBezierIntersectionPointEvidence2::Exact(edge[0].clone()),
                    RationalBezierIntersectionPointEvidence2::Exact(edge[1].clone()),
                    policy,
                )
                .unwrap() else {
                    panic!("an exact rectangle edge must define a retained chord");
                };
                BezierSplitFragment2::AlgebraicChord(chord)
            })
            .collect();
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            let seam_source = exact_chord_rectangle(&policy, 0);
            let seam_source = if reverse {
                let fragments = seam_source.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .rev()
                    .map(|fragment| fragment.reversed().unwrap())
                    .collect();
                let boundary = CurveRegionBoundaryLoop2::new(fragments, &policy).unwrap();
                CurveRegion2::try_new_with_loop_topology(
                    vec![boundary],
                    vec![CurveRegionLoopRole::Material],
                    vec![FillRule::NonZero],
                    vec![CurveBoundaryInteriorSide2::Right],
                )
                .unwrap()
            } else {
                seam_source
            };
            let seam = seam_source
                .fillet_loop_vertex_by_radius(0, 0, q(1, 8), CurveCornerMode2::TrimOnly, &policy)
                .expect("the exact-chord loop seam must retain fillet semantics");
            assert_eq!(seam.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(seam) = seam.value else {
                panic!("the exact-chord seam fillet must be unique");
            };
            assert_eq!(
                certified(
                    seam.classify_point(&Point2::new(Real::from(2), Real::from(2)), &policy)
                        .unwrap(),
                ),
                Classification::Decided(RegionPointLocation::Inside),
            );
        }

        let source = exact_chord_rectangle(&policy, 0);
        let first = source
            .fillet_loop_vertex_by_radius(0, 1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)
            .expect("canonical exact chords must reuse the authoritative fillet solver");
        assert_eq!(first.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(first) = first.value else {
            panic!("a convex exact-chord corner must have one in-domain fillet");
        };
        let fragments = first.boundary_loops()[0].fragments();
        assert_eq!(fragments.len(), 5);
        assert_eq!(
            fragments
                .iter()
                .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                .count(),
            4,
        );
        assert!(fragments.iter().any(|fragment| matches!(
            fragment,
            BezierSplitFragment2::Materialized {
                curve: hypercurve::BezierSubcurve2::RationalQuadratic(_),
                ..
            }
        )));
        assert!(matches!(
            certified(first.filled_area(&policy).unwrap()),
            Classification::Decided(Some(_))
        ));

        let fragment_count = fragments.len();
        let next_chord_corner = (0..fragment_count)
            .find(|index| {
                matches!(
                    fragments[(index + fragment_count - 1) % fragment_count],
                    BezierSplitFragment2::AlgebraicChord(_)
                ) && matches!(fragments[*index], BezierSplitFragment2::AlgebraicChord(_))
            })
            .expect("the once-filleted rectangle retains another chord/chord corner");
        let second = first
            .fillet_loop_vertex_by_radius(
                0,
                next_chord_corner,
                q(1, 4),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("a later canonical chord corner must remain filletable");
        assert_eq!(second.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(second) = second.value else {
            panic!("the repeated exact-chord fillet must remain unique");
        };
        assert_eq!(
            certified(
                second
                    .classify_point(&Point2::new(Real::from(2), Real::from(2)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Inside),
        );
        assert_eq!(
            certified(
                second
                    .classify_point(&Point2::new(Real::from(-1), Real::from(-1)), &policy)
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Outside),
        );

        let disjoint = exact_chord_rectangle(&policy, 10);
        let replay = second
            .boolean_regions(&disjoint, &policy)
            .expect("a retained fillet must re-enter the canonical Boolean kernel");
        assert_eq!(replay.certainty, CurveCertainty::Certified);
        assert_eq!(replay.value.union().boundary_loops().len(), 2);
        assert!(replay.value.intersection().is_empty());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_endpoint_chord_pairs_share_the_linear_fillet_kernel() {
    let source = |policy: &CurveContext, reverse: bool| {
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![-q(1, 2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
        let parameter =
            decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
        let selected = |start: Point2, end: Point2| {
            RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2])
                    .unwrap()
                    .point_at_algebraic_parameter(&parameter, policy)
                    .unwrap(),
            )
        };
        let corner = RationalBezierIntersectionPointEvidence2::Exact(p(0, 0));
        let incoming = selected(p(-5, 0), p(-4, 0));
        let outgoing = selected(p(0, 4), p(0, 5));
        let chord = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(start, end, policy).unwrap(),
            ))
        };
        let mut fragments = vec![
            chord(incoming.clone(), corner.clone()),
            chord(corner, outgoing.clone()),
            chord(outgoing, incoming),
        ];
        let interior_side = if reverse {
            fragments = fragments
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().unwrap())
                .collect();
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        };
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![interior_side],
        )
        .unwrap()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            let region = source(&policy, reverse);
            let fragments = region.boundary_loops()[0].fragments();
            let corner = (0..fragments.len())
                .find(|index| {
                    let BezierSplitFragment2::AlgebraicChord(previous) =
                        &fragments[(index + fragments.len() - 1) % fragments.len()]
                    else {
                        return false;
                    };
                    let BezierSplitFragment2::AlgebraicChord(next) = &fragments[*index] else {
                        return false;
                    };
                    previous.end().as_exact() == Some(&p(0, 0))
                        && next.start().as_exact() == Some(&p(0, 0))
                })
                .expect("the selected-endpoint triangle retains its represented corner");
            let result = region
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    Real::one(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("selected-endpoint support chords must share the linear fillet kernel");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(filleted) = result.value else {
                panic!("the selected-endpoint right angle must have one exact fillet");
            };
            let fragments = filleted.boundary_loops()[0].fragments();
            assert_eq!(fragments.len(), 4);
            assert_eq!(
                fragments
                    .iter()
                    .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicChord(_)))
                    .count(),
                3,
            );
            assert!(fragments.iter().any(|fragment| matches!(
                fragment,
                BezierSplitFragment2::Materialized {
                    curve: hypercurve::BezierSubcurve2::RationalQuadratic(_),
                    ..
                }
            )));
            assert_eq!(
                certified(filleted.classify_point(&p(-2, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                certified(filleted.classify_point(&p(1, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside),
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_endpoint_chords_share_linear_arc_fillet_incidence() {
    let source = |policy: &CurveContext, reverse: bool| {
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![-q(1, 2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
        let parameter =
            decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
        let selected = |start: Point2, end: Point2| {
            RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2])
                    .unwrap()
                    .point_at_algebraic_parameter(&parameter, policy)
                    .unwrap(),
            )
        };
        let lower_left = selected(p(-3, 0), p(-2, 0));
        let upper_left = selected(p(-3, 1), p(-2, 1));
        let corner = RationalBezierIntersectionPointEvidence2::Exact(p(0, 0));
        let upper_right = RationalBezierIntersectionPointEvidence2::Exact(p(1, 1));
        let chord = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(start, end, policy).unwrap(),
            ))
        };

        let native_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap();
        let native_path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-3, 0), p(0, 0)).unwrap()),
            Curve2::from(native_arc),
            Curve2::from(LineSeg2::try_new(p(1, 1), p(-3, 1)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-3, 1), p(-3, 0)).unwrap()),
        ])
        .unwrap();
        let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[native_path],
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            policy,
        )
        .unwrap()
        .into_value();
        let arc = native.boundary_loops()[0]
            .fragments()
            .iter()
            .find(|fragment| {
                matches!(
                    fragment,
                    BezierSplitFragment2::Materialized {
                        curve: hypercurve::BezierSubcurve2::RationalQuadratic(_),
                        ..
                    }
                )
            })
            .expect("the native quarter circle materializes as one rational quadratic")
            .clone();
        let mut fragments = vec![
            chord(lower_left.clone(), corner),
            arc,
            chord(upper_right, upper_left.clone()),
            chord(upper_left, lower_left),
        ];
        let interior_side = if reverse {
            fragments = fragments
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().unwrap())
                .collect();
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        };
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![interior_side],
        )
        .unwrap()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            let region = source(&policy, reverse);
            let fragments = region.boundary_loops()[0].fragments();
            let corner = (0..fragments.len())
                .find(|index| {
                    match (
                        &fragments[(index + fragments.len() - 1) % fragments.len()],
                        &fragments[*index],
                    ) {
                        (
                            BezierSplitFragment2::AlgebraicChord(previous),
                            BezierSplitFragment2::Materialized {
                                curve: hypercurve::BezierSubcurve2::RationalQuadratic(next),
                                ..
                            },
                        ) => {
                            previous.end().as_exact() == Some(&p(0, 0)) && next.start() == &p(0, 0)
                        }
                        (
                            BezierSplitFragment2::Materialized {
                                curve: hypercurve::BezierSubcurve2::RationalQuadratic(previous),
                                ..
                            },
                            BezierSplitFragment2::AlgebraicChord(next),
                        ) => {
                            previous.end() == &p(0, 0) && next.start().as_exact() == Some(&p(0, 0))
                        }
                        _ => false,
                    }
                })
                .expect("the mixed selected-chord/circular corner remains explicit");
            let result = region
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    q(1, 2),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a represented-support chord must reuse line/circle incidence");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(filleted) = result.value else {
                panic!("the retained chord/circular corner must have one exact fillet");
            };
            assert_eq!(
                certified(
                    filleted
                        .classify_point(&Point2::new(-Real::one(), q(1, 2)), &policy)
                        .unwrap()
                ),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                certified(filleted.classify_point(&p(2, 0), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside),
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_endpoint_chords_share_linear_bezier_fillet_incidence() {
    let source = |policy: &CurveContext, reverse: bool| {
        let polynomial = decided(
            BezierParameterPolynomial::try_new_power_basis(
                vec![-q(1, 2), Real::zero(), Real::one()],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap());
        let parameter =
            decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap());
        let selected = |start: Point2, end: Point2| {
            RationalBezierIntersectionPointEvidence2::Algebraic(
                RationalBezier2::try_new(vec![start, end], vec![Real::one(); 2])
                    .unwrap()
                    .point_at_algebraic_parameter(&parameter, policy)
                    .unwrap(),
            )
        };
        let lower_left = selected(p(-5, 0), p(-4, 0));
        let upper_left = selected(p(-5, 2), p(-4, 2));
        let corner = RationalBezierIntersectionPointEvidence2::Exact(p(0, 0));
        let upper_right = RationalBezierIntersectionPointEvidence2::Exact(p(1, 2));
        let chord = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(start, end, policy).unwrap(),
            ))
        };
        let quadratic = BezierSplitFragment2::Materialized {
            start: hypercurve::BezierParameter2::Exact(Real::zero()),
            end: hypercurve::BezierParameter2::Exact(Real::one()),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                p(0, 0),
                p(0, 1),
                p(1, 2),
            )),
        };
        let mut fragments = vec![
            chord(lower_left.clone(), corner),
            quadratic,
            chord(upper_right, upper_left.clone()),
            chord(upper_left, lower_left),
        ];
        let interior_side = if reverse {
            fragments = fragments
                .iter()
                .rev()
                .map(|fragment| fragment.reversed().unwrap())
                .collect();
            CurveBoundaryInteriorSide2::Right
        } else {
            CurveBoundaryInteriorSide2::Left
        };
        let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();
        CurveRegion2::try_new_with_loop_topology(
            vec![boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![interior_side],
        )
        .unwrap()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            let region = source(&policy, reverse);
            let fragments = region.boundary_loops()[0].fragments();
            let corner = (0..fragments.len())
                .find(|index| {
                    match (
                        &fragments[(index + fragments.len() - 1) % fragments.len()],
                        &fragments[*index],
                    ) {
                        (
                            BezierSplitFragment2::AlgebraicChord(previous),
                            BezierSplitFragment2::Materialized {
                                curve: hypercurve::BezierSubcurve2::Quadratic(next),
                                ..
                            },
                        ) => {
                            previous.end().as_exact() == Some(&p(0, 0)) && next.start() == &p(0, 0)
                        }
                        (
                            BezierSplitFragment2::Materialized {
                                curve: hypercurve::BezierSubcurve2::Quadratic(previous),
                                ..
                            },
                            BezierSplitFragment2::AlgebraicChord(next),
                        ) => {
                            previous.end() == &p(0, 0) && next.start().as_exact() == Some(&p(0, 0))
                        }
                        _ => false,
                    }
                })
                .expect("the mixed selected-chord/quadratic corner remains explicit");
            let result = region
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    q(15, 4),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a represented-support chord must reuse line/Bezier incidence");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let candidates = match result.value {
                CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                CurveCornerSolutions2::Multiple(candidates) => candidates,
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!(
                        "the retained chord/quadratic corner lost its exact fillet: policy={policy:?}, reverse={reverse}, reason={reason:?}"
                    )
                }
            };
            let filleted = candidates
                .into_iter()
                .find(|candidate| {
                    candidate.boundary_loops()[0]
                        .fragments()
                        .iter()
                        .any(|fragment| {
                            matches!(
                                fragment,
                                BezierSplitFragment2::Materialized {
                                    curve: hypercurve::BezierSubcurve2::RationalQuadratic(_),
                                    ..
                                }
                            )
                        })
                })
                .expect("one exact candidate must publish the circular fillet span");
            assert_eq!(
                certified(filleted.classify_point(&p(-3, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Inside),
            );
            assert_eq!(
                certified(filleted.classify_point(&p(2, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside),
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn selected_circle_support_chord_corners_retain_algebraic_fillet_centers() {
    let clipping_region = |policy: &CurveContext| {
        let points = [
            Point2::new(-Real::one(), -Real::one()),
            Point2::new(q(3, 4), -Real::one()),
            Point2::new(q(3, 4), Real::from(2)),
            Point2::new(-Real::one(), Real::from(2)),
            Point2::new(-Real::one(), -Real::one()),
        ];
        let contour = Contour2::try_new(
            points
                .windows(2)
                .map(|edge| {
                    Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
                })
                .collect(),
        )
        .unwrap();
        CurveRegion2::try_from_native_material_contours(vec![contour], policy)
            .unwrap()
            .into_value()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let rounded = axis_aligned_algebraic_rectangle(&policy)
            .offset(q(1, 10), &OffsetCornerStyle2::Round, &policy)
            .expect("the selected-field round source must remain exact")
            .into_value();
        let clipped = rounded
            .boolean_regions(&clipping_region(&policy), &policy)
            .expect("the native line must clip the selected circle exactly");
        assert_eq!(clipped.certainty, CurveCertainty::Certified);
        let clipped = clipped.value.intersection().clone();
        let fragments = clipped.boundary_loops()[0].fragments();
        let fragment_count = fragments.len();
        let fragment_kinds = fragments
            .iter()
            .map(|fragment| match fragment {
                BezierSplitFragment2::Materialized { .. } => "materialized",
                BezierSplitFragment2::AlgebraicEndpointImages { .. } => "endpoint-images",
                BezierSplitFragment2::AnalyticParallel(_) => "analytic-parallel",
                BezierSplitFragment2::AlgebraicChord(_) => "chord",
                BezierSplitFragment2::AlgebraicCuspSemicircle(_) => "selected-circle",
                BezierSplitFragment2::Unresolved { .. } => "unresolved",
            })
            .collect::<Vec<_>>();
        let corners = (0..fragment_count)
            .filter(|index| {
                let previous = &fragments[(index + fragment_count - 1) % fragment_count];
                let next = &fragments[*index];
                matches!(
                    (previous, next),
                    (
                        BezierSplitFragment2::AlgebraicChord(_),
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                    ) | (
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_),
                        BezierSplitFragment2::AlgebraicChord(_)
                    )
                )
            })
            .collect::<Vec<_>>();
        if corners.is_empty() {
            panic!(
                "the clipped round boundary must publish a support-chord/circle corner: {fragment_kinds:?}"
            );
        }
        let cusp_count = fragments
            .iter()
            .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_)))
            .count();

        let mut outcomes = Vec::new();
        let mut filleted = Vec::new();
        for corner in corners {
            let result = clipped
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    q(1, 100),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "selected-circle/support-chord corner {corner} must retain its exact fillet center: {error:?}"
                    )
                });
            assert_eq!(
                result.certainty,
                CurveCertainty::Certified,
                "policy={policy:?}, corner={corner}"
            );
            let candidate_count = result.value.candidate_count();
            let no_solution_reason = result.value.no_solution_reason();
            match result.value {
                CurveCornerSolutions2::Unique(region) => {
                    filleted.push((corner, region));
                }
                CurveCornerSolutions2::NoSolution(_) | CurveCornerSolutions2::Multiple(_) => {
                    outcomes.push((corner, candidate_count, no_solution_reason));
                }
            }
        }
        assert_eq!(
            filleted.len(),
            2,
            "both transverse selected-circle/support-chord orientations must have one fillet: {outcomes:?}"
        );
        for (_, filleted) in filleted {
            assert_eq!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
                    .count(),
                cusp_count + 1,
            );
            assert_eq!(
                certified(
                    filleted
                        .classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy)
                        .unwrap(),
                ),
                Classification::Decided(RegionPointLocation::Inside),
            );

            let disjoint =
                CurveRegion2::try_from_native_material_contours(vec![square(2, 2, 3, 3)], &policy)
                    .unwrap()
                    .into_value();
            let replay = filleted
                .boolean_regions(&disjoint, &policy)
                .expect("the retained algebraic fillet must re-enter the Boolean kernel");
            assert_eq!(replay.certainty, CurveCertainty::Certified);
            assert_eq!(replay.value.union().boundary_loops().len(), 2);
            assert!(replay.value.intersection().is_empty());
        }
    }
}

#[cfg(feature = "predicates")]
fn analytic_parallel_cap_region(policy: &CurveContext) -> CurveRegion2 {
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 0), p(0, 4), p(2, 0))),
        Curve2::from(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, -2), p(-2, -2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-2, -2), p(-2, 0)).unwrap()),
    ])
    .unwrap();
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        policy,
    )
    .unwrap()
    .into_value()
}

#[cfg(feature = "predicates")]
#[test]
fn analytic_parallel_support_corners_retain_algebraic_fillet_centers() {
    let source = |policy: &CurveContext| {
        analytic_parallel_cap_region(policy)
            .offset(q(1, 10), &OffsetCornerStyle2::Bevel, policy)
            .unwrap()
            .into_value()
    };

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let expected_certainty = if policy == CurveContext::STRICT {
            CurveCertainty::Certified
        } else {
            CurveCertainty::Approximate512Consumed
        };
        let region = source(&policy);
        let fragments = region.boundary_loops()[0].fragments();
        let fragment_count = fragments.len();
        let fragment_kinds = fragments
            .iter()
            .map(|fragment| match fragment {
                BezierSplitFragment2::Materialized { .. } => "materialized",
                BezierSplitFragment2::AlgebraicEndpointImages { .. } => "endpoint-images",
                BezierSplitFragment2::AnalyticParallel(_) => "analytic-parallel",
                BezierSplitFragment2::AlgebraicChord(_) => "chord",
                BezierSplitFragment2::AlgebraicCuspSemicircle(_) => "selected-circle",
                BezierSplitFragment2::Unresolved { .. } => "unresolved",
            })
            .collect::<Vec<_>>();
        let corners = (0..fragment_count)
            .filter(|index| {
                let previous = &fragments[(index + fragment_count - 1) % fragment_count];
                let next = &fragments[*index];
                matches!(
                    (previous, next),
                    (
                        BezierSplitFragment2::AnalyticParallel(_),
                        BezierSplitFragment2::AlgebraicChord(_)
                            | BezierSplitFragment2::Materialized { .. }
                    ) | (
                        BezierSplitFragment2::AlgebraicChord(_)
                            | BezierSplitFragment2::Materialized { .. },
                        BezierSplitFragment2::AnalyticParallel(_)
                    )
                )
            })
            .collect::<Vec<_>>();
        assert!(
            !corners.is_empty(),
            "the exact offset must retain analytic/support corners: {fragment_kinds:?}"
        );
        let selected_circle_count = fragments
            .iter()
            .filter(|fragment| matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_)))
            .count();

        let mut filleted = Vec::new();
        let mut outcomes = Vec::new();
        for corner in corners {
            let result = region
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    q(1, 100),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "analytic-parallel/support corner {corner} must fillet exactly: {error:?}; fragments={fragment_kinds:?}"
                    )
                });
            assert_eq!(
                result.certainty, expected_certainty,
                "policy={policy:?}, corner={corner}"
            );
            match result.value {
                CurveCornerSolutions2::Unique(candidate) => filleted.push(candidate),
                other => {
                    outcomes.push((corner, other.candidate_count(), other.no_solution_reason()))
                }
            }
        }
        assert_eq!(
            filleted.len(),
            2,
            "both analytic-parallel endpoint orientations must fillet: {outcomes:?}; fragments={fragment_kinds:?}"
        );
        for filleted in filleted {
            assert_eq!(
                filleted.boundary_loops()[0]
                    .fragments()
                    .iter()
                    .filter(|fragment| {
                        matches!(fragment, BezierSplitFragment2::AlgebraicCuspSemicircle(_))
                    })
                    .count(),
                selected_circle_count + 1,
            );
            assert_eq!(
                certified(filleted.classify_point(&p(10, 10), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside),
            );
            let disjoint =
                CurveRegion2::try_from_native_material_contours(vec![square(4, 4, 5, 5)], &policy)
                    .unwrap()
                    .into_value();
            let replay = filleted
                .boolean_regions(&disjoint, &policy)
                .expect("the retained analytic fillet must re-enter the Boolean kernel");
            assert_eq!(replay.certainty, CurveCertainty::Certified);
            assert_eq!(replay.value.union().boundary_loops().len(), 2);
            assert!(replay.value.intersection().is_empty());
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn analytic_parallel_miter_tangent_legs_have_no_nondegenerate_fillet() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = analytic_parallel_cap_region(&policy)
            .offset(
                q(1, 10),
                &OffsetCornerStyle2::Miter {
                    limit: Real::from(100),
                },
                &policy,
            )
            .expect("the exact analytic miter must retain its tangent construction")
            .into_value();
        let assert_tangent_corners = |region: &CurveRegion2| {
            let fragments = region.boundary_loops()[0].fragments();
            let corners = (0..fragments.len())
                .filter(|index| {
                    matches!(
                        (
                            &fragments[(index + fragments.len() - 1) % fragments.len()],
                            &fragments[*index],
                        ),
                        (
                            BezierSplitFragment2::AnalyticParallel(_),
                            BezierSplitFragment2::Materialized { .. }
                        ) | (
                            BezierSplitFragment2::Materialized { .. },
                            BezierSplitFragment2::AnalyticParallel(_)
                        )
                    )
                })
                .collect::<Vec<_>>();
            assert_eq!(corners.len(), 2);
            for corner in corners {
                let result = region
                    .fillet_loop_vertex_by_radius(
                        0,
                        corner,
                        q(1, 100),
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    )
                    .expect("a certified tangent miter junction must be classified exactly");
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_eq!(
                    result.value,
                    CurveCornerSolutions2::NoSolution(CurveCornerNoSolution2::NoTangentCircle)
                );
            }
        };
        assert_tangent_corners(&region);

        let transformed = region
            .transform_affine(
                &Real::zero(),
                &Real::from(2),
                &Real::from(2),
                &Real::zero(),
                &Real::from(3),
                &Real::from(-1),
                &policy,
            )
            .expect("similarity and loop reversal must preserve exact tangent provenance")
            .into_value();
        assert_tangent_corners(&transformed);
    }
}

#[cfg(feature = "predicates")]
#[test]
fn analytic_parallel_rejected_miters_remain_transverse_fillet_candidates() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let region = analytic_parallel_cap_region(&policy)
            .offset(
                q(1, 10),
                &OffsetCornerStyle2::Miter { limit: Real::one() },
                &policy,
            )
            .expect("the rejected analytic miter must become an exact bevel")
            .into_value();
        let fragments = region.boundary_loops()[0].fragments();
        let corners = (0..fragments.len())
            .filter(|index| {
                matches!(
                    (
                        &fragments[(index + fragments.len() - 1) % fragments.len()],
                        &fragments[*index],
                    ),
                    (
                        BezierSplitFragment2::AnalyticParallel(_),
                        BezierSplitFragment2::Materialized { .. }
                    ) | (
                        BezierSplitFragment2::Materialized { .. },
                        BezierSplitFragment2::AnalyticParallel(_)
                    )
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(corners.len(), 2);
        for corner in corners {
            let result = region
                .fillet_loop_vertex_by_radius(
                    0,
                    corner,
                    q(1, 100),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("a rejected miter bevel must remain exactly filletable");
            assert!(
                matches!(result.value, CurveCornerSolutions2::Unique(_)),
                "policy={policy:?}, corner={corner}, result={result:?}"
            );
        }
    }
}

#[cfg(feature = "predicates")]
#[test]
fn exact_support_cutter_reenters_correlated_chord_collinearly() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_rectangle(&policy);
        let rounded = source
            .offset(q(1, 20), &OffsetCornerStyle2::Round, &policy)
            .unwrap()
            .into_value();
        let wide = source
            .transform_affine(
                &Real::from(4),
                &Real::zero(),
                &Real::zero(),
                &q(7, 10),
                &q(1, 10),
                &q(3, 10),
                &policy,
            )
            .unwrap()
            .into_value();
        let cutter = wide
            .offset(
                q(1, 40),
                &OffsetCornerStyle2::Miter {
                    limit: Real::from(2),
                },
                &policy,
            )
            .unwrap()
            .into_value();
        let first = rounded.boolean_regions(&cutter, &policy).unwrap();
        assert_eq!(first.certainty, CurveCertainty::Certified);
        let first = first.into_value().intersection().clone();
        let fragments = first.boundary_loops()[0].fragments();
        let retained_index = fragments
            .iter()
            .position(|fragment| {
                matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicChord(chord)
                        if matches!(
                            chord.start(),
                            RationalBezierIntersectionPointEvidence2::AlgebraicCuspChord(_)
                        )
                )
            })
            .expect("the exact support must retain its selected-circle contact");
        let BezierSplitFragment2::AlgebraicChord(retained) = &fragments[retained_index] else {
            unreachable!("the retained fragment was selected as a chord")
        };
        let cusp_index = (retained_index + fragments.len() - 1) % fragments.len();
        let cusp = match &fragments[cusp_index] {
            fragment @ BezierSplitFragment2::AlgebraicCuspSemicircle(_) => fragment.clone(),
            _ => panic!("the correlated chord must retain its adjacent selected circle"),
        };
        let mapped_chamfer = first
            .chamfer_loop_vertex_by_setbacks(
                0,
                retained_index,
                q(1, 1000),
                q(1, 1000),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("a Boolean-mapped selected-circle endpoint must chamfer exactly");
        assert_eq!(mapped_chamfer.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(mapped_chamfer) = mapped_chamfer.value else {
            panic!("the mapped cusp/chord junction must have one exact chamfer");
        };
        assert!(
            mapped_chamfer.boundary_loops()[0]
                .fragments()
                .iter()
                .any(|fragment| matches!(
                    fragment,
                    BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                ))
        );
        let mapped_reentry = mapped_chamfer
            .chamfer_loop_vertex_by_setbacks(
                0,
                retained_index,
                q(1, 2000),
                q(1, 2000),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("a rotated mapped cusp endpoint must remain reusable");
        assert_eq!(mapped_reentry.certainty, CurveCertainty::Certified);
        let CurveCornerSolutions2::Unique(mapped_reentry) = mapped_reentry.value else {
            panic!("the rotated mapped cusp endpoint must have one exact re-entry");
        };
        for (point, expected) in [
            (Point2::new(q(1, 2), q(1, 2)), RegionPointLocation::Inside),
            (
                Point2::new(-Real::one(), Real::zero()),
                RegionPointLocation::Outside,
            ),
        ] {
            assert_eq!(
                certified(mapped_reentry.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected),
            );
        }
        let before_cusp_index = (cusp_index + fragments.len() - 1) % fragments.len();
        let (preceding, closure_end) = match &fragments[before_cusp_index] {
            BezierSplitFragment2::AlgebraicChord(chord) => (chord.clone(), chord.start().clone()),
            _ => panic!("the selected circle must retain its preceding endpoint evidence"),
        };
        let after_retained_index = (retained_index + 1) % fragments.len();
        let (after_retained, closure_start) = match &fragments[after_retained_index] {
            fragment @ BezierSplitFragment2::Materialized { curve, .. } => (
                fragment.clone(),
                RationalBezierIntersectionPointEvidence2::Exact(decided(
                    curve.point_at(&Real::one(), &policy),
                )),
            ),
            fragment @ BezierSplitFragment2::AlgebraicChord(chord) => {
                (fragment.clone(), chord.end().clone())
            }
            _ => panic!("the exact-support chord must retain its exact vertical neighbor"),
        };
        let closure = |start, end| {
            BezierSplitFragment2::AlgebraicChord(decided(
                BezierAlgebraicChord2::try_new(start, end, &policy).unwrap(),
            ))
        };
        let retained_boundary = CurveRegionBoundaryLoop2::new(
            vec![
                BezierSplitFragment2::AlgebraicChord(retained.clone()),
                after_retained,
                closure(closure_start, closure_end),
                BezierSplitFragment2::AlgebraicChord(preceding),
                cusp,
            ],
            &policy,
        )
        .unwrap();
        let retained_region = CurveRegion2::try_new_with_loop_topology(
            vec![retained_boundary],
            vec![CurveRegionLoopRole::Material],
            vec![FillRule::NonZero],
            vec![CurveBoundaryInteriorSide2::Left],
        )
        .unwrap();

        let replay_points = [
            Point2::new(-Real::one(), Real::zero()),
            Point2::new(Real::one(), Real::zero()),
            Point2::new(Real::one(), q(41, 40)),
            Point2::new(-Real::one(), q(41, 40)),
        ];
        let replay_clip = CurveRegion2::try_from_native_material_contours(
            vec![
                Contour2::try_new(
                    (0..replay_points.len())
                        .map(|index| {
                            Segment2::Line(
                                LineSeg2::try_new(
                                    replay_points[index].clone(),
                                    replay_points[(index + 1) % replay_points.len()].clone(),
                                )
                                .unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap(),
            ],
            &policy,
        )
        .unwrap()
        .into_value();
        let mapped_replay = mapped_reentry
            .boolean_regions(&replay_clip, &policy)
            .expect("a re-chamfered mapped cusp endpoint must enter the Boolean kernel");
        assert_eq!(mapped_replay.certainty, CurveCertainty::Certified);
        assert!(!mapped_replay.value.union().is_empty());
        assert!(!mapped_replay.value.intersection().is_empty());
        let full_replay_evidence = first
            .intersect_region(&replay_clip, &policy)
            .expect("the complete retained intersection must replay the exact support");
        assert_eq!(full_replay_evidence.certainty, CurveCertainty::Certified);
        assert!(full_replay_evidence.value.is_complete());
        assert_eq!(full_replay_evidence.value.overlaps().len(), 1);
        let replay_evidence = retained_region
            .intersect_region(&replay_clip, &policy)
            .expect("the retained correlated chord must overlap its exact support line");
        assert_eq!(replay_evidence.certainty, CurveCertainty::Certified);
        let replay_blockers = replay_evidence
            .value
            .blockers()
            .iter()
            .map(|blocker| {
                (
                    blocker.first().fragment_index(),
                    blocker.first().family(),
                    blocker.second().fragment_index(),
                    blocker.second().family(),
                    blocker.uncertainty_reason(),
                )
            })
            .collect::<Vec<_>>();
        assert!(replay_evidence.value.is_complete(), "{replay_blockers:?}");
        assert_eq!(replay_evidence.value.overlaps().len(), 1);

        let replay = retained_region
            .boolean_regions(&replay_clip, &policy)
            .expect("the retained correlated overlap must enter all four later Booleans");
        assert_eq!(replay.certainty, CurveCertainty::Certified);
        assert!(!replay.value.union().is_empty());
        assert!(!replay.value.intersection().is_empty());
        assert!(replay.value.difference().is_empty());
        assert!(!replay.value.xor().is_empty());

        let touch_points = [
            Point2::new(-Real::one(), q(41, 40)),
            Point2::new(Real::from(2), q(41, 40)),
            Point2::new(Real::from(2), Real::from(2)),
            Point2::new(-Real::one(), Real::from(2)),
        ];
        let touch_box = CurveRegion2::try_from_native_material_contours(
            vec![
                Contour2::try_new(
                    (0..touch_points.len())
                        .map(|index| {
                            Segment2::Line(
                                LineSeg2::try_new(
                                    touch_points[index].clone(),
                                    touch_points[(index + 1) % touch_points.len()].clone(),
                                )
                                .unwrap(),
                            )
                        })
                        .collect(),
                )
                .unwrap(),
            ],
            &policy,
        )
        .unwrap()
        .into_value();
        let touch_cut = touch_box
            .boolean_regions(&rounded, &policy)
            .expect("the exact upper box must subtract the rounded operand");
        assert_eq!(touch_cut.certainty, CurveCertainty::Certified);
        let touch_region = touch_cut.into_value().difference().clone();
        assert!(!touch_region.is_empty());
        let touch_evidence = first
            .intersect_region(&touch_region, &policy)
            .expect("the retained selected-circle/chord endpoint must support a point touch");
        let touch_certainty = CurveCertainty::Certified;
        assert_eq!(touch_evidence.certainty, touch_certainty);
        let touch_blockers = touch_evidence
            .value
            .blockers()
            .iter()
            .map(|blocker| {
                (
                    blocker.first().loop_index(),
                    blocker.first().fragment_index(),
                    blocker.first().family(),
                    blocker.second().loop_index(),
                    blocker.second().fragment_index(),
                    blocker.second().family(),
                    blocker.uncertainty_reason(),
                )
            })
            .collect::<Vec<_>>();
        assert!(touch_evidence.value.is_complete(), "{touch_blockers:?}");
        assert!(!touch_evidence.value.contacts().is_empty());
        assert!(touch_evidence.value.overlaps().is_empty());

        let touch = first
            .boolean_regions(&touch_region, &policy)
            .expect("the correlated point touch must enter all four later Booleans");
        assert_eq!(touch.certainty, touch_certainty);
        assert!(!touch.value.union().is_empty());
        assert!(touch.value.intersection().is_empty());
        assert!(!touch.value.difference().is_empty());
        assert!(!touch.value.xor().is_empty());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn algebraic_chords_survive_nonsingular_exact_affine_transforms() {
    let distance = q(1, 20);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let transformed = axis_aligned_algebraic_rectangle(&policy)
            .transform_affine(
                &Real::from(2),
                &Real::zero(),
                &Real::zero(),
                &Real::from(3),
                &Real::from(5),
                &Real::from(-1),
                &policy,
            )
            .expect("an anisotropic nonsingular affine map preserves straight chords");
        assert_eq!(transformed.certainty, CurveCertainty::Certified);
        for (point, expected) in [
            (p(6, 0), RegionPointLocation::Inside),
            (p(7, 0), RegionPointLocation::Outside),
        ] {
            assert_eq!(
                certified(transformed.value.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected),
            );
        }
        let rounded = transformed
            .value
            .offset(distance.clone(), &OffsetCornerStyle2::Round, &policy)
            .expect("the transformed cardinal proof must remain usable by exact offsets");
        assert_eq!(rounded.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                rounded
                    .value
                    .classify_point(
                        &Point2::new(Real::from(6), Real::from(-1) - &distance),
                        &policy,
                    )
                    .unwrap(),
            ),
            Classification::Decided(RegionPointLocation::Boundary),
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn nonconvex_algebraic_chord_expansion_is_exact_and_local_collapse_is_explicit() {
    let miter = OffsetCornerStyle2::Miter {
        limit: Real::from(2),
    };
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let source = axis_aligned_algebraic_l_region(&policy);
        let expanded = source.offset(q(1, 20), &miter, &policy).unwrap();
        assert_eq!(expanded.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                expanded
                    .value
                    .classify_point(&Point2::new(q(47, 100), q(3, 4)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(
                expanded
                    .value
                    .classify_point(&Point2::new(q(2, 5), q(4, 5)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Outside)
        );

        let contracted = source.offset(-q(1, 20), &miter, &policy).unwrap();
        assert_eq!(contracted.certainty, CurveCertainty::Certified);
        assert_eq!(
            certified(
                contracted
                    .value
                    .classify_point(&Point2::new(q(3, 5), q(3, 4)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(
                contracted
                    .value
                    .classify_point(&Point2::new(q(13, 25), q(3, 4)), &policy)
                    .unwrap()
            ),
            Classification::Decided(RegionPointLocation::Outside)
        );

        let post_collapse = source.offset(-q(3, 20), &miter, &policy).unwrap();
        assert_eq!(post_collapse.certainty, CurveCertainty::Certified);
        assert_eq!(post_collapse.value.boundary_loops().len(), 1);
        assert_eq!(post_collapse.value.boundary_loops()[0].fragments().len(), 4);
        for (point, expected) in [
            (Point2::new(q(1, 4), q(1, 4)), RegionPointLocation::Inside),
            (Point2::new(q(11, 20), q(1, 4)), RegionPointLocation::Inside),
            (
                Point2::new(q(14, 25), q(1, 4)),
                RegionPointLocation::Outside,
            ),
            (Point2::new(q(3, 5), q(3, 4)), RegionPointLocation::Outside),
            (
                Point2::new(q(3, 20), q(1, 4)),
                RegionPointLocation::Boundary,
            ),
        ] {
            assert_eq!(
                certified(post_collapse.value.classify_point(&point, &policy).unwrap()),
                Classification::Decided(expected)
            );
        }

        let fully_collapsed = source.offset(-q(1, 4), &miter, &policy).unwrap();
        assert_eq!(fully_collapsed.certainty, CurveCertainty::Certified);
        assert!(fully_collapsed.value.is_empty());
    }
}

#[cfg(feature = "predicates")]
#[test]
fn algebraic_chord_erosion_splits_a_collapsed_neck_exactly() {
    let miter = OffsetCornerStyle2::Miter {
        limit: Real::from(2),
    };
    for (policy, fill_rule, reverse) in [
        (CurveContext::STRICT, FillRule::NonZero, false),
        (CurveContext::STRICT, FillRule::EvenOdd, true),
        (CurveContext::APPROXIMATE_512, FillRule::NonZero, false),
        (CurveContext::APPROXIMATE_512, FillRule::EvenOdd, true),
    ] {
        let source = axis_aligned_algebraic_dumbbell_region(&policy, fill_rule, reverse);
        for radius in [Real::one(), q(11, 10)] {
            let split = source.offset(-radius, &miter, &policy).unwrap();
            assert_eq!(split.certainty, CurveCertainty::Certified);
            assert_eq!(split.value.boundary_loops().len(), 2);
            assert_eq!(
                certified(split.value.loop_roles(&policy).unwrap()),
                Classification::Decided(vec![
                    CurveRegionLoopRole::Material,
                    CurveRegionLoopRole::Material,
                ])
            );
            assert!(
                split
                    .value
                    .boundary_loops()
                    .iter()
                    .all(|boundary| boundary.fragments().len() == 4)
            );
            for (point, expected) in [
                (p(2, 2), RegionPointLocation::Inside),
                (p(10, 2), RegionPointLocation::Inside),
                (p(6, 2), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    certified(split.value.classify_point(&point, &policy).unwrap()),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn unified_region_bounds_cover_native_and_higher_order_carriers_exactly() {
    let policy = CurveContext::STRICT;
    let native =
        CurveRegion2::try_from_native_material_contours(vec![square(-3, -2, 7, 5)], &policy)
            .unwrap()
            .into_value();
    let native_bounds = decided(native.bounds(&policy).unwrap());
    assert_eq!(native_bounds.min_x(), &Real::from(-3));
    assert_eq!(native_bounds.min_y(), &Real::from(-2));
    assert_eq!(native_bounds.max_x(), &Real::from(7));
    assert_eq!(native_bounds.max_y(), &Real::from(5));

    let curved = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[double_wound_quadratic_cap()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    let curved_bounds = decided(curved.bounds(&policy).unwrap());
    assert_eq!(curved_bounds.min_x(), &Real::from(-2));
    assert_eq!(curved_bounds.min_y(), &Real::zero());
    assert_eq!(curved_bounds.max_x(), &Real::from(2));
    assert_eq!(curved_bounds.max_y(), &Real::from(4));

    assert!(
        CurveRegion2::empty()
            .bounds(&policy)
            .unwrap()
            .map(|classification| classification.is_uncertain())
            .into_value()
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_components() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_material_contours(
        vec![square(0, 0, 2, 2), square(4, 0, 6, 2)],
        &policy,
    )
    .unwrap()
    .into_value();

    let offset = promoted
        .offset(Real::from(2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(60))
    );
}

#[test]
fn unified_region_offset_regularizes_overlapping_expanded_voids() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 20, 16)],
        vec![square(5, 5, 7, 7), square(9, 5, 11, 7)],
        &policy,
    )
    .unwrap()
    .into_value();

    let offset = promoted
        .offset(Real::from(-2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert_eq!(native.hole_contours().len(), 1);
    assert_eq!(
        certified(offset.classify_point(&p(8, 6), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_expansion_regularizes_a_closed_concavity() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy)
        .unwrap()
        .into_value();

    let offset = promoted
        .offset(Real::from(3), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let native = decided(offset.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        certified(offset.classify_point(&p(5, 8), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(-2, -2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(14, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_contracts_nonconvex_material_before_its_medial_collapse() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source
        .offset(-Real::one(), &sharp_offset(), &policy)
        .unwrap()
        .into_value();

    assert_eq!(
        certified(eroded.classify_point(&p(1, 1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Boundary)
    );
    assert_eq!(
        certified(eroded.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_discards_nonconvex_material_after_wavefront_collapse() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![u_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source
        .offset(Real::from(-2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();

    assert!(eroded.is_empty());
    assert_eq!(
        certified(eroded.classify_point(&p(5, 1), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_nonconvex_erosion_splits_at_a_collapsed_neck() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![dumbbell_shape()], &policy)
        .unwrap()
        .into_value();

    let eroded = source
        .offset(-q(3, 2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let native = decided(eroded.native_contours_fast_path(&policy).unwrap());

    assert_eq!(native.material_contours().len(), 2);
    assert!(native.hole_contours().is_empty());
    for point in [p(2, 2), p(10, 2)] {
        assert_eq!(
            certified(eroded.classify_point(&point, &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
    assert_eq!(
        certified(eroded.classify_point(&p(6, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
}

#[test]
fn unified_region_convex_contraction_decides_collapse_and_over_contraction() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy)
        .unwrap()
        .into_value();

    let near = source
        .offset(-q(3, 2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let near_bounds = decided(near.bounds(&policy).unwrap());
    assert_eq!(near_bounds.min_x(), &q(3, 2));
    assert_eq!(near_bounds.min_y(), &q(3, 2));
    assert_eq!(near_bounds.max_x(), &q(5, 2));
    assert_eq!(near_bounds.max_y(), &q(5, 2));
    assert!(
        source
            .offset(Real::from(-2), &sharp_offset(), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
    assert!(
        source
            .offset(Real::from(-3), &sharp_offset(), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

#[test]
fn unified_region_convex_erosion_handles_orientation_and_redundant_edges() {
    let policy = CurveContext::STRICT;
    for contour in [reversed(&square(0, 0, 4, 4)), square_with_redundant_edge()] {
        let source = CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
            .unwrap()
            .into_value();
        let eroded = source
            .offset(Real::from(-1), &sharp_offset(), &policy)
            .unwrap()
            .into_value();
        let bounds = decided(eroded.bounds(&policy).unwrap());
        assert_eq!(bounds.min_x(), &Real::one());
        assert_eq!(bounds.min_y(), &Real::one());
        assert_eq!(bounds.max_x(), &Real::from(3));
        assert_eq!(bounds.max_y(), &Real::from(3));
        assert_eq!(
            certified(eroded.classify_point(&p(2, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn unified_region_convex_erosion_keeps_symbolic_diagonal_offsets_and_collapse_exact() {
    let policy = CurveContext::STRICT;
    let source =
        CurveRegion2::try_from_native_material_contours(vec![right_isosceles_triangle()], &policy)
            .unwrap()
            .into_value();
    let root_two = Real::from(2).sqrt().unwrap();

    let eroded = source
        .offset(Real::from(-1), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let native = decided(eroded.native_contours_fast_path(&policy).unwrap());
    let vertices = native.material_contours()[0]
        .segments()
        .iter()
        .map(|segment| segment.start().clone())
        .collect::<Vec<_>>();
    let far_axis_coordinate = 3.0 - std::f64::consts::SQRT_2;
    for expected in [
        (1.0, 1.0),
        (far_axis_coordinate, 1.0),
        (1.0, far_axis_coordinate),
    ] {
        assert!(vertices.iter().any(|vertex| {
            let x = vertex.x().to_f64_lossy().unwrap();
            let y = vertex.y().to_f64_lossy().unwrap();
            (x - expected.0).abs() < 1.0e-12 && (y - expected.1).abs() < 1.0e-12
        }));
    }
    assert!(
        vertices
            .iter()
            .flat_map(|vertex| [vertex.x(), vertex.y()])
            .any(|coordinate| {
                let facts = coordinate.detailed_facts();
                !facts.base.exact_rational
                    && (facts
                        .symbolic
                        .dependencies
                        .contains(SymbolicDependencyMask::SQRT)
                        || facts
                            .symbolic
                            .dependencies
                            .contains(SymbolicDependencyMask::OPAQUE))
            }),
        "the diagonal offset must remain an exact non-rational computable value"
    );

    let collapse_distance = Real::from(4) - Real::from(2) * root_two;
    assert!(
        source
            .offset(-collapse_distance, &sharp_offset(), &policy)
            .unwrap()
            .into_value()
            .is_empty(),
        "the exact radical inradius must collapse the triangle without a blocker"
    );
    assert!(
        source
            .offset(Real::from(-2), &sharp_offset(), &policy)
            .unwrap()
            .into_value()
            .is_empty()
    );
}

#[test]
fn unified_region_positive_offset_removes_exactly_collapsed_convex_hole() {
    let policy = CurveContext::STRICT;
    let source = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 20, 20)],
        vec![square(5, 5, 15, 15)],
        &policy,
    )
    .unwrap()
    .into_value();

    let expanded = source
        .offset(Real::from(5), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    assert_eq!(decided(expanded.loop_roles(&policy).unwrap()).len(), 1);
    assert_eq!(
        certified(expanded.classify_point(&p(10, 10), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_native_arrangement_exposes_immediate_evidence() {
    let source = square(0, 0, 4, 4).segments().to_vec();
    let result =
        CurveRegion2::arrange_unordered_segments(source, FillRule::NonZero, &CurveContext::STRICT)
            .unwrap()
            .into_value();

    assert!(result.region().is_some());
    assert_eq!(result.fill_rule(), FillRule::NonZero);
    assert_eq!(result.source_segment_count(), 4);
    assert!(result.status().is_native_exact());
    assert_eq!(result.blocker(), None);
}

#[test]
fn native_self_crossing_walk_regularizes_with_both_fill_rules() {
    let policy = CurveContext::STRICT;
    for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
        let contour = bow_tie_contour(fill_rule);
        let classification = CurveRegion2::try_from_regularized_native_contour(&contour, &policy)
            .unwrap()
            .into_value();
        let region = decided(classification);
        let native = decided(region.native_contours_fast_path(&policy).unwrap());
        assert_eq!(native.material_contours().len(), 2);
        assert!(native.hole_contours().is_empty());
        assert_eq!(
            certified(region.classify_point(&p(2, 3), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(region.classify_point(&p(2, 1), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            certified(region.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(Real::from(8))
        );
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_self_crossing_walks() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill_rule in [FillRule::NonZero, FillRule::EvenOdd] {
            let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[bow_tie_path()],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value();
            let region = raw.regularized_region(&policy).unwrap().into_value();
            let native = decided(region.native_contours_fast_path(&policy).unwrap());
            assert_eq!(native.material_contours().len(), 2);
            assert!(native.hole_contours().is_empty());
            for (point, expected) in [
                (p(2, 3), RegionPointLocation::Inside),
                (p(2, 1), RegionPointLocation::Inside),
                (p(0, 2), RegionPointLocation::Outside),
            ] {
                assert_eq!(
                    certified(region.classify_point(&point, &policy).unwrap()),
                    Classification::Decided(expected)
                );
            }
            assert_eq!(
                decided(region.filled_area(&policy).unwrap()),
                Some(Real::from(8))
            );
        }
    }
}

#[test]
fn authoritative_curve_region_regularizes_polynomial_and_rational_self_crossings() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for rational_reparameterization in [false, true] {
            let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[self_crossing_cubic_path(rational_reparameterization)],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &policy,
            )
            .unwrap()
            .into_value();
            let region = raw.regularized_region(&policy).unwrap().into_value();
            assert_eq!(region.boundary_loops().len(), 2);
            assert_eq!(
                certified(region.classify_point(&p(2, 1), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Inside)
            );
            assert_eq!(
                certified(region.classify_point(&p(-8, -8), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside)
            );
        }
    }
}

#[test]
fn native_self_overlap_regularization_honors_winding_multiplicity() {
    let policy = CurveContext::STRICT;

    let nonzero = decided(
        CurveRegion2::try_from_regularized_native_contour(
            &double_wound_square(FillRule::NonZero),
            &policy,
        )
        .unwrap()
        .into_value(),
    );
    let native = decided(nonzero.native_contours_fast_path(&policy).unwrap());
    assert_eq!(native.material_contours().len(), 1);
    assert!(native.hole_contours().is_empty());
    assert_eq!(
        decided(nonzero.filled_area(&policy).unwrap()),
        Some(Real::from(100))
    );

    let even_odd = decided(
        CurveRegion2::try_from_regularized_native_contour(
            &double_wound_square(FillRule::EvenOdd),
            &policy,
        )
        .unwrap()
        .into_value(),
    );
    assert!(even_odd.is_empty());
}

#[test]
fn authoritative_curve_region_arrangement_honors_coincident_winding_multiplicity() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let regularize = |fill_rule| {
            let contour = double_wound_square(fill_rule);
            let raw = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[path_from_contour(&contour)],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value();
            raw.regularized_region(&policy).unwrap().into_value()
        };

        let nonzero = regularize(FillRule::NonZero);
        assert_eq!(
            decided(nonzero.filled_area(&policy).unwrap()),
            Some(Real::from(100))
        );
        assert!(regularize(FillRule::EvenOdd).is_empty());
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_signed_loop_composition() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let paths = [
            path_from_contour(&square(0, 0, 4, 4)),
            path_from_contour(&square(2, 0, 6, 4)),
        ];
        let union = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &paths,
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
            &[FillRule::NonZero, FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value()
        .regularized_region(&policy)
        .unwrap()
        .into_value();
        assert_eq!(
            decided(union.filled_area(&policy).unwrap()),
            Some(Real::from(24))
        );
        assert_eq!(
            certified(union.classify_point(&p(3, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );

        let cancellation = CurveRegion2::try_from_signed_boundary_paths_with_loop_semantics(
            &[paths[0].clone(), paths[0].clone()],
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            &[FillRule::NonZero, FillRule::NonZero],
            &policy,
        )
        .unwrap()
        .into_value()
        .regularized_region(&policy)
        .unwrap()
        .into_value();
        assert!(cancellation.is_empty());
    }
}

#[test]
fn authoritative_curve_region_arrangement_regularizes_nonlinear_winding() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let regularize = |fill_rule| {
            CurveRegion2::try_from_boundary_paths_with_loop_semantics(
                &[double_wound_quadratic_cap()],
                &[CurveRegionLoopRole::Material],
                &[fill_rule],
                &policy,
            )
            .unwrap()
            .into_value()
            .regularized_region(&policy)
            .unwrap()
            .into_value()
        };
        let nonzero = regularize(FillRule::NonZero);
        assert_eq!(
            certified(nonzero.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(RegionPointLocation::Inside)
        );
        assert_eq!(
            decided(nonzero.filled_area(&policy).unwrap()),
            Some(q(32, 3))
        );
        assert!(regularize(FillRule::EvenOdd).is_empty());
    }
}

#[test]
fn region_promotion_retains_explicit_roles_and_line_fast_path() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        Vec::new(),
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(promoted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Material,]
    );
    assert_eq!(
        decided(promoted.filled_side_is_left(&policy).unwrap()),
        &[true, true]
    );
    let profiles = decided(promoted.boundary_profiles(&policy).unwrap());
    assert_eq!(profiles.len(), 2);
    assert!(profiles.iter().all(|profile| profile.holes().is_empty()));

    for (point, expected) in [
        (p(-1, 5), RegionPointLocation::Outside),
        (p(1, 1), RegionPointLocation::Inside),
        (p(5, 5), RegionPointLocation::Inside),
    ] {
        assert_eq!(
            certified(promoted.classify_point(&point, &policy).unwrap()),
            Classification::Decided(expected)
        );
    }
    assert_eq!(
        certified(promoted.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "nested explicit material must not be reinterpreted as an even-odd hole"
    );

    assert!(matches!(
        certified(promoted.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}

#[test]
fn transformed_promotion_retains_explicit_roles_without_the_source_fast_path() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        Vec::new(),
        &policy,
    )
    .unwrap()
    .into_value();

    let transformed = promoted
        .transform_affine(
            &Real::from(2),
            &Real::zero(),
            &Real::zero(),
            &Real::from(3),
            &Real::from(5),
            &Real::from(-4),
            &policy,
        )
        .unwrap()
        .into_value();

    assert_eq!(
        decided(transformed.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Material]
    );
    assert_eq!(
        certified(transformed.classify_point(&p(15, 11), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "a transformed nested material island must retain its explicit role"
    );

    assert!(matches!(
        certified(transformed.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}

#[test]
fn similarity_rotation_preserves_unified_region_semantics_and_fast_path() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10)],
        vec![square(2, 2, 8, 8)],
        &policy,
    )
    .unwrap()
    .into_value();
    let quarter_turn = Similarity2::try_from_real_affine(
        Real::zero(),
        Real::from(-1),
        Real::one(),
        Real::zero(),
        Real::from(20),
        Real::from(3),
    )
    .unwrap();

    let rotated = region
        .transform_similarity(&quarter_turn, &policy)
        .unwrap()
        .into_value();

    assert!(matches!(
        certified(rotated.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
    assert_eq!(
        certified(rotated.classify_point(&p(15, 4), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(rotated.classify_point(&p(15, 8), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(rotated.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
}

#[test]
fn exact_profiles_assign_holes_to_the_smallest_containing_material() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(3, 3, 7, 7)],
        &policy,
    )
    .unwrap()
    .into_value();

    let profiles = decided(promoted.boundary_profiles(&policy).unwrap());

    assert_eq!(profiles.len(), 2);
    assert!(profiles[0].holes().is_empty());
    assert_eq!(profiles[1].material_loop_index(), 1);
    assert_eq!(profiles[1].hole_loop_indices(), &[2]);
    assert_eq!(
        decided(promoted.filled_area(&policy).unwrap()),
        Some(Real::from(120))
    );
}

#[test]
fn affine_line_fast_path_preserves_nonzero_and_even_odd_fill_rules() {
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let promoted = CurveRegion2::try_from_native_material_contours(
            vec![double_wound_square(fill_rule)],
            &policy,
        )
        .unwrap()
        .into_value();
        let transformed = promoted
            .transform_affine(
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &policy,
            )
            .unwrap()
            .into_value();

        assert_eq!(transformed.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert_eq!(
            certified(transformed.classify_point(&p(10, 5), &policy).unwrap()),
            Classification::Decided(expected)
        );
        assert!(matches!(
            certified(transformed.native_contours_fast_path(&policy).unwrap()),
            Classification::Decided(_)
        ));
    }
}

#[test]
fn authored_loop_semantics_drive_nonzero_and_even_odd_classification() {
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let path = path_from_contour(&double_wound_square(fill_rule));
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[path],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
            &policy,
        )
        .unwrap()
        .into_value();

        assert_eq!(region.loop_fill_rules(), Some([fill_rule].as_slice()));
        assert_eq!(
            certified(region.classify_point(&p(5, 5), &policy).unwrap()),
            Classification::Decided(expected)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(if fill_rule == FillRule::NonZero {
                Real::from(100)
            } else {
                Real::zero()
            })
        );
    }
}

#[test]
fn nonlinear_curved_winding_honors_authored_fill_rules_exactly() {
    let policy = CurveContext::STRICT;
    for (fill_rule, expected) in [
        (FillRule::NonZero, RegionPointLocation::Inside),
        (FillRule::EvenOdd, RegionPointLocation::Outside),
    ] {
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[double_wound_quadratic_cap()],
            &[CurveRegionLoopRole::Material],
            &[fill_rule],
            &policy,
        )
        .unwrap()
        .into_value();

        assert_eq!(
            region
                .offset(Real::zero(), &sharp_offset(), &policy)
                .unwrap()
                .into_value(),
            region,
            "zero offset must preserve higher-order regions"
        );

        assert_eq!(
            certified(region.classify_point(&p(0, 2), &policy).unwrap()),
            Classification::Decided(expected)
        );
        let expected_depth = i32::from(expected == RegionPointLocation::Inside);
        assert_eq!(
            certified(region.signed_depth(&p(0, 2), &policy).unwrap()),
            Classification::Decided(expected_depth)
        );
        assert_eq!(
            decided(region.filled_area(&policy).unwrap()),
            Some(if fill_rule == FillRule::NonZero {
                q(32, 3)
            } else {
                Real::zero()
            })
        );
        let transformed = region
            .transform_affine(
                &Real::one(),
                &Real::one(),
                &Real::zero(),
                &Real::one(),
                &Real::zero(),
                &Real::zero(),
                &policy,
            )
            .unwrap()
            .into_value();
        assert_eq!(
            certified(transformed.classify_point(&p(2, 2), &policy).unwrap()),
            Classification::Decided(expected)
        );
    }
}

#[test]
fn nonperiodic_self_contact_does_not_claim_a_green_integral_as_filled_area() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[bow_tie_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.filled_area(&policy).unwrap()),
        None,
        "a self-crossing traversal needs arrangement regularization before its Green integral is a filled-set area"
    );
}

#[test]
fn native_contour_constructors_and_signed_depth_need_no_region_wrapper() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10), square(2, 2, 8, 8)],
        vec![square(4, 4, 6, 6)],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.loop_roles(&policy).unwrap()),
        vec![
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Material,
            CurveRegionLoopRole::Hole,
        ]
    );
    assert_eq!(
        certified(region.signed_depth(&p(1, 1), &policy).unwrap()),
        Classification::Decided(1)
    );
    assert_eq!(
        certified(region.signed_depth(&p(3, 3), &policy).unwrap()),
        Classification::Decided(2)
    );
    assert_eq!(
        certified(region.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(1)
    );
    assert_eq!(
        certified(region.signed_depth(&p(0, 5), &policy).unwrap()),
        Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)
    );
    let boundaries = vec![square(2, 2, 8, 8), square(0, 0, 10, 10)];
    let nested = decided(
        CurveRegion2::try_from_native_boundary_contours(boundaries.clone(), &policy)
            .unwrap()
            .into_value(),
    );
    let borrowed = decided(
        CurveRegion2::try_from_native_boundary_contours_borrowed(&boundaries, &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(
        decided(nested.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        certified(nested.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(0)
    );
    assert_eq!(
        certified(borrowed.signed_depth(&p(5, 5), &policy).unwrap()),
        Classification::Decided(0)
    );
}
#[test]
fn authored_line_arc_paths_retain_the_native_offset_engine() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[full_circle_path(5)],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();

    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
    let expanded = region
        .offset(Real::from(2), &sharp_offset(), &policy)
        .unwrap()
        .into_value();
    let bounds = decided(expanded.bounds(&policy).unwrap());
    assert_eq!(bounds.min_x(), &Real::from(-7));
    assert_eq!(bounds.min_y(), &Real::from(-7));
    assert_eq!(bounds.max_x(), &Real::from(7));
    assert_eq!(bounds.max_y(), &Real::from(7));
}

#[test]
fn authored_nested_material_roles_certify_filled_sides_directly() {
    let policy = CurveContext::STRICT;
    let outer = path_from_contour(&square(0, 0, 10, 10));
    let inner = path_from_contour(&square(2, 2, 8, 8));
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[outer, inner],
        &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Material],
        &[FillRule::NonZero, FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(region.filled_side_is_left(&policy).unwrap()),
        &[true, true]
    );
    assert_eq!(
        certified(region.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Decided(_)
    ));
}
#[test]
fn unified_region_chamfer_and_fillet_use_authoritative_path_kernel() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_material_contours(vec![square(0, 0, 4, 4)], &policy)
        .unwrap()
        .into_value();

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(
        decided(chamfered.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    assert_eq!(
        chamfered.loop_fill_rules(),
        Some([FillRule::NonZero].as_slice())
    );
    assert_eq!(
        certified(chamfered.classify_point(&p(2, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );

    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 0, q(3, 4), q(1, 4), &p(1, 1), false, &policy)
            .unwrap()
            .into_value(),
    );
    assert_eq!(
        decided(filleted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material]
    );
    assert_eq!(
        filleted.loop_fill_rules(),
        Some([FillRule::NonZero].as_slice())
    );
    assert_eq!(
        certified(filleted.classify_point(&p(2, 2), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
}

#[test]
fn unified_region_chamfer_and_fillet_edit_materialized_higher_order_loops() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[quadratic_fillet_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .unwrap()
    .into_value();
    assert!(matches!(
        certified(region.native_contours_fast_path(&policy).unwrap()),
        Classification::Uncertain(_)
    ));

    let chamfered = decided(
        region
            .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &policy)
            .unwrap()
            .into_value(),
    );
    let filleted = decided(
        region
            .fillet_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &p(3, 1), false, &policy)
            .unwrap()
            .into_value(),
    );

    assert_eq!(chamfered.boundary_loops()[0].len(), 6);
    assert_eq!(filleted.boundary_loops()[0].len(), 7);
    for edited in [&chamfered, &filleted] {
        assert_eq!(
            decided(edited.loop_roles(&policy).unwrap()),
            vec![CurveRegionLoopRole::Material]
        );
        assert_eq!(
            edited.loop_fill_rules(),
            Some([FillRule::NonZero].as_slice())
        );
    }
}

#[cfg(feature = "predicates")]
#[test]
fn materialized_boundary_paths_obey_terminal_policy_once() {
    let start = Point2::new(Real::pi() + Real::e(), Real::zero());
    let end = Point2::new(Real::e() + Real::pi(), Real::zero());
    let path = CurvePath2::try_new(vec![Curve2::from(QuadraticBezier2::new(
        start,
        p(0, 1),
        end,
    ))])
    .expect("one-curve path construction has no adjacency decision");
    let constructed =
        CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::APPROXIMATE_512)
            .expect("the authorized terminal must construct the symbolic loop");
    assert_eq!(
        constructed.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let region = constructed.into_value();

    let strict = region
        .materialized_boundary_paths(&CurveContext::STRICT)
        .expect("strict materialization must preserve the symbolic closing seam uncertainty");
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );

    let approximate = region
        .materialized_boundary_paths(&CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal must materialize the exact boundary");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let Classification::Decided(paths) = approximate.value else {
        panic!("the symbolic boundary is exactly representable");
    };
    assert_eq!(paths.len(), 1);
    assert_eq!(paths[0].curves().len(), 1);
    assert!(matches!(
        paths[0].curves()[0].geometry(),
        hypercurve::CurveGeometry2::QuadraticBezier(_)
    ));

    assert_eq!(
        region
            .materialized_boundary_paths(&CurveContext::APPROXIMATE_512)
            .expect("terminal replay remains authorized")
            .certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(
        region
            .materialized_boundary_paths(&CurveContext::STRICT)
            .expect("strict replay remains an explicit classification")
            .value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );
}

#[cfg(feature = "predicates")]
#[test]
fn higher_order_region_fillet_obeys_terminal_policy_once() {
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[quadratic_fillet_path()],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &CurveContext::STRICT,
    )
    .unwrap()
    .into_value();
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let center = Point2::new(Real::from(3) + undecidable_zero, Real::one());

    let strict = region
        .fillet_loop_vertex_by_parameters(
            0,
            1,
            q(3, 4),
            q(1, 2),
            &center,
            false,
            &CurveContext::STRICT,
        )
        .unwrap();
    assert_eq!(strict.certainty, CurveCertainty::Certified);
    assert_eq!(
        strict.value,
        Classification::Uncertain(hypercurve::UncertaintyReason::RealSign)
    );

    let approximate = region
        .fillet_loop_vertex_by_parameters(
            0,
            1,
            q(3, 4),
            q(1, 2),
            &center,
            false,
            &CurveContext::APPROXIMATE_512,
        )
        .unwrap();
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let Classification::Decided(filleted) = approximate.value else {
        panic!("the authorized terminal must complete the higher-order region fillet");
    };
    assert_eq!(filleted.boundary_loops()[0].len(), 7);
}

#[test]
fn unified_region_offset_expands_material_and_contracts_holes() {
    let policy = CurveContext::STRICT;
    let region = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10)],
        vec![square(3, 3, 7, 7)],
        &policy,
    )
    .unwrap()
    .into_value();

    let offset = region
        .offset(Real::one(), &sharp_offset(), &policy)
        .unwrap()
        .into_value();

    assert_eq!(
        certified(offset.classify_point(&p(0, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    assert_eq!(
        certified(offset.classify_point(&p(3, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Inside),
        "positive region offset must contract a hole"
    );
    assert_eq!(
        certified(offset.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    assert_eq!(
        decided(offset.filled_area(&policy).unwrap()),
        Some(Real::from(140))
    );
}

#[test]
fn region_promotion_retains_hole_role_for_projection() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::try_from_native_contours(
        vec![square(0, 0, 10, 10)],
        vec![square(2, 2, 8, 8)],
        &policy,
    )
    .unwrap()
    .into_value();

    assert_eq!(
        decided(promoted.loop_roles(&policy).unwrap()),
        vec![CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole]
    );
    assert_eq!(
        decided(promoted.filled_side_is_left(&policy).unwrap()),
        &[true, false]
    );
    assert_eq!(
        certified(promoted.classify_point(&p(5, 5), &policy).unwrap()),
        Classification::Decided(RegionPointLocation::Outside)
    );
    let exact_profiles = decided(promoted.boundary_profiles(&policy).unwrap());
    assert_eq!(exact_profiles.len(), 1);
    assert_eq!(exact_profiles[0].material_loop_index(), 0);
    assert_eq!(exact_profiles[0].hole_loop_indices(), &[1]);
    assert_eq!(exact_profiles[0].holes().len(), 1);

    let options = FiniteProjectionOptions::try_new(0.01).unwrap();
    let profiles = decided(
        promoted
            .project_to_finite_profiles(&options, &policy)
            .unwrap(),
    );
    assert_eq!(profiles.len(), 1);
    assert_eq!(profiles[0].holes().len(), 1);
}

#[test]
fn empty_region_promotion_is_decided_and_reusable() {
    let policy = CurveContext::STRICT;
    let promoted = CurveRegion2::empty();

    assert!(promoted.is_empty());
    assert!(decided(promoted.loop_roles(&policy).unwrap()).is_empty());
    assert!(decided(promoted.filled_side_is_left(&policy).unwrap()).is_empty());
    assert!(
        decided(promoted.native_contours_fast_path(&policy).unwrap())
            .material_contours()
            .is_empty()
    );
    assert_eq!(CurveRegion2::empty(), CurveRegion2::default());
}
