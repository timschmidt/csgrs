#[cfg(feature = "predicates")]
use hypercurve::{
    BezierParameter2, BezierSplitFragment2, CurveCertainty, CurveFamily2, CurveOperation2,
    CurvePathBooleanFragmentAction2, ExactCurveError, QuadraticBezier2, RationalQuadraticBezier2,
    RegionPointLocation, UncertaintyReason,
};
use hypercurve::{
    BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveBoundaryInteriorSide2,
    CurveContext, CurveGeometry2, CurvePath2, CurvePathOverlapAction2, LineSeg2, Point2,
    RationalBezier2, RationalBezierIntersectionPointEvidence2, RationalBezierOverlapOrientation2,
    Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

#[cfg(feature = "predicates")]
fn symbolic_rectangle_path(width: Real) -> CurvePath2 {
    let points = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(width.clone(), Real::zero()),
        Point2::new(width, Real::one()),
        Point2::new(Real::zero(), Real::one()),
    ];
    CurvePath2::try_new(
        (0..points.len())
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn assert_real_close(left: &Real, right: &Real, tolerance: f64) {
    let left = left.to_f64_lossy().expect("left Real is approximable");
    let right = right.to_f64_lossy().expect("right Real is approximable");
    assert!(
        (left - right).abs() <= tolerance,
        "expected {left} to be within {tolerance} of {right}"
    );
}

#[test]
fn top_level_rational_intersection_immediately_returns_sources_and_topology() {
    let first = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![Point2::new(r(0), r(0)), Point2::new(q(1, 2), r(0)), p(1, 1)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let second = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![Point2::new(r(0), q(1, 4)), Point2::new(r(1), q(1, 4))],
            vec![r(1), r(1)],
        )
        .unwrap(),
    ));

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 1);
    assert!(evidence.is_complete());
    assert!(!evidence.is_disjoint());
    assert_eq!(evidence.contacts().len(), 1);
    assert!(evidence.blockers().is_empty());
    let contact = &evidence.contacts()[0];
    assert_eq!(contact.first().exact_curve_parameter(), Some(q(1, 2)));
    assert_eq!(contact.second().exact_curve_parameter(), Some(q(1, 2)));
    assert!(matches!(
        contact.point(),
        RationalBezierIntersectionPointEvidence2::Exact(point)
            if point == &Point2::new(q(1, 2), q(1, 4))
    ));
    assert_eq!(topology.first().len(), 1);
    assert_eq!(topology.second().len(), 1);
    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.second()[0].fragments().len(), 2);
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 4);
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 4);
}

#[test]
#[cfg(feature = "predicates")]
fn top_level_intersection_retains_implicit_conic_transversality() {
    let conic = Curve2::from(
        RationalBezier2::try_new(vec![p(1, 0), p(1, 1), p(0, 1)], vec![r(1), r(1), r(2)]).unwrap(),
    );
    let cubic_line = Curve2::from(
        RationalBezier2::try_new(
            vec![
                Point2::new(q(3, 5), r(-1)),
                Point2::new(q(3, 5), r(0)),
                Point2::new(q(3, 5), r(1)),
                Point2::new(q(3, 5), r(2)),
            ],
            vec![r(1); 4],
        )
        .unwrap(),
    );

    let result = conic
        .intersect_curve(&cubic_line, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(result.contacts().len(), 1);
    assert!(result.contacts()[0].is_certified_transverse());
}

#[test]
fn top_level_nurbs_intersection_deduplicates_a_shared_knot_contact() {
    let spline = Curve2::try_nurbs(
        1,
        vec![p(0, 0), p(1, 1), p(2, 0)],
        vec![r(1), r(1), r(1)],
        vec![r(0), r(0), r(1), r(2), r(2)],
    )
    .unwrap();
    let line = Curve2::from(LineSeg2::try_new(p(0, 1), p(2, 1)).unwrap());

    let topology = spline
        .intersection_topology(&line, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 2);
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 1, "{evidence:?}");
    let contact = &evidence.contacts()[0];
    assert_eq!(contact.first().exact_curve_parameter(), Some(r(1)));
    assert_eq!(contact.second().exact_curve_parameter(), Some(q(1, 2)));
    assert_eq!(topology.first().len(), 2);
    assert_eq!(topology.second().len(), 1);
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 4);
}

#[test]
fn top_level_shared_component_retains_certified_overlap() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let second = first.clone();
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete());
    assert!(!evidence.is_disjoint());
    assert!(evidence.contacts().is_empty());
    assert!(evidence.blockers().is_empty());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Same
    );
    let graph = topology.arrangement_graph().unwrap();
    assert_eq!(graph.len(), 2);
    let traversal =
        decided(graph.traverse_retained_deduplicating_materialized_overlaps(&CurveContext::STRICT));
    assert_eq!(traversal.shadowed_fragment_indices(), &[1]);
    assert_eq!(traversal.traversal().len(), 1);
}

#[test]
fn independently_rebuilt_degree_elevated_rational_image_is_a_complete_overlap() {
    let base =
        RationalBezier2::try_new(vec![p(0, 0), p(2, 3), p(4, 0)], vec![r(1), r(2), r(1)]).unwrap();
    let elevated = base.elevated_to_degree(5).unwrap();
    let independent = RationalBezier2::try_new(
        elevated.control_points().to_vec(),
        elevated.weights().to_vec(),
    )
    .unwrap();
    let first = Curve2::from(base);
    let second = Curve2::from(independent);

    let evidence = first
        .intersect_curve(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(evidence.overlaps()[0].first_range().start(), &Real::zero());
    assert_eq!(evidence.overlaps()[0].first_range().end(), &Real::one());
    assert_eq!(evidence.overlaps()[0].second_range().start(), &Real::zero());
    assert_eq!(evidence.overlaps()[0].second_range().end(), &Real::one());
    assert_eq!(
        evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Same
    );
}

#[test]
fn top_level_partial_nonlinear_overlap_splits_at_retained_ranges() {
    let policy = CurveContext::STRICT;
    let source = RationalBezier2::try_new(
        vec![p(0, 0), p(1, 3), p(3, 3), p(4, 0)],
        vec![r(1), r(2), r(3), r(4)],
    )
    .unwrap();
    let first_curve = decided(
        source
            .subcurve_between_exact(&Real::zero(), &q(3, 4), &policy)
            .unwrap(),
    );
    let second_curve = decided(
        source
            .subcurve_between_exact(&q(1, 4), &Real::one(), &policy)
            .unwrap(),
    );
    let first = Curve2::new(CurveGeometry2::RationalBezier(first_curve));
    let second = Curve2::new(CurveGeometry2::RationalBezier(second_curve));

    let topology = first
        .intersection_topology(&second, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert!(evidence.contacts().is_empty());
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_eq!(overlap.first_span_index(), 0);
    assert_eq!(overlap.second_span_index(), 0);
    assert_eq!(overlap.first_range().start(), &q(1, 3));
    assert_eq!(overlap.first_range().end(), &Real::one());
    assert_eq!(overlap.second_range().start(), &Real::zero());
    assert_eq!(overlap.second_range().end(), &q(2, 3));

    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.second()[0].fragments().len(), 2);
    let graph = topology.arrangement_graph_view().unwrap();
    assert_eq!(graph.len(), 4);
    let traversal = decided(graph.traverse_retained_deduplicating_materialized_overlaps(&policy));
    assert_eq!(traversal.shadowed_fragment_indices().len(), 1);
}

#[test]
#[cfg(feature = "predicates")]
fn top_level_line_image_overlap_preserves_algebraic_split_boundary() {
    let first = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let second = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(vec![Point2::new(q(1, 2), r(0)), p(1, 0)], vec![r(1), r(1)])
            .unwrap(),
    ));

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0].first_range().start(),
        BezierParameter2::Algebraic(_)
    ));

    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert!(matches!(
        topology.first()[0].fragments()[1],
        BezierSplitFragment2::AlgebraicEndpointImages { .. }
    ));
    assert_eq!(topology.second()[0].fragments().len(), 1);
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 3);

    let first_path = CurvePath2::try_new(vec![first]).unwrap();
    let second_path = CurvePath2::try_new(vec![second]).unwrap();
    let path_topology = first_path
        .intersection_topology(&second_path, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(
        path_topology.first()[0].materializations()[0]
            .fragments()
            .len(),
        2
    );
    assert_eq!(
        path_topology.second()[0].materializations()[0]
            .fragments()
            .len(),
        1
    );
}

#[test]
#[cfg(feature = "predicates")]
fn path_boolean_consumes_algebraic_line_image_overlap_boundary() {
    let parameterized_bottom = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![p(0, 0), Point2::new(q(1, 4), r(0)), p(1, 0)],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let first = CurvePath2::try_new(vec![
        parameterized_bottom,
        Curve2::from(LineSeg2::try_new(p(1, 0), p(1, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, 1), p(0, 1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 1), p(0, 0)).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(Point2::new(q(1, 2), r(0)), Point2::new(q(1, 2), r(-1))).unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(Point2::new(q(1, 2), r(-1)), p(1, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, -1), p(1, 0)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, 0), Point2::new(q(1, 2), r(0))).unwrap()),
    ])
    .unwrap();

    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0].overlap().first_range().start(),
        BezierParameter2::Algebraic(_)
    ));

    let selection = selections.union();
    assert_eq!(selection.overlap_resolutions().len(), 1);
    assert_eq!(selection.arrangement_graph_view().unwrap().len(), 7);
    let traversal = selection.traversal_view().unwrap();
    assert_eq!(traversal.closed_count(), 1);
    assert!(selection.region_view().is_ok());
}

#[test]
#[cfg(feature = "predicates")]
fn path_boolean_consumes_irrational_polynomial_graph_overlap() {
    let partial_parabola = Curve2::new(CurveGeometry2::RationalBezier(
        RationalBezier2::try_new(
            vec![
                Point2::new(q(1, 2), q(1, 4)),
                Point2::new(q(3, 4), q(1, 2)),
                p(1, 1),
            ],
            vec![r(1), r(1), r(1)],
        )
        .unwrap(),
    ));
    let nonlinear_parabola = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 8), r(0)),
            Point2::new(q(1, 3), q(1, 24)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![r(1); 5],
    )
    .unwrap();
    let first = CurvePath2::try_new(vec![
        partial_parabola,
        Curve2::from(LineSeg2::try_new(p(1, 1), Point2::new(q(1, 2), q(1, 4))).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::new(CurveGeometry2::RationalBezier(
            nonlinear_parabola.reversed(),
        )),
        Curve2::from(LineSeg2::try_new(p(0, 0), p(0, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, -1), p(1, -1)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(1, -1), p(1, 1)).unwrap()),
    ])
    .unwrap();

    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert!(matches!(
        evidence.overlaps()[0].overlap().second_range().start(),
        BezierParameter2::Algebraic(_)
    ));

    let selection = selections.union();
    assert_eq!(selection.overlap_resolutions().len(), 1);
    assert_eq!(selection.traversal_view().unwrap().closed_count(), 1);
    let region = selection.region_view().unwrap();
    assert!(matches!(
        region
            .materialized_boundary_paths(&CurveContext::STRICT)
            .unwrap()
            .into_value(),
        Classification::Uncertain(_)
    ));
}

#[test]
#[cfg(feature = "predicates")]
fn path_boolean_surfaces_report_terminal_use_and_preserve_strict_rejection() {
    let first = symbolic_rectangle_path(Real::pi() + Real::e());
    let second = symbolic_rectangle_path(Real::e() + Real::pi());
    let approximate = CurveContext::APPROXIMATE_512;

    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &approximate,
        )
        .expect("the authorized terminal must resolve equivalent symbolic boundaries");
    assert_eq!(selections.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(
        selections
            .value
            .union()
            .region_view()
            .unwrap()
            .boundary_loops()
            .len(),
        1
    );

    let strict = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap_err();
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Boolean
    ));

    let selection = first
        .boolean_selection(
            &second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &approximate,
        )
        .expect("single-selection Boolean must report the same terminal");
    assert_eq!(selection.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(
        selection
            .value
            .region_view()
            .unwrap()
            .boundary_loops()
            .len(),
        1
    );

    let region = first
        .boolean_region(
            &second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &approximate,
        )
        .expect("materialized path Boolean must report the same terminal");
    assert_eq!(region.certainty, CurveCertainty::Approximate512Consumed);
    assert_eq!(region.value.boundary_loops().len(), 1);
}

#[test]
fn top_level_polynomial_trims_reuse_certified_source_lineage() {
    let source = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
        p(0, 0),
        p(1, 3),
        p(3, 3),
        p(4, 0),
    )));
    let first = source.subcurve(Real::zero(), q(3, 4)).unwrap();
    let second = source.subcurve(q(1, 4), Real::one()).unwrap();

    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(evidence.overlaps()[0].first_range().start(), &q(1, 3));
    assert_eq!(evidence.overlaps()[0].first_range().end(), &Real::one());
    assert_eq!(evidence.overlaps()[0].second_range().start(), &Real::zero());
    assert_eq!(evidence.overlaps()[0].second_range().end(), &q(2, 3));
    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.second()[0].fragments().len(), 2);

    let reversed = second.reversed().unwrap();
    let reversed_evidence = first
        .intersect_curve(&reversed, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert!(reversed_evidence.is_complete());
    assert_eq!(reversed_evidence.overlaps().len(), 1);
    assert_eq!(
        reversed_evidence.overlaps()[0].second_range(),
        &hypercurve::ParamRange::new(Real::one(), q(1, 3))
    );
    assert_eq!(
        reversed_evidence.overlaps()[0].orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn top_level_disjoint_curves_produce_a_complete_empty_evidence() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
    let second = Curve2::from(LineSeg2::try_new(p(0, 2), p(1, 2)).unwrap());
    let evidence = first
        .intersect_curve(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();

    assert!(evidence.is_complete());
    assert!(evidence.is_disjoint());
    assert!(evidence.contacts().is_empty());
    assert!(evidence.blockers().is_empty());
}

#[test]
fn top_level_arc_dispatch_filters_circle_witnesses_and_retains_exact_parameters() {
    let first =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let second =
        Curve2::from(CircularArc2::try_from_center(p(3, 0), p(13, 0), p(8, 0), true).unwrap());
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 4);

    assert!(evidence.is_complete());
    assert_eq!(evidence.contacts().len(), 1);
    let contact = &evidence.contacts()[0];
    assert!(contact.first().local_parameter().is_exact());
    assert!(contact.second().local_parameter().is_exact());
    assert!(matches!(
        contact.point(),
        RationalBezierIntersectionPointEvidence2::Exact(point) if point == &p(4, 3)
    ));
    assert_eq!(topology.result().contacts().len(), 1);
}

#[test]
#[cfg(feature = "predicates")]
fn curve_and_path_intersections_report_terminal_use_without_upgrading_arc_caches() {
    let undecidable_zero = (Real::pi() + Real::e()) - (Real::e() + Real::pi());
    let arc = Curve2::from(
        CircularArc2::try_from_center(
            p(3, 0),
            p(3, 2),
            Point2::new(Real::from(3_i8) + undecidable_zero, Real::one()),
            false,
        )
        .unwrap(),
    );
    let line = Curve2::from(LineSeg2::try_new(p(2, 1), p(5, 1)).unwrap());

    let approximate = arc
        .intersect_curve(&line, &CurveContext::APPROXIMATE_512)
        .expect("the authorized terminal must resolve the ambiguous semicircle");
    assert_eq!(
        approximate.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(approximate.value.is_complete());
    assert_eq!(approximate.value.contacts().len(), 1);
    assert!(matches!(
        approximate.value.contacts()[0].point(),
        RationalBezierIntersectionPointEvidence2::Exact(_)
    ));

    let strict = arc
        .intersect_curve(&line, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Intersection
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let topology = arc
        .intersection_topology(&line, &CurveContext::APPROXIMATE_512)
        .expect("topology must replay the authorized terminal from retained arc facts");
    assert_eq!(topology.certainty, CurveCertainty::Approximate512Consumed);
    assert!(topology.value.result().is_complete());
    assert_eq!(topology.value.result().contacts().len(), 1);

    let arc_path = CurvePath2::try_new(vec![arc]).unwrap();
    let line_path = CurvePath2::try_new(vec![line]).unwrap();
    let path_result = arc_path
        .intersect_path(&line_path, &CurveContext::APPROXIMATE_512)
        .expect("path intersection must preserve terminal certainty");
    assert_eq!(
        path_result.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert!(path_result.value.is_complete());
    assert_eq!(path_result.value.contacts().len(), 1);

    let strict_path = arc_path
        .intersection_topology(&line_path, &CurveContext::STRICT)
        .unwrap_err();
    assert!(matches!(
        strict_path,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Intersection
                && blocker.reason() == UncertaintyReason::RealSign
    ));
}

#[test]
fn native_line_arc_dispatch_preserves_operand_order_and_exact_parameters() {
    let line = Curve2::from(LineSeg2::try_new(p(4, -4), p(4, 4)).unwrap());
    let arc =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let policy = CurveContext::STRICT;

    let topology = line
        .intersection_topology(&arc, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.span_pair_count(), 2);
    assert!(evidence.is_complete());
    assert_eq!(evidence.contacts().len(), 1);
    assert_eq!(
        evidence.contacts()[0].first().exact_curve_parameter(),
        Some(q(7, 8))
    );
    assert!(evidence.contacts()[0].second().local_parameter().is_exact());
    assert!(matches!(
        evidence.contacts()[0].point(),
        RationalBezierIntersectionPointEvidence2::Exact(point) if point == &p(4, 3)
    ));
    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.second()[0].fragments().len(), 2);
    assert_eq!(topology.second()[1].fragments().len(), 1);

    let reversed_evidence = arc.intersect_curve(&line, &policy).unwrap().into_value();
    assert_eq!(reversed_evidence.contacts().len(), 1);
    assert!(
        reversed_evidence.contacts()[0]
            .first()
            .local_parameter()
            .is_exact()
    );
    assert_eq!(
        reversed_evidence.contacts()[0]
            .second()
            .exact_curve_parameter(),
        Some(q(7, 8))
    );
}

#[test]
fn native_arc_dispatch_retains_partial_same_circle_overlap_ranges() {
    let first =
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap());
    let second =
        Curve2::from(CircularArc2::try_from_center(p(4, 3), p(0, 5), p(0, 0), false).unwrap());
    let policy = CurveContext::STRICT;
    let topology = first
        .intersection_topology(&second, &policy)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_ne!(overlap.first_range().start(), &Real::zero());
    assert_eq!(overlap.first_range().end(), &Real::one());
    assert_eq!(overlap.second_range().start(), &Real::zero());
    assert_eq!(overlap.second_range().end(), &Real::one());
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );

    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.first()[1].fragments().len(), 1);
    assert_eq!(topology.second()[0].fragments().len(), 1);

    let reversed =
        Curve2::from(CircularArc2::try_from_center(p(0, 5), p(4, 3), p(0, 0), true).unwrap());
    let reversed_evidence = first
        .intersect_curve(&reversed, &policy)
        .unwrap()
        .into_value();
    assert_eq!(reversed_evidence.overlaps().len(), 1);
    let reversed_overlap = &reversed_evidence.overlaps()[0];
    assert_eq!(reversed_overlap.second_range().start(), &Real::one());
    assert_eq!(reversed_overlap.second_range().end(), &Real::zero());
    assert_eq!(
        reversed_overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn path_boolean_selection_resolves_partial_same_circle_arc_boundaries() {
    let first = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap()),
        Curve2::from(LineSeg2::try_new(p(-5, 0), p(5, 0)).unwrap()),
    ])
    .unwrap();
    let second = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(p(4, 3), p(0, 5), p(0, 0), false).unwrap()),
        Curve2::from(LineSeg2::try_new(p(0, 5), p(4, 3)).unwrap()),
    ])
    .unwrap();
    let first_area = first
        .bezier_boundary_loop(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .boundary_loop()
        .signed_area(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let first_area = decided(first_area).unwrap();
    let second_area = second
        .bezier_boundary_loop(&CurveContext::STRICT)
        .unwrap()
        .into_value()
        .boundary_loop()
        .signed_area(&CurveContext::STRICT)
        .unwrap()
        .into_value();
    let second_area = decided(second_area).unwrap();
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);

    let cases = [
        (BooleanOp::Union, first_area.clone()),
        (BooleanOp::Intersection, second_area.clone()),
        (BooleanOp::Difference, &first_area - &second_area),
        (BooleanOp::Xor, &first_area - &second_area),
    ];
    for (operation, expected_area) in cases {
        let selection = selections.selection(operation);
        let traversal = selection
            .traversal_view()
            .unwrap_or_else(|error| panic!("{operation:?} traversal: {error:?}"));
        assert!(
            traversal.chains().iter().all(|chain| chain.is_closed()),
            "{operation:?}: {:?}",
            traversal.chains()
        );
        let region = selection
            .region_view()
            .unwrap_or_else(|error| panic!("{operation:?} region: {error:?}"));
        let actual_area = region
            .signed_area(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let actual_area = decided(actual_area)
            .unwrap_or_else(|| panic!("{operation:?} did not retain an exact area"));
        assert_real_close(&actual_area, &expected_area, 1.0e-10);
    }
}

fn rectangle(x0: i32, y0: i32, x1: i32, y1: i32) -> CurvePath2 {
    let points = [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)];
    CurvePath2::try_new(
        (0..4)
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(points[index].clone(), points[(index + 1) % 4].clone())
                        .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn closed_under_curve(curve: Curve2, lower_y: i32) -> CurvePath2 {
    let start = curve.start().clone();
    let end = curve.end().clone();
    let lower_end = Point2::new(end.x().clone(), r(lower_y));
    let lower_start = Point2::new(start.x().clone(), r(lower_y));
    CurvePath2::try_new(vec![
        curve,
        Curve2::from(LineSeg2::try_new(end, lower_end.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(lower_end, lower_start.clone()).unwrap()),
        Curve2::from(LineSeg2::try_new(lower_start, start).unwrap()),
    ])
    .unwrap()
}

#[test]
fn path_boolean_consumes_partial_nonlinear_shared_boundary() {
    let policy = CurveContext::STRICT;
    let source = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
        p(0, 0),
        p(1, 3),
        p(3, 3),
        p(4, 0),
    )));
    let first_curve = source.subcurve(Real::zero(), q(3, 4)).unwrap();
    let second_curve = source.subcurve(q(1, 4), Real::one()).unwrap();
    let first = closed_under_curve(first_curve, -5);
    let second = closed_under_curve(second_curve, -6);
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Right,
            CurveBoundaryInteriorSide2::Right,
            &policy,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    assert_eq!(
        evidence.overlaps()[0].overlap().first_range().start(),
        &q(1, 3)
    );
    assert_eq!(
        evidence.overlaps()[0].overlap().second_range().end(),
        &q(2, 3)
    );

    for operation in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        let selection = selections.selection(operation);
        assert_eq!(selection.overlap_resolutions().len(), 1);
        assert!(selection.kept_fragment_count() > 0);
        let traversal = selection
            .traversal_view()
            .unwrap_or_else(|error| panic!("{operation:?} traversal: {error:?}"));
        assert!(traversal.chains().iter().all(|chain| chain.is_closed()));
        let region = selection
            .region_view()
            .unwrap_or_else(|error| panic!("{operation:?} region: {error:?}"));
        assert!(!region.boundary_loops().is_empty());
        assert!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            )
            .is_some()
        );
    }
}

#[test]
fn path_pair_immediate_topology_splits_each_authored_curve_once() {
    let first = rectangle(0, 0, 2, 2);
    let second = rectangle(1, -1, 3, 1);
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();
    assert_eq!(evidence.authored_curve_pair_count(), 16);
    assert_eq!(evidence.candidate_curve_pair_count(), 2);
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert!(evidence.overlaps().is_empty());

    assert_eq!(topology.first().len(), 4);
    assert_eq!(topology.second().len(), 4);
    assert_eq!(
        topology
            .first()
            .iter()
            .flat_map(|curve| curve.materializations())
            .map(|span| span.fragments().len())
            .sum::<usize>(),
        6
    );
    assert_eq!(
        topology
            .second()
            .iter()
            .flat_map(|curve| curve.materializations())
            .map(|span| span.fragments().len())
            .sum::<usize>(),
        6
    );
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 12);
    assert_eq!(topology.arrangement_graph_view().unwrap().len(), 12);
}

#[test]
fn path_overlap_ownership_uses_exact_orientation_and_boolean_side_logic() {
    let first = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap(),
    )])
    .unwrap();
    let same = first.clone();
    let reversed = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(p(2, 0), p(0, 0)).unwrap(),
    )])
    .unwrap();
    let policy = CurveContext::STRICT;
    let same_evidence = first.intersect_path(&same, &policy).unwrap().into_value();
    let reversed_evidence = first
        .intersect_path(&reversed, &policy)
        .unwrap()
        .into_value();

    let action = |evidence: &hypercurve::CurvePathIntersectionResult2, operation| {
        evidence.resolve_overlap_ownership(
            operation,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
        )[0]
        .action()
    };
    assert_eq!(
        action(&same_evidence, BooleanOp::Union),
        CurvePathOverlapAction2::KeepFirst
    );
    assert_eq!(
        action(&same_evidence, BooleanOp::Intersection),
        CurvePathOverlapAction2::KeepFirst
    );
    assert_eq!(
        action(&same_evidence, BooleanOp::Difference),
        CurvePathOverlapAction2::DiscardBoth
    );
    assert_eq!(
        action(&same_evidence, BooleanOp::Xor),
        CurvePathOverlapAction2::DiscardBoth
    );

    assert_eq!(
        action(&reversed_evidence, BooleanOp::Union),
        CurvePathOverlapAction2::DiscardBoth
    );
    assert_eq!(
        action(&reversed_evidence, BooleanOp::Intersection),
        CurvePathOverlapAction2::DiscardBoth
    );
    assert_eq!(
        action(&reversed_evidence, BooleanOp::Difference),
        CurvePathOverlapAction2::KeepFirst
    );
    assert_eq!(
        action(&reversed_evidence, BooleanOp::Xor),
        CurvePathOverlapAction2::DiscardBoth
    );
}

#[test]
fn native_line_dispatch_retains_partial_overlap_ranges_and_split_endpoints() {
    let first = Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap());
    let second = Curve2::from(LineSeg2::try_new(p(2, 0), p(6, 0)).unwrap());
    let topology = first
        .intersection_topology(&second, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let evidence = topology.result();

    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = &evidence.overlaps()[0];
    assert_eq!(overlap.first_range().start(), &q(1, 2));
    assert_eq!(overlap.first_range().end(), &r(1));
    assert_eq!(overlap.second_range().start(), &r(0));
    assert_eq!(overlap.second_range().end(), &q(1, 2));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Same
    );

    assert_eq!(topology.first()[0].fragments().len(), 2);
    assert_eq!(topology.second()[0].fragments().len(), 2);

    let reversed = Curve2::from(LineSeg2::try_new(p(6, 0), p(2, 0)).unwrap());
    let reversed_evidence = first
        .intersect_curve(&reversed, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let reversed_overlap = &reversed_evidence.overlaps()[0];
    assert_eq!(reversed_overlap.second_range().start(), &r(1));
    assert_eq!(reversed_overlap.second_range().end(), &q(1, 2));
    assert_eq!(
        reversed_overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );
}

#[test]
fn path_boolean_selection_resolves_partial_reversed_shared_line_boundaries() {
    let first = rectangle(0, 0, 2, 4);
    let second = rectangle(2, 1, 4, 3);
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.overlaps().len(), 1);
    let overlap = evidence.overlaps()[0].overlap();
    assert_eq!(overlap.first_range().start(), &q(1, 4));
    assert_eq!(overlap.first_range().end(), &q(3, 4));
    assert_eq!(overlap.second_range().start(), &r(1));
    assert_eq!(overlap.second_range().end(), &r(0));
    assert_eq!(
        overlap.orientation(),
        RationalBezierOverlapOrientation2::Reversed
    );

    let cases = [
        (BooleanOp::Union, 8_usize, r(12)),
        (BooleanOp::Intersection, 0_usize, r(0)),
        (BooleanOp::Difference, 6_usize, r(8)),
        (BooleanOp::Xor, 8_usize, r(12)),
    ];
    for (operation, expected_kept, expected_area) in cases {
        let selection = selections.selection(operation);
        assert_eq!(selection.kept_fragment_count(), expected_kept);
        let region = selection
            .region_view()
            .unwrap_or_else(|error| panic!("{operation:?} region: {error:?}"));
        assert_eq!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            ),
            Some(expected_area)
        );
    }
}

#[test]
fn path_boolean_selection_materializes_exact_regularized_operation_matrix() {
    let first = rectangle(0, 0, 2, 2);
    let second = rectangle(1, -1, 3, 1);
    let policy = CurveContext::STRICT;
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap()
        .into_value();
    let cases = [
        (BooleanOp::Union, r(7), 8_usize),
        (BooleanOp::Intersection, r(1), 4_usize),
        (BooleanOp::Difference, r(3), 6_usize),
        (BooleanOp::Xor, r(6), 12_usize),
    ];

    for (operation, expected_area, expected_kept) in cases {
        let selection = selections.selection(operation);
        assert_eq!(selection.kept_fragment_count(), expected_kept);
        assert_eq!(
            selection.arrangement_graph_view().unwrap().len(),
            expected_kept
        );
        let traversal = selection.traversal_view().unwrap();
        assert!(std::ptr::eq(traversal, selection.traversal_view().unwrap()));
        assert!(traversal.chains().iter().all(|chain| chain.is_closed()));
        let region = selections.region(operation);
        assert!(std::ptr::eq(region, selection.region_view().unwrap()));
        assert_eq!(
            decided(region.signed_area(&policy).unwrap().into_value()),
            Some(expected_area)
        );
    }

    let direct = first
        .boolean_region(
            &second,
            BooleanOp::Union,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &policy,
        )
        .unwrap()
        .into_value();
    assert_eq!(
        decided(direct.signed_area(&policy).unwrap().into_value()),
        Some(r(7))
    );
}

#[test]
fn path_boolean_selection_consumes_complete_shared_boundaries() {
    let first = rectangle(0, 0, 2, 2);
    let second = first.clone();
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let cases = [
        (BooleanOp::Union, 4_usize, r(4)),
        (BooleanOp::Intersection, 4_usize, r(4)),
        (BooleanOp::Difference, 0_usize, r(0)),
        (BooleanOp::Xor, 0_usize, r(0)),
    ];

    for (operation, expected_kept, expected_area) in cases {
        let selection = selections.selection(operation);
        assert_eq!(selection.overlap_resolutions().len(), 4);
        assert_eq!(selection.kept_fragment_count(), expected_kept);
        let traversal = selection
            .traversal_view()
            .unwrap_or_else(|error| panic!("{operation:?} traversal: {error:?}"));
        assert!(
            traversal.chains().iter().all(|chain| chain.is_closed()),
            "{operation:?}: {:?}",
            traversal.chains()
        );
        let region = selection
            .region_view()
            .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
        assert_eq!(
            decided(
                region
                    .signed_area(&CurveContext::STRICT)
                    .unwrap()
                    .into_value()
            ),
            Some(expected_area)
        );
    }
}

#[test]
fn path_boolean_selection_preserves_disjoint_exact_conic_boundaries() {
    let circle = |center_x: i32| {
        CurvePath2::try_new(vec![Curve2::from(
            CircularArc2::try_from_center(
                p(center_x + 1, 0),
                p(center_x + 1, 0),
                p(center_x, 0),
                false,
            )
            .unwrap(),
        )])
        .unwrap()
    };
    let first = circle(0);
    let second = circle(4);
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();

    let union = selections.union();
    assert_eq!(union.kept_fragment_count(), 8);
    assert_eq!(union.region_view().unwrap().boundary_loops().len(), 2);

    let intersection = selections.intersection();
    assert_eq!(intersection.kept_fragment_count(), 0);
    assert!(intersection.region_view().unwrap().is_empty());
}

#[test]
fn path_boolean_selection_traverses_overlapping_circles_with_exact_radical_splits() {
    let circle = |center_x: i32| {
        CurvePath2::try_new(vec![Curve2::from(
            CircularArc2::try_from_center(
                p(center_x + 1, 0),
                p(center_x + 1, 0),
                p(center_x, 0),
                false,
            )
            .unwrap(),
        )])
        .unwrap()
    };
    let first = circle(0);
    let second = circle(1);
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    assert!(evidence.contacts().iter().all(|contact| {
        contact.contact().first().local_parameter().is_exact()
            && contact.contact().second().local_parameter().is_exact()
    }));
    assert_eq!(selections.topology().result().contacts().len(), 2);

    for operation in [BooleanOp::Union, BooleanOp::Intersection] {
        let selection = selections.selection(operation);
        assert!(selection.kept_fragment_count() > 0);
        assert!(
            selection
                .fragments()
                .iter()
                .all(|fragment| !fragment.fragment().is_algebraic_endpoint_images())
        );
        selection.arrangement_graph_view().unwrap();
        let traversal = selection.traversal_view().unwrap();
        assert!(
            traversal.chains().iter().all(|chain| chain.is_closed()),
            "{operation:?}: {:?}",
            traversal.chains()
        );
        let region = selection.region_view().unwrap();
        assert_eq!(region.boundary_loops().len(), 1);
        assert!(!region.boundary_loops()[0].has_algebraic_fragments());
    }
}

#[test]
#[cfg(feature = "predicates")]
fn path_difference_and_xor_reverse_algebraic_parabola_contacts_exactly() {
    let first = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4))),
        Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap()),
    ])
    .unwrap();
    let second = rectangle(-3, 2, 3, 5);
    let selections = first
        .boolean_selections(
            &second,
            CurveBoundaryInteriorSide2::Left,
            CurveBoundaryInteriorSide2::Left,
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
    let evidence = selections.result();
    assert!(evidence.is_complete(), "{:?}", evidence.blockers());
    assert_eq!(evidence.contacts().len(), 2);
    let topology = selections.topology();
    assert!(
        topology
            .first()
            .iter()
            .chain(topology.second())
            .flat_map(|split| split.materializations())
            .flat_map(|materialization| materialization.fragments())
            .any(|fragment| fragment.is_algebraic_endpoint_images())
    );

    for operation in [BooleanOp::Difference, BooleanOp::Xor] {
        let selection = selections.selection(operation);
        assert!(selection.fragments().iter().any(|fragment| {
            fragment.action() == CurvePathBooleanFragmentAction2::KeepReversed
                && fragment.fragment().is_algebraic_endpoint_images()
        }));
        let traversal = selection
            .traversal_view()
            .unwrap_or_else(|error| panic!("{operation:?} traversal: {error:?}"));
        assert!(
            traversal.chains().iter().all(|chain| chain.is_closed()),
            "{operation:?}: {:?}",
            traversal.chains()
        );
        let region = selection
            .region_view()
            .unwrap_or_else(|error| panic!("{operation:?}: {error:?}"));
        assert!(region.has_algebraic_fragments());
        assert_eq!(
            region
                .classify_point(&p(0, 1), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Inside),
            "{operation:?} retained algebraic interior"
        );
        assert_eq!(
            region
                .classify_point(&p(0, 3), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Outside),
            "{operation:?} retained algebraic overlap interior"
        );
        assert_eq!(
            region
                .classify_point(&p(0, 0), &CurveContext::STRICT)
                .unwrap()
                .into_value(),
            Classification::Decided(RegionPointLocation::Boundary),
            "{operation:?} retained algebraic boundary"
        );
        let transformed = region
            .transform_affine(
                &r(-2),
                &r(0),
                &r(0),
                &r(3),
                &r(7),
                &r(-1),
                &CurveContext::STRICT,
            )
            .unwrap_or_else(|error| panic!("{operation:?} affine transform: {error:?}"))
            .into_value();
        assert!(transformed.has_algebraic_fragments());
        for (point, expected) in [
            (p(7, 2), RegionPointLocation::Inside),
            (p(7, 8), RegionPointLocation::Outside),
            (p(7, -1), RegionPointLocation::Boundary),
        ] {
            assert_eq!(
                transformed
                    .classify_point(&point, &CurveContext::STRICT)
                    .unwrap()
                    .into_value(),
                Classification::Decided(expected),
                "{operation:?} transformed algebraic classification"
            );
        }
    }
}

#[cfg(feature = "predicates")]
fn equivalent_parabola_curves() -> Vec<(CurveFamily2, Curve2)> {
    let controls = [p(-2, 4), p(0, -4), p(2, 4)];
    let elevated_controls = [
        controls[0].clone(),
        Point2::new(q(-2, 3), q(-4, 3)),
        Point2::new(q(2, 3), q(-4, 3)),
        controls[2].clone(),
    ];
    vec![
        (
            CurveFamily2::QuadraticBezier,
            Curve2::from(QuadraticBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            )),
        ),
        (
            CurveFamily2::CubicBezier,
            Curve2::from(CubicBezier2::new(
                elevated_controls[0].clone(),
                elevated_controls[1].clone(),
                elevated_controls[2].clone(),
                elevated_controls[3].clone(),
            )),
        ),
        (
            CurveFamily2::RationalQuadraticBezier,
            Curve2::from(
                RationalQuadraticBezier2::try_new(
                    controls[0].clone(),
                    controls[1].clone(),
                    controls[2].clone(),
                    r(1),
                    r(1),
                    r(1),
                )
                .unwrap(),
            ),
        ),
        (
            CurveFamily2::RationalBezier,
            Curve2::from(RationalBezier2::try_new(controls.to_vec(), vec![r(1); 3]).unwrap()),
        ),
        (
            CurveFamily2::PolynomialBSpline,
            Curve2::try_polynomial_bspline(
                2,
                controls.to_vec(),
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            )
            .unwrap(),
        ),
        (
            CurveFamily2::Nurbs,
            Curve2::try_nurbs(
                2,
                controls.to_vec(),
                vec![r(1); 3],
                vec![r(0), r(0), r(0), r(1), r(1), r(1)],
            )
            .unwrap(),
        ),
    ]
}

#[test]
#[cfg(feature = "predicates")]
fn equivalent_top_level_families_complete_independent_region_booleans() {
    let cutter = rectangle(-3, 2, 3, 5);
    let policy = CurveContext::STRICT;
    for (family, curve) in equivalent_parabola_curves() {
        let source = CurvePath2::try_new(vec![
            curve,
            Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4)).unwrap()),
        ])
        .unwrap();
        let selections = source
            .boolean_selections(
                &cutter,
                CurveBoundaryInteriorSide2::Left,
                CurveBoundaryInteriorSide2::Left,
                &policy,
            )
            .unwrap()
            .into_value();
        let evidence = selections.result();
        assert!(
            evidence.is_complete(),
            "{family:?}: {:#?}",
            evidence.blockers()
        );
        assert_eq!(evidence.contacts().len(), 2, "{family:?}");

        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            let _region = selections
                .selection(operation)
                .region_view()
                .unwrap_or_else(|error| panic!("{family:?} {operation:?}: {error:?}"));
        }
    }
}
