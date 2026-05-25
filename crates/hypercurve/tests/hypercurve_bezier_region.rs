use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementFragment2,
    BezierArrangementGraph2, BezierParameter2, BezierParameterInterval, BezierParameterPolynomial,
    BezierRegion2, BezierRetainedBoundaryLoop2, BezierRetainedCurveEnvelope2,
    BezierRetainedEndpointEnvelope2, BezierRetainedRegion2, BezierRetainedRegionLoopRole,
    BezierSplitFragment2, Classification, CurvePolicy, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real, UncertaintyReason,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
    }
}

fn exact(value: Real) -> BezierParameter2 {
    decided(BezierParameter2::exact(value, &policy()).unwrap())
}

fn algebraic_midpoint_parameter() -> BezierAlgebraicParameter2 {
    let polynomial = decided(
        BezierParameterPolynomial::try_new_power_basis(vec![r(-1), r(2)], &policy()).unwrap(),
    );
    let interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), &policy()).unwrap());
    decided(BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy()).unwrap())
}

fn algebraic_image(curve: &QuadraticBezier2) -> BezierAlgebraicEndpointImage2 {
    BezierAlgebraicEndpointImage2::quadratic(curve, &algebraic_midpoint_parameter(), &policy())
        .unwrap()
}

fn line_midpoint_curve(start_x: i32, mid_x: i32, end_x: i32) -> QuadraticBezier2 {
    QuadraticBezier2::new(p(start_x, 0), p(mid_x, 0), p(end_x, 0))
}

fn materialized_line_fragment(
    source_curve_index: usize,
    start: Point2,
    midpoint: Point2,
    end: Point2,
) -> BezierArrangementFragment2 {
    BezierArrangementFragment2::new(
        source_curve_index,
        0,
        BezierSplitFragment2::Materialized {
            start: exact(r(0)),
            end: exact(r(1)),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                start, midpoint, end,
            )),
        },
    )
}

fn retained_line_loop(vertices: &[Point2]) -> BezierRetainedBoundaryLoop2 {
    let mut fragments = Vec::new();
    for edge in vertices.windows(2) {
        fragments.push(BezierSplitFragment2::Materialized {
            start: exact(r(0)),
            end: exact(r(1)),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                edge[0].clone(),
                edge[0].lerp(&edge[1], q(1, 2)),
                edge[1].clone(),
            )),
        });
    }
    let first = vertices.first().expect("test loop has vertices");
    let last = vertices.last().expect("test loop has vertices");
    fragments.push(BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            last.clone(),
            last.lerp(first, q(1, 2)),
            first.clone(),
        )),
    });
    BezierRetainedBoundaryLoop2::new(fragments)
}

#[test]
fn closed_polynomial_arrangement_materializes_retained_region_with_exact_area() {
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let upper_split = decided(
        upper
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );
    let lower_split = decided(
        lower
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );
    let graph = BezierArrangementGraph2::from_split_materializations(&[upper_split, lower_split]);
    let traversal = decided(graph.traverse_branch_free(&policy()));
    let region = decided(BezierRegion2::from_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(region.len(), 1);
    assert_eq!(region.boundary_loops()[0].len(), 4);
    assert_eq!(region.signed_area().unwrap(), Some(q(-32, 3)));
}

#[test]
fn open_arrangement_chain_does_not_materialize_region() {
    let first = QuadraticBezier2::new(p(0, 0), p(1, 1), p(2, 0));
    let second = QuadraticBezier2::new(p(2, 0), p(3, -1), p(4, 0));
    let first_split = decided(first.split_at_parameters(&[], &policy()).unwrap());
    let second_split = decided(second.split_at_parameters(&[], &policy()).unwrap());
    let graph = BezierArrangementGraph2::from_split_materializations(&[first_split, second_split]);
    let traversal = decided(graph.traverse_branch_free(&policy()));

    assert_eq!(
        BezierRegion2::from_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn conic_region_boundary_materializes_but_area_is_explicitly_unsupported() {
    let upper =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(2, 2), p(4, 0), q(1, 2)).unwrap();
    let lower = RationalQuadraticBezier2::try_unit_end_weights(p(4, 0), p(2, -2), p(0, 0), q(1, 2))
        .unwrap();
    let upper_split = decided(
        upper
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );
    let lower_split = decided(
        lower
            .split_at_parameters(&[exact(q(1, 2))], &policy())
            .unwrap(),
    );
    let graph = BezierArrangementGraph2::from_split_materializations(&[upper_split, lower_split]);
    let traversal = decided(graph.traverse_branch_free(&policy()));
    let region = decided(BezierRegion2::from_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(region.len(), 1);
    assert_eq!(region.boundary_loops()[0].len(), 4);
    assert_eq!(region.signed_area().unwrap(), None);
}

#[test]
fn resolved_linear_overlap_traversal_materializes_native_and_retained_regions() {
    let graph = BezierArrangementGraph2::new(vec![
        materialized_line_fragment(0, p(0, 0), p(2, 0), p(4, 0)),
        materialized_line_fragment(1, p(2, 0), p(3, 0), p(4, 0)),
        materialized_line_fragment(2, p(4, 0), p(4, 1), p(4, 2)),
        materialized_line_fragment(3, p(4, 2), p(2, 2), p(0, 2)),
        materialized_line_fragment(4, p(0, 2), p(0, 1), p(0, 0)),
    ]);

    assert_eq!(
        graph.traverse_retained_deduplicating_materialized_overlaps(&policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let traversal = decided(graph.traverse_retained_splitting_linear_overlaps(&policy()));
    assert_eq!(traversal.refinement().resolved_overlaps().len(), 1);

    let retained =
        decided(BezierRetainedRegion2::from_retained_linear_overlap_traversal(&traversal));
    assert_eq!(retained.len(), 1);
    assert_eq!(retained.boundary_loops()[0].len(), 5);
    assert!(!retained.has_algebraic_fragments());
    assert_eq!(retained.signed_area().unwrap(), Some(r(8)));

    let native = decided(BezierRegion2::from_retained_linear_overlap_traversal(
        &traversal,
    ));
    assert_eq!(native.len(), 1);
    assert_eq!(native.boundary_loops()[0].len(), 5);
    assert_eq!(native.signed_area().unwrap(), Some(r(8)));
}

#[test]
fn retained_line_image_role_report_assigns_nested_material_and_hole() {
    let outer = retained_line_loop(&[p(0, 0), p(6, 0), p(6, 6), p(0, 6)]);
    let same_orientation_inner = retained_line_loop(&[p(2, 2), p(4, 2), p(4, 4), p(2, 4)]);
    let retained = BezierRetainedRegion2::new(vec![outer, same_orientation_inner]);

    let report = decided(retained.line_image_role_report(&policy()).unwrap());

    assert_eq!(
        report.roles(),
        &[
            BezierRetainedRegionLoopRole::Material,
            BezierRetainedRegionLoopRole::Hole
        ]
    );
    assert_eq!(report.material_loop_indices(), vec![0]);
    assert_eq!(report.hole_loop_indices(), vec![1]);
    assert_eq!(
        report.to_region().filled_area(&policy()).unwrap(),
        Classification::Decided(Some(r(32)))
    );
}

#[test]
fn retained_line_image_role_report_rejects_nonlinear_line_image_loop() {
    let nonlinear_edge = BezierSplitFragment2::Materialized {
        start: exact(r(0)),
        end: exact(r(1)),
        curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            p(0, 0),
            p(1, 0),
            p(4, 0),
        )),
    };
    let retained = BezierRetainedRegion2::new(vec![BezierRetainedBoundaryLoop2::new(vec![
        nonlinear_edge,
        BezierSplitFragment2::Materialized {
            start: exact(r(0)),
            end: exact(r(1)),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                p(4, 0),
                p(4, 2),
                p(4, 4),
            )),
        },
        BezierSplitFragment2::Materialized {
            start: exact(r(0)),
            end: exact(r(1)),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                p(4, 4),
                p(2, 4),
                p(0, 4),
            )),
        },
        BezierSplitFragment2::Materialized {
            start: exact(r(0)),
            end: exact(r(1)),
            curve: hypercurve::BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                p(0, 4),
                p(0, 2),
                p(0, 0),
            )),
        },
    ])]);

    assert_eq!(
        retained.line_image_role_report(&policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn retained_curve_envelope_includes_native_bezier_interior_extrema() {
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let graph = BezierArrangementGraph2::from_split_materializations(&[
        decided(upper.split_at_parameters(&[], &policy()).unwrap()),
        decided(lower.split_at_parameters(&[], &policy()).unwrap()),
    ]);
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));
    let retained = decided(BezierRetainedRegion2::from_retained_arrangement_traversal(
        &graph, &traversal,
    ));

    let endpoint_envelope = decided(BezierRetainedEndpointEnvelope2::from_region(
        &retained,
        &policy(),
    ));
    assert_eq!(endpoint_envelope.envelope().min(), &p(0, 0));
    assert_eq!(endpoint_envelope.envelope().max(), &p(4, 0));

    let curve_envelope = decided(BezierRetainedCurveEnvelope2::from_region(
        &retained,
        &policy(),
    ));
    assert_eq!(curve_envelope.envelope().min(), &p(0, -2));
    assert_eq!(curve_envelope.envelope().max(), &p(4, 2));
    assert_eq!(curve_envelope.exact_fragment_count(), 2);
}

#[test]
fn retained_region_materializes_closed_algebraic_carrier_loop_without_area_sampling() {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_parameter());
    let p0_right = algebraic_image(&line_midpoint_curve(-1, 0, 1));
    let p1_right = algebraic_image(&line_midpoint_curve(0, 1, 2));
    let p1_left = algebraic_image(&line_midpoint_curve(2, 1, 0));
    let p0_left = algebraic_image(&line_midpoint_curve(1, 0, -1));
    let first = BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter.clone(),
        start_image: Some(p0_right),
        end_image: Some(p1_right),
    };
    let second = BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter,
        start_image: Some(p1_left),
        end_image: Some(p0_left),
    };
    let graph = BezierArrangementGraph2::new(vec![
        BezierArrangementFragment2::new(0, 0, first),
        BezierArrangementFragment2::new(1, 0, second),
    ]);
    let traversal = decided(graph.traverse_retained_with_tangent_order(&policy()));

    assert_eq!(
        BezierRegion2::from_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
    let retained = decided(BezierRetainedRegion2::from_retained_arrangement_traversal(
        &graph, &traversal,
    ));

    assert_eq!(retained.len(), 1);
    assert_eq!(retained.boundary_loops()[0].len(), 2);
    assert!(retained.has_algebraic_fragments());
    assert_eq!(retained.signed_area().unwrap(), None);
    let envelope = decided(BezierRetainedEndpointEnvelope2::from_region(
        &retained,
        &policy(),
    ));
    assert_eq!(envelope.envelope().min(), &p(0, 0));
    assert_eq!(envelope.envelope().max(), &p(1, 0));
    assert_eq!(envelope.algebraic_endpoint_count(), 4);
    assert_eq!(envelope.native_endpoint_count(), 0);
    assert!(envelope.has_algebraic_endpoints());
    assert_eq!(
        BezierRetainedCurveEnvelope2::from_region(&retained, &policy()),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn retained_region_rejects_unresolved_carriers_even_when_marked_closed() {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_parameter());
    let graph = BezierArrangementGraph2::new(vec![BezierArrangementFragment2::new(
        0,
        0,
        BezierSplitFragment2::Unresolved {
            start: parameter.clone(),
            end: parameter,
        },
    )]);
    let traversal = hypercurve::BezierArrangementTraversal2::new(vec![
        hypercurve::BezierArrangementChain2::new(vec![0], true),
    ]);

    assert_eq!(
        BezierRetainedRegion2::from_retained_arrangement_traversal(&graph, &traversal),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn retained_endpoint_envelope_rejects_incomplete_algebraic_endpoint_evidence() {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_parameter());
    let partial = BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter,
        start_image: Some(algebraic_image(&line_midpoint_curve(-1, 0, 1))),
        end_image: None,
    };
    let retained =
        BezierRetainedRegion2::new(vec![BezierRetainedBoundaryLoop2::new(vec![partial])]);

    assert_eq!(
        BezierRetainedEndpointEnvelope2::from_region(&retained, &policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

proptest! {
    #[test]
    fn symmetric_quadratic_lens_area_scales_exactly(
        height in 1_i32..=12,
    ) {
        let upper = QuadraticBezier2::new(p(0, 0), p(2, height), p(4, 0));
        let lower = QuadraticBezier2::new(p(4, 0), p(2, -height), p(0, 0));
        let graph = BezierArrangementGraph2::from_split_materializations(&[
            decided(upper.split_at_parameters(&[exact(q(1, 2))], &policy()).unwrap()),
            decided(lower.split_at_parameters(&[exact(q(1, 2))], &policy()).unwrap()),
        ]);
        let traversal = decided(graph.traverse_branch_free(&policy()));
        let region = decided(BezierRegion2::from_arrangement_traversal(&graph, &traversal));

        prop_assert_eq!(region.signed_area().unwrap(), Some(q(-8 * height, 3)));
    }
}
