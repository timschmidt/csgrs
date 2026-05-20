use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Axis2, BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2,
    BezierBooleanArrangementReadinessReport2, BezierBooleanAssemblyReadinessReport2,
    BezierBooleanBatchHandoffReport2, BezierBooleanConstructionReadinessReport2,
    BezierBooleanCubicFragmentReport2, BezierBooleanEmissionPlanReport2,
    BezierBooleanFragmentOwnershipLocation, BezierBooleanHandoffReport2,
    BezierBooleanLoopAssemblyPlanReport2, BezierBooleanLoopClosureReport2,
    BezierBooleanLoopContainmentFact2, BezierBooleanLoopContainmentFactReport2,
    BezierBooleanLoopGraphFactReport2, BezierBooleanLoopGraphFacts2,
    BezierBooleanLoopGraphTraversalReport2, BezierBooleanLoopGraphWalkReport2,
    BezierBooleanLoopNestingDepthFact2, BezierBooleanLoopNestingDepthFactReport2,
    BezierBooleanLoopNestingRoleReport2, BezierBooleanLoopRoleAssignmentReport2,
    BezierBooleanOutputLoopReport2, BezierBooleanOutputLoopRole,
    BezierBooleanOverlapResolutionReport2, BezierBooleanOwnershipClassificationReport2,
    BezierBooleanOwnershipFact2, BezierBooleanOwnershipFactReport2,
    BezierBooleanPathSchedulerReport2, BezierBooleanQuadraticFragmentReport2,
    BezierBooleanRationalQuadraticFragmentReport2, BezierBooleanRegionAssemblyReport2,
    BezierBooleanResultReport2, BezierBooleanSplitPlanReport2,
    BezierBooleanTraversalPreconditionReport2, BezierBooleanTraversalScheduleReport2,
    BezierBooleanUniformOwnershipFactReport2, BezierFlatteningOptions,
    BezierIntersectionRegionIsolationBudget, BezierMonotoneSpan, BezierPathRangeBatchReport2,
    BezierPathRangeOrderReport2, BooleanOp, CubicBezier2, CurvePolicy, LineSeg2, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, Real, certify_bezier_intersection_region_isolation,
    isolate_bezier_intersection_regions, isolate_bezier_intersection_regions_until_width,
    refine_bezier_intersection_regions, summarize_bezier_intersection_regions,
};

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn half() -> Real {
    (Real::one() / Real::from(2_i8)).unwrap()
}

fn ratio(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn span(start: Real, end: Real) -> BezierMonotoneSpan {
    BezierMonotoneSpan::new(start, end)
}

fn quadratic_through_point_at(
    parameter: Real,
    target: &Point2,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
) -> QuadraticBezier2 {
    let one_minus_t = Real::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t;
    let b1 = Real::from(2_i8) * &parameter * &one_minus_t;
    let b2 = &parameter * &parameter;
    let p0 = Point2::new(
        target.x() + &Real::from(first_offset.0),
        target.y() + &Real::from(first_offset.1),
    );
    let p1 = Point2::new(
        target.x() + &Real::from(second_offset.0),
        target.y() + &Real::from(second_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y());
    let p2_x = (numerator_x / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    let p2_y = (numerator_y / &b2)
        .expect("nonzero parameter gives nonzero quadratic Bernstein endpoint weight");
    QuadraticBezier2::new(p0, p1, Point2::new(p2_x, p2_y))
}

fn cubic_through_point_at(
    parameter: Real,
    target: &Point2,
    first_offset: (i32, i32),
    second_offset: (i32, i32),
    third_offset: (i32, i32),
) -> CubicBezier2 {
    let one_minus_t = Real::one() - &parameter;
    let b0 = &one_minus_t * &one_minus_t * &one_minus_t;
    let b1 = Real::from(3_i8) * &parameter * &one_minus_t * &one_minus_t;
    let b2 = Real::from(3_i8) * &parameter * &parameter * &one_minus_t;
    let b3 = &parameter * &parameter * &parameter;
    let p0 = Point2::new(
        target.x() + &Real::from(first_offset.0),
        target.y() + &Real::from(first_offset.1),
    );
    let p1 = Point2::new(
        target.x() + &Real::from(second_offset.0),
        target.y() + &Real::from(second_offset.1),
    );
    let p2 = Point2::new(
        target.x() + &Real::from(third_offset.0),
        target.y() + &Real::from(third_offset.1),
    );
    let numerator_x = target.x() - &(&b0 * p0.x()) - &(&b1 * p1.x()) - &(&b2 * p2.x());
    let numerator_y = target.y() - &(&b0 * p0.y()) - &(&b1 * p1.y()) - &(&b2 * p2.y());
    let p3_x = (numerator_x / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    let p3_y = (numerator_y / &b3)
        .expect("nonzero parameter gives nonzero cubic Bernstein endpoint weight");
    CubicBezier2::new(p0, p1, p2, Point2::new(p3_x, p3_y))
}

fn main() {
    let quadratic = QuadraticBezier2::new(p(0, 0), p(17, 23), p(41, -5));
    let cubic = CubicBezier2::new(p(-11, 7), p(3, 31), p(29, -37), p(53, 2));
    let rational = RationalQuadraticBezier2::try_unit_end_weights(
        p(0, 0),
        p(17, 23),
        p(41, -5),
        Real::from(2_i8),
    )
    .unwrap();

    bench_quadratic_facts(&quadratic);
    bench_cubic_facts(&cubic);
    bench_rational_quadratic_facts(&rational);
    bench_bezier_topology(&quadratic, &cubic);
    bench_bezier_flattening(&quadratic, &cubic);
}

fn bench_quadratic_facts(curve: &QuadraticBezier2) {
    let iterations = 100_000;
    let policy = CurvePolicy::certified();
    let half = half();
    let linear = QuadraticBezier2::new(p(0, 0), p(3, 0), p(6, 0));
    let start = Instant::now();
    let mut checksum = 0_u16;
    for _ in 0..iterations {
        let facts = black_box(curve).structural_facts();
        checksum ^= facts.derivative_known_zero_mask;
        checksum ^= facts.second_difference_known_nonzero_mask;
        checksum ^= format!("{:?}", black_box(curve).length_bounds()).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).refined_length_bounds(3)).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).refined_prefix_length_bounds(half.clone(), 3, &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).inverse_length_parameter_region(Real::one(), 8, 3, &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(&linear).inverse_length_parameter_region(Real::from(2_i8), 0, 0, &policy)
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).signed_area_contribution()).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).prefix_signed_area_contribution(half.clone(), &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            BezierAreaPrefixSums2::from_quadratics([black_box(curve)])
                .and_then(|table| table.range_contribution(0..1))
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).area_moments_contribution()).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).prefix_area_moments_contribution(half.clone(), &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            BezierAreaMomentPrefixSums2::from_quadratics([black_box(curve)])
                .and_then(|table| table.range_contribution(0..1))
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_line_image(&policy)).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_point_image(&policy)).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_source_report(&policy)).len() as u16;
        checksum ^= format!(
            "{:?}",
            hypercurve::BezierFitSourceBatchReport2::from_quadratics(
                [black_box(curve), black_box(&linear)],
                &policy
            )
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            hypercurve::BezierFitSourcePrefixSums2::from_quadratics(
                [black_box(curve), black_box(&linear)],
                &policy
            )
            .and_then(|table| table.range_report(0..2))
        )
        .len() as u16;
    }
    let elapsed = start.elapsed();
    println!("quadratic_bezier_facts: {iterations} iterations in {elapsed:?} checksum={checksum}");
}

fn bench_cubic_facts(curve: &CubicBezier2) {
    let iterations = 100_000;
    let policy = CurvePolicy::certified();
    let half = half();
    let linear = CubicBezier2::new(p(0, 0), p(3, 0), p(6, 0), p(9, 0));
    let start = Instant::now();
    let mut checksum = 0_u16;
    for _ in 0..iterations {
        let facts = black_box(curve).structural_facts();
        checksum ^= facts.derivative_known_nonzero_mask;
        checksum ^= u16::from(facts.curvature_known_nonzero_mask);
        checksum ^= format!("{:?}", black_box(curve).length_bounds()).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).refined_length_bounds(3)).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).refined_prefix_length_bounds(half.clone(), 3, &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).inverse_length_parameter_region(Real::one(), 8, 3, &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(&linear).inverse_length_parameter_region(Real::from(3_i8), 0, 0, &policy)
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).signed_area_contribution()).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).prefix_signed_area_contribution(half.clone(), &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            BezierAreaPrefixSums2::from_cubics([black_box(curve)])
                .and_then(|table| table.range_contribution(0..1))
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).area_moments_contribution()).len() as u16;
        checksum ^= format!(
            "{:?}",
            black_box(curve).prefix_area_moments_contribution(half.clone(), &policy)
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            BezierAreaMomentPrefixSums2::from_cubics([black_box(curve)])
                .and_then(|table| table.range_contribution(0..1))
        )
        .len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_line_image(&policy)).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_point_image(&policy)).len() as u16;
        checksum ^= format!("{:?}", black_box(curve).fit_source_report(&policy)).len() as u16;
        checksum ^= format!(
            "{:?}",
            hypercurve::BezierFitSourceBatchReport2::from_cubics(
                [black_box(curve), black_box(&linear)],
                &policy
            )
        )
        .len() as u16;
        checksum ^= format!(
            "{:?}",
            hypercurve::BezierFitSourcePrefixSums2::from_cubics(
                [black_box(curve), black_box(&linear)],
                &policy
            )
            .and_then(|table| table.range_report(0..2))
        )
        .len() as u16;
    }
    let elapsed = start.elapsed();
    println!("cubic_bezier_facts: {iterations} iterations in {elapsed:?} checksum={checksum}");
}

fn bench_rational_quadratic_facts(curve: &RationalQuadraticBezier2) {
    let iterations = 100_000;
    let policy = CurvePolicy::certified();
    let line = LineSeg2::try_new(p(-2, 0), p(60, 0)).unwrap();
    let shifted = RationalQuadraticBezier2::try_unit_end_weights(
        p(100, 100),
        p(117, 123),
        p(141, 95),
        Real::from(2_i8),
    )
    .unwrap();
    let crossing = RationalQuadraticBezier2::try_unit_end_weights(
        p(0, 8),
        p(17, -23),
        p(41, 8),
        Real::from(2_i8),
    )
    .unwrap();
    let rational_line = RationalQuadraticBezier2::try_unit_end_weights(
        p(0, 0),
        p(17, 0),
        p(41, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let rational_cross_line = RationalQuadraticBezier2::try_unit_end_weights(
        p(17, -20),
        p(17, 0),
        p(17, 20),
        Real::from(2_i8),
    )
    .unwrap();
    let collapsed_rational =
        RationalQuadraticBezier2::try_unit_end_weights(p(3, 4), p(3, 4), p(3, 4), Real::from(2_i8))
            .unwrap();
    let negative_rational_line = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(17, 0),
        p(41, 0),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let negative_rational_cross_line = RationalQuadraticBezier2::try_new(
        p(17, -20),
        p(17, 0),
        p(17, 20),
        Real::from(-1_i8),
        Real::from(-2_i8),
        Real::from(-1_i8),
    )
    .unwrap();
    let endpoint_arch =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(2, 3), p(4, 0), Real::from(2_i8))
            .unwrap();
    let endpoint_probe = CubicBezier2::new(p(2, 2), p(3, 5), p(5, 5), p(6, 4));
    let shared_endpoint_rational_first =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(1, 2), p(2, 0), Real::from(2_i8))
            .unwrap();
    let shared_endpoint_rational_second =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(1, 0), p(2, 8), Real::from(2_i8))
            .unwrap();
    let equal_weight_arch =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(20, 16), p(40, 0), Real::one())
            .unwrap();
    let equal_weight_crossing =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 8), p(20, -8), p(40, 8), Real::one())
            .unwrap();
    let equal_weight_polynomial_gap = QuadraticBezier2::new(p(0, 1), p(20, 17), p(40, 1));
    let equal_weight_polynomial_cubic_gap = CubicBezier2::new(
        p(0, 1),
        Point2::new(ratio(40, 3), ratio(35, 3)),
        Point2::new(ratio(80, 3), ratio(35, 3)),
        p(40, 1),
    );
    let non_equal_rational_polynomial_graph = RationalQuadraticBezier2::try_new(
        Point2::new(Real::zero(), Real::one()),
        Point2::new(ratio(1, 4), Real::one()),
        Point2::new(Real::one(), Real::one()),
        Real::one(),
        Real::from(2_i8),
        Real::from(3_i8),
    )
    .unwrap();
    let non_equal_polynomial_graph_baseline = QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(ratio(1, 2), Real::zero()),
        Point2::new(Real::one(), Real::zero()),
    );
    let non_equal_polynomial_graph_crossing = QuadraticBezier2::new(
        Point2::new(Real::zero(), Real::from(2_i8)),
        Point2::new(ratio(1, 2), Real::zero()),
        Point2::new(Real::one(), Real::from(2_i8)),
    );
    let non_equal_cubic_graph_baseline = CubicBezier2::new(
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(ratio(1, 3), Real::zero()),
        Point2::new(ratio(2, 3), Real::zero()),
        Point2::new(Real::one(), Real::zero()),
    );
    let non_equal_cubic_graph_crossing = CubicBezier2::new(
        Point2::new(Real::zero(), Real::from(2_i8)),
        Point2::new(ratio(1, 3), Real::zero()),
        Point2::new(ratio(2, 3), Real::zero()),
        Point2::new(Real::one(), Real::from(2_i8)),
    );
    let projective_scaled = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(17, 23),
        p(41, -5),
        Real::from(4_i8),
        Real::from(8_i8),
        Real::from(4_i8),
    )
    .unwrap();
    let projective_reversed = RationalQuadraticBezier2::try_new(
        p(41, -5),
        p(17, 23),
        p(0, 0),
        Real::from(4_i8),
        Real::from(8_i8),
        Real::from(4_i8),
    )
    .unwrap();
    let matching_weight_crossing = RationalQuadraticBezier2::try_new(
        Point2::new(Real::from(0_i8), Real::from(1_i8)),
        Point2::new(Real::from(17_i8), Real::from(23_i8) - ratio(1, 4)),
        Point2::new(Real::from(41_i8), Real::from(-7_i8)),
        Real::one(),
        Real::from(2_i8),
        Real::one(),
    )
    .unwrap();
    let matching_weight_graph_gap = RationalQuadraticBezier2::try_unit_end_weights(
        p(0, 12),
        p(17, 35),
        p(41, 5),
        Real::from(2_i8),
    )
    .unwrap();
    let dyadic_parameter = ratio(1, 512);
    let dyadic_target = match curve.point_at(dyadic_parameter.clone(), &policy) {
        hypercurve::Classification::Decided(point) => point,
        hypercurve::Classification::Uncertain(_) => p(0, 0),
    };
    let dyadic_polynomial =
        quadratic_through_point_at(dyadic_parameter, &dyadic_target, (5, 7), (-3, 11));
    let dyadic_cubic =
        cubic_through_point_at(ratio(1, 512), &dyadic_target, (5, 7), (-3, 11), (13, -5));
    let polynomial = QuadraticBezier2::new(p(100, 100), p(117, 123), p(141, 95));
    let polynomial_baseline = QuadraticBezier2::new(p(0, 0), p(17, 0), p(41, 0));
    let cubic = CubicBezier2::new(p(-120, -100), p(-117, -123), p(-141, -95), p(-155, -90));
    let start = Instant::now();
    let mut checksum = 0_u8;
    for _ in 0..iterations {
        let facts = black_box(curve).structural_facts();
        checksum ^= facts.weight_known_nonzero_mask;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_point_image(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).fit_exact_line_image(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).offset_preflight(&policy)).len() as u8;
        checksum ^= format!(
            "{:?}{:?}",
            black_box(curve).offset_left_staged(Real::one(), &policy),
            black_box(curve).offset_right_staged(Real::one(), &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&collapsed_rational).offset_left_staged(Real::one(), &policy)
        )
        .len() as u8;
        checksum ^= format!("{:?}", black_box(curve).conic_kind(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).monotone_spans(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).certified_bounds(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).relation_to_line(&line, &policy)).len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_line_with_contacts(&line, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).parameters_for_point(&p(17, 9), &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&shifted, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&rational_line).relation_to_rational_quadratic(&rational_cross_line, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&negative_rational_line)
                .relation_to_rational_quadratic(&negative_rational_cross_line, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_quadratic(&polynomial, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_quadratic(&polynomial_baseline, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&endpoint_arch).relation_to_cubic(&endpoint_probe, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&shared_endpoint_rational_first)
                .relation_to_rational_quadratic(&shared_endpoint_rational_second, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&equal_weight_arch)
                .relation_to_rational_quadratic(&equal_weight_crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&equal_weight_arch).graph_order_to_quadratic_over_axis(
                &equal_weight_polynomial_gap,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&equal_weight_polynomial_gap).graph_order_to_rational_quadratic_over_axis(
                &equal_weight_arch,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&equal_weight_arch).graph_order_to_cubic_over_axis(
                &equal_weight_polynomial_cubic_gap,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&equal_weight_polynomial_cubic_gap)
                .graph_order_to_rational_quadratic_over_axis(
                    &equal_weight_arch,
                    Axis2::X,
                    &policy,
                )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph).graph_order_to_quadratic_over_axis(
                &non_equal_polynomial_graph_baseline,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_polynomial_graph_baseline)
                .graph_order_to_rational_quadratic_over_axis(
                    &non_equal_rational_polynomial_graph,
                    Axis2::X,
                    &policy,
                )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph)
                .relation_to_quadratic(&non_equal_polynomial_graph_baseline, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph).graph_order_to_quadratic_over_axis(
                &non_equal_polynomial_graph_crossing,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph)
                .relation_to_quadratic(&non_equal_polynomial_graph_crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph).graph_order_to_cubic_over_axis(
                &non_equal_cubic_graph_baseline,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_cubic_graph_baseline).graph_order_to_rational_quadratic_over_axis(
                &non_equal_rational_polynomial_graph,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph)
                .relation_to_cubic(&non_equal_cubic_graph_baseline, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph).graph_order_to_cubic_over_axis(
                &non_equal_cubic_graph_crossing,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(&non_equal_rational_polynomial_graph)
                .relation_to_cubic(&non_equal_cubic_graph_crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&projective_scaled, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&projective_reversed, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&matching_weight_crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&matching_weight_graph_gap, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).graph_order_to_rational_quadratic_over_axis(
                &matching_weight_graph_gap,
                Axis2::X,
                &policy,
            )
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_quadratic(&dyadic_polynomial, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_cubic(&dyadic_cubic, &policy)
        )
        .len() as u8;
        checksum ^=
            format!("{:?}", black_box(curve).relation_to_cubic(&cubic, &policy)).len() as u8;
    }
    let elapsed = start.elapsed();
    println!(
        "rational_quadratic_bezier_facts: {iterations} iterations in {elapsed:?} checksum={checksum}"
    );
}

fn bench_bezier_topology(quadratic: &QuadraticBezier2, cubic: &CubicBezier2) {
    let iterations = 25_000;
    let policy = CurvePolicy::certified();
    let line = LineSeg2::try_new(p(-2, 0), p(60, 0)).unwrap();
    let shifted_cubic = CubicBezier2::new(p(100, 100), p(104, 130), p(140, 90), p(155, 105));
    let crossing_quadratic = QuadraticBezier2::new(p(0, 8), p(17, -23), p(41, 8));
    let line_image_quadratic = QuadraticBezier2::new(p(0, 0), p(20, 0), p(41, 0));
    let line_image_cubic = CubicBezier2::new(p(20, -20), p(20, -10), p(20, 10), p(20, 20));
    let isolated_line_root_cubic = CubicBezier2::new(p(0, -1), p(20, 1), p(40, 1), p(60, 1));
    let point_image_quadratic = QuadraticBezier2::new(p(20, 8), p(20, 8), p(20, 8));
    let tangent_line_image = QuadraticBezier2::new(p(0, 8), p(20, 8), p(40, 8));
    let tangent_arch = QuadraticBezier2::new(p(0, 0), p(20, 16), p(40, 0));
    let endpoint_probe = CubicBezier2::new(p(20, 8), p(23, 20), p(50, 20), p(52, 19));
    let shared_endpoint_midpoint_first = QuadraticBezier2::new(p(0, 0), p(30, 60), p(60, 0));
    let shared_endpoint_midpoint_second = QuadraticBezier2::new(p(0, 0), p(30, 0), p(60, 120));
    let raised_arch = QuadraticBezier2::new(p(0, 4), p(20, 20), p(40, 4));
    let degree_normalized_arch = QuadraticBezier2::new(p(0, 0), p(30, 60), p(60, 0));
    let degree_elevated_arch = CubicBezier2::new(p(0, 0), p(20, 40), p(40, 40), p(60, 0));
    let degree_elevated_raised_arch = CubicBezier2::new(p(0, 4), p(20, 44), p(40, 44), p(60, 4));
    let degree_elevated_crossing_arch =
        CubicBezier2::new(p(0, -10), p(20, 60), p(40, 60), p(60, -10));
    let midpoint_hit_cubic = CubicBezier2::new(p(0, 120), p(20, 0), p(40, 0), p(60, 120));
    let quarter_hit_quadratic = QuadraticBezier2::new(p(0, 0), p(30, 620), p(60, 0));
    let quarter_hit_cubic = CubicBezier2::new(p(0, 480), p(20, 0), p(40, 0), p(60, 1920));
    let thirty_second_hit_quadratic = QuadraticBezier2::new(p(0, 0), p(30, 150), p(60, 0));
    let thirty_second_hit_cubic = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -310));
    let sixty_fourth_hit_quadratic = QuadraticBezier2::new(p(0, 0), p(30, 310), p(60, 0));
    let sixty_fourth_hit_cubic = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -630));
    let one_hundred_twenty_eighth_hit_quadratic =
        QuadraticBezier2::new(p(0, 0), p(30, 630), p(60, 0));
    let one_hundred_twenty_eighth_hit_cubic =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -1270));
    let two_hundred_fifty_sixth_hit_quadratic =
        QuadraticBezier2::new(p(0, 0), p(30, 1270), p(60, 0));
    let two_hundred_fifty_sixth_hit_cubic =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -2550));
    let five_hundred_twelfth_hit_quadratic = QuadraticBezier2::new(p(0, 0), p(30, 2550), p(60, 0));
    let five_hundred_twelfth_hit_cubic =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -5110));
    let non_dyadic_graph_quadratic = QuadraticBezier2::new(p(0, 0), p(30, 0), p(60, 0));
    let non_dyadic_graph_cubic = CubicBezier2::new(p(0, 10), p(20, 0), p(40, -10), p(60, -10));
    let non_dyadic_quadratic_root_first = QuadraticBezier2::new(p(0, 0), p(30, 20), p(60, 0));
    let non_dyadic_quadratic_root_second = QuadraticBezier2::new(p(0, -10), p(30, 25), p(60, 20));
    let non_graph_cubic_base = [p(0, 0), p(30, 70), p(60, -20), p(90, 30)];
    let non_graph_deep_dyadic_difference = [
        ratio(-1, 1024),
        ratio(1021, 3072),
        ratio(2045, 3072),
        ratio(1023, 1024),
    ];
    let non_graph_deep_dyadic_first = CubicBezier2::new(
        non_graph_cubic_base[0].clone(),
        non_graph_cubic_base[1].clone(),
        non_graph_cubic_base[2].clone(),
        non_graph_cubic_base[3].clone(),
    );
    let non_graph_deep_dyadic_second = CubicBezier2::new(
        Point2::new(
            non_graph_cubic_base[0].x() - &non_graph_deep_dyadic_difference[0],
            non_graph_cubic_base[0].y() - &non_graph_deep_dyadic_difference[0],
        ),
        Point2::new(
            non_graph_cubic_base[1].x() - &non_graph_deep_dyadic_difference[1],
            non_graph_cubic_base[1].y() - &non_graph_deep_dyadic_difference[1],
        ),
        Point2::new(
            non_graph_cubic_base[2].x() - &non_graph_deep_dyadic_difference[2],
            non_graph_cubic_base[2].y() - &non_graph_deep_dyadic_difference[2],
        ),
        Point2::new(
            non_graph_cubic_base[3].x() - &non_graph_deep_dyadic_difference[3],
            non_graph_cubic_base[3].y() - &non_graph_deep_dyadic_difference[3],
        ),
    );
    let non_graph_irreducible_difference =
        [Real::from(-1_i8), Real::one(), Real::one(), Real::one()];
    let non_graph_irreducible_second = CubicBezier2::new(
        Point2::new(
            non_graph_cubic_base[0].x() - &non_graph_irreducible_difference[0],
            non_graph_cubic_base[0].y() - &non_graph_irreducible_difference[0],
        ),
        Point2::new(
            non_graph_cubic_base[1].x() - &non_graph_irreducible_difference[1],
            non_graph_cubic_base[1].y() - &non_graph_irreducible_difference[1],
        ),
        Point2::new(
            non_graph_cubic_base[2].x() - &non_graph_irreducible_difference[2],
            non_graph_cubic_base[2].y() - &non_graph_irreducible_difference[2],
        ),
        Point2::new(
            non_graph_cubic_base[3].x() - &non_graph_irreducible_difference[3],
            non_graph_cubic_base[3].y() - &non_graph_irreducible_difference[3],
        ),
    );
    let cubic_quarter_hit = CubicBezier2::new(p(0, 40), p(20, 0), p(40, 0), p(60, 360));
    let cubic_eighth_hit = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, 3290));
    let cubic_sixteenth_hit = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -4950));
    let cubic_thirty_second_hit = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -178870));
    let cubic_sixty_fourth_hit = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -2016630));
    let cubic_one_hundred_twenty_eighth_hit =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -18533110));
    let cubic_two_hundred_fifty_sixth_hit =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -157980150));
    let cubic_five_hundred_twelfth_hit =
        CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, -1302932470));
    let quarter = (Real::one() / Real::from(4_i8)).unwrap();
    let cubic_endpoint_probe = QuadraticBezier2::new(
        p(200, 200),
        p(180, 220),
        degree_elevated_arch.point_at(quarter),
    );
    let retained_regions = [
        hypercurve::BezierCurveIntersectionRegion::new(span(half(), half()), span(half(), half())),
        hypercurve::BezierCurveIntersectionRegion::new(
            span(ratio(1, 4), ratio(1, 2)),
            span(ratio(1, 4), ratio(1, 2)),
        ),
        hypercurve::BezierCurveIntersectionRegion::new(
            span(ratio(1, 4), ratio(1, 2)),
            span(ratio(1, 8), ratio(3, 8)),
        ),
    ];
    let boolean_rational_fragment_source = RationalQuadraticBezier2::try_unit_end_weights(
        p(0, 0),
        p(20, 16),
        p(40, 0),
        Real::from(2_i8),
    )
    .unwrap();
    let boolean_overlap_first = LineSeg2::try_new(p(0, 0), p(40, 0)).unwrap();
    let boolean_overlap_second = LineSeg2::try_new(p(20, 0), p(60, 0)).unwrap();
    let boolean_overlap_handoff = BezierBooleanHandoffReport2::from_relation(
        &hypercurve::BezierCurveRelation::LineSegmentIntersection {
            intersection: boolean_overlap_first
                .intersect_line(&boolean_overlap_second, &policy)
                .unwrap(),
        },
    );
    let start = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let y_roots = black_box(quadratic).axis_monotone_parameters(Axis2::Y, &policy);
        let spans = black_box(quadratic).monotone_spans(&policy);
        let bounds = black_box(quadratic).certified_bounds(&policy);
        let line_relation = black_box(quadratic).relation_to_line(&line, &policy);
        let line_contact_relation =
            black_box(quadratic).relation_to_line_with_contacts(&line, &policy);
        let point_parameters = black_box(quadratic).parameters_for_point(&p(17, 9), &policy);
        let cubic_line_relation = black_box(cubic).relation_to_line(&line, &policy);
        let cubic_line_contact_relation =
            black_box(cubic).relation_to_line_with_contacts(&line, &policy);
        let mixed_relation = black_box(quadratic).relation_to_cubic(&shifted_cubic, &policy);
        let region_relation =
            black_box(quadratic).relation_to_quadratic(&crossing_quadratic, &policy);
        let line_image_relation =
            black_box(&line_image_quadratic).relation_to_cubic(&line_image_cubic, &policy);
        let line_image_isolated_relation =
            black_box(&line_image_quadratic).relation_to_cubic(&isolated_line_root_cubic, &policy);
        let point_image_relation =
            black_box(&point_image_quadratic).relation_to_quadratic(&tangent_arch, &policy);
        let line_image_curve_relation =
            black_box(&tangent_line_image).relation_to_quadratic(&tangent_arch, &policy);
        let endpoint_relation =
            black_box(&endpoint_probe).relation_to_quadratic(&tangent_arch, &policy);
        let shared_endpoint_midpoint_relation = black_box(&shared_endpoint_midpoint_first)
            .relation_to_quadratic(&shared_endpoint_midpoint_second, &policy);
        let same_axis_no_hit_relation =
            black_box(&tangent_arch).relation_to_quadratic(&raised_arch, &policy);
        let degree_normalized_no_hit_relation = black_box(&degree_normalized_arch)
            .relation_to_cubic(&degree_elevated_raised_arch, &policy);
        let degree_normalized_graph_order = black_box(&degree_normalized_arch)
            .graph_order_to_cubic_over_axis(&degree_elevated_raised_arch, Axis2::X, &policy);
        let degree_normalized_crossing_graph_order = black_box(&degree_normalized_arch)
            .graph_order_to_cubic_over_axis(&degree_elevated_crossing_arch, Axis2::X, &policy);
        let degree_normalized_graph_contact_order = black_box(&degree_normalized_arch)
            .graph_contact_order_to_cubic_over_axis(
                &degree_elevated_crossing_arch,
                Axis2::X,
                &policy,
            );
        let degree_elevated_identity_relation =
            black_box(&degree_normalized_arch).relation_to_cubic(&degree_elevated_arch, &policy);
        let mixed_degree_midpoint_relation =
            black_box(&degree_normalized_arch).relation_to_cubic(&midpoint_hit_cubic, &policy);
        let mixed_degree_quarter_relation =
            black_box(&quarter_hit_quadratic).relation_to_cubic(&quarter_hit_cubic, &policy);
        let mixed_degree_thirty_second_relation = black_box(&thirty_second_hit_quadratic)
            .relation_to_cubic(&thirty_second_hit_cubic, &policy);
        let mixed_degree_sixty_fourth_relation = black_box(&sixty_fourth_hit_quadratic)
            .relation_to_cubic(&sixty_fourth_hit_cubic, &policy);
        let mixed_degree_one_hundred_twenty_eighth_relation =
            black_box(&one_hundred_twenty_eighth_hit_quadratic)
                .relation_to_cubic(&one_hundred_twenty_eighth_hit_cubic, &policy);
        let mixed_degree_two_hundred_fifty_sixth_relation =
            black_box(&two_hundred_fifty_sixth_hit_quadratic)
                .relation_to_cubic(&two_hundred_fifty_sixth_hit_cubic, &policy);
        let mixed_degree_five_hundred_twelfth_relation =
            black_box(&five_hundred_twelfth_hit_quadratic)
                .relation_to_cubic(&five_hundred_twelfth_hit_cubic, &policy);
        let mixed_degree_non_dyadic_graph_relation = black_box(&non_dyadic_graph_quadratic)
            .relation_to_cubic(&non_dyadic_graph_cubic, &policy);
        let non_dyadic_quadratic_root_relation = black_box(&non_dyadic_quadratic_root_first)
            .relation_to_quadratic(&non_dyadic_quadratic_root_second, &policy);
        let non_graph_deep_dyadic_relation = black_box(&non_graph_deep_dyadic_first)
            .relation_to_cubic(&non_graph_deep_dyadic_second, &policy);
        let non_graph_irreducible_relation = black_box(&non_graph_deep_dyadic_first)
            .relation_to_cubic(&non_graph_irreducible_second, &policy);
        let cubic_quarter_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_quarter_hit, &policy);
        let cubic_eighth_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_eighth_hit, &policy);
        let cubic_sixteenth_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_sixteenth_hit, &policy);
        let cubic_thirty_second_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_thirty_second_hit, &policy);
        let cubic_sixty_fourth_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_sixty_fourth_hit, &policy);
        let cubic_one_hundred_twenty_eighth_relation = black_box(&degree_elevated_arch)
            .relation_to_cubic(&cubic_one_hundred_twenty_eighth_hit, &policy);
        let cubic_two_hundred_fifty_sixth_relation = black_box(&degree_elevated_arch)
            .relation_to_cubic(&cubic_two_hundred_fifty_sixth_hit, &policy);
        let cubic_five_hundred_twelfth_relation = black_box(&degree_elevated_arch)
            .relation_to_cubic(&cubic_five_hundred_twelfth_hit, &policy);
        let cubic_endpoint_relation =
            black_box(&cubic_endpoint_probe).relation_to_cubic(&degree_elevated_arch, &policy);
        let inflections = black_box(cubic).inflection_classification(&policy);
        let quadratic_offset_preflight = black_box(quadratic).offset_preflight(&policy);
        let cubic_offset_preflight = black_box(cubic).offset_preflight(&policy);
        let quadratic_staged_offset = black_box(quadratic).offset_left_staged(Real::one(), &policy);
        let cubic_staged_offset = black_box(cubic).offset_left_staged(Real::one(), &policy);
        let quadratic_staged_right_offset =
            black_box(quadratic).offset_right_staged(Real::one(), &policy);
        let cubic_staged_right_offset = black_box(cubic).offset_right_staged(Real::one(), &policy);
        let quadratic_offset_adapter_report =
            quadratic_staged_offset
                .as_ref()
                .ok()
                .and_then(|classification| match classification {
                    hypercurve::Classification::Decided(candidate) => {
                        Some(candidate.adapter_report())
                    }
                    hypercurve::Classification::Uncertain(_) => None,
                });
        let region_summary = summarize_bezier_intersection_regions(black_box(&retained_regions));
        let region_refinements = refine_bezier_intersection_regions(black_box(&retained_regions));
        let region_isolation = isolate_bezier_intersection_regions(
            black_box(&retained_regions),
            BezierIntersectionRegionIsolationBudget {
                max_steps: 16,
                max_depth: 2,
                max_terminal_regions: 32,
            },
        );
        let targeted_region_isolation = isolate_bezier_intersection_regions_until_width(
            black_box(&retained_regions),
            BezierIntersectionRegionIsolationBudget {
                max_steps: 32,
                max_depth: 4,
                max_terminal_regions: 64,
            },
            ratio(1, 16),
        );
        let region_isolation_certificate =
            certify_bezier_intersection_region_isolation(&targeted_region_isolation);
        let boolean_handoff_from_line_image =
            BezierBooleanHandoffReport2::from_classified_relation(&line_image_relation);
        let boolean_handoff_from_regions =
            BezierBooleanHandoffReport2::from_classified_relation(&region_relation);
        let boolean_handoff_from_certificate =
            BezierBooleanHandoffReport2::from_isolation_certificate(&region_isolation_certificate);
        let boolean_handoff_batch = BezierBooleanBatchHandoffReport2::from_handoff_reports(&[
            boolean_handoff_from_line_image.clone(),
            boolean_handoff_from_regions.clone(),
            boolean_handoff_from_certificate.clone(),
        ]);
        let boolean_overlap_resolution =
            BezierBooleanOverlapResolutionReport2::from_handoff_reports(
                &[boolean_overlap_handoff.clone()],
                &policy,
            );
        let path_order_from_graph = BezierPathRangeOrderReport2::from_classified_graph_order(
            &degree_normalized_graph_order,
        );
        let path_order_from_contact =
            BezierPathRangeOrderReport2::from_classified_graph_contact_order(
                &degree_normalized_graph_contact_order,
            );
        let path_range_batch = BezierPathRangeBatchReport2::from_range_reports(&[
            path_order_from_graph.clone(),
            path_order_from_contact.clone(),
        ]);
        let boolean_path_scheduler = BezierBooleanPathSchedulerReport2::from_batches(
            boolean_handoff_batch.clone(),
            path_range_batch.clone(),
        );
        let boolean_split_plan =
            BezierBooleanSplitPlanReport2::from_scheduler(&boolean_path_scheduler);
        let boolean_split_plan_audit = boolean_split_plan.audit(&policy);
        let boolean_split_insertion_report = boolean_split_plan.insertion_report(&policy);
        let boolean_construction_readiness =
            BezierBooleanConstructionReadinessReport2::from_scheduler(
                boolean_path_scheduler.clone(),
                &policy,
            );
        let boolean_quadratic_fragments = match boolean_construction_readiness.clone() {
            hypercurve::Classification::Decided(readiness) => {
                BezierBooleanQuadraticFragmentReport2::from_first_curve_readiness(
                    quadratic, &readiness, &policy,
                )
            }
            hypercurve::Classification::Uncertain(reason) => {
                hypercurve::Classification::Uncertain(reason)
            }
        };
        let boolean_cubic_fragments = match boolean_construction_readiness.clone() {
            hypercurve::Classification::Decided(readiness) => {
                BezierBooleanCubicFragmentReport2::from_second_curve_readiness(
                    cubic, &readiness, &policy,
                )
            }
            hypercurve::Classification::Uncertain(reason) => {
                hypercurve::Classification::Uncertain(reason)
            }
        };
        let boolean_rational_quadratic_fragments = match boolean_construction_readiness.clone() {
            hypercurve::Classification::Decided(readiness) => {
                BezierBooleanRationalQuadraticFragmentReport2::from_first_curve_readiness(
                    &boolean_rational_fragment_source,
                    &readiness,
                    &policy,
                )
            }
            hypercurve::Classification::Uncertain(reason) => {
                hypercurve::Classification::Uncertain(reason)
            }
        };
        let boolean_arrangement_readiness = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => format!(
                "{:?}",
                BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps
                )
            ),
            other => format!("{other:?}"),
        };
        let boolean_traversal_preconditions = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                format!(
                    "{:?}",
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_traversal_schedule = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                format!(
                    "{:?}",
                    BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                        &preconditions,
                        first,
                        second
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_ownership_classification = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                format!(
                    "{:?}",
                    BezierBooleanOwnershipClassificationReport2::from_schedule(
                        &schedule,
                        BooleanOp::Union,
                        &ownerships
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_ownership_facts = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let facts = schedule
                    .steps
                    .iter()
                    .map(|step| BezierBooleanOwnershipFact2 {
                        step: step.clone(),
                        opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanOwnershipFactReport2::from_schedule_facts(&schedule, &facts)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_uniform_ownership_facts = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                format!(
                    "{:?}",
                    BezierBooleanUniformOwnershipFactReport2::from_schedule_locations(
                        &schedule,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_emission_plan = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                format!(
                    "{:?}",
                    BezierBooleanEmissionPlanReport2::from_ownership(&ownership)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_assembly_readiness = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                format!(
                    "{:?}",
                    BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                        &emission, first, second
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_assembly_plan = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                format!(
                    "{:?}",
                    BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                        &assembly, &emission
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_graph_traversal = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                format!(
                    "{:?}",
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
                        &plan,
                        0,
                        overlaps.resolved_events.len()
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_graph_facts = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let facts = BezierBooleanLoopGraphFacts2 {
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: 0,
                    resolved_overlap_count: 0,
                };
                format!(
                    "{:?}",
                    BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &facts)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_graph_walk = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                format!(
                    "{:?}",
                    BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_graph_walk_closure = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk =
                    BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
                format!(
                    "{:?}",
                    BezierBooleanLoopClosureReport2::from_quadratic_graph_walk(
                        &walk, &plan, first, second
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_closure = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                format!(
                    "{:?}",
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_output_loops = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                format!(
                    "{:?}",
                    BezierBooleanOutputLoopReport2::from_loop_closure(&closure)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_graph_walk_output_loops = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                format!(
                    "{:?}",
                    BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                        &walk, &plan, first, second
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_nesting_roles = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let depths = vec![0_usize; output.loops.len()];
                format!(
                    "{:?}",
                    BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(&output, &depths)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_nesting_depth_facts = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let facts = (0..output.loops.len())
                    .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                        loop_index,
                        nesting_depth: 0,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(
                        &output, &facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_containment_facts = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let containment_facts = if output.loops.len() > 1 {
                    vec![BezierBooleanLoopContainmentFact2 {
                        container_loop_index: 0,
                        contained_loop_index: 1,
                    }]
                } else {
                    Vec::new()
                };
                format!(
                    "{:?}",
                    BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
                        &output,
                        &containment_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_loop_roles = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let roles = vec![BezierBooleanOutputLoopRole::Material; output.loops.len()];
                format!(
                    "{:?}",
                    BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &roles)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_region_assembly = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let roles = vec![BezierBooleanOutputLoopRole::Material; output.loops.len()];
                let assigned =
                    BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &roles);
                format!(
                    "{:?}",
                    BezierBooleanRegionAssemblyReport2::from_role_assignment(&assigned)
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_graph_walk_region_assembly = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                let output = BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                    &walk, &plan, first, second,
                );
                let depth_facts = (0..output.loops.len())
                    .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                        loop_index,
                        nesting_depth: 0,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanRegionAssemblyReport2::from_quadratic_graph_walk_depth_facts(
                        &walk,
                        &plan,
                        first,
                        second,
                        &depth_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_graph_walk_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                let output = BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                    &walk, &plan, first, second,
                );
                let depth_facts = (0..output.loops.len())
                    .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                        loop_index,
                        nesting_depth: 0,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_graph_walk_depth_facts(
                        &walk,
                        &plan,
                        first,
                        second,
                        &depth_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_schedule_graph_walk_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownership_facts = schedule
                    .steps
                    .iter()
                    .map(|step| BezierBooleanOwnershipFact2 {
                        step: step.clone(),
                        opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                    })
                    .collect::<Vec<_>>();
                let ownership = BezierBooleanOwnershipFactReport2::from_schedule_facts(
                    &schedule,
                    &ownership_facts,
                )
                .classify(&schedule, BooleanOp::Union);
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                let output = BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                    &walk, &plan, first, second,
                );
                let depth_facts = (0..output.loops.len())
                    .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                        loop_index,
                        nesting_depth: 0,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_schedule_graph_walk_depth_facts(
                        &schedule,
                        BooleanOp::Union,
                        &ownership_facts,
                        first,
                        second,
                        0,
                        0,
                        &walk_indices,
                        &depth_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_schedule_graph_fact_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownership_facts = schedule
                    .steps
                    .iter()
                    .map(|step| BezierBooleanOwnershipFact2 {
                        step: step.clone(),
                        opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                    })
                    .collect::<Vec<_>>();
                let ownership = BezierBooleanOwnershipFactReport2::from_schedule_facts(
                    &schedule,
                    &ownership_facts,
                )
                .classify(&schedule, BooleanOp::Union);
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                let output = BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                    &walk, &plan, first, second,
                );
                let graph_facts = BezierBooleanLoopGraphFacts2 {
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: 0,
                    resolved_overlap_count: 0,
                };
                let depth_facts = (0..output.loops.len())
                    .map(|loop_index| BezierBooleanLoopNestingDepthFact2 {
                        loop_index,
                        nesting_depth: 0,
                    })
                    .collect::<Vec<_>>();
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_schedule_graph_fact_walk_depth_facts(
                        &schedule,
                        BooleanOp::Union,
                        &ownership_facts,
                        first,
                        second,
                        &graph_facts,
                        &walk_indices,
                        &depth_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_schedule_graph_fact_containment_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownership_facts = schedule
                    .steps
                    .iter()
                    .map(|step| BezierBooleanOwnershipFact2 {
                        step: step.clone(),
                        opposite_location: BezierBooleanFragmentOwnershipLocation::Outside,
                    })
                    .collect::<Vec<_>>();
                let ownership = BezierBooleanOwnershipFactReport2::from_schedule_facts(
                    &schedule,
                    &ownership_facts,
                )
                .classify(&schedule, BooleanOp::Union);
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let traversal =
                    BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(&plan, 0, 0);
                let walk_indices = (0..plan.emitted_steps.len()).collect::<Vec<_>>();
                let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                    &traversal,
                    &plan,
                    &walk_indices,
                );
                let output = BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(
                    &walk, &plan, first, second,
                );
                let containment_facts = if output.loops.len() > 1 {
                    vec![BezierBooleanLoopContainmentFact2 {
                        container_loop_index: 0,
                        contained_loop_index: 1,
                    }]
                } else {
                    Vec::new()
                };
                let graph_facts = BezierBooleanLoopGraphFacts2 {
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: 0,
                    resolved_overlap_count: 0,
                };
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_schedule_graph_fact_walk_containment_facts(
                        &schedule,
                        BooleanOp::Union,
                        &ownership_facts,
                        first,
                        second,
                        &graph_facts,
                        &walk_indices,
                        &containment_facts
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_uniform_identity_containment_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let graph_facts = BezierBooleanLoopGraphFacts2 {
                    emitted_step_count: schedule.steps.len(),
                    branch_vertex_count: 0,
                    resolved_overlap_count: 0,
                };
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_schedule_uniform_graph_fact_identity_containment_facts(
                        &schedule,
                        BooleanOp::Union,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                        first,
                        second,
                        &graph_facts,
                        &[]
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_uniform_linear_identity_containment_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_quadratic_schedule_uniform_linear_identity_containment_facts(
                        &schedule,
                        BooleanOp::Union,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                        BezierBooleanFragmentOwnershipLocation::Outside,
                        first,
                        second,
                        &[]
                    )
                )
            }
            other => format!("{other:?}"),
        };
        let boolean_result = match (
            &boolean_quadratic_fragments,
            &boolean_quadratic_fragments,
            &boolean_overlap_resolution,
        ) {
            (
                hypercurve::Classification::Decided(first),
                hypercurve::Classification::Decided(second),
                hypercurve::Classification::Decided(overlaps),
            ) => {
                let readiness = BezierBooleanArrangementReadinessReport2::from_quadratic_fragments(
                    first, second, overlaps,
                );
                let preconditions =
                    BezierBooleanTraversalPreconditionReport2::from_quadratic_fragments(
                        &readiness, first, second,
                    );
                let schedule = BezierBooleanTraversalScheduleReport2::from_quadratic_fragments(
                    &preconditions,
                    first,
                    second,
                );
                let ownerships =
                    vec![BezierBooleanFragmentOwnershipLocation::Outside; schedule.steps.len()];
                let ownership = BezierBooleanOwnershipClassificationReport2::from_schedule(
                    &schedule,
                    BooleanOp::Union,
                    &ownerships,
                );
                let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
                let assembly = BezierBooleanAssemblyReadinessReport2::from_quadratic_fragments(
                    &emission, first, second,
                );
                let plan = BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(
                    &assembly, &emission,
                );
                let closure =
                    BezierBooleanLoopClosureReport2::from_quadratic_fragments(&plan, first, second);
                let output = BezierBooleanOutputLoopReport2::from_loop_closure(&closure);
                let roles = vec![BezierBooleanOutputLoopRole::Material; output.loops.len()];
                let assigned =
                    BezierBooleanLoopRoleAssignmentReport2::from_output_loops(&output, &roles);
                let region = BezierBooleanRegionAssemblyReport2::from_role_assignment(&assigned);
                format!(
                    "{:?}",
                    BezierBooleanResultReport2::from_region_assembly(&region)
                )
            }
            other => format!("{other:?}"),
        };

        checksum ^= format!(
            "{y_roots:?}{spans:?}{bounds:?}{line_relation:?}{line_contact_relation:?}{point_parameters:?}{cubic_line_relation:?}{cubic_line_contact_relation:?}{mixed_relation:?}{region_relation:?}{line_image_relation:?}{line_image_isolated_relation:?}{point_image_relation:?}{line_image_curve_relation:?}{endpoint_relation:?}{shared_endpoint_midpoint_relation:?}{same_axis_no_hit_relation:?}{degree_normalized_no_hit_relation:?}{degree_normalized_graph_order:?}{degree_normalized_crossing_graph_order:?}{degree_normalized_graph_contact_order:?}{degree_elevated_identity_relation:?}{mixed_degree_midpoint_relation:?}{mixed_degree_quarter_relation:?}{mixed_degree_thirty_second_relation:?}{mixed_degree_sixty_fourth_relation:?}{mixed_degree_one_hundred_twenty_eighth_relation:?}{mixed_degree_two_hundred_fifty_sixth_relation:?}{mixed_degree_five_hundred_twelfth_relation:?}{mixed_degree_non_dyadic_graph_relation:?}{non_dyadic_quadratic_root_relation:?}{non_graph_deep_dyadic_relation:?}{non_graph_irreducible_relation:?}{cubic_quarter_relation:?}{cubic_eighth_relation:?}{cubic_sixteenth_relation:?}{cubic_thirty_second_relation:?}{cubic_sixty_fourth_relation:?}{cubic_one_hundred_twenty_eighth_relation:?}{cubic_two_hundred_fifty_sixth_relation:?}{cubic_five_hundred_twelfth_relation:?}{cubic_endpoint_relation:?}{inflections:?}{quadratic_offset_preflight:?}{cubic_offset_preflight:?}{quadratic_staged_offset:?}{cubic_staged_offset:?}{quadratic_staged_right_offset:?}{cubic_staged_right_offset:?}{quadratic_offset_adapter_report:?}{region_summary:?}{region_refinements:?}{region_isolation:?}{targeted_region_isolation:?}{region_isolation_certificate:?}{boolean_handoff_from_line_image:?}{boolean_handoff_from_regions:?}{boolean_handoff_from_certificate:?}{boolean_handoff_batch:?}{boolean_overlap_resolution:?}{path_order_from_graph:?}{path_order_from_contact:?}{path_range_batch:?}{boolean_path_scheduler:?}{boolean_split_plan:?}{boolean_split_plan_audit:?}{boolean_split_insertion_report:?}{boolean_construction_readiness:?}{boolean_quadratic_fragments:?}{boolean_cubic_fragments:?}{boolean_rational_quadratic_fragments:?}{boolean_arrangement_readiness:?}{boolean_traversal_preconditions:?}{boolean_traversal_schedule:?}{boolean_ownership_classification:?}{boolean_ownership_facts:?}{boolean_uniform_ownership_facts:?}{boolean_emission_plan:?}{boolean_assembly_readiness:?}{boolean_loop_assembly_plan:?}{boolean_loop_graph_traversal:?}{boolean_loop_graph_facts:?}{boolean_loop_graph_walk:?}{boolean_loop_graph_walk_closure:?}{boolean_loop_closure:?}{boolean_output_loops:?}{boolean_graph_walk_output_loops:?}{boolean_nesting_roles:?}{boolean_nesting_depth_facts:?}{boolean_loop_containment_facts:?}{boolean_loop_roles:?}{boolean_region_assembly:?}{boolean_graph_walk_region_assembly:?}{boolean_graph_walk_result:?}{boolean_schedule_graph_walk_result:?}{boolean_schedule_graph_fact_result:?}{boolean_schedule_graph_fact_containment_result:?}{boolean_uniform_identity_containment_result:?}{boolean_uniform_linear_identity_containment_result:?}{boolean_result:?}"
        )
        .len();
    }
    let elapsed = start.elapsed();
    println!(
        "bezier_topology_predicates: {iterations} iterations in {elapsed:?} checksum={checksum}"
    );
}

fn bench_bezier_flattening(quadratic: &QuadraticBezier2, cubic: &CubicBezier2) {
    let iterations = 10_000;
    let policy = CurvePolicy::certified();
    let options = BezierFlatteningOptions::try_new(Real::one(), 12, &policy).unwrap();
    let start = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let quadratic_polyline = black_box(quadratic).flatten_certified(&options, &policy);
        let cubic_polyline = black_box(cubic).flatten_certified(&options, &policy);
        let simplified = quadratic_polyline
            .clone()
            .map(|polyline| polyline.simplify_exact_collinear(&policy));
        let simplification_certificate = simplified.clone().map(|simplified| match simplified {
            hypercurve::Classification::Decided(polyline) => {
                format!("{:?}", polyline.simplification_certificate())
            }
            hypercurve::Classification::Uncertain(reason) => format!("{reason:?}"),
        });
        let offset_preview = simplified.clone().map(|simplified| match simplified {
            hypercurve::Classification::Decided(polyline) => {
                format!(
                    "{:?}{:?}",
                    polyline.display_offset_left(Real::one()),
                    polyline.display_offset_right(Real::one())
                )
            }
            hypercurve::Classification::Uncertain(reason) => format!("{reason:?}"),
        });
        let checked_offset = simplified.clone().map(|simplified| match simplified {
            hypercurve::Classification::Decided(polyline) => {
                format!(
                    "{:?}{:?}",
                    polyline.checked_offset_left(Real::one(), &policy),
                    polyline.checked_offset_right(Real::one(), &policy)
                )
            }
            hypercurve::Classification::Uncertain(reason) => format!("{reason:?}"),
        });
        let line_fit = quadratic_polyline
            .clone()
            .map(|polyline| format!("{:?}", polyline.fit_exact_line(&policy)));
        let point_fit = quadratic_polyline
            .clone()
            .map(|polyline| format!("{:?}", polyline.fit_exact_point(&policy)));
        let fit_readiness = quadratic_polyline
            .clone()
            .map(|polyline| format!("{:?}", polyline.fit_readiness_report(&policy)));
        let fit_certificate =
            quadratic_polyline
                .clone()
                .map(|polyline| match polyline.fit_exact_point(&policy) {
                    Ok(hypercurve::Classification::Decided(
                        hypercurve::BezierPointFitRelation::Fit(fit),
                    )) => format!("{:?}", fit.fit_certificate()),
                    other => format!("{other:?}"),
                });
        let exact_line_offset =
            quadratic_polyline
                .clone()
                .map(|polyline| match polyline.fit_exact_line(&policy) {
                    Ok(hypercurve::Classification::Decided(
                        hypercurve::BezierLineFitRelation::Fit(fit),
                    )) => {
                        let offset = fit.offset_left_exact(Real::one());
                        let right_offset = fit.offset_right_exact(Real::one());
                        format!(
                            "{:?}{:?}{:?}",
                            offset,
                            right_offset,
                            offset.as_ref().map(|offset| offset.fit_certificate())
                        )
                    }
                    other => format!("{other:?}"),
                });
        checksum ^= format!(
            "{quadratic_polyline:?}{cubic_polyline:?}{simplified:?}{simplification_certificate:?}{offset_preview:?}{checked_offset:?}{line_fit:?}{point_fit:?}{fit_readiness:?}{fit_certificate:?}{exact_line_offset:?}"
        )
        .len();
    }
    let elapsed = start.elapsed();
    println!(
        "bezier_certified_flattening: {iterations} iterations in {elapsed:?} checksum={checksum}"
    );
}
