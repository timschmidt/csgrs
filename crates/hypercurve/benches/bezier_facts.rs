use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Axis2, BezierAreaMomentPrefixSums2, BezierAreaPrefixSums2, BezierFlatteningOptions,
    CubicBezier2, CurvePolicy, LineSeg2, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
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
    }
    let elapsed = start.elapsed();
    println!("quadratic_bezier_facts: {iterations} iterations in {elapsed:?} checksum={checksum}");
}

fn bench_cubic_facts(curve: &CubicBezier2) {
    let iterations = 100_000;
    let policy = CurvePolicy::certified();
    let half = half();
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
    let endpoint_arch =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(2, 3), p(4, 0), Real::from(2_i8))
            .unwrap();
    let endpoint_probe = CubicBezier2::new(p(2, 2), p(3, 5), p(5, 5), p(6, 4));
    let equal_weight_arch =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(20, 16), p(40, 0), Real::one())
            .unwrap();
    let equal_weight_crossing =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 8), p(20, -8), p(40, 8), Real::one())
            .unwrap();
    let projective_scaled = RationalQuadraticBezier2::try_new(
        p(0, 0),
        p(17, 23),
        p(41, -5),
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
        checksum ^= format!("{:?}", black_box(curve).fit_exact_line_image(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).conic_kind(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).monotone_spans(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).certified_bounds(&policy)).len() as u8;
        checksum ^= format!("{:?}", black_box(curve).relation_to_line(&line, &policy)).len() as u8;
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
            black_box(&equal_weight_arch)
                .relation_to_rational_quadratic(&equal_weight_crossing, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&projective_scaled, &policy)
        )
        .len() as u8;
        checksum ^= format!(
            "{:?}",
            black_box(curve).relation_to_rational_quadratic(&matching_weight_crossing, &policy)
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
    let tangent_line_image = QuadraticBezier2::new(p(0, 8), p(20, 8), p(40, 8));
    let tangent_arch = QuadraticBezier2::new(p(0, 0), p(20, 16), p(40, 0));
    let endpoint_probe = CubicBezier2::new(p(20, 8), p(23, 20), p(50, 20), p(52, 19));
    let raised_arch = QuadraticBezier2::new(p(0, 4), p(20, 20), p(40, 4));
    let degree_normalized_arch = QuadraticBezier2::new(p(0, 0), p(30, 60), p(60, 0));
    let degree_elevated_arch = CubicBezier2::new(p(0, 0), p(20, 40), p(40, 40), p(60, 0));
    let degree_elevated_raised_arch = CubicBezier2::new(p(0, 4), p(20, 44), p(40, 44), p(60, 4));
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
    let start = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let y_roots = black_box(quadratic).axis_monotone_parameters(Axis2::Y, &policy);
        let spans = black_box(quadratic).monotone_spans(&policy);
        let bounds = black_box(quadratic).certified_bounds(&policy);
        let line_relation = black_box(quadratic).relation_to_line(&line, &policy);
        let point_parameters = black_box(quadratic).parameters_for_point(&p(17, 9), &policy);
        let cubic_line_relation = black_box(cubic).relation_to_line(&line, &policy);
        let mixed_relation = black_box(quadratic).relation_to_cubic(&shifted_cubic, &policy);
        let region_relation =
            black_box(quadratic).relation_to_quadratic(&crossing_quadratic, &policy);
        let line_image_relation =
            black_box(&line_image_quadratic).relation_to_cubic(&line_image_cubic, &policy);
        let line_image_curve_relation =
            black_box(&tangent_line_image).relation_to_quadratic(&tangent_arch, &policy);
        let endpoint_relation =
            black_box(&endpoint_probe).relation_to_quadratic(&tangent_arch, &policy);
        let same_axis_no_hit_relation =
            black_box(&tangent_arch).relation_to_quadratic(&raised_arch, &policy);
        let degree_normalized_no_hit_relation = black_box(&degree_normalized_arch)
            .relation_to_cubic(&degree_elevated_raised_arch, &policy);
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

        checksum ^= format!(
            "{y_roots:?}{spans:?}{bounds:?}{line_relation:?}{point_parameters:?}{cubic_line_relation:?}{mixed_relation:?}{region_relation:?}{line_image_relation:?}{line_image_curve_relation:?}{endpoint_relation:?}{same_axis_no_hit_relation:?}{degree_normalized_no_hit_relation:?}{degree_elevated_identity_relation:?}{mixed_degree_midpoint_relation:?}{mixed_degree_quarter_relation:?}{mixed_degree_thirty_second_relation:?}{mixed_degree_sixty_fourth_relation:?}{mixed_degree_one_hundred_twenty_eighth_relation:?}{mixed_degree_two_hundred_fifty_sixth_relation:?}{mixed_degree_five_hundred_twelfth_relation:?}{mixed_degree_non_dyadic_graph_relation:?}{non_dyadic_quadratic_root_relation:?}{non_graph_deep_dyadic_relation:?}{non_graph_irreducible_relation:?}{cubic_quarter_relation:?}{cubic_eighth_relation:?}{cubic_sixteenth_relation:?}{cubic_thirty_second_relation:?}{cubic_sixty_fourth_relation:?}{cubic_one_hundred_twenty_eighth_relation:?}{cubic_two_hundred_fifty_sixth_relation:?}{cubic_five_hundred_twelfth_relation:?}{cubic_endpoint_relation:?}{inflections:?}"
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
        let offset_preview = simplified.clone().map(|simplified| match simplified {
            hypercurve::Classification::Decided(polyline) => {
                format!("{:?}", polyline.display_offset_left(Real::one()))
            }
            hypercurve::Classification::Uncertain(reason) => format!("{reason:?}"),
        });
        let line_fit = quadratic_polyline
            .clone()
            .map(|polyline| format!("{:?}", polyline.fit_exact_line(&policy)));
        let exact_line_offset =
            quadratic_polyline
                .clone()
                .map(|polyline| match polyline.fit_exact_line(&policy) {
                    Ok(hypercurve::Classification::Decided(
                        hypercurve::BezierLineFitRelation::Fit(fit),
                    )) => {
                        format!("{:?}", fit.offset_left_exact(Real::one()))
                    }
                    other => format!("{other:?}"),
                });
        checksum ^= format!(
            "{quadratic_polyline:?}{cubic_polyline:?}{simplified:?}{offset_preview:?}{line_fit:?}{exact_line_offset:?}"
        )
        .len();
    }
    let elapsed = start.elapsed();
    println!(
        "bezier_certified_flattening: {iterations} iterations in {elapsed:?} checksum={checksum}"
    );
}
