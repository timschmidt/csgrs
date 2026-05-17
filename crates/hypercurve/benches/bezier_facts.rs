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
    let cubic_quarter_hit = CubicBezier2::new(p(0, 40), p(20, 0), p(40, 0), p(60, 360));
    let cubic_eighth_hit = CubicBezier2::new(p(0, 10), p(20, 0), p(40, 0), p(60, 3290));
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
        let cubic_quarter_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_quarter_hit, &policy);
        let cubic_eighth_relation =
            black_box(&degree_elevated_arch).relation_to_cubic(&cubic_eighth_hit, &policy);
        let cubic_endpoint_relation =
            black_box(&cubic_endpoint_probe).relation_to_cubic(&degree_elevated_arch, &policy);
        let inflections = black_box(cubic).inflection_classification(&policy);

        checksum ^= format!(
            "{y_roots:?}{spans:?}{bounds:?}{line_relation:?}{point_parameters:?}{cubic_line_relation:?}{mixed_relation:?}{region_relation:?}{line_image_relation:?}{line_image_curve_relation:?}{endpoint_relation:?}{same_axis_no_hit_relation:?}{degree_normalized_no_hit_relation:?}{degree_elevated_identity_relation:?}{mixed_degree_midpoint_relation:?}{mixed_degree_quarter_relation:?}{cubic_quarter_relation:?}{cubic_eighth_relation:?}{cubic_endpoint_relation:?}{inflections:?}"
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
