use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Classification, Curve2, CurveContext, CurveResult, NurbsCurve2, Point2,
    PolynomialBSplineCurve2, PolynomialSplineCurve2, RationalBSplineCurve2,
    RationalQuadraticBSplineCurve2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("benchmark denominator is nonzero")
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn large_nurbs_control_count() -> usize {
    std::env::var("HYPERCURVE_BENCH_NURBS_CONTROLS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(256)
        .clamp(4, i32::MAX as usize)
}

fn large_nurbs_iterations() -> u32 {
    std::env::var("HYPERCURVE_BENCH_NURBS_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(50)
        .max(1)
}

fn large_nurbs_inputs(control_count: usize) -> (Vec<Point2>, Vec<Real>, Vec<Real>) {
    let controls = (0..control_count)
        .map(|index| {
            let x = i32::try_from(index).unwrap();
            let y = i32::try_from(index.wrapping_mul(17) % 31).unwrap() - 15;
            p(x, y)
        })
        .collect();
    let weights = (0..control_count)
        .map(|index| r(i32::try_from(index % 5 + 1).unwrap()))
        .collect();
    let domain_end = i32::try_from(control_count - 3).unwrap();
    let knots = std::iter::repeat_n(r(0), 4)
        .chain((1..domain_end).map(r))
        .chain(std::iter::repeat_n(r(domain_end), 4))
        .collect();
    (controls, weights, knots)
}

fn bench_large_nurbs() {
    let control_count = large_nurbs_control_count();
    let iterations = large_nurbs_iterations();
    let (controls, weights, knots) = large_nurbs_inputs(control_count);

    let started = Instant::now();
    let mut cold_checksum = 0_usize;
    for _ in 0..iterations {
        let curve = NurbsCurve2::try_new(
            3,
            controls.clone(),
            weights.clone(),
            knots.clone(),
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        cold_checksum ^= black_box(curve.bezier_decomposition().unwrap().spans().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_large_cold_decomposition_{control_count}_controls: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cold_checksum}",
        elapsed / iterations
    );

    let curve = NurbsCurve2::try_new(3, controls, weights, knots, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    let domain_end = i32::try_from(control_count - 3).unwrap();
    let parameter = q(domain_end, 2);
    curve
        .point_at(&parameter)
        .expect("large NURBS midpoint evaluates exactly");
    let started = Instant::now();
    let mut evaluation_checksum = 0_usize;
    for _ in 0..iterations {
        let point = curve.point_at(&parameter).unwrap();
        evaluation_checksum ^=
            black_box(point.x().to_f64_lossy().unwrap_or_default().to_bits() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_large_cached_evaluation_{control_count}_controls: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={evaluation_checksum}",
        elapsed / iterations
    );
}

fn main() -> CurveResult<()> {
    if std::env::var_os("HYPERCURVE_BENCH_NURBS_ONLY").is_some() {
        bench_large_nurbs();
        return Ok(());
    }

    let policy = CurveContext::STRICT;
    let spline = decided(PolynomialBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(spline.extract_bezier_spans(&policy)?);
        let facts = decided(extraction.span_fact_evidence(&policy)?);
        checksum ^= black_box(
            extraction.spans().len() + extraction.inserted_knot_count() + facts.span_facts().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let cached_polynomial = PolynomialSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )
    .expect("benchmark polynomial spline is valid")
    .into_value();
    let started = Instant::now();
    let mut cached_polynomial_checksum = 0_usize;
    for _ in 0..iterations {
        let decomposition = cached_polynomial
            .bezier_decomposition()
            .expect("benchmark decomposition remains exact");
        cached_polynomial_checksum ^= black_box(
            decomposition.spans().len()
                + decomposition.intervals().len()
                + decomposition.inserted_knot_count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "polynomial_spline_cached_top_level_query: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cached_polynomial_checksum}",
        elapsed / iterations
    );

    let promoted_polynomial = Curve2::from(cached_polynomial);
    let started = Instant::now();
    let mut promotion_checksum = 0_usize;
    for _ in 0..iterations {
        promotion_checksum ^= black_box(
            promoted_polynomial
                .native_bezier_fragments(&CurveContext::STRICT)
                .expect("polynomial promotion remains exact")
                .into_value()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "polynomial_spline_cached_native_promotion: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={promotion_checksum}",
        elapsed / iterations
    );

    let rational = decided(RationalQuadraticBSplineCurve2::try_new(
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(4), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut rational_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational.extract_bezier_spans(&policy)?);
        let facts = decided(extraction.span_fact_evidence(&policy)?);
        rational_checksum ^= black_box(
            extraction.spans().len()
                + extraction.inserted_knot_count()
                + facts
                    .span_facts()
                    .iter()
                    .filter(|span| span.weight_domain().is_some())
                    .count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_quadratic_bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={rational_checksum}",
        elapsed / iterations
    );

    let rational_cubic = decided(RationalBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut rational_cubic_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational_cubic.extract_bezier_spans(&policy)?);
        let facts = decided(extraction.span_fact_evidence(&policy)?);
        rational_cubic_checksum ^= black_box(
            extraction.spans().len()
                + extraction.inserted_knot_count()
                + extraction.refined_weights().len()
                + facts.span_facts().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={rational_cubic_checksum}",
        elapsed / iterations
    );

    let equal_weight_rational_cubic = decided(RationalBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(5), r(5), r(5), r(5), r(5)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut native_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(equal_weight_rational_cubic.extract_bezier_spans(&policy)?);
        let evidence = decided(extraction.native_topology_evidence(&policy)?);
        let native = decided(extraction.native_subcurves(&policy)?);
        native_checksum ^= black_box(
            native.len()
                + evidence.span_evidence().len()
                + usize::from(evidence.is_fully_native_exact())
                + extraction.inserted_knot_count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_native_subcurves: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={native_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut topology_status_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational_cubic.extract_bezier_spans(&policy)?);
        let evidence = decided(extraction.native_topology_evidence(&policy)?);
        topology_status_checksum ^= black_box(
            evidence.span_evidence().len()
                + evidence
                    .span_evidence()
                    .iter()
                    .filter(|span| span.status().is_retained_evidence())
                    .count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_topology_status_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={topology_status_checksum}",
        elapsed / iterations
    );

    let cached_nurbs = NurbsCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )
    .expect("benchmark NURBS is valid")
    .into_value();
    let started = Instant::now();
    let mut cached_checksum = 0_usize;
    for _ in 0..iterations {
        let decomposition = cached_nurbs
            .bezier_decomposition()
            .expect("benchmark decomposition remains exact");
        let native = cached_nurbs
            .native_subcurves()
            .expect("general rational cubic remains native");
        cached_checksum ^= black_box(
            decomposition.spans().len() + decomposition.inserted_knot_count() + native.len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cached_top_level_queries: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cached_checksum}",
        elapsed / iterations
    );

    let parameter = (r(1) / r(2)).expect("two is nonzero");
    cached_nurbs
        .point_at(&parameter)
        .expect("initial rational evaluation remains exact");
    let started = Instant::now();
    let mut evaluation_count = 0_u32;
    for _ in 0..iterations {
        black_box(
            cached_nurbs
                .point_at(&parameter)
                .expect("cached rational evaluation remains exact"),
        );
        evaluation_count += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cached_general_rational_evaluation: {iterations} iterations in {elapsed:?} ({:?}/iter), count={evaluation_count}",
        elapsed / iterations
    );

    cached_nurbs
        .derivative_at(&parameter)
        .expect("initial rational derivative remains exact");
    let started = Instant::now();
    let mut derivative_count = 0_u32;
    for _ in 0..iterations {
        black_box(
            cached_nurbs
                .derivative_at(&parameter)
                .expect("cached rational derivative remains exact"),
        );
        derivative_count += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cached_general_rational_derivative: {iterations} iterations in {elapsed:?} ({:?}/iter), count={derivative_count}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut higher_derivative_count = 0_usize;
    for _ in 0..iterations {
        higher_derivative_count += black_box(
            cached_nurbs
                .derivatives_at(&parameter, 3)
                .expect("cached higher rational derivatives remain exact")
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cached_general_rational_derivatives_1_through_3: {iterations} iterations in {elapsed:?} ({:?}/iter), count={higher_derivative_count}",
        elapsed / iterations
    );

    let periodic_controls = vec![p(0, 0), p(2, 0), p(2, 2), p(0, 2)];
    let periodic_weights = vec![r(1), r(2), r(3), r(4)];
    let periodic_knots = (0..=4).map(r).collect::<Vec<_>>();
    let construction_iterations = 2_000_u32;
    let started = Instant::now();
    let mut periodic_construction_checksum = 0_usize;
    for _ in 0..construction_iterations {
        let curve = NurbsCurve2::try_new_periodic(
            2,
            periodic_controls.clone(),
            periodic_weights.clone(),
            periodic_knots.clone(),
            &policy,
        )
        .expect("periodic benchmark NURBS is valid")
        .into_value();
        periodic_construction_checksum ^=
            black_box(curve.control_points().len() + curve.knots().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_periodic_exact_construction: {construction_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={periodic_construction_checksum}",
        elapsed / construction_iterations
    );

    let periodic = NurbsCurve2::try_new_periodic(
        2,
        periodic_controls,
        periodic_weights,
        periodic_knots,
        &policy,
    )
    .expect("periodic benchmark NURBS is valid")
    .into_value();
    let wrapped_parameter = r(4_000_000) + q(1, 2);
    periodic
        .point_at_wrapped(&wrapped_parameter)
        .expect("large periodic parameter wraps exactly");
    let started = Instant::now();
    let mut periodic_evaluation_count = 0_u32;
    for _ in 0..iterations {
        black_box(
            periodic
                .point_at_wrapped(&wrapped_parameter)
                .expect("cached periodic evaluation remains exact"),
        );
        periodic_evaluation_count += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cached_large_periodic_evaluation: {iterations} iterations in {elapsed:?} ({:?}/iter), count={periodic_evaluation_count}",
        elapsed / iterations
    );

    let refinement_source = || {
        NurbsCurve2::try_new(
            2,
            vec![p(0, 0), p(2, 4), p(4, 0)],
            vec![r(1), r(2), r(1)],
            vec![r(0), r(0), r(0), r(2), r(2), r(2)],
            &policy,
        )
        .expect("refinement benchmark NURBS is valid")
        .into_value()
    };
    let refinement_iterations = 2_000_u32;
    let cold_batch_inputs = (0..refinement_iterations)
        .map(|_| refinement_source())
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut batch_refinement_checksum = 0_usize;
    for curve in &cold_batch_inputs {
        batch_refinement_checksum ^= black_box(
            curve
                .insert_knots(vec![r(1), r(1)])
                .unwrap()
                .control_points()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cold_batch_knot_refinement: {refinement_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={batch_refinement_checksum}",
        elapsed / refinement_iterations
    );

    let cold_sequential_inputs = (0..refinement_iterations)
        .map(|_| refinement_source())
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut sequential_refinement_checksum = 0_usize;
    for curve in &cold_sequential_inputs {
        sequential_refinement_checksum ^= black_box(
            curve
                .insert_knot(r(1))
                .unwrap()
                .insert_knot(r(1))
                .unwrap()
                .control_points()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_cold_sequential_knot_refinement: {refinement_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={sequential_refinement_checksum}",
        elapsed / refinement_iterations
    );

    let retained_refinement = refinement_source();
    retained_refinement.insert_knots(vec![r(1), r(1)]).unwrap();
    let started = Instant::now();
    let mut retained_refinement_checksum = 0_usize;
    for _ in 0..iterations {
        retained_refinement_checksum ^= black_box(
            retained_refinement
                .insert_knots(vec![r(1), r(1)])
                .unwrap()
                .control_points()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_retained_batch_knot_refinement: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_refinement_checksum}",
        elapsed / iterations
    );

    let removal_knot = q(1, 2);
    let removal_source = || {
        refinement_source()
            .insert_knot(removal_knot.clone())
            .expect("removal benchmark refinement remains exact")
    };
    let removal_inputs = (0..refinement_iterations)
        .map(|_| removal_source())
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut removal_checksum = 0_usize;
    for curve in &removal_inputs {
        removal_checksum ^= black_box(
            curve
                .remove_knot(removal_knot.clone())
                .unwrap()
                .expect("inserted benchmark knot is removable")
                .control_points()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_exact_knot_removal_proof: {refinement_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={removal_checksum}",
        elapsed / refinement_iterations
    );

    let retained_removal = removal_source();
    retained_removal
        .remove_knot(removal_knot.clone())
        .unwrap()
        .expect("inserted benchmark knot is removable");
    let started = Instant::now();
    let mut retained_removal_checksum = 0_usize;
    for _ in 0..iterations {
        retained_removal_checksum ^= black_box(
            retained_removal
                .remove_knot(removal_knot.clone())
                .unwrap()
                .expect("retained benchmark knot is removable")
                .control_points()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_retained_knot_removal_proof: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_removal_checksum}",
        elapsed / iterations
    );

    let elevation_source = || {
        NurbsCurve2::try_new(
            3,
            vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
            vec![r(1), r(2), r(4), r(8), r(16)],
            vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
            &policy,
        )
        .expect("degree-elevation benchmark NURBS is valid")
        .into_value()
    };
    let elevation_inputs = (0..1_000).map(|_| elevation_source()).collect::<Vec<_>>();
    let started = Instant::now();
    let mut elevation_checksum = 0_usize;
    for curve in &elevation_inputs {
        let elevated = curve.degree_elevation(6).unwrap();
        elevation_checksum ^= black_box(elevated.spans().len() + elevated.target_degree());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_exact_span_degree_elevation: {} curves in {elapsed:?} ({:?}/curve), checksum={elevation_checksum}",
        elevation_inputs.len(),
        elapsed / u32::try_from(elevation_inputs.len()).unwrap()
    );

    let retained_elevation = elevation_source();
    retained_elevation.degree_elevation(6).unwrap();
    let started = Instant::now();
    let mut retained_elevation_checksum = 0_usize;
    for _ in 0..iterations {
        let elevated = retained_elevation.degree_elevation(6).unwrap();
        retained_elevation_checksum ^= black_box(elevated.spans().len() + elevated.target_degree());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_retained_span_degree_elevation: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_elevation_checksum}",
        elapsed / iterations
    );

    let elevated_curve_inputs = (0..1_000).map(|_| elevation_source()).collect::<Vec<_>>();
    let started = Instant::now();
    let mut elevated_curve_checksum = 0_usize;
    for curve in &elevated_curve_inputs {
        let elevated = curve.elevated_to_degree(6).unwrap();
        elevated_curve_checksum ^=
            black_box(elevated.control_points().len() + elevated.knots().len() + elevated.degree());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_exact_continuity_preserving_degree_elevation: {} curves in {elapsed:?} ({:?}/curve), checksum={elevated_curve_checksum}",
        elevated_curve_inputs.len(),
        elapsed / u32::try_from(elevated_curve_inputs.len()).unwrap()
    );

    let retained_elevated_curve = elevation_source();
    retained_elevated_curve.elevated_to_degree(6).unwrap();
    let started = Instant::now();
    let mut retained_elevated_curve_checksum = 0_usize;
    for _ in 0..iterations {
        let elevated = retained_elevated_curve.elevated_to_degree(6).unwrap();
        retained_elevated_curve_checksum ^= black_box(elevated.control_points().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_retained_continuity_preserving_degree_elevation: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_elevated_curve_checksum}",
        elapsed / iterations
    );

    let interpolation_points = vec![p(0, 0), p(1, 4), p(4, 3), p(6, 0)];
    let interpolation_inputs = (0..1_000)
        .map(|_| interpolation_points.clone())
        .collect::<Vec<_>>();
    let started = Instant::now();
    let mut interpolation_checksum = 0_usize;
    for points in interpolation_inputs {
        let curve = NurbsCurve2::interpolate_uniform(3, points, &policy)
            .unwrap()
            .into_value();
        interpolation_checksum ^= black_box(curve.control_points().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_exact_global_interpolation: 1000 curves in {elapsed:?} ({:?}/curve), checksum={interpolation_checksum}",
        elapsed / 1_000
    );

    let symbolic_interpolation_points = vec![p(0, 0), p(1, 0), p(3, 0), p(6, 0)];
    let symbolic_interpolation_count = 100_u32;
    let started = Instant::now();
    let mut symbolic_interpolation_checksum = 0_usize;
    for _ in 0..symbolic_interpolation_count {
        let curve =
            NurbsCurve2::interpolate_centripetal(2, symbolic_interpolation_points.clone(), &policy)
                .unwrap()
                .into_value();
        symbolic_interpolation_checksum ^= black_box(curve.control_points().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_exact_symbolic_centripetal_interpolation: {symbolic_interpolation_count} curves in {elapsed:?} ({:?}/curve), checksum={symbolic_interpolation_checksum}",
        elapsed / symbolic_interpolation_count
    );

    let retained_interpolation = NurbsCurve2::interpolate_uniform(3, interpolation_points, &policy)
        .unwrap()
        .into_value();
    let started = Instant::now();
    let mut retained_interpolation_checksum = 0_usize;
    for _ in 0..iterations {
        let replay = retained_interpolation.clone();
        retained_interpolation_checksum ^= black_box(replay.control_points().len());
    }
    let elapsed = started.elapsed();
    println!(
        "nurbs_clone_interpolated_curve: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_interpolation_checksum}",
        elapsed / iterations
    );

    bench_large_nurbs();

    Ok(())
}
