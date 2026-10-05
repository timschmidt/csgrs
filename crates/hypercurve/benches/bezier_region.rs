#[path = "../tests/support/mod.rs"]
mod support;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BooleanOp, BulgeVertex2, Classification, Contour2, Curve2,
    CurveContext, CurveError, CurvePath2, CurvePoint2, CurveRegion2, CurveRegionLoopRole,
    CurveResult, FillRule, LineSeg2, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(xmin, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymax), Real::zero()),
        BulgeVertex2::new(p(xmin, ymax), Real::zero()),
    ])
    .unwrap()
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn square_path(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> CurveResult<CurvePath2> {
    let points = [
        p(min_x, min_y),
        p(max_x, min_y),
        p(max_x, max_y),
        p(min_x, max_y),
    ];
    CurvePath2::try_new(
        (0..points.len())
            .map(|index| {
                LineSeg2::try_new(
                    points[index].clone(),
                    points[(index + 1) % points.len()].clone(),
                )
                .map(Curve2::from)
            })
            .collect::<CurveResult<Vec<_>>>()?,
    )
    .map_err(|error| match error {
        hypercurve::ExactCurveError::Invalid { cause, .. } => cause,
        hypercurve::ExactCurveError::Blocked(blocker) => CurveError::Topology(format!(
            "square benchmark path blocked: {:?}",
            blocker.reason()
        )),
    })
}

fn square_region(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> CurveResult<CurveRegion2> {
    let path = square_path(min_x, min_y, max_x, max_y)?;
    CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd).map_err(|error| {
        match error {
            hypercurve::ExactCurveError::Invalid { cause, .. } => cause,
            hypercurve::ExactCurveError::Blocked(blocker) => CurveError::Topology(format!(
                "square benchmark region blocked: {:?}",
                blocker.reason()
            )),
        }
    })
}

fn path_region(path: &CurvePath2, policy: &CurveContext) -> CurveResult<CurveRegion2> {
    crate::support::under(policy, || {
        CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            std::slice::from_ref(path),
            &[CurveRegionLoopRole::Material],
            &[FillRule::EvenOdd],
        )
    })
    .map(|outcome| outcome.into_value())
    .map_err(|error| match error {
        hypercurve::ExactCurveError::Invalid { cause, .. } => cause,
        hypercurve::ExactCurveError::Blocked(blocker) => CurveError::Topology(format!(
            "benchmark path promotion blocked: {:?}",
            blocker.reason()
        )),
    })
}

fn algebraic_polynomial_parameter(
    coefficients: Vec<Real>,
    interval_start: Real,
    interval_end: Real,
    policy: &CurveContext,
) -> CurveResult<BezierParameter2> {
    let polynomial = decided(
        crate::support::under_classified_result(policy, || {
            BezierParameterPolynomial::try_new_power_basis(coefficients)
        })
        .expect("benchmark fixture remains exact"),
    );
    let interval = decided(
        crate::support::under_classified_result(policy, || {
            BezierParameterInterval::try_new(interval_start, interval_end)
        })
        .expect("benchmark fixture remains exact"),
    );
    Ok(BezierParameter2::Algebraic(decided(
        crate::support::under_classified_result(policy, || {
            BezierAlgebraicParameter2::try_isolate(polynomial, interval)
        })
        .expect("benchmark fixture remains exact"),
    )))
}

fn algebraic_chord(start: Point2, end: Point2, policy: &CurveContext) -> CurveResult<Curve2> {
    Ok(crate::support::under(policy, || {
        Curve2::try_line(CurvePoint2::from(start), CurvePoint2::from(end))
    })
    .expect("the benchmark chord must be admitted")
    .into_value())
}

fn benchmark_measurements(
    region: &CurveRegion2,
    policy: &CurveContext,
) -> Result<(), Box<dyn std::error::Error>> {
    let iterations = std::env::var("HYPERCURVE_BEZIER_REGION_MEASURE_ITERATIONS")
        .ok()
        .and_then(|value| value.parse::<u32>().ok())
        .unwrap_or(1_000_000);
    let expected_area = crate::support::under(policy, || region.signed_area())?
        .into_value()
        .expect("benchmark square has an exact area");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let area = crate::support::under(black_box(policy), || black_box(region).signed_area())?
            .into_value()
            .expect("cached square area remains exact");
        checksum += black_box(area == expected_area) as usize;
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_cached_signed_area: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let curve = hypercurve::CubicBezier2::new(p(0, 0), p(1, 3), p(3, -2), p(4, 0));
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(black_box(&curve).signed_area_contribution()?);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_subcurve_exact_signed_area: {iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / iterations
    );

    let started = Instant::now();
    for _ in 0..iterations {
        black_box(black_box(&curve).area_moments_contribution()?);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_subcurve_exact_area_moments: {iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let policy = CurveContext::STRICT;
    let first_region = square_region(0, 0, 4, 4)?;
    let second_region = square_region(2, 0, 6, 4)?;
    if std::env::var_os("HYPERCURVE_BEZIER_REGION_MEASURE_ONLY").is_some() {
        return benchmark_measurements(&first_region, &policy);
    }
    let region_clone_iterations = 1_000_000_u32;
    let started = Instant::now();
    let mut region_clone_checksum = 0_usize;
    for _ in 0..region_clone_iterations {
        let cloned = black_box(&first_region).clone();
        region_clone_checksum =
            region_clone_checksum.wrapping_add(black_box(cloned.boundary_loops().len()));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_clone: {region_clone_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={region_clone_checksum}",
        elapsed / region_clone_iterations
    );

    black_box(CurveRegion2::empty());
    let started = Instant::now();
    let mut empty_region_checksum = 0_usize;
    for _ in 0..region_clone_iterations {
        empty_region_checksum =
            empty_region_checksum.wrapping_add(black_box(CurveRegion2::empty()).len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_empty: {region_clone_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={empty_region_checksum}",
        elapsed / region_clone_iterations
    );
    if std::env::var_os("HYPERCURVE_BEZIER_REGION_CARRIER_ONLY").is_some() {
        return Ok(());
    }

    let moment_curve = hypercurve::CubicBezier2::new(p(0, 0), p(1, 3), p(3, -2), p(4, 0));
    let moment_iterations = 20_000_u32;
    let started = Instant::now();
    for _ in 0..moment_iterations {
        black_box(black_box(&moment_curve).area_moments_contribution()?);
    }
    let elapsed = started.elapsed();
    println!(
        "cubic_bezier_exact_area_moments: {moment_iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / moment_iterations
    );

    let prefix_parameter = q(3, 4);
    let started = Instant::now();
    for _ in 0..moment_iterations {
        black_box(decided(crate::support::under_classified_result(
            &policy,
            || black_box(&moment_curve).prefix_area_moments_contribution(prefix_parameter.clone()),
        )?));
    }
    let elapsed = started.elapsed();
    println!(
        "cubic_bezier_prefix_area_moments: {moment_iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / moment_iterations
    );

    let started = Instant::now();
    for _ in 0..moment_iterations {
        black_box(decided(crate::support::under_classified_result(
            &policy,
            || black_box(&moment_curve).prefix_length_bounds(prefix_parameter.clone()),
        )?));
    }
    let elapsed = started.elapsed();
    println!(
        "cubic_bezier_prefix_length_bounds: {moment_iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / moment_iterations
    );

    let region_boolean_iterations = 1_000_u32;
    let started = Instant::now();
    let mut region_boolean_checksum = 0_usize;
    for _ in 0..region_boolean_iterations {
        let region = crate::support::under(&policy, || {
            first_region.boolean_region(&second_region, BooleanOp::Union)
        })
        .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?
        .value;
        region_boolean_checksum ^= black_box(region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_boolean_immediate_union: {region_boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={region_boolean_checksum}",
        elapsed / region_boolean_iterations
    );

    let started = Instant::now();
    let mut batch_region_boolean_checksum = 0_usize;
    let batch_region_boolean_iterations = 1_000_u32;
    for _ in 0..batch_region_boolean_iterations {
        let results =
            crate::support::under(&policy, || first_region.boolean_regions(&second_region))
                .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?
                .value;
        batch_region_boolean_checksum ^= black_box(
            results.union().boundary_loops().len()
                + results.intersection().boundary_loops().len()
                + results.difference().boundary_loops().len()
                + results.xor().boundary_loops().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_boolean_immediate_all_ops: {batch_region_boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={batch_region_boolean_checksum}",
        elapsed / batch_region_boolean_iterations
    );

    let curved = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-2, 4), p(0, -4), p(2, 4))),
        Curve2::from(LineSeg2::try_new(p(2, 4), p(-2, 4))?),
    ])
    .map_err(|error| CurveError::Topology(format!("curved benchmark path: {error}")))?;
    let cutter = square_path(-3, 2, 3, 5)?;
    let curved_region = path_region(&curved, &policy)?;
    let cutter_region = path_region(&cutter, &policy)?;
    let algebraic = crate::support::under(&policy, || {
        curved_region.boolean_region(&cutter_region, BooleanOp::Difference)
    })
    .map_err(|error| CurveError::Topology(format!("curved benchmark setup: {error}")))?
    .into_value();
    let crossing = square_region(-2, -1, 2, 1)?;
    let curved_boolean_iterations = 100_u32;
    let started = Instant::now();
    let mut curved_boolean_checksum = 0_usize;
    for _ in 0..curved_boolean_iterations {
        let results = crate::support::under(&policy, || algebraic.boolean_regions(&crossing))
            .map_err(|error| CurveError::Topology(format!("curved benchmark: {error}")))?
            .value;
        curved_boolean_checksum = curved_boolean_checksum.wrapping_add(black_box(
            results.topology_fragment_count()
                + results.topology_point_classification_count()
                + results.union().boundary_loops().len(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_boolean_retained_algebraic_all_ops: {curved_boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={curved_boolean_checksum}",
        elapsed / curved_boolean_iterations
    );

    let algebraic_ray_path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), r(0)),
            p(1, 1),
        )),
        Curve2::from(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(1, 2), q(1, 2)),
            p(0, 0),
        )),
    ])?;
    let algebraic_ray_region = even_odd_region(&[algebraic_ray_path], &policy)?;
    let algebraic_ray_query = CurvePoint2::from(Point2::new(q(1, 2), q(3, 8)));
    let classification_iterations = 2_000_u32;
    let started = Instant::now();
    let mut classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = crate::support::under(&policy, || {
            algebraic_ray_region.classify_point(black_box(&algebraic_ray_query))
        })?
        .into_value();
        classification_checksum =
            classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_region_algebraic_ray_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={classification_checksum}",
        elapsed / classification_iterations
    );

    let half = BezierParameter2::Exact(q(1, 2));
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let lens_path = halved_loop(
        [Curve2::from(upper.clone()), lower.clone().into()],
        &half,
        &policy,
    )?;

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let region = even_odd_region(std::slice::from_ref(&lens_path), &policy)?;
        checksum ^= black_box(
            format!(
                "{:?}",
                crate::support::under(&policy, || region.signed_area())?.into_value()
            )
            .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let classified_region = even_odd_region(std::slice::from_ref(&lens_path), &policy)?;
    let classified_point = hypercurve::CurvePoint2::from(p(2, 0));
    crate::support::under(&policy, || {
        classified_region.classify_point(&classified_point)
    })?
    .into_value();
    let started = Instant::now();
    let mut curved_classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = crate::support::under(black_box(&policy), || {
            classified_region.classify_point(black_box(&classified_point))
        })?
        .into_value();
        curved_classification_checksum =
            curved_classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_cached_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={curved_classification_checksum}",
        elapsed / classification_iterations
    );

    let immediate_region = crate::support::under(&policy, || {
        CurveRegion2::try_from_native_material_contours(vec![rectangle(-4, -4, 4, 4)])
    })
    .unwrap()
    .into_value();
    let native_point = hypercurve::CurvePoint2::from(p(1, 1));
    crate::support::under(&policy, || immediate_region.classify_point(&native_point))?.into_value();
    let started = Instant::now();
    let mut native_classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = crate::support::under(black_box(&policy), || {
            immediate_region.classify_point(black_box(&native_point))
        })?
        .into_value();
        native_classification_checksum =
            native_classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_immediate_native_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={native_classification_checksum}",
        elapsed / classification_iterations
    );

    let started = Instant::now();
    let mut retained_checksum = 0_usize;
    for _ in 0..iterations {
        let region = even_odd_region(std::slice::from_ref(&lens_path), &policy)?;
        retained_checksum ^= black_box(
            format!(
                "{:?}",
                crate::support::under(&policy, || region.signed_area())?.into_value()
            )
            .len(),
        );
        let envelope = crate::support::under(&policy, || region.bounds())?.into_value();
        retained_checksum ^= black_box(format!("{envelope:?}").len());
        let roles = crate::support::under(&policy, || region.loop_roles())?.into_value();
        retained_checksum ^= black_box(roles.len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_checksum}",
        elapsed / iterations
    );

    let algebraic_cut =
        algebraic_polynomial_parameter(vec![r(-1), r(0), r(2)], q(2, 3), q(3, 4), &policy)?;
    let (head, tail) = crate::support::under(&policy, || {
        Curve2::from(upper.clone()).split_at(algebraic_cut.into())
    })?
    .into_value();
    let algebraic_path = CurvePath2::try_new(vec![head, tail, lower.into()])?;
    let algebraic_region = even_odd_region(&[algebraic_path], &policy)?;
    let algebraic_region_query = hypercurve::CurvePoint2::from(p(2, 0));
    crate::support::under(&policy, || {
        algebraic_region.classify_point(&algebraic_region_query)
    })?
    .into_value();
    let started = Instant::now();
    let mut algebraic_classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = crate::support::under(black_box(&policy), || {
            algebraic_region.classify_point(black_box(&algebraic_region_query))
        })?
        .into_value();
        algebraic_classification_checksum =
            algebraic_classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_cached_algebraic_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_classification_checksum}",
        elapsed / classification_iterations
    );
    let started = Instant::now();
    let mut algebraic_envelope_checksum = 0_usize;
    for _ in 0..iterations {
        let envelope = crate::support::under(&policy, || algebraic_region.bounds())?.into_value();
        algebraic_envelope_checksum ^= black_box(format!("{envelope:?}").len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_algebraic_source_envelope: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_envelope_checksum}",
        elapsed / iterations
    );

    let mut algebraic_paths = Vec::with_capacity(2);
    for (min, max) in [(-3, 3), (-1, 1)] {
        algebraic_paths.push(CurvePath2::try_new(vec![
            algebraic_chord(p(min, min), p(max, min), &policy)?,
            algebraic_chord(p(max, min), p(max, max), &policy)?,
            algebraic_chord(p(max, max), p(min, max), &policy)?,
            algebraic_chord(p(min, max), p(min, min), &policy)?,
        ])?);
    }
    let algebraic_line_region = even_odd_region(&algebraic_paths, &policy)?;
    let started = Instant::now();
    let mut algebraic_line_role_checksum = 0_usize;
    for _ in 0..iterations {
        let roles =
            crate::support::under(&policy, || algebraic_line_region.loop_roles())?.into_value();
        let material_count = roles
            .iter()
            .filter(|role| matches!(role, CurveRegionLoopRole::Material))
            .count();
        let hole_count = roles.len() - material_count;
        algebraic_line_role_checksum ^= black_box(roles.len() + material_count + hole_count);
        algebraic_line_role_checksum ^= black_box(
            format!(
                "{:?}",
                crate::support::under(&policy, || algebraic_line_region.filled_area())?
            )
            .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_algebraic_line_authoritative_roles: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_line_role_checksum}",
        elapsed / iterations
    );

    // Two squares traverse their shared edge in opposite directions.
    let overlap_paths = [square_path(0, 0, 2, 2)?, square_path(2, 0, 4, 2)?];
    let started = Instant::now();
    let mut overlap_checksum = 0_usize;
    for _ in 0..iterations {
        let retained = even_odd_region(&overlap_paths, &policy)?;
        overlap_checksum ^= black_box(
            format!(
                "{:?}",
                crate::support::under(&policy, || retained.signed_area())?.into_value()
            )
            .len(),
        );
        let roles = crate::support::under(&policy, || retained.loop_roles())?.into_value();
        overlap_checksum ^= black_box(roles.len());
        overlap_checksum ^= black_box(usize::from(
            crate::support::under(&policy, || retained.filled_area())?
                .into_value()
                .is_some(),
        ));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_resolved_overlap_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={overlap_checksum}",
        elapsed / iterations
    );

    let conic_upper =
        RationalQuadraticBezier2::try_unit_end_weights(p(0, 0), p(2, 2), p(4, 0), q(1, 2))?;
    let conic_lower =
        RationalQuadraticBezier2::try_unit_end_weights(p(4, 0), p(2, -2), p(0, 0), q(1, 2))?;
    let conic_path = halved_loop([conic_upper.into(), conic_lower.into()], &half, &policy)?;
    let started = Instant::now();
    let mut conic_checksum = 0_usize;
    for _ in 0..iterations {
        let region = even_odd_region(std::slice::from_ref(&conic_path), &policy)?;
        conic_checksum ^= black_box(
            format!(
                "{:?}",
                crate::support::under(&policy, || region.signed_area())?.into_value()
            )
            .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_conic_region_exact_area: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={conic_checksum}",
        elapsed / iterations
    );

    Ok(())
}

/// Splits each closed-loop curve at one parameter and joins the halves.
fn halved_loop(
    curves: [Curve2; 2],
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> Result<CurvePath2, Box<dyn std::error::Error>> {
    let mut halves = Vec::with_capacity(4);
    for curve in curves {
        let (first, second) =
            crate::support::under(policy, || curve.split_at(parameter.clone().into()))?
                .into_value();
        halves.extend([first, second]);
    }
    Ok(CurvePath2::try_new(halves)?)
}

fn even_odd_region(
    paths: &[CurvePath2],
    policy: &CurveContext,
) -> Result<CurveRegion2, Box<dyn std::error::Error>> {
    Ok(crate::support::under(policy, || {
        CurveRegion2::try_from_boundary_paths(paths, FillRule::EvenOdd)
    })?
    .into_value())
}
