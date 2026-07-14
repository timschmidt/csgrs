use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementFragment2,
    BezierArrangementGraph2, BezierBoundaryLoop2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, BezierRegion2, BezierRetainedCurveEnvelope2,
    BezierRetainedEndpointEnvelope2, BezierSplitFragment2, BezierSubcurve2, BooleanOp,
    Classification, Curve2, CurveError, CurvePath2, CurvePolicy, CurveRegion2,
    CurveRegionBoundaryLoop2, CurveResult, LineSeg2, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real,
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

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn line_fragment(
    source_curve_index: usize,
    start: Point2,
    control: Point2,
    end: Point2,
) -> BezierArrangementFragment2 {
    BezierArrangementFragment2::new(
        source_curve_index,
        0,
        BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(r(0)),
            end: BezierParameter2::Exact(r(1)),
            curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, control, end)),
        },
    )
}

fn retained_loop(fragments: Vec<BezierSplitFragment2>) -> CurveResult<CurveRegionBoundaryLoop2> {
    CurveRegionBoundaryLoop2::new(fragments)
}

fn square_region(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> CurveResult<CurveRegion2> {
    let points = [
        p(min_x, min_y),
        p(max_x, min_y),
        p(max_x, max_y),
        p(min_x, max_y),
    ];
    let path = CurvePath2::try_new(
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
    })?;
    CurveRegion2::try_from_boundary_paths(&[path]).map_err(|error| match error {
        hypercurve::ExactCurveError::Invalid { cause, .. } => cause,
        hypercurve::ExactCurveError::Blocked(blocker) => CurveError::Topology(format!(
            "square benchmark region blocked: {:?}",
            blocker.reason()
        )),
    })
}

fn algebraic_polynomial_parameter(
    coefficients: Vec<Real>,
    interval_start: Real,
    interval_end: Real,
    policy: &CurvePolicy,
) -> CurveResult<BezierParameter2> {
    let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        coefficients,
        policy,
    )?);
    let interval = decided(BezierParameterInterval::try_new(
        interval_start,
        interval_end,
        policy,
    )?);
    Ok(BezierParameter2::algebraic(decided(
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy)?,
    )))
}

fn algebraic_midpoint_root(policy: &CurvePolicy) -> CurveResult<BezierAlgebraicParameter2> {
    let polynomial = decided(BezierParameterPolynomial::try_new_power_basis(
        vec![r(-1), r(2)],
        policy,
    )?);
    let interval = decided(BezierParameterInterval::try_new(q(2, 5), q(3, 5), policy)?);
    Ok(decided(BezierAlgebraicParameter2::try_isolate(
        polynomial, interval, policy,
    )?))
}

fn algebraic_constant_point_image(
    point: Point2,
    policy: &CurvePolicy,
) -> CurveResult<BezierAlgebraicEndpointImage2> {
    let curve = QuadraticBezier2::new(point.clone(), point.clone(), point);
    BezierAlgebraicEndpointImage2::quadratic(&curve, &algebraic_midpoint_root(policy)?, policy)
}

fn retained_algebraic_line_fragment(
    start: Point2,
    end: Point2,
    policy: &CurvePolicy,
) -> CurveResult<BezierSplitFragment2> {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_root(policy)?);
    Ok(BezierSplitFragment2::AlgebraicEndpointImages {
        reversed: false,
        start: parameter.clone(),
        end: parameter,
        source_curve: None,
        start_image: Some(algebraic_constant_point_image(start, policy)?),
        end_image: Some(algebraic_constant_point_image(end, policy)?),
    })
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let first_region = square_region(0, 0, 4, 4)?;
    let second_region = square_region(2, 0, 6, 4)?;
    let region_boolean_iterations = 1_000_u32;
    let started = Instant::now();
    let mut region_boolean_checksum = 0_usize;
    for _ in 0..region_boolean_iterations {
        let prepared = first_region
            .try_prepare_boolean(&second_region, &policy)
            .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?;
        let region = prepared
            .boolean_region(BooleanOp::Union)
            .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?;
        region_boolean_checksum ^= black_box(region.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_boolean_prepare_and_union: {region_boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={region_boolean_checksum}",
        elapsed / region_boolean_iterations
    );

    let prepared_region_boolean = first_region
        .try_prepare_boolean(&second_region, &policy)
        .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?;
    prepared_region_boolean
        .boolean_region_view(BooleanOp::Union)
        .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?;
    let started = Instant::now();
    let mut cached_region_boolean_checksum = 0_usize;
    let cached_region_boolean_iterations = 2_000_u32;
    for _ in 0..cached_region_boolean_iterations {
        cached_region_boolean_checksum ^= black_box(
            prepared_region_boolean
                .boolean_region_view(BooleanOp::Union)
                .map_err(|error| CurveError::Topology(format!("region benchmark: {error}")))?
                .boundary_loops()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_boolean_cached_union: {cached_region_boolean_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cached_region_boolean_checksum}",
        elapsed / cached_region_boolean_iterations
    );

    let algebraic_ray_loop = BezierBoundaryLoop2::new(vec![
        BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), r(0)),
            p(1, 1),
        )),
        BezierSubcurve2::Quadratic(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(1, 2), q(1, 2)),
            p(0, 0),
        )),
    ])?;
    let algebraic_ray_query = Point2::new(q(1, 2), q(3, 8));
    let classification_iterations = 2_000_u32;
    let started = Instant::now();
    let mut classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location =
            decided(algebraic_ray_loop.classify_point(black_box(&algebraic_ray_query), &policy)?);
        classification_checksum =
            classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_region_algebraic_ray_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={classification_checksum}",
        elapsed / classification_iterations
    );

    let half = decided(BezierParameter2::exact(q(1, 2), &policy)?);
    let upper = QuadraticBezier2::new(p(0, 0), p(2, 4), p(4, 0));
    let lower = QuadraticBezier2::new(p(4, 0), p(2, -4), p(0, 0));
    let graph = BezierArrangementGraph2::from_split_materializations(&[
        decided(upper.split_at_parameters(std::slice::from_ref(&half), &policy)?),
        decided(lower.split_at_parameters(std::slice::from_ref(&half), &policy)?),
    ])?;
    let traversal = decided(graph.traverse_branch_free(&policy));

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let region = decided(BezierRegion2::from_arrangement_traversal(
            &graph, &traversal,
        ));
        checksum ^= black_box(format!("{:?}", region.signed_area()?).len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let retained_traversal = decided(graph.traverse_retained_with_tangent_order(&policy));
    let classified_region = decided(CurveRegion2::from_retained_arrangement_traversal(
        &graph,
        &retained_traversal,
    ));
    let classified_point = p(2, 0);
    decided(classified_region.classify_point(&classified_point, &policy)?);
    let started = Instant::now();
    let mut curved_classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = decided(
            classified_region.classify_point(black_box(&classified_point), black_box(&policy))?,
        );
        curved_classification_checksum =
            curved_classification_checksum.wrapping_add(black_box(location as usize));
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_cached_classification: {classification_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={curved_classification_checksum}",
        elapsed / classification_iterations
    );

    let started = Instant::now();
    let mut retained_checksum = 0_usize;
    for _ in 0..iterations {
        let region = decided(CurveRegion2::from_retained_arrangement_traversal(
            &graph,
            &retained_traversal,
        ));
        retained_checksum ^= black_box(format!("{:?}", region.signed_area()?).len());
        if let Classification::Decided(envelope) =
            BezierRetainedEndpointEnvelope2::from_region(&region, &policy)
        {
            retained_checksum ^= black_box(format!("{:?}", envelope.envelope()).len());
        }
        if let Classification::Decided(envelope) =
            BezierRetainedCurveEnvelope2::from_region(&region, &policy)
        {
            retained_checksum ^= black_box(format!("{:?}", envelope.envelope()).len());
        }
        if let Classification::Decided(report) = region.line_image_role_report(&policy)? {
            retained_checksum ^= black_box(report.roles().len());
        }
        if let Classification::Decided(report) = region.signed_area_role_report(&policy)? {
            retained_checksum ^= black_box(report.roles().len() + report.signed_areas().len());
        }
        if let Classification::Decided(report) = region.curved_nesting_role_report(&policy)? {
            retained_checksum ^= black_box(report.roles().len() + report.sample_points().len());
        }
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_region_materialization: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_checksum}",
        elapsed / iterations
    );

    let algebraic_split = decided(upper.split_at_parameters(
        &[algebraic_polynomial_parameter(
            vec![r(-1), r(0), r(2)],
            q(2, 3),
            q(3, 4),
            &policy,
        )?],
        &policy,
    )?);
    let mut algebraic_loop_fragments = algebraic_split.fragments().to_vec();
    algebraic_loop_fragments.extend(
        algebraic_split
            .fragments()
            .iter()
            .rev()
            .map(BezierSplitFragment2::reversed)
            .collect::<CurveResult<Vec<_>>>()?,
    );
    let algebraic_loop = retained_loop(algebraic_loop_fragments)?;
    let mut algebraic_region_fragments = algebraic_split.fragments().to_vec();
    algebraic_region_fragments.push(BezierSplitFragment2::Materialized {
        start: BezierParameter2::Exact(Real::zero()),
        end: BezierParameter2::Exact(Real::one()),
        curve: BezierSubcurve2::Quadratic(lower),
    });
    let algebraic_region = CurveRegion2::new(vec![retained_loop(algebraic_region_fragments)?])?;
    let algebraic_region_query = p(2, 0);
    decided(algebraic_region.classify_point(&algebraic_region_query, &policy)?);
    let started = Instant::now();
    let mut algebraic_classification_checksum = 0_usize;
    for _ in 0..classification_iterations {
        let location = decided(
            algebraic_region
                .classify_point(black_box(&algebraic_region_query), black_box(&policy))?,
        );
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
        let envelope = decided(BezierRetainedCurveEnvelope2::from_loop(
            &algebraic_loop,
            &policy,
        ));
        algebraic_envelope_checksum ^=
            black_box(format!("{:?}", envelope.envelope()).len() + envelope.exact_fragment_count());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_algebraic_source_envelope: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_envelope_checksum}",
        elapsed / iterations
    );

    let exact_endpoint_algebraic_curve = QuadraticBezier2::new(p(0, 0), p(0, 0), p(8, 0));
    let exact_endpoint_split = decided(exact_endpoint_algebraic_curve.split_at_parameters(
        &[algebraic_polynomial_parameter(
            vec![r(-1), r(0), r(8)],
            q(1, 3),
            q(2, 5),
            &policy,
        )?],
        &policy,
    )?);
    let exact_endpoint_fragment = exact_endpoint_split.fragments()[0].clone();
    let exact_endpoint_loop = retained_loop(vec![
        exact_endpoint_fragment.clone(),
        exact_endpoint_fragment.reversed()?,
    ])?;
    let started = Instant::now();
    let mut algebraic_exact_endpoint_checksum = 0_usize;
    for _ in 0..iterations {
        let envelope = decided(BezierRetainedCurveEnvelope2::from_loop(
            &exact_endpoint_loop,
            &policy,
        ));
        algebraic_exact_endpoint_checksum ^=
            black_box(format!("{:?}", envelope.envelope()).len() + envelope.exact_fragment_count());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_algebraic_exact_endpoint_envelope: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_exact_endpoint_checksum}",
        elapsed / iterations
    );

    let algebraic_line_region = CurveRegion2::new(vec![
        retained_loop(vec![
            retained_algebraic_line_fragment(p(-3, -3), p(3, -3), &policy)?,
            retained_algebraic_line_fragment(p(3, -3), p(3, 3), &policy)?,
            retained_algebraic_line_fragment(p(3, 3), p(-3, 3), &policy)?,
            retained_algebraic_line_fragment(p(-3, 3), p(-3, -3), &policy)?,
        ])?,
        retained_loop(vec![
            retained_algebraic_line_fragment(p(-1, -1), p(1, -1), &policy)?,
            retained_algebraic_line_fragment(p(1, -1), p(1, 1), &policy)?,
            retained_algebraic_line_fragment(p(1, 1), p(-1, 1), &policy)?,
            retained_algebraic_line_fragment(p(-1, 1), p(-1, -1), &policy)?,
        ])?,
    ])?;
    let started = Instant::now();
    let mut algebraic_line_role_checksum = 0_usize;
    for _ in 0..iterations {
        if let Classification::Decided(report) =
            algebraic_line_region.line_image_role_report(&policy)?
        {
            algebraic_line_role_checksum ^= black_box(
                report.roles().len()
                    + report.material_loop_indices().len()
                    + report.hole_loop_indices().len(),
            );
            algebraic_line_role_checksum ^=
                black_box(format!("{:?}", report.to_region().filled_area(&policy)?).len());
        }
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_retained_algebraic_line_role_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={algebraic_line_role_checksum}",
        elapsed / iterations
    );

    let overlap_graph = BezierArrangementGraph2::new(vec![
        line_fragment(0, p(0, 0), p(2, 0), p(4, 0)),
        line_fragment(1, p(2, 0), p(3, 0), p(4, 0)),
        line_fragment(2, p(4, 0), p(4, 1), p(4, 2)),
        line_fragment(3, p(4, 2), p(2, 2), p(0, 2)),
        line_fragment(4, p(0, 2), p(0, 1), p(0, 0)),
    ])?;
    let overlap_traversal =
        decided(overlap_graph.traverse_retained_splitting_linear_overlaps(&policy));
    let started = Instant::now();
    let mut overlap_checksum = 0_usize;
    for _ in 0..iterations {
        let native = decided(BezierRegion2::from_retained_linear_overlap_traversal(
            &overlap_traversal,
        ));
        overlap_checksum ^= black_box(format!("{:?}", native.signed_area()?).len());
        let retained = decided(CurveRegion2::from_retained_linear_overlap_traversal(
            &overlap_traversal,
        ));
        overlap_checksum ^= black_box(format!("{:?}", retained.signed_area()?).len());
        if let Classification::Decided(report) = retained.line_image_role_report(&policy)? {
            overlap_checksum ^= black_box(usize::from(
                report.to_region().filled_area(&policy)?.is_decided(),
            ));
        }
        if let Classification::Decided(report) = retained.signed_area_role_report(&policy)? {
            overlap_checksum ^= black_box(report.roles().len());
        }
        if let Classification::Decided(report) = retained.curved_nesting_role_report(&policy)? {
            overlap_checksum ^= black_box(report.roles().len() + report.sample_points().len());
        }
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
    let conic_graph = BezierArrangementGraph2::from_split_materializations(&[
        decided(conic_upper.split_at_parameters(std::slice::from_ref(&half), &policy)?),
        decided(conic_lower.split_at_parameters(std::slice::from_ref(&half), &policy)?),
    ])?;
    let conic_traversal = decided(conic_graph.traverse_branch_free(&policy));
    let started = Instant::now();
    let mut conic_checksum = 0_usize;
    for _ in 0..iterations {
        let region = decided(BezierRegion2::from_arrangement_traversal(
            &conic_graph,
            &conic_traversal,
        ));
        conic_checksum ^= black_box(format!("{:?}", region.signed_area()?).len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_conic_region_exact_area: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={conic_checksum}",
        elapsed / iterations
    );

    Ok(())
}
