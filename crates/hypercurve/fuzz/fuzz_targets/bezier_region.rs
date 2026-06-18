#![no_main]

use hypercurve::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierArrangementGraph2,
    BezierParameter2, BezierParameterInterval, BezierParameterPolynomial, BezierRegion2,
    BezierRetainedBoundaryLoop2, BezierRetainedCurveEnvelope2, BezierRetainedEndpointEnvelope2,
    BezierRetainedRegion2, BezierSplitFragment2, Classification, CurvePolicy, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fn algebraic_sqrt_half(policy: &CurvePolicy) -> Option<BezierParameter2> {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i32), Real::from(0_i32), Real::from(2_i32)],
        policy,
    )
    .ok()?
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(_) => return None,
    };
    let interval =
        match BezierParameterInterval::try_new(rational(2, 3), rational(3, 4), policy).ok()? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(_) => return None,
        };
    let parameter =
        match BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).ok()? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(_) => return None,
        };
    Some(BezierParameter2::algebraic(parameter))
}

fn algebraic_sqrt_eighth(policy: &CurvePolicy) -> Option<BezierParameter2> {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i32), Real::from(0_i32), Real::from(8_i32)],
        policy,
    )
    .ok()?
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(_) => return None,
    };
    let interval =
        match BezierParameterInterval::try_new(rational(1, 3), rational(2, 5), policy).ok()? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(_) => return None,
        };
    let parameter =
        match BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).ok()? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(_) => return None,
        };
    Some(BezierParameter2::algebraic(parameter))
}

fn algebraic_midpoint_root(policy: &CurvePolicy) -> Option<BezierAlgebraicParameter2> {
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        vec![Real::from(-1_i32), Real::from(2_i32)],
        policy,
    )
    .ok()?
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(_) => return None,
    };
    let interval =
        match BezierParameterInterval::try_new(rational(2, 5), rational(3, 5), policy).ok()? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(_) => return None,
        };
    match BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).ok()? {
        Classification::Decided(parameter) => Some(parameter),
        Classification::Uncertain(_) => None,
    }
}

fn constant_point_image(
    point: Point2,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicEndpointImage2> {
    let curve = QuadraticBezier2::new(point.clone(), point.clone(), point);
    BezierAlgebraicEndpointImage2::quadratic(&curve, &algebraic_midpoint_root(policy)?, policy).ok()
}

fn algebraic_line_fragment(
    start: Point2,
    end: Point2,
    policy: &CurvePolicy,
) -> Option<BezierSplitFragment2> {
    let parameter = BezierParameter2::algebraic(algebraic_midpoint_root(policy)?);
    Some(BezierSplitFragment2::AlgebraicEndpointImages {
        start: parameter.clone(),
        end: parameter,
        source_curve: None,
        start_image: Some(constant_point_image(start, policy)?),
        end_image: Some(constant_point_image(end, policy)?),
    })
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    let policy = CurvePolicy::certified();
    let mut materializations = Vec::new();
    for chunk in data.chunks(8).take(8) {
        if chunk.len() < 8 {
            break;
        }
        let curve = QuadraticBezier2::new(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
        );
        let mut parameters = Vec::new();
        if let Ok(Classification::Decided(parameter)) =
            BezierParameter2::exact(unit_from_byte(chunk[6]), &policy)
        {
            parameters.push(parameter);
        }
        if let Ok(Classification::Decided(materialization)) =
            curve.split_at_parameters(&parameters, &policy)
        {
            materializations.push(materialization);
        }
        if let Some(algebraic) = algebraic_sqrt_half(&policy)
            && let Ok(Classification::Decided(split)) =
                curve.split_at_parameters(&[algebraic], &policy)
            && let Some(fragment) = split.fragments().first()
        {
            if let Ok(loop_) = BezierRetainedBoundaryLoop2::new(vec![fragment.clone()]) {
                let _ = BezierRetainedCurveEnvelope2::from_loop(&loop_, &policy);
            }
        }
        if let Some(algebraic) = algebraic_sqrt_eighth(&policy)
            && let Ok(Classification::Decided(split)) =
                curve.split_at_parameters(&[algebraic], &policy)
            && let Some(fragment) = split.fragments().first()
        {
            if let Ok(loop_) = BezierRetainedBoundaryLoop2::new(vec![fragment.clone()]) {
                let _ = BezierRetainedCurveEnvelope2::from_loop(&loop_, &policy);
            }
        }
    }

    if let Ok(graph) = BezierArrangementGraph2::from_split_materializations(&materializations) {
        if let Classification::Decided(traversal) = graph.traverse_branch_free(&policy) {
            let _ = BezierRegion2::from_arrangement_traversal(&graph, &traversal)
                .map(|region| region.signed_area());
        }
        if let Classification::Decided(traversal) =
            graph.traverse_retained_with_tangent_order(&policy)
        {
            let _ = BezierRetainedRegion2::from_retained_arrangement_traversal(&graph, &traversal)
                .map(|region| {
                    let _ = region.signed_area();
                    let _ = region.line_image_role_report(&policy);
                    let _ = region.signed_area_role_report(&policy);
                    let _ = region.curved_nesting_role_report(&policy);
                    let _ = BezierRetainedEndpointEnvelope2::from_region(&region, &policy);
                    let _ = BezierRetainedCurveEnvelope2::from_region(&region, &policy);
                });
        }
        if let Classification::Decided(traversal) =
            graph.traverse_retained_splitting_linear_overlaps(&policy)
        {
            let _ = BezierRegion2::from_retained_linear_overlap_traversal(&traversal)
                .map(|region| region.signed_area());
            let _ = BezierRetainedRegion2::from_retained_linear_overlap_traversal(&traversal).map(
                |region| {
                    let _ = region.signed_area();
                    let _ = region.line_image_role_report(&policy);
                    let _ = region.signed_area_role_report(&policy);
                    let _ = region.curved_nesting_role_report(&policy);
                    let _ = BezierRetainedEndpointEnvelope2::from_region(&region, &policy);
                    let _ = BezierRetainedCurveEnvelope2::from_region(&region, &policy);
                },
            );
        }
    }

    let algebraic_outer = [
        (
            Point2::new(rational(-3, 1), rational(-3, 1)),
            Point2::new(rational(3, 1), rational(-3, 1)),
        ),
        (
            Point2::new(rational(3, 1), rational(-3, 1)),
            Point2::new(rational(3, 1), rational(3, 1)),
        ),
        (
            Point2::new(rational(3, 1), rational(3, 1)),
            Point2::new(rational(-3, 1), rational(3, 1)),
        ),
        (
            Point2::new(rational(-3, 1), rational(3, 1)),
            Point2::new(rational(-3, 1), rational(-3, 1)),
        ),
    ];
    let algebraic_inner = [
        (
            Point2::new(rational(-1, 1), rational(-1, 1)),
            Point2::new(rational(1, 1), rational(-1, 1)),
        ),
        (
            Point2::new(rational(1, 1), rational(-1, 1)),
            Point2::new(rational(1, 1), rational(1, 1)),
        ),
        (
            Point2::new(rational(1, 1), rational(1, 1)),
            Point2::new(rational(-1, 1), rational(1, 1)),
        ),
        (
            Point2::new(rational(-1, 1), rational(1, 1)),
            Point2::new(rational(-1, 1), rational(-1, 1)),
        ),
    ];
    let outer = algebraic_outer
        .into_iter()
        .filter_map(|(start, end)| algebraic_line_fragment(start, end, &policy))
        .collect::<Vec<_>>();
    let inner = algebraic_inner
        .into_iter()
        .filter_map(|(start, end)| algebraic_line_fragment(start, end, &policy))
        .collect::<Vec<_>>();
    if outer.len() == 4 && inner.len() == 4 {
        if let (Ok(outer), Ok(inner)) = (
            BezierRetainedBoundaryLoop2::new(outer),
            BezierRetainedBoundaryLoop2::new(inner),
        ) {
            if let Ok(region) = BezierRetainedRegion2::new(vec![outer, inner]) {
                let _ = region.line_image_role_report(&policy);
            }
        }
    }

    for chunk in data.chunks(9).take(4) {
        if chunk.len() < 9 {
            break;
        }
        let weight = Real::from((chunk[8] % 31) as i32 + 1);
        if let Ok(conic) = RationalQuadraticBezier2::try_unit_end_weights(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
            weight,
        ) {
            let _ = conic.signed_area_contribution();
        }
    }
});
