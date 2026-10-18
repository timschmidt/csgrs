//! Affine forms reuse retained point fields before Cartesian elimination.

use super::*;

pub(crate) fn point_affine_form_order_to_real(
    point: &CurvePoint2,
    a: &Real,
    b: &Real,
    value: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    use std::cmp::Ordering;
    let order = |sign| match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    };
    if a.zero_status() == ZeroKnowledge::Zero && b.zero_status() == ZeroKnowledge::Zero {
        return Ok(compare_reals(&Real::zero(), value, policy).map_or(
            Classification::Uncertain(UncertaintyReason::Ordering),
            Classification::Decided,
        ));
    }
    match &point.0 {
        CurvePointData2::Exact(point) => Ok(compare_reals(
            &Real::dot2_refs([a, b], [point.x(), point.y()]),
            value,
            policy,
        )
        .map_or(
            Classification::Uncertain(UncertaintyReason::Ordering),
            Classification::Decided,
        )),
        CurvePointData2::Algebraic(point) => match point.predicate_evaluator(policy)? {
            Classification::Decided(predicate) => {
                predicate.linear_order_to_real(a, b, value, policy)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        CurvePointData2::AnalyticParallel(point) => {
            Ok(point.linear_residual_sign(a, b, value, policy)?.map(order))
        }
        CurvePointData2::AlgebraicCuspChord(point) => {
            point.linear_order_to_real(a, b, value, policy)
        }
        CurvePointData2::Endpoint(endpoint) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => {
                point_affine_form_order_to_real(&point, a, b, value, policy)
            }
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        CurvePointData2::Similarity(image) => {
            if !policy.accepts_retained_policy(image.data.policy) {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            let (m00, m01, m10, m11, tx, ty) = image.data.transform.affine_components();
            point_affine_form_order_to_real(
                &image.data.source,
                &Real::dot2_refs([a, b], [m00, m10]),
                &Real::dot2_refs([a, b], [m01, m11]),
                &(value - Real::dot2_refs([a, b], [tx, ty])),
                policy,
            )
        }
        _ => {
            // Existing correlated chord predicates already own these families.
            // A finite point on the requested level line supplies their origin.
            let strict = policy.strict_counterpart();
            let origin = if matches!(
                real_sign(a, &strict),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                Point2::new((value / a)?, Real::zero())
            } else if matches!(
                real_sign(b, &strict),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                Point2::new(Real::zero(), (value / b)?)
            } else {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            };
            algebraic_chord_point_linear_order_to_exact(point, &origin, a, b, policy)
        }
    }
}

pub(crate) fn point_affine_form_image(
    source: &CurvePoint2,
    [a, b, c]: &[Real; 3],
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    let strict = policy.strict_counterpart();
    match real_sign(&Real::dot2_refs([a, b], [a, b]), &strict) {
        Some(RealSign::Zero) => Ok(Classification::Decided(
            Point2::new(c.clone(), Real::zero()).into(),
        )),
        Some(RealSign::Positive) => {
            // Completing one nonzero row to a similarity does not reconstruct
            // either coordinate or change the point's original source field.
            let transform = Similarity2::try_from_real_affine(
                a.clone(),
                b.clone(),
                -b,
                a.clone(),
                c.clone(),
                Real::zero(),
            )?;
            Ok(Classification::Decided(
                BezierSimilarityPoint2::new(source.clone(), transform, &strict).into(),
            ))
        }
        _ => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

pub(crate) fn point_affine_form_bounds(
    source: &CurvePoint2,
    [a, b, c]: &[Real; 3],
    policy: &CurveContext,
) -> CurveResult<Classification<(Real, Real)>> {
    if a.zero_status() == ZeroKnowledge::Zero && b.zero_status() == ZeroKnowledge::Zero {
        return Ok(Classification::Decided((c.clone(), c.clone())));
    }
    let bounds = match algebraic_chord_endpoint_bounds_refined(source, 0, policy) {
        Classification::Decided(bounds) => bounds,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let exact =
        |value: &Real| RealInterval::from_values([value.clone()]).expect("one scalar interval");
    let interval = real_interval_from_axis(&bounds, Axis2::X)
        .multiply(&exact(a))
        .and_then(|x| {
            real_interval_from_axis(&bounds, Axis2::Y)
                .multiply(&exact(b))
                .map(|y| x.add(&y).add(&exact(c)))
        });
    Ok(interval.map_or(
        Classification::Uncertain(UncertaintyReason::Ordering),
        |interval| Classification::Decided((interval.lower, interval.upper)),
    ))
}
