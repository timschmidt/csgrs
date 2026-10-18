//! Lossless line/arc views shared by retained integration and native promotion.

use super::*;

pub(crate) fn retained_native_fragment_segment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            materialized_native_subcurve_segment(curve, policy)
        }
        BezierSplitFragment2::RetainedBezier {
            reversed,
            start,
            end,
            source_curve,
            start_image,
            end_image,
        } => {
            let source = match materialized_native_subcurve_segment(source_curve, policy)? {
                Classification::Decided(source) => source,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            // The source's circle and traversal certificate covers its unit
            // chart. Exterior continuations need separate traversal evidence.
            if matches!(source, Segment2::Arc(_)) {
                for (lower, upper) in [
                    (&BezierParameter2::Exact(Real::zero()), start),
                    (end, &BezierParameter2::Exact(Real::one())),
                ] {
                    match lower.cmp_by_refinement_with_policy(upper, policy)? {
                        Classification::Decided(
                            std::cmp::Ordering::Less | std::cmp::Ordering::Equal,
                        ) => {}
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            // Endpoint evaluation below reuses the selected root and source
            // image; it never samples or replaces the retained source chart.
            let endpoint = |parameter: &BezierParameter2,
                            image: Option<&BezierAlgebraicEndpointImage2>|
             -> CurveResult<Classification<Point2>> {
                if let Some(image) = image {
                    match image.point()? {
                        Classification::Decided(point) => match point.exact_native_point(policy)? {
                            Classification::Decided(Some(point)) => {
                                return Ok(Classification::Decided(point));
                            }
                            Classification::Decided(None) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        },
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                if matches!(source, Segment2::Arc(_))
                    && let BezierParameter2::Algebraic(parameter) = parameter
                {
                    match parameter.represented_quadratic_point_with_policy(policy)? {
                        Classification::Decided(Some(value)) => {
                            return Ok(subcurve_point_at(source_curve, value, policy));
                        }
                        Classification::Decided(None) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(retained_bezier_endpoint_point(
                    parameter,
                    image,
                    source_curve,
                    policy,
                ))
            };
            let start_point = match endpoint(start, start_image.as_ref())? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let end_point = match endpoint(end, end_image.as_ref())? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let (start_point, end_point) = if *reversed {
                (end_point, start_point)
            } else {
                (start_point, end_point)
            };
            Ok(Classification::Decided(match source {
                Segment2::Line(_) => Segment2::Line(LineSeg2::try_new(start_point, end_point)?),
                Segment2::Arc(arc) => Segment2::Arc(CircularArc2::new_with_certified_radius(
                    start_point,
                    end_point,
                    arc.center().clone(),
                    arc.radius_squared_ref().clone(),
                    arc.is_clockwise() ^ *reversed,
                    None,
                )),
            }))
        }
        BezierSplitFragment2::AlgebraicChord(chord) => {
            if let Some(line) = chord.exact_line() {
                return Ok(Classification::Decided(Segment2::Line(line)));
            }
            let start = match native_endpoint(chord.start(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let end = match native_endpoint(chord.end(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            Ok(Classification::Decided(Segment2::Line(LineSeg2::try_new(
                start, end,
            )?)))
        }
        _ => retained_line_fragment_segment(fragment, policy).map(|line| line.map(Segment2::Line)),
    }
}

fn native_endpoint(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<Point2>> {
    match point {
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            Ok(match point.exact_native_point(policy)? {
                Classification::Decided(Some(point)) => Classification::Decided(point),
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            })
        }
        CurvePoint2(CurvePointData2::Endpoint(endpoint)) => match endpoint.resolve(policy)? {
            Classification::Decided(Some(point)) => native_endpoint(&point, policy),
            Classification::Decided(None) => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        },
        _ => retained_native_line_point(point, policy),
    }
}

pub(crate) fn materialized_native_subcurve_segment(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    if let BezierSubcurve2::Quadratic(curve) = curve
        && let Some(line) = curve.retained_exact_line_image()
    {
        return Ok(Classification::Decided(Segment2::Line(line.clone())));
    }
    if let BezierSubcurve2::RationalQuadratic(curve) = curve
        && curve.retained_circular_conic().is_some()
    {
        return crate::arc_bezier::rational_quadratic_circular_arc(curve, policy).map(
            |arc| match arc {
                Classification::Decided(Some(arc)) => Classification::Decided(Segment2::Arc(arc)),
                Classification::Decided(None) => {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    let line_blocker = match subcurve_fit_exact_line_image(curve, policy)? {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => {
            return Ok(Classification::Decided(Segment2::Line(fit.line().clone())));
        }
        Classification::Decided(BezierLineImageFitRelation::NotLine) => {
            UncertaintyReason::Unsupported
        }
        // A line recognizer can decline zero or mixed-sign homogeneous
        // weights even when an independent circle certificate succeeds.
        Classification::Uncertain(reason) => reason,
    };
    let arc = match curve {
        BezierSubcurve2::RationalQuadratic(curve) => {
            crate::arc_bezier::rational_quadratic_circular_arc(curve, policy)
        }
        BezierSubcurve2::Rational(curve) => {
            crate::arc_bezier::rational_bezier_circular_arc(curve, policy)
        }
        BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => {
            return Ok(Classification::Uncertain(line_blocker));
        }
    }?;
    Ok(match arc {
        Classification::Decided(Some(arc)) => Classification::Decided(Segment2::Arc(arc)),
        Classification::Decided(None) => Classification::Uncertain(line_blocker),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}
