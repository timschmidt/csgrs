//! Exact minor-arc winding keeps selected point fields and native predicate paths.

use super::*;

impl BezierAlgebraicCuspSemicircleAlgebraicRay2 {
    pub(crate) fn endpoint_side_signs(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        self.endpoint_side_signs_for_point(
            &point.point_image().clone().into(),
            side_x,
            side_y,
            policy,
        )
    }
    pub(crate) fn contains_point(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        self.contains_point_for_point(&point.point_image().clone().into(), policy)
    }
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        self.forward_ray_winding_delta_skipping_incident_origin_for_point(
            &point.point_image().clone().into(),
            direction_x,
            direction_y,
            policy,
        )
    }
    pub(crate) fn forward_ray_winding_delta(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        point_on_supporting_circle: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.forward_ray_winding_delta_for_point(
            &point.point_image().clone().into(),
            direction_x,
            direction_y,
            point_on_supporting_circle,
            policy,
        )
    }
}

impl BezierAlgebraicCuspSemicircleAlgebraicRay2 {
    pub(crate) fn endpoint_side_signs_for_point(
        &self,
        point: &CurvePoint2,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        let mut signs = [RealSign::Zero; 2];
        for (index, endpoint) in [&self.start, &self.end].into_iter().enumerate() {
            signs[index] = match point_linear_difference(endpoint, point, side_x, side_y, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(signs))
    }

    pub(crate) fn contains_point_for_point(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let start = &self.start;
        let end = &self.end;
        let center = &self.center;
        match point_circle_residual(center, point, &self.radius_squared, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Decided(RealSign::Zero) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(point_oriented_side(start, end, point, policy)?.map(|side| {
            if self.clockwise {
                side != crate::classify::LineSide::Right
            } else {
                side != crate::classify::LineSide::Left
            }
        }))
    }

    /// Omits this finite arc's transverse contact at an algebraic side-ray
    /// origin while preserving any second forward circle contact.
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin_for_point(
        &self,
        point: &CurvePoint2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        match self.contains_point_for_point(point, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let center = &self.center;
        let radial_sign =
            match point_linear_difference(point, center, direction_x, direction_y, policy)? {
                Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        // The ordinary minor-arc predicate evaluates a point on the circle
        // from its outside limit.  An inward ray therefore includes the
        // origin crossing; an outward ray does not.  Remove exactly that
        // signed crossing while retaining a second forward contact when it
        // lies on this same finite semicircle.
        let skipped_origin_delta = if radial_sign == RealSign::Negative {
            if self.clockwise { 1 } else { -1 }
        } else {
            0
        };
        self.forward_ray_winding_delta_for_point(point, direction_x, direction_y, true, policy)
            .map(|delta| delta.map(|delta| Some(delta - skipped_origin_delta)))
    }

    pub(crate) fn forward_ray_winding_delta_for_point(
        &self,
        point: &CurvePoint2,
        direction_x: &Real,
        direction_y: &Real,
        point_on_supporting_circle: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let start = &self.start;
        let end = &self.end;
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let order =
            |first: &CurvePoint2, second: &CurvePoint2, x_factor: &Real, y_factor: &Real| {
                point_linear_order(first, second, x_factor, y_factor, policy)
            };
        let start_y = match order(start, point, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_y = match order(end, point, &side_x, &side_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let line_side = match point_oriented_side(start, end, point, policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let inside_circle = if point_on_supporting_circle {
            false
        } else {
            let center = &self.center;
            match point_circle_residual(center, point, &self.radius_squared, policy)? {
                Classification::Decided(sign) => sign == RealSign::Negative,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let is_ccw = !self.clockwise;
        let point_is_left = if is_ccw {
            line_side == crate::classify::LineSide::Left
        } else {
            line_side != crate::classify::LineSide::Right
        };
        let decision = crate::contour::minor_arc_winding_decision(
            start_y != std::cmp::Ordering::Greater,
            end_y == std::cmp::Ordering::Greater,
            point_is_left,
            inside_circle,
            is_ccw,
        );
        let (lower, upper, delta) = match decision {
            crate::contour::MinorArcWindingDecision::Delta(delta) => {
                return Ok(Classification::Decided(delta));
            }
            crate::contour::MinorArcWindingDecision::PointBetweenStartAndEnd(delta) => {
                (start, end, delta)
            }
            crate::contour::MinorArcWindingDecision::PointBetweenEndAndStart(delta) => {
                (end, start, delta)
            }
        };
        let lower_x = match order(lower, point, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if lower_x != std::cmp::Ordering::Less {
            return Ok(Classification::Decided(0));
        }
        let upper_x = match order(upper, point, direction_x, direction_y)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(
            if upper_x == std::cmp::Ordering::Greater {
                delta
            } else {
                0
            },
        ))
    }
}

fn point_linear_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    a: &Real,
    b: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    if let (CurvePointData2::Algebraic(first), CurvePointData2::Algebraic(second)) =
        (&first.0, &second.0)
    {
        let first = match first.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let second = match second.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        return algebraic_point_linear_order(&first, &second, a, b, policy);
    }
    match common_point_linear_order(first, second, a, b, policy)? {
        decided @ Classification::Decided(_) => Ok(decided),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted) => Ok(
            Classification::Uncertain(UncertaintyReason::BudgetExhausted),
        ),
        Classification::Uncertain(_) => {
            algebraic_chord_points_linear_order(first, second, a, b, policy)
        }
    }
}
fn point_linear_difference(
    first: &CurvePoint2,
    second: &CurvePoint2,
    a: &Real,
    b: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    Ok(
        point_linear_order(first, second, a, b, policy)?.map(|order| match order {
            std::cmp::Ordering::Less => RealSign::Negative,
            std::cmp::Ordering::Equal => RealSign::Zero,
            std::cmp::Ordering::Greater => RealSign::Positive,
        }),
    )
}
fn point_oriented_side(
    start: &CurvePoint2,
    end: &CurvePoint2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    if let (
        CurvePointData2::Algebraic(start),
        CurvePointData2::Algebraic(end),
        CurvePointData2::Algebraic(point),
    ) = (&start.0, &end.0, &point.0)
    {
        let start = match start.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match end.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        return algebraic_point_oriented_line_side(&start, &end, &point, policy);
    }
    common_point_oriented_side(start, end, point, policy)
}
fn point_circle_residual(
    center: &CurvePoint2,
    point: &CurvePoint2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if let (CurvePointData2::Algebraic(center), CurvePointData2::Algebraic(point)) =
        (&center.0, &point.0)
    {
        let center = match center.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        return algebraic_point_circle_residual_sign(&center, &point, radius_squared, policy);
    }
    common_point_circle_residual_sign(point, center, radius_squared, policy)
}

impl BezierAlgebraicCuspSemicircleFragment2 {
    pub(super) fn point_evidence_ray_evaluator(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleAlgebraicRay2>> {
        self.validate_policy(policy)?;
        let center = match self.semicircle().center_point_evidence(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut endpoints = Vec::with_capacity(2);
        for parameter in [&self.data.start, &self.data.end] {
            match parameter.coincident_point_evidence(self.semicircle(), policy)? {
                Classification::Decided(Some(point)) => endpoints.push(point),
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        if self.is_reversed() {
            endpoints.reverse();
        }
        let end = endpoints.pop().expect("two endpoints");
        let start = endpoints.pop().expect("two endpoints");
        Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleAlgebraicRay2 {
                start,
                end,
                center,
                radius_squared: self.semicircle().radial_distance()
                    * self.semicircle().radial_distance(),
                clockwise: self.semicircle().is_clockwise() ^ self.is_reversed(),
            },
        ))
    }
}
