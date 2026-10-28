//! Retained point queries reuse line and minor-arc winding and region ownership.

use super::*;
use crate::bezier_offset::{BezierAlgebraicChord2, algebraic_chord_points_linear_order};
use std::cmp::Ordering;

fn prepare_point_ray_fragments(
    boundary: &CurveRegionBoundaryLoop2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<AlgebraicRayRetainedFragment2>>> {
    let mut result = Vec::with_capacity(boundary.fragments().len());
    for fragment in boundary.fragments() {
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(circle) = fragment {
            match circle.algebraic_ray_evaluator(policy)? {
                Classification::Decided(ray) => {
                    result.push(AlgebraicRayRetainedFragment2::AlgebraicCusp(ray))
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            continue;
        }
        let chord = match fragment {
            BezierSplitFragment2::AlgebraicChord(chord) => chord.clone(),
            _ => {
                let line = match retained_line_fragment_segment(fragment, policy)? {
                    Classification::Decided(line) => line,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match BezierAlgebraicChord2::try_new(
                    line.start().clone().into(),
                    line.end().clone().into(),
                    policy,
                )? {
                    Classification::Decided(chord) => chord,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        chord.validate_policy(policy)?;
        result.push(AlgebraicRayRetainedFragment2::AlgebraicChord(chord));
    }
    Ok(Classification::Decided(result))
}

fn prepared_point_ray_winding(
    fragments: &[AlgebraicRayRetainedFragment2],
    point: &CurvePoint2,
    direction_x: &Real,
    direction_y: &Real,
    skip_incident_origin: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedRayWinding>> {
    // A direction can be discarded for an endpoint contact or an undecided
    // predicate. Do not consume an approximate terminal before another ray
    // (or the existing probe) has a chance to establish an exact answer.
    // Keep the caller's replay policy for qualified source dependencies.
    policy.strict_predicate_pass(|| {
        let mut incident = Vec::with_capacity(fragments.len());
        for fragment in fragments {
            let contains = match fragment {
                AlgebraicRayRetainedFragment2::AlgebraicCusp(circle) => {
                    circle.contains_point_for_point(point, policy)?
                }
                AlgebraicRayRetainedFragment2::AlgebraicChord(chord) => {
                    chord.contains_point(point, policy)?
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            };
            match contains {
                Classification::Decided(true) if !skip_incident_origin => {
                    return Ok(Classification::Decided(RetainedRayWinding::Boundary));
                }
                Classification::Decided(contains) => incident.push(contains),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let side_x = -direction_y;
        let side_y = direction_x;
        let mut winding = 0_i32;
        for (fragment, incident) in fragments.iter().zip(incident) {
            let delta = match fragment {
                AlgebraicRayRetainedFragment2::AlgebraicCusp(circle) => {
                    let sides = match circle
                        .endpoint_side_signs_for_point(point, &side_x, side_y, policy)?
                    {
                        Classification::Decided(sides) => sides,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    if sides.contains(&RealSign::Zero) {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    if incident {
                        match circle.forward_ray_winding_delta_skipping_incident_origin_for_point(
                            point,
                            direction_x,
                            direction_y,
                            policy,
                        )? {
                            Classification::Decided(Some(delta)) => Classification::Decided(delta),
                            Classification::Decided(None) => {
                                return Err(CurveError::Topology(
                                    "a retained arc lost its certified ray-origin incidence".into(),
                                ));
                            }
                            Classification::Uncertain(reason) => Classification::Uncertain(reason),
                        }
                    } else {
                        circle.forward_ray_winding_delta_for_point(
                            point,
                            direction_x,
                            direction_y,
                            false,
                            policy,
                        )?
                    }
                }
                AlgebraicRayRetainedFragment2::AlgebraicChord(chord) => {
                    let mut sides = [Ordering::Equal; 2];
                    for (side, endpoint) in sides.iter_mut().zip([chord.start(), chord.end()]) {
                        *side = match algebraic_chord_points_linear_order(
                            endpoint, point, &side_x, side_y, policy,
                        )? {
                            Classification::Decided(order) => order,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                    }
                    if sides.contains(&Ordering::Equal) {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    if incident || sides[0] == sides[1] {
                        Classification::Decided(0)
                    } else {
                        let side = match chord.oriented_support_side(point, policy)? {
                            Classification::Decided(side) => side,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        Classification::Decided(crate::contour::line_winding_decision(
                            sides[0] != Ordering::Greater,
                            sides[1] == Ordering::Greater,
                            side,
                        ))
                    }
                }
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            };
            match delta {
                Classification::Decided(delta) => winding += delta,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(RetainedRayWinding::Winding(
            winding,
        )))
    })
}

pub(super) fn point_evidence_loop_winding(
    boundary: &CurveRegionBoundaryLoop2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedRayWinding>> {
    let fragments = match prepare_point_ray_fragments(boundary, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut reason = UncertaintyReason::Boundary;
    // At most two endpoint-collinear slopes per fragment; the same exact
    // nondegeneracy bound is used by native algebraic point-ray queries.
    for slope in 0..fragments.len().saturating_mul(2).saturating_add(1) {
        let Ok(slope) = u64::try_from(slope) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match prepared_point_ray_winding(
            &fragments,
            point,
            &Real::one(),
            &Real::from(slope),
            false,
            policy,
        )? {
            decided @ Classification::Decided(_) => return Ok(decided),
            Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                return Ok(Classification::Uncertain(
                    UncertaintyReason::BudgetExhausted,
                ));
            }
            Classification::Uncertain(last) => reason = last,
        }
    }
    Ok(Classification::Uncertain(reason))
}

impl CurveRegion2 {
    pub(crate) fn classify_point_evidence_by_ray(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RegionPointLocation>> {
        let mut windings = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary in &self.data.boundary_loops {
            match point_evidence_loop_winding(boundary, point, policy)? {
                Classification::Decided(RetainedRayWinding::Boundary) => {
                    return Ok(Classification::Decided(RegionPointLocation::Boundary));
                }
                Classification::Decided(RetainedRayWinding::Winding(winding)) => {
                    windings.push(winding)
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        self.region_location_from_loop_windings(&windings)
            .map(Classification::Decided)
    }

    pub(crate) fn point_evidence_loop_windings_from_boundary_side_ray(
        &self,
        point: &CurvePoint2,
        direction_x: Real,
        direction_y: Real,
        source_loop_index: usize,
        source_fragment_index: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<i32>>> {
        if let CurvePointData2::Algebraic(point) = &point.0 {
            match self.algebraic_loop_windings_from_boundary_side_ray(
                point,
                direction_x.clone(),
                direction_y.clone(),
                source_loop_index,
                source_fragment_index,
                policy,
            )? {
                decided @ Classification::Decided(_) => return Ok(decided),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted) => {
                    return Ok(Classification::Uncertain(
                        UncertaintyReason::BudgetExhausted,
                    ));
                }
                Classification::Uncertain(_) => {}
            }
        }
        if self
            .data
            .boundary_loops
            .get(source_loop_index)
            .and_then(|boundary| boundary.fragments().get(source_fragment_index))
            .is_none()
        {
            return Err(CurveError::Topology(
                "retained point side-ray source is outside the boundary".into(),
            ));
        }
        match real_sign(
            &(&direction_x * &direction_x + &direction_y * &direction_y),
            policy,
        ) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "a side-ray squared norm is negative".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let mut windings = Vec::with_capacity(self.data.boundary_loops.len());
        for boundary in &self.data.boundary_loops {
            let fragments = match prepare_point_ray_fragments(boundary, policy)? {
                Classification::Decided(fragments) => fragments,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match prepared_point_ray_winding(
                &fragments,
                point,
                &direction_x,
                &direction_y,
                true,
                policy,
            )? {
                Classification::Decided(RetainedRayWinding::Winding(winding)) => {
                    windings.push(winding)
                }
                Classification::Decided(RetainedRayWinding::Boundary) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(windings))
    }
}
