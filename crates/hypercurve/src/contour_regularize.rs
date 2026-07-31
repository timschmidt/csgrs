//! Exact regularization of self-intersecting native contours.
//!
//! A closed walk can revisit arrangement vertices even though its filled set
//! has an ordinary manifold boundary. This module splits the walk at certified
//! self contacts, decomposes its directed traversal into simple cycles, and
//! combines those cycles according to the authored fill rule. No finite sample
//! or epsilon perturbation is used.

use hyperreal::RealSign;

use crate::classify::{compare_reals, is_zero, real_sign};
use crate::{
    Aabb2, BooleanOp, Classification, Contour2, ContourIntersection, CurveContext, CurveError,
    CurveResult, FillRule, IntersectionKind, LineArcRegion2, Point2, Segment2, UncertaintyReason,
};

impl Contour2 {
    pub(crate) fn regularize_self_intersections_native(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let intersections = self.intersect_self(policy)?;
        if let Some(reason) = intersections.events().iter().find_map(|event| match event {
            ContourIntersection::Uncertain(blocker) => Some(blocker.reason),
            _ => None,
        }) {
            return Ok(Classification::Uncertain(reason));
        }
        if intersections.is_empty() {
            return Ok(Classification::Decided(
                LineArcRegion2::from_material_contours(vec![self.clone()]),
            ));
        }

        let fragments = match self.split_at_self_intersections(&intersections, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cycles = match decompose_closed_walk(fragments.fragments(), self.fill_rule(), policy)? {
            Classification::Decided(cycles) => cycles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        regularize_cycles(cycles, self.fill_rule(), policy)
    }

    /// Regularizes a raw inward all-line offset and removes wavefront cycles
    /// whose source-parallel traversal has reversed after collapse.
    ///
    /// Offset construction retains the original directed line support on every
    /// emitted fragment. Self-contact splitting preserves that evidence, which
    /// makes pruning exact and avoids choosing an epsilon-sized interior sample.
    pub(crate) fn regularize_contracting_line_offset_native(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let intersections = self.intersect_self(policy)?;
        if let Some(reason) = intersections.events().iter().find_map(|event| match event {
            ContourIntersection::Uncertain(blocker) => Some(blocker.reason),
            _ => None,
        }) {
            return Ok(Classification::Uncertain(reason));
        }
        if intersections.is_empty() {
            return Ok(match contour_follows_regular_offset_branch(self, policy) {
                Classification::Decided(true) => {
                    Classification::Decided(LineArcRegion2::from_material_contours(vec![
                        self.clone(),
                    ]))
                }
                Classification::Decided(false) => Classification::Decided(LineArcRegion2::empty()),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }

        let fragments = match self.split_at_self_intersections(&intersections, policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let cycles = match decompose_closed_walk(fragments.fragments(), self.fill_rule(), policy)? {
            Classification::Decided(cycles) => cycles,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained = Vec::new();
        for cycle in cycles {
            match contour_follows_regular_offset_branch(&cycle, policy) {
                Classification::Decided(true) => retained.push(cycle),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        regularize_cycles(retained, self.fill_rule(), policy)
    }
}

fn contour_follows_regular_offset_branch(
    contour: &Contour2,
    policy: &CurveContext,
) -> Classification<bool> {
    for segment in contour.segments() {
        let Segment2::Line(line) = segment else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        match line.retained_offset_direction_matches_source(policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Classification::Decided(false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(true)
}

fn decompose_closed_walk(
    fragments: &[crate::ContourFragment],
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let Some(first) = fragments.first() else {
        return Err(CurveError::EmptyCurveString);
    };
    let mut vertices = vec![first.segment.start().clone()];
    let mut edges = Vec::<Segment2>::new();
    let mut cycles = Vec::new();

    for fragment in fragments {
        match points_equal(
            vertices.last().expect("walk retains a current vertex"),
            fragment.segment.start(),
            policy,
        ) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "self-intersection fragments are not in connected traversal order".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        edges.push(fragment.segment.clone());
        let repeated = match find_vertex(&vertices, fragment.segment.end(), policy) {
            Classification::Decided(repeated) => repeated,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(index) = repeated {
            let cycle_edges = edges.split_off(index);
            vertices.truncate(index + 1);
            let cycle = Contour2::try_new_with_fill_rule(cycle_edges, fill_rule)?;
            match cycle.signed_area()? {
                Some(area) => match real_sign(&area, policy) {
                    Some(RealSign::Positive | RealSign::Negative) => cycles.push(cycle),
                    Some(RealSign::Zero) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                },
                None => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            }
        } else {
            vertices.push(fragment.segment.end().clone());
        }
    }

    if !edges.is_empty() || vertices.len() != 1 {
        return Err(CurveError::Topology(
            "self-intersection fragment traversal did not decompose into closed cycles".into(),
        ));
    }
    Ok(Classification::Decided(cycles))
}

fn find_vertex(
    vertices: &[Point2],
    point: &Point2,
    policy: &CurveContext,
) -> Classification<Option<usize>> {
    for (index, candidate) in vertices.iter().enumerate() {
        match points_equal(candidate, point, policy) {
            Classification::Decided(true) => return Classification::Decided(Some(index)),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(None)
}

fn points_equal(first: &Point2, second: &Point2, policy: &CurveContext) -> Classification<bool> {
    match is_zero(&first.distance_squared(second), policy) {
        Some(equal) => Classification::Decided(equal),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn regularize_cycles(
    cycles: Vec<Contour2>,
    fill_rule: FillRule,
    policy: &CurveContext,
) -> CurveResult<Classification<LineArcRegion2>> {
    match fill_rule {
        FillRule::EvenOdd => regularize_even_odd_cycles(cycles, policy),
        FillRule::NonZero => regularize_nonzero_cycles(cycles, policy),
    }
}

fn regularize_even_odd_cycles(
    cycles: Vec<Contour2>,
    policy: &CurveContext,
) -> CurveResult<Classification<LineArcRegion2>> {
    let mut result = LineArcRegion2::empty();
    for cycle in cycles {
        let component = LineArcRegion2::from_material_contours(vec![cycle]);
        result = match boolean(&result, &component, BooleanOp::Xor, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(result))
}

fn regularize_nonzero_cycles(
    cycles: Vec<Contour2>,
    policy: &CurveContext,
) -> CurveResult<Classification<LineArcRegion2>> {
    // Each retained region is one disjoint integer-winding layer. The implicit
    // unbounded layer has winding zero and never needs materialization.
    let mut layers = Vec::<(i32, LineArcRegion2)>::new();
    for cycle in cycles {
        let Some(area) = cycle.signed_area()? else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let delta = match real_sign(&area, policy) {
            Some(RealSign::Positive) => 1,
            Some(RealSign::Negative) => -1,
            Some(RealSign::Zero) => continue,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let cycle_region = LineArcRegion2::from_material_contours(vec![cycle]);
        layers = match add_winding_cycle(layers, cycle_region, delta, policy)? {
            Classification::Decided(layers) => layers,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }

    let mut result = LineArcRegion2::empty();
    for (_, layer) in layers {
        result = match boolean(&result, &layer, BooleanOp::Union, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(result))
}

fn add_winding_cycle(
    layers: Vec<(i32, LineArcRegion2)>,
    cycle: LineArcRegion2,
    delta: i32,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<(i32, LineArcRegion2)>>> {
    let mut remaining = cycle;
    let mut output = Vec::<(i32, LineArcRegion2)>::new();
    for (winding, layer) in layers {
        let inside = match boolean(&layer, &remaining, BooleanOp::Intersection, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let outside = match boolean(&layer, &remaining, BooleanOp::Difference, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let next_remaining = match boolean(&remaining, &layer, BooleanOp::Difference, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for (target_winding, region) in [(winding, outside), (winding + delta, inside)] {
            if let Classification::Uncertain(reason) =
                merge_layer(&mut output, target_winding, region, policy)?
            {
                return Ok(Classification::Uncertain(reason));
            }
        }
        remaining = next_remaining;
    }
    if let Classification::Uncertain(reason) = merge_layer(&mut output, delta, remaining, policy)? {
        return Ok(Classification::Uncertain(reason));
    }
    Ok(Classification::Decided(output))
}

fn merge_layer(
    layers: &mut Vec<(i32, LineArcRegion2)>,
    winding: i32,
    region: LineArcRegion2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    if winding == 0 || region.is_empty() {
        return Ok(Classification::Decided(()));
    }
    if let Some((_, existing)) = layers
        .iter_mut()
        .find(|(candidate, _)| *candidate == winding)
    {
        *existing = match boolean(existing, &region, BooleanOp::Union, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    } else {
        layers.push((winding, region));
    }
    Ok(Classification::Decided(()))
}

fn boolean(
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    operation: BooleanOp,
    policy: &CurveContext,
) -> CurveResult<Classification<LineArcRegion2>> {
    if certified_interior_disjoint(first, second, policy)? {
        return Ok(Classification::Decided(match operation {
            BooleanOp::Union | BooleanOp::Xor => concatenate_regions(first, second),
            BooleanOp::Intersection => LineArcRegion2::empty(),
            BooleanOp::Difference => first.clone(),
        }));
    }
    first.boolean_region(second, operation, FillRule::NonZero, policy)
}

fn certified_interior_disjoint(
    first: &LineArcRegion2,
    second: &LineArcRegion2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    if first.is_empty() || second.is_empty() {
        return Ok(true);
    }
    let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
        Aabb2::from_region(first, policy)?,
        Aabb2::from_region(second, policy)?,
    ) else {
        return Ok(false);
    };
    let separated = [
        compare_reals(first_bounds.max_x(), second_bounds.min_x(), policy),
        compare_reals(second_bounds.max_x(), first_bounds.min_x(), policy),
        compare_reals(first_bounds.max_y(), second_bounds.min_y(), policy),
        compare_reals(second_bounds.max_y(), first_bounds.min_y(), policy),
    ]
    .into_iter()
    .any(|ordering| {
        matches!(
            ordering,
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        )
    });
    if !separated {
        return Ok(false);
    }

    let contacts = first.intersect_region(second, policy)?;
    Ok(contacts.overlap_event_count() == 0
        && contacts.uncertain_event_count() == 0
        && contacts.pairs().iter().all(|pair| {
            pair.intersections().events().iter().all(|event| {
                matches!(
                    event,
                    ContourIntersection::Point(point)
                        if matches!(point.kind, IntersectionKind::Endpoint | IntersectionKind::Tangent)
                )
            })
        }))
}

fn concatenate_regions(first: &LineArcRegion2, second: &LineArcRegion2) -> LineArcRegion2 {
    let mut material = first.material_contours().to_vec();
    material.extend(second.material_contours().iter().cloned());
    let mut holes = first.hole_contours().to_vec();
    holes.extend(second.hole_contours().iter().cloned());
    LineArcRegion2::new(material, holes)
}
