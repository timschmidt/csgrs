//! Exact cycle selection for contracting native line offsets.
//!
//! A collapsed wavefront can revisit arrangement vertices even though its
//! filled set has an ordinary manifold boundary. This module splits that walk
//! at certified self contacts, decomposes its directed traversal into simple
//! cycles, and discards cycles whose retained source direction proves that the
//! wavefront has inverted. The selected cycles are regularized and composed by
//! the authoritative `CurveRegion2` kernel; no second Boolean engine lives
//! here, and no finite sample or epsilon perturbation is used.

use hyperreal::RealSign;

use crate::classify::{is_zero, real_sign};
use crate::{
    Classification, Contour2, ContourIntersection, CurveContext, CurveError, CurveResult, FillRule,
    Point2, Segment2, UncertaintyReason,
};

impl Contour2 {
    /// Selects cycles from a raw inward all-line offset and removes wavefront
    /// cycles whose source-parallel traversal has reversed after collapse.
    ///
    /// Offset construction retains the original directed line support on every
    /// emitted fragment. Self-contact splitting preserves that evidence, which
    /// makes pruning exact and avoids choosing an epsilon-sized interior sample.
    pub(crate) fn retained_contracting_line_offset_cycles(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let intersections = self.intersect_self(policy)?;
        if let Some(reason) = intersections.events().iter().find_map(|event| match event {
            ContourIntersection::Uncertain(blocker) => Some(blocker.reason),
            _ => None,
        }) {
            return Ok(Classification::Uncertain(reason));
        }
        if intersections.is_empty() {
            return Ok(match contour_follows_regular_offset_branch(self, policy) {
                Classification::Decided(true) => Classification::Decided(vec![self.clone()]),
                Classification::Decided(false) => Classification::Decided(Vec::new()),
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
        Ok(Classification::Decided(retained))
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
