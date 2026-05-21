//! Polygon ring Real classifiers.

use core::cmp::Ordering;

use crate::classify::{RingConvexity, RingPointLocation};
use crate::geometry::Point2;
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign,
};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::orient2d_with_policy;
use crate::predicates::segment::classify_point_segment_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{resolve_real_sign, signed_term_filter};
use hyperreal::{Real, ZeroKnowledge};

/// Structural facts retained for one closed polygonal ring.
///
/// These facts are cheap summaries over exact `Real` coordinates. They are
/// useful for algorithm selection, but they are not topology certificates:
/// later containment, visibility, and intersection decisions must still call
/// exact predicates. That mirrors Yap's object-fact layer from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Ring2Facts {
    /// Number of vertices in the caller-supplied ring.
    pub vertex_count: usize,
    /// Number of cyclic edges structurally known to collapse to a point.
    pub known_degenerate_edges: usize,
    /// Number of non-degenerate cyclic edges structurally known horizontal or vertical.
    pub known_axis_aligned_edges: usize,
    /// Number of cyclic edges with unknown coordinate-zero status.
    pub unknown_edge_zero_status: usize,
    /// Certified sign of twice the signed ring area, when available.
    pub signed_area: Option<Sign>,
    /// Certified local turn consistency for the ring.
    pub convexity: RingConvexity,
}

/// Build structural facts for a closed polygonal ring.
pub fn ring2_facts(points: &[Point2]) -> Ring2Facts {
    ring2_facts_with_policy(points, PredicatePolicy::default())
}

/// Build structural facts for a closed polygonal ring with an explicit policy.
pub fn ring2_facts_with_policy(points: &[Point2], policy: PredicatePolicy) -> Ring2Facts {
    let refs: Vec<_> = points.iter().collect();
    ring2_facts_refs(&refs, policy)
}

/// Build structural facts for a closed polygonal ring stored as indices.
pub fn indexed_ring2_facts(points: &[Point2], ring: &[usize]) -> Option<Ring2Facts> {
    indexed_ring2_facts_with_policy(points, ring, PredicatePolicy::default())
}

/// Build structural facts for an indexed closed polygonal ring with an explicit
/// policy.
pub fn indexed_ring2_facts_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> Option<Ring2Facts> {
    let refs = indexed_ring_refs(points, ring)?;
    Some(ring2_facts_refs(&refs, policy))
}

fn ring2_facts_refs(points: &[&Point2], policy: PredicatePolicy) -> Ring2Facts {
    let mut facts = Ring2Facts {
        vertex_count: points.len(),
        known_degenerate_edges: 0,
        known_axis_aligned_edges: 0,
        unknown_edge_zero_status: 0,
        signed_area: ring_area_sign_refs(points, policy).value(),
        convexity: ring_convexity_refs(points, policy),
    };

    if points.len() < 2 {
        return facts;
    }

    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        let dx = sub_ref(&next.x, &current.x);
        let dy = sub_ref(&next.y, &current.y);

        match (dx.structural_facts().zero, dy.structural_facts().zero) {
            (ZeroKnowledge::Zero, ZeroKnowledge::Zero) => facts.known_degenerate_edges += 1,
            (ZeroKnowledge::Zero, ZeroKnowledge::NonZero)
            | (ZeroKnowledge::NonZero, ZeroKnowledge::Zero) => {
                facts.known_axis_aligned_edges += 1;
            }
            (ZeroKnowledge::Unknown, _) | (_, ZeroKnowledge::Unknown) => {
                facts.unknown_edge_zero_status += 1;
            }
            (ZeroKnowledge::NonZero, ZeroKnowledge::NonZero) => {}
        }
    }

    facts
}

/// Classify local turn consistency for a closed polygonal ring.
pub fn ring_convexity(points: &[Point2]) -> RingConvexity {
    ring_convexity_with_policy(points, PredicatePolicy::default())
}

/// Classify local turn consistency for a closed polygonal ring with a policy.
pub fn ring_convexity_with_policy(points: &[Point2], policy: PredicatePolicy) -> RingConvexity {
    let refs: Vec<_> = points.iter().collect();
    ring_convexity_refs(&refs, policy)
}

/// Classify local turn consistency for an indexed closed polygonal ring.
pub fn indexed_ring_convexity(points: &[Point2], ring: &[usize]) -> Option<RingConvexity> {
    indexed_ring_convexity_with_policy(points, ring, PredicatePolicy::default())
}

/// Classify local turn consistency for an indexed closed polygonal ring with a
/// policy.
pub fn indexed_ring_convexity_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> Option<RingConvexity> {
    let refs = indexed_ring_refs(points, ring)?;
    Some(ring_convexity_refs(&refs, policy))
}

/// Return the sign of twice the signed area of a closed polygonal ring.
///
/// The input may repeat its first vertex at the end; the repeated closing edge
/// contributes zero. The function evaluates the shoelace determinant exactly
/// and reports only its sign. The determinant form is the standard polygon area
/// formula described in computational-geometry texts such as de Berg, Cheong,
/// van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed., Springer, 2008. This function keeps the determinant
/// in `hyperlimit` because orientation/winding is a predicate-level decision;
/// ring storage and material/hole roles belong in `hypercurve` or `hypertri`.
pub fn ring_area_sign(points: &[Point2]) -> PredicateOutcome<Sign> {
    ring_area_sign_with_policy(points, PredicatePolicy::default())
}

/// Return the sign of twice the signed area of a closed polygonal ring with an
/// explicit predicate escalation policy.
pub fn ring_area_sign_with_policy(
    points: &[Point2],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let refs: Vec<_> = points.iter().collect();
    ring_area_sign_refs(&refs, policy)
}

/// Return the sign of twice the signed area of an indexed closed polygonal ring.
pub fn indexed_ring_area_sign(points: &[Point2], ring: &[usize]) -> PredicateOutcome<Sign> {
    indexed_ring_area_sign_with_policy(points, ring, PredicatePolicy::default())
}

/// Return the sign of twice the signed area of an indexed closed polygonal ring
/// with an explicit predicate escalation policy.
pub fn indexed_ring_area_sign_with_policy(
    points: &[Point2],
    ring: &[usize],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let Some(refs) = indexed_ring_refs(points, ring) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    };
    ring_area_sign_refs(&refs, policy)
}

fn ring_area_sign_refs(points: &[&Point2], policy: PredicatePolicy) -> PredicateOutcome<Sign> {
    if points.len() < 3 {
        return PredicateOutcome::decided(
            Sign::Zero,
            crate::predicate::Certainty::Exact,
            crate::predicate::Escalation::Structural,
        );
    }

    let mut terms = Vec::with_capacity(points.len() * 2);
    let mut area: Option<Real> = None;

    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let positive = mul_ref(&points[index].x, &points[next].y);
        let negative = mul_ref(&points[index].y, &points[next].x);
        let wedge = sub_ref(&positive, &negative);
        terms.push((positive, Sign::Positive));
        terms.push((negative, Sign::Negative));
        area = Some(match area {
            Some(current) => add_ref(&current, &wedge),
            None => wedge,
        });
    }

    let area = area.expect("three or more points produce at least one wedge");
    resolve_real_sign(
        &area,
        policy,
        || {
            let refs: Vec<_> = terms.iter().map(|(term, sign)| (term, *sign)).collect();
            signed_term_filter(&refs)
        },
        || None,
        RefinementNeed::RealRefinement,
    )
}

/// Classify a point against a closed polygonal ring by the even-odd rule.
pub fn classify_point_ring_even_odd(
    ring: &[Point2],
    point: &Point2,
) -> PredicateOutcome<RingPointLocation> {
    classify_point_ring_even_odd_with_policy(ring, point, PredicatePolicy::default())
}

/// Classify a point against a closed polygonal ring by the even-odd rule with an
/// explicit predicate escalation policy.
///
/// Boundary checks are performed first with exact point-on-segment
/// classification. Interior parity is then decided by an orientation-form ray
/// crossing test so no edge/ray intersection coordinate is constructed. This is
/// the standard crossing-number idea discussed by Hormann and Agathos, "The
/// Point in Polygon Problem for Arbitrary Polygons," *Computational Geometry*
/// 20.3 (2001), but every crossing decision is certified through exact signs in
/// the exact-geometric-computation model of Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
pub fn classify_point_ring_even_odd_with_policy(
    ring: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingPointLocation> {
    let refs: Vec<_> = ring.iter().collect();
    classify_point_ring_even_odd_refs(&refs, point, policy)
}

/// Classify a point against an indexed closed polygonal ring by the even-odd
/// rule.
pub fn classify_point_indexed_ring_even_odd(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
) -> PredicateOutcome<RingPointLocation> {
    classify_point_indexed_ring_even_odd_with_policy(
        points,
        ring,
        point,
        PredicatePolicy::default(),
    )
}

/// Classify a point against an indexed closed polygonal ring by the even-odd
/// rule with an explicit predicate escalation policy.
pub fn classify_point_indexed_ring_even_odd_with_policy(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingPointLocation> {
    let Some(refs) = indexed_ring_refs(points, ring) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    };
    classify_point_ring_even_odd_refs(&refs, point, policy)
}

fn classify_point_ring_even_odd_refs(
    ring: &[&Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<RingPointLocation> {
    if ring.len() < 3 {
        return PredicateOutcome::decided(
            RingPointLocation::Outside,
            Certainty::Exact,
            Escalation::Structural,
        );
    }

    let mut trace = DecisionTrace::default();
    let mut inside = false;

    for index in 0..ring.len() {
        let a = &ring[index];
        let b = &ring[(index + 1) % ring.len()];

        match decided(
            classify_point_segment_with_policy(a, b, point, policy),
            &mut trace,
        ) {
            Ok(location) if location.is_on_segment() => {
                return PredicateOutcome::decided(
                    RingPointLocation::Boundary,
                    trace.certainty,
                    trace.stage,
                );
            }
            Ok(_) => {}
            Err(unknown) => return unknown.into_outcome(),
        }

        let a_above = match compare_greater(&a.y, &point.y, policy, &mut trace) {
            Ok(value) => value,
            Err(unknown) => return unknown.into_outcome(),
        };
        let b_above = match compare_greater(&b.y, &point.y, policy, &mut trace) {
            Ok(value) => value,
            Err(unknown) => return unknown.into_outcome(),
        };
        if a_above == b_above {
            continue;
        }

        let orientation = match decided(orient2d_with_policy(a, b, point, policy), &mut trace) {
            Ok(sign) => sign,
            Err(unknown) => return unknown.into_outcome(),
        };
        let upward = match compare_greater(&b.y, &a.y, policy, &mut trace) {
            Ok(value) => value,
            Err(unknown) => return unknown.into_outcome(),
        };

        let crosses_right = matches!(
            (upward, orientation),
            (true, Sign::Positive) | (false, Sign::Negative)
        );
        if crosses_right {
            inside = !inside;
        }
    }

    PredicateOutcome::decided(
        if inside {
            RingPointLocation::Inside
        } else {
            RingPointLocation::Outside
        },
        trace.certainty,
        trace.stage,
    )
}

/// Return whether `point` is inside or on the boundary of `ring` by the
/// even-odd rule.
pub fn point_in_ring_even_odd(ring: &[Point2], point: &Point2) -> PredicateOutcome<bool> {
    point_in_ring_even_odd_with_policy(ring, point, PredicatePolicy::default())
}

/// Return whether `point` is inside or on the boundary of `ring` by the
/// even-odd rule with an explicit predicate escalation policy.
pub fn point_in_ring_even_odd_with_policy(
    ring: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_ring_even_odd_with_policy(ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether `point` is inside or on the boundary of an indexed ring by
/// the even-odd rule.
pub fn point_in_indexed_ring_even_odd(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
) -> PredicateOutcome<bool> {
    point_in_indexed_ring_even_odd_with_policy(points, ring, point, PredicatePolicy::default())
}

/// Return whether `point` is inside or on the boundary of an indexed ring by
/// the even-odd rule with an explicit predicate escalation policy.
pub fn point_in_indexed_ring_even_odd_with_policy(
    points: &[Point2],
    ring: &[usize],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_indexed_ring_even_odd_with_policy(points, ring, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn indexed_ring_refs<'a>(points: &'a [Point2], ring: &[usize]) -> Option<Vec<&'a Point2>> {
    ring.iter().map(|&index| points.get(index)).collect()
}

fn ring_convexity_refs(points: &[&Point2], policy: PredicatePolicy) -> RingConvexity {
    let len = open_ring_len(points);
    if len < 3 {
        return RingConvexity::Degenerate;
    }

    let mut saw_positive = false;
    let mut saw_negative = false;
    for index in 0..len {
        let previous = points[(index + len - 1) % len];
        let current = points[index];
        let next = points[(index + 1) % len];
        let Some(sign) = orient2d_with_policy(previous, current, next, policy).value() else {
            return RingConvexity::Unknown;
        };

        match sign {
            Sign::Positive => saw_positive = true,
            Sign::Negative => saw_negative = true,
            Sign::Zero => {}
        }

        if saw_positive && saw_negative {
            return RingConvexity::MixedTurns;
        }
    }

    if saw_positive || saw_negative {
        RingConvexity::LocallyConvex
    } else {
        RingConvexity::Degenerate
    }
}

fn open_ring_len(points: &[&Point2]) -> usize {
    let mut len = points.len();
    if len > 1 && points[0] == points[len - 1] {
        len -= 1;
    }
    len
}

fn compare_greater(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(decided(compare_reals_with_policy(left, right, policy), trace)? == Ordering::Greater)
}

#[derive(Clone, Copy)]
struct DecisionTrace {
    certainty: Certainty,
    stage: Escalation,
}

impl Default for DecisionTrace {
    fn default() -> Self {
        Self {
            certainty: Certainty::Exact,
            stage: Escalation::Structural,
        }
    }
}

#[derive(Clone, Copy)]
struct UnknownDecision {
    needed: RefinementNeed,
    stage: Escalation,
}

impl UnknownDecision {
    fn into_outcome<T>(self) -> PredicateOutcome<T> {
        PredicateOutcome::unknown(self.needed, self.stage)
    }
}

fn decided<T>(
    outcome: PredicateOutcome<T>,
    trace: &mut DecisionTrace,
) -> Result<T, UnknownDecision> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            trace.certainty = max_certainty(trace.certainty, certainty);
            trace.stage = max_stage(trace.stage, stage);
            Ok(value)
        }
        PredicateOutcome::Unknown { needed, stage } => Err(UnknownDecision { needed, stage }),
    }
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
    }
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    #[test]
    fn ring_area_sign_classifies_winding_and_degenerate_rings() {
        let ccw = [p2(0, 0), p2(4, 0), p2(4, 3), p2(0, 3)];
        let cw = [p2(0, 0), p2(0, 3), p2(4, 3), p2(4, 0)];
        let line = [p2(0, 0), p2(1, 1), p2(2, 2)];

        assert_eq!(ring_area_sign(&ccw).value(), Some(Sign::Positive));
        assert_eq!(ring_area_sign(&cw).value(), Some(Sign::Negative));
        assert_eq!(ring_area_sign(&line).value(), Some(Sign::Zero));
        assert_eq!(ring_area_sign(&[]).value(), Some(Sign::Zero));
    }

    #[test]
    fn point_ring_even_odd_classifies_inside_outside_and_boundary() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];

        assert_eq!(
            classify_point_ring_even_odd(&ring, &p2(2, 2)).value(),
            Some(RingPointLocation::Inside)
        );
        assert_eq!(
            classify_point_ring_even_odd(&ring, &p2(5, 2)).value(),
            Some(RingPointLocation::Outside)
        );
        assert_eq!(
            classify_point_ring_even_odd(&ring, &p2(4, 2)).value(),
            Some(RingPointLocation::Boundary)
        );
        assert_eq!(point_in_ring_even_odd(&ring, &p2(4, 2)).value(), Some(true));
    }

    #[test]
    fn point_ring_even_odd_accepts_repeated_closing_vertex() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4), p2(0, 0)];

        assert_eq!(
            classify_point_ring_even_odd(&ring, &p2(1, 1)).value(),
            Some(RingPointLocation::Inside)
        );
    }

    #[test]
    fn ring_facts_count_raw_edges_but_classify_open_turns() {
        let ring = [p2(0, 0), p2(4, 0), p2(4, 3), p2(0, 3), p2(0, 0)];
        let facts = ring2_facts(&ring);

        assert_eq!(facts.vertex_count, 5);
        assert_eq!(facts.known_degenerate_edges, 1);
        assert_eq!(facts.known_axis_aligned_edges, 4);
        assert_eq!(facts.unknown_edge_zero_status, 0);
        assert_eq!(facts.signed_area, Some(Sign::Positive));
        assert_eq!(facts.convexity, RingConvexity::LocallyConvex);
    }

    #[test]
    fn indexed_ring_predicates_reuse_caller_topology() {
        let points = [p2(9, 9), p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
        let ring = [1, 2, 3, 4];

        assert_eq!(
            indexed_ring_area_sign(&points, &ring).value(),
            Some(Sign::Positive)
        );
        assert_eq!(
            classify_point_indexed_ring_even_odd(&points, &ring, &p2(2, 2)).value(),
            Some(RingPointLocation::Inside)
        );
        assert_eq!(
            indexed_ring_convexity(&points, &ring),
            Some(RingConvexity::LocallyConvex)
        );
        assert!(indexed_ring2_facts(&points, &[1, 99]).is_none());
    }
}
