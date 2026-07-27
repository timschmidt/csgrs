//! Stateless exact predicates for 2D axis-aligned boxes.
//!
//! This module deliberately does not define or own a bounding-box data
//! structure. Curve, triangulation, and broad-phase crates keep their own
//! storage and call these helpers to certify inclusive box predicates over
//! borrowed min/max points.

use crate::classify::{
    Aabb2Intersection, Aabb2PointLocation, Aabb3Intersection, Aabb3PointLocation,
    ClosedIntervalIntersection, RealIntervalLocation,
};
use crate::geometry::{Aabb2Facts, Point2, Point3};
use crate::predicate::PredicatePolicy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed};
use crate::predicates::interval::{
    classify_closed_interval_intersection_with_policy, classify_real_closed_interval_with_policy,
};
use crate::predicates::order::compare_reals_with_policy;
use core::cmp::Ordering;

/// Classify a point relative to a closed 2D axis-aligned box.
pub fn classify_point_aabb2(
    min: &Point2,
    max: &Point2,
    point: &Point2,
) -> PredicateOutcome<Aabb2PointLocation> {
    classify_point_aabb2_with_policy(min, max, point, PredicatePolicy)
}

/// Classify a point relative to a closed 2D axis-aligned box with an explicit
/// predicate escalation policy.
///
/// The min/max corners may be supplied in either coordinate order; each axis is
/// normalized by exact interval predicates. These box predicates are safe
/// broad-phase filters for arrangements, curve intersection, and triangulation
/// candidate pruning. Boxes reduce candidate sets, but final topology still
/// belongs to orientation and incidence predicates.
pub(crate) fn classify_point_aabb2_with_policy(
    min: &Point2,
    max: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb2PointLocation> {
    let mut trace = DecisionTrace::default();

    let x = match decided(
        classify_real_closed_interval_with_policy(&point.x, &min.x, &max.x, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !x.is_inside_or_boundary() {
        return PredicateOutcome::decided(
            Aabb2PointLocation::Outside,
            trace.certainty,
            trace.stage,
        );
    }

    let y = match decided(
        classify_real_closed_interval_with_policy(&point.y, &min.y, &max.y, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !y.is_inside_or_boundary() {
        return PredicateOutcome::decided(
            Aabb2PointLocation::Outside,
            trace.certainty,
            trace.stage,
        );
    }

    let location = if is_interval_boundary(x) || is_interval_boundary(y) {
        Aabb2PointLocation::Boundary
    } else {
        Aabb2PointLocation::Inside
    };
    PredicateOutcome::decided(location, trace.certainty, trace.stage)
}

/// Return whether a point lies in a closed 2D axis-aligned box.
pub fn point_in_aabb2(min: &Point2, max: &Point2, point: &Point2) -> PredicateOutcome<bool> {
    point_in_aabb2_with_policy(min, max, point, PredicatePolicy)
}

/// Return whether a point lies in a closed 2D axis-aligned box with an explicit
/// predicate escalation policy.
pub(crate) fn point_in_aabb2_with_policy(
    min: &Point2,
    max: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_aabb2_with_policy(min, max, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether a point lies in the closed axis-aligned bounding box of a
/// 2D triangle.
pub fn point_in_triangle2_aabb(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> PredicateOutcome<bool> {
    point_in_triangle2_aabb_with_policy(a, b, c, point, PredicatePolicy)
}

/// Return whether a point lies in the closed axis-aligned bounding box of a
/// 2D triangle with an explicit predicate escalation policy.
///
/// This is a rejection filter for downstream exact triangle predicates. Every
/// min/max and interval decision is exact, preserving the boundary between
/// filters and final topology.
pub(crate) fn point_in_triangle2_aabb_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    let mut trace = DecisionTrace::default();

    let (min_x, max_x) = match min_max3(&a.x, &b.x, &c.x, policy, &mut trace) {
        Ok(bounds) => bounds,
        Err(unknown) => return unknown.into_outcome(),
    };
    let x = match decided(
        classify_real_closed_interval_with_policy(&point.x, min_x, max_x, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !x.is_inside_or_boundary() {
        return PredicateOutcome::decided(false, trace.certainty, trace.stage);
    }

    let (min_y, max_y) = match min_max3(&a.y, &b.y, &c.y, policy, &mut trace) {
        Ok(bounds) => bounds,
        Err(unknown) => return unknown.into_outcome(),
    };
    let y = match decided(
        classify_real_closed_interval_with_policy(&point.y, min_y, max_y, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };

    PredicateOutcome::decided(y.is_inside_or_boundary(), trace.certainty, trace.stage)
}

/// Classify a point relative to a closed 3D axis-aligned box.
pub fn classify_point_aabb3(
    min: &Point3,
    max: &Point3,
    point: &Point3,
) -> PredicateOutcome<Aabb3PointLocation> {
    classify_point_aabb3_with_policy(min, max, point, PredicatePolicy)
}

/// Classify a point relative to a closed 3D axis-aligned box with an explicit
/// predicate escalation policy.
///
/// The min/max corners may be supplied in either coordinate order; each axis is
/// normalized by exact interval predicates.
pub(crate) fn classify_point_aabb3_with_policy(
    min: &Point3,
    max: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb3PointLocation> {
    let mut trace = DecisionTrace::default();

    let x = match decided(
        classify_real_closed_interval_with_policy(&point.x, &min.x, &max.x, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !x.is_inside_or_boundary() {
        return PredicateOutcome::decided(
            Aabb3PointLocation::Outside,
            trace.certainty,
            trace.stage,
        );
    }

    let y = match decided(
        classify_real_closed_interval_with_policy(&point.y, &min.y, &max.y, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !y.is_inside_or_boundary() {
        return PredicateOutcome::decided(
            Aabb3PointLocation::Outside,
            trace.certainty,
            trace.stage,
        );
    }

    let z = match decided(
        classify_real_closed_interval_with_policy(&point.z, &min.z, &max.z, policy),
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    if !z.is_inside_or_boundary() {
        return PredicateOutcome::decided(
            Aabb3PointLocation::Outside,
            trace.certainty,
            trace.stage,
        );
    }

    let location = if is_interval_boundary(x) || is_interval_boundary(y) || is_interval_boundary(z)
    {
        Aabb3PointLocation::Boundary
    } else {
        Aabb3PointLocation::Inside
    };
    PredicateOutcome::decided(location, trace.certainty, trace.stage)
}

/// Return whether a point lies in a closed 3D axis-aligned box.
pub fn point_in_aabb3(min: &Point3, max: &Point3, point: &Point3) -> PredicateOutcome<bool> {
    point_in_aabb3_with_policy(min, max, point, PredicatePolicy)
}

/// Return whether a point lies in a closed 3D axis-aligned box with an explicit
/// predicate escalation policy.
pub(crate) fn point_in_aabb3_with_policy(
    min: &Point3,
    max: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_aabb3_with_policy(min, max, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_inside_or_boundary(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the intersection relation between two closed 2D axis-aligned boxes.
pub fn classify_aabb2_intersection(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
) -> PredicateOutcome<Aabb2Intersection> {
    classify_aabb2_intersection_with_policy(
        first_min,
        first_max,
        second_min,
        second_max,
        PredicatePolicy,
    )
}

/// Classify the intersection relation between two closed 2D axis-aligned boxes
/// with an explicit predicate escalation policy.
///
/// `Touching` covers edge and corner contact with zero area. `Overlapping`
/// means both coordinate intervals overlap over positive length, so the box
/// intersection has positive area.
pub(crate) fn classify_aabb2_intersection_with_policy(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb2Intersection> {
    classify_aabb2_intersection_with_policy_and_facts(
        first_min,
        first_max,
        second_min,
        second_max,
        policy,
        crate::geometry::aabb2_facts(first_min, first_max),
        crate::geometry::aabb2_facts(second_min, second_max),
    )
}

/// Classify the intersection relation between two closed 2D axis-aligned boxes
/// with caller-cached structural facts.
///
/// The facts are used only after exact interval predicates prove both axes
/// intersect. A structurally zero-area input box cannot have a positive-area
/// box intersection, so the final relation is `Touching` rather than
/// `Overlapping`. This is a local exact specialization of the box broad phase;
/// uncertain extent facts do not decide topology by themselves.
pub fn classify_aabb2_intersection_with_facts(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
    first_facts: Aabb2Facts,
    second_facts: Aabb2Facts,
) -> PredicateOutcome<Aabb2Intersection> {
    classify_aabb2_intersection_with_policy_and_facts(
        first_min,
        first_max,
        second_min,
        second_max,
        PredicatePolicy,
        first_facts,
        second_facts,
    )
}

/// Classify the intersection relation between two closed 2D axis-aligned boxes
/// with both an explicit policy and caller-cached structural facts.
pub(crate) fn classify_aabb2_intersection_with_policy_and_facts(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
    policy: PredicatePolicy,
    first_facts: Aabb2Facts,
    second_facts: Aabb2Facts,
) -> PredicateOutcome<Aabb2Intersection> {
    let mut trace = DecisionTrace::default();

    let x = match decided(
        classify_closed_interval_intersection_with_policy(
            &first_min.x,
            &first_max.x,
            &second_min.x,
            &second_max.x,
            policy,
        ),
        &mut trace,
    ) {
        Ok(intersection) => intersection,
        Err(unknown) => return unknown.into_outcome(),
    };
    if x == ClosedIntervalIntersection::Disjoint {
        return PredicateOutcome::decided(
            Aabb2Intersection::Disjoint,
            trace.certainty,
            trace.stage,
        );
    }

    let y = match decided(
        classify_closed_interval_intersection_with_policy(
            &first_min.y,
            &first_max.y,
            &second_min.y,
            &second_max.y,
            policy,
        ),
        &mut trace,
    ) {
        Ok(intersection) => intersection,
        Err(unknown) => return unknown.into_outcome(),
    };
    if y == ClosedIntervalIntersection::Disjoint {
        return PredicateOutcome::decided(
            Aabb2Intersection::Disjoint,
            trace.certainty,
            trace.stage,
        );
    }

    let zero_area_input =
        first_facts.known_zero_area() == Some(true) || second_facts.known_zero_area() == Some(true);
    let relation = if x == ClosedIntervalIntersection::Touching
        || y == ClosedIntervalIntersection::Touching
        || zero_area_input
    {
        Aabb2Intersection::Touching
    } else {
        Aabb2Intersection::Overlapping
    };
    PredicateOutcome::decided(relation, trace.certainty, trace.stage)
}

/// Return whether two closed 2D axis-aligned boxes intersect.
pub fn aabb2s_intersect(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
) -> PredicateOutcome<bool> {
    aabb2s_intersect_with_policy(
        first_min,
        first_max,
        second_min,
        second_max,
        PredicatePolicy,
    )
}

/// Return whether two closed 2D axis-aligned boxes intersect with an explicit
/// predicate escalation policy.
pub(crate) fn aabb2s_intersect_with_policy(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_aabb2_intersection_with_policy(
        first_min, first_max, second_min, second_max, policy,
    ) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.intersects(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the intersection relation between two closed 3D axis-aligned boxes.
pub fn classify_aabb3_intersection(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
) -> PredicateOutcome<Aabb3Intersection> {
    classify_aabb3_intersection_with_policy(
        first_min,
        first_max,
        second_min,
        second_max,
        PredicatePolicy,
    )
}

/// Classify the intersection relation between two closed 3D axis-aligned boxes
/// with an explicit predicate escalation policy.
///
/// This is the 3D counterpart to [`classify_aabb2_intersection_with_policy`].
/// It is a certified broad-phase predicate: `Disjoint` may reject a pair, while
/// `Touching` and `Overlapping` are still only candidates for exact
/// narrow-phase predicates before topology is mutated.
pub(crate) fn classify_aabb3_intersection_with_policy(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb3Intersection> {
    let mut trace = DecisionTrace::default();

    let x = match decided(
        classify_closed_interval_intersection_with_policy(
            &first_min.x,
            &first_max.x,
            &second_min.x,
            &second_max.x,
            policy,
        ),
        &mut trace,
    ) {
        Ok(intersection) => intersection,
        Err(unknown) => return unknown.into_outcome(),
    };
    if x == ClosedIntervalIntersection::Disjoint {
        return PredicateOutcome::decided(
            Aabb3Intersection::Disjoint,
            trace.certainty,
            trace.stage,
        );
    }

    let y = match decided(
        classify_closed_interval_intersection_with_policy(
            &first_min.y,
            &first_max.y,
            &second_min.y,
            &second_max.y,
            policy,
        ),
        &mut trace,
    ) {
        Ok(intersection) => intersection,
        Err(unknown) => return unknown.into_outcome(),
    };
    if y == ClosedIntervalIntersection::Disjoint {
        return PredicateOutcome::decided(
            Aabb3Intersection::Disjoint,
            trace.certainty,
            trace.stage,
        );
    }

    let z = match decided(
        classify_closed_interval_intersection_with_policy(
            &first_min.z,
            &first_max.z,
            &second_min.z,
            &second_max.z,
            policy,
        ),
        &mut trace,
    ) {
        Ok(intersection) => intersection,
        Err(unknown) => return unknown.into_outcome(),
    };
    if z == ClosedIntervalIntersection::Disjoint {
        return PredicateOutcome::decided(
            Aabb3Intersection::Disjoint,
            trace.certainty,
            trace.stage,
        );
    }

    let zero_extent_input = match aabb3_has_zero_extent_axis(
        first_min, first_max, second_min, second_max, policy, &mut trace,
    ) {
        Ok(value) => value,
        Err(unknown) => return unknown.into_outcome(),
    };
    let relation = if x == ClosedIntervalIntersection::Touching
        || y == ClosedIntervalIntersection::Touching
        || z == ClosedIntervalIntersection::Touching
        || zero_extent_input
    {
        Aabb3Intersection::Touching
    } else {
        Aabb3Intersection::Overlapping
    };
    PredicateOutcome::decided(relation, trace.certainty, trace.stage)
}

/// Return whether two closed 3D axis-aligned boxes intersect inclusively.
pub fn aabb3s_intersect(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
) -> PredicateOutcome<bool> {
    aabb3s_intersect_with_policy(
        first_min,
        first_max,
        second_min,
        second_max,
        PredicatePolicy,
    )
}

/// Return whether two closed 3D axis-aligned boxes intersect with an explicit
/// predicate escalation policy.
pub(crate) fn aabb3s_intersect_with_policy(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_aabb3_intersection_with_policy(
        first_min, first_max, second_min, second_max, policy,
    ) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.intersects(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn is_interval_boundary(location: RealIntervalLocation) -> bool {
    matches!(
        location,
        RealIntervalLocation::AtLowerEndpoint | RealIntervalLocation::AtUpperEndpoint
    )
}

fn aabb3_has_zero_extent_axis(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(
        interval_has_zero_extent(&first_min.x, &first_max.x, policy, trace)?
            || interval_has_zero_extent(&first_min.y, &first_max.y, policy, trace)?
            || interval_has_zero_extent(&first_min.z, &first_max.z, policy, trace)?
            || interval_has_zero_extent(&second_min.x, &second_max.x, policy, trace)?
            || interval_has_zero_extent(&second_min.y, &second_max.y, policy, trace)?
            || interval_has_zero_extent(&second_min.z, &second_max.z, policy, trace)?,
    )
}

fn interval_has_zero_extent(
    first: &hyperreal::Real,
    second: &hyperreal::Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(decided(compare_reals_with_policy(first, second, policy), trace)? == Ordering::Equal)
}

fn min_max3<'a>(
    first: &'a hyperreal::Real,
    second: &'a hyperreal::Real,
    third: &'a hyperreal::Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<(&'a hyperreal::Real, &'a hyperreal::Real), UnknownDecision> {
    let (mut min, mut max) = (first, first);
    for value in [second, third] {
        if decided(compare_reals_with_policy(value, min, policy), trace)? == Ordering::Less {
            min = value;
        }
        if decided(compare_reals_with_policy(value, max, policy), trace)? == Ordering::Greater {
            max = value;
        }
    }
    Ok((min, max))
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

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    #[test]
    fn point_aabb_classifier_distinguishes_inside_boundary_and_outside() {
        let min = p2(0, 0);
        let max = p2(4, 3);

        assert_eq!(
            classify_point_aabb2(&min, &max, &p2(2, 1)).value(),
            Some(Aabb2PointLocation::Inside)
        );
        assert_eq!(
            classify_point_aabb2(&min, &max, &p2(4, 1)).value(),
            Some(Aabb2PointLocation::Boundary)
        );
        assert_eq!(
            classify_point_aabb2(&max, &min, &p2(5, 1)).value(),
            Some(Aabb2PointLocation::Outside)
        );
        assert_eq!(point_in_aabb2(&min, &max, &p2(4, 1)).value(), Some(true));
    }

    #[test]
    fn point_triangle_aabb_filter_uses_exact_coordinate_bounds() {
        let a = p2(4, 1);
        let b = p2(0, 3);
        let c = p2(2, -1);

        assert_eq!(
            point_in_triangle2_aabb(&a, &b, &c, &p2(2, 1)).value(),
            Some(true)
        );
        assert_eq!(
            point_in_triangle2_aabb(&a, &b, &c, &p2(5, 1)).value(),
            Some(false)
        );
    }

    #[test]
    fn aabb_intersection_distinguishes_disjoint_touching_and_overlap() {
        assert_eq!(
            classify_aabb2_intersection(&p2(0, 0), &p2(2, 2), &p2(3, 0), &p2(5, 2)).value(),
            Some(Aabb2Intersection::Disjoint)
        );
        assert_eq!(
            classify_aabb2_intersection(&p2(0, 0), &p2(2, 2), &p2(2, 1), &p2(4, 3)).value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            classify_aabb2_intersection(&p2(0, 0), &p2(3, 3), &p2(2, 1), &p2(4, 4)).value(),
            Some(Aabb2Intersection::Overlapping)
        );
        assert_eq!(
            aabb2s_intersect(&p2(0, 0), &p2(2, 2), &p2(2, 2), &p2(5, 5)).value(),
            Some(true)
        );
    }

    #[test]
    fn aabb3_intersection_distinguishes_disjoint_touching_and_overlap() {
        assert_eq!(
            classify_aabb3_intersection(&p3(0, 0, 0), &p3(2, 2, 2), &p3(3, 0, 0), &p3(5, 2, 2))
                .value(),
            Some(Aabb3Intersection::Disjoint)
        );
        assert_eq!(
            classify_aabb3_intersection(&p3(0, 0, 0), &p3(2, 2, 2), &p3(2, 1, 1), &p3(4, 3, 3))
                .value(),
            Some(Aabb3Intersection::Touching)
        );
        assert_eq!(
            classify_aabb3_intersection(&p3(0, 0, 0), &p3(3, 3, 3), &p3(2, 1, 1), &p3(4, 4, 4))
                .value(),
            Some(Aabb3Intersection::Overlapping)
        );
        assert_eq!(
            aabb3s_intersect(&p3(0, 0, 0), &p3(2, 2, 2), &p3(2, 2, 2), &p3(5, 5, 5)).value(),
            Some(true)
        );
    }

    #[test]
    fn point_aabb3_classifier_distinguishes_inside_boundary_and_outside() {
        let min = p3(0, 0, 0);
        let max = p3(4, 3, 2);

        assert_eq!(
            classify_point_aabb3(&min, &max, &p3(2, 1, 1)).value(),
            Some(Aabb3PointLocation::Inside)
        );
        assert_eq!(
            classify_point_aabb3(&min, &max, &p3(4, 1, 1)).value(),
            Some(Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            classify_point_aabb3(&max, &min, &p3(5, 1, 1)).value(),
            Some(Aabb3PointLocation::Outside)
        );
        assert_eq!(point_in_aabb3(&min, &max, &p3(4, 1, 1)).value(), Some(true));
    }

    #[test]
    fn immediate_aabb_predicates_accept_cached_extent_facts() {
        let min = p2(0, 0);
        let max = p2(5, 0);
        let facts = crate::geometry::aabb2_facts(&min, &max);

        assert!(facts.known_segment());
        assert!(facts.has_sparse_extent_support());
        assert_eq!(
            classify_point_aabb2(&min, &max, &p2(3, 0)).value(),
            Some(Aabb2PointLocation::Boundary)
        );
        assert_eq!(point_in_aabb2(&min, &max, &p2(6, 0)).value(), Some(false));
    }

    #[test]
    fn immediate_aabb_intersection_preserves_point_segment_area_cases() {
        let point_min = p2(2, 2);
        let point_max = p2(2, 2);
        let segment_min = p2(0, 2);
        let segment_max = p2(4, 2);
        let area_min = p2(1, 1);
        let area_max = p2(3, 3);

        let point_facts = crate::geometry::aabb2_facts(&point_min, &point_max);
        let segment_facts = crate::geometry::aabb2_facts(&segment_min, &segment_max);
        let area_facts = crate::geometry::aabb2_facts(&area_min, &area_max);

        assert!(point_facts.known_point());
        assert!(segment_facts.known_segment());
        assert_eq!(
            classify_aabb2_intersection_with_facts(
                &point_min,
                &point_max,
                &segment_min,
                &segment_max,
                point_facts,
                segment_facts,
            )
            .value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            classify_aabb2_intersection_with_facts(
                &segment_min,
                &segment_max,
                &area_min,
                &area_max,
                segment_facts,
                area_facts,
            )
            .value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            aabb2s_intersect(&area_min, &area_max, &point_min, &point_max).value(),
            Some(true)
        );
    }

    #[test]
    fn immediate_aabb3_predicates_use_borrowed_storage() {
        let min = p3(0, 0, 0);
        let max = p3(4, 4, 4);
        let other_min = p3(4, 1, 1);
        let other_max = p3(6, 3, 3);

        assert_eq!(
            classify_point_aabb3(&min, &max, &p3(2, 2, 2)).value(),
            Some(Aabb3PointLocation::Inside)
        );
        assert_eq!(
            point_in_aabb3(&min, &max, &p3(5, 2, 2)).value(),
            Some(false)
        );
        assert_eq!(
            classify_aabb3_intersection(&min, &max, &other_min, &other_max).value(),
            Some(Aabb3Intersection::Touching)
        );
        assert_eq!(
            aabb3s_intersect(&min, &max, &other_min, &other_max).value(),
            Some(true)
        );
    }
}
