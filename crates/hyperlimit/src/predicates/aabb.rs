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
use hyperreal::Real;

/// Return whether a point lies in an ordered closed 2D box using borrowed
/// coordinates.
///
/// `min[axis] <= max[axis]` is a caller-provided precondition. This
/// coordinate-borrowed form lets curve and broad-phase crates reuse the
/// canonical Hyperlimit cascade without cloning their own point carriers into
/// [`Point2`].
/// Policy-controlled ordered-coordinate AABB predicate.
#[inline]
pub fn point_in_ordered_aabb2_coordinates_with_policy(
    min: [&Real; 2],
    max: [&Real; 2],
    point: [&Real; 2],
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    if let (Some(min), Some(max), Some(point)) = (
        exact_rational_coordinates2(min),
        exact_rational_coordinates2(max),
        exact_rational_coordinates2(point),
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "point_in_ordered_aabb2_coordinates",
            "exact-rational"
        );
        let inside = (0..2).all(|axis| min[axis] <= point[axis] && point[axis] <= max[axis]);
        return PredicateOutcome::decided(inside, Certainty::Exact, Escalation::Exact);
    }

    let mut trace = DecisionTrace::default();
    for axis in 0..2 {
        let below_minimum = match decided(
            compare_reals_with_policy(point[axis], min[axis], policy),
            &mut trace,
        ) {
            Ok(ordering) => ordering == Ordering::Less,
            Err(unknown) => return unknown.into_outcome(),
        };
        if below_minimum {
            return PredicateOutcome::decided(false, trace.certainty, trace.stage);
        }

        let above_maximum = match decided(
            compare_reals_with_policy(point[axis], max[axis], policy),
            &mut trace,
        ) {
            Ok(ordering) => ordering == Ordering::Greater,
            Err(unknown) => return unknown.into_outcome(),
        };
        if above_maximum {
            return PredicateOutcome::decided(false, trace.certainty, trace.stage);
        }
    }

    PredicateOutcome::decided(true, trace.certainty, trace.stage)
}

/// Return whether two ordered closed 2D boxes intersect inclusively using
/// borrowed coordinates.
///
/// Both min/max pairs must already be ordered on every axis. Edge and corner
/// contact count as intersection.
/// Policy-controlled ordered-coordinate AABB intersection predicate.
#[inline]
pub fn ordered_aabb2s_intersect_coordinates_with_policy(
    first_min: [&Real; 2],
    first_max: [&Real; 2],
    second_min: [&Real; 2],
    second_max: [&Real; 2],
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    if let (Some(first_min), Some(first_max), Some(second_min), Some(second_max)) = (
        exact_rational_coordinates2(first_min),
        exact_rational_coordinates2(first_max),
        exact_rational_coordinates2(second_min),
        exact_rational_coordinates2(second_max),
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "ordered_aabb2s_intersect_coordinates",
            "exact-rational"
        );
        let intersects = (0..2)
            .all(|axis| first_max[axis] >= second_min[axis] && second_max[axis] >= first_min[axis]);
        return PredicateOutcome::decided(intersects, Certainty::Exact, Escalation::Exact);
    }

    let mut trace = DecisionTrace::default();
    for axis in 0..2 {
        for (left, right) in [
            (first_max[axis], second_min[axis]),
            (second_max[axis], first_min[axis]),
        ] {
            let separated =
                match decided(compare_reals_with_policy(left, right, policy), &mut trace) {
                    Ok(ordering) => ordering == Ordering::Less,
                    Err(unknown) => return unknown.into_outcome(),
                };
            if separated {
                return PredicateOutcome::decided(false, trace.certainty, trace.stage);
            }
        }
    }

    PredicateOutcome::decided(true, trace.certainty, trace.stage)
}

/// Classify a point relative to a closed 2D axis-aligned box with an explicit
/// predicate escalation policy.
///
/// The min/max corners may be supplied in either coordinate order; each axis is
/// normalized by exact interval predicates. These box predicates are safe
/// broad-phase filters for arrangements, curve intersection, and triangulation
/// candidate pruning. Boxes reduce candidate sets, but final topology still
/// belongs to orientation and incidence predicates.
pub fn classify_point_aabb2_with_policy(
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

/// Return whether a point lies in a closed 2D axis-aligned box with an explicit
/// predicate escalation policy.
pub fn point_in_aabb2_with_policy(
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
/// 2D triangle with an explicit predicate escalation policy.
///
/// This is a rejection filter for downstream exact triangle predicates. Every
/// min/max and interval decision is exact, preserving the boundary between
/// filters and final topology.
pub fn point_in_triangle2_aabb_with_policy(
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

/// Classify a point relative to a closed 3D axis-aligned box with an explicit
/// predicate escalation policy.
///
/// The min/max corners may be supplied in either coordinate order; each axis is
/// normalized by exact interval predicates.
pub fn classify_point_aabb3_with_policy(
    min: &Point3,
    max: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb3PointLocation> {
    if let (Some(min), Some(max), Some(point)) = (
        exact_rational_coordinates3([&min.x, &min.y, &min.z]),
        exact_rational_coordinates3([&max.x, &max.y, &max.z]),
        exact_rational_coordinates3([&point.x, &point.y, &point.z]),
    ) {
        crate::trace_dispatch!("hyperlimit", "classify_point_aabb3", "exact-rational");
        let mut boundary = false;
        for axis in 0..3 {
            let (lower, upper) = if min[axis] <= max[axis] {
                (min[axis], max[axis])
            } else {
                (max[axis], min[axis])
            };
            if point[axis] < lower || point[axis] > upper {
                return PredicateOutcome::decided(
                    Aabb3PointLocation::Outside,
                    Certainty::Exact,
                    Escalation::Exact,
                );
            }
            boundary |= point[axis] == lower || point[axis] == upper;
        }
        return PredicateOutcome::decided(
            if boundary {
                Aabb3PointLocation::Boundary
            } else {
                Aabb3PointLocation::Inside
            },
            Certainty::Exact,
            Escalation::Exact,
        );
    }

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

/// Return whether a point lies in the relative interior of an ordered 3D box.
///
/// `min` and `max` must be ordered on every axis. Positive-width axes use open
/// interval membership, while zero-width axes require exact equality. This is
/// the relative-interior predicate used by lower-dimensional subdivision
/// cells embedded in 3D.
#[inline]
pub fn point_in_ordered_aabb3_relative_interior_with_policy(
    min: &Point3,
    max: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    let min = [&min.x, &min.y, &min.z];
    let max = [&max.x, &max.y, &max.z];
    let point = [&point.x, &point.y, &point.z];

    if let (Some(min), Some(max), Some(point)) = (
        exact_rational_coordinates3(min),
        exact_rational_coordinates3(max),
        exact_rational_coordinates3(point),
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "point_in_ordered_aabb3_relative_interior",
            "exact-rational"
        );
        let inside = (0..3).all(|axis| {
            if min[axis] == max[axis] {
                point[axis] == min[axis]
            } else {
                min[axis] < point[axis] && point[axis] < max[axis]
            }
        });
        return PredicateOutcome::decided(inside, Certainty::Exact, Escalation::Exact);
    }

    let mut trace = DecisionTrace::default();
    for axis in 0..3 {
        let extent = match decided(
            compare_reals_with_policy(min[axis], max[axis], policy),
            &mut trace,
        ) {
            Ok(ordering) => ordering,
            Err(unknown) => return unknown.into_outcome(),
        };
        if extent == Ordering::Equal {
            let on_axis = match decided(
                compare_reals_with_policy(point[axis], min[axis], policy),
                &mut trace,
            ) {
                Ok(ordering) => ordering == Ordering::Equal,
                Err(unknown) => return unknown.into_outcome(),
            };
            if !on_axis {
                return PredicateOutcome::decided(false, trace.certainty, trace.stage);
            }
            continue;
        }

        let above_minimum = match decided(
            compare_reals_with_policy(point[axis], min[axis], policy),
            &mut trace,
        ) {
            Ok(ordering) => ordering == Ordering::Greater,
            Err(unknown) => return unknown.into_outcome(),
        };
        if !above_minimum {
            return PredicateOutcome::decided(false, trace.certainty, trace.stage);
        }
        let below_maximum = match decided(
            compare_reals_with_policy(point[axis], max[axis], policy),
            &mut trace,
        ) {
            Ok(ordering) => ordering == Ordering::Less,
            Err(unknown) => return unknown.into_outcome(),
        };
        if !below_maximum {
            return PredicateOutcome::decided(false, trace.certainty, trace.stage);
        }
    }
    PredicateOutcome::decided(true, trace.certainty, trace.stage)
}

/// Return whether a point lies in a closed 3D axis-aligned box with an explicit
/// predicate escalation policy.
pub fn point_in_aabb3_with_policy(
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

/// Classify the intersection relation between two closed 2D axis-aligned boxes
/// with an explicit predicate escalation policy.
///
/// `Touching` covers edge and corner contact with zero area. `Overlapping`
/// means both coordinate intervals overlap over positive length, so the box
/// intersection has positive area.
pub fn classify_aabb2_intersection_with_policy(
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
        crate::geometry::aabb2_facts(first_min, first_max),
        crate::geometry::aabb2_facts(second_min, second_max),
        policy,
    )
}

/// Classify the intersection relation between two closed 2D axis-aligned boxes
/// with an explicit policy and caller-cached structural facts.
///
/// The facts are used only after exact interval predicates prove both axes
/// intersect. A structurally zero-area input box cannot have a positive-area
/// box intersection, so the final relation is `Touching` rather than
/// `Overlapping`. This is a local exact specialization of the box broad phase;
/// uncertain extent facts do not decide topology by themselves.
pub fn classify_aabb2_intersection_with_policy_and_facts(
    first_min: &Point2,
    first_max: &Point2,
    second_min: &Point2,
    second_max: &Point2,
    first_facts: Aabb2Facts,
    second_facts: Aabb2Facts,
    policy: PredicatePolicy,
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

/// Return whether two closed 2D axis-aligned boxes intersect with an explicit
/// predicate escalation policy.
pub fn aabb2s_intersect_with_policy(
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

/// Classify the intersection relation between two closed 3D axis-aligned boxes
/// with an explicit predicate escalation policy.
///
/// This is the 3D counterpart to [`classify_aabb2_intersection_with_policy`].
/// It is a certified broad-phase predicate: `Disjoint` may reject a pair, while
/// `Touching` and `Overlapping` are still only candidates for exact
/// narrow-phase predicates before topology is mutated.
pub fn classify_aabb3_intersection_with_policy(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Aabb3Intersection> {
    if let (Some(first_min), Some(first_max), Some(second_min), Some(second_max)) = (
        exact_rational_coordinates3([&first_min.x, &first_min.y, &first_min.z]),
        exact_rational_coordinates3([&first_max.x, &first_max.y, &first_max.z]),
        exact_rational_coordinates3([&second_min.x, &second_min.y, &second_min.z]),
        exact_rational_coordinates3([&second_max.x, &second_max.y, &second_max.z]),
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "classify_aabb3_intersection",
            "exact-rational"
        );
        let mut touching = false;
        for axis in 0..3 {
            let (first_lower, first_upper) = if first_min[axis] <= first_max[axis] {
                (first_min[axis], first_max[axis])
            } else {
                (first_max[axis], first_min[axis])
            };
            let (second_lower, second_upper) = if second_min[axis] <= second_max[axis] {
                (second_min[axis], second_max[axis])
            } else {
                (second_max[axis], second_min[axis])
            };
            if first_upper < second_lower || second_upper < first_lower {
                return PredicateOutcome::decided(
                    Aabb3Intersection::Disjoint,
                    Certainty::Exact,
                    Escalation::Exact,
                );
            }
            touching |= first_upper == second_lower
                || second_upper == first_lower
                || first_lower == first_upper
                || second_lower == second_upper;
        }
        return PredicateOutcome::decided(
            if touching {
                Aabb3Intersection::Touching
            } else {
                Aabb3Intersection::Overlapping
            },
            Certainty::Exact,
            Escalation::Exact,
        );
    }

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

/// Return whether two ordered closed 3D boxes intersect inclusively.
///
/// Both min/max pairs must already be ordered on every axis. Skipping interval
/// normalization makes this the canonical broad-phase predicate for retained
/// AABB structures.
#[inline]
pub fn ordered_aabb3s_intersect_with_policy(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    ordered_aabb3_pairwise_relation(
        first_min,
        first_max,
        second_min,
        second_max,
        policy,
        OrderedAabb3Relation::Intersects,
    )
}

/// Return whether one ordered closed 3D box contains another inclusively.
///
/// Both min/max pairs must already be ordered on every axis.
#[inline]
pub fn ordered_aabb3_contains_with_policy(
    outer_min: &Point3,
    outer_max: &Point3,
    inner_min: &Point3,
    inner_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    ordered_aabb3_pairwise_relation(
        outer_min,
        outer_max,
        inner_min,
        inner_max,
        policy,
        OrderedAabb3Relation::Contains,
    )
}

#[derive(Clone, Copy)]
enum OrderedAabb3Relation {
    Intersects,
    Contains,
}

#[inline]
fn ordered_aabb3_pairwise_relation(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
    relation: OrderedAabb3Relation,
) -> PredicateOutcome<bool> {
    let first_min = [&first_min.x, &first_min.y, &first_min.z];
    let first_max = [&first_max.x, &first_max.y, &first_max.z];
    let second_min = [&second_min.x, &second_min.y, &second_min.z];
    let second_max = [&second_max.x, &second_max.y, &second_max.z];

    if let (Some(first_min), Some(first_max), Some(second_min), Some(second_max)) = (
        exact_rational_coordinates3(first_min),
        exact_rational_coordinates3(first_max),
        exact_rational_coordinates3(second_min),
        exact_rational_coordinates3(second_max),
    ) {
        crate::trace_dispatch!("hyperlimit", "ordered_aabb3_relation", "exact-rational");
        let value = match relation {
            OrderedAabb3Relation::Intersects => (0..3).all(|axis| {
                first_max[axis] >= second_min[axis] && second_max[axis] >= first_min[axis]
            }),
            OrderedAabb3Relation::Contains => (0..3).all(|axis| {
                first_min[axis] <= second_min[axis] && first_max[axis] >= second_max[axis]
            }),
        };
        return PredicateOutcome::decided(value, Certainty::Exact, Escalation::Exact);
    }

    let mut trace = DecisionTrace::default();
    for axis in 0..3 {
        let comparisons = match relation {
            OrderedAabb3Relation::Intersects => [
                (first_max[axis], second_min[axis], Ordering::Less),
                (second_max[axis], first_min[axis], Ordering::Less),
            ],
            OrderedAabb3Relation::Contains => [
                (first_min[axis], second_min[axis], Ordering::Greater),
                (first_max[axis], second_max[axis], Ordering::Less),
            ],
        };
        for (left, right, rejecting_ordering) in comparisons {
            let ordering = match decided(compare_reals_with_policy(left, right, policy), &mut trace)
            {
                Ok(ordering) => ordering,
                Err(unknown) => return unknown.into_outcome(),
            };
            if ordering == rejecting_ordering {
                return PredicateOutcome::decided(false, trace.certainty, trace.stage);
            }
        }
    }
    PredicateOutcome::decided(true, trace.certainty, trace.stage)
}

/// Return whether two closed 3D axis-aligned boxes intersect with an explicit
/// predicate escalation policy.
pub fn aabb3s_intersect_with_policy(
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

#[inline]
fn exact_rational_coordinates2(coordinates: [&Real; 2]) -> Option<[&hyperreal::Rational; 2]> {
    let [Some(x), Some(y)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    Some([x, y])
}

#[inline]
fn exact_rational_coordinates3(coordinates: [&Real; 3]) -> Option<[&hyperreal::Rational; 3]> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    Some([x, y, z])
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
        Certainty::Approximate => 2,
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

    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

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
            crate::classify_point_aabb2(&min, &max, &p2(2, 1), APPROX).value(),
            Some(Aabb2PointLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_aabb2(&min, &max, &p2(4, 1), APPROX).value(),
            Some(Aabb2PointLocation::Boundary)
        );
        assert_eq!(
            crate::classify_point_aabb2(&max, &min, &p2(5, 1), APPROX).value(),
            Some(Aabb2PointLocation::Outside)
        );
        assert_eq!(
            crate::point_in_aabb2(&min, &max, &p2(4, 1), APPROX).value(),
            Some(true)
        );
    }

    #[test]
    fn point_triangle_aabb_filter_uses_exact_coordinate_bounds() {
        let a = p2(4, 1);
        let b = p2(0, 3);
        let c = p2(2, -1);

        assert_eq!(
            crate::point_in_triangle2_aabb(&a, &b, &c, &p2(2, 1), APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::point_in_triangle2_aabb(&a, &b, &c, &p2(5, 1), APPROX).value(),
            Some(false)
        );
    }

    #[test]
    fn aabb_intersection_distinguishes_disjoint_touching_and_overlap() {
        assert_eq!(
            crate::classify_aabb2_intersection(&p2(0, 0), &p2(2, 2), &p2(3, 0), &p2(5, 2), APPROX)
                .value(),
            Some(Aabb2Intersection::Disjoint)
        );
        assert_eq!(
            crate::classify_aabb2_intersection(&p2(0, 0), &p2(2, 2), &p2(2, 1), &p2(4, 3), APPROX)
                .value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            crate::classify_aabb2_intersection(&p2(0, 0), &p2(3, 3), &p2(2, 1), &p2(4, 4), APPROX)
                .value(),
            Some(Aabb2Intersection::Overlapping)
        );
        assert_eq!(
            crate::aabb2s_intersect(&p2(0, 0), &p2(2, 2), &p2(2, 2), &p2(5, 5), APPROX).value(),
            Some(true)
        );
    }

    #[test]
    fn aabb3_intersection_distinguishes_disjoint_touching_and_overlap() {
        assert_eq!(
            crate::classify_aabb3_intersection(
                &p3(0, 0, 0),
                &p3(2, 2, 2),
                &p3(3, 0, 0),
                &p3(5, 2, 2),
                APPROX
            )
            .value(),
            Some(Aabb3Intersection::Disjoint)
        );
        assert_eq!(
            crate::classify_aabb3_intersection(
                &p3(0, 0, 0),
                &p3(2, 2, 2),
                &p3(2, 1, 1),
                &p3(4, 3, 3),
                APPROX
            )
            .value(),
            Some(Aabb3Intersection::Touching)
        );
        assert_eq!(
            crate::classify_aabb3_intersection(
                &p3(0, 0, 0),
                &p3(3, 3, 3),
                &p3(2, 1, 1),
                &p3(4, 4, 4),
                APPROX
            )
            .value(),
            Some(Aabb3Intersection::Overlapping)
        );
        assert_eq!(
            crate::aabb3s_intersect(
                &p3(0, 0, 0),
                &p3(2, 2, 2),
                &p3(2, 2, 2),
                &p3(5, 5, 5),
                APPROX
            )
            .value(),
            Some(true)
        );
    }

    #[test]
    fn point_aabb3_classifier_distinguishes_inside_boundary_and_outside() {
        let min = p3(0, 0, 0);
        let max = p3(4, 3, 2);

        assert_eq!(
            crate::classify_point_aabb3(&min, &max, &p3(2, 1, 1), APPROX).value(),
            Some(Aabb3PointLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_aabb3(&min, &max, &p3(4, 1, 1), APPROX).value(),
            Some(Aabb3PointLocation::Boundary)
        );
        assert_eq!(
            crate::classify_point_aabb3(&max, &min, &p3(5, 1, 1), APPROX).value(),
            Some(Aabb3PointLocation::Outside)
        );
        assert_eq!(
            crate::point_in_aabb3(&min, &max, &p3(4, 1, 1), APPROX).value(),
            Some(true)
        );
    }

    #[test]
    fn ordered_aabb3_predicates_cover_overlap_containment_and_relative_interior() {
        let outer_min = p3(0, 0, 0);
        let outer_max = p3(4, 4, 4);
        let inner_min = p3(1, 0, 1);
        let inner_max = p3(3, 0, 3);

        assert_eq!(
            crate::ordered_aabb3s_intersect(&outer_min, &outer_max, &inner_min, &inner_max, APPROX)
                .value(),
            Some(true)
        );
        assert_eq!(
            crate::ordered_aabb3_contains(&outer_min, &outer_max, &inner_min, &inner_max, APPROX)
                .value(),
            Some(true)
        );
        assert_eq!(
            crate::ordered_aabb3s_intersect(
                &outer_min,
                &outer_max,
                &p3(5, 1, 1),
                &p3(6, 2, 2),
                APPROX
            )
            .value(),
            Some(false)
        );
        assert_eq!(
            crate::point_in_ordered_aabb3_relative_interior(
                &inner_min,
                &inner_max,
                &p3(2, 0, 2),
                APPROX
            )
            .value(),
            Some(true)
        );
        assert_eq!(
            crate::point_in_ordered_aabb3_relative_interior(
                &inner_min,
                &inner_max,
                &p3(1, 0, 2),
                APPROX
            )
            .value(),
            Some(false)
        );
        assert_eq!(
            crate::point_in_ordered_aabb3_relative_interior(
                &inner_min,
                &inner_max,
                &p3(2, 1, 2),
                APPROX
            )
            .value(),
            Some(false)
        );
    }

    #[test]
    fn ordered_aabb2_coordinate_predicates_cover_overlap_and_point_membership() {
        let outer_min = p2(0, 0);
        let outer_max = p2(4, 4);
        let touching_min = p2(4, 1);
        let touching_max = p2(6, 3);
        let disjoint_min = p2(5, 1);
        let disjoint_max = p2(6, 3);
        let inside = p2(2, 3);
        let boundary = p2(4, 3);
        let outside = p2(5, 3);

        fn coordinates(point: &Point2) -> [&Real; 2] {
            [&point.x, &point.y]
        }
        assert_eq!(
            crate::ordered_aabb2s_intersect_coordinates(
                coordinates(&outer_min),
                coordinates(&outer_max),
                coordinates(&touching_min),
                coordinates(&touching_max),
                APPROX
            )
            .value(),
            Some(true)
        );
        assert_eq!(
            crate::ordered_aabb2s_intersect_coordinates(
                coordinates(&outer_min),
                coordinates(&outer_max),
                coordinates(&disjoint_min),
                coordinates(&disjoint_max),
                APPROX
            )
            .value(),
            Some(false)
        );
        for (point, expected) in [(&inside, true), (&boundary, true), (&outside, false)] {
            assert_eq!(
                crate::point_in_ordered_aabb2_coordinates(
                    coordinates(&outer_min),
                    coordinates(&outer_max),
                    coordinates(point),
                    APPROX
                )
                .value(),
                Some(expected)
            );
        }
    }

    #[test]
    fn immediate_aabb_predicates_accept_cached_extent_facts() {
        let min = p2(0, 0);
        let max = p2(5, 0);
        let facts = crate::geometry::aabb2_facts(&min, &max);

        assert!(facts.known_segment());
        assert!(facts.has_sparse_extent_support());
        assert_eq!(
            crate::classify_point_aabb2(&min, &max, &p2(3, 0), APPROX).value(),
            Some(Aabb2PointLocation::Boundary)
        );
        assert_eq!(
            crate::point_in_aabb2(&min, &max, &p2(6, 0), APPROX).value(),
            Some(false)
        );
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
            crate::classify_aabb2_intersection_with_facts(
                &point_min,
                &point_max,
                &segment_min,
                &segment_max,
                point_facts,
                segment_facts,
                APPROX
            )
            .value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            crate::classify_aabb2_intersection_with_facts(
                &segment_min,
                &segment_max,
                &area_min,
                &area_max,
                segment_facts,
                area_facts,
                APPROX
            )
            .value(),
            Some(Aabb2Intersection::Touching)
        );
        assert_eq!(
            crate::aabb2s_intersect(&area_min, &area_max, &point_min, &point_max, APPROX).value(),
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
            crate::classify_point_aabb3(&min, &max, &p3(2, 2, 2), APPROX).value(),
            Some(Aabb3PointLocation::Inside)
        );
        assert_eq!(
            crate::point_in_aabb3(&min, &max, &p3(5, 2, 2), APPROX).value(),
            Some(false)
        );
        assert_eq!(
            crate::classify_aabb3_intersection(&min, &max, &other_min, &other_max, APPROX).value(),
            Some(Aabb3Intersection::Touching)
        );
        assert_eq!(
            crate::aabb3s_intersect(&min, &max, &other_min, &other_max, APPROX).value(),
            Some(true)
        );
    }
}
