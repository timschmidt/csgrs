//! Borrowed-coordinate predicates for closed 2D and 3D axis-aligned boxes.
//!
//! This module supplies decisions rather than a bounding-box storage type.
//! General point/intersection queries normalize each axis through comparisons;
//! APIs named ordered require the caller to supply ordered endpoints and do not
//! validate that precondition. Collapsed boxes remain geometric sets, not an
//! empty-box encoding. Relative interior has a separate 3D predicate.
//!
//! Selected paths first borrow structurally available rational coordinates and
//! return Exact/Exact results. Generic paths consume comparisons under the
//! supplied policy and join their weakest certainty and latest stage. They stop
//! at the first rejection or Unknown in axis order; no strict-first composite
//! replay is performed here. Approximate results retain their qualification.
//!
//! A box decision can reject geometry only with admissible certainty and valid
//! enclosing bounds; overlap does not establish contact of the bounded objects.
//! The 2D intersection classifier additionally trusts structural extent facts,
//! with the limitations documented on its facts-bearing entry point.

use crate::classify::{
    Aabb2Intersection, Aabb2PointLocation, Aabb3Intersection, Aabb3PointLocation,
    ClosedIntervalIntersection, RealIntervalLocation,
};
use crate::geometry::{Aabb2Facts, Point2, Point3};
use crate::predicate::PredicatePolicy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome};
use crate::predicates::decision::{DecisionTrace, UnknownDecision, decided};
use crate::predicates::interval::{
    classify_closed_interval_intersection_with_extent_policy,
    classify_closed_interval_intersection_with_policy, classify_real_closed_interval_with_policy,
};
use crate::predicates::order::compare_reals_with_policy;
use core::cmp::Ordering;
use hyperreal::Real;

/// Test inclusive point membership in an ordered 2D box without building Point2 values.
///
/// Require `min[axis] <= max[axis]`; endpoint order is neither normalized nor
/// validated. If every coordinate exposes a borrowed rational, use rational
/// comparisons with an Exact/Exact receipt. Otherwise compare point to minimum
/// then maximum on x before y, returning on the first outside result or Unknown.
/// Successful paths retain all consumed comparison qualification.
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

/// Test inclusive intersection of two ordered 2D boxes from borrowed coordinates.
///
/// Both endpoint pairs must be ordered on each axis; this is not checked.
/// Edge/corner contact counts. A fully borrowed-rational path returns Exact/Exact.
/// Otherwise test first maximum against second minimum, then second maximum
/// against first minimum, on x before y. The first strict separation or Unknown
/// returns immediately; decided outcomes join consumed certainty and stage.
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

/// Classify a point in the closed box spanned by either ordering of two corners.
///
/// Normalize and classify the x interval, then y. The first outside or Unknown
/// result stops evaluation. A retained endpoint label on either axis yields
/// Boundary, otherwise Inside; points on collapsed boxes are therefore Boundary.
/// All consumed decisions follow `policy`, and their joined receipt can be
/// approximate. This tests box membership, not the geometry enclosed by a bound.
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

/// Map 2D box Inside/Boundary to true, retaining the classifier's receipt or Unknown.
///
/// Corners may be reversed; normalization belongs to the delegated classifier.
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

/// Test the query against the inclusive coordinate bounds of three planar points.
///
/// Borrow minima/maxima on x first and test membership before doing any y bounds.
/// For each axis, visit b then c, comparing against the current minimum then
/// maximum; equal values retain the earlier endpoint. Unknown exits immediately.
/// All extent-selection and interval receipts are joined, including approximation.
/// Degenerate triples are allowed. True only establishes bounding-box membership,
/// not triangle membership; no orientation or incidence decision is performed.
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

/// Classify point membership in a closed 3D box with per-axis corner normalization.
///
/// If all nine coordinates expose borrowed rationals, compare them directly and
/// return Exact/Exact. Otherwise classify intervals in x/y/z order under policy.
/// Stop at the first Outside or Unknown; if all axes contain the point, any endpoint
/// match yields Boundary, otherwise Inside. Collapsed-axis membership is Boundary,
/// not relative interior. The generic result retains its weakest consumed certainty
/// and latest stage rather than claiming every comparison was exact.
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

/// Test relative interior of an ordered box, including lower-dimensional cells.
///
/// Require ordered endpoints; the precondition is not validated or normalized.
/// Positive-width axes require strict interior membership, while zero-width axes
/// require equality. Thus a point box contains its point in relative interior.
/// Borrowed-rational coordinates use direct comparisons and Exact/Exact receipts.
/// The generic path visits x/y/z, compares extent first, then checks the equality
/// or open-interval condition. The first false or Unknown returns; all consumed
/// certainty/stages are joined, so terminal extent equality remains approximate.
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

/// Map 3D box Inside/Boundary to true without discarding its receipt or Unknown.
///
/// The delegated classifier handles reversed corners and collapsed axes.
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

/// Classify two closed 2D boxes after building structural facts for both extents.
///
/// Both fact sets are computed before interval classification, even if x later
/// separates. Endpoints may be reversed; interval predicates normalize them.
/// Delegate to [`classify_aabb2_intersection_with_policy_and_facts`] for the
/// Touching/Overlapping distinction, including its zero-area recognition limit.
/// This is not an unconditional positive-area certificate for every Overlapping
/// label, and policy-authorized approximation remains in the outcome receipt.
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

/// Combine axis interval relations with caller-associated structural extent facts.
///
/// Normalize and classify x, then y, stopping at the first Disjoint or Unknown.
/// After both axes intersect, an interval Touching label or either
/// `known_zero_area() == Some(true)` produces Touching; otherwise return Overlapping.
/// Only that fact field is consumed. Its truth and association with the current
/// corners are trusted, not checked, so mutation requires fresh valid facts.
///
/// The interval relation alone can label a contained singleton Overlapping. This
/// 2D path drops the interval helper's zero-extent flags and uses structural facts
/// instead. None is not a proof of nonzero extent, but this branch treats it
/// the same as false.
/// Consequently Overlapping is not by itself a positive-area proof when zero-area
/// facts are unavailable. Child decision receipts are joined, but they do not
/// certify caller-authored or stale facts. Approximation remains qualified.
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

/// Map both non-Disjoint 2D box relations to true, preserving receipt or Unknown.
///
/// This inclusive test uses general corner normalization and includes collapsed
/// point/segment boxes. It does not require positive intersection area.
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

/// Classify two closed 3D boxes, including contact on collapsed axes.
///
/// Normalize both corner pairs on each axis. A fully borrowed-rational path returns
/// Exact/Exact after direct comparisons. Otherwise visit x/y/z interval queries,
/// retaining their zero-extent flags as well as their relations and receipts.
/// The first separation or Unknown stops evaluation. If all axes meet, endpoint
/// contact or any zero-width input axis yields Touching; otherwise Overlapping.
/// With certified comparisons this distinguishes zero from positive intersection
/// volume. Approximate comparisons qualify the result. For enclosing bounds,
/// non-Disjoint remains only a candidate for the objects' narrow-phase query.
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

    let (x, x_zero_extent) = match decided(
        classify_closed_interval_intersection_with_extent_policy(
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

    let (y, y_zero_extent) = match decided(
        classify_closed_interval_intersection_with_extent_policy(
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

    let (z, z_zero_extent) = match decided(
        classify_closed_interval_intersection_with_extent_policy(
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

    let zero_extent_input = x_zero_extent || y_zero_extent || z_zero_extent;
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

/// Borrow two ordered point-corner pairs and test inclusive 3D box intersection.
///
/// Both boxes must already have ordered endpoints on every axis. Delegate to the
/// coordinate form without cloning corners, preserving its outcome and receipt.
#[inline]
pub fn ordered_aabb3s_intersect_with_policy(
    first_min: &Point3,
    first_max: &Point3,
    second_min: &Point3,
    second_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    ordered_aabb3s_intersect_coordinates_with_policy(
        [&first_min.x, &first_min.y, &first_min.z],
        [&first_max.x, &first_max.y, &first_max.z],
        [&second_min.x, &second_min.y, &second_min.z],
        [&second_max.x, &second_max.y, &second_max.z],
        policy,
    )
}

/// Test inclusive intersection of ordered 3D boxes using borrowed coordinate arrays.
///
/// Both endpoint pairs must be ordered; no normalization or order validation occurs.
/// Direct rational comparisons return Exact/Exact. Otherwise visit x/y/z and test
/// first maximum against second minimum before second maximum against first minimum.
/// Stop at the first strict separation or Unknown, retaining consumed qualification.
/// Touching and collapsed intersections count as true.
#[inline]
pub fn ordered_aabb3s_intersect_coordinates_with_policy(
    first_min: [&Real; 3],
    first_max: [&Real; 3],
    second_min: [&Real; 3],
    second_max: [&Real; 3],
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

/// Test inclusive containment of one ordered 3D box by another.
///
/// Require both min/max pairs ordered on every axis; this is not validated.
/// The rational path returns Exact/Exact. Otherwise, in x/y/z order, reject when
/// outer minimum is greater than inner minimum or outer maximum is less than inner
/// maximum. Unknown stops before later comparisons; successful paths retain joined
/// qualification. Equal faces and contained lower-dimensional boxes are allowed.
#[inline]
pub fn ordered_aabb3_contains_with_policy(
    outer_min: &Point3,
    outer_max: &Point3,
    inner_min: &Point3,
    inner_max: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    ordered_aabb3_pairwise_relation(
        [&outer_min.x, &outer_min.y, &outer_min.z],
        [&outer_max.x, &outer_max.y, &outer_max.z],
        [&inner_min.x, &inner_min.y, &inner_min.z],
        [&inner_max.x, &inner_max.y, &inner_max.z],
        policy,
        OrderedAabb3Relation::Contains,
    )
}

/// Select the comparison schedule for ordered 3D box overlap or containment.
#[derive(Clone, Copy)]
enum OrderedAabb3Relation {
    /// Require opposing face intervals to meet on every axis.
    Intersects,
    /// Require the first box to contain the second on every axis.
    Contains,
}

/// Share rational borrowing and ordered-axis traversal for two relation schedules.
///
/// Endpoints are trusted ordered. Intersects compares opposing faces; Contains
/// compares corresponding minima then maxima. Rational borrowing succeeds only
/// for the complete coordinate tuple. The generic path uses a non-inlined scalar
/// comparison boundary, short-circuiting the first rejecting ordering or Unknown
/// and joining successful receipts. It does not run a separate strict replay.
#[inline]
fn ordered_aabb3_pairwise_relation(
    first_min: [&Real; 3],
    first_max: [&Real; 3],
    second_min: [&Real; 3],
    second_max: [&Real; 3],
    policy: PredicatePolicy,
    relation: OrderedAabb3Relation,
) -> PredicateOutcome<bool> {
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
            let ordering = match decided(compare_aabb_reals(left, right, policy), &mut trace) {
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

/// Forward policy-controlled comparison through a deliberately non-inlined boundary.
///
/// This keeps the full scalar cascade out of each comparison site in the ordered
/// box loop. The boundary is a code-layout choice; it changes neither the delegated
/// value nor its receipt, and does not itself establish a performance measurement.
#[inline(never)]
fn compare_aabb_reals(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    // Keep the complete comparison cascade behind one stable call boundary.
    // Otherwise consumer-side inlining can expand it into the six-comparison
    // AABB loop, increasing broad-phase instructions and linked code.
    compare_reals_with_policy(left, right, policy)
}

/// Map Touching/Overlapping 3D boxes to true, retaining the classifier's receipt.
///
/// General corner normalization and zero-width axes are handled by the classifier;
/// Unknown remains Unknown rather than being coerced to a boolean.
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

/// Recognize either endpoint label, excluding interval interior and exterior labels.
fn is_interval_boundary(location: RealIntervalLocation) -> bool {
    matches!(
        location,
        RealIntervalLocation::AtLowerEndpoint | RealIntervalLocation::AtUpperEndpoint
    )
}

/// Borrow both structurally available rational coordinates, or return None.
///
/// This uses `Real::exact_rational_ref`, not general symbolic rational reduction.
/// It does not clone or round coordinates; rational arithmetic may still have
/// variable cost, including for lazy rational storage.
#[inline]
fn exact_rational_coordinates2(coordinates: [&Real; 2]) -> Option<[&hyperreal::Rational; 2]> {
    let [Some(x), Some(y)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    Some([x, y])
}

/// Borrow all three structurally available rationals, declining if any is unavailable.
///
/// Symbolic values remain on the Real comparison path even when another reducer
/// could prove them rational. No conversion, rounding, or denominator normalization
/// is forced by this coordinate-borrowing helper.
#[inline]
fn exact_rational_coordinates3(coordinates: [&Real; 3]) -> Option<[&hyperreal::Rational; 3]> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    Some([x, y, z])
}

/// Borrow extrema by comparing second then third against the evolving bounds.
///
/// Each value is compared with the current minimum before the current maximum.
/// Ties retain the earlier reference. Decided comparisons update the caller's trace;
/// Unknown returns immediately, leaving any prior trace updates intact.
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

/// Finite box-relation, rational/symbolic, policy-qualification, and receipt-join controls.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::RefinementNeed;
    use crate::predicates::decision::{certainty_rank, max_certainty, max_stage, stage_rank};

    /// Approximation-permitted policy used by rational controls and explicit terminal cases.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Assert the normalized trigonometric identity, then return a harder angle-addition zero.
    fn terminally_unresolved_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        // Preserve the original identity as a newly certified proof case.
        assert_eq!(
            original.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        // A separate angle-addition identity retains the terminal-policy control.
        crate::test_support::terminally_unresolved_zero()
    }

    /// Construct an exact integer scalar for box fixtures.
    fn real(value: i32) -> Real {
        Real::from(value)
    }

    /// Construct an exact integer-coordinate planar fixture point.
    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    /// Construct an exact integer-coordinate spatial fixture point.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    /// Translate an integer by pi to keep fixture coordinates on the symbolic path.
    fn symbolic(value: i32) -> Real {
        Real::pi() + Real::from(value)
    }

    /// Translate both integer coordinates by pi for planar comparison fixtures.
    fn symbolic_p2(x: i32, y: i32) -> Point2 {
        Point2::new(symbolic(x), symbolic(y))
    }

    /// Translate all integer coordinates by pi for spatial comparison fixtures.
    fn symbolic_p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(symbolic(x), symbolic(y), symbolic(z))
    }

    /// Check integer 2D location labels, a reversed-corner query, and inclusive boundary mapping.
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

    /// Check one contained and one x-separated query against three integer point bounds.
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

    /// Check integer 2D separation, edge contact, area overlap, and inclusive corner contact.
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

    /// Check integer 3D separation, face contact, volume overlap, and inclusive corner contact.
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

    /// Check integer 3D labels, a reversed-corner query, and inclusive boundary mapping.
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

    /// Check ordered overlap/containment and relative interior of a planar box.
    ///
    /// Borrowed rational overlap must be Exact under both policies. A symbolic contact
    /// control must be strict Unknown and approximate true with Approximate certainty.
    /// The remaining assertions check values, not dispatch counts or stage labels.
    #[test]
    fn ordered_aabb3_predicates_cover_overlap_containment_and_relative_interior() {
        let outer_min = p3(0, 0, 0);
        let outer_max = p3(4, 4, 4);
        let inner_min = p3(1, 0, 1);
        let inner_max = p3(3, 0, 3);

        for policy in [PredicatePolicy::STRICT, APPROX] {
            let borrowed = crate::ordered_aabb3s_intersect_coordinates(
                [&outer_min.x, &outer_min.y, &outer_min.z],
                [&outer_max.x, &outer_max.y, &outer_max.z],
                [&inner_min.x, &inner_min.y, &inner_min.z],
                [&inner_max.x, &inner_max.y, &inner_max.z],
                policy,
            );
            assert!(matches!(
                borrowed,
                PredicateOutcome::Decided {
                    value: true,
                    certainty: Certainty::Exact,
                    ..
                }
            ));
        }

        let terminal_zero = terminally_unresolved_zero();
        let symbolic_max = Point3::new(terminal_zero, Real::one(), Real::one());
        let touching_min = p3(0, 0, 0);
        let touching_max = p3(2, 2, 2);
        let symbolic = |policy| {
            crate::ordered_aabb3s_intersect_coordinates(
                [&outer_min.x, &outer_min.y, &outer_min.z],
                [&symbolic_max.x, &symbolic_max.y, &symbolic_max.z],
                [&touching_min.x, &touching_min.y, &touching_min.z],
                [&touching_max.x, &touching_max.y, &touching_max.z],
                policy,
            )
        };
        assert!(matches!(
            symbolic(PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            symbolic(APPROX),
            PredicateOutcome::Decided {
                value: true,
                certainty: Certainty::Approximate,
                ..
            }
        ));

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

    /// Check borrowed integer boxes for contact/separation and three membership locations.
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

        /// Borrow the planar fixture's x/y coordinates without constructing another point.
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

    /// Check segment extent facts and point queries on a collapsed integer box.
    ///
    /// The computed facts are inspected here but not passed into either point predicate;
    /// this fixture does not demonstrate caller-cache consumption by those predicates.
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

    /// Check point/segment/area fact classifications and selected inclusive intersections.
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

    /// Check integer 3D point and box relation values through borrowed-corner APIs.
    ///
    /// These assertions do not measure allocations, copies, or memory usage.
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

    /// Check pi-translated point/box overlap cases under STRICT.
    ///
    /// The values exercise symbolic input shapes without asserting dispatch counts.
    #[test]
    fn symbolic_ordered_aabb2_predicates_exercise_the_real_fallback() {
        /// Borrow the planar fixture's x/y coordinates without constructing another point.
        fn coordinates(point: &Point2) -> [&Real; 2] {
            [&point.x, &point.y]
        }

        let min = symbolic_p2(0, 0);
        let max = symbolic_p2(4, 4);
        for (point, expected) in [
            (symbolic_p2(2, 2), true),
            (symbolic_p2(-1, 2), false),
            (symbolic_p2(5, 2), false),
            (symbolic_p2(2, -1), false),
            (symbolic_p2(2, 5), false),
        ] {
            assert_eq!(
                crate::point_in_ordered_aabb2_coordinates(
                    coordinates(&min),
                    coordinates(&max),
                    coordinates(&point),
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(expected)
            );
        }

        for (other_min, other_max, expected) in [
            (symbolic_p2(1, 1), symbolic_p2(3, 3), true),
            (symbolic_p2(4, 1), symbolic_p2(6, 3), true),
            (symbolic_p2(5, 1), symbolic_p2(6, 3), false),
            (symbolic_p2(-2, 1), symbolic_p2(-1, 3), false),
            (symbolic_p2(1, 5), symbolic_p2(3, 6), false),
        ] {
            assert_eq!(
                crate::ordered_aabb2s_intersect_coordinates(
                    coordinates(&min),
                    coordinates(&max),
                    coordinates(&other_min),
                    coordinates(&other_max),
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(expected)
            );
        }
    }

    /// Check selected pi-translated interior, endpoint, and outside locations in 2D/3D.
    ///
    /// The tables cover each spatial axis, not every endpoint/order combination.
    #[test]
    fn symbolic_point_box_predicates_cover_every_axis_and_boundary_route() {
        let min2 = symbolic_p2(0, 0);
        let max2 = symbolic_p2(4, 4);
        for (point, expected) in [
            (symbolic_p2(2, 2), Aabb2PointLocation::Inside),
            (symbolic_p2(0, 2), Aabb2PointLocation::Boundary),
            (symbolic_p2(2, 0), Aabb2PointLocation::Boundary),
            (symbolic_p2(-1, 2), Aabb2PointLocation::Outside),
            (symbolic_p2(2, 5), Aabb2PointLocation::Outside),
        ] {
            assert_eq!(
                crate::classify_point_aabb2(&min2, &max2, &point, PredicatePolicy::STRICT).value(),
                Some(expected)
            );
        }

        let min3 = symbolic_p3(0, 0, 0);
        let max3 = symbolic_p3(4, 4, 4);
        for (point, expected) in [
            (symbolic_p3(2, 2, 2), Aabb3PointLocation::Inside),
            (symbolic_p3(0, 2, 2), Aabb3PointLocation::Boundary),
            (symbolic_p3(2, 0, 2), Aabb3PointLocation::Boundary),
            (symbolic_p3(2, 2, 4), Aabb3PointLocation::Boundary),
            (symbolic_p3(-1, 2, 2), Aabb3PointLocation::Outside),
            (symbolic_p3(2, 5, 2), Aabb3PointLocation::Outside),
            (symbolic_p3(2, 2, 5), Aabb3PointLocation::Outside),
        ] {
            assert_eq!(
                crate::classify_point_aabb3(&min3, &max3, &point, PredicatePolicy::STRICT).value(),
                Some(expected)
            );
        }
        assert_eq!(
            crate::point_in_aabb3(&min3, &max3, &symbolic_p3(2, 2, 2), PredicatePolicy::STRICT,)
                .value(),
            Some(true)
        );
    }

    /// Check a pi-translated planar box's interior, open-axis ends, and collapsed-axis miss.
    #[test]
    fn symbolic_relative_interior_covers_zero_and_positive_extent_axes() {
        let min = symbolic_p3(0, 0, 0);
        let max = symbolic_p3(4, 0, 4);
        for (point, expected) in [
            (symbolic_p3(2, 0, 2), true),
            (symbolic_p3(0, 0, 2), false),
            (symbolic_p3(4, 0, 2), false),
            (symbolic_p3(2, 1, 2), false),
            (symbolic_p3(2, 0, 4), false),
        ] {
            assert_eq!(
                crate::point_in_ordered_aabb3_relative_interior(
                    &min,
                    &max,
                    &point,
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(expected)
            );
        }
    }

    /// Check symbolic 3D overlap, contact, separation on each axis, and zero-width contact.
    ///
    /// Also compare nested boxes in both containment directions; no cost or dispatch
    /// assertions accompany these relation values.
    #[test]
    fn symbolic_aabb3_intersections_exercise_interval_and_extent_fallbacks() {
        let first_min = symbolic_p3(0, 0, 0);
        let first_max = symbolic_p3(4, 4, 4);
        for (second_min, second_max, expected) in [
            (
                symbolic_p3(1, 1, 1),
                symbolic_p3(3, 3, 3),
                Aabb3Intersection::Overlapping,
            ),
            (
                symbolic_p3(4, 1, 1),
                symbolic_p3(6, 3, 3),
                Aabb3Intersection::Touching,
            ),
            (
                symbolic_p3(5, 1, 1),
                symbolic_p3(6, 3, 3),
                Aabb3Intersection::Disjoint,
            ),
            (
                symbolic_p3(1, 5, 1),
                symbolic_p3(3, 6, 3),
                Aabb3Intersection::Disjoint,
            ),
            (
                symbolic_p3(1, 1, 5),
                symbolic_p3(3, 3, 6),
                Aabb3Intersection::Disjoint,
            ),
            (
                symbolic_p3(1, 2, 1),
                symbolic_p3(3, 2, 3),
                Aabb3Intersection::Touching,
            ),
        ] {
            assert_eq!(
                crate::classify_aabb3_intersection(
                    &first_min,
                    &first_max,
                    &second_min,
                    &second_max,
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(expected)
            );
        }

        let outer_min = symbolic_p3(0, 0, 0);
        let outer_max = symbolic_p3(6, 6, 6);
        let inner_min = symbolic_p3(1, 1, 1);
        let inner_max = symbolic_p3(5, 5, 5);
        assert_eq!(
            crate::ordered_aabb3_contains(
                &outer_min,
                &outer_max,
                &inner_min,
                &inner_max,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(true)
        );
        assert_eq!(
            crate::ordered_aabb3_contains(
                &inner_min,
                &inner_max,
                &outer_min,
                &outer_max,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(false)
        );
    }

    /// Check selected strict Unknown paths through location, relative-interior, and overlap wrappers.
    #[test]
    fn aabb_wrappers_propagate_strict_unknowns_without_boolean_coercion() {
        /// Test only the outcome variant, without asserting its refinement need or stage.
        fn is_unknown<T>(outcome: PredicateOutcome<T>) -> bool {
            matches!(outcome, PredicateOutcome::Unknown { .. })
        }

        let unresolved = terminally_unresolved_zero();
        let min2 = p2(0, 0);
        let max2 = p2(1, 1);
        let unknown_x2 = Point2::new(unresolved.clone(), Real::from(0));
        let unknown_y2 = Point2::new(Real::from(0), unresolved.clone());
        assert!(is_unknown(crate::classify_point_aabb2(
            &min2,
            &max2,
            &unknown_x2,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(crate::classify_point_aabb2(
            &min2,
            &max2,
            &unknown_y2,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(crate::point_in_aabb2(
            &min2,
            &max2,
            &unknown_x2,
            PredicatePolicy::STRICT,
        )));

        let min3 = p3(0, 0, 0);
        let max3 = p3(1, 1, 1);
        for point in [
            Point3::new(unresolved.clone(), Real::from(0), Real::from(0)),
            Point3::new(Real::from(0), unresolved.clone(), Real::from(0)),
            Point3::new(Real::from(0), Real::from(0), unresolved.clone()),
        ] {
            assert!(is_unknown(crate::classify_point_aabb3(
                &min3,
                &max3,
                &point,
                PredicatePolicy::STRICT,
            )));
        }
        assert!(is_unknown(crate::point_in_aabb3(
            &min3,
            &max3,
            &Point3::new(unresolved.clone(), Real::from(0), Real::from(0)),
            PredicatePolicy::STRICT,
        )));

        let unknown_max3 = Point3::new(unresolved.clone(), Real::from(1), Real::from(1));
        assert!(is_unknown(crate::point_in_ordered_aabb3_relative_interior(
            &min3,
            &unknown_max3,
            &p3(0, 0, 0),
            PredicatePolicy::STRICT,
        )));
        let zero_width = p3(0, 1, 1);
        assert!(is_unknown(crate::point_in_ordered_aabb3_relative_interior(
            &min3,
            &zero_width,
            &Point3::new(unresolved.clone(), Real::from(0), Real::from(0)),
            PredicatePolicy::STRICT,
        )));

        let first_min2 = p2(-1, -1);
        let first_max2 = Point2::new(unresolved.clone(), Real::from(1));
        assert!(is_unknown(crate::classify_aabb2_intersection(
            &first_min2,
            &first_max2,
            &min2,
            &max2,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(crate::aabb2s_intersect(
            &first_min2,
            &first_max2,
            &min2,
            &max2,
            PredicatePolicy::STRICT,
        )));

        let first_min3 = p3(-1, -1, -1);
        let first_max3 = Point3::new(unresolved, Real::from(1), Real::from(1));
        assert!(is_unknown(crate::classify_aabb3_intersection(
            &first_min3,
            &first_max3,
            &min3,
            &max3,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(crate::aabb3s_intersect(
            &first_min3,
            &first_max3,
            &min3,
            &max3,
            PredicatePolicy::STRICT,
        )));
    }

    /// Check symbolic uncertainty in borrowed comparisons and triangle x/y bounds or membership.
    #[test]
    fn borrowed_and_triangle_aabb_predicates_preserve_each_strict_unknown() {
        /// Test only the outcome variant, without asserting its refinement need or stage.
        fn is_unknown<T>(outcome: PredicateOutcome<T>) -> bool {
            matches!(outcome, PredicateOutcome::Unknown { .. })
        }

        let unresolved = terminally_unresolved_zero();
        let minus_one = real(-1);
        let zero = real(0);
        let one = real(1);

        assert!(is_unknown(point_in_ordered_aabb2_coordinates_with_policy(
            [&zero, &zero],
            [&one, &one],
            [&unresolved, &zero],
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(point_in_ordered_aabb2_coordinates_with_policy(
            [&minus_one, &zero],
            [&zero, &one],
            [&unresolved, &zero],
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(
            ordered_aabb2s_intersect_coordinates_with_policy(
                [&minus_one, &minus_one],
                [&unresolved, &one],
                [&zero, &zero],
                [&one, &one],
                PredicatePolicy::STRICT,
            )
        ));

        let point = p2(0, 0);
        assert!(is_unknown(point_in_triangle2_aabb_with_policy(
            &Point2::new(unresolved.clone(), real(0)),
            &p2(0, 1),
            &p2(1, 2),
            &point,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(point_in_triangle2_aabb_with_policy(
            &p2(0, 0),
            &p2(1, 1),
            &p2(2, 2),
            &Point2::new(unresolved.clone(), real(0)),
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(point_in_triangle2_aabb_with_policy(
            &Point2::new(real(0), unresolved.clone()),
            &p2(1, 0),
            &p2(2, 1),
            &point,
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(point_in_triangle2_aabb_with_policy(
            &p2(0, 0),
            &p2(1, 1),
            &p2(2, 2),
            &Point2::new(real(0), unresolved),
            PredicatePolicy::STRICT,
        )));
    }

    /// Check selected later-axis separations, Unknowns, and rational corner reversal.
    ///
    /// Includes the interval zero-extent helper, without proving exhaustive fallback
    /// coverage or any specific dispatch count.
    #[test]
    fn aabb_fallbacks_cover_late_axis_separation_and_unknowns() {
        /// Test only the outcome variant, without asserting its refinement need or stage.
        fn is_unknown<T>(outcome: PredicateOutcome<T>) -> bool {
            matches!(outcome, PredicateOutcome::Unknown { .. })
        }

        let unresolved = terminally_unresolved_zero();

        assert!(is_unknown(
            point_in_ordered_aabb3_relative_interior_with_policy(
                &p3(0, 0, 0),
                &p3(1, 1, 1),
                &Point3::new(unresolved.clone(), real(0), real(0)),
                PredicatePolicy::STRICT,
            )
        ));
        assert!(is_unknown(
            point_in_ordered_aabb3_relative_interior_with_policy(
                &p3(-1, 0, 0),
                &p3(0, 1, 1),
                &Point3::new(unresolved.clone(), real(0), real(0)),
                PredicatePolicy::STRICT,
            )
        ));

        assert_eq!(
            classify_aabb2_intersection_with_policy(
                &p2(0, 0),
                &p2(2, 1),
                &p2(1, 2),
                &p2(3, 3),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Aabb2Intersection::Disjoint),
        );
        assert!(is_unknown(classify_aabb2_intersection_with_policy(
            &p2(-1, -1),
            &Point2::new(real(1), unresolved.clone()),
            &p2(0, 0),
            &p2(2, 1),
            PredicatePolicy::STRICT,
        )));

        // Reverse both boxes on one axis to exercise exact-rational endpoint
        // normalization without changing the geometric intersection.
        assert_eq!(
            classify_aabb3_intersection_with_policy(
                &p3(4, 0, 0),
                &p3(0, 4, 4),
                &p3(3, 1, 1),
                &p3(1, 3, 3),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Aabb3Intersection::Overlapping),
        );

        assert!(is_unknown(classify_aabb3_intersection_with_policy(
            &p3(-1, -1, -1),
            &Point3::new(real(1), unresolved.clone(), real(1)),
            &p3(0, 0, 0),
            &p3(2, 1, 2),
            PredicatePolicy::STRICT,
        )));
        assert!(is_unknown(classify_aabb3_intersection_with_policy(
            &p3(-1, -1, -1),
            &Point3::new(real(1), real(1), unresolved.clone()),
            &p3(0, 0, 0),
            &p3(2, 2, 1),
            PredicatePolicy::STRICT,
        )));

        assert!(is_unknown(
            classify_closed_interval_intersection_with_extent_policy(
                &unresolved,
                &real(0),
                &real(0),
                &real(1),
                PredicatePolicy::STRICT,
            )
        ));
    }

    /// Check all finite certainty/stage rank pairs and selected DecisionTrace updates.
    ///
    /// A decided child updates the trace; an Unknown child returns an error. The test
    /// does not assert the trace state after that error or arbitrary geometric decisions.
    #[test]
    fn aabb_decision_trace_rankings_are_total() {
        let certainties = [
            Certainty::Exact,
            Certainty::Filtered,
            Certainty::Approximate,
        ];
        for left in certainties {
            for right in certainties {
                assert_eq!(
                    certainty_rank(max_certainty(left, right)),
                    certainty_rank(left).max(certainty_rank(right)),
                );
            }
        }

        let stages = [
            Escalation::Structural,
            Escalation::Filter,
            Escalation::Exact,
            Escalation::Refined,
            Escalation::Undecided,
        ];
        for left in stages {
            for right in stages {
                assert_eq!(
                    stage_rank(max_stage(left, right)),
                    stage_rank(left).max(stage_rank(right)),
                );
            }
        }

        let mut trace = DecisionTrace::default();
        assert!(matches!(
            decided(
                PredicateOutcome::decided(7_u8, Certainty::Filtered, Escalation::Filter),
                &mut trace,
            ),
            Ok(7)
        ));
        assert_eq!(trace.certainty, Certainty::Filtered);
        assert_eq!(trace.stage, Escalation::Filter);
        assert!(
            decided::<u8>(
                PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided),
                &mut trace,
            )
            .is_err()
        );
    }
}
