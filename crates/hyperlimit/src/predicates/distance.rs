//! Exact point-distance comparison predicates.
//!
//! Distance comparisons use squared Euclidean distance so predicate callers do
//! not force square-root construction or lossy approximations.

use core::cmp::Ordering;

use crate::classify::{
    AabbSphereIntersection, CircleLineRelation, CircleSegmentRelation, SphereIntersection,
    SpherePointLocation,
};
use crate::geometry::{Point2, Point3};
use crate::predicate::{PredicateOutcome, PredicatePolicy};
use crate::predicates::order::compare_reals_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use hyperreal::Real;

/// Reusable explicit 3D sphere classifier.
#[derive(Clone, Copy, Debug)]
pub struct PreparedExplicitSphere3<'a> {
    center: &'a Point3,
    radius_squared: &'a Real,
}

impl<'a> PreparedExplicitSphere3<'a> {
    /// Prepare an explicit sphere from a center and squared radius.
    pub fn new(center: &'a Point3, radius_squared: &'a Real) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_explicit_sphere3", "new");
        Self {
            center,
            radius_squared,
        }
    }

    /// Return the borrowed sphere center.
    pub const fn center(&self) -> &'a Point3 {
        self.center
    }

    /// Return the borrowed squared radius.
    pub const fn radius_squared(&self) -> &'a Real {
        self.radius_squared
    }

    /// Classify a point using the default predicate policy.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<SpherePointLocation> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point using an explicit predicate policy.
    pub(crate) fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SpherePointLocation> {
        classify_point_sphere3_with_policy(self.center, self.radius_squared, point, policy)
    }
}

/// Compare squared distances from `anchor` to `left` and `right`.
pub fn compare_point2_distance_squared(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
) -> PredicateOutcome<Ordering> {
    compare_point2_distance_squared_with_policy(anchor, left, right, PredicatePolicy::default())
}

/// Compare squared distances from `anchor` to `left` and `right` with an
/// explicit predicate escalation policy.
///
/// Squared-distance comparison is the exact form needed by nearest-candidate
/// selection in bridge construction, snapping, and broad-phase refinement. It
/// avoids constructing a square root and asks the Real sign resolver to
/// certify `|anchor-left|^2 - |anchor-right|^2`. This is the standard
/// distance-ordering reduction used throughout computational geometry texts
/// such as de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
/// Algorithms and Applications*, 3rd ed., Springer, 2008, and it keeps the
/// final sign decision in the exact-geometric-computation model of Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub(crate) fn compare_point2_distance_squared_with_policy(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let left_distance = squared_distance2(anchor, left);
    let right_distance = squared_distance2(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Compare squared 3D distances from `anchor` to `left` and `right`.
pub fn compare_point3_distance_squared(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
) -> PredicateOutcome<Ordering> {
    compare_point3_distance_squared_with_policy(anchor, left, right, PredicatePolicy::default())
}

/// Compare squared 3D distances from `anchor` to `left` and `right` with an
/// explicit predicate escalation policy.
///
/// This is the 3D lift of [`compare_point2_distance_squared`]. It compares
/// `|anchor-left|^2` and `|anchor-right|^2` through exact `Real` predicates,
/// avoiding square-root construction and primitive-float tie decisions.
pub(crate) fn compare_point3_distance_squared_with_policy(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let left_distance = squared_distance3(anchor, left);
    let right_distance = squared_distance3(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Classify the relation between a 2D circle boundary and an infinite line.
pub fn classify_circle_line2(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
) -> PredicateOutcome<CircleLineRelation> {
    classify_circle_line2_with_policy(center, radius_squared, a, b, PredicatePolicy::default())
}

/// Classify the relation between a 2D circle boundary and an infinite line
/// using an explicit predicate policy.
///
/// The decision compares `|(center-a) x (b-a)|^2` with
/// `radius_squared * |b-a|^2`, so it never constructs a square root or divides
/// by line length. This is the standard line/circle discriminant written in
/// squared-distance form; keeping it as an exact sign comparison follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub(crate) fn classify_circle_line2_with_policy(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<CircleLineRelation> {
    let direction = vector2_between(a, b);
    let direction_norm = norm_squared2(&direction);
    match compare_reals_with_policy(&direction_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty,
            stage,
        } => {
            return PredicateOutcome::decided(CircleLineRelation::DegenerateLine, certainty, stage);
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let offset = vector2_between(a, center);
    let cross = sub_ref(
        &mul_ref(&offset.x, &direction.y),
        &mul_ref(&offset.y, &direction.x),
    );
    let numerator = mul_ref(&cross, &cross);
    let scaled_radius = mul_ref(radius_squared, &direction_norm);
    match compare_reals_with_policy(&numerator, &scaled_radius, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let relation = match value {
                Ordering::Less => CircleLineRelation::Secant,
                Ordering::Equal => CircleLineRelation::Tangent,
                Ordering::Greater => CircleLineRelation::Disjoint,
            };
            PredicateOutcome::decided(relation, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the relation between a 2D circle boundary and a closed segment.
pub fn classify_circle_segment2(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
) -> PredicateOutcome<CircleSegmentRelation> {
    classify_circle_segment2_with_policy(center, radius_squared, a, b, PredicatePolicy::default())
}

/// Classify the relation between a 2D circle boundary and a closed segment
/// using an explicit predicate policy.
///
/// Endpoint distance signs decide whether a crossing can occur on the closed
/// interval. When both endpoints are outside, the exact point-segment distance
/// comparison distinguishes disjoint, tangent, and secant cases without a
/// primitive tolerance. Degenerate segments reduce to exact point/circle
/// classification.
pub(crate) fn classify_circle_segment2_with_policy(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<CircleSegmentRelation> {
    let direction_norm = norm_squared2(&vector2_between(a, b));
    match compare_reals_with_policy(&direction_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {
            return match compare_point2_distance_squared_to_threshold_with_policy(
                a,
                center,
                radius_squared,
                policy,
            ) {
                PredicateOutcome::Decided {
                    value,
                    certainty,
                    stage,
                } => {
                    let relation = match value {
                        Ordering::Less => CircleSegmentRelation::ContainedInside,
                        Ordering::Equal => CircleSegmentRelation::Tangent,
                        Ordering::Greater => CircleSegmentRelation::Disjoint,
                    };
                    PredicateOutcome::decided(relation, certainty, stage)
                }
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            };
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let a_cmp =
        compare_point2_distance_squared_to_threshold_with_policy(a, center, radius_squared, policy);
    let b_cmp =
        compare_point2_distance_squared_to_threshold_with_policy(b, center, radius_squared, policy);
    let (a_order, b_order, certainty, stage) = match (a_cmp, b_cmp) {
        (
            PredicateOutcome::Decided {
                value: a_value,
                certainty: a_certainty,
                stage: a_stage,
            },
            PredicateOutcome::Decided {
                value: b_value,
                certainty: b_certainty,
                stage: b_stage,
            },
        ) => (
            a_value,
            b_value,
            max_distance_certainty(a_certainty, b_certainty),
            max_distance_stage(a_stage, b_stage),
        ),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    if a_order == Ordering::Equal && b_order == Ordering::Equal {
        return PredicateOutcome::decided(CircleSegmentRelation::Secant, certainty, stage);
    }
    if a_order == Ordering::Equal || b_order == Ordering::Equal {
        let other = if a_order == Ordering::Equal {
            b_order
        } else {
            a_order
        };
        let relation = if other == Ordering::Less {
            CircleSegmentRelation::Tangent
        } else {
            CircleSegmentRelation::Secant
        };
        return PredicateOutcome::decided(relation, certainty, stage);
    }
    if (a_order == Ordering::Less) != (b_order == Ordering::Less) {
        return PredicateOutcome::decided(CircleSegmentRelation::Secant, certainty, stage);
    }
    if a_order == Ordering::Less && b_order == Ordering::Less {
        return PredicateOutcome::decided(CircleSegmentRelation::ContainedInside, certainty, stage);
    }

    match compare_point_segment2_distance_squared_with_policy(center, a, b, radius_squared, policy)
    {
        PredicateOutcome::Decided {
            value,
            certainty: distance_certainty,
            stage: distance_stage,
        } => {
            let relation = match value {
                Ordering::Less => CircleSegmentRelation::Secant,
                Ordering::Equal => CircleSegmentRelation::Tangent,
                Ordering::Greater => CircleSegmentRelation::Disjoint,
            };
            PredicateOutcome::decided(
                relation,
                max_distance_certainty(certainty, distance_certainty),
                max_distance_stage(stage, distance_stage),
            )
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Compare the squared distance from `point` to the infinite 3D line `ab`
/// against `threshold_squared`.
pub fn compare_point_line3_distance_squared(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
) -> PredicateOutcome<Ordering> {
    compare_point_line3_distance_squared_with_policy(
        point,
        a,
        b,
        threshold_squared,
        PredicatePolicy::default(),
    )
}

/// Compare the squared distance from `point` to the infinite 3D line `ab`
/// against `threshold_squared` with an explicit predicate policy.
///
/// The predicate avoids constructing the projected point or dividing by
/// `|b-a|^2`: it compares `|(point-a) x (b-a)|^2` with
/// `threshold_squared * |b-a|^2`. This is the standard squared-distance
/// reduction for point-line queries in computational geometry, while the
/// division-free exact decision boundary follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
pub(crate) fn compare_point_line3_distance_squared_with_policy(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let direction = vector3_between(a, b);
    let direction_norm = norm_squared3(&direction);
    match compare_reals_with_policy(&direction_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {
            return compare_point3_distance_squared_to_threshold_with_policy(
                point,
                a,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let offset = vector3_between(a, point);
    let cross = cross3(&offset, &direction);
    let numerator = norm_squared3(&cross);
    let scaled_threshold = mul_ref(threshold_squared, &direction_norm);
    compare_reals_with_policy(&numerator, &scaled_threshold, policy)
}

/// Compare the squared distance from `point` to the closed 3D segment `ab`
/// against `threshold_squared`.
pub fn compare_point_segment3_distance_squared(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
) -> PredicateOutcome<Ordering> {
    compare_point_segment3_distance_squared_with_policy(
        point,
        a,
        b,
        threshold_squared,
        PredicatePolicy::default(),
    )
}

/// Compare the squared distance from `point` to the closed 3D segment `ab`
/// against `threshold_squared` with an explicit predicate policy.
///
/// Projection signs select the closest endpoint or the interior line-distance
/// branch exactly. No square roots, normalized direction vectors, or
/// primitive-float tolerances are used; this keeps closest-feature decisions in
/// Yap's exact-geometric-computation model.
pub(crate) fn compare_point_segment3_distance_squared_with_policy(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let direction = vector3_between(a, b);
    let direction_norm = norm_squared3(&direction);
    match compare_reals_with_policy(&direction_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {
            return compare_point3_distance_squared_to_threshold_with_policy(
                point,
                a,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let ap = vector3_between(a, point);
    let projection = dot3(&ap, &direction);
    match compare_reals_with_policy(&projection, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less | Ordering::Equal,
            ..
        } => {
            return compare_point3_distance_squared_to_threshold_with_policy(
                point,
                a,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    match compare_reals_with_policy(&projection, &direction_norm, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater | Ordering::Equal,
            ..
        } => {
            return compare_point3_distance_squared_to_threshold_with_policy(
                point,
                b,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    compare_point_line3_distance_squared_with_policy(point, a, b, threshold_squared, policy)
}

/// Compare the squared distance from `point` to `plane` against
/// `threshold_squared`.
pub fn compare_point_plane_distance_squared(
    point: &Point3,
    plane: &crate::plane::Plane3,
    threshold_squared: &Real,
) -> PredicateOutcome<Ordering> {
    compare_point_plane_distance_squared_with_policy(
        point,
        plane,
        threshold_squared,
        PredicatePolicy::default(),
    )
}

/// Compare the squared distance from `point` to `plane` against
/// `threshold_squared` with an explicit predicate policy.
///
/// The signed plane expression is squared and compared against
/// `threshold_squared * |normal|^2`, so the predicate never normalizes the
/// plane or constructs a square root. Degenerate zero-normal planes fall back
/// to comparing the squared offset expression directly, making invalid input
/// behavior explicit and exact rather than tolerance-defined.
pub(crate) fn compare_point_plane_distance_squared_with_policy(
    point: &Point3,
    plane: &crate::plane::Plane3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let expression = point_plane_expression(point, plane);
    let numerator = mul_ref(&expression, &expression);
    let normal_norm = squared_point3_norm(&plane.normal);
    match compare_reals_with_policy(&normal_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => return compare_reals_with_policy(&numerator, threshold_squared, policy),
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    let scaled_threshold = mul_ref(threshold_squared, &normal_norm);
    compare_reals_with_policy(&numerator, &scaled_threshold, policy)
}

/// Classify the intersection of two closed explicit 3D spheres.
pub fn classify_sphere3_intersection(
    first_center: &Point3,
    first_radius: &Real,
    second_center: &Point3,
    second_radius: &Real,
) -> PredicateOutcome<SphereIntersection> {
    classify_sphere3_intersection_with_policy(
        first_center,
        first_radius,
        second_center,
        second_radius,
        PredicatePolicy::default(),
    )
}

/// Classify the intersection of two closed explicit 3D spheres with an
/// explicit predicate policy.
///
/// The API accepts radii, not squared radii, because sphere-sphere contact is
/// decided by `|c0-c1|^2` versus `(r0+r1)^2`. Negative radii are rejected as
/// unsupported domain input instead of being silently reinterpreted. This keeps
/// the exact algebraic relation square-root-free while preserving Yap's rule
/// that invalid or undecidable geometric states stay explicit.
pub(crate) fn classify_sphere3_intersection_with_policy(
    first_center: &Point3,
    first_radius: &Real,
    second_center: &Point3,
    second_radius: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<SphereIntersection> {
    if let Some(outcome) = reject_negative_radius(first_radius, policy) {
        return outcome;
    }
    if let Some(outcome) = reject_negative_radius(second_radius, policy) {
        return outcome;
    }

    let radius_sum = add_ref(first_radius, second_radius);
    let radius_sum_squared = mul_ref(&radius_sum, &radius_sum);
    match compare_point3_distance_squared_to_threshold_with_policy(
        first_center,
        second_center,
        &radius_sum_squared,
        policy,
    ) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let relation = match value {
                Ordering::Less => SphereIntersection::Overlapping,
                Ordering::Equal => SphereIntersection::Touching,
                Ordering::Greater => SphereIntersection::Disjoint,
            };
            PredicateOutcome::decided(relation, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the intersection of a closed 3D AABB and an explicit sphere whose
/// radius is supplied squared.
pub fn classify_aabb3_sphere_intersection(
    min: &Point3,
    max: &Point3,
    center: &Point3,
    radius_squared: &Real,
) -> PredicateOutcome<AabbSphereIntersection> {
    classify_aabb3_sphere_intersection_with_policy(
        min,
        max,
        center,
        radius_squared,
        PredicatePolicy::default(),
    )
}

/// Classify the intersection of a closed 3D AABB and an explicit sphere with
/// an explicit predicate policy.
///
/// The nearest-point distance is formed as the sum of squared outside-axis
/// violations, which is the standard AABB/sphere broad-phase predicate. Each
/// axis comparison is exact and inclusive, and the final comparison stays in
/// squared-distance form. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), for the exact decision boundary.
pub(crate) fn classify_aabb3_sphere_intersection_with_policy(
    min: &Point3,
    max: &Point3,
    center: &Point3,
    radius_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<AabbSphereIntersection> {
    let distance_squared = match aabb3_center_distance_squared(min, max, center, policy) {
        Ok(distance) => distance,
        Err(outcome) => return outcome,
    };
    match compare_reals_with_policy(&distance_squared, radius_squared, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let relation = match value {
                Ordering::Less => AabbSphereIntersection::Overlapping,
                Ordering::Equal => AabbSphereIntersection::Touching,
                Ordering::Greater => AabbSphereIntersection::Disjoint,
            };
            PredicateOutcome::decided(relation, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify a point relative to an explicit 3D sphere with squared radius.
pub fn classify_point_sphere3(
    center: &Point3,
    radius_squared: &Real,
    point: &Point3,
) -> PredicateOutcome<SpherePointLocation> {
    classify_point_sphere3_with_policy(center, radius_squared, point, PredicatePolicy::default())
}

/// Classify a point relative to an explicit 3D sphere with squared radius and
/// an explicit predicate escalation policy.
///
/// The API accepts squared radius so callers do not need to construct square
/// roots. Domain validation for nonnegative radius remains with the caller that
/// owns the sphere object; this predicate only certifies the distance relation.
pub(crate) fn classify_point_sphere3_with_policy(
    center: &Point3,
    radius_squared: &Real,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<SpherePointLocation> {
    let distance_squared = squared_distance3(center, point);
    match compare_reals_with_policy(&distance_squared, radius_squared, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let location = match value {
                Ordering::Less => SpherePointLocation::Inside,
                Ordering::Equal => SpherePointLocation::On,
                Ordering::Greater => SpherePointLocation::Outside,
            };
            PredicateOutcome::decided(location, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn reject_negative_radius<T>(
    radius: &Real,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<T>> {
    match compare_reals_with_policy(radius, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => Some(PredicateOutcome::unknown(
            crate::predicate::RefinementNeed::Unsupported,
            crate::predicate::Escalation::Undecided,
        )),
        PredicateOutcome::Decided { .. } => None,
        PredicateOutcome::Unknown { needed, stage } => {
            Some(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn aabb3_center_distance_squared(
    min: &Point3,
    max: &Point3,
    center: &Point3,
    policy: PredicatePolicy,
) -> Result<Real, PredicateOutcome<AabbSphereIntersection>> {
    let dx = outside_interval_delta(&center.x, &min.x, &max.x, policy)?;
    let dy = outside_interval_delta(&center.y, &min.y, &max.y, policy)?;
    let dz = outside_interval_delta(&center.z, &min.z, &max.z, policy)?;
    Ok(Real::signed_product_sum(
        [true; 3],
        [[&dx, &dx], [&dy, &dy], [&dz, &dz]],
    ))
}

fn outside_interval_delta(
    value: &Real,
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
) -> Result<Real, PredicateOutcome<AabbSphereIntersection>> {
    let (min, max) = match compare_reals_with_policy(first, second, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => (second, first),
        PredicateOutcome::Decided { .. } => (first, second),
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    };
    match compare_reals_with_policy(value, min, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            ..
        } => return Ok(sub_ref(min, value)),
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    }
    match compare_reals_with_policy(value, max, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            ..
        } => Ok(sub_ref(value, max)),
        PredicateOutcome::Decided { .. } => Ok(Real::from(0)),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn compare_point3_distance_squared_to_threshold_with_policy(
    point: &Point3,
    target: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let distance_squared = squared_distance3(point, target);
    compare_reals_with_policy(&distance_squared, threshold_squared, policy)
}

fn compare_point2_distance_squared_to_threshold_with_policy(
    point: &Point2,
    target: &Point2,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let distance_squared = squared_distance2(point, target);
    compare_reals_with_policy(&distance_squared, threshold_squared, policy)
}

fn compare_point_segment2_distance_squared_with_policy(
    point: &Point2,
    a: &Point2,
    b: &Point2,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let direction = vector2_between(a, b);
    let direction_norm = norm_squared2(&direction);
    match compare_reals_with_policy(&direction_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        } => {
            return compare_point2_distance_squared_to_threshold_with_policy(
                point,
                a,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let ap = vector2_between(a, point);
    let projection = dot2(&ap, &direction);
    match compare_reals_with_policy(&projection, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less | Ordering::Equal,
            ..
        } => {
            return compare_point2_distance_squared_to_threshold_with_policy(
                point,
                a,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }
    match compare_reals_with_policy(&projection, &direction_norm, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater | Ordering::Equal,
            ..
        } => {
            return compare_point2_distance_squared_to_threshold_with_policy(
                point,
                b,
                threshold_squared,
                policy,
            );
        }
        PredicateOutcome::Decided { .. } => {}
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let cross = sub_ref(&mul_ref(&ap.x, &direction.y), &mul_ref(&ap.y, &direction.x));
    let numerator = mul_ref(&cross, &cross);
    let scaled_threshold = mul_ref(threshold_squared, &direction_norm);
    compare_reals_with_policy(&numerator, &scaled_threshold, policy)
}

fn squared_distance2(left: &Point2, right: &Point2) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy))
}

fn squared_distance3(left: &Point3, right: &Point3) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    let dz = sub_ref(&right.z, &left.z);
    let xy = add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy));
    add_ref(&xy, &mul_ref(&dz, &dz))
}

#[derive(Clone, Debug)]
struct Vector2Real {
    x: Real,
    y: Real,
}

fn vector2_between(start: &Point2, end: &Point2) -> Vector2Real {
    Vector2Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
    }
}

fn dot2(left: &Vector2Real, right: &Vector2Real) -> Real {
    Real::signed_product_sum([true; 2], [[&left.x, &right.x], [&left.y, &right.y]])
}

fn norm_squared2(vector: &Vector2Real) -> Real {
    Real::signed_product_sum([true; 2], [[&vector.x, &vector.x], [&vector.y, &vector.y]])
}

#[derive(Clone, Debug)]
struct Vector3Real {
    x: Real,
    y: Real,
    z: Real,
}

fn vector3_between(start: &Point3, end: &Point3) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
        z: sub_ref(&end.z, &start.z),
    }
}

fn cross3(left: &Vector3Real, right: &Vector3Real) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&mul_ref(&left.y, &right.z), &mul_ref(&left.z, &right.y)),
        y: sub_ref(&mul_ref(&left.z, &right.x), &mul_ref(&left.x, &right.z)),
        z: sub_ref(&mul_ref(&left.x, &right.y), &mul_ref(&left.y, &right.x)),
    }
}

fn dot3(left: &Vector3Real, right: &Vector3Real) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left.x, &right.x],
            [&left.y, &right.y],
            [&left.z, &right.z],
        ],
    )
}

fn norm_squared3(vector: &Vector3Real) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&vector.x, &vector.x],
            [&vector.y, &vector.y],
            [&vector.z, &vector.z],
        ],
    )
}

fn squared_point3_norm(point: &Point3) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&point.x, &point.x],
            [&point.y, &point.y],
            [&point.z, &point.z],
        ],
    )
}

fn point_plane_expression(point: &Point3, plane: &crate::plane::Plane3) -> Real {
    let one = Real::one();
    Real::signed_product_sum(
        [true; 4],
        [
            [&plane.normal.x, &point.x],
            [&plane.normal.y, &point.y],
            [&plane.normal.z, &point.z],
            [&plane.offset, &one],
        ],
    )
}

fn max_distance_certainty(
    left: crate::predicate::Certainty,
    right: crate::predicate::Certainty,
) -> crate::predicate::Certainty {
    match (left, right) {
        (crate::predicate::Certainty::Filtered, _) | (_, crate::predicate::Certainty::Filtered) => {
            crate::predicate::Certainty::Filtered
        }
        _ => crate::predicate::Certainty::Exact,
    }
}

fn max_distance_stage(
    left: crate::predicate::Escalation,
    right: crate::predicate::Escalation,
) -> crate::predicate::Escalation {
    if distance_stage_rank(left) >= distance_stage_rank(right) {
        left
    } else {
        right
    }
}

fn distance_stage_rank(stage: crate::predicate::Escalation) -> u8 {
    match stage {
        crate::predicate::Escalation::Structural => 0,
        crate::predicate::Escalation::Filter => 1,
        crate::predicate::Escalation::Exact => 2,
        crate::predicate::Escalation::Refined => 3,
        crate::predicate::Escalation::Undecided => 4,
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
    fn squared_distance_comparison_avoids_square_roots() {
        let anchor = p2(0, 0);
        let near = p2(3, 4);
        let far = p2(6, 8);
        let also_near = p2(-3, -4);

        assert_eq!(
            compare_point2_distance_squared(&anchor, &near, &far).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point2_distance_squared(&anchor, &near, &also_near).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point2_distance_squared(&anchor, &far, &near).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn squared_distance3_comparison_avoids_square_roots() {
        let anchor = p3(0, 0, 0);
        let near = p3(1, 2, 2);
        let far = p3(2, 4, 4);
        let also_near = p3(-1, -2, -2);

        assert_eq!(
            compare_point3_distance_squared(&anchor, &near, &far).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point3_distance_squared(&anchor, &near, &also_near).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point3_distance_squared(&anchor, &far, &near).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn point_sphere3_classifier_uses_squared_radius() {
        let center = p3(0, 0, 0);
        let radius_squared = hyperreal::Real::from(25);
        let sphere = PreparedExplicitSphere3::new(&center, &radius_squared);

        assert_eq!(sphere.center(), &center);
        assert_eq!(sphere.radius_squared(), &radius_squared);
        assert_eq!(
            classify_point_sphere3(&center, &radius_squared, &p3(1, 2, 2)).value(),
            Some(SpherePointLocation::Inside)
        );
        assert_eq!(
            sphere.classify_point(&p3(3, 4, 0)).value(),
            Some(SpherePointLocation::On)
        );
        assert_eq!(
            sphere.classify_point(&p3(6, 0, 0)).value(),
            Some(SpherePointLocation::Outside)
        );
    }

    #[test]
    fn point_line3_distance_comparison_is_scaled_without_division() {
        let point = p3(0, 3, 4);
        let a = p3(0, 0, 0);
        let b = p3(2, 0, 0);

        assert_eq!(
            compare_point_line3_distance_squared(&point, &a, &b, &Real::from(24)).value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            compare_point_line3_distance_squared(&point, &a, &b, &Real::from(25)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point_line3_distance_squared(&point, &a, &b, &Real::from(26)).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn circle_line2_relation_uses_squared_discriminant() {
        let center = p2(0, 0);
        let radius_squared = Real::from(25);

        assert_eq!(
            classify_circle_line2(&center, &radius_squared, &p2(-10, 0), &p2(10, 0)).value(),
            Some(CircleLineRelation::Secant)
        );
        assert_eq!(
            classify_circle_line2(&center, &radius_squared, &p2(-10, 5), &p2(10, 5)).value(),
            Some(CircleLineRelation::Tangent)
        );
        assert_eq!(
            classify_circle_line2(&center, &radius_squared, &p2(-10, 6), &p2(10, 6)).value(),
            Some(CircleLineRelation::Disjoint)
        );
        assert_eq!(
            classify_circle_line2(&center, &radius_squared, &p2(1, 1), &p2(1, 1)).value(),
            Some(CircleLineRelation::DegenerateLine)
        );
    }

    #[test]
    fn circle_segment2_relation_respects_closed_segment_interval() {
        let center = p2(0, 0);
        let radius_squared = Real::from(25);

        assert_eq!(
            classify_circle_segment2(&center, &radius_squared, &p2(-10, 0), &p2(10, 0)).value(),
            Some(CircleSegmentRelation::Secant)
        );
        assert_eq!(
            classify_circle_segment2(&center, &radius_squared, &p2(-10, 5), &p2(10, 5)).value(),
            Some(CircleSegmentRelation::Tangent)
        );
        assert_eq!(
            classify_circle_segment2(&center, &radius_squared, &p2(-2, 0), &p2(2, 0)).value(),
            Some(CircleSegmentRelation::ContainedInside)
        );
        assert_eq!(
            classify_circle_segment2(&center, &radius_squared, &p2(6, 0), &p2(10, 0)).value(),
            Some(CircleSegmentRelation::Disjoint)
        );
    }

    #[test]
    fn point_segment3_distance_comparison_selects_endpoint_or_interior() {
        let a = p3(0, 0, 0);
        let b = p3(10, 0, 0);

        assert_eq!(
            compare_point_segment3_distance_squared(&p3(5, 3, 4), &a, &b, &Real::from(25)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point_segment3_distance_squared(&p3(13, 4, 0), &a, &b, &Real::from(25)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point_segment3_distance_squared(&p3(-1, 0, 0), &a, &b, &Real::from(0)).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn point_plane_distance_comparison_uses_unnormalized_normal() {
        let plane = crate::plane::Plane3::new(p3(0, 0, 2), Real::from(-6));
        let point = p3(0, 0, 5);

        assert_eq!(
            compare_point_plane_distance_squared(&point, &plane, &Real::from(3)).value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            compare_point_plane_distance_squared(&point, &plane, &Real::from(4)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point_plane_distance_squared(&point, &plane, &Real::from(5)).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn degenerate_distance_carriers_have_explicit_exact_fallbacks() {
        let point = p3(3, 4, 0);
        let anchor = p3(0, 0, 0);
        let zero_normal_plane = crate::plane::Plane3::new(p3(0, 0, 0), Real::from(5));

        assert_eq!(
            compare_point_line3_distance_squared(&point, &anchor, &anchor, &Real::from(25)).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            compare_point_segment3_distance_squared(&point, &anchor, &anchor, &Real::from(24))
                .value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            compare_point_plane_distance_squared(&point, &zero_normal_plane, &Real::from(25))
                .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn sphere3_intersection_uses_radius_sum_squared() {
        let first = p3(0, 0, 0);
        let second = p3(3, 4, 0);

        assert_eq!(
            classify_sphere3_intersection(&first, &Real::from(2), &second, &Real::from(2)).value(),
            Some(SphereIntersection::Disjoint)
        );
        assert_eq!(
            classify_sphere3_intersection(&first, &Real::from(2), &second, &Real::from(3)).value(),
            Some(SphereIntersection::Touching)
        );
        assert_eq!(
            classify_sphere3_intersection(&first, &Real::from(4), &second, &Real::from(3)).value(),
            Some(SphereIntersection::Overlapping)
        );
        assert!(
            classify_sphere3_intersection(&first, &Real::from(-1), &second, &Real::from(3))
                .value()
                .is_none()
        );
    }

    #[test]
    fn aabb3_sphere_intersection_uses_closest_box_point_distance() {
        let min = p3(0, 0, 0);
        let max = p3(2, 2, 2);

        assert_eq!(
            classify_aabb3_sphere_intersection(&min, &max, &p3(5, 2, 2), &Real::from(4)).value(),
            Some(AabbSphereIntersection::Disjoint)
        );
        assert_eq!(
            classify_aabb3_sphere_intersection(&min, &max, &p3(5, 2, 2), &Real::from(9)).value(),
            Some(AabbSphereIntersection::Touching)
        );
        assert_eq!(
            classify_aabb3_sphere_intersection(&min, &max, &p3(1, 1, 1), &Real::from(1)).value(),
            Some(AabbSphereIntersection::Overlapping)
        );
    }
}
