//! Squared-distance comparisons and circle/ball contact classifications.
//!
//! Rational kernels preserve exact arithmetic; generic expressions use explicit
//! predicate policies and can return Unknown or approximate decisions. Squared
//! formulas avoid square roots, but some composite paths discard branch receipts;
//! each entrypoint documents its aggregation limits. Degenerate-carrier conventions
//! and circle/segment endpoint labels are not universal geometric certificates.
//! These functions supply no separate total-work or coefficient-growth budget.

use core::cmp::Ordering;

use crate::classify::{
    AabbSphereIntersection, CircleLineRelation, CircleSegmentRelation, SphereIntersection,
    SpherePointLocation,
};
use crate::geometry::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed};
use crate::predicates::order::compare_reals_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use hyperreal::{Rational, Real};

/// Compare `|anchor-left|^2` with `|anchor-right|^2` without constructing square roots.
///
/// First tries a fused rational difference when every coordinate exposes a borrowed
/// exact rational, returning Exact/Exact. Otherwise builds Real squared distances
/// and returns the policy-controlled comparison, including approximation or Unknown.
/// Less selects the nearer left candidate; equality preserves a distance tie.
pub fn compare_point2_distance_squared_with_policy(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if let Some(outcome) = exact_rational_point2_distance_ordering(anchor, left, right) {
        return outcome;
    }
    let left_distance = squared_distance2(anchor, left);
    let right_distance = squared_distance2(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Compare two squared spatial distances from a common anchor without square roots.
///
/// Uses a fused rational difference when all coordinates expose borrowed exact
/// rationals, otherwise compares Real expressions under policy. Returns the
/// selected path's receipt or Unknown; see the planar counterpart for ordering.
pub fn compare_point3_distance_squared_with_policy(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if let Some(outcome) = exact_rational_point3_distance_ordering(anchor, left, right) {
        return outcome;
    }
    let left_distance = squared_distance3(anchor, left);
    let right_distance = squared_distance3(anchor, right);
    compare_reals_with_policy(&left_distance, &right_distance, policy)
}

/// Classify a circle boundary against the infinite line through a and b.
///
/// For a nonzero direction d, compares `((center-a) x d)^2` with
/// `radius_squared * |d|^2`: less/equal/greater gives Secant/Tangent/Disjoint.
/// Coincident endpoints give DegenerateLine without classifying the point.
/// Nonnegative squared radius is a caller precondition, not checked here.
///
/// A borrowed nondyadic rational squared radius enables the all-rational fast path.
/// Otherwise the direction-norm guard runs before the discriminant. Its receipt
/// is retained for DegenerateLine but discarded on the nondegenerate path, whose
/// outcome contains only the final comparison receipt. Thus approximate-policy
/// branch qualification is not fully aggregated. Encountered Unknown propagates.
pub fn classify_circle_line2_with_policy(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<CircleLineRelation> {
    if exact_non_dyadic_rational(radius_squared)
        && let Some(outcome) = exact_rational_circle_line2(center, radius_squared, a, b)
    {
        return outcome;
    }
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

/// Classify a circle boundary and closed segment using endpoint-distance partitions.
///
/// Requires a nonnegative squared radius; this is not validated. A collapsed
/// segment is ContainedInside, Tangent, or Disjoint according to its point distance.
/// For distinct endpoints, two on-boundary points give Secant; one on-boundary
/// endpoint gives Tangent if the other is inside, and Secant if it is outside.
/// One inside/one outside gives Secant; two inside give ContainedInside. For two
/// outside endpoints, minimum segment distance selects Secant/Tangent/Disjoint.
/// These labels do not certify differential tangency or the number of boundary
/// intersections; see [`CircleSegmentRelation`].
///
/// Tries the rational kernel for a borrowed nondyadic rational squared radius.
/// The generic path discards the direction-norm guard's receipt, then combines endpoint and
/// any minimum-distance receipts. The latter helper also discards its projection
/// guards, so the outer receipt may omit branch qualification. Both endpoint calls
/// run before the first Unknown is selected; other encountered Unknown propagates.
pub fn classify_circle_segment2_with_policy(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<CircleSegmentRelation> {
    if exact_non_dyadic_rational(radius_squared)
        && let Some(outcome) = exact_rational_circle_segment2(center, radius_squared, a, b)
    {
        return outcome;
    }
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

/// Compare squared distance to the infinite spatial line ab with a scalar threshold.
///
/// For nonzero d = b-a, compares `|(point-a) x d|^2` with
/// `threshold_squared * |d|^2`, without projection, division, or square root.
/// Coincident endpoints reduce to distance from a. The threshold may be any Real;
/// a negative value is compared algebraically rather than rejected.
///
/// A borrowed nondyadic rational threshold enables the rational kernel. The
/// generic path drops the direction-norm guard's receipt and returns the selected
/// distance comparison's receipt, so qualification may omit a branch decision.
/// An unresolved guard or final comparison returns Unknown.
pub fn compare_point_line3_distance_squared_with_policy(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if exact_non_dyadic_rational(threshold_squared)
        && let Some(outcome) =
            exact_rational_point_line3_distance_ordering(point, a, b, threshold_squared)
    {
        return outcome;
    }
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

/// Compare squared distance to closed segment ab with a scalar threshold.
///
/// Coincident endpoints use distance to a. Otherwise projection `u = (point-a).d`
/// selects a for u <= 0, b for u >= |d|^2, or the infinite-line distance between
/// them. No division or square root is needed; negative thresholds are not rejected.
///
/// A borrowed nondyadic rational threshold enables the rational kernel. In the
/// generic path, norm and projection guard receipts are discarded; only the chosen
/// distance outcome is returned. The interior line helper checks the norm again.
/// Unknown guards propagate, but successful approximate guards are not necessarily
/// represented by the final receipt.
pub fn compare_point_segment3_distance_squared_with_policy(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if exact_non_dyadic_rational(threshold_squared)
        && let Some(outcome) =
            exact_rational_point_segment3_distance_ordering(point, a, b, threshold_squared)
    {
        return outcome;
    }
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

/// Compare squared distance to a closed spatial triangle with a scalar threshold.
///
/// First tries the all-rational kernel, including dyadic thresholds. Otherwise a
/// zero squared normal reduces to the minimum distance to AB, BC, CA, covering
/// collinear or repeated vertices. A positive squared normal permits barycentric
/// numerators for the orthogonal projection. All three are compared with zero;
/// any negative numerator selects the edge minimum even if another is unresolved.
/// With no negative numerator, uncertainty propagates; all nonnegative numerators
/// select the squared plane-expression comparison scaled by the squared normal.
/// A decided negative squared normal becomes Unsupported with that decision's stage.
///
/// The generic path aggregates its decided normal/barycentric receipts with the
/// selected distance outcome. Edge helpers currently discard some internal branch
/// receipts, so this aggregation does not restore all inherited qualification.
/// Negative scalar thresholds are accepted for comparison, not validated as radii.
/// No closest point or barycentric coordinates are constructed.
pub fn compare_point_triangle3_distance_squared_with_policy(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if let Some(outcome) =
        exact_rational_point_triangle3_distance_ordering(point, a, b, c, threshold_squared)
    {
        return outcome;
    }

    let ab = vector3_between(a, b);
    let ac = vector3_between(a, c);
    let normal = cross3(&ab, &ac);
    let normal_norm = norm_squared3(&normal);
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    match compare_reals_with_policy(&normal_norm, &0.into(), policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: decision_certainty,
            stage: decision_stage,
        } => {
            return merge_distance_outcome(
                compare_point_triangle_edge_minimum(point, a, b, c, threshold_squared, policy),
                decision_certainty,
                decision_stage,
            );
        }
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            certainty: decision_certainty,
            stage: decision_stage,
        } => {
            certainty = max_distance_certainty(certainty, decision_certainty);
            stage = max_distance_stage(stage, decision_stage);
        }
        PredicateOutcome::Decided {
            value: Ordering::Less,
            stage,
            ..
        } => {
            return PredicateOutcome::unknown(RefinementNeed::Unsupported, stage);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    let ap = vector3_between(a, point);
    let ab_norm = norm_squared3(&ab);
    let ac_norm = norm_squared3(&ac);
    let ab_ac = dot3(&ab, &ac);
    let ap_ab = dot3(&ap, &ab);
    let ap_ac = dot3(&ap, &ac);
    let determinant = sub_ref(&mul_ref(&ab_norm, &ac_norm), &mul_ref(&ab_ac, &ab_ac));
    let beta_numerator = sub_ref(&mul_ref(&ac_norm, &ap_ab), &mul_ref(&ab_ac, &ap_ac));
    let gamma_numerator = sub_ref(&mul_ref(&ab_norm, &ap_ac), &mul_ref(&ab_ac, &ap_ab));
    let alpha_numerator = sub_ref(&sub_ref(&determinant, &beta_numerator), &gamma_numerator);

    let mut outside = false;
    let mut unknown = None;
    for numerator in [&alpha_numerator, &beta_numerator, &gamma_numerator] {
        match compare_reals_with_policy(numerator, &0.into(), policy) {
            PredicateOutcome::Decided {
                value,
                certainty: decision_certainty,
                stage: decision_stage,
            } => {
                certainty = max_distance_certainty(certainty, decision_certainty);
                stage = max_distance_stage(stage, decision_stage);
                outside |= value == Ordering::Less;
            }
            PredicateOutcome::Unknown { needed, stage } => {
                unknown.get_or_insert((needed, stage));
            }
        }
    }

    if outside {
        return merge_distance_outcome(
            compare_point_triangle_edge_minimum(point, a, b, c, threshold_squared, policy),
            certainty,
            stage,
        );
    }
    if let Some((needed, stage)) = unknown {
        return PredicateOutcome::unknown(needed, stage);
    }

    let signed_distance_numerator = dot3(&ap, &normal);
    let numerator_squared = mul_ref(&signed_distance_numerator, &signed_distance_numerator);
    let scaled_threshold = mul_ref(threshold_squared, &normal_norm);
    merge_distance_outcome(
        compare_reals_with_policy(&numerator_squared, &scaled_threshold, policy),
        certainty,
        stage,
    )
}

/// Compare a squared plane expression with a scaled squared-distance threshold.
///
/// For nonzero normal n, compares `(n.point + offset)^2` with
/// `threshold_squared * |n|^2`, without normalization or square root. A zero normal
/// instead compares `offset^2` directly with the threshold. That fallback is a
/// scalar convention, not Euclidean distance to the zero-normal equation's set;
/// invalid planes are not rejected. Negative thresholds are also not rejected.
///
/// A borrowed nondyadic rational threshold enables the rational kernel. The
/// generic path discards the normal-norm guard's receipt, returning only the
/// selected comparison outcome. Unknown propagates, but successful branch
/// qualification may be missing from the final receipt.
pub fn compare_point_plane_distance_squared_with_policy(
    point: &Point3,
    plane: &crate::plane::Plane3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if exact_non_dyadic_rational(threshold_squared)
        && let Some(outcome) =
            exact_rational_point_plane_distance_ordering(point, plane, threshold_squared)
    {
        return outcome;
    }
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

/// Classify intersection of two closed balls using radii, not squared radii.
///
/// Checks each radius against zero in order. A negative radius returns
/// Unknown(Unsupported, Undecided); unresolved radius checks propagate. Otherwise
/// compares center-distance squared with `(first_radius + second_radius)^2`.
/// Less/equal/greater means Overlapping/Touching/Disjoint, including containment
/// and zero-radius cases; it does not classify intersection of surface spheres.
///
/// Successful radius-validation receipts are discarded. The result carries only
/// the distance-comparison receipt, which may omit approximate domain decisions.
pub fn classify_sphere3_intersection_with_policy(
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

/// Compare a ball's squared radius with its center's minimum squared distance to a box.
///
/// Each pair of corner coordinates may be reversed: sorts them under policy,
/// then clamps the center on x, y, z in order. Sums squared outside-axis distances
/// and compares with radius_squared, combining all consumed decision receipts.
/// An encountered Unknown stops the query. No square root is constructed.
///
/// Requires a nonnegative squared radius, without checking it. Zero-width boxes
/// are accepted. Touching means equality of the two squared quantities, including
/// a zero-radius ball centered anywhere in the box; Overlapping does not promise
/// positive intersection volume for degenerate inputs.
pub fn classify_aabb3_sphere_intersection_with_policy(
    min: &Point3,
    max: &Point3,
    center: &Point3,
    radius_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<AabbSphereIntersection> {
    let mut trace = (Certainty::Exact, Escalation::Structural);
    let distance_squared = {
        let dx = match outside_interval_delta(&center.x, &min.x, &max.x, policy, &mut trace) {
            Ok(delta) => delta,
            Err(outcome) => return outcome,
        };
        let dy = match outside_interval_delta(&center.y, &min.y, &max.y, policy, &mut trace) {
            Ok(delta) => delta,
            Err(outcome) => return outcome,
        };
        let dz = match outside_interval_delta(&center.z, &min.z, &max.z, policy, &mut trace) {
            Ok(delta) => delta,
            Err(outcome) => return outcome,
        };
        Real::signed_product_sum([true; 3], [[&dx, &dx], [&dy, &dy], [&dz, &dz]])
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
            PredicateOutcome::decided(
                relation,
                max_distance_certainty(trace.0, certainty),
                max_distance_stage(trace.1, stage),
            )
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Map squared center-to-point distance versus squared radius to Inside, On, or Outside.
///
/// Uses Real squared-distance expressions and retains the comparison receipt or
/// Unknown. Nonnegative radius_squared is a caller precondition and is not checked;
/// there is no separate rational-kernel dispatch at this entrypoint.
pub fn classify_point_sphere3_with_policy(
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

/// Return a typed Unknown for a negative or unresolved radius; None permits continuation.
///
/// Negative maps to Unsupported/Undecided. Successful nonnegative comparisons lose
/// their receipts, including approximate qualification; None is not proof data.
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

/// Compute distance to the interval spanned by two endpoints, accumulating decision receipts.
///
/// Orders endpoints first, then compares with the lower bound; a strict lower
/// violation returns immediately. Otherwise checks the upper bound and returns its
/// violation or zero for inclusive membership. Unknown stops with the AABB/ball
/// outcome type. Successful earlier decisions remain in trace even on later failure.
#[inline(always)]
fn outside_interval_delta(
    value: &Real,
    first: &Real,
    second: &Real,
    policy: PredicatePolicy,
    trace: &mut (Certainty, Escalation),
) -> Result<Real, PredicateOutcome<AabbSphereIntersection>> {
    let (min, max) = match compare_reals_with_policy(first, second, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            trace.0 = max_distance_certainty(trace.0, certainty);
            trace.1 = max_distance_stage(trace.1, stage);
            if value == Ordering::Greater {
                (second, first)
            } else {
                (first, second)
            }
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    };
    match compare_reals_with_policy(value, min, policy) {
        PredicateOutcome::Decided {
            value: ordering,
            certainty,
            stage,
        } => {
            trace.0 = max_distance_certainty(trace.0, certainty);
            trace.1 = max_distance_stage(trace.1, stage);
            if ordering == Ordering::Less {
                return Ok(sub_ref(min, value));
            }
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return Err(PredicateOutcome::unknown(needed, stage));
        }
    }
    match compare_reals_with_policy(value, max, policy) {
        PredicateOutcome::Decided {
            value: ordering,
            certainty,
            stage,
        } => {
            trace.0 = max_distance_certainty(trace.0, certainty);
            trace.1 = max_distance_stage(trace.1, stage);
            Ok(if ordering == Ordering::Greater {
                sub_ref(value, max)
            } else {
                Real::from(0)
            })
        }
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

/// Compare the minimum of three closed-edge squared distances with the threshold.
///
/// Queries AB, BC, CA in order. A Less result is sufficient by itself and returns
/// immediately with that edge's receipt, even after earlier Unknown or weaker
/// decisions. Otherwise remembers the first Unknown and returns it after the scan;
/// only an entirely decided scan can yield Equal or Greater with combined receipts.
/// Does not repair receipts omitted by the segment helpers.
fn compare_point_triangle_edge_minimum(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let mut equal = false;
    let mut unknown = None;
    for (start, end) in [(a, b), (b, c), (c, a)] {
        match compare_point_segment3_distance_squared_with_policy(
            point,
            start,
            end,
            threshold_squared,
            policy,
        ) {
            PredicateOutcome::Decided {
                value: Ordering::Less,
                certainty,
                stage,
            } => return PredicateOutcome::decided(Ordering::Less, certainty, stage),
            PredicateOutcome::Decided {
                value,
                certainty: decision_certainty,
                stage: decision_stage,
            } => {
                equal |= value == Ordering::Equal;
                certainty = max_distance_certainty(certainty, decision_certainty);
                stage = max_distance_stage(stage, decision_stage);
            }
            PredicateOutcome::Unknown {
                needed,
                stage: unknown_stage,
            } => {
                unknown.get_or_insert((needed, unknown_stage));
            }
        }
    }
    if let Some((needed, stage)) = unknown {
        PredicateOutcome::unknown(needed, stage)
    } else {
        PredicateOutcome::decided(
            if equal {
                Ordering::Equal
            } else {
                Ordering::Greater
            },
            certainty,
            stage,
        )
    }
}

/// Merge incoming certainty/stage into a decided distance outcome; forward Unknown unchanged.
fn merge_distance_outcome(
    outcome: PredicateOutcome<Ordering>,
    certainty: Certainty,
    stage: Escalation,
) -> PredicateOutcome<Ordering> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty: decision_certainty,
            stage: decision_stage,
        } => PredicateOutcome::decided(
            value,
            max_distance_certainty(certainty, decision_certainty),
            max_distance_stage(stage, decision_stage),
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Compare spatial point distance with an arbitrary scalar threshold under policy.
///
/// A borrowed nondyadic rational threshold first enables the all-rational kernel;
/// otherwise compares the constructed squared-distance expression. Returns the
/// selected comparison receipt, without validating threshold nonnegativity.
fn compare_point3_distance_squared_to_threshold_with_policy(
    point: &Point3,
    target: &Point3,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if exact_non_dyadic_rational(threshold_squared)
        && let Some(outcome) =
            exact_rational_point3_distance_threshold_ordering(point, target, threshold_squared)
    {
        return outcome;
    }
    let distance_squared = squared_distance3(point, target);
    compare_reals_with_policy(&distance_squared, threshold_squared, policy)
}

/// Compare planar point distance with an arbitrary scalar threshold under policy.
///
/// Tries the all-rational kernel only for a borrowed nondyadic rational threshold,
/// then falls back to Real expressions. Preserves the selected comparison outcome.
fn compare_point2_distance_squared_to_threshold_with_policy(
    point: &Point2,
    target: &Point2,
    threshold_squared: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if exact_non_dyadic_rational(threshold_squared)
        && let Some(outcome) =
            exact_rational_point2_distance_threshold_ordering(point, target, threshold_squared)
    {
        return outcome;
    }
    let distance_squared = squared_distance2(point, target);
    compare_reals_with_policy(&distance_squared, threshold_squared, policy)
}

/// Compare squared distance to a closed planar segment with a scalar threshold.
///
/// Tests the direction norm, then projection against zero and that norm to select
/// endpoint a, endpoint b, or the interior line. The interior comparison is
/// `cross(point-a,b-a)^2` versus `threshold_squared * |b-a|^2`. Coincident endpoints
/// use distance to a; negative thresholds are compared without rejection.
///
/// Unlike the spatial entrypoint, has no whole-segment rational dispatch. Norm and
/// projection guards discard their successful receipts; the returned outcome is
/// only the chosen distance comparison, so branch qualification can be lost.
/// An encountered Unknown propagates.
pub fn compare_point_segment2_distance_squared_with_policy(
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

/// Construct `(right.x-left.x)^2 + (right.y-left.y)^2` as Real expressions.
fn squared_distance2(left: &Point2, right: &Point2) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy))
}

/// Compare planar squared distances using one fused rational signed product sum.
///
/// Requires borrowed exact rationals for all six coordinates; otherwise returns
/// None without symbolic normalization. Success carries Exact/Exact.
#[inline]
fn exact_rational_point2_distance_ordering(
    anchor: &Point2,
    left: &Point2,
    right: &Point2,
) -> Option<PredicateOutcome<Ordering>> {
    let ax = anchor.x.exact_rational_ref()?;
    let ay = anchor.y.exact_rational_ref()?;
    let lx = left.x.exact_rational_ref()?;
    let ly = left.y.exact_rational_ref()?;
    let rx = right.x.exact_rational_ref()?;
    let ry = right.y.exact_rational_ref()?;
    let ldx = lx - ax;
    let ldy = ly - ay;
    let rdx = rx - ax;
    let rdy = ry - ay;
    Some(exact_distance_outcome(
        Rational::signed_product_sum_ordering(
            [true, true, false, false],
            [[&ldx, &ldx], [&ldy, &ldy], [&rdx, &rdx], [&rdy, &rdy]],
        ),
    ))
}

/// Compare spatial squared distances using a fused rational signed product sum.
///
/// Requires borrowed exact rationals for all nine coordinates, otherwise None.
/// No symbolic normalization is attempted; success carries Exact/Exact.
#[inline]
fn exact_rational_point3_distance_ordering(
    anchor: &Point3,
    left: &Point3,
    right: &Point3,
) -> Option<PredicateOutcome<Ordering>> {
    let ax = anchor.x.exact_rational_ref()?;
    let ay = anchor.y.exact_rational_ref()?;
    let az = anchor.z.exact_rational_ref()?;
    let lx = left.x.exact_rational_ref()?;
    let ly = left.y.exact_rational_ref()?;
    let lz = left.z.exact_rational_ref()?;
    let rx = right.x.exact_rational_ref()?;
    let ry = right.y.exact_rational_ref()?;
    let rz = right.z.exact_rational_ref()?;
    let ldx = lx - ax;
    let ldy = ly - ay;
    let ldz = lz - az;
    let rdx = rx - ax;
    let rdy = ry - ay;
    let rdz = rz - az;
    Some(exact_distance_outcome(
        Rational::signed_product_sum_ordering(
            [true, true, true, false, false, false],
            [
                [&ldx, &ldx],
                [&ldy, &ldy],
                [&ldz, &ldz],
                [&rdx, &rdx],
                [&rdy, &rdy],
                [&rdz, &rdz],
            ],
        ),
    ))
}

/// Attach Exact/Exact to the rational planar threshold comparison, or forward its None.
#[inline]
fn exact_rational_point2_distance_threshold_ordering(
    point: &Point2,
    target: &Point2,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    Some(exact_distance_outcome(
        exact_rational_point2_distance_threshold(point, target, threshold_squared)?,
    ))
}

/// Compare planar squared distance minus threshold in one rational product sum.
///
/// Returns None if any coordinate or threshold lacks a borrowed exact rational.
/// The threshold may be negative; no geometric radius validation occurs.
#[inline]
fn exact_rational_point2_distance_threshold(
    point: &Point2,
    target: &Point2,
    threshold_squared: &Real,
) -> Option<Ordering> {
    let px = point.x.exact_rational_ref()?;
    let py = point.y.exact_rational_ref()?;
    let tx = target.x.exact_rational_ref()?;
    let ty = target.y.exact_rational_ref()?;
    let threshold = threshold_squared.exact_rational_ref()?;
    let dx = px - tx;
    let dy = py - ty;
    let one = Rational::one();
    Some(Rational::signed_product_sum_ordering(
        [true, true, false],
        [[&dx, &dx], [&dy, &dy], [threshold, &one]],
    ))
}

/// Attach Exact/Exact to the rational spatial threshold comparison, or forward its None.
#[inline]
fn exact_rational_point3_distance_threshold_ordering(
    point: &Point3,
    target: &Point3,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    Some(exact_distance_outcome(
        exact_rational_point3_distance_threshold(point, target, threshold_squared)?,
    ))
}

/// Compare spatial squared distance minus threshold in one rational product sum.
///
/// Requires borrowed exact rationals for all coordinates and threshold, otherwise
/// None. Does not normalize symbolic expressions or validate a radius domain.
#[inline]
fn exact_rational_point3_distance_threshold(
    point: &Point3,
    target: &Point3,
    threshold_squared: &Real,
) -> Option<Ordering> {
    let px = point.x.exact_rational_ref()?;
    let py = point.y.exact_rational_ref()?;
    let pz = point.z.exact_rational_ref()?;
    let tx = target.x.exact_rational_ref()?;
    let ty = target.y.exact_rational_ref()?;
    let tz = target.z.exact_rational_ref()?;
    let threshold = threshold_squared.exact_rational_ref()?;
    let dx = px - tx;
    let dy = py - ty;
    let dz = pz - tz;
    let one = Rational::one();
    Some(Rational::signed_product_sum_ordering(
        [true, true, true, false],
        [[&dx, &dx], [&dy, &dy], [&dz, &dz], [threshold, &one]],
    ))
}

/// Classify circle/line contact entirely with borrowed rational data, or return None.
///
/// Zero direction yields DegenerateLine; otherwise compares squared cross product
/// with radius_squared times squared direction. Every result is Exact/Exact, but
/// radius nonnegativity is a caller precondition. Supports dyadic values too;
/// the public caller's nondyadic gate is a dispatch choice.
#[inline]
fn exact_rational_circle_line2(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
) -> Option<PredicateOutcome<CircleLineRelation>> {
    let cx = center.x.exact_rational_ref()?;
    let cy = center.y.exact_rational_ref()?;
    let ax = a.x.exact_rational_ref()?;
    let ay = a.y.exact_rational_ref()?;
    let bx = b.x.exact_rational_ref()?;
    let by = b.y.exact_rational_ref()?;
    let radius = radius_squared.exact_rational_ref()?;
    let dx = bx - ax;
    let dy = by - ay;
    let direction_norm = Rational::signed_product_sum([true; 2], [[&dx, &dx], [&dy, &dy]]);
    if direction_norm.is_zero() {
        return Some(PredicateOutcome::decided(
            CircleLineRelation::DegenerateLine,
            Certainty::Exact,
            Escalation::Exact,
        ));
    }
    let ox = cx - ax;
    let oy = cy - ay;
    let cross = Rational::signed_product_sum([true, false], [[&ox, &dy], [&oy, &dx]]);
    let ordering = Rational::signed_product_sum_ordering(
        [true, false],
        [[&cross, &cross], [radius, &direction_norm]],
    );
    let relation = match ordering {
        Ordering::Less => CircleLineRelation::Secant,
        Ordering::Equal => CircleLineRelation::Tangent,
        Ordering::Greater => CircleLineRelation::Disjoint,
    };
    Some(PredicateOutcome::decided(
        relation,
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Apply the circle/segment endpoint partition using rational arithmetic only.
///
/// Requires borrowed rationals for all coordinates and radius_squared, otherwise
/// None; does not validate the radius domain. Handles collapse, endpoint contacts,
/// inside/outside partitions, and the projected minimum for two outside endpoints.
/// Uses the same Tangent/Secant conventions as [`CircleSegmentRelation`] and returns
/// Exact/Exact. Dyadic values are supported when this helper is called directly.
#[inline]
fn exact_rational_circle_segment2(
    center: &Point2,
    radius_squared: &Real,
    a: &Point2,
    b: &Point2,
) -> Option<PredicateOutcome<CircleSegmentRelation>> {
    let cx = center.x.exact_rational_ref()?;
    let cy = center.y.exact_rational_ref()?;
    let ax = a.x.exact_rational_ref()?;
    let ay = a.y.exact_rational_ref()?;
    let bx = b.x.exact_rational_ref()?;
    let by = b.y.exact_rational_ref()?;
    let radius = radius_squared.exact_rational_ref()?;
    let dx = bx - ax;
    let dy = by - ay;
    let direction_norm = Rational::signed_product_sum([true; 2], [[&dx, &dx], [&dy, &dy]]);
    let a_distance = exact_rational_point2_distance_threshold(a, center, radius_squared)?;
    if direction_norm.is_zero() {
        let relation = match a_distance {
            Ordering::Less => CircleSegmentRelation::ContainedInside,
            Ordering::Equal => CircleSegmentRelation::Tangent,
            Ordering::Greater => CircleSegmentRelation::Disjoint,
        };
        return Some(PredicateOutcome::decided(
            relation,
            Certainty::Exact,
            Escalation::Exact,
        ));
    }
    let b_distance = exact_rational_point2_distance_threshold(b, center, radius_squared)?;
    let relation = if a_distance == Ordering::Equal && b_distance == Ordering::Equal {
        CircleSegmentRelation::Secant
    } else if a_distance == Ordering::Equal || b_distance == Ordering::Equal {
        let other = if a_distance == Ordering::Equal {
            b_distance
        } else {
            a_distance
        };
        if other == Ordering::Less {
            CircleSegmentRelation::Tangent
        } else {
            CircleSegmentRelation::Secant
        }
    } else if (a_distance == Ordering::Less) != (b_distance == Ordering::Less) {
        CircleSegmentRelation::Secant
    } else if a_distance == Ordering::Less {
        CircleSegmentRelation::ContainedInside
    } else {
        let apx = cx - ax;
        let apy = cy - ay;
        let projection = Rational::signed_product_sum([true; 2], [[&apx, &dx], [&apy, &dy]]);
        let distance_ordering = if !projection.is_positive() {
            a_distance
        } else if projection >= direction_norm {
            b_distance
        } else {
            let cross = Rational::signed_product_sum([true, false], [[&apx, &dy], [&apy, &dx]]);
            Rational::signed_product_sum_ordering(
                [true, false],
                [[&cross, &cross], [radius, &direction_norm]],
            )
        };
        match distance_ordering {
            Ordering::Less => CircleSegmentRelation::Secant,
            Ordering::Equal => CircleSegmentRelation::Tangent,
            Ordering::Greater => CircleSegmentRelation::Disjoint,
        }
    };
    Some(PredicateOutcome::decided(
        relation,
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Compare rational squared line distance against a rational threshold, or return None.
///
/// All coordinates and threshold must expose borrowed exact rationals. Collapsed
/// endpoints reduce to point distance; otherwise a fused cross-norm/threshold sum
/// avoids division. Success is Exact/Exact; negative thresholds are allowed.
#[inline]
fn exact_rational_point_line3_distance_ordering(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    let [
        Some(px),
        Some(py),
        Some(pz),
        Some(ax),
        Some(ay),
        Some(az),
        Some(bx),
        Some(by),
        Some(bz),
        Some(threshold),
    ] = [
        &point.x,
        &point.y,
        &point.z,
        &a.x,
        &a.y,
        &a.z,
        &b.x,
        &b.y,
        &b.z,
        threshold_squared,
    ]
    .map(Real::exact_rational_ref)
    else {
        return None;
    };
    let dx = bx - ax;
    let dy = by - ay;
    let dz = bz - az;
    let direction_norm =
        Rational::signed_product_sum([true; 3], [[&dx, &dx], [&dy, &dy], [&dz, &dz]]);
    if direction_norm.is_zero() {
        return exact_rational_point3_distance_threshold_ordering(point, a, threshold_squared);
    }
    let ox = px - ax;
    let oy = py - ay;
    let oz = pz - az;
    let cross_x = Rational::signed_product_sum([true, false], [[&oy, &dz], [&oz, &dy]]);
    let cross_y = Rational::signed_product_sum([true, false], [[&oz, &dx], [&ox, &dz]]);
    let cross_z = Rational::signed_product_sum([true, false], [[&ox, &dy], [&oy, &dx]]);
    Some(exact_distance_outcome(
        Rational::signed_product_sum_ordering(
            [true, true, true, false],
            [
                [&cross_x, &cross_x],
                [&cross_y, &cross_y],
                [&cross_z, &cross_z],
                [threshold, &direction_norm],
            ],
        ),
    ))
}

/// Select a rational closest endpoint or interior line and compare its squared distance.
///
/// Coordinates must expose borrowed exact rationals; the selected delegate also
/// requires a rational threshold. Collapse uses a; projection <= 0 uses a and
/// projection >= squared direction uses b. Missing rational data gives None;
/// success is Exact/Exact, without threshold-domain validation.
#[inline]
fn exact_rational_point_segment3_distance_ordering(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    let [
        Some(px),
        Some(py),
        Some(pz),
        Some(ax),
        Some(ay),
        Some(az),
        Some(bx),
        Some(by),
        Some(bz),
    ] = [
        &point.x, &point.y, &point.z, &a.x, &a.y, &a.z, &b.x, &b.y, &b.z,
    ]
    .map(Real::exact_rational_ref)
    else {
        return None;
    };
    let dx = bx - ax;
    let dy = by - ay;
    let dz = bz - az;
    let direction_norm =
        Rational::signed_product_sum([true; 3], [[&dx, &dx], [&dy, &dy], [&dz, &dz]]);
    if direction_norm.is_zero() {
        return exact_rational_point3_distance_threshold_ordering(point, a, threshold_squared);
    }
    let apx = px - ax;
    let apy = py - ay;
    let apz = pz - az;
    let projection =
        Rational::signed_product_sum([true; 3], [[&apx, &dx], [&apy, &dy], [&apz, &dz]]);
    if !projection.is_positive() {
        return exact_rational_point3_distance_threshold_ordering(point, a, threshold_squared);
    }
    if projection >= direction_norm {
        return exact_rational_point3_distance_threshold_ordering(point, b, threshold_squared);
    }
    exact_rational_point_line3_distance_ordering(point, a, b, threshold_squared)
}

/// Compare rational closed-triangle distance by projection numerators or edge minimum.
///
/// Requires borrowed exact rationals for every coordinate and threshold, otherwise
/// None. Zero normal or an outside orthogonal projection uses AB/BC/CA distances;
/// an included projection uses squared plane expression scaled by squared normal.
/// No division or closest-point construction is required. Success is Exact/Exact,
/// including dyadic data and degenerate triangles; thresholds are not validated.
#[inline]
fn exact_rational_point_triangle3_distance_ordering(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    let [
        Some(px),
        Some(py),
        Some(pz),
        Some(ax),
        Some(ay),
        Some(az),
        Some(bx),
        Some(by),
        Some(bz),
        Some(cx),
        Some(cy),
        Some(cz),
        Some(threshold),
    ] = [
        &point.x,
        &point.y,
        &point.z,
        &a.x,
        &a.y,
        &a.z,
        &b.x,
        &b.y,
        &b.z,
        &c.x,
        &c.y,
        &c.z,
        threshold_squared,
    ]
    .map(Real::exact_rational_ref)
    else {
        return None;
    };

    let abx = bx - ax;
    let aby = by - ay;
    let abz = bz - az;
    let acx = cx - ax;
    let acy = cy - ay;
    let acz = cz - az;
    let apx = px - ax;
    let apy = py - ay;
    let apz = pz - az;
    let normal_x = Rational::signed_product_sum([true, false], [[&aby, &acz], [&abz, &acy]]);
    let normal_y = Rational::signed_product_sum([true, false], [[&abz, &acx], [&abx, &acz]]);
    let normal_z = Rational::signed_product_sum([true, false], [[&abx, &acy], [&aby, &acx]]);
    let normal_norm = Rational::signed_product_sum(
        [true; 3],
        [
            [&normal_x, &normal_x],
            [&normal_y, &normal_y],
            [&normal_z, &normal_z],
        ],
    );
    if normal_norm.is_zero() {
        return exact_rational_point_triangle_edge_minimum(point, a, b, c, threshold_squared)
            .map(exact_distance_outcome);
    }

    let ab_norm =
        Rational::signed_product_sum([true; 3], [[&abx, &abx], [&aby, &aby], [&abz, &abz]]);
    let ac_norm =
        Rational::signed_product_sum([true; 3], [[&acx, &acx], [&acy, &acy], [&acz, &acz]]);
    let ab_ac = Rational::signed_product_sum([true; 3], [[&abx, &acx], [&aby, &acy], [&abz, &acz]]);
    let ap_ab = Rational::signed_product_sum([true; 3], [[&apx, &abx], [&apy, &aby], [&apz, &abz]]);
    let ap_ac = Rational::signed_product_sum([true; 3], [[&apx, &acx], [&apy, &acy], [&apz, &acz]]);
    let determinant =
        Rational::signed_product_sum([true, false], [[&ab_norm, &ac_norm], [&ab_ac, &ab_ac]]);
    let beta_numerator =
        Rational::signed_product_sum([true, false], [[&ac_norm, &ap_ab], [&ab_ac, &ap_ac]]);
    let gamma_numerator =
        Rational::signed_product_sum([true, false], [[&ab_norm, &ap_ac], [&ab_ac, &ap_ab]]);
    let beta_gamma = &beta_numerator + &gamma_numerator;
    let alpha_numerator = &determinant - &beta_gamma;
    if alpha_numerator.is_negative()
        || beta_numerator.is_negative()
        || gamma_numerator.is_negative()
    {
        return exact_rational_point_triangle_edge_minimum(point, a, b, c, threshold_squared)
            .map(exact_distance_outcome);
    }

    let signed_distance_numerator = Rational::signed_product_sum(
        [true; 3],
        [[&apx, &normal_x], [&apy, &normal_y], [&apz, &normal_z]],
    );
    Some(exact_distance_outcome(
        Rational::signed_product_sum_ordering(
            [true, false],
            [
                [&signed_distance_numerator, &signed_distance_numerator],
                [threshold, &normal_norm],
            ],
        ),
    ))
}

/// Compare rational distances to AB, BC, CA, stopping at the first Less.
///
/// None from any visited delegate returns immediately. If all decide without Less,
/// any Equal yields Equal, otherwise Greater. Extracts values from delegates that
/// produce Exact/Exact; this helper returns only the ordering.
#[inline]
fn exact_rational_point_triangle_edge_minimum(
    point: &Point3,
    a: &Point3,
    b: &Point3,
    c: &Point3,
    threshold_squared: &Real,
) -> Option<Ordering> {
    let mut equal = false;
    for (start, end) in [(a, b), (b, c), (c, a)] {
        match exact_rational_point_segment3_distance_ordering(point, start, end, threshold_squared)?
            .value()?
        {
            Ordering::Less => return Some(Ordering::Less),
            Ordering::Equal => equal = true,
            Ordering::Greater => {}
        }
    }
    Some(if equal {
        Ordering::Equal
    } else {
        Ordering::Greater
    })
}

/// Compare rational plane-expression square with a scaled threshold, or return None.
///
/// Requires borrowed rational point, normal, offset, and threshold data. For zero
/// normal compares offset squared directly with threshold, matching the generic
/// scalar fallback rather than geometric distance to that equation. Success is
/// Exact/Exact; there is no plane validity or threshold-domain check.
#[inline]
fn exact_rational_point_plane_distance_ordering(
    point: &Point3,
    plane: &crate::plane::Plane3,
    threshold_squared: &Real,
) -> Option<PredicateOutcome<Ordering>> {
    let [
        Some(px),
        Some(py),
        Some(pz),
        Some(nx),
        Some(ny),
        Some(nz),
        Some(offset),
        Some(threshold),
    ] = [
        &point.x,
        &point.y,
        &point.z,
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
        threshold_squared,
    ]
    .map(Real::exact_rational_ref)
    else {
        return None;
    };
    let one = Rational::one();
    let expression =
        Rational::signed_product_sum([true; 4], [[nx, px], [ny, py], [nz, pz], [offset, &one]]);
    let normal_norm = Rational::signed_product_sum([true; 3], [[nx, nx], [ny, ny], [nz, nz]]);
    let ordering = if normal_norm.is_zero() {
        Rational::signed_product_sum_ordering(
            [true, false],
            [[&expression, &expression], [threshold, &one]],
        )
    } else {
        Rational::signed_product_sum_ordering(
            [true, false],
            [[&expression, &expression], [threshold, &normal_norm]],
        )
    };
    Some(exact_distance_outcome(ordering))
}

/// Attach Exact certainty and Exact escalation to a rationally established ordering.
#[inline]
fn exact_distance_outcome(ordering: Ordering) -> PredicateOutcome<Ordering> {
    PredicateOutcome::decided(ordering, Certainty::Exact, Escalation::Exact)
}

/// Test whether a scalar exposes a borrowed rational whose denominator is not a power of two.
///
/// This is a dispatch gate, not a proof that other representations are irrational;
/// it does not ask for an exact rational normal form.
#[inline(always)]
fn exact_non_dyadic_rational(value: &Real) -> bool {
    value
        .exact_rational_ref()
        .is_some_and(|rational| !rational.is_dyadic())
}

/// Construct the sum of squared coordinate differences for two spatial points.
fn squared_distance3(left: &Point3, right: &Point3) -> Real {
    let dx = sub_ref(&right.x, &left.x);
    let dy = sub_ref(&right.y, &left.y);
    let dz = sub_ref(&right.z, &left.z);
    let xy = add_ref(&mul_ref(&dx, &dx), &mul_ref(&dy, &dy));
    add_ref(&xy, &mul_ref(&dz, &dz))
}

/// Private unnormalized planar displacement expressions used by distance formulas.
#[derive(Clone, Debug)]
struct Vector2Real {
    /// X component of the unnormalized vector expression.
    x: Real,
    /// Y component of the unnormalized vector expression.
    y: Real,
}

/// Construct the planar displacement end-start without any length or nonzero test.
fn vector2_between(start: &Point2, end: &Point2) -> Vector2Real {
    Vector2Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
    }
}

/// Construct a planar dot product using the shared signed-product-sum kernel.
fn dot2(left: &Vector2Real, right: &Vector2Real) -> Real {
    Real::signed_product_sum([true; 2], [[&left.x, &right.x], [&left.y, &right.y]])
}

/// Construct the sum of two squared displacement components without taking a square root.
fn norm_squared2(vector: &Vector2Real) -> Real {
    Real::signed_product_sum([true; 2], [[&vector.x, &vector.x], [&vector.y, &vector.y]])
}

/// Private unnormalized spatial vector expressions for projection, normal, and distance formulas.
#[derive(Clone, Debug)]
struct Vector3Real {
    /// X component of the unnormalized vector expression.
    x: Real,
    /// Y component of the unnormalized vector expression.
    y: Real,
    /// Z component of the unnormalized vector expression.
    z: Real,
}

/// Construct the spatial displacement end-start without a nondegeneracy test.
fn vector3_between(start: &Point3, end: &Point3) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
        z: sub_ref(&end.z, &start.z),
    }
}

/// Construct left cross right from three differences of Real products, without resolving signs.
fn cross3(left: &Vector3Real, right: &Vector3Real) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&mul_ref(&left.y, &right.z), &mul_ref(&left.z, &right.y)),
        y: sub_ref(&mul_ref(&left.z, &right.x), &mul_ref(&left.x, &right.z)),
        z: sub_ref(&mul_ref(&left.x, &right.y), &mul_ref(&left.y, &right.x)),
    }
}

/// Construct a spatial dot product with the shared signed-product-sum kernel.
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

/// Construct the sum of three squared vector components without normalizing the vector.
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

/// Treat point coordinates as a vector and form their squared norm.
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

/// Construct `plane.normal . point + plane.offset` as a four-term signed product sum.
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

/// Select the weaker qualification in the order Exact, Filtered, Approximate.
fn max_distance_certainty(
    left: crate::predicate::Certainty,
    right: crate::predicate::Certainty,
) -> crate::predicate::Certainty {
    match (left, right) {
        (crate::predicate::Certainty::Approximate, _)
        | (_, crate::predicate::Certainty::Approximate) => crate::predicate::Certainty::Approximate,
        (crate::predicate::Certainty::Filtered, _) | (_, crate::predicate::Certainty::Filtered) => {
            crate::predicate::Certainty::Filtered
        }
        _ => crate::predicate::Certainty::Exact,
    }
}

/// Select the more advanced distance escalation stage, retaining left on an equal rank.
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

/// Rank Structural, Filter, Exact, Refined, Undecided as 0 through 4 for receipt aggregation.
fn distance_stage_rank(stage: crate::predicate::Escalation) -> u8 {
    match stage {
        crate::predicate::Escalation::Structural => 0,
        crate::predicate::Escalation::Filter => 1,
        crate::predicate::Escalation::Exact => 2,
        crate::predicate::Escalation::Refined => 3,
        crate::predicate::Escalation::Undecided => 4,
    }
}

/// Finite distance, contact-label, rational-kernel, and policy-receipt controls.
///
/// Most tests compare values; the rational and AABB-specific controls also inspect
/// selected complete receipts or dispatch counts. These are not exhaustive geometric
/// or representation-closure tests.
#[cfg(test)]
mod tests {
    use super::*;

    /// Policy authorizing the terminal approximation used by finite distance controls.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Construct a planar fixture with exact integer coordinates.
    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(hyperreal::Real::from(x), hyperreal::Real::from(y))
    }

    /// Construct a spatial fixture with exact integer coordinates.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            hyperreal::Real::from(x),
            hyperreal::Real::from(y),
            hyperreal::Real::from(z),
        )
    }

    /// Construct an exact integer ratio; panic if the denominator makes division invalid.
    fn r(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    /// Construct a planar point from two exact numerator/denominator pairs.
    fn rp2(x: (i32, i32), y: (i32, i32)) -> Point2 {
        Point2::new(r(x.0, x.1), r(y.0, y.1))
    }

    /// Construct a spatial point from three exact numerator/denominator pairs.
    fn rp3(x: (i32, i32), y: (i32, i32), z: (i32, i32)) -> Point3 {
        Point3::new(r(x.0, x.1), r(y.0, y.1), r(z.0, z.1))
    }

    /// Assert the original unit-circle identity normalizes to zero, then return the unresolved control.
    ///
    /// Separates that now-certified identity from the shared angle-addition expression
    /// used to exercise terminal-policy behavior.
    fn terminal_zero() -> Real {
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

    /// Translate an integer planar fixture by the symbolic vector (pi, e).
    fn shifted2(x: i32, y: i32) -> Point2 {
        Point2::new(Real::pi() + Real::from(x), Real::e() + Real::from(y))
    }

    /// Translate an integer spatial fixture by the symbolic vector (pi, e, pi).
    fn shifted3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(
            Real::pi() + Real::from(x),
            Real::e() + Real::from(y),
            Real::pi() + Real::from(z),
        )
    }

    /// Check rational contact equality at squared distance 1/9.
    ///
    /// Circle tests compare relation values; line, segment, and plane tests assert the
    /// complete Equal/Exact/Exact outcome.
    #[test]
    fn exact_rational_distance_kernels_preserve_tangent_boundaries() {
        let zero2 = rp2((0, 1), (0, 1));
        let line_a2 = rp2((-1, 1), (1, 3));
        let line_b2 = rp2((1, 1), (1, 3));
        let threshold = r(1, 9);
        assert_eq!(
            exact_rational_circle_line2(&zero2, &threshold, &line_a2, &line_b2)
                .unwrap()
                .value(),
            Some(CircleLineRelation::Tangent)
        );
        assert_eq!(
            exact_rational_circle_segment2(&zero2, &threshold, &line_a2, &line_b2)
                .unwrap()
                .value(),
            Some(CircleSegmentRelation::Tangent)
        );

        let point = rp3((0, 1), (1, 3), (0, 1));
        let a = rp3((-1, 1), (0, 1), (0, 1));
        let b = rp3((1, 1), (0, 1), (0, 1));
        let plane = crate::plane::Plane3::new(rp3((0, 1), (1, 1), (0, 1)), Real::from(0));
        for outcome in [
            exact_rational_point_line3_distance_ordering(&point, &a, &b, &threshold),
            exact_rational_point_segment3_distance_ordering(&point, &a, &b, &threshold),
            exact_rational_point_plane_distance_ordering(&point, &plane, &threshold),
        ] {
            let outcome = outcome.expect("all inputs are exact rationals");
            assert!(matches!(
                outcome,
                PredicateOutcome::Decided {
                    value: Ordering::Equal,
                    certainty: Certainty::Exact,
                    stage: Escalation::Exact,
                }
            ));
        }
    }

    /// Check planar distance ordering for the 3-4-5, doubled, and reflected fixtures.
    ///
    /// Asserts ordering values; absence of square-root construction is established by
    /// implementation inspection, not a dispatch assertion in this test.
    #[test]
    fn squared_distance_comparison_avoids_square_roots() {
        let anchor = p2(0, 0);
        let near = p2(3, 4);
        let far = p2(6, 8);
        let also_near = p2(-3, -4);

        assert_eq!(
            crate::compare_point2_distance_squared(&anchor, &near, &far, APPROX).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            crate::compare_point2_distance_squared(&anchor, &near, &also_near, APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point2_distance_squared(&anchor, &far, &near, APPROX).value(),
            Some(Ordering::Greater)
        );
    }

    /// Check spatial distance ordering for (1,2,2), its double, and its reflection.
    ///
    /// Compares values only; the test does not count square-root dispatches.
    #[test]
    fn squared_distance3_comparison_avoids_square_roots() {
        let anchor = p3(0, 0, 0);
        let near = p3(1, 2, 2);
        let far = p3(2, 4, 4);
        let also_near = p3(-1, -2, -2);

        assert_eq!(
            crate::compare_point3_distance_squared(&anchor, &near, &far, APPROX).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            crate::compare_point3_distance_squared(&anchor, &near, &also_near, APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point3_distance_squared(&anchor, &far, &near, APPROX).value(),
            Some(Ordering::Greater)
        );
    }

    /// Check Inside, On, and Outside values for a ball of squared radius 25.
    #[test]
    fn point_sphere3_classifier_uses_squared_radius() {
        let center = p3(0, 0, 0);
        let radius_squared = hyperreal::Real::from(25);

        assert_eq!(
            crate::classify_point_sphere3(&center, &radius_squared, &p3(1, 2, 2), APPROX).value(),
            Some(SpherePointLocation::Inside)
        );
        assert_eq!(
            crate::classify_point_sphere3(&center, &radius_squared, &p3(3, 4, 0), APPROX).value(),
            Some(SpherePointLocation::On)
        );
        assert_eq!(
            crate::classify_point_sphere3(&center, &radius_squared, &p3(6, 0, 0), APPROX).value(),
            Some(SpherePointLocation::Outside)
        );
    }

    /// Check line-distance squared 25 against thresholds 24, 25, and 26 without asserting dispatch.
    #[test]
    fn point_line3_distance_comparison_is_scaled_without_division() {
        let point = p3(0, 3, 4);
        let a = p3(0, 0, 0);
        let b = p3(2, 0, 0);

        assert_eq!(
            crate::compare_point_line3_distance_squared(&point, &a, &b, &Real::from(24), APPROX)
                .value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            crate::compare_point_line3_distance_squared(&point, &a, &b, &Real::from(25), APPROX)
                .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_line3_distance_squared(&point, &a, &b, &Real::from(26), APPROX)
                .value(),
            Some(Ordering::Less)
        );
    }

    /// Check secant, tangent, disjoint, and coincident-endpoint line controls.
    #[test]
    fn circle_line2_relation_uses_squared_discriminant() {
        let center = p2(0, 0);
        let radius_squared = Real::from(25);

        assert_eq!(
            crate::classify_circle_line2(&center, &radius_squared, &p2(-10, 0), &p2(10, 0), APPROX)
                .value(),
            Some(CircleLineRelation::Secant)
        );
        assert_eq!(
            crate::classify_circle_line2(&center, &radius_squared, &p2(-10, 5), &p2(10, 5), APPROX)
                .value(),
            Some(CircleLineRelation::Tangent)
        );
        assert_eq!(
            crate::classify_circle_line2(&center, &radius_squared, &p2(-10, 6), &p2(10, 6), APPROX)
                .value(),
            Some(CircleLineRelation::Disjoint)
        );
        assert_eq!(
            crate::classify_circle_line2(&center, &radius_squared, &p2(1, 1), &p2(1, 1), APPROX)
                .value(),
            Some(CircleLineRelation::DegenerateLine)
        );
    }

    /// Check finite segment secant, tangent, contained, and disjoint relation values.
    #[test]
    fn circle_segment2_relation_respects_closed_segment_interval() {
        let center = p2(0, 0);
        let radius_squared = Real::from(25);

        assert_eq!(
            crate::classify_circle_segment2(
                &center,
                &radius_squared,
                &p2(-10, 0),
                &p2(10, 0),
                APPROX
            )
            .value(),
            Some(CircleSegmentRelation::Secant)
        );
        assert_eq!(
            crate::classify_circle_segment2(
                &center,
                &radius_squared,
                &p2(-10, 5),
                &p2(10, 5),
                APPROX
            )
            .value(),
            Some(CircleSegmentRelation::Tangent)
        );
        assert_eq!(
            crate::classify_circle_segment2(
                &center,
                &radius_squared,
                &p2(-2, 0),
                &p2(2, 0),
                APPROX
            )
            .value(),
            Some(CircleSegmentRelation::ContainedInside)
        );
        assert_eq!(
            crate::classify_circle_segment2(
                &center,
                &radius_squared,
                &p2(6, 0),
                &p2(10, 0),
                APPROX
            )
            .value(),
            Some(CircleSegmentRelation::Disjoint)
        );
    }

    /// Check spatial segment distances for interior, beyond-b, and before-a projections.
    #[test]
    fn point_segment3_distance_comparison_selects_endpoint_or_interior() {
        let a = p3(0, 0, 0);
        let b = p3(10, 0, 0);

        assert_eq!(
            crate::compare_point_segment3_distance_squared(
                &p3(5, 3, 4),
                &a,
                &b,
                &Real::from(25),
                APPROX
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_segment3_distance_squared(
                &p3(13, 4, 0),
                &a,
                &b,
                &Real::from(25),
                APPROX
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_segment3_distance_squared(
                &p3(-1, 0, 0),
                &a,
                &b,
                &Real::from(0),
                APPROX
            )
            .value(),
            Some(Ordering::Greater)
        );
    }

    /// Check finite triangle face/edge/vertex distances and collinear/point degeneracy.
    ///
    /// Includes symbolically translated face, edge, and collinear controls; compares
    /// values without asserting complete receipts or dispatch paths.
    #[test]
    fn point_triangle3_distance_comparison_covers_face_edges_vertices_and_degeneracy() {
        let a = p3(0, 0, 0);
        let b = p3(4, 0, 0);
        let c = p3(0, 4, 0);

        for (point, threshold, expected) in [
            (p3(1, 1, 3), 8, Ordering::Greater),
            (p3(1, 1, 3), 9, Ordering::Equal),
            (p3(1, 1, 3), 10, Ordering::Less),
            (p3(2, -3, 4), 25, Ordering::Equal),
            (p3(5, 0, 0), 1, Ordering::Equal),
        ] {
            assert_eq!(
                crate::compare_point_triangle3_distance_squared(
                    &point,
                    &a,
                    &b,
                    &c,
                    &Real::from(threshold),
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &shifted3(1, 1, 3),
                &shifted3(0, 0, 0),
                &shifted3(4, 0, 0),
                &shifted3(0, 4, 0),
                &Real::from(9),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &shifted3(2, -3, 4),
                &shifted3(0, 0, 0),
                &shifted3(4, 0, 0),
                &shifted3(0, 4, 0),
                &Real::from(25),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &shifted3(0, 0, 0),
                &shifted3(-10, 0, 0),
                &shifted3(10, 0, 0),
                &shifted3(20, 0, 0),
                &Real::from(0),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );

        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &p3(0, 0, 0),
                &p3(-10, 0, 0),
                &p3(10, 0, 0),
                &p3(20, 0, 0),
                &Real::from(0),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &p3(1, 2, 2),
                &p3(0, 0, 0),
                &p3(0, 0, 0),
                &p3(0, 0, 0),
                &Real::from(9),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    /// Assert Equal/Exact/Exact for a rational face distance of 1/9, plus edge/degenerate values.
    ///
    /// Also checks the rational helper declines a pi coordinate; no dispatch count is asserted.
    #[test]
    fn non_dyadic_point_triangle3_distance_uses_exact_rational_boundary() {
        let a = rp3((0, 1), (0, 1), (1, 3));
        let b = rp3((2, 1), (0, 1), (1, 3));
        let c = rp3((0, 1), (2, 1), (1, 3));
        let point = rp3((1, 3), (1, 3), (0, 1));
        let threshold = r(1, 9);

        assert!(matches!(
            crate::compare_point_triangle3_distance_squared(&point, &a, &b, &c, &threshold, APPROX,),
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                certainty: Certainty::Exact,
                stage: Escalation::Exact,
            }
        ));

        let edge_point = rp3((1, 3), (-1, 3), (0, 1));
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &edge_point,
                &rp3((0, 1), (0, 1), (0, 1)),
                &rp3((2, 1), (0, 1), (0, 1)),
                &rp3((0, 1), (2, 1), (0, 1)),
                &threshold,
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_triangle3_distance_squared(
                &rp3((0, 1), (1, 3), (0, 1)),
                &rp3((-1, 1), (0, 1), (0, 1)),
                &rp3((1, 1), (0, 1), (0, 1)),
                &rp3((2, 1), (0, 1), (0, 1)),
                &threshold,
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            exact_rational_point_triangle3_distance_ordering(
                &Point3::new(Real::pi(), Real::zero(), Real::zero()),
                &a,
                &b,
                &c,
                &threshold,
            ),
            None
        );
    }

    /// Check distance squared 4 to plane 2z-6=0 against thresholds 3, 4, and 5.
    #[test]
    fn point_plane_distance_comparison_uses_unnormalized_normal() {
        let plane = crate::plane::Plane3::new(p3(0, 0, 2), Real::from(-6));
        let point = p3(0, 0, 5);

        assert_eq!(
            crate::compare_point_plane_distance_squared(&point, &plane, &Real::from(3), APPROX)
                .value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            crate::compare_point_plane_distance_squared(&point, &plane, &Real::from(4), APPROX)
                .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_plane_distance_squared(&point, &plane, &Real::from(5), APPROX)
                .value(),
            Some(Ordering::Less)
        );
    }

    /// Check collapsed line/segment point-distance values and the zero-normal plane scalar convention.
    ///
    /// The plane with offset 5 compares as squared value 25; this is not a geometric
    /// distance to its empty zero set. Assertions inspect values, not receipts.
    #[test]
    fn degenerate_distance_carriers_have_explicit_exact_fallbacks() {
        let point = p3(3, 4, 0);
        let anchor = p3(0, 0, 0);
        let zero_normal_plane = crate::plane::Plane3::new(p3(0, 0, 0), Real::from(5));

        assert_eq!(
            crate::compare_point_line3_distance_squared(
                &point,
                &anchor,
                &anchor,
                &Real::from(25),
                APPROX
            )
            .value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_point_segment3_distance_squared(
                &point,
                &anchor,
                &anchor,
                &Real::from(24),
                APPROX
            )
            .value(),
            Some(Ordering::Greater)
        );
        assert_eq!(
            crate::compare_point_plane_distance_squared(
                &point,
                &zero_normal_plane,
                &Real::from(25),
                APPROX
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    /// Check closed-ball disjoint, external-contact, and overlap values, plus no value for a negative radius.
    #[test]
    fn sphere3_intersection_uses_radius_sum_squared() {
        let first = p3(0, 0, 0);
        let second = p3(3, 4, 0);

        assert_eq!(
            crate::classify_sphere3_intersection(
                &first,
                &Real::from(2),
                &second,
                &Real::from(2),
                APPROX
            )
            .value(),
            Some(SphereIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_sphere3_intersection(
                &first,
                &Real::from(2),
                &second,
                &Real::from(3),
                APPROX
            )
            .value(),
            Some(SphereIntersection::Touching)
        );
        assert_eq!(
            crate::classify_sphere3_intersection(
                &first,
                &Real::from(4),
                &second,
                &Real::from(3),
                APPROX
            )
            .value(),
            Some(SphereIntersection::Overlapping)
        );
        assert!(
            crate::classify_sphere3_intersection(
                &first,
                &Real::from(-1),
                &second,
                &Real::from(3),
                APPROX
            )
            .value()
            .is_none()
        );
    }

    /// Check finite box/ball disjoint, equality-contact, and interior-center overlap values.
    #[test]
    fn aabb3_sphere_intersection_uses_closest_box_point_distance() {
        let min = p3(0, 0, 0);
        let max = p3(2, 2, 2);

        assert_eq!(
            crate::classify_aabb3_sphere_intersection(
                &min,
                &max,
                &p3(5, 2, 2),
                &Real::from(4),
                APPROX
            )
            .value(),
            Some(AabbSphereIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_aabb3_sphere_intersection(
                &min,
                &max,
                &p3(5, 2, 2),
                &Real::from(9),
                APPROX
            )
            .value(),
            Some(AabbSphereIntersection::Touching)
        );
        assert_eq!(
            crate::classify_aabb3_sphere_intersection(
                &min,
                &max,
                &p3(1, 1, 1),
                &Real::from(1),
                APPROX
            )
            .value(),
            Some(AabbSphereIntersection::Overlapping)
        );
    }

    /// Check translated distance/contact fixtures using shared pi/e coordinate shifts.
    ///
    /// Includes the current endpoint Tangent/Secant partition and reversed box corners.
    /// Tests expected values, not selected dispatches or complete certainty receipts.
    #[test]
    fn symbolic_translations_exercise_generic_distance_cascades() {
        let anchor2 = shifted2(0, 0);
        assert_eq!(
            crate::compare_point2_distance_squared(
                &anchor2,
                &shifted2(1, 0),
                &shifted2(2, 0),
                APPROX,
            )
            .value(),
            Some(Ordering::Less)
        );

        let anchor3 = shifted3(0, 0, 0);
        assert_eq!(
            crate::compare_point3_distance_squared(
                &anchor3,
                &shifted3(1, 2, 2),
                &shifted3(2, 4, 4),
                APPROX,
            )
            .value(),
            Some(Ordering::Less)
        );

        for (height, expected) in [
            (0, CircleLineRelation::Secant),
            (5, CircleLineRelation::Tangent),
            (6, CircleLineRelation::Disjoint),
        ] {
            assert_eq!(
                crate::classify_circle_line2(
                    &anchor2,
                    &Real::from(25),
                    &shifted2(-10, height),
                    &shifted2(10, height),
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }

        for (a, b, expected) in [
            ((0, 0), (0, 0), CircleSegmentRelation::ContainedInside),
            ((5, 0), (5, 0), CircleSegmentRelation::Tangent),
            ((6, 0), (6, 0), CircleSegmentRelation::Disjoint),
            ((-5, 0), (5, 0), CircleSegmentRelation::Secant),
            ((5, 0), (0, 0), CircleSegmentRelation::Tangent),
            ((5, 0), (10, 0), CircleSegmentRelation::Secant),
            ((0, 0), (10, 0), CircleSegmentRelation::Secant),
            ((-2, 0), (2, 0), CircleSegmentRelation::ContainedInside),
            ((-10, 5), (10, 5), CircleSegmentRelation::Tangent),
            ((6, 0), (10, 0), CircleSegmentRelation::Disjoint),
        ] {
            assert_eq!(
                crate::classify_circle_segment2(
                    &anchor2,
                    &Real::from(25),
                    &shifted2(a.0, a.1),
                    &shifted2(b.0, b.1),
                    APPROX,
                )
                .value(),
                Some(expected),
                "segment case {a:?}..{b:?}"
            );
        }

        let line_a = shifted3(0, 0, 0);
        let line_b = shifted3(10, 0, 0);
        assert_eq!(
            crate::compare_point_line3_distance_squared(
                &shifted3(5, 3, 4),
                &line_a,
                &line_b,
                &Real::from(25),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
        for (point, threshold, expected) in [
            (shifted3(-1, 0, 0), 0, Ordering::Greater),
            (shifted3(5, 3, 4), 25, Ordering::Equal),
            (shifted3(13, 4, 0), 25, Ordering::Equal),
        ] {
            assert_eq!(
                crate::compare_point_segment3_distance_squared(
                    &point,
                    &line_a,
                    &line_b,
                    &Real::from(threshold),
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }

        let plane = crate::plane::Plane3::new(p3(0, 0, 2), Real::from(-2) * Real::pi());
        assert_eq!(
            crate::compare_point_plane_distance_squared(
                &shifted3(0, 0, 2),
                &plane,
                &Real::from(4),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );

        assert_eq!(
            crate::classify_point_sphere3(&anchor3, &Real::from(25), &shifted3(3, 4, 0), APPROX,)
                .value(),
            Some(SpherePointLocation::On)
        );
        assert_eq!(
            crate::classify_sphere3_intersection(
                &anchor3,
                &Real::from(2),
                &shifted3(3, 4, 0),
                &Real::from(3),
                APPROX,
            )
            .value(),
            Some(SphereIntersection::Touching)
        );

        assert_eq!(
            crate::classify_aabb3_sphere_intersection(
                &shifted3(2, 2, 2),
                &shifted3(0, 0, 0),
                &shifted3(-1, -2, -2),
                &Real::from(9),
                APPROX,
            )
            .value(),
            Some(AabbSphereIntersection::Touching)
        );
    }

    /// Check finite circle and spatial-distance relation tables with squared threshold 1/9.
    ///
    /// Covers named relations and orderings, not every endpoint partition or geometric
    /// configuration; assertions compare values without dispatch counts.
    #[test]
    fn non_dyadic_rational_distance_kernels_cover_every_relation() {
        let center = rp2((0, 1), (0, 1));
        let radius_squared = r(1, 9);
        for (height, expected) in [
            ((0, 1), CircleLineRelation::Secant),
            ((1, 3), CircleLineRelation::Tangent),
            ((2, 3), CircleLineRelation::Disjoint),
        ] {
            assert_eq!(
                crate::classify_circle_line2(
                    &center,
                    &radius_squared,
                    &rp2((-1, 1), height),
                    &rp2((1, 1), height),
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }
        assert_eq!(
            crate::classify_circle_line2(&center, &radius_squared, &center, &center, APPROX,)
                .value(),
            Some(CircleLineRelation::DegenerateLine)
        );

        for (a, b, expected) in [
            (
                rp2((0, 1), (0, 1)),
                rp2((0, 1), (0, 1)),
                CircleSegmentRelation::ContainedInside,
            ),
            (
                rp2((1, 3), (0, 1)),
                rp2((1, 3), (0, 1)),
                CircleSegmentRelation::Tangent,
            ),
            (
                rp2((1, 1), (0, 1)),
                rp2((1, 1), (0, 1)),
                CircleSegmentRelation::Disjoint,
            ),
            (
                rp2((-1, 1), (0, 1)),
                rp2((1, 1), (0, 1)),
                CircleSegmentRelation::Secant,
            ),
            (
                rp2((-1, 6), (0, 1)),
                rp2((1, 6), (0, 1)),
                CircleSegmentRelation::ContainedInside,
            ),
            (
                rp2((1, 1), (0, 1)),
                rp2((2, 1), (0, 1)),
                CircleSegmentRelation::Disjoint,
            ),
        ] {
            assert_eq!(
                crate::classify_circle_segment2(&center, &radius_squared, &a, &b, APPROX).value(),
                Some(expected)
            );
        }

        let line_a = rp3((-1, 1), (0, 1), (0, 1));
        let line_b = rp3((1, 1), (0, 1), (0, 1));
        let plane = crate::plane::Plane3::new(p3(0, 1, 0), Real::from(0));
        for (height, expected) in [
            ((0, 1), Ordering::Less),
            ((1, 3), Ordering::Equal),
            ((1, 1), Ordering::Greater),
        ] {
            let point = rp3((0, 1), height, (0, 1));
            assert_eq!(
                crate::compare_point_line3_distance_squared(
                    &point,
                    &line_a,
                    &line_b,
                    &radius_squared,
                    APPROX,
                )
                .value(),
                Some(expected)
            );
            assert_eq!(
                crate::compare_point_segment3_distance_squared(
                    &point,
                    &line_a,
                    &line_b,
                    &radius_squared,
                    APPROX,
                )
                .value(),
                Some(expected)
            );
            assert_eq!(
                crate::compare_point_plane_distance_squared(
                    &point,
                    &plane,
                    &radius_squared,
                    APPROX,
                )
                .value(),
                Some(expected)
            );
        }
    }

    /// Check rational point thresholds in 2D/3D and None for a pi coordinate.
    #[test]
    fn exact_rational_threshold_helpers_cover_two_and_three_dimensions() {
        let origin2 = rp2((0, 1), (0, 1));
        let origin3 = rp3((0, 1), (0, 1), (0, 1));
        let threshold = r(1, 9);

        for (point2, point3, expected) in [
            (
                rp2((0, 1), (0, 1)),
                rp3((0, 1), (0, 1), (0, 1)),
                Ordering::Less,
            ),
            (
                rp2((1, 3), (0, 1)),
                rp3((1, 3), (0, 1), (0, 1)),
                Ordering::Equal,
            ),
            (
                rp2((1, 1), (0, 1)),
                rp3((1, 1), (0, 1), (0, 1)),
                Ordering::Greater,
            ),
        ] {
            assert_eq!(
                exact_rational_point2_distance_threshold_ordering(&point2, &origin2, &threshold,)
                    .and_then(PredicateOutcome::value),
                Some(expected)
            );
            assert_eq!(
                exact_rational_point3_distance_threshold_ordering(&point3, &origin3, &threshold,)
                    .and_then(PredicateOutcome::value),
                Some(expected)
            );
            assert_eq!(
                exact_rational_point3_distance_threshold(&point3, &origin3, &threshold),
                Some(expected)
            );
        }

        assert_eq!(
            exact_rational_point2_distance_threshold_ordering(
                &Point2::new(Real::pi(), Real::zero()),
                &origin2,
                &threshold,
            ),
            None
        );
        assert_eq!(
            exact_rational_point3_distance_threshold_ordering(
                &Point3::new(Real::pi(), Real::zero(), Real::zero()),
                &origin3,
                &threshold,
            ),
            None
        );
    }

    /// Check selected unresolved source/radius inputs remain Unknown under STRICT.
    ///
    /// Also checks a negative second radius under APPROX yields Unknown; does not assert
    /// specific unresolved reasons or stages.
    #[test]
    fn strict_distance_predicates_preserve_unresolved_input() {
        let zero = terminal_zero();
        let uncertain2 = Point2::new(zero.clone(), Real::from(0));
        let uncertain3 = Point3::new(zero.clone(), Real::from(0), Real::from(0));

        assert!(matches!(
            crate::classify_circle_line2(
                &p2(0, 0),
                &Real::from(1),
                &p2(0, 0),
                &uncertain2,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_circle_segment2(
                &p2(0, 0),
                &Real::from(1),
                &p2(0, 0),
                &uncertain2,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::compare_point_segment3_distance_squared(
                &p3(0, 0, 0),
                &p3(0, 0, 0),
                &uncertain3,
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::compare_point_triangle3_distance_squared(
                &p3(0, 1, 0),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &Point3::new(Real::from(0), zero.clone(), Real::from(0)),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_point_sphere3(
                &p3(0, 0, 0),
                &zero,
                &p3(0, 0, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_sphere3_intersection(
                &p3(0, 0, 0),
                &zero,
                &p3(1, 0, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_sphere3_intersection(
                &p3(0, 0, 0),
                &Real::from(1),
                &p3(1, 0, 0),
                &Real::from(-1),
                APPROX,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            crate::classify_aabb3_sphere_intersection(
                &p3(0, 0, 0),
                &uncertain3,
                &p3(0, 0, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check selected strict norm, projection, threshold, and radius comparison failures.
    ///
    /// Finite fixtures assert Unknown without constraining its reason/stage; the test
    /// name does not establish exhaustive coverage of all comparison paths.
    #[test]
    fn strict_distance_subpredicates_propagate_each_unresolved_comparison() {
        let unresolved = terminal_zero();
        let unresolved_line_end = Point2::new(unresolved.clone(), Real::from(0));
        assert!(matches!(
            classify_circle_line2_with_policy(
                &p2(0, 0),
                &Real::from(1),
                &p2(0, 0),
                &unresolved_line_end,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_circle_line2_with_policy(
                &Point2::new(Real::from(0), unresolved.clone()),
                &Real::from(0),
                &p2(0, 0),
                &p2(1, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            classify_circle_segment2_with_policy(
                &p2(0, 0),
                &unresolved,
                &p2(0, 0),
                &p2(0, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_circle_segment2_with_policy(
                &Point2::new(Real::from(0), unresolved.clone()),
                &Real::from(0),
                &p2(-2, 0),
                &p2(2, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let unresolved_line_end3 = Point3::new(unresolved.clone(), 0.into(), 0.into());
        assert!(matches!(
            compare_point_line3_distance_squared_with_policy(
                &p3(0, 1, 0),
                &p3(0, 0, 0),
                &unresolved_line_end3,
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            compare_point_segment3_distance_squared_with_policy(
                &Point3::new(unresolved.clone(), 1.into(), 0.into()),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            compare_point_segment3_distance_squared_with_policy(
                &Point3::new(&Real::from(1) + &unresolved, 1.into(), 0.into()),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let uncertain_plane = crate::plane::Plane3::new(
            Point3::new(unresolved.clone(), 0.into(), 0.into()),
            0.into(),
        );
        assert!(matches!(
            compare_point_plane_distance_squared_with_policy(
                &p3(0, 0, 0),
                &uncertain_plane,
                &Real::from(0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_sphere3_intersection_with_policy(
                &p3(0, 0, 0),
                &Real::from(0),
                &Point3::new(unresolved.clone(), 0.into(), 0.into()),
                &Real::from(0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_aabb3_sphere_intersection_with_policy(
                &p3(0, 0, 0),
                &p3(1, 1, 1),
                &p3(0, 0, 0),
                &unresolved,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check all three axes and three interval branches across STRICT/APPROX/STRICT calls.
    ///
    /// Rebuilds inputs for each policy, so this is not a retained-object cache transition
    /// test. Requires strict Unknown and approximate Overlapping/Approximate/Refined.
    /// With dispatch-trace, checks 10 comparisons for completed approximate queries
    /// and the expected strict prefix count at the first unresolved decision.
    #[test]
    fn aabb3_sphere_preserves_approximate_axis_decisions() {
        let mut failures = Vec::new();
        for axis in 0..3 {
            for branch in ["lower", "upper", "endpoints"] {
                for policy in [PredicatePolicy::STRICT, APPROX, PredicatePolicy::STRICT] {
                    let zero = terminal_zero();
                    let mut min = [Real::zero(), Real::zero(), Real::zero()];
                    let mut max = [Real::one(), Real::one(), Real::one()];
                    let mut center = [Real::zero(), Real::zero(), Real::zero()];
                    match branch {
                        "lower" => center[axis] = zero,
                        "upper" => center[axis] = Real::one() + zero,
                        "endpoints" => {
                            min[axis] = zero;
                            max[axis] = Real::zero();
                            center[axis] = Real::one();
                        }
                        _ => unreachable!(),
                    }
                    let [x, y, z] = min;
                    let min = Point3::new(x, y, z);
                    let [x, y, z] = max;
                    let max = Point3::new(x, y, z);
                    let [x, y, z] = center;
                    let center = Point3::new(x, y, z);
                    #[cfg(feature = "dispatch-trace")]
                    let _recording = {
                        hyperreal::dispatch_trace::reset();
                        hyperreal::dispatch_trace::recording_scope()
                    };
                    let outcome = crate::classify_aabb3_sphere_intersection(
                        &min,
                        &max,
                        &center,
                        &4.into(),
                        policy,
                    );
                    #[cfg(feature = "dispatch-trace")]
                    {
                        let trace = hyperreal::dispatch_trace::snapshot_trace();
                        let comparisons: u64 = trace
                            .dispatch
                            .iter()
                            .filter(|row| {
                                row.layer == "hyperlimit" && row.operation == "compare_reals"
                            })
                            .map(|row| row.count)
                            .sum();
                        // Three ordered decisions per completed axis; STRICT
                        // stops at the first unresolved comparison.
                        let expected = if policy == APPROX {
                            10
                        } else {
                            3 * axis as u64
                                + match branch {
                                    "lower" => 2,
                                    "upper" => 3,
                                    "endpoints" => 1,
                                    _ => unreachable!(),
                                }
                        };
                        assert_eq!(comparisons, expected);
                    }
                    let correct = if policy == PredicatePolicy::STRICT {
                        matches!(outcome, PredicateOutcome::Unknown { .. })
                    } else {
                        matches!(
                            outcome,
                            PredicateOutcome::Decided {
                                value: AabbSphereIntersection::Overlapping,
                                certainty: Certainty::Approximate,
                                stage: Escalation::Refined,
                            }
                        )
                    };
                    if !correct {
                        failures.push(format!("{axis} {branch} {policy:?}: {outcome:?}"));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    /// Check 216 integer box/ball cases against independent interval-distance arithmetic.
    ///
    /// Varies both policies, corner order, axis, center coordinate, and squared radius;
    /// asserts each complete Exact/Exact relation, including zero-radius equality.
    #[test]
    fn aabb3_sphere_rational_relations_and_receipts_survive_axis_ordering() {
        for policy in [PredicatePolicy::STRICT, APPROX] {
            for reversed in [false, true] {
                let (min, max) = if reversed {
                    (p3(1, 1, 1), p3(0, 0, 0))
                } else {
                    (p3(0, 0, 0), p3(1, 1, 1))
                };
                for axis in 0..3 {
                    for coordinate in -2_i32..=3 {
                        let mut coordinates = [0; 3];
                        coordinates[axis] = coordinate;
                        let center = p3(coordinates[0], coordinates[1], coordinates[2]);
                        // Integer distance to [0,1], independent of Real predicates.
                        let distance = if coordinate < 0 {
                            -coordinate
                        } else if coordinate > 1 {
                            coordinate - 1
                        } else {
                            0
                        };
                        for radius_squared in [0, 1, 4] {
                            let expected = match (distance * distance).cmp(&radius_squared) {
                                Ordering::Less => AabbSphereIntersection::Overlapping,
                                Ordering::Equal => AabbSphereIntersection::Touching,
                                Ordering::Greater => AabbSphereIntersection::Disjoint,
                            };
                            assert_eq!(
                                crate::classify_aabb3_sphere_intersection(
                                    &min,
                                    &max,
                                    &center,
                                    &radius_squared.into(),
                                    policy,
                                ),
                                PredicateOutcome::decided(
                                    expected,
                                    Certainty::Exact,
                                    Escalation::Exact
                                )
                            );
                        }
                    }
                }
            }
        }
    }

    /// Check interval lower/upper Unknown and selected planar segment endpoint/projection paths.
    ///
    /// The collapsed segment assertion only requires a decided result; the beyond-b
    /// fixture asserts Equal, while unresolved projections assert Unknown.
    #[test]
    fn interval_distance_helper_propagates_lower_and_upper_order_uncertainty() {
        assert!(matches!(
            outside_interval_delta(
                &terminal_zero(),
                &Real::from(0),
                &Real::from(1),
                PredicatePolicy::STRICT,
                &mut (Certainty::Exact, Escalation::Structural),
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));
        assert!(matches!(
            outside_interval_delta(
                &(&Real::from(1) + &terminal_zero()),
                &Real::from(0),
                &Real::from(1),
                PredicatePolicy::STRICT,
                &mut (Certainty::Exact, Escalation::Structural),
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        assert!(matches!(
            compare_point_segment2_distance_squared_with_policy(
                &p2(1, 1),
                &p2(0, 0),
                &p2(0, 0),
                &Real::from(1),
                APPROX,
            ),
            PredicateOutcome::Decided { .. }
        ));
        assert!(matches!(
            compare_point_segment2_distance_squared_with_policy(
                &Point2::new(terminal_zero(), Real::from(1)),
                &p2(0, 0),
                &p2(1, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            compare_point_segment2_distance_squared_with_policy(
                &Point2::new(&Real::from(1) + &terminal_zero(), Real::from(1)),
                &p2(0, 0),
                &p2(1, 0),
                &Real::from(1),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            compare_point_segment2_distance_squared_with_policy(
                &p2(2, 0),
                &p2(0, 0),
                &p2(1, 0),
                &Real::from(1),
                APPROX,
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    /// Check seven rational segment cases, including endpoint-label conventions and outside projection intervals.
    #[test]
    fn exact_circle_segment_kernel_covers_endpoint_and_interval_partitions() {
        let center = p2(0, 0);
        let radius = Real::from(1);
        let relation = |a: Point2, b: Point2| {
            exact_rational_circle_segment2(&center, &radius, &a, &b)
                .and_then(PredicateOutcome::value)
                .expect("rational circle/segment query should decide")
        };

        assert_eq!(relation(p2(-1, 0), p2(1, 0)), CircleSegmentRelation::Secant);
        assert_eq!(relation(p2(1, 0), p2(0, 0)), CircleSegmentRelation::Tangent);
        assert_eq!(relation(p2(1, 0), p2(2, 0)), CircleSegmentRelation::Secant);
        assert_eq!(relation(p2(0, 0), p2(1, 0)), CircleSegmentRelation::Tangent);
        assert_eq!(relation(p2(0, 0), p2(2, 0)), CircleSegmentRelation::Secant);
        assert_eq!(
            relation(p2(2, 1), p2(3, 1)),
            CircleSegmentRelation::Disjoint
        );
        assert_eq!(
            relation(p2(-3, 1), p2(-2, 1)),
            CircleSegmentRelation::Disjoint
        );
    }

    /// Check rational-helper refusal of pi and collapsed/endpoint/zero-normal scalar fallback values.
    #[test]
    fn exact_distance_helpers_cover_nonrational_and_degenerate_endpoint_fallbacks() {
        let symbolic2 = Point2::new(Real::pi(), Real::from(0));
        assert!(
            exact_rational_point_line3_distance_ordering(
                &Point3::new(Real::pi(), 0.into(), 0.into()),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &Real::from(0),
            )
            .is_none()
        );
        assert!(
            exact_rational_point_segment3_distance_ordering(
                &Point3::new(Real::pi(), 0.into(), 0.into()),
                &p3(0, 0, 0),
                &p3(1, 0, 0),
                &Real::from(0),
            )
            .is_none()
        );
        assert!(
            exact_rational_point_plane_distance_ordering(
                &Point3::new(Real::pi(), 0.into(), 0.into()),
                &crate::plane::Plane3::new(p3(1, 0, 0), 0.into()),
                &Real::from(0),
            )
            .is_none()
        );
        assert!(symbolic2.x.exact_rational_ref().is_none());

        for expected in [Ordering::Less, Ordering::Equal, Ordering::Greater] {
            let threshold = match expected {
                Ordering::Less => Real::from(2),
                Ordering::Equal => Real::from(1),
                Ordering::Greater => Real::from(0),
            };
            assert_eq!(
                exact_rational_point_line3_distance_ordering(
                    &p3(1, 0, 0),
                    &p3(0, 0, 0),
                    &p3(0, 0, 0),
                    &threshold,
                )
                .and_then(PredicateOutcome::value),
                Some(expected)
            );
            assert_eq!(
                exact_rational_point_segment3_distance_ordering(
                    &p3(1, 0, 0),
                    &p3(0, 0, 0),
                    &p3(0, 0, 0),
                    &threshold,
                )
                .and_then(PredicateOutcome::value),
                Some(expected)
            );
        }

        assert_eq!(
            exact_rational_point_segment3_distance_ordering(
                &p3(-1, 0, 0),
                &p3(0, 0, 0),
                &p3(2, 0, 0),
                &Real::from(1),
            )
            .and_then(PredicateOutcome::value),
            Some(Ordering::Equal)
        );
        assert_eq!(
            exact_rational_point_segment3_distance_ordering(
                &p3(3, 0, 0),
                &p3(0, 0, 0),
                &p3(2, 0, 0),
                &Real::from(1),
            )
            .and_then(PredicateOutcome::value),
            Some(Ordering::Equal)
        );
        assert_eq!(
            exact_rational_point_plane_distance_ordering(
                &p3(0, 0, 0),
                &crate::plane::Plane3::new(p3(0, 0, 0), Real::from(1)),
                &Real::from(1),
            )
            .and_then(PredicateOutcome::value),
            Some(Ordering::Equal)
        );
    }

    /// Check selected certainty/stage maxima and every stage rank, not every pair of ranks.
    #[test]
    fn distance_trace_ranking_covers_all_certainties_and_stages() {
        assert_eq!(
            max_distance_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(
            max_distance_certainty(Certainty::Filtered, Certainty::Approximate),
            Certainty::Approximate
        );
        assert_eq!(
            max_distance_stage(Escalation::Filter, Escalation::Exact),
            Escalation::Exact
        );
        assert_eq!(distance_stage_rank(Escalation::Structural), 0);
        assert_eq!(distance_stage_rank(Escalation::Filter), 1);
        assert_eq!(distance_stage_rank(Escalation::Exact), 2);
        assert_eq!(distance_stage_rank(Escalation::Refined), 3);
        assert_eq!(distance_stage_rank(Escalation::Undecided), 4);
    }

    /// Check nondyadic point thresholds, a second-endpoint boundary label, and selected strict Unknown paths.
    #[test]
    fn generic_distance_threshold_and_late_unknown_paths_are_covered() {
        let third = r(1, 3);
        assert_eq!(
            compare_point2_distance_squared_to_threshold_with_policy(
                &p2(0, 0),
                &p2(0, 0),
                &third,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point3_distance_squared_to_threshold_with_policy(
                &p3(0, 0, 0),
                &p3(0, 0, 0),
                &third,
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Less)
        );

        // Put the boundary endpoint second so the generic endpoint partition
        // exercises both halves of its symmetric selection.
        assert_eq!(
            classify_circle_segment2_with_policy(
                &p2(0, 0),
                &Real::from(1),
                &p2(0, 0),
                &p2(1, 0),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(CircleSegmentRelation::Tangent)
        );

        let unresolved = terminal_zero();
        assert!(matches!(
            classify_circle_segment2_with_policy(
                &Point2::new(unresolved.clone(), Real::zero()),
                &Real::zero(),
                &p2(0, 0),
                &p2(1, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_circle_segment2_with_policy(
                &p2(0, 0),
                &Real::zero(),
                &p2(1, 0),
                &Point2::new(unresolved.clone(), Real::zero()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            compare_point_segment2_distance_squared_with_policy(
                &p2(0, 1),
                &p2(0, 0),
                &Point2::new(unresolved, Real::zero()),
                &Real::one(),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }
}
