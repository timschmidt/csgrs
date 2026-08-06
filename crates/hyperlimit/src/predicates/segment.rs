//! Segment incidence and intersection classifiers.
//!
//! The algorithms here use only orientation signs and exact interval tests.
//! This keeps segment topology in `hyperlimit` while leaving segment storage,
//! DCELs, rings, and sweep state to higher crates such as `hypercurve` and
//! `hypertri`.

use crate::classify::{PointSegmentLocation, Segment3Intersection, SegmentIntersection};
use crate::geometry::{Point2, Point3, Segment2Facts};
use crate::predicate::PredicatePolicy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::{
    Line2Orientation, orient2d_with_orientation_and_policy, orient2d_with_policy,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign_direct};
use core::cmp::Ordering;
use hyperreal::Real;

/// Classify `point` relative to the closed 3D segment `ab` with an explicit
/// predicate escalation policy.
///
/// Collinearity is certified by the three exact components of
/// `(b - a) x (point - a)`. Interval containment then uses exact coordinate
/// comparisons on all three axes.
pub fn classify_point_segment3_with_policy(
    a: &Point3,
    b: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();

    match points_equal3(a, b, policy, &mut trace) {
        Ok(true) => {
            return match classify_degenerate_point_segment3(a, point, policy, &mut trace) {
                Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
                Err(unknown) => unknown.into_outcome(),
            };
        }
        Ok(false) => {}
        Err(unknown) => return unknown.into_outcome(),
    }

    match point_segment3_cross_signs(a, b, point, policy, &mut trace) {
        Ok([Sign::Zero, Sign::Zero, Sign::Zero]) => {}
        Ok(_) => {
            return PredicateOutcome::decided(
                PointSegmentLocation::OffLine,
                trace.certainty,
                trace.stage,
            );
        }
        Err(unknown) => return unknown.into_outcome(),
    }

    match classify_collinear_point_segment3(a, b, point, policy, &mut trace) {
        Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Classify `point` relative to the closed segment `ab` with an explicit
/// predicate escalation policy.
pub fn classify_point_segment_with_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None, None)
}

/// Classify `point` relative to closed segment `ab` while reusing exact
/// orientation evidence for the same ordered endpoints.
///
/// The retained evidence only schedules certified determinant filters. An
/// inconclusive filter enters the same complete exact/refinement cascade as
/// [`classify_point_segment_with_policy`], and interval containment remains a
/// policy-aware exact decision.
pub fn classify_point_segment_with_orientation_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None, Some(orientation))
}

/// Classify a point already certified collinear with the segment endpoints.
///
/// This is an internal composition helper for predicates that already retain
/// an `orient2d(a, b, point) == 0` certificate. It avoids rebuilding the same
/// determinant while preserving the full endpoint/interior/outside report.
pub(crate) fn classify_collinear_point_segment_with_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();
    match classify_collinear_point_segment(a, b, point, policy, &mut trace) {
        Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Classify `point` relative to the closed segment `ab` with both an explicit
/// policy and cached segment structural facts.
///
/// The facts are advisory exact metadata. They can skip the orientation
/// determinant for a structurally degenerate segment, but the point equality
/// decision still goes through exact Real predicates while retaining reusable
/// object facts for degeneracy-aware algorithms.
pub fn classify_point_segment_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    segment_facts: Segment2Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, Some(segment_facts), None)
}

fn classify_point_segment_impl(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    segment_facts: Option<Segment2Facts>,
    orientation: Option<&Line2Orientation>,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();

    if segment_facts.and_then(Segment2Facts::known_degenerate) == Some(true) {
        return match classify_degenerate_point_segment(a, point, policy, &mut trace) {
            Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
            Err(unknown) => unknown.into_outcome(),
        };
    }

    let orientation_outcome = match orientation {
        Some(orientation) => orient2d_with_orientation_and_policy(a, b, point, orientation, policy),
        None => orient2d_with_policy(a, b, point, policy),
    };
    let orientation = match decided(orientation_outcome, &mut trace) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };

    if orientation != Sign::Zero {
        return PredicateOutcome::decided(
            PointSegmentLocation::OffLine,
            trace.certainty,
            trace.stage,
        );
    }

    match classify_collinear_point_segment(a, b, point, policy, &mut trace) {
        Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Return whether `point` lies on the closed 3D segment `ab` with an explicit
/// predicate escalation policy.
pub fn point_on_segment3_with_policy(
    a: &Point3,
    b: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_segment3_with_policy(a, b, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether `point` lies on the closed segment `ab` with an explicit
/// predicate escalation policy.
pub fn point_on_segment_with_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_segment_with_policy(a, b, point, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether `point` lies on closed segment `ab` while reusing exact
/// orientation evidence for the same ordered endpoints.
pub fn point_on_segment_with_orientation_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_segment_with_orientation_and_policy(a, b, point, orientation, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Return whether `point` lies on the closed segment `ab` with both an explicit
/// policy and cached segment structural facts.
pub fn point_on_segment_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    segment_facts: Segment2Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_segment_with_policy_and_facts(a, b, point, segment_facts, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Construct the exact intersection point of two supporting 2D lines.
///
/// The construction uses the standard line parameter
/// `t = cross(c - a, d - c) / cross(b - a, d - c)` and returns
/// `a + t (b - a)`. It does not classify the closed segments: callers that
/// require segment topology must first use [`classify_segment_intersection_with_policy`]
/// and invoke this constructor only for a certified proper crossing. `None`
/// means the lines are parallel or the exact scalar backend could not form the
/// required quotient.
pub fn construct_line_intersection_point(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Option<Point2> {
    if [&a.x, &a.y, &b.x, &b.y, &c.x, &c.y, &d.x, &d.y]
        .into_iter()
        .all(Real::is_exact_dyadic_rational)
    {
        let first_start = [&a.x, &a.y];
        let first_end = [&b.x, &b.y];
        let second_start = [&c.x, &c.y];
        let second_end = [&d.x, &d.y];
        if let Some((_, [x, y])) = Real::exact_rational_line_intersection2_point_known_dyadic(
            first_start,
            first_end,
            second_start,
            second_end,
        ) {
            crate::trace_dispatch!(
                "hyperlimit",
                "line-intersection-point",
                "exact-dyadic-stack"
            );
            return Some(Point2::new(x, y));
        }
        if let Some((_, [x, y])) = Real::exact_rational_line_intersection2_point_known_dyadic_wide(
            first_start,
            first_end,
            second_start,
            second_end,
        ) {
            crate::trace_dispatch!("hyperlimit", "line-intersection-point", "exact-dyadic-wide");
            return Some(Point2::new(x, y));
        }
        crate::trace_dispatch!(
            "hyperlimit",
            "line-intersection-point",
            "exact-dyadic-declined"
        );
    }
    if let Some([x, y]) = Real::exact_rational_line_intersection2_point_known_exact(
        [&a.x, &a.y],
        [&b.x, &b.y],
        [&c.x, &c.y],
        [&d.x, &d.y],
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "line-intersection-point",
            "exact-rational-fused"
        );
        return Some(Point2::new(x, y));
    }
    crate::trace_dispatch!("hyperlimit", "line-intersection-point", "general-real");

    let ab_x = sub_ref(&b.x, &a.x);
    let ab_y = sub_ref(&b.y, &a.y);
    let cd_x = sub_ref(&d.x, &c.x);
    let cd_y = sub_ref(&d.y, &c.y);
    let ca_x = sub_ref(&c.x, &a.x);
    let ca_y = sub_ref(&c.y, &a.y);

    let denominator = cross2(&ab_x, &ab_y, &cd_x, &cd_y);
    let numerator = cross2(&ca_x, &ca_y, &cd_x, &cd_y);
    let t = (&numerator / &denominator).ok()?;

    Some(Point2::new(
        add_ref(&a.x, &mul_ref(&t, &ab_x)),
        add_ref(&a.y, &mul_ref(&t, &ab_y)),
    ))
}

/// Classify the intersection of closed 3D segments `ab` and `cd` with an
/// explicit predicate escalation policy.
///
/// The nonparallel branch uses the standard exact line-parameter identities
/// `t = ((c-a) x (d-c))_k / ((b-a) x (d-c))_k` and
/// `u = ((c-a) x (b-a))_k / ((b-a) x (d-c))_k` on a certified nonzero
/// component `k`. It compares the rational parameters to `[0, 1]` without
/// division. The parallel branch reduces to exact collinear point/segment
/// tests. Exact signs decide the combinatorial relation, with explicit
/// skew/coplanar separation in 3D.
pub fn classify_segment3_intersection_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Segment3Intersection> {
    let mut trace = DecisionTrace::default();

    let first_degenerate = match points_equal3(a, b, policy, &mut trace) {
        Ok(value) => value,
        Err(unknown) => return unknown.into_outcome(),
    };
    let second_degenerate = match points_equal3(c, d, policy, &mut trace) {
        Ok(value) => value,
        Err(unknown) => return unknown.into_outcome(),
    };

    if first_degenerate && second_degenerate {
        return match points_equal3(a, c, policy, &mut trace) {
            Ok(true) => PredicateOutcome::decided(
                Segment3Intersection::Identical,
                trace.certainty,
                trace.stage,
            ),
            Ok(false) => PredicateOutcome::decided(
                Segment3Intersection::CoplanarDisjoint,
                trace.certainty,
                trace.stage,
            ),
            Err(unknown) => unknown.into_outcome(),
        };
    }
    if first_degenerate {
        return point_segment3_intersection_from_classifier(classify_point_segment3_with_policy(
            c, d, a, policy,
        ));
    }
    if second_degenerate {
        return point_segment3_intersection_from_classifier(classify_point_segment3_with_policy(
            a, b, c, policy,
        ));
    }

    let ab = vector3_between(a, b);
    let cd = vector3_between(c, d);
    let ac = vector3_between(a, c);
    let normal = cross3(&ab, &cd);
    let normal_signs = match signs3(&normal, policy, &mut trace) {
        Ok(signs) => signs,
        Err(unknown) => return unknown.into_outcome(),
    };

    if normal_signs == [Sign::Zero, Sign::Zero, Sign::Zero] {
        let collinearity = cross3(&ac, &ab);
        return match signs3(&collinearity, policy, &mut trace) {
            Ok([Sign::Zero, Sign::Zero, Sign::Zero]) => {
                match classify_collinear_segments3(a, b, c, d, policy, &mut trace) {
                    Ok(relation) => {
                        PredicateOutcome::decided(relation, trace.certainty, trace.stage)
                    }
                    Err(unknown) => unknown.into_outcome(),
                }
            }
            Ok(_) => PredicateOutcome::decided(
                Segment3Intersection::CoplanarDisjoint,
                trace.certainty,
                trace.stage,
            ),
            Err(unknown) => unknown.into_outcome(),
        };
    }

    let coplanarity = dot3(&ac, &normal);
    match sign_of_real(&coplanarity, policy, &mut trace) {
        Ok(Sign::Zero) => {}
        Ok(_) => {
            return PredicateOutcome::decided(
                Segment3Intersection::SkewDisjoint,
                trace.certainty,
                trace.stage,
            );
        }
        Err(unknown) => return unknown.into_outcome(),
    }

    let axis = nonzero_axis(normal_signs).expect("normal has a certified nonzero component");
    let t_numerators = cross3(&ac, &cd);
    let u_numerators = cross3(&ac, &ab);
    let t = match classify_parameter_01(
        coordinate(&t_numerators, axis),
        coordinate(&normal, axis),
        policy,
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };
    let u = match classify_parameter_01(
        coordinate(&u_numerators, axis),
        coordinate(&normal, axis),
        policy,
        &mut trace,
    ) {
        Ok(location) => location,
        Err(unknown) => return unknown.into_outcome(),
    };

    if !t.on_segment || !u.on_segment {
        return PredicateOutcome::decided(
            Segment3Intersection::CoplanarDisjoint,
            trace.certainty,
            trace.stage,
        );
    }
    let relation = if t.on_boundary || u.on_boundary {
        Segment3Intersection::EndpointTouch
    } else {
        Segment3Intersection::Proper
    };
    PredicateOutcome::decided(relation, trace.certainty, trace.stage)
}

/// Classify the intersection of closed segments `ab` and `cd` with an explicit
/// predicate escalation policy.
pub fn classify_segment_intersection_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_impl(a, b, c, d, policy, None, None)
}

/// Classify the intersection of closed segments `ab` and `cd` with both an
/// explicit policy and cached structural facts for both segments.
///
/// Known-degenerate facts let this function reduce point-segment or point-point
/// cases before evaluating the four-orientation classifier. The reduction never
/// accepts lossy coordinates: every remaining equality or containment question
/// is certified by exact Real predicates.
pub fn classify_segment_intersection_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    first_facts: Segment2Facts,
    second_facts: Segment2Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_impl(a, b, c, d, policy, Some(first_facts), Some(second_facts))
}

fn classify_segment_intersection_impl(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
    first_facts: Option<Segment2Facts>,
    second_facts: Option<Segment2Facts>,
) -> PredicateOutcome<SegmentIntersection> {
    if let Some(outcome) = classify_known_degenerate_segment_intersection(
        a,
        b,
        c,
        d,
        policy,
        first_facts,
        second_facts,
    ) {
        return outcome;
    }

    let mut trace = DecisionTrace::default();

    // This is the standard four-orientation segment classifier. Every
    // orientation and interval comparison routes through exact
    // hyperreal-backed determinant signs.
    let o1 = match decided(orient2d_with_policy(a, b, c, policy), &mut trace) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };
    let o2 = match decided(orient2d_with_policy(a, b, d, policy), &mut trace) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };
    let o3 = match decided(orient2d_with_policy(c, d, a, policy), &mut trace) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };
    let o4 = match decided(orient2d_with_policy(c, d, b, policy), &mut trace) {
        Ok(sign) => sign,
        Err(unknown) => return unknown.into_outcome(),
    };

    if o1 == Sign::Zero && o2 == Sign::Zero && o3 == Sign::Zero && o4 == Sign::Zero {
        return match classify_collinear_segments(a, b, c, d, policy, &mut trace) {
            Ok(kind) => PredicateOutcome::decided(kind, trace.certainty, trace.stage),
            Err(unknown) => unknown.into_outcome(),
        };
    }

    if opposite_strict(o1, o2) && opposite_strict(o3, o4) {
        return PredicateOutcome::decided(
            SegmentIntersection::Proper,
            trace.certainty,
            trace.stage,
        );
    }

    for (segment_start, segment_end, point, sign) in
        [(a, b, c, o1), (a, b, d, o2), (c, d, a, o3), (c, d, b, o4)]
    {
        if sign == Sign::Zero {
            match classify_collinear_point_segment(
                segment_start,
                segment_end,
                point,
                policy,
                &mut trace,
            ) {
                Ok(location) if location.is_on_segment() => {
                    return PredicateOutcome::decided(
                        SegmentIntersection::EndpointTouch,
                        trace.certainty,
                        trace.stage,
                    );
                }
                Ok(_) => {}
                Err(unknown) => return unknown.into_outcome(),
            }
        }
    }

    PredicateOutcome::decided(SegmentIntersection::Disjoint, trace.certainty, trace.stage)
}

fn classify_known_degenerate_segment_intersection(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
    first_facts: Option<Segment2Facts>,
    second_facts: Option<Segment2Facts>,
) -> Option<PredicateOutcome<SegmentIntersection>> {
    match (
        first_facts.and_then(Segment2Facts::known_degenerate),
        second_facts.and_then(Segment2Facts::known_degenerate),
    ) {
        (Some(true), Some(true)) => {
            let mut trace = DecisionTrace::default();
            Some(match points_equal(a, c, policy, &mut trace) {
                Ok(true) => PredicateOutcome::decided(
                    SegmentIntersection::Identical,
                    trace.certainty,
                    trace.stage,
                ),
                Ok(false) => PredicateOutcome::decided(
                    SegmentIntersection::Disjoint,
                    trace.certainty,
                    trace.stage,
                ),
                Err(unknown) => unknown.into_outcome(),
            })
        }
        (Some(true), _) => Some(point_segment_intersection_from_classifier(
            classify_point_segment_impl(c, d, a, policy, second_facts, None),
        )),
        (_, Some(true)) => Some(point_segment_intersection_from_classifier(
            classify_point_segment_impl(a, b, c, policy, first_facts, None),
        )),
        _ => None,
    }
}

fn point_segment_intersection_from_classifier(
    outcome: PredicateOutcome<PointSegmentLocation>,
) -> PredicateOutcome<SegmentIntersection> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(
            if value.is_on_segment() {
                SegmentIntersection::EndpointTouch
            } else {
                SegmentIntersection::Disjoint
            },
            certainty,
            stage,
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn point_segment3_intersection_from_classifier(
    outcome: PredicateOutcome<PointSegmentLocation>,
) -> PredicateOutcome<Segment3Intersection> {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(
            if value.is_on_segment() {
                Segment3Intersection::EndpointTouch
            } else {
                Segment3Intersection::CoplanarDisjoint
            },
            certainty,
            stage,
        ),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn classify_collinear_segments(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<SegmentIntersection, UnknownDecision> {
    if (points_equal(a, c, policy, trace)? && points_equal(b, d, policy, trace)?)
        || (points_equal(a, d, policy, trace)? && points_equal(b, c, policy, trace)?)
    {
        return Ok(SegmentIntersection::Identical);
    }

    let mut shared = Vec::new();
    if classify_collinear_point_segment(a, b, c, policy, trace)?.is_on_segment() {
        push_unique_point(&mut shared, c, policy, trace)?;
    }
    if classify_collinear_point_segment(a, b, d, policy, trace)?.is_on_segment() {
        push_unique_point(&mut shared, d, policy, trace)?;
    }
    if classify_collinear_point_segment(c, d, a, policy, trace)?.is_on_segment() {
        push_unique_point(&mut shared, a, policy, trace)?;
    }
    if classify_collinear_point_segment(c, d, b, policy, trace)?.is_on_segment() {
        push_unique_point(&mut shared, b, policy, trace)?;
    }

    Ok(match shared.len() {
        0 => SegmentIntersection::Disjoint,
        1 => SegmentIntersection::EndpointTouch,
        _ => SegmentIntersection::CollinearOverlap,
    })
}

fn classify_collinear_segments3(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<Segment3Intersection, UnknownDecision> {
    if (points_equal3(a, c, policy, trace)? && points_equal3(b, d, policy, trace)?)
        || (points_equal3(a, d, policy, trace)? && points_equal3(b, c, policy, trace)?)
    {
        return Ok(Segment3Intersection::Identical);
    }

    let mut shared = Vec::new();
    if classify_collinear_point_segment3(a, b, c, policy, trace)?.is_on_segment() {
        push_unique_point3(&mut shared, c, policy, trace)?;
    }
    if classify_collinear_point_segment3(a, b, d, policy, trace)?.is_on_segment() {
        push_unique_point3(&mut shared, d, policy, trace)?;
    }
    if classify_collinear_point_segment3(c, d, a, policy, trace)?.is_on_segment() {
        push_unique_point3(&mut shared, a, policy, trace)?;
    }
    if classify_collinear_point_segment3(c, d, b, policy, trace)?.is_on_segment() {
        push_unique_point3(&mut shared, b, policy, trace)?;
    }

    Ok(match shared.len() {
        0 => Segment3Intersection::CoplanarDisjoint,
        1 => Segment3Intersection::EndpointTouch,
        _ => Segment3Intersection::CollinearOverlap,
    })
}

fn classify_collinear_point_segment(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<PointSegmentLocation, UnknownDecision> {
    if !between_closed(&a.x, &b.x, &point.x, policy, trace)?
        || !between_closed(&a.y, &b.y, &point.y, policy, trace)?
    {
        return Ok(PointSegmentLocation::CollinearOutside);
    }

    if points_equal(a, point, policy, trace)? || points_equal(b, point, policy, trace)? {
        Ok(PointSegmentLocation::OnEndpoint)
    } else {
        Ok(PointSegmentLocation::OnSegment)
    }
}

fn classify_degenerate_point_segment(
    endpoint: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<PointSegmentLocation, UnknownDecision> {
    if points_equal(endpoint, point, policy, trace)? {
        Ok(PointSegmentLocation::OnEndpoint)
    } else {
        Ok(PointSegmentLocation::CollinearOutside)
    }
}

fn classify_collinear_point_segment3(
    a: &Point3,
    b: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<PointSegmentLocation, UnknownDecision> {
    if !between_closed(&a.x, &b.x, &point.x, policy, trace)?
        || !between_closed(&a.y, &b.y, &point.y, policy, trace)?
        || !between_closed(&a.z, &b.z, &point.z, policy, trace)?
    {
        return Ok(PointSegmentLocation::CollinearOutside);
    }

    if points_equal3(a, point, policy, trace)? || points_equal3(b, point, policy, trace)? {
        Ok(PointSegmentLocation::OnEndpoint)
    } else {
        Ok(PointSegmentLocation::OnSegment)
    }
}

fn classify_degenerate_point_segment3(
    endpoint: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<PointSegmentLocation, UnknownDecision> {
    if points_equal3(endpoint, point, policy, trace)? {
        Ok(PointSegmentLocation::OnEndpoint)
    } else {
        Ok(PointSegmentLocation::CollinearOutside)
    }
}

fn point_segment3_cross_signs(
    a: &Point3,
    b: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<[Sign; 3], UnknownDecision> {
    let abx = sub_ref(&b.x, &a.x);
    let aby = sub_ref(&b.y, &a.y);
    let abz = sub_ref(&b.z, &a.z);
    let apx = sub_ref(&point.x, &a.x);
    let apy = sub_ref(&point.y, &a.y);
    let apz = sub_ref(&point.z, &a.z);

    let cross_x = sub_ref(&mul_ref(&aby, &apz), &mul_ref(&abz, &apy));
    let cross_y = sub_ref(&mul_ref(&abz, &apx), &mul_ref(&abx, &apz));
    let cross_z = sub_ref(&mul_ref(&abx, &apy), &mul_ref(&aby, &apx));

    Ok([
        sign_of_real(&cross_x, policy, trace)?,
        sign_of_real(&cross_y, policy, trace)?,
        sign_of_real(&cross_z, policy, trace)?,
    ])
}

#[derive(Clone, Debug)]
struct Vector3Real {
    x: Real,
    y: Real,
    z: Real,
}

#[derive(Clone, Copy, Debug)]
struct Parameter01 {
    on_segment: bool,
    on_boundary: bool,
}

fn vector3_between(start: &Point3, end: &Point3) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
        z: sub_ref(&end.z, &start.z),
    }
}

fn cross2(left_x: &Real, left_y: &Real, right_x: &Real, right_y: &Real) -> Real {
    sub_ref(&mul_ref(left_x, right_y), &mul_ref(left_y, right_x))
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

fn signs3(
    value: &Vector3Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<[Sign; 3], UnknownDecision> {
    Ok([
        sign_of_real(&value.x, policy, trace)?,
        sign_of_real(&value.y, policy, trace)?,
        sign_of_real(&value.z, policy, trace)?,
    ])
}

fn nonzero_axis(signs: [Sign; 3]) -> Option<usize> {
    signs.into_iter().position(|sign| sign != Sign::Zero)
}

fn coordinate(vector: &Vector3Real, axis: usize) -> &Real {
    match axis {
        0 => &vector.x,
        1 => &vector.y,
        2 => &vector.z,
        _ => unreachable!("3D vector axis is in 0..3"),
    }
}

fn classify_parameter_01(
    numerator: &Real,
    denominator: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<Parameter01, UnknownDecision> {
    let denominator_sign = sign_of_real(denominator, policy, trace)?;
    let (normalized_numerator, normalized_denominator) = if denominator_sign == Sign::Negative {
        (
            sub_ref(&Real::from(0), numerator),
            sub_ref(&Real::from(0), denominator),
        )
    } else {
        (numerator.clone(), denominator.clone())
    };
    let numerator_sign = sign_of_real(&normalized_numerator, policy, trace)?;
    if numerator_sign == Sign::Negative {
        return Ok(Parameter01 {
            on_segment: false,
            on_boundary: false,
        });
    }
    let upper_margin = sub_ref(&normalized_denominator, &normalized_numerator);
    let upper_sign = sign_of_real(&upper_margin, policy, trace)?;
    if upper_sign == Sign::Negative {
        return Ok(Parameter01 {
            on_segment: false,
            on_boundary: false,
        });
    }
    Ok(Parameter01 {
        on_segment: true,
        on_boundary: numerator_sign == Sign::Zero || upper_sign == Sign::Zero,
    })
}

fn between_closed(
    a: &Real,
    b: &Real,
    point: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    let pa = sign_of_difference(point, a, policy, trace)?;
    let pb = sign_of_difference(point, b, policy, trace)?;
    Ok(matches!(
        (pa, pb),
        (Sign::Zero, _)
            | (_, Sign::Zero)
            | (Sign::Positive, Sign::Negative)
            | (Sign::Negative, Sign::Positive)
    ))
}

fn points_equal(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(
        sign_of_difference(&left.x, &right.x, policy, trace)? == Sign::Zero
            && sign_of_difference(&left.y, &right.y, policy, trace)? == Sign::Zero,
    )
}

fn points_equal3(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<bool, UnknownDecision> {
    Ok(
        sign_of_difference(&left.x, &right.x, policy, trace)? == Sign::Zero
            && sign_of_difference(&left.y, &right.y, policy, trace)? == Sign::Zero
            && sign_of_difference(&left.z, &right.z, policy, trace)? == Sign::Zero,
    )
}

fn push_unique_point<'a>(
    points: &mut Vec<&'a Point2>,
    point: &'a Point2,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<(), UnknownDecision> {
    for existing in points.iter() {
        if points_equal(existing, point, policy, trace)? {
            return Ok(());
        }
    }
    points.push(point);
    Ok(())
}

fn push_unique_point3<'a>(
    points: &mut Vec<&'a Point3>,
    point: &'a Point3,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<(), UnknownDecision> {
    for existing in points.iter() {
        if points_equal3(existing, point, policy, trace)? {
            return Ok(());
        }
    }
    points.push(point);
    Ok(())
}

fn sign_of_difference(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<Sign, UnknownDecision> {
    decided(
        map_outcome(
            compare_reals_with_policy(left, right, policy),
            |ordering| match ordering {
                Ordering::Less => Sign::Negative,
                Ordering::Equal => Sign::Zero,
                Ordering::Greater => Sign::Positive,
            },
        ),
        trace,
    )
}

fn sign_of_real(
    value: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<Sign, UnknownDecision> {
    decided(
        resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement),
        trace,
    )
}

fn opposite_strict(left: Sign, right: Sign) -> bool {
    matches!(
        (left, right),
        (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive)
    )
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

fn decided<T: Copy>(
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

    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(real(x), real(y))
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    #[test]
    fn point_segment_classifier_distinguishes_endpoint_inside_and_outside() {
        let a = p2(0, 0);
        let b = p2(4, 0);

        assert_eq!(
            crate::classify_point_segment(&a, &b, &p2(2, 0), APPROX).value(),
            Some(PointSegmentLocation::OnSegment)
        );
        assert_eq!(
            crate::classify_point_segment(&a, &b, &p2(4, 0), APPROX).value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            crate::classify_point_segment(&a, &b, &p2(5, 0), APPROX).value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
        assert_eq!(
            crate::classify_point_segment(&a, &b, &p2(2, 1), APPROX).value(),
            Some(PointSegmentLocation::OffLine)
        );
    }

    #[test]
    fn retained_line_orientation_preserves_complete_point_segment_classification() {
        let rational = |numerator, denominator| {
            hyperreal::Real::from(hyperreal::Rational::fraction(numerator, denominator).unwrap())
        };
        let a = Point2::new(rational(1, 3), real(0));
        let b = Point2::new(rational(7, 3), real(0));
        let queries = [
            Point2::new(rational(4, 3), real(0)),
            b.clone(),
            Point2::new(rational(10, 3), real(0)),
            Point2::new(rational(4, 3), rational(1, 7)),
        ];
        let orientation = crate::line2_orientation(&a, &b);

        for policy in [PredicatePolicy::STRICT, APPROX] {
            for query in &queries {
                let direct = crate::classify_point_segment(&a, &b, query, policy);
                let retained = crate::classify_point_segment_with_orientation(
                    &a,
                    &b,
                    query,
                    &orientation,
                    policy,
                );
                assert_eq!(retained, direct);
                assert_eq!(
                    crate::point_on_segment_with_orientation(&a, &b, query, &orientation, policy,),
                    crate::point_on_segment(&a, &b, query, policy),
                );
            }
        }
    }

    #[test]
    fn point_segment3_classifier_distinguishes_endpoint_inside_outside_and_offline() {
        let a = p3(0, 0, 0);
        let b = p3(4, 4, 4);

        assert_eq!(
            crate::classify_point_segment3(&a, &b, &p3(2, 2, 2), APPROX).value(),
            Some(PointSegmentLocation::OnSegment)
        );
        assert_eq!(
            crate::classify_point_segment3(&a, &b, &p3(4, 4, 4), APPROX).value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            crate::classify_point_segment3(&a, &b, &p3(5, 5, 5), APPROX).value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
        assert_eq!(
            crate::classify_point_segment3(&a, &b, &p3(2, 2, 3), APPROX).value(),
            Some(PointSegmentLocation::OffLine)
        );
        assert_eq!(
            crate::point_on_segment3(&a, &b, &p3(2, 2, 2), APPROX).value(),
            Some(true)
        );
    }

    #[test]
    fn segment_classifier_reports_proper_endpoint_overlap_and_identical() {
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 4),
                &p2(0, 4),
                &p2(4, 0),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Proper)
        );
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 0),
                &p2(4, 0),
                &p2(6, 0),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::EndpointTouch)
        );
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 0),
                &p2(2, 0),
                &p2(6, 0),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::CollinearOverlap)
        );
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 0),
                &p2(4, 0),
                &p2(0, 0),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Identical)
        );
    }

    #[test]
    fn line_intersection_point_constructs_exact_crossing() {
        assert_eq!(
            crate::construct_line_intersection_point(&p2(0, 0), &p2(4, 4), &p2(0, 4), &p2(4, 0)),
            Some(p2(2, 2))
        );
        assert_eq!(
            crate::construct_line_intersection_point(&p2(0, 0), &p2(4, 0), &p2(4, 1), &p2(6, 1)),
            None
        );
        // Construction is deliberately independent of closed-segment
        // topology: these disjoint segments have intersecting supporting lines.
        assert_eq!(
            crate::construct_line_intersection_point(&p2(0, 0), &p2(1, 0), &p2(2, -1), &p2(2, 1)),
            Some(p2(2, 0))
        );
    }

    #[test]
    fn line_intersection_point_fuses_nondyadic_exact_rationals() {
        let rational = |numerator, denominator| {
            Real::new(hyperreal::Rational::fraction(numerator, denominator).unwrap())
        };
        let zero = Real::zero();
        let two_thirds = rational(2, 3);
        let one_third = rational(1, 3);
        let a = Point2::new(zero.clone(), zero.clone());
        let b = Point2::new(two_thirds.clone(), two_thirds.clone());
        let c = Point2::new(zero.clone(), two_thirds.clone());
        let d = Point2::new(two_thirds, zero);

        let point = construct_line_intersection_point(&a, &b, &c, &d)
            .expect("nonparallel exact lines have an exact intersection");
        assert_eq!(point.x, one_third);
        assert_eq!(point.y, one_third);
    }

    #[test]
    fn segment_classifier_reports_disjoint_collinear_and_skew_cases() {
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 0),
                &p2(5, 0),
                &p2(6, 0),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_segment_intersection(
                &p2(0, 0),
                &p2(4, 0),
                &p2(5, 1),
                &p2(6, 1),
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
    }

    #[test]
    fn fact_aware_point_segment_classifier_handles_degenerate_segments() {
        let endpoint = p2(2, 3);
        let facts = crate::geometry::segment2_facts(&endpoint, &endpoint);

        assert_eq!(
            crate::classify_point_segment_with_facts(
                &endpoint, &endpoint, &endpoint, facts, APPROX
            )
            .value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            crate::classify_point_segment_with_facts(
                &endpoint,
                &endpoint,
                &p2(2, 4),
                facts,
                APPROX
            )
            .value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
        assert_eq!(
            crate::point_on_segment_with_facts(&endpoint, &endpoint, &endpoint, facts, APPROX)
                .value(),
            Some(true)
        );
    }

    #[test]
    fn fact_aware_segment_classifier_reduces_point_segment_cases() {
        let point = p2(2, 0);
        let point_facts = crate::geometry::segment2_facts(&point, &point);
        let start = p2(0, 0);
        let end = p2(4, 0);
        let segment_facts = crate::geometry::segment2_facts(&start, &end);

        assert_eq!(
            crate::classify_segment_intersection_with_facts(
                &point,
                &point,
                &start,
                &end,
                point_facts,
                segment_facts,
                APPROX
            )
            .value(),
            Some(SegmentIntersection::EndpointTouch)
        );

        let other_point = p2(9, 0);
        let other_facts = crate::geometry::segment2_facts(&other_point, &other_point);
        assert_eq!(
            crate::classify_segment_intersection_with_facts(
                &point,
                &point,
                &other_point,
                &other_point,
                point_facts,
                other_facts,
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            crate::classify_segment_intersection_with_facts(
                &point,
                &point,
                &point,
                &point,
                point_facts,
                point_facts,
                APPROX
            )
            .value(),
            Some(SegmentIntersection::Identical)
        );
    }

    #[test]
    fn immediate_segment_predicates_reuse_cached_facts() {
        let a = p2(0, 0);
        let b = p2(4, 0);
        let facts = crate::geometry::segment2_facts(&a, &b);
        assert_eq!(facts.known_degenerate(), Some(false));
        assert_eq!(
            crate::classify_point_segment_with_facts(&a, &b, &p2(2, 0), facts, APPROX).value(),
            Some(PointSegmentLocation::OnSegment)
        );

        let point = p2(2, 0);
        let point_facts = crate::geometry::segment2_facts(&point, &point);
        assert_eq!(
            crate::classify_segment_intersection_with_facts(
                &a,
                &b,
                &point,
                &point,
                facts,
                point_facts,
                APPROX
            )
            .value(),
            Some(SegmentIntersection::EndpointTouch)
        );
    }

    #[test]
    fn immediate_segment3_predicates_use_borrowed_endpoints() {
        let a = p3(0, 0, 0);
        let b = p3(0, 0, 3);

        assert_eq!(
            crate::classify_point_segment3(&a, &b, &p3(0, 0, 2), APPROX).value(),
            Some(PointSegmentLocation::OnSegment)
        );
        assert_eq!(
            crate::point_on_segment3(&a, &b, &p3(0, 1, 2), APPROX).value(),
            Some(false)
        );
    }

    #[test]
    fn segment3_classifier_distinguishes_skew_coplanar_and_crossing_cases() {
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(2, -1, 0),
                &p3(2, 1, 0),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::Proper)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(4, 0, 0),
                &p3(4, 2, 0),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::EndpointTouch)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(5, 1, 0),
                &p3(6, 1, 0),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::CoplanarDisjoint)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(2, -1, 1),
                &p3(2, 1, 1),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::SkewDisjoint)
        );
    }

    #[test]
    fn segment3_classifier_reports_collinear_overlap_and_identical() {
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 4, 4),
                &p3(2, 2, 2),
                &p3(6, 6, 6),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::CollinearOverlap)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 4, 4),
                &p3(4, 4, 4),
                &p3(0, 0, 0),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::Identical)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 4, 4),
                &p3(5, 5, 5),
                &p3(6, 6, 6),
                APPROX
            )
            .value(),
            Some(Segment3Intersection::CoplanarDisjoint)
        );
    }

    #[test]
    fn immediate_segment3_classifies_intersection() {
        let a = p3(0, 0, 0);
        let b = p3(4, 0, 0);
        let c = p3(2, -1, 0);
        let d = p3(2, 1, 0);

        assert_eq!(
            crate::classify_segment3_intersection(&a, &b, &c, &d, APPROX).value(),
            Some(Segment3Intersection::Proper)
        );
    }
}
