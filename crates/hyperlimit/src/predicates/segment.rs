//! Closed-segment incidence, intersection relations, and supporting-line construction.
//!
//! Orientation, cross-product, and scalar interval decisions follow the selected
//! policy; exact coordinate storage does not make every decision exact. Retained
//! orientation/fact entrypoints require matching current sources and do not
//! independently validate that association. Most classifiers accumulate local
//! receipts, with the one-collapsed-spatial-segment delegation exception documented
//! below. Line construction instead returns an Option and drops decision metadata.
//! Geometry storage, rings, and sweep state remain in consuming crates.

use crate::classify::{PointSegmentLocation, Segment3Intersection, SegmentIntersection};
use crate::geometry::{Point2, Point3, Segment2Facts};
use crate::predicate::PredicatePolicy;
use crate::predicate::{PredicateOutcome, RefinementNeed, Sign};
use crate::predicates::decision::{DecisionTrace, UnknownDecision, max_certainty, max_stage};
use crate::predicates::order::compare_reals_with_policy;
use crate::predicates::orient::{
    Line2Orientation, Line2OrientationQuery, orient2d_with_orientation_and_policy,
    orient2d_with_orientation_and_query_and_policy, orient2d_with_policy,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign_direct};
use core::cmp::Ordering;
use hyperreal::Real;

/// Classify a point against a closed spatial segment under the supplied policy.
///
/// Checks endpoint equality first; a collapsed segment gives OnEndpoint for an
/// equal query and CollinearOutside otherwise. For a noncollapsed segment, resolves
/// all three components of `(b-a) x (point-a)` before deciding OffLine or testing
/// coordinate intervals and endpoints. A later unresolved component can therefore
/// return Unknown even after an earlier nonzero component. Accumulates consumed
/// decision receipts; approximate-policy decisions retain their qualification.
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

/// Classify a point against a closed planar segment using orientation then intervals.
///
/// A nonzero orientation gives OffLine. Zero proceeds through inclusive x/y bounds
/// and endpoint equality to OnEndpoint, OnSegment, or CollinearOutside. Collapsed
/// segments follow the same zero-orientation path. Consumed receipts are retained
/// and an encountered Unknown propagates.
pub fn classify_point_segment_with_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None, None, None)
}

/// Classify a planar point using retained line evidence for the same ordered endpoints.
///
/// Source association is a caller precondition: retained filters can decide before
/// reading current endpoint coordinates and are not revalidated here. Rebuild
/// stale evidence after changes. A filter decline uses the orientation owner's
/// remaining cascade; interval checks follow the supplied policy and retain their
/// receipts rather than guaranteeing an exact result under every policy.
pub fn classify_point_segment_with_orientation_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None, Some(orientation), None)
}

/// Classify a planar point with matching retained line and query evidence.
///
/// Both records must correspond to the supplied current coordinates and ordered
/// endpoints. Neither association is checked before reuse. Inherits the scheduling
/// and receipt behavior of [`classify_point_segment_with_orientation_and_policy`].
pub fn classify_point_segment_with_orientation_and_query_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    query: &Line2OrientationQuery,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None, Some(orientation), Some(query))
}

/// Classify a planar point using a caller-established orientation outcome when admissible.
///
/// The outcome must describe `orient2(a, b, point)` for these current inputs.
/// Only policy acceptance of its certainty is checked; source identity, the sign,
/// and the claimed certificate/stage are trusted. Unknown or disallowed receipts
/// trigger fresh orientation. Nonzero gives OffLine; zero skips the determinant
/// and tests coordinate bounds/endpoints. The result combines the retained or
/// recomputed receipt with those consumed by interval/equality decisions.
pub fn classify_point_segment_with_known_orientation_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: PredicateOutcome<Sign>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();
    let orientation = match reusable_orientation(orientation, policy) {
        Some(orientation) => orientation,
        None => orient2d_with_policy(a, b, point, policy),
    };
    let orientation = match decided(orientation, &mut trace) {
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

/// Accept a decided orientation if the policy accepts its certainty; otherwise return None.
///
/// Does not validate the claimed sign, stage, source tuple, or evidence identity.
fn reusable_orientation(
    orientation: PredicateOutcome<Sign>,
    policy: PredicatePolicy,
) -> Option<PredicateOutcome<Sign>> {
    match orientation {
        PredicateOutcome::Decided { certainty, .. } if policy.accepts(certainty) => {
            Some(orientation)
        }
        _ => None,
    }
}

/// Classify a point assumed collinear with the planar endpoints, without an orientation query.
///
/// Callers must establish the collinearity relation and combine any inherited
/// qualification themselves. This outcome starts a fresh trace for coordinate
/// interval/equality decisions only; an off-line point inside the box can otherwise
/// be classified as OnSegment.
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

/// Classify a planar point using segment facts that must match the current endpoints.
///
/// Known-degenerate facts bypass orientation and compare only `a` with the query;
/// `b` is not rechecked. Other facts fall through to the ordinary path. Stale facts
/// can change the answer, so source association is a caller precondition. Consumed
/// predicate receipts are preserved, but facts carry no inherited receipt here.
pub fn classify_point_segment_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    segment_facts: Segment2Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, Some(segment_facts), None, None)
}

/// Apply a trusted degeneracy shortcut, then selected retained/direct orientation and bounds.
///
/// Known-degenerate facts use only endpoint a. Otherwise line+query evidence takes
/// priority over line evidence, then direct orientation. Query evidence without a
/// line is ignored. Nonzero orientation returns OffLine; zero delegates to the
/// collinear classifier, accumulating receipts and propagating Unknown.
fn classify_point_segment_impl(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    segment_facts: Option<Segment2Facts>,
    orientation: Option<&Line2Orientation>,
    query: Option<&Line2OrientationQuery>,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();

    if segment_facts.and_then(Segment2Facts::known_degenerate) == Some(true) {
        return match classify_degenerate_point_segment(a, point, policy, &mut trace) {
            Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
            Err(unknown) => unknown.into_outcome(),
        };
    }

    let orientation_outcome = match (orientation, query) {
        (Some(orientation), Some(query)) => {
            orient2d_with_orientation_and_query_and_policy(a, b, point, orientation, query, policy)
        }
        (Some(orientation), None) => {
            orient2d_with_orientation_and_policy(a, b, point, orientation, policy)
        }
        (None, _) => orient2d_with_policy(a, b, point, policy),
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

/// Map the spatial classifier to closed-segment membership, preserving its receipt or Unknown.
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

/// Map the planar classifier to closed-segment membership, preserving its receipt or Unknown.
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

/// Map retained-line classification to closed membership, preserving its outcome metadata.
///
/// Inherits the matching-source precondition of
/// [`classify_point_segment_with_orientation_and_policy`].
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

/// Map retained line/query classification to closed membership with the same receipt.
///
/// Both retained records must match current inputs, as required by
/// [`classify_point_segment_with_orientation_and_query_and_policy`].
pub fn point_on_segment_with_orientation_and_query_and_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    query: &Line2OrientationQuery,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match classify_point_segment_with_orientation_and_query_and_policy(
        a,
        b,
        point,
        orientation,
        query,
        policy,
    ) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Map fact-assisted classification to closed membership, preserving its receipt or Unknown.
///
/// Facts must match the supplied endpoints; see
/// [`classify_point_segment_with_policy_and_facts`] for the trusted degeneracy shortcut.
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

/// Construct a supporting-line intersection using STRICT policy.
///
/// Uses the policy-bearing constructor's exact rational paths or the expression
/// `a + t*(b-a)`, where `t = cross(c-a,d-c) / cross(b-a,d-c)`. Parameters need not
/// lie in `[0, 1]`; segment membership requires a separate incidence/classification
/// check. None combines parallel/coincident/degenerate lines with scalar quotient
/// failure or unresolved nonzeroness; it is not a typed no-solution proof. No
/// certainty receipt or explicit intersection-incidence record is returned.
pub fn construct_line_intersection_point(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Option<Point2> {
    construct_line_intersection_point_with_policy(a, b, c, d, PredicatePolicy::STRICT)
}

/// Construct a supporting-line intersection, using policy ordering in the general fallback.
///
/// Tries the small dyadic, wide dyadic, and fused exact-rational paths in that
/// order before general Real expressions. A decided zero denominator returns
/// None; a decided nonzero denominator uses the assuming-nonzero reciprocal hook.
/// If the comparison is Unknown, ordinary reciprocal construction is still tried
/// and may succeed. Any reciprocal error becomes None.
///
/// The comparison's certainty/stage are discarded, including approximate decisions.
/// The returned point therefore carries no receipt justifying policy-dependent
/// construction or source incidence. No segment-domain check is performed; retain
/// external evidence when consuming this Option as an exact construction.
pub fn construct_line_intersection_point_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
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
    let reciprocal = match compare_reals_with_policy(&denominator, &Real::zero(), policy).value() {
        Some(Ordering::Equal) => return None,
        Some(Ordering::Less | Ordering::Greater) => {
            denominator.inverse_ref_assuming_nonzero().ok()?
        }
        None => denominator.inverse_ref().ok()?,
    };
    let t = numerator * reciprocal;

    Some(Point2::new(
        add_ref(&a.x, &mul_ref(&t, &ab_x)),
        add_ref(&a.y, &mul_ref(&t, &ab_y)),
    ))
}

/// Classify two closed spatial segments, including collapsed, parallel, and skew cases.
///
/// Tests both endpoint pairs for collapse. Two collapsed segments are Identical
/// or CoplanarDisjoint; one collapsed segment delegates to point membership.
/// Otherwise cross-product signs distinguish parallel lines, a scalar triple
/// product checks coplanarity, and a nonzero normal component supplies line
/// parameters `t = ((c-a)x(d-c))_k / ((b-a)x(d-c))_k` and
/// `u = ((c-a)x(b-a))_k / ((b-a)x(d-c))_k`. Parameter membership in `[0, 1]` is
/// compared without division. Parallel collinear segments use endpoint incidence.
///
/// Most branches accumulate consumed receipts and propagate Unknown. The
/// one-collapsed-segment branches return a fresh point-classifier outcome without
/// combining the preceding collapse-test trace; their receipt omits that earlier
/// qualification. Classification constructs no intersection point or overlap arc.
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

/// Classify closed planar segments from four ordered orientations and endpoint incidence.
///
/// All four signs are obtained before selecting the all-collinear, proper-crossing,
/// or endpoint-touch/disjoint branch. Collinear reversed endpoints can be Identical;
/// point segments are included. Accumulates consumed receipts and stops on an
/// encountered Unknown. Returns a relation, not constructed intersection geometry.
pub fn classify_segment_intersection_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_impl(a, b, c, d, policy, None, None)
}

/// Classify planar segments using source-matching facts for optional collapse reductions.
///
/// Known-degenerate facts are trusted without rechecking their endpoints. Two
/// collapsed operands reduce to point equality; one reduces to point membership.
/// Other inputs use four orientations. Facts carry no source binding or receipt
/// here, so stale or misassociated snapshots can produce incorrect classifications.
/// The remaining decisions use the supplied policy and preserve their receipts.
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

/// Try trusted known-collapse reductions, then use the ordinary four-orientation classifier.
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

    classify_segment_intersection_from_orientations(a, b, c, d, None, policy)
}

/// Classify planar segments with four caller-established orientation outcomes.
///
/// Array order is `(a,b,c)`, `(a,b,d)`, `(c,d,a)`, `(c,d,b)`. Each accepted decided
/// outcome must describe that current tuple; only policy acceptance of certainty
/// is checked. Unknown/disallowed entries are recomputed in order. Collinear and
/// endpoint checks then use coordinate comparisons, combining consumed receipts.
/// Claimed signs, stages, and source identities are not independently validated.
pub fn classify_segment_intersection_with_known_orientations_and_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    orientations: [PredicateOutcome<Sign>; 4],
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_from_orientations(a, b, c, d, Some(orientations), policy)
}

/// Resolve four signs in tuple order, then classify collinearity, crossing, or endpoint contact.
///
/// Accepted retained outcomes bypass determinant evaluation. All-zero signs invoke
/// collinear endpoint collection; two opposite sign pairs give Proper. Remaining
/// zero-sign endpoints are checked in order until contact is found, after which
/// later endpoint checks are skipped. Unknown preserves its reason/stage.
fn classify_segment_intersection_from_orientations(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    retained: Option<[PredicateOutcome<Sign>; 4]>,
    policy: PredicatePolicy,
) -> PredicateOutcome<SegmentIntersection> {
    let mut trace = DecisionTrace::default();

    // Resolve the four orientations in this order, reusing policy-admissible
    // caller outcomes when supplied; later interval decisions use the same policy.
    let triples = [(a, b, c), (a, b, d), (c, d, a), (c, d, b)];
    let mut signs = [Sign::Zero; 4];
    for (index, (first, second, point)) in triples.into_iter().enumerate() {
        let outcome =
            match retained.and_then(|retained| reusable_orientation(retained[index], policy)) {
                Some(outcome) => outcome,
                None => orient2d_with_policy(first, second, point, policy),
            };
        signs[index] = match decided(outcome, &mut trace) {
            Ok(sign) => sign,
            Err(unknown) => return unknown.into_outcome(),
        };
    }
    let [o1, o2, o3, o4] = signs;

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

    let endpoint_touch = [(a, b, c, o1), (a, b, d, o2), (c, d, a, o3), (c, d, b, o4)]
        .into_iter()
        .try_fold(false, |touch, (segment_start, segment_end, point, sign)| {
            if touch || sign != Sign::Zero {
                Ok(touch)
            } else {
                classify_collinear_point_segment(
                    segment_start,
                    segment_end,
                    point,
                    policy,
                    &mut trace,
                )
                .map(PointSegmentLocation::is_on_segment)
            }
        });
    finish_planar_endpoint_touch(endpoint_touch, trace)
}

/// Package EndpointTouch or Disjoint with the accumulated trace, or propagate the endpoint-check Unknown.
fn finish_planar_endpoint_touch(
    endpoint_touch: Result<bool, UnknownDecision>,
    trace: DecisionTrace,
) -> PredicateOutcome<SegmentIntersection> {
    match endpoint_touch {
        Ok(true) => PredicateOutcome::decided(
            SegmentIntersection::EndpointTouch,
            trace.certainty,
            trace.stage,
        ),
        Ok(false) => {
            PredicateOutcome::decided(SegmentIntersection::Disjoint, trace.certainty, trace.stage)
        }
        Err(unknown) => unknown.into_outcome(),
    }
}

/// Reduce only facts known to mark collapsed planar segments, trusting their source association.
///
/// Return None if neither operand is known collapsed. For both, compare a and c;
/// for one, map membership against the other segment. No endpoint collapse is
/// revalidated, and the facts themselves contribute no certainty receipt.
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
            classify_point_segment_impl(c, d, a, policy, second_facts, None, None),
        )),
        (_, Some(true)) => Some(point_segment_intersection_from_classifier(
            classify_point_segment_impl(a, b, c, policy, first_facts, None, None),
        )),
        _ => None,
    }
}

/// Map point membership to EndpointTouch or Disjoint, forwarding the supplied receipt or Unknown.
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

/// Map point membership to EndpointTouch or CoplanarDisjoint, forwarding only the supplied outcome metadata.
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

/// Assume a common planar support and classify identical endpoints or unique shared endpoints.
///
/// Check equal endpoint pairs in either order first. Then test c, d against ab
/// and a, b against cd, deduplicating by policy equality. Zero, one, or multiple
/// shared endpoints give Disjoint, EndpointTouch, or CollinearOverlap. Updates
/// the caller trace; collinearity and its qualification are caller responsibilities.
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

/// Assume a common spatial support and classify identity or unique shared endpoints.
///
/// Tests endpoint equality in both orders, then c/d on ab and a/b on cd. Zero,
/// one, or multiple distinct shared endpoints give CoplanarDisjoint, EndpointTouch,
/// or CollinearOverlap. Consumed receipts update the caller trace; no supporting
/// line collinearity is established here.
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

/// Assume planar collinearity and check inclusive x then y intervals before endpoint equality.
///
/// An excluded coordinate gives CollinearOutside; matching a or b gives OnEndpoint;
/// otherwise return OnSegment. Updates the trace and propagates Unknown. Without
/// the collinearity precondition this is only a bounding-box membership test.
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

/// Compare a collapsed planar endpoint with the query, yielding OnEndpoint or CollinearOutside.
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

/// Assume spatial collinearity and test inclusive x/y/z bounds, then endpoint equality.
///
/// Returns CollinearOutside, OnEndpoint, or OnSegment while updating the trace.
/// Does not establish collinearity or distinguish off-line points inside the box.
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

/// Compare a collapsed spatial endpoint with the query, yielding OnEndpoint or CollinearOutside.
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

/// Construct `(b-a) x (point-a)` and resolve x, y, z signs in order.
///
/// Stops on Unknown, not on a nonzero component; successful decisions update the
/// caller trace even when a later component prevents classification.
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

/// Three exact scalar expressions used for spatial differences, cross products, and dot products.
#[derive(Clone, Debug)]
struct Vector3Real {
    /// Exact x-component expression.
    x: Real,
    /// Exact y-component expression.
    y: Real,
    /// Exact z-component expression.
    z: Real,
}

/// Closed-unit-interval and endpoint flags for a ratio with a nonzero denominator.
#[derive(Clone, Copy, Debug)]
struct Parameter01 {
    /// Whether the normalized numerator lies between zero and denominator, inclusively.
    on_segment: bool,
    /// Whether an admitted parameter equals zero or one; false when outside.
    on_boundary: bool,
}

/// Form the exact vector difference end minus start without normalization.
fn vector3_between(start: &Point3, end: &Point3) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&end.x, &start.x),
        y: sub_ref(&end.y, &start.y),
        z: sub_ref(&end.z, &start.z),
    }
}

/// Form the planar determinant `left_x*right_y - left_y*right_x` as a scalar expression.
fn cross2(left_x: &Real, left_y: &Real, right_x: &Real, right_y: &Real) -> Real {
    sub_ref(&mul_ref(left_x, right_y), &mul_ref(left_y, right_x))
}

/// Form the spatial cross-product expressions in x/y/z order without resolving signs.
fn cross3(left: &Vector3Real, right: &Vector3Real) -> Vector3Real {
    Vector3Real {
        x: sub_ref(&mul_ref(&left.y, &right.z), &mul_ref(&left.z, &right.y)),
        y: sub_ref(&mul_ref(&left.z, &right.x), &mul_ref(&left.x, &right.z)),
        z: sub_ref(&mul_ref(&left.x, &right.y), &mul_ref(&left.y, &right.x)),
    }
}

/// Build the three positive signed-product terms of a spatial dot product through Real arithmetic.
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

/// Resolve vector components x, y, z in order, updating the trace and stopping at Unknown.
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

/// Return the first x/y/z index whose supplied sign is nonzero, without validating its proof.
fn nonzero_axis(signs: [Sign; 3]) -> Option<usize> {
    signs.into_iter().position(|sign| sign != Sign::Zero)
}

/// Borrow x, y, or z for an axis index required to be below three.
///
/// Invalid indices trigger a debug assertion; without debug assertions every
/// index other than zero or one selects z.
fn coordinate(vector: &Vector3Real, axis: usize) -> &Real {
    debug_assert!(axis < 3, "3D vector axis is in 0..3");
    match axis {
        0 => &vector.x,
        1 => &vector.y,
        _ => &vector.z,
    }
}

/// Classify numerator/denominator in `[0, 1]` without division, assuming nonzero denominator.
///
/// Resolve denominator sign and negate both values if negative. Then check the
/// normalized numerator and denominator-minus-numerator, stopping at negativity;
/// equality to either endpoint marks boundary. A zero denominator is not rejected
/// here, so callers must establish nonzeroness separately. Updates the trace.
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

/// Check inclusive membership between unordered scalar endpoints using both difference signs.
///
/// Both comparisons run even when the first gives zero; a later Unknown prevents
/// a result. Equality to either endpoint or opposing signs means membership.
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

/// Compare planar coordinates x then y under policy, short-circuiting at a nonzero difference or Unknown.
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

/// Compare spatial coordinates x, y, z under policy, short-circuiting at a nonzero difference or Unknown.
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

/// Append a borrowed planar point only after it differs from every retained point under policy.
///
/// A matching point stops the scan; Unknown aborts without appending. Earlier
/// comparison receipts remain in the caller trace.
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

/// Append a borrowed spatial point unless an earlier point compares equal; propagate Unknown before append.
///
/// Comparisons follow vector order and update the caller trace.
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

/// Map policy scalar ordering to a sign while recording its receipt or forwarding Unknown.
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

/// Resolve a scalar sign with RealRefinement as the uncertainty need, updating the caller trace.
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

/// Return true only for one Positive and one Negative sign; Zero is never opposite.
fn opposite_strict(left: Sign, right: Sign) -> bool {
    matches!(
        (left, right),
        (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive)
    )
}

/// Extract a decided value while combining weakest certainty and latest stage into the trace.
///
/// Unknown returns its reason/stage unchanged and does not update the trace.
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

/// Finite segment, supporting-line, retained-evidence, and uncertainty controls.
///
/// Most integer assertions extract values. Selected retained/direct checks compare
/// complete outcomes; the retained-collinearity control explicitly tests policy
/// receipt reuse. These fixtures do not constitute exhaustive segment closure.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::{Certainty, Escalation};
    use crate::predicates::decision::{certainty_rank, stage_rank};
    use crate::test_support::exact_normal_positive;

    /// Policy allowing an approximate terminal for the decided controls; strict cases select STRICT explicitly.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Convert a test integer exactly to Real.
    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    /// Build a planar test point from exact integer coordinates.
    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(real(x), real(y))
    }

    /// Build a spatial test point from exact integer coordinates.
    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    /// Require normal-form proof of the unit-circle identity at e, then return the shared unresolved zero.
    ///
    /// Keeps the original certified regression alongside the angle-addition fixture
    /// used for strict uncertainty controls.
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

    /// Extract parameter flags or panic with fixture context when the helper returns Unknown.
    fn decided_parameter(
        outcome: Result<Parameter01, UnknownDecision>,
        message: &str,
    ) -> Parameter01 {
        match outcome {
            Ok(value) => value,
            Err(_) => panic!("{message}"),
        }
    }

    /// Check planar interior, endpoint, collinear-exterior, and off-line values on an integer segment.
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

    /// Compare complete direct, retained-line, and retained-query outcomes for four rational queries.
    ///
    /// Both policies exercise interior, endpoint, outside, and off-line cases, including
    /// Boolean wrappers. Agreement does not independently measure dispatch counts.
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
                let query_evidence = crate::line2_orientation_query(query);
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
                    classify_point_segment_with_orientation_and_query_and_policy(
                        &a,
                        &b,
                        query,
                        &orientation,
                        &query_evidence,
                        policy,
                    ),
                    direct,
                );
                assert_eq!(
                    crate::point_on_segment_with_orientation(&a, &b, query, &orientation, policy,),
                    crate::point_on_segment(&a, &b, query, policy),
                );
                assert_eq!(
                    point_on_segment_with_orientation_and_query_and_policy(
                        &a,
                        &b,
                        query,
                        &orientation,
                        &query_evidence,
                        policy,
                    ),
                    crate::point_on_segment(&a, &b, query, policy),
                );
            }
        }
    }

    /// Check spatial location values, Boolean membership, and collapsed endpoint controls.
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

        assert_eq!(
            crate::classify_point_segment3(&a, &a, &a, APPROX).value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            crate::classify_point_segment3(&a, &a, &p3(1, 0, 0), APPROX).value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
    }

    /// Check planar proper crossing, endpoint touch, partial collinear overlap, and reversed identity.
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

    /// Check a rational crossing, parallel rejection, and supporting-line crossing outside a segment.
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

    /// Check symbolic positive-extent construction and strict rejection of an unresolved zero extent.
    ///
    /// Both strict constructor entrypoints must produce the expected x and zero y;
    /// the classification control checks a Proper value. No dispatch count is asserted.
    #[test]
    fn line_intersection_point_reuses_policy_nonzero_determinant() {
        let extent = exact_normal_positive();
        let half = (Real::from(1) / Real::from(2)).unwrap();
        let crossing_x = extent.clone() * &half;
        let a = Point2::new(Real::zero(), Real::zero());
        let b = Point2::new(extent.clone(), Real::zero());
        let c = Point2::new(crossing_x.clone(), Real::from(-1));
        let d = Point2::new(crossing_x.clone(), Real::from(1));
        assert_eq!(
            classify_segment_intersection_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT,)
                .value(),
            Some(SegmentIntersection::Proper)
        );

        for point in [
            construct_line_intersection_point_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT),
            construct_line_intersection_point(&a, &b, &c, &d),
        ] {
            let point = point.expect("the certified determinant should construct the crossing");
            assert_eq!(
                compare_reals_with_policy(&point.x, &crossing_x, PredicatePolicy::STRICT).value(),
                Some(Ordering::Equal)
            );
            assert_eq!(point.y, Real::zero());
        }

        let unresolved_extent = terminal_zero();
        let unresolved_x = unresolved_extent.clone() * &half;
        assert!(
            construct_line_intersection_point_with_policy(
                &Point2::new(Real::zero(), Real::zero()),
                &Point2::new(unresolved_extent, Real::zero()),
                &Point2::new(unresolved_x.clone(), Real::from(-1)),
                &Point2::new(unresolved_x, Real::from(1)),
                PredicatePolicy::STRICT,
            )
            .is_none()
        );
    }

    /// Require small dyadic-kernel decline and the exact midpoint of a large-integer crossing.
    ///
    /// Extent is `i64::MAX^2`; the final constructor result is checked, without an
    /// assertion selecting its successful dispatch path.
    #[test]
    fn line_intersection_point_uses_wide_dyadic_carrier() {
        let word = hyperreal::Rational::new(i64::MAX);
        let extent = &word * &word;
        let extent_minus_one = &extent - &hyperreal::Rational::new(1);
        let extent_real = Real::from(extent.clone());
        let extent_minus_one_real = Real::from(extent_minus_one.clone());
        let zero = Real::zero();
        let first_start = Point2::new(zero.clone(), zero.clone());
        let first_end = Point2::new(extent_real.clone(), extent_minus_one_real.clone());
        let second_start = Point2::new(zero.clone(), extent_minus_one_real.clone());
        let second_end = Point2::new(extent_real.clone(), zero);

        assert!(
            Real::exact_rational_line_intersection2_point_known_dyadic(
                [&first_start.x, &first_start.y],
                [&first_end.x, &first_end.y],
                [&second_start.x, &second_start.y],
                [&second_end.x, &second_end.y],
            )
            .is_none()
        );
        assert_eq!(
            construct_line_intersection_point(&first_start, &first_end, &second_start, &second_end,),
            Some(Point2::new(
                (&extent_real / &Real::from(2)).unwrap(),
                (&extent_minus_one_real / &Real::from(2)).unwrap(),
            ))
        );
    }

    /// Require the exact `(1/3, 1/3)` crossing from non-dyadic endpoints; no dispatch stage is asserted.
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

    /// Check planar disjoint values for collinear and parallel separated segments.
    ///
    /// Despite the historical name, these are 2D fixtures and do not exercise skew 3D lines.
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

    /// Check matching and distinct point queries using facts derived from a collapsed planar segment.
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

    /// Check collapsed point-on-segment, disjoint point pairs, and identical points with matching facts.
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

    /// Check noncollapsed facts and a collapsed-second-operand control; no cache-hit count is asserted.
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

    /// Check spatial interior membership and an off-line Boolean query using borrowed endpoints.
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

    /// Check spatial Proper, EndpointTouch, CoplanarDisjoint, and SkewDisjoint values.
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

    /// Check spatial collinear overlap, reversed identity, separation, and collapsed-operand relations.
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

        let point = p3(2, 2, 2);
        assert_eq!(
            crate::classify_segment3_intersection(&point, &point, &point, &point, APPROX).value(),
            Some(Segment3Intersection::Identical)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &point,
                &point,
                &p3(9, 9, 9),
                &p3(9, 9, 9),
                APPROX,
            )
            .value(),
            Some(Segment3Intersection::CoplanarDisjoint)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &point,
                &point,
                &p3(0, 0, 0),
                &p3(4, 4, 4),
                APPROX,
            )
            .value(),
            Some(Segment3Intersection::EndpointTouch)
        );
        assert_eq!(
            crate::classify_segment3_intersection(
                &p3(0, 0, 0),
                &p3(4, 4, 4),
                &point,
                &point,
                APPROX,
            )
            .value(),
            Some(Segment3Intersection::EndpointTouch)
        );
    }

    /// Require a Proper value through the public spatial segment wrapper on an integer crossing.
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

    /// Require Unknown for selected orientation, coordinate, collapse, and retained-evidence queries.
    ///
    /// Exercises planar/spatial and Boolean wrappers; assertions leave uncertainty
    /// reason and stage unconstrained.
    #[test]
    fn strict_point_segment_apis_propagate_each_unresolved_decision_class() {
        let origin2 = p2(0, 0);
        let x2 = p2(4, 0);
        let unresolved_x2 = Point2::new(terminal_zero(), real(0));
        let unresolved_y2 = Point2::new(real(2), terminal_zero());

        assert!(matches!(
            classify_point_segment_with_policy(
                &origin2,
                &unresolved_x2,
                &origin2,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_segment_with_policy(
                &origin2,
                &x2,
                &unresolved_y2,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_collinear_point_segment_with_policy(
                &origin2,
                &x2,
                &unresolved_x2,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_on_segment_with_policy(&origin2, &x2, &unresolved_y2, PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));

        let orientation = crate::line2_orientation(&origin2, &x2);
        let query = crate::line2_orientation_query(&unresolved_y2);
        assert!(matches!(
            point_on_segment_with_orientation_and_policy(
                &origin2,
                &x2,
                &unresolved_y2,
                &orientation,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_on_segment_with_orientation_and_query_and_policy(
                &origin2,
                &x2,
                &unresolved_y2,
                &orientation,
                &query,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let degenerate_facts = crate::geometry::segment2_facts(&origin2, &origin2);
        assert!(matches!(
            classify_point_segment_with_policy_and_facts(
                &origin2,
                &origin2,
                &unresolved_x2,
                degenerate_facts,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_on_segment_with_policy_and_facts(
                &origin2,
                &origin2,
                &unresolved_x2,
                degenerate_facts,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let origin3 = p3(0, 0, 0);
        let x3 = p3(4, 0, 0);
        let unresolved_x3 = Point3::new(terminal_zero(), real(0), real(0));
        let unresolved_y3 = Point3::new(real(2), terminal_zero(), real(0));
        assert!(matches!(
            classify_point_segment3_with_policy(
                &origin3,
                &unresolved_x3,
                &origin3,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_segment3_with_policy(
                &origin3,
                &origin3,
                &unresolved_x3,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_segment3_with_policy(
                &origin3,
                &x3,
                &unresolved_y3,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_point_segment3_with_policy(
                &origin3,
                &x3,
                &unresolved_x3,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_on_segment3_with_policy(&origin3, &x3, &unresolved_y3, PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Require Unknown for selected spatial collapse, cross-product, parallel, and coplanarity controls.
    ///
    /// Two decided controls check that supporting-line crossings outside finite
    /// segments are disjoint in 2D and coplanar-disjoint in 3D.
    #[test]
    fn strict_segment_intersection_propagates_unresolved_geometry() {
        let origin = p3(0, 0, 0);

        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &origin,
                &Point3::new(terminal_zero(), real(0), real(0)),
                &p3(0, 1, 0),
                &p3(0, 2, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let unresolved_point = Point3::new(terminal_zero(), real(0), real(0));
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &origin,
                &origin,
                &unresolved_point,
                &unresolved_point,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let cross_unknown = Point3::new(real(0), terminal_zero(), real(1));
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &origin,
                &p3(1, 0, 0),
                &origin,
                &cross_unknown,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let shifted_start = Point3::new(real(0), terminal_zero(), real(0));
        let shifted_end = Point3::new(real(1), terminal_zero(), real(0));
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &origin,
                &p3(1, 0, 0),
                &shifted_start,
                &shifted_end,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let elevated_start = Point3::new(real(0), real(0), terminal_zero());
        let elevated_end = Point3::new(real(0), real(1), terminal_zero());
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &origin,
                &p3(1, 0, 0),
                &elevated_start,
                &elevated_end,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(
            classify_segment3_intersection_with_policy(
                &origin,
                &p3(1, 0, 0),
                &p3(2, -1, 0),
                &p3(2, 1, 0),
                APPROX,
            )
            .value(),
            Some(Segment3Intersection::CoplanarDisjoint)
        );
        assert_eq!(
            classify_segment_intersection_with_policy(
                &p2(0, 0),
                &p2(1, 0),
                &p2(2, -1),
                &p2(2, 1),
                APPROX,
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
    }

    /// Check selected collinear/fact uncertainties and relation-packaging helper outputs.
    ///
    /// Unknown and decided wrappers are tested without independently inspecting every
    /// receipt field.
    #[test]
    fn strict_planar_segment_intersection_propagates_collinear_and_fact_uncertainty() {
        let unresolved = Point2::new(terminal_zero(), real(0));
        assert!(matches!(
            classify_segment_intersection_with_policy(
                &p2(0, 0),
                &p2(4, 0),
                &unresolved,
                &p2(6, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let origin = p2(0, 0);
        let unresolved_point = Point2::new(terminal_zero(), real(0));
        let origin_facts = crate::geometry::segment2_facts(&origin, &origin);
        let unresolved_facts =
            crate::geometry::segment2_facts(&unresolved_point, &unresolved_point);
        assert!(matches!(
            classify_segment_intersection_with_policy_and_facts(
                &origin,
                &origin,
                &unresolved_point,
                &unresolved_point,
                origin_facts,
                unresolved_facts,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            point_segment_intersection_from_classifier(PredicateOutcome::unknown(
                RefinementNeed::RealRefinement,
                Escalation::Undecided,
            )),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            point_segment3_intersection_from_classifier(PredicateOutcome::unknown(
                RefinementNeed::RealRefinement,
                Escalation::Undecided,
            )),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(
            point_segment_intersection_from_classifier(PredicateOutcome::decided(
                PointSegmentLocation::CollinearOutside,
                Certainty::Exact,
                Escalation::Exact,
            ))
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            point_segment3_intersection_from_classifier(PredicateOutcome::decided(
                PointSegmentLocation::CollinearOutside,
                Certainty::Exact,
                Escalation::Exact,
            ))
            .value(),
            Some(Segment3Intersection::CoplanarDisjoint)
        );

        let trace = DecisionTrace::default();
        assert_eq!(
            finish_planar_endpoint_touch(Ok(true), trace).value(),
            Some(SegmentIntersection::EndpointTouch)
        );
        assert_eq!(
            finish_planar_endpoint_touch(Ok(false), trace).value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert!(matches!(
            finish_planar_endpoint_touch(
                Err(UnknownDecision {
                    needed: RefinementNeed::RealRefinement,
                    stage: Escalation::Undecided,
                }),
                trace,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Require Unknown for selected later orientation, normal, collinear, and parameter decisions.
    ///
    /// Despite the name, assertions do not constrain the reported escalation stage.
    #[test]
    fn strict_segment_classifiers_propagate_each_late_uncertainty_stage() {
        let unresolved_d = Point2::new(terminal_zero(), real(1));
        assert!(matches!(
            classify_segment_intersection_with_policy(
                &p2(0, 0),
                &p2(4, 0),
                &p2(0, 1),
                &unresolved_d,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let normal_unknown_end = Point3::new(real(0), real(1), &Real::from(1) + &terminal_zero());
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(0, 0, 1),
                &normal_unknown_end,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &Point3::new(terminal_zero(), 0.into(), 0.into()),
                &p3(6, 0, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let t_unknown = terminal_zero();
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &Point3::new(t_unknown.clone(), (-1).into(), 0.into()),
                &Point3::new(t_unknown, 1.into(), 0.into()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        let u_unknown = terminal_zero();
        assert!(matches!(
            classify_segment3_intersection_with_policy(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &Point3::new(2.into(), u_unknown, 0.into()),
                &p3(2, 1, 0),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Check partial overlap in 2D/3D and one 3D endpoint touch using the internal collinear helpers.
    ///
    /// These are three fixed arrangements, not every endpoint permutation.
    #[test]
    fn collinear_overlap_collection_covers_each_endpoint_order() {
        let mut trace = DecisionTrace::default();
        assert_eq!(
            classify_collinear_segments(
                &p2(0, 0),
                &p2(4, 0),
                &p2(-1, 0),
                &p2(2, 0),
                APPROX,
                &mut trace,
            )
            .ok(),
            Some(SegmentIntersection::CollinearOverlap)
        );

        let mut trace = DecisionTrace::default();
        assert_eq!(
            classify_collinear_segments3(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(-1, 0, 0),
                &p3(2, 0, 0),
                APPROX,
                &mut trace,
            )
            .ok(),
            Some(Segment3Intersection::CollinearOverlap)
        );

        let mut trace = DecisionTrace::default();
        assert_eq!(
            classify_collinear_segments3(
                &p3(0, 0, 0),
                &p3(4, 0, 0),
                &p3(4, 0, 0),
                &p3(6, 0, 0),
                APPROX,
                &mut trace,
            )
            .ok(),
            Some(Segment3Intersection::EndpointTouch)
        );
    }

    /// Check symbolic crossing at `(pi, 1)`, four ratio cases, and the three vector coordinates.
    ///
    /// Ratio controls include below/above, negative-denominator interior, and zero
    /// boundary; they do not exercise zero denominators.
    #[test]
    fn line_constructor_and_parameter_helpers_cover_general_real_and_bounds() {
        let pi = Real::pi();
        let intersection = construct_line_intersection_point(
            &Point2::new(pi.clone(), real(0)),
            &Point2::new(pi.clone(), real(2)),
            &Point2::new(real(0), real(1)),
            &Point2::new(&pi * &Real::from(2), real(1)),
        )
        .expect("nonparallel symbolic supporting lines intersect");
        assert_eq!(intersection, Point2::new(pi, real(1)));

        let mut trace = DecisionTrace::default();
        let below = decided_parameter(
            classify_parameter_01(&real(-1), &real(2), APPROX, &mut trace),
            "rational parameter should decide",
        );
        assert!(!below.on_segment);
        let above = decided_parameter(
            classify_parameter_01(&real(3), &real(2), APPROX, &mut trace),
            "rational parameter should decide",
        );
        assert!(!above.on_segment);
        let reversed = decided_parameter(
            classify_parameter_01(&real(-1), &real(-2), APPROX, &mut trace),
            "negative denominator should be normalized",
        );
        assert!(reversed.on_segment);
        assert!(!reversed.on_boundary);
        let boundary = decided_parameter(
            classify_parameter_01(&real(0), &real(2), APPROX, &mut trace),
            "zero parameter should decide",
        );
        assert!(boundary.on_segment);
        assert!(boundary.on_boundary);

        let vector = Vector3Real {
            x: real(1),
            y: real(2),
            z: real(3),
        };
        assert_eq!(coordinate(&vector, 0), &real(1));
        assert_eq!(coordinate(&vector, 1), &real(2));
        assert_eq!(coordinate(&vector, 2), &real(3));
    }

    /// Check duplicate suppression, strict opposite signs, certainty/stage rankings, and Unknown forwarding.
    #[test]
    fn segment_decision_helpers_cover_uniqueness_signs_and_trace_ranks() {
        let a = p2(0, 0);
        let b = p2(1, 0);
        let mut points = Vec::new();
        let mut trace = DecisionTrace::default();
        assert!(push_unique_point(&mut points, &a, APPROX, &mut trace).is_ok());
        assert!(push_unique_point(&mut points, &a, APPROX, &mut trace).is_ok());
        assert_eq!(points.len(), 1);

        let a3 = p3(0, 0, 0);
        let mut points3 = Vec::new();
        assert!(push_unique_point3(&mut points3, &a3, APPROX, &mut trace).is_ok());
        assert!(push_unique_point3(&mut points3, &a3, APPROX, &mut trace).is_ok());
        assert_eq!(points3.len(), 1);

        assert!(opposite_strict(Sign::Positive, Sign::Negative));
        assert!(opposite_strict(Sign::Negative, Sign::Positive));
        assert!(!opposite_strict(Sign::Zero, Sign::Positive));
        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(certainty_rank(Certainty::Exact), 0);
        assert_eq!(certainty_rank(Certainty::Filtered), 1);
        assert_eq!(certainty_rank(Certainty::Approximate), 2);
        assert_eq!(
            max_stage(Escalation::Filter, Escalation::Exact),
            Escalation::Exact
        );
        assert_eq!(stage_rank(Escalation::Structural), 0);
        assert_eq!(stage_rank(Escalation::Filter), 1);
        assert_eq!(stage_rank(Escalation::Exact), 2);
        assert_eq!(stage_rank(Escalation::Refined), 3);
        assert_eq!(stage_rank(Escalation::Undecided), 4);

        let unknown = decided::<Sign>(
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided),
            &mut trace,
        )
        .expect_err("unknown predicate outcomes must propagate");
        assert!(matches!(
            unknown.into_outcome::<()>(),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(b, p2(1, 0));
    }

    /// Check caller-established zero orientations bypass unresolved determinant expressions.
    ///
    /// Strict policy must recompute disallowed approximate or Unknown outcomes, while
    /// approximate policy must retain the supplied approximate receipt. Also checks
    /// all-collinear segment relations, partial-retention uncertainty, and a fresh
    /// proper crossing. Supplied signs are justified by fixture identities; the API
    /// does not validate their source association.
    #[test]
    fn retained_collinearity_decides_only_the_position_along_the_line() {
        // The fixture identity puts the point on the line, while the current
        // strict determinant query leaves four times that zero unresolved.
        let a = p2(0, 0);
        let b = p2(4, 4);
        let inside = Point2::new(real(2), real(2) + terminal_zero());
        let outside = Point2::new(real(6), real(6) + terminal_zero());
        let off = p2(2, 3);
        let zero = PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Structural);
        let strict = PredicatePolicy::STRICT;
        assert!(matches!(
            classify_point_segment_with_policy(&a, &b, &inside, strict),
            PredicateOutcome::Unknown { .. }
        ));
        for (point, expected) in [
            (&inside, PointSegmentLocation::OnSegment),
            (&outside, PointSegmentLocation::CollinearOutside),
        ] {
            assert_eq!(
                classify_point_segment_with_known_orientation_and_policy(
                    &a, &b, point, zero, strict
                )
                .value(),
                Some(expected)
            );
        }
        // A nonzero retained sign is off the line; an approximate one is not
        // a strict proof, and an undecided one is replaced by the determinant.
        let positive =
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Structural);
        assert_eq!(
            classify_point_segment_with_known_orientation_and_policy(
                &a, &b, &off, positive, strict
            )
            .value(),
            Some(PointSegmentLocation::OffLine)
        );
        let approximate =
            PredicateOutcome::decided(Sign::Zero, Certainty::Approximate, Escalation::Undecided);
        let unknown =
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided);
        for retained in [approximate, unknown] {
            assert!(matches!(
                classify_point_segment_with_known_orientation_and_policy(
                    &a, &b, &inside, retained, strict
                ),
                PredicateOutcome::Unknown { .. }
            ));
            assert_eq!(
                classify_point_segment_with_known_orientation_and_policy(
                    &a, &b, &off, retained, strict
                )
                .value(),
                Some(PointSegmentLocation::OffLine)
            );
        }
        assert_eq!(
            classify_point_segment_with_known_orientation_and_policy(
                &a,
                &b,
                &inside,
                approximate,
                APPROX
            ),
            PredicateOutcome::decided(
                PointSegmentLocation::OnSegment,
                Certainty::Approximate,
                Escalation::Undecided
            )
        );

        // Segments: `cd` lies beyond `ab` on the same line, and `ed` overlaps
        // it, which only the retained orientations of their endpoints
        // establish.
        let c = Point2::new(real(5), real(5) + terminal_zero());
        let d = Point2::new(real(8), real(8) + terminal_zero());
        let e = Point2::new(real(1), real(1) + terminal_zero());
        let retained = |known: [bool; 4]| known.map(|known| if known { zero } else { unknown });
        assert!(matches!(
            classify_segment_intersection_with_policy(&a, &b, &c, &d, strict),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            classify_segment_intersection_with_known_orientations_and_policy(
                &a,
                &b,
                &c,
                &d,
                retained([true; 4]),
                strict
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            classify_segment_intersection_with_known_orientations_and_policy(
                &a,
                &b,
                &e,
                &d,
                retained([true; 4]),
                strict
            )
            .value(),
            Some(SegmentIntersection::CollinearOverlap)
        );
        // An unretained orientation is still evaluated, and here undecided.
        assert!(matches!(
            classify_segment_intersection_with_known_orientations_and_policy(
                &a,
                &b,
                &c,
                &d,
                retained([true, false, true, true]),
                strict
            ),
            PredicateOutcome::Unknown { .. }
        ));
        // A segment crossing `ab` keeps the ordinary classification.
        assert_eq!(
            classify_segment_intersection_with_known_orientations_and_policy(
                &a,
                &b,
                &p2(0, 4),
                &p2(4, 0),
                retained([false; 4]),
                strict
            )
            .value(),
            Some(SegmentIntersection::Proper)
        );
    }
}
