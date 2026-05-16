//! Segment incidence and intersection classifiers.
//!
//! The algorithms here use only orientation signs and exact interval tests.
//! This keeps segment topology in `hyperlimit` while leaving segment storage,
//! DCELs, rings, and sweep state to higher crates such as `hypercurve` and
//! `hypertri`.

use crate::classify::{PointSegmentLocation, SegmentIntersection};
use crate::geometry::{Point2, Segment2Facts};
use crate::predicate::{
    Certainty, Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed, Sign,
};
use crate::predicates::orient::orient2d_with_policy;
use crate::real::sub_ref;
use crate::resolve::resolve_real_sign;
use hyperreal::Real;

/// Reusable exact predicates for one closed 2D segment.
///
/// A prepared segment stores borrowed endpoints plus [`Segment2Facts`]. It is a
/// predicate helper, not segment topology: ownership of edge ids, constraints,
/// rings, and DCEL handles remains in higher crates. The prepared form follows
/// Yap's exact-geometric-computation guidance to retain geometric-object facts
/// across repeated predicates; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug)]
pub struct PreparedSegment2<'a> {
    start: &'a Point2,
    end: &'a Point2,
    facts: Segment2Facts,
}

impl<'a> PreparedSegment2<'a> {
    /// Prepare a segment and compute its structural facts.
    pub fn new(start: &'a Point2, end: &'a Point2) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_segment2", "new");
        Self::from_facts(start, end, crate::geometry::segment2_facts(start, end))
    }

    /// Prepare a segment from caller-cached structural facts.
    ///
    /// The caller must pass facts for the same endpoint pair. Conservative facts
    /// merely leave fast paths unused, but non-conservative facts can change
    /// which exact branch is evaluated.
    pub const fn from_facts(start: &'a Point2, end: &'a Point2, facts: Segment2Facts) -> Self {
        Self { start, end, facts }
    }

    /// Return the segment start endpoint.
    pub const fn start(&self) -> &'a Point2 {
        self.start
    }

    /// Return the segment end endpoint.
    pub const fn end(&self) -> &'a Point2 {
        self.end
    }

    /// Return cached structural facts for this segment.
    pub const fn facts(&self) -> Segment2Facts {
        self.facts
    }

    /// Classify a point relative to this segment using the default policy.
    pub fn classify_point(&self, point: &Point2) -> PredicateOutcome<PointSegmentLocation> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point relative to this segment using an explicit policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<PointSegmentLocation> {
        classify_point_segment_with_policy_and_facts(
            self.start, self.end, point, policy, self.facts,
        )
    }

    /// Return whether a point lies on this segment using the default policy.
    pub fn point_on_segment(&self, point: &Point2) -> PredicateOutcome<bool> {
        self.point_on_segment_with_policy(point, PredicatePolicy::default())
    }

    /// Return whether a point lies on this segment using an explicit policy.
    pub fn point_on_segment_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<bool> {
        point_on_segment_with_policy_and_facts(self.start, self.end, point, policy, self.facts)
    }

    /// Classify this segment's intersection with another prepared segment using
    /// the default policy.
    pub fn classify_intersection(
        &self,
        other: &PreparedSegment2,
    ) -> PredicateOutcome<SegmentIntersection> {
        self.classify_intersection_with_policy(other, PredicatePolicy::default())
    }

    /// Classify this segment's intersection with another prepared segment using
    /// an explicit policy.
    ///
    /// Degenerate point-segment cases use cached facts before falling back to
    /// the standard four-orientation classifier from de Berg, Cheong, van
    /// Kreveld, and Overmars, *Computational Geometry: Algorithms and
    /// Applications*, 3rd ed., Springer, 2008. Every equality or containment
    /// result is still certified through exact Real predicates.
    pub fn classify_intersection_with_policy(
        &self,
        other: &PreparedSegment2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<SegmentIntersection> {
        classify_segment_intersection_with_policy_and_facts(
            self.start,
            self.end,
            other.start,
            other.end,
            policy,
            self.facts,
            other.facts,
        )
    }
}

/// Classify `point` relative to the closed segment `ab`.
pub fn classify_point_segment(
    a: &Point2,
    b: &Point2,
    point: &Point2,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_with_policy(a, b, point, PredicatePolicy::default())
}

/// Classify `point` relative to the closed segment `ab` with an explicit
/// predicate escalation policy.
pub fn classify_point_segment_with_policy(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, None)
}

/// Classify `point` relative to the closed segment `ab` using cached segment
/// structural facts.
pub fn classify_point_segment_with_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    segment_facts: Segment2Facts,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_with_policy_and_facts(
        a,
        b,
        point,
        PredicatePolicy::default(),
        segment_facts,
    )
}

/// Classify `point` relative to the closed segment `ab` with both an explicit
/// policy and cached segment structural facts.
///
/// The facts are advisory exact metadata. They can skip the orientation
/// determinant for a structurally degenerate segment, but the point equality
/// decision still goes through exact Real predicates. This preserves the
/// exact-geometric-computation boundary described by Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), while
/// retaining object facts in the sense used by de Berg, Cheong, van Kreveld,
/// and Overmars for degeneracy-aware geometric algorithms in *Computational
/// Geometry: Algorithms and Applications*, 3rd ed., Springer, 2008.
pub fn classify_point_segment_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    segment_facts: Segment2Facts,
) -> PredicateOutcome<PointSegmentLocation> {
    classify_point_segment_impl(a, b, point, policy, Some(segment_facts))
}

fn classify_point_segment_impl(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    segment_facts: Option<Segment2Facts>,
) -> PredicateOutcome<PointSegmentLocation> {
    let mut trace = DecisionTrace::default();

    if segment_facts.and_then(Segment2Facts::known_degenerate) == Some(true) {
        return match classify_degenerate_point_segment(a, point, policy, &mut trace) {
            Ok(location) => PredicateOutcome::decided(location, trace.certainty, trace.stage),
            Err(unknown) => unknown.into_outcome(),
        };
    }

    let orientation = match decided(orient2d_with_policy(a, b, point, policy), &mut trace) {
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

/// Return whether `point` lies on the closed segment `ab`.
pub fn point_on_segment(a: &Point2, b: &Point2, point: &Point2) -> PredicateOutcome<bool> {
    point_on_segment_with_policy(a, b, point, PredicatePolicy::default())
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

/// Return whether `point` lies on the closed segment `ab` using cached segment
/// structural facts.
pub fn point_on_segment_with_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    segment_facts: Segment2Facts,
) -> PredicateOutcome<bool> {
    point_on_segment_with_policy_and_facts(a, b, point, PredicatePolicy::default(), segment_facts)
}

/// Return whether `point` lies on the closed segment `ab` with both an explicit
/// policy and cached segment structural facts.
pub fn point_on_segment_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
    segment_facts: Segment2Facts,
) -> PredicateOutcome<bool> {
    match classify_point_segment_with_policy_and_facts(a, b, point, policy, segment_facts) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value.is_on_segment(), certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Classify the intersection of closed segments `ab` and `cd`.
pub fn classify_segment_intersection(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_with_policy(a, b, c, d, PredicatePolicy::default())
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

/// Classify the intersection of closed segments `ab` and `cd` using cached
/// structural facts for both segments.
pub fn classify_segment_intersection_with_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    first_facts: Segment2Facts,
    second_facts: Segment2Facts,
) -> PredicateOutcome<SegmentIntersection> {
    classify_segment_intersection_with_policy_and_facts(
        a,
        b,
        c,
        d,
        PredicatePolicy::default(),
        first_facts,
        second_facts,
    )
}

/// Classify the intersection of closed segments `ab` and `cd` with both an
/// explicit policy and cached structural facts for both segments.
///
/// Known-degenerate facts let this function reduce point-segment or point-point
/// cases before evaluating the four-orientation classifier. The reduction never
/// accepts lossy coordinates: every remaining equality or containment question
/// is certified by exact Real predicates. This follows Yap's exact
/// computation boundary and the degeneracy handling discipline in de Berg,
/// Cheong, van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed., Springer, 2008.
pub fn classify_segment_intersection_with_policy_and_facts(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
    first_facts: Segment2Facts,
    second_facts: Segment2Facts,
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

    // This is the standard four-orientation segment classifier described in
    // de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
    // Algorithms and Applications*, 3rd ed., Springer, 2008. The difference in
    // this crate is numerical: every orientation and interval comparison routes
    // through exact hyperreal-backed signs, following the exact-geometric
    // computation discipline of Yap, "Towards Exact Geometric Computation,"
    // *Computational Geometry* 7.1-2 (1997), and the determinant-sign focus of
    // Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust
    // Geometric Predicates," *Discrete & Computational Geometry* 18.3 (1997).
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
            classify_point_segment_impl(c, d, a, policy, second_facts),
        )),
        (_, Some(true)) => Some(point_segment_intersection_from_classifier(
            classify_point_segment_impl(a, b, c, policy, first_facts),
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

fn sign_of_difference(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
    trace: &mut DecisionTrace,
) -> Result<Sign, UnknownDecision> {
    let diff = sub_ref(left, right);
    decided(
        resolve_real_sign(
            &diff,
            policy,
            || None,
            || None,
            RefinementNeed::RealRefinement,
        ),
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

    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(real(x), real(y))
    }

    #[test]
    fn point_segment_classifier_distinguishes_endpoint_inside_and_outside() {
        let a = p2(0, 0);
        let b = p2(4, 0);

        assert_eq!(
            classify_point_segment(&a, &b, &p2(2, 0)).value(),
            Some(PointSegmentLocation::OnSegment)
        );
        assert_eq!(
            classify_point_segment(&a, &b, &p2(4, 0)).value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            classify_point_segment(&a, &b, &p2(5, 0)).value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
        assert_eq!(
            classify_point_segment(&a, &b, &p2(2, 1)).value(),
            Some(PointSegmentLocation::OffLine)
        );
    }

    #[test]
    fn segment_classifier_reports_proper_endpoint_overlap_and_identical() {
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 4), &p2(0, 4), &p2(4, 0)).value(),
            Some(SegmentIntersection::Proper)
        );
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 0), &p2(4, 0), &p2(6, 0)).value(),
            Some(SegmentIntersection::EndpointTouch)
        );
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 0), &p2(2, 0), &p2(6, 0)).value(),
            Some(SegmentIntersection::CollinearOverlap)
        );
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 0), &p2(4, 0), &p2(0, 0)).value(),
            Some(SegmentIntersection::Identical)
        );
    }

    #[test]
    fn segment_classifier_reports_disjoint_collinear_and_skew_cases() {
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 0), &p2(5, 0), &p2(6, 0)).value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            classify_segment_intersection(&p2(0, 0), &p2(4, 0), &p2(5, 1), &p2(6, 1)).value(),
            Some(SegmentIntersection::Disjoint)
        );
    }

    #[test]
    fn fact_aware_point_segment_classifier_handles_degenerate_segments() {
        let endpoint = p2(2, 3);
        let facts = crate::geometry::segment2_facts(&endpoint, &endpoint);

        assert_eq!(
            classify_point_segment_with_facts(&endpoint, &endpoint, &endpoint, facts).value(),
            Some(PointSegmentLocation::OnEndpoint)
        );
        assert_eq!(
            classify_point_segment_with_facts(&endpoint, &endpoint, &p2(2, 4), facts).value(),
            Some(PointSegmentLocation::CollinearOutside)
        );
        assert_eq!(
            point_on_segment_with_facts(&endpoint, &endpoint, &endpoint, facts).value(),
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
            classify_segment_intersection_with_facts(
                &point,
                &point,
                &start,
                &end,
                point_facts,
                segment_facts
            )
            .value(),
            Some(SegmentIntersection::EndpointTouch)
        );

        let other_point = p2(9, 0);
        let other_facts = crate::geometry::segment2_facts(&other_point, &other_point);
        assert_eq!(
            classify_segment_intersection_with_facts(
                &point,
                &point,
                &other_point,
                &other_point,
                point_facts,
                other_facts
            )
            .value(),
            Some(SegmentIntersection::Disjoint)
        );
        assert_eq!(
            classify_segment_intersection_with_facts(
                &point,
                &point,
                &point,
                &point,
                point_facts,
                point_facts
            )
            .value(),
            Some(SegmentIntersection::Identical)
        );
    }

    #[test]
    fn prepared_segment_reuses_cached_facts_for_point_and_intersection_queries() {
        let a = p2(0, 0);
        let b = p2(4, 0);
        let prepared = PreparedSegment2::new(&a, &b);
        assert_eq!(prepared.facts().known_degenerate(), Some(false));
        assert_eq!(
            prepared.classify_point(&p2(2, 0)).value(),
            Some(PointSegmentLocation::OnSegment)
        );

        let point = p2(2, 0);
        let prepared_point = PreparedSegment2::new(&point, &point);
        assert_eq!(
            prepared.classify_intersection(&prepared_point).value(),
            Some(SegmentIntersection::EndpointTouch)
        );
    }
}
