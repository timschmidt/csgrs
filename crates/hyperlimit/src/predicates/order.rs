//! Policy-controlled ordering, equality and selection over exact Real values.
//!
//! These helpers are predicates rather than algebra: ordering is a decision
//! about scalar values, so its local certainty belongs in
//! [`PredicateOutcome`](crate::PredicateOutcome). Approximate terminals retain
//! their qualification; a later exact comparison does not reconstruct source
//! provenance lost before these calls.
//!
//! Coordinate equality searches for a sufficient unequal-coordinate witness;
//! lexicographic order instead requires earlier coordinates to compare Equal
//! before inspecting later ones. Reference selection and reciprocal/clamp
//! construction retain decision receipts where their return types allow it.

use crate::predicate::PredicatePolicy;
use core::cmp::Ordering;

use crate::geometry::{Point2, Point3};
use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::real::sub_ref;
use crate::resolve::{map_outcome, resolve_real_sign_direct};
use hyperreal::{CertifiedRealOrdering, Problem, Rational, Real, RealOrderingCertificate};

/// Decide the sign of one Real value with an explicit predicate policy.
///
/// Delegates to the direct scalar resolver with RealRefinement as its unresolved
/// hint. Decided may be Approximate; Zero with that label is not a zero proof.
pub fn classify_real_sign_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    resolve_real_sign_direct(value, policy, RefinementNeed::RealRefinement)
}

/// Construct a reciprocal after deciding nonzero status through the supplied
/// predicate policy.
///
/// The returned outcome retains the certainty and escalation stage of the
/// denominator decision. This is the construction counterpart to
/// [`classify_real_sign_with_policy`]: downstream exact geometry can reuse one policy
/// decision instead of asking `Real` division to repeat the same potentially
/// expensive refinement for every coordinate.
///
/// Unknown becomes `Ok(Unknown)` without constructing an inverse. A decided
/// nonzero sign permits the scalar's assuming-nonzero constructor and preserves
/// its receipt on success; any scalar construction error propagates.
///
/// A decided Zero returns `Problem::DivideByZero` even when the sign is
/// Approximate. The error has no certainty receipt, so it alone does not prove
/// mathematical zero under a terminal-permitting policy.
pub fn reciprocal_real_with_policy(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<PredicateOutcome<Real>, Problem> {
    match classify_real_sign_with_policy(value, policy) {
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => Err(Problem::DivideByZero),
        PredicateOutcome::Decided {
            certainty, stage, ..
        } => Ok(PredicateOutcome::decided(
            value.inverse_ref_assuming_nonzero()?,
            certainty,
            stage,
        )),
        PredicateOutcome::Unknown { needed, stage } => Ok(PredicateOutcome::unknown(needed, stage)),
    }
}

/// Multiplies by a policy-approved reciprocal, preserving its outcome receipt.
///
/// Unknown and scalar errors propagate. Even a zero numerator requires the
/// denominator decision; this helper does not cancel an undefined quotient.
pub(crate) fn divide_real_with_policy(
    numerator: &Real,
    denominator: &Real,
    policy: PredicatePolicy,
) -> Result<PredicateOutcome<Real>, Problem> {
    Ok(map_outcome(
        reciprocal_real_with_policy(denominator, policy)?,
        |reciprocal| numerator * reciprocal,
    ))
}

/// Decides two signs, batching structurally rational inputs before scalar cascades.
///
/// Two rational coordinates return Exact/Exact. Otherwise both scalar sign
/// resolvers execute, left then right, even if the first is Unknown. Two decided
/// signs join the weakest certainty and highest-ranked stage. An unresolved
/// result forwards the first Unknown's hint/stage, without the other receipt.
pub fn classify_real_sign_pair_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<(Sign, Sign)> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("hyperlimit", "classify_real_sign_pair", "exact-rational");
        return PredicateOutcome::decided(
            (exact_rational_sign(left), exact_rational_sign(right)),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    crate::trace_dispatch!("hyperlimit", "classify_real_sign_pair", "scalar-cascades");
    match (
        classify_real_sign_with_policy(left, policy),
        classify_real_sign_with_policy(right, policy),
    ) {
        (
            PredicateOutcome::Decided {
                value: left,
                certainty: left_certainty,
                stage: left_stage,
            },
            PredicateOutcome::Decided {
                value: right,
                certainty: right_certainty,
                stage: right_stage,
            },
        ) => PredicateOutcome::decided(
            (left, right),
            max_certainty(left_certainty, right_certainty),
            max_stage(left_stage, right_stage),
        ),
        (PredicateOutcome::Unknown { needed, stage }, _)
        | (_, PredicateOutcome::Unknown { needed, stage }) => {
            PredicateOutcome::unknown(needed, stage)
        }
    }
}

/// Compare two Real values with an explicit predicate escalation policy.
///
/// This keeps Real ordering on the same exact predicate pipeline as
/// orientation and incidence tests. Higher crates use it for leftmost-vertex
/// selection, ray-crossing tests, interval comparisons, and deterministic tie
/// breaking without importing primitive-float ordering into topology code.
/// Numerical structure may be carried by Real objects, while geometric
/// decisions ask a predicate layer to certify signs.
///
/// Two structurally rational operands compare directly with Exact/Exact receipt.
/// Otherwise tries the scalar's certified comparison down to precision -512;
/// only an unresolved comparison constructs a difference for the policy sign
/// resolver. The precision threshold is not a total-work or memory bound.
/// Success preserves certificate-derived stage, or the fallback qualification;
/// approximate Equal is not a proof of equality. Unknown remains explicit.
#[inline]
pub fn compare_reals_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("hyperlimit", "compare_reals", "exact-rational");
        return PredicateOutcome::decided(
            left.partial_cmp(right)
                .expect("exact rational ordering is total"),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    match left.certified_cmp_until(right, PredicatePolicy::MAX_REFINEMENT_PRECISION) {
        CertifiedRealOrdering::Known {
            ordering,
            certificate,
        } => {
            let stage = ordering_certificate_stage(certificate);
            crate::trace_dispatch!("hyperlimit", "compare_reals", "certified-real-ordering");
            PredicateOutcome::decided(ordering, Certainty::Exact, stage)
        }
        CertifiedRealOrdering::Unknown { .. } => {
            let difference = sub_ref(left, right);
            crate::trace_dispatch!("hyperlimit", "compare_reals", "difference-sign-cascade");
            map_outcome(
                classify_real_sign_with_policy(&difference, policy),
                ordering_from_sign,
            )
        }
    }
}

/// Maps structural, rational and refinement ordering certificates to local stages.
/// This classification records proof kind, not chronological execution order.
#[inline(always)]
fn ordering_certificate_stage(certificate: RealOrderingCertificate) -> Escalation {
    match certificate {
        RealOrderingCertificate::StructuralEquality
        | RealOrderingCertificate::StructuralFacts
        | RealOrderingCertificate::DifferenceStructuralFacts => Escalation::Structural,
        RealOrderingCertificate::ExactRationalComparison => Escalation::Exact,
        RealOrderingCertificate::BoundedRefinement { .. } => Escalation::Refined,
    }
}

/// Converts a difference sign to Less, Equal or Greater without another decision.
#[inline(always)]
const fn ordering_from_sign(sign: Sign) -> Ordering {
    match sign {
        Sign::Negative => Ordering::Less,
        Sign::Zero => Ordering::Equal,
        Sign::Positive => Ordering::Greater,
    }
}

/// Return whether `left <= right` under the policy-controlled Real ordering predicate.
/// Retains its certainty, stage and any unresolved outcome.
pub fn real_le_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Less | Ordering::Equal)
    })
}

/// Return whether `left >= right` under the policy-controlled Real ordering predicate.
/// Retains its certainty, stage and any unresolved outcome.
pub fn real_ge_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        matches!(ordering, Ordering::Greater | Ordering::Equal)
    })
}

/// Return the smaller of two Real references using the policy-controlled ordering predicate.
///
/// Equal selects `left`; no value is reconstructed. Preserves the comparison
/// receipt, including qualification if the selected order is approximate.
pub fn real_min_with_policy<'a>(
    left: &'a Real,
    right: &'a Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<&'a Real> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        if ordering == Ordering::Greater {
            right
        } else {
            left
        }
    })
}

/// Return the larger of two Real references using the policy-controlled ordering predicate.
///
/// Equal selects `left`; no value is reconstructed. Preserves the comparison
/// receipt, including qualification if the selected order is approximate.
pub fn real_max_with_policy<'a>(
    left: &'a Real,
    right: &'a Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<&'a Real> {
    map_outcome(compare_reals_with_policy(left, right, policy), |ordering| {
        if ordering == Ordering::Less {
            right
        } else {
            left
        }
    })
}

/// Clamp a Real value to an interval under an explicit predicate policy.
///
/// First compares `min` with `max`; Greater returns Unknown/Unsupported at
/// Undecided stage, rather than swapping bounds. Next compares the value with
/// `min`, returning a clone of `min` if below. Otherwise compares with `max`,
/// returning a clone of `max` if above or the original owned value if within.
/// Equality at the lower bound still proceeds to the upper comparison.
///
/// Success joins the weakest certainty and highest stage of consumed decisions.
/// The first Unknown stops evaluation and forwards its own hint/stage. Under
/// an approximate policy the selected value carries that qualification; no
/// exact interval-membership proof is added by construction.
pub fn real_clamp_with_policy(
    value: Real,
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> PredicateOutcome<Real> {
    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;

    let min_max = match compare_reals_with_policy(min, max, policy) {
        PredicateOutcome::Decided {
            value,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            value
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    if min_max == Ordering::Greater {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    }

    match compare_reals_with_policy(&value, min, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Less,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            return PredicateOutcome::decided(min.clone(), certainty, stage);
        }
        PredicateOutcome::Decided {
            certainty: value_certainty,
            stage: value_stage,
            ..
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    }

    match compare_reals_with_policy(&value, max, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Greater,
            certainty: value_certainty,
            stage: value_stage,
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            PredicateOutcome::decided(max.clone(), certainty, stage)
        }
        PredicateOutcome::Decided {
            certainty: value_certainty,
            stage: value_stage,
            ..
        } => {
            certainty = max_certainty(certainty, value_certainty);
            stage = max_stage(stage, value_stage);
            PredicateOutcome::decided(value, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

/// Compare two 2D points lexicographically by `(x, y)` with an explicit policy.
///
/// Compares x first and y only after a decided Equal x. An Unknown x stops
/// immediately even if y could prove inequality; use [`point2_equal_with_policy`]
/// for the unordered coordinate-equality question. Receipts join only consumed
/// comparisons, retaining approximate qualification. Certified results can
/// support event queues and canonicalization; no polygon or sweep topology is
/// established by this coordinate order alone.
pub fn compare_point2_lexicographic_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    match compare_reals_with_policy(&left.x, &right.x, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_with_policy(&left.y, &right.y, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            ),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        decided_or_unknown => decided_or_unknown,
    }
}

/// Return whether two 2D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality compares exact Real coordinate values under a decision policy.
/// Keeping it here avoids each arrangement, curve, or triangulation crate
/// reimplementing coordinate comparison with different uncertainty handling.
/// One certified unequal coordinate suffices, even if another is undecided.
/// Equality requires evidence for every coordinate. Lexicographic ordering
/// retains its separate, ordered-coordinate contract.
///
/// Rational x inequality returns immediately; rational equality of x followed
/// by rational y decides directly. Otherwise searches coordinates with STRICT
/// first, continuing past Unknown to seek a certified inequality witness. Only
/// an unresolved strict pass is retried under a different requested policy.
/// A terminal result preserves Approximate and must not be used as exact
/// equality or as source/construction identity.
pub fn point2_equal_with_policy(
    left: &Point2,
    right: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    if let (Some(left_x), Some(right_x)) =
        (left.x.exact_rational_ref(), right.x.exact_rational_ref())
    {
        if left_x != right_x {
            crate::trace_dispatch!("hyperlimit", "point2_equal", "exact-rational-x");
            return PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
        }
        if let (Some(left_y), Some(right_y)) =
            (left.y.exact_rational_ref(), right.y.exact_rational_ref())
        {
            crate::trace_dispatch!("hyperlimit", "point2_equal", "exact-rational-xy");
            return PredicateOutcome::decided(
                left_y == right_y,
                Certainty::Exact,
                Escalation::Exact,
            );
        }
    }

    crate::trace_dispatch!("hyperlimit", "point2_equal", "coordinate-witnesses");
    coordinates_equal(&[(&left.x, &right.x), (&left.y, &right.y)], policy)
}

/// Conjoins coordinate equalities, searching for a sufficient false witness.
///
/// Traverses pairs in order under STRICT, retaining the first Unknown while
/// continuing to later pairs. A false comparison suffices immediately and
/// supplies its own receipt, independent of earlier equal/unknown coordinates.
/// All true comparisons join their receipts; an empty slice is Exact/Structural
/// true. Only a still-Unknown strict result triggers a second pass under a
/// different requested policy, which may consume approximate decisions.
fn coordinates_equal(pairs: &[(&Real, &Real)], policy: PredicatePolicy) -> PredicateOutcome<bool> {
    let evaluate = |policy| {
        let mut result = PredicateOutcome::decided(true, Certainty::Exact, Escalation::Structural);
        for &(left, right) in pairs {
            result = result.and(map_outcome(
                compare_reals_with_policy(left, right, policy),
                |ordering| ordering == Ordering::Equal,
            ));
            if result.value() == Some(false) {
                return result;
            }
        }
        result
    };
    // Search all coordinates for an admissible proof before consuming any
    // approximate terminal. An unrelated undecided coordinate is not a
    // dependency of a certified inequality witness.
    let certified = evaluate(PredicatePolicy::STRICT);
    if matches!(certified, PredicateOutcome::Unknown { .. }) && policy != PredicatePolicy::STRICT {
        evaluate(policy)
    } else {
        certified
    }
}

/// Compare two 3D points lexicographically by `(x, y, z)` with an explicit
/// policy.
///
/// This is the 3D counterpart to [`compare_point2_lexicographic_with_policy`].
/// Only Equal advances to the next coordinate; a non-equal result or Unknown
/// stops. Successful comparisons join consumed certainty/stages, so approximate
/// equality of earlier coordinates qualifies the final order too. This ordered
/// question differs from [`point3_equal_with_policy`]'s search for any inequality.
pub fn compare_point3_lexicographic_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Ordering> {
    match compare_reals_with_policy(&left.x, &right.x, policy) {
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            certainty: x_certainty,
            stage: x_stage,
        } => match compare_reals_with_policy(&left.y, &right.y, policy) {
            PredicateOutcome::Decided {
                value: Ordering::Equal,
                certainty: y_certainty,
                stage: y_stage,
            } => match compare_reals_with_policy(&left.z, &right.z, policy) {
                PredicateOutcome::Decided {
                    value,
                    certainty: z_certainty,
                    stage: z_stage,
                } => PredicateOutcome::decided(
                    value,
                    max_certainty(max_certainty(x_certainty, y_certainty), z_certainty),
                    max_stage(max_stage(x_stage, y_stage), z_stage),
                ),
                PredicateOutcome::Unknown { needed, stage } => {
                    PredicateOutcome::unknown(needed, stage)
                }
            },
            PredicateOutcome::Decided {
                value,
                certainty: y_certainty,
                stage: y_stage,
            } => PredicateOutcome::decided(
                value,
                max_certainty(x_certainty, y_certainty),
                max_stage(x_stage, y_stage),
            ),
            PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
        },
        decided_or_unknown => decided_or_unknown,
    }
}

/// Return whether two 3D points have equal coordinates with an explicit
/// predicate escalation policy.
///
/// Point equality compares exact Real coordinate values under a decision policy.
/// Keeping the 3D form beside [`point2_equal_with_policy`] gives callers a direct semantic
/// API for vertex identity and normal-row deduplication instead of requiring a
/// zero-radius sphere classification.
/// A certified unequal coordinate suffices; equality requires every coordinate.
/// Searches x, y, z under STRICT first, continuing past unresolved coordinates.
/// Only an unresolved strict pass is retried under a different requested policy;
/// any terminal qualification survives in the result. Coordinate equality does
/// not establish a shared source, selected root or construction identity.
pub fn point3_equal_with_policy(
    left: &Point3,
    right: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    coordinates_equal(
        &[
            (&left.x, &right.x),
            (&left.y, &right.y),
            (&left.z, &right.z),
        ],
        policy,
    )
}

/// Classifies an exact rational against zero using its total numeric ordering.
#[inline(always)]
fn exact_rational_sign(value: &Rational) -> Sign {
    match value
        .partial_cmp(&Rational::zero())
        .expect("exact rational ordering is total")
    {
        Ordering::Less => Sign::Negative,
        Ordering::Equal => Sign::Zero,
        Ordering::Greater => Sign::Positive,
    }
}

/// Retains the weaker certainty by rank, choosing the left on a tie.
#[inline(always)]
fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    if certainty_rank(left) >= certainty_rank(right) {
        left
    } else {
        right
    }
}

/// Ranks Exact, Filtered and Approximate for conservative receipt composition.
#[inline(always)]
fn certainty_rank(certainty: Certainty) -> u8 {
    match certainty {
        Certainty::Exact => 0,
        Certainty::Filtered => 1,
        Certainty::Approximate => 2,
    }
}

/// Retains the higher-ranked stage, choosing the left on a tie.
#[inline(always)]
fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

/// Ranks Structural through Undecided for summaries, not chronological traces.
#[inline(always)]
fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

/// Ordering values, symbolic proof replay, qualification and construction controls.
#[cfg(test)]
mod tests {
    use super::*;

    /// Allows the terminal while exact and cached proofs remain preferred.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Converts an integer fixture directly to an exact Real.
    fn real(value: i32) -> hyperreal::Real {
        hyperreal::Real::from(value)
    }

    /// Retains a proved unit-circle identity before supplying a separate
    /// angle-addition zero for unresolved/terminal-policy tests.
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

    /// Checks rational sign values, Unknown in either position and positive pi/e signs.
    #[test]
    fn paired_signs_batch_exact_rationals_and_preserve_unknowns() {
        assert_eq!(
            crate::classify_real_sign_pair(&real(-1), &real(0), APPROX).value(),
            Some((Sign::Negative, Sign::Zero))
        );

        let undecidable = terminally_unresolved_zero();
        assert!(matches!(
            classify_real_sign_pair_with_policy(&real(1), &undecidable, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_real_sign_pair_with_policy(&undecidable, &real(1), PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            classify_real_sign_pair_with_policy(&Real::pi(), &Real::e(), PredicatePolicy::STRICT)
                .value(),
            Some((Sign::Positive, Sign::Positive))
        );
    }

    /// Checks Less, Equal and Greater values on integers, without asserting a dispatch path.
    #[test]
    fn real_ordering_uses_exact_difference_sign() {
        assert_eq!(
            crate::compare_reals(&real(1), &real(2), APPROX).value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            crate::compare_reals(&real(2), &real(2), APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::compare_reals(&real(3), &real(2), APPROX).value(),
            Some(Ordering::Greater)
        );
    }

    /// Checks strict equality and zero difference of two nested symbolic parameter formulas.
    #[test]
    fn exact_normal_form_resolves_nested_parameter_replay() {
        let root_two = real(2).sqrt().unwrap();
        let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
        let half = (real(1) / real(2)).unwrap();
        let shared_offset = root_two.clone() * real(3) + half;
        let contact = (((root_two.clone() * real(4) - shared_offset.clone()) * Real::pi())
            * root_two_over_pi.clone()
            / real(4))
        .unwrap();
        let domain = (((root_two * real(2) - shared_offset) * Real::pi()) * root_two_over_pi
            / real(4))
        .unwrap()
            + real(1);

        assert_eq!(
            compare_reals_with_policy(&contact, &domain, PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            classify_real_sign_with_policy(&(contact - domain), PredicatePolicy::STRICT).value(),
            Some(Sign::Zero)
        );
    }

    /// Checks that a 2^-600 residual is proved positive and retained in scalar
    /// facts, with exact Structural receipts on warm and fresh comparisons/equality.
    #[test]
    fn exact_normal_form_precedes_terminal_approximate_equality() {
        // Rebuild the symbolic cancellation plus a positive residual for cold comparisons.
        let fresh_positive = || {
            let root_two = real(2).sqrt().unwrap();
            let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
            let half = (real(1) / real(2)).unwrap();
            let shared_offset = root_two.clone() * real(3) + half;
            let contact = (((root_two.clone() * real(4) - shared_offset.clone()) * Real::pi())
                * root_two_over_pi.clone()
                / real(4))
            .unwrap();
            let domain = (((root_two * real(2) - shared_offset) * Real::pi()) * root_two_over_pi
                / real(4))
            .unwrap()
                + real(1);
            let tiny = real(2).powi_i64(-600).unwrap();
            contact - domain + tiny
        };
        let positive = fresh_positive();

        assert_eq!(positive.immediate_sign(), None);
        assert_eq!(
            classify_real_sign_with_policy(&positive, APPROX),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Structural)
        );
        // The first symbolic proof must be retained, not recomputed or
        // replaced by terminal approximate equality on a later query.
        assert_eq!(
            positive.immediate_sign(),
            Some(hyperreal::RealSign::Positive)
        );
        assert_eq!(
            compare_reals_with_policy(&positive, &Real::zero(), APPROX),
            PredicateOutcome::decided(Ordering::Greater, Certainty::Exact, Escalation::Structural)
        );
        assert_eq!(
            compare_reals_with_policy(&fresh_positive(), &Real::zero(), APPROX),
            PredicateOutcome::decided(Ordering::Greater, Certainty::Exact, Escalation::Structural)
        );

        let left = Point2::new(positive, Real::zero());
        let right = Point2::new(Real::zero(), Real::zero());
        assert_eq!(
            point2_equal_with_policy(&left, &right, APPROX),
            PredicateOutcome::decided(false, Certainty::Exact, Escalation::Structural)
        );
        assert_eq!(
            point2_equal_with_policy(&Point2::new(fresh_positive(), Real::zero()), &right, APPROX),
            PredicateOutcome::decided(false, Certainty::Exact, Escalation::Structural)
        );
    }

    /// Checks y ordering after equal x values.
    #[test]
    fn point2_lexicographic_ordering_uses_y_as_tie_breaker() {
        let left = Point2::new(real(1), real(4));
        let right = Point2::new(real(1), real(5));

        assert_eq!(
            crate::compare_point2_lexicographic(&left, &right, APPROX).value(),
            Some(Ordering::Less)
        );
    }

    /// Checks rational equal/unequal points and a matching symbolic y coordinate.
    #[test]
    fn point2_equal_uses_exact_coordinate_ordering() {
        let left = Point2::new(real(1), real(4));
        let same = Point2::new(real(1), real(4));
        let different = Point2::new(real(1), real(5));

        assert_eq!(
            crate::point2_equal(&left, &same, APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::point2_equal(&left, &different, APPROX).value(),
            Some(false)
        );
        assert_eq!(
            crate::point2_equal(&left, &Point2::new(real(2), real(4)), APPROX).value(),
            Some(false)
        );

        let symbolic_left = Point2::new(real(1), Real::pi());
        let symbolic_same = Point2::new(real(1), Real::pi());
        assert_eq!(
            point2_equal_with_policy(&symbolic_left, &symbolic_same, PredicatePolicy::STRICT)
                .value(),
            Some(true)
        );
    }

    /// Checks z ordering after equal x and y values.
    #[test]
    fn point3_lexicographic_ordering_uses_z_as_second_tie_breaker() {
        let left = Point3::new(real(1), real(4), real(6));
        let right = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            crate::compare_point3_lexicographic(&left, &right, APPROX).value(),
            Some(Ordering::Less)
        );
    }

    /// Checks equal rational spatial points and a differing z coordinate.
    #[test]
    fn point3_equal_uses_exact_coordinate_ordering() {
        let left = Point3::new(real(1), real(4), real(6));
        let same = Point3::new(real(1), real(4), real(6));
        let different = Point3::new(real(1), real(4), real(7));

        assert_eq!(
            crate::point3_equal(&left, &same, APPROX).value(),
            Some(true)
        );
        assert_eq!(
            crate::point3_equal(&left, &different, APPROX).value(),
            Some(false)
        );
    }

    /// Checks bound/min/max/clamp values, reversed-bound rejection and strict
    /// unresolved decisions at normalization and each value-bound comparison.
    #[test]
    fn real_min_max_clamp_and_bounds_use_order_predicates() {
        let low = real(1);
        let mid = real(2);
        let high = real(3);

        assert_eq!(crate::real_le(&low, &mid, APPROX).value(), Some(true));
        assert_eq!(crate::real_ge(&high, &mid, APPROX).value(), Some(true));
        assert_eq!(crate::real_min(&high, &low, APPROX).value(), Some(&low));
        assert_eq!(crate::real_min(&low, &high, APPROX).value(), Some(&low));
        assert_eq!(crate::real_max(&high, &low, APPROX).value(), Some(&high));
        assert_eq!(crate::real_max(&low, &high, APPROX).value(), Some(&high));
        assert_eq!(
            crate::real_clamp(mid.clone(), &low, &high, APPROX).value(),
            Some(mid)
        );
        assert_eq!(
            crate::real_clamp(real(0), &low, &high, APPROX).value(),
            Some(low.clone())
        );
        assert_eq!(
            crate::real_clamp(real(4), &low, &high, APPROX).value(),
            Some(high)
        );

        assert!(matches!(
            real_clamp_with_policy(real(2), &real(3), &real(1), PredicatePolicy::STRICT),
            PredicateOutcome::Unknown {
                needed: RefinementNeed::Unsupported,
                ..
            }
        ));

        let unresolved = terminally_unresolved_zero();
        assert!(matches!(
            real_clamp_with_policy(real(1), &unresolved, &real(0), PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            real_clamp_with_policy(real(0), &unresolved, &real(10), PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            real_clamp_with_policy(
                unresolved.clone(),
                &real(0),
                &real(10),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            real_clamp_with_policy(unresolved, &real(-10), &real(0), PredicatePolicy::STRICT,),
            PredicateOutcome::Unknown { .. }
        ));
    }

    /// Checks lexicographic early decisions/Unknowns and selected certificate,
    /// sign and rank mappings; it does not instrument dispatch counts.
    #[test]
    fn lexicographic_unknowns_and_ranking_helpers_cover_all_stages() {
        let unresolved = terminally_unresolved_zero();
        let zero = Real::zero();

        assert!(matches!(
            compare_point2_lexicographic_with_policy(
                &Point2::new(zero.clone(), unresolved.clone()),
                &Point2::new(zero.clone(), zero.clone()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert_eq!(
            compare_point2_lexicographic_with_policy(
                &Point2::new(real(-1), zero.clone()),
                &Point2::new(real(1), zero.clone()),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Less)
        );

        assert_eq!(
            compare_point3_lexicographic_with_policy(
                &Point3::new(zero.clone(), real(-1), zero.clone()),
                &Point3::new(zero.clone(), real(1), zero.clone()),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Less)
        );
        assert_eq!(
            compare_point3_lexicographic_with_policy(
                &Point3::new(real(-1), zero.clone(), zero.clone()),
                &Point3::new(real(1), zero.clone(), zero.clone()),
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(Ordering::Less)
        );
        assert!(matches!(
            compare_point3_lexicographic_with_policy(
                &Point3::new(zero.clone(), unresolved.clone(), zero.clone()),
                &Point3::new(zero.clone(), zero.clone(), zero.clone()),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            compare_point3_lexicographic_with_policy(
                &Point3::new(zero.clone(), zero.clone(), unresolved),
                &Point3::new(zero.clone(), zero.clone(), zero),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(
            ordering_certificate_stage(RealOrderingCertificate::ExactRationalComparison),
            Escalation::Exact
        );
        assert_eq!(ordering_from_sign(Sign::Negative), Ordering::Less);
        assert_eq!(ordering_from_sign(Sign::Zero), Ordering::Equal);
        assert_eq!(ordering_from_sign(Sign::Positive), Ordering::Greater);
        assert_eq!(exact_rational_sign(&Rational::new(-1)), Sign::Negative);
        assert_eq!(exact_rational_sign(&Rational::zero()), Sign::Zero);
        assert_eq!(exact_rational_sign(&Rational::new(1)), Sign::Positive);
        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Filtered),
            Certainty::Filtered
        );
        assert_eq!(
            max_certainty(Certainty::Exact, Certainty::Approximate),
            Certainty::Approximate
        );
        assert_eq!(
            max_stage(Escalation::Structural, Escalation::Filter),
            Escalation::Filter
        );
        assert_eq!(
            max_stage(Escalation::Exact, Escalation::Refined),
            Escalation::Refined
        );
        assert_eq!(
            max_stage(Escalation::Exact, Escalation::Undecided),
            Escalation::Undecided
        );
    }

    /// Checks the reciprocal relation for two, canonical zero failure and strict
    /// Unknown propagation; no refinement-reuse count is asserted.
    #[test]
    fn reciprocal_reuses_the_policy_nonzero_decision() {
        let reciprocal = crate::reciprocal_real(&real(2), APPROX)
            .expect("two is nonzero")
            .value()
            .expect("the reciprocal is decided");
        assert_eq!(
            crate::compare_reals(&(real(2) * reciprocal), &real(1), APPROX).value(),
            Some(Ordering::Equal)
        );
        assert_eq!(
            crate::reciprocal_real(&real(0), APPROX),
            Err(Problem::DivideByZero)
        );

        let undecidable = terminally_unresolved_zero();
        assert!(matches!(
            reciprocal_real_with_policy(&undecidable, PredicatePolicy::STRICT),
            Ok(PredicateOutcome::Unknown { .. })
        ));
    }
}
