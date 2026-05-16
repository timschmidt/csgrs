//! Classification helpers for curve topology.
//!
//! These helpers centralize the "branch only after the sign/order relation is
//! known" rule that keeps geometry algorithms robust. The exact-predicate
//! discipline follows Shewchuk, "Adaptive Precision Floating-Point Arithmetic
//! and Fast Robust Geometric Predicates" (*Discrete & Computational Geometry*
//! 18(3), 305-363, 1997). `EdgePreview` is the named exception for UI and IO
//! boundaries where lossy finite-precision output is already part of the
//! contract; finite-precision intersection output and degeneracy issues are
//! discussed by Hobby, "Practical Segment Intersection with Finite Precision
//! Output" (*Computational Geometry* 13(4), 199-214, 1999).

use std::cmp::Ordering;

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::{CurvePolicy, NumericMode, Point2};

/// Result of a classification step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification<T> {
    /// The classification was decided.
    Decided(T),
    /// The active policy could not decide the classification.
    Uncertain(UncertaintyReason),
}

impl<T> Classification<T> {
    /// Maps a decided value while preserving uncertainty unchanged.
    pub fn map<U, F>(self, f: F) -> Classification<U>
    where
        F: FnOnce(T) -> U,
    {
        match self {
            Self::Decided(value) => Classification::Decided(f(value)),
            Self::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }
}

/// Reason an operation could not decide a topology branch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UncertaintyReason {
    /// A Real sign could not be proven under the active policy.
    RealSign,
    /// Predicate policy could not decide the branch.
    Predicate,
    /// Parameter ordering could not be decided.
    Ordering,
    /// The query lies on a boundary where the requested Real result is undefined.
    Boundary,
    /// The requested operation is not supported by this slice.
    Unsupported,
}

/// Side of an oriented line.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineSide {
    /// Point lies to the left of the oriented line.
    Left,
    /// Point lies to the right of the oriented line.
    Right,
    /// Point lies on the line.
    On,
}

impl LineSide {
    pub(crate) const fn from_real_sign(sign: RealSign) -> Self {
        match sign {
            RealSign::Positive => Self::Left,
            RealSign::Negative => Self::Right,
            RealSign::Zero => Self::On,
        }
    }

    #[cfg(feature = "predicates")]
    pub(crate) const fn from_predicate_sign(sign: hyperlimit::Sign) -> Self {
        match sign {
            hyperlimit::Sign::Positive => Self::Left,
            hyperlimit::Sign::Negative => Self::Right,
            hyperlimit::Sign::Zero => Self::On,
        }
    }
}

pub(crate) fn classify_oriented_line(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<LineSide> {
    if matches!(policy.numeric_mode, NumericMode::EdgePreview) {
        // Preview mode is a display/editing classifier. Use the current Real
        // approximation consistently here instead of sending rotated radical
        // expressions into the certified predicate path, otherwise arc sweep
        // checks can reject legitimate preview intersections before the exact
        // segment relation has a chance to retain them as candidates.
        let det = orient2d_real_expr(from, to, point);
        return real_sign(&det, policy)
            .map(LineSide::from_real_sign)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign));
    }

    #[cfg(feature = "predicates")]
    {
        // This is the orientation determinant used throughout planar
        // computational geometry. When available, route it through hyperlimit's
        // certified predicate path rather than comparing approximate floats,
        // matching Shewchuk's robust-predicate recommendation for topology
        // branches.
        let predicate_outcome = hyperlimit::orient::orient2d_with_policy(
            &predicate_point(from),
            &predicate_point(to),
            &predicate_point(point),
            policy.predicate_policy,
        );
        match predicate_outcome {
            hyperlimit::PredicateOutcome::Decided { value, .. } => {
                Classification::Decided(LineSide::from_predicate_sign(value))
            }
            hyperlimit::PredicateOutcome::Unknown { .. } => {
                Classification::Uncertain(UncertaintyReason::Predicate)
            }
        }
    }

    #[cfg(not(feature = "predicates"))]
    {
        let det = orient2d_real_expr(from, to, point);
        real_sign(&det, policy)
            .map(LineSide::from_real_sign)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
    }
}

pub(crate) fn orient2d_real_expr(from: &Point2, to: &Point2, point: &Point2) -> Real {
    let abx = to.x() - from.x();
    let aby = to.y() - from.y();
    let acx = point.x() - from.x();
    let acy = point.y() - from.y();
    (&abx * &acy) - (&aby * &acx)
}

pub(crate) fn real_sign(value: &Real, policy: &CurvePolicy) -> Option<RealSign> {
    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let Some(value) = value.to_f64_approx()
        && value.is_finite()
    {
        // Edge-preview mode is allowed to collapse a hyperreal value to the
        // current `f64` approximation before committing a UI/display branch.
        // This keeps radical expressions from carrying stale structural signs
        // into broad-phase and sweep tests.
        return if value > 0.0 {
            Some(RealSign::Positive)
        } else if value < 0.0 {
            Some(RealSign::Negative)
        } else {
            Some(RealSign::Zero)
        };
    }

    if let Some(sign) = value.structural_facts().sign {
        return Some(sign);
    }

    if let Some(sign) = value.refine_sign_until(-4096) {
        return Some(sign);
    }

    None
}

pub(crate) fn is_zero(value: &Real, policy: &CurvePolicy) -> Option<bool> {
    match value.zero_status() {
        ZeroStatus::Zero => Some(true),
        ZeroStatus::NonZero => Some(false),
        ZeroStatus::Unknown => real_sign(value, policy).map(|sign| sign == RealSign::Zero),
    }
}

pub(crate) fn compare_reals(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<Ordering> {
    let delta = left - right;
    real_sign(&delta, policy).map(|sign| match sign {
        RealSign::Negative => Ordering::Less,
        RealSign::Zero => Ordering::Equal,
        RealSign::Positive => Ordering::Greater,
    })
}

pub(crate) fn compare_reals_for_split_ordering(
    left: &Real,
    right: &Real,
    policy: &CurvePolicy,
) -> Option<Ordering> {
    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let (Some(left), Some(right)) = (left.to_f64_approx(), right.to_f64_approx())
        && left.is_finite()
        && right.is_finite()
    {
        // Split marker ordering feeds display/event reconstruction, not a
        // certified topology decision, in `EdgePreview`. Comparing the same
        // finite values that will be rendered avoids artificial branch
        // vertices from unsimplified radical expressions; this is the same
        // finite-output boundary Hobby treats as separate from exact segment
        // intersection predicates.
        return left.partial_cmp(&right);
    }

    compare_reals(left, right, policy)
}

pub(crate) fn sort_pair(a: Real, b: Real, policy: &CurvePolicy) -> Option<(Real, Real)> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Greater => Some((b, a)),
        Ordering::Less | Ordering::Equal => Some((a, b)),
    }
}

pub(crate) fn max_real(a: Real, b: Real, policy: &CurvePolicy) -> Option<Real> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Less => Some(b),
        Ordering::Equal | Ordering::Greater => Some(a),
    }
}

pub(crate) fn min_real(a: Real, b: Real, policy: &CurvePolicy) -> Option<Real> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Greater => Some(b),
        Ordering::Less | Ordering::Equal => Some(a),
    }
}

pub(crate) fn in_closed_unit_interval(value: &Real, policy: &CurvePolicy) -> Option<bool> {
    // Edge-preview f64 parameters are candidate filters only: decisively
    // out-of-range values cannot represent finite segment hits, while
    // near-boundary values still fall through to exact comparison.
    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let Some(approx) = value.to_f64_approx()
    {
        let tolerance = policy
            .tolerance
            .map(|tolerance| tolerance.absolute.max(tolerance.relative))
            .unwrap_or(1e-12);
        if approx.is_finite() && (approx < -tolerance || approx > 1.0 + tolerance) {
            return Some(false);
        }
    }

    let zero = Real::zero();
    let one = Real::one();
    let lower = compare_reals(value, &zero, policy)?;
    let upper = compare_reals(value, &one, policy)?;
    Some(!matches!(lower, Ordering::Less) && !matches!(upper, Ordering::Greater))
}

pub(crate) fn at_unit_interval_endpoint(value: &Real, policy: &CurvePolicy) -> Option<bool> {
    let zero = Real::zero();
    let one = Real::one();
    let at_zero = compare_reals(value, &zero, policy)? == Ordering::Equal;
    let at_one = compare_reals(value, &one, policy)? == Ordering::Equal;
    Some(at_zero || at_one)
}

#[cfg(feature = "predicates")]
fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x().clone(), point.y().clone())
}
