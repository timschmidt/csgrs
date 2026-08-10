//! Classification helpers for curve topology.
//!
//! These helpers centralize the "branch only after the sign/order relation is
//! known" rule that keeps geometry algorithms robust. The exact-predicate
//! discipline follows adaptive robust predicates. `EdgePreview` is the named exception for UI and IO
//! boundaries where lossy finite-precision output is already part of the
//! contract; finite-precision intersection output and degeneracy issues are
//! handled by the finite-output segment-intersection adapter.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::{CurveContext, Point2};

/// Result of a classification step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification<T> {
    /// The classification was decided.
    Decided(T),
    /// The active policy could not decide the classification.
    Uncertain(UncertaintyReason),
}

impl<T> Classification<T> {
    /// Returns true when this classification contains a decided value.
    pub const fn is_decided(&self) -> bool {
        matches!(self, Self::Decided(_))
    }

    /// Returns true when this classification carries an explicit uncertainty reason.
    pub const fn is_uncertain(&self) -> bool {
        matches!(self, Self::Uncertain(_))
    }

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

    pub(crate) const fn from_predicate_sign(sign: hypersolve::PredicateSign) -> Self {
        match sign {
            hypersolve::PredicateSign::Positive => Self::Left,
            hypersolve::PredicateSign::Negative => Self::Right,
            hypersolve::PredicateSign::Zero => Self::On,
        }
    }
}

pub(crate) fn classify_oriented_line(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<LineSide> {
    if policy.is_edge_preview() {
        // Preview mode is a display/editing classifier. Use the current Real
        // approximation consistently here instead of sending rotated radical
        // expressions into the certified predicate path, otherwise arc sweep
        // checks can reject legitimate preview intersections before the exact
        // segment relation has a chance to retain them as candidates.
        let det = orient2_real_expr(from, to, point);
        return real_sign(&det, policy)
            .map(LineSide::from_real_sign)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign));
    }

    {
        // This is the orientation determinant used throughout planar
        // computational geometry. When available, route it through hyperlimit's
        // certified predicate path rather than comparing approximate floats,
        // matching robust-predicate practice for topology
        // branches.
        let predicate_outcome = hyperlimit::orient2(
            &predicate_point(from),
            &predicate_point(to),
            &predicate_point(point),
            policy.predicate_policy(),
        );
        match policy.consume_predicate(predicate_outcome) {
            Some(value) => Classification::Decided(LineSide::from_predicate_sign(value)),
            None => {
                let det = orient2_real_expr(from, to, point);
                real_sign(&det, policy)
                    .map(LineSide::from_real_sign)
                    .map(Classification::Decided)
                    .unwrap_or(Classification::Uncertain(UncertaintyReason::Predicate))
            }
        }
    }
}

pub(crate) fn orient2_real_expr(from: &Point2, to: &Point2, point: &Point2) -> Real {
    let abx = to.x() - from.x();
    let aby = to.y() - from.y();
    let acx = point.x() - from.x();
    let acy = point.y() - from.y();
    (&abx * &acy) - (&aby * &acx)
}

pub(crate) fn real_sign(value: &Real, policy: &CurveContext) -> Option<RealSign> {
    if value.zero_status() == ZeroStatus::Zero {
        return Some(RealSign::Zero);
    }

    if policy.is_edge_preview()
        && let Some(value) = value.to_f64_lossy()
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

    policy
        .consume_predicate(hypersolve::classify_real_sign_predicate(
            value,
            policy.predicate_policy(),
        ))
        .map(|sign| match sign {
            hypersolve::PredicateSign::Negative => RealSign::Negative,
            hypersolve::PredicateSign::Zero => RealSign::Zero,
            hypersolve::PredicateSign::Positive => RealSign::Positive,
        })
}

pub(crate) fn is_zero(value: &Real, policy: &CurveContext) -> Option<bool> {
    match value.zero_status() {
        ZeroStatus::Zero => Some(true),
        ZeroStatus::NonZero => Some(false),
        ZeroStatus::Unknown => real_sign(value, policy).map(|sign| sign == RealSign::Zero),
    }
}

pub(crate) fn compare_reals(left: &Real, right: &Real, policy: &CurveContext) -> Option<Ordering> {
    if std::ptr::eq(left, right) {
        return Some(Ordering::Equal);
    }
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        return left.partial_cmp(right);
    }

    if !policy.is_edge_preview() {
        // Curve parameter ordering is a topology predicate: it decides whether
        // an intersection root lies on a segment, whether two split markers
        // coincide, and how degenerate overlaps are classified. Route the sign
        // of `left - right` through hyperlimit's predicate pipeline so scalar
        // ordering has the same certified/unknown boundary as orientation.
        // This follows the exactness model's exact geometric computation split between exact
        // predicate decisions and approximate edge views.
        if let Some(ordering) = policy.consume_predicate(hypersolve::compare_real_predicate(
            left,
            right,
            policy.predicate_policy(),
        )) {
            return Some(ordering);
        }
    }

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
    policy: &CurveContext,
) -> Option<Ordering> {
    if policy.is_edge_preview()
        && let (Some(left), Some(right)) = (left.to_f64_lossy(), right.to_f64_lossy())
        && left.is_finite()
        && right.is_finite()
    {
        // Split marker ordering feeds display/event reconstruction, not a
        // certified topology decision, in `EdgePreview`. Comparing the same
        // finite values that will be rendered avoids artificial branch
        // vertices from unsimplified radical expressions; this is the same
        // finite-output boundary finite-output segment intersection treats as separate from exact segment
        // intersection predicates.
        return left.partial_cmp(&right);
    }

    compare_reals(left, right, policy)
}

pub(crate) fn sort_pair(a: Real, b: Real, policy: &CurveContext) -> Option<(Real, Real)> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Greater => Some((b, a)),
        Ordering::Less | Ordering::Equal => Some((a, b)),
    }
}

pub(crate) fn max_real(a: Real, b: Real, policy: &CurveContext) -> Option<Real> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Less => Some(b),
        Ordering::Equal | Ordering::Greater => Some(a),
    }
}

pub(crate) fn min_real(a: Real, b: Real, policy: &CurveContext) -> Option<Real> {
    match compare_reals(&a, &b, policy)? {
        Ordering::Greater => Some(b),
        Ordering::Less | Ordering::Equal => Some(a),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ClosedUnitIntervalLocation {
    Outside,
    Start,
    Interior,
    End,
}

pub(crate) fn closed_unit_interval_location(
    value: &Real,
    policy: &CurveContext,
) -> Option<ClosedUnitIntervalLocation> {
    // Edge-preview f64 parameters are candidate filters only: decisively
    // out-of-range values cannot represent finite segment hits, while
    // near-boundary values still fall through to exact comparison.
    if policy.is_edge_preview()
        && let Some(approx) = value.to_f64_lossy()
    {
        let tolerance = crate::policy::preview_tolerance()
            .map(|tolerance| tolerance.absolute.max(tolerance.relative))
            .unwrap_or(1e-12);
        if approx.is_finite() && (approx < -tolerance || approx > 1.0 + tolerance) {
            return Some(ClosedUnitIntervalLocation::Outside);
        }
    }

    let zero = Real::zero();
    let one = Real::one();
    let lower = compare_reals(value, &zero, policy)?;
    let upper = compare_reals(value, &one, policy)?;
    Some(
        if matches!(lower, Ordering::Less) || matches!(upper, Ordering::Greater) {
            ClosedUnitIntervalLocation::Outside
        } else if matches!(lower, Ordering::Equal) {
            ClosedUnitIntervalLocation::Start
        } else if matches!(upper, Ordering::Equal) {
            ClosedUnitIntervalLocation::End
        } else {
            ClosedUnitIntervalLocation::Interior
        },
    )
}

pub(crate) fn in_closed_unit_interval(value: &Real, policy: &CurveContext) -> Option<bool> {
    closed_unit_interval_location(value, policy)
        .map(|location| location != ClosedUnitIntervalLocation::Outside)
}

pub(crate) fn at_unit_interval_endpoint(value: &Real, policy: &CurveContext) -> Option<bool> {
    closed_unit_interval_location(value, policy).map(|location| {
        matches!(
            location,
            ClosedUnitIntervalLocation::Start | ClosedUnitIntervalLocation::End
        )
    })
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x().clone(), point.y().clone())
}
