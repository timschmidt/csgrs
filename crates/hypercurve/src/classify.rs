//! Classification helpers for curve topology.

use std::cmp::Ordering;

use hyperlattice::{Backend, Scalar, ScalarSign, ZeroStatus};

use crate::{CurvePolicy, NumericMode, Point2};

/// Result of a classification step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification<T> {
    /// The classification was decided.
    Decided(T),
    /// The active policy could not decide the classification.
    Uncertain(UncertaintyReason),
}

/// Reason an operation could not decide a topology branch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UncertaintyReason {
    /// A scalar sign could not be proven or approximated under the active policy.
    ScalarSign,
    /// Predicate policy could not decide the branch.
    Predicate,
    /// Parameter ordering could not be decided.
    Ordering,
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
    #[cfg(not(feature = "predicates"))]
    pub(crate) const fn from_scalar_sign(sign: ScalarSign) -> Self {
        match sign {
            ScalarSign::Positive => Self::Left,
            ScalarSign::Negative => Self::Right,
            ScalarSign::Zero => Self::On,
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

pub(crate) fn classify_oriented_line<B: Backend>(
    from: &Point2<B>,
    to: &Point2<B>,
    point: &Point2<B>,
    policy: &CurvePolicy,
) -> Classification<LineSide> {
    #[cfg(feature = "predicates")]
    {
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
        let det = orient2d_scalar(from, to, point);
        scalar_sign(&det, policy)
            .map(LineSide::from_scalar_sign)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::ScalarSign))
    }
}

#[cfg(not(feature = "predicates"))]
pub(crate) fn orient2d_scalar<B: Backend>(
    from: &Point2<B>,
    to: &Point2<B>,
    point: &Point2<B>,
) -> Scalar<B> {
    let abx = to.x() - from.x();
    let aby = to.y() - from.y();
    let acx = point.x() - from.x();
    let acy = point.y() - from.y();
    (&abx * &acy) - (&aby * &acx)
}

pub(crate) fn scalar_sign<B: Backend>(
    value: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<ScalarSign> {
    if let Some(sign) = value.structural_facts().sign {
        return Some(sign);
    }

    if !matches!(policy.numeric_mode, NumericMode::Approximate)
        && let Some(sign) = value.refine_sign_until(-512)
    {
        return Some(sign);
    }

    if matches!(policy.numeric_mode, NumericMode::Approximate) {
        return value.to_f64_approx().and_then(|value| {
            if value > 0.0 {
                Some(ScalarSign::Positive)
            } else if value < 0.0 {
                Some(ScalarSign::Negative)
            } else if value == 0.0 {
                Some(ScalarSign::Zero)
            } else {
                None
            }
        });
    }

    None
}

pub(crate) fn is_zero<B: Backend>(value: &Scalar<B>, policy: &CurvePolicy) -> Option<bool> {
    match value.zero_status() {
        ZeroStatus::Zero => Some(true),
        ZeroStatus::NonZero => Some(false),
        ZeroStatus::Unknown => scalar_sign(value, policy).map(|sign| sign == ScalarSign::Zero),
    }
}

pub(crate) fn compare_scalars<B: Backend>(
    left: &Scalar<B>,
    right: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<Ordering> {
    let delta = left - right;
    scalar_sign(&delta, policy).map(|sign| match sign {
        ScalarSign::Negative => Ordering::Less,
        ScalarSign::Zero => Ordering::Equal,
        ScalarSign::Positive => Ordering::Greater,
    })
}

pub(crate) fn sort_pair<B: Backend>(
    a: Scalar<B>,
    b: Scalar<B>,
    policy: &CurvePolicy,
) -> Option<(Scalar<B>, Scalar<B>)> {
    match compare_scalars(&a, &b, policy)? {
        Ordering::Greater => Some((b, a)),
        Ordering::Less | Ordering::Equal => Some((a, b)),
    }
}

pub(crate) fn max_scalar<B: Backend>(
    a: Scalar<B>,
    b: Scalar<B>,
    policy: &CurvePolicy,
) -> Option<Scalar<B>> {
    match compare_scalars(&a, &b, policy)? {
        Ordering::Less => Some(b),
        Ordering::Equal | Ordering::Greater => Some(a),
    }
}

pub(crate) fn min_scalar<B: Backend>(
    a: Scalar<B>,
    b: Scalar<B>,
    policy: &CurvePolicy,
) -> Option<Scalar<B>> {
    match compare_scalars(&a, &b, policy)? {
        Ordering::Greater => Some(b),
        Ordering::Less | Ordering::Equal => Some(a),
    }
}

pub(crate) fn in_closed_unit_interval<B: Backend>(
    value: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    let zero = Scalar::<B>::zero();
    let one = Scalar::<B>::one();
    let lower = compare_scalars(value, &zero, policy)?;
    let upper = compare_scalars(value, &one, policy)?;
    Some(!matches!(lower, Ordering::Less) && !matches!(upper, Ordering::Greater))
}

pub(crate) fn at_unit_interval_endpoint<B: Backend>(
    value: &Scalar<B>,
    policy: &CurvePolicy,
) -> Option<bool> {
    let zero = Scalar::<B>::zero();
    let one = Scalar::<B>::one();
    let at_zero = compare_scalars(value, &zero, policy)? == Ordering::Equal;
    let at_one = compare_scalars(value, &one, policy)? == Ordering::Equal;
    Some(at_zero || at_one)
}

#[cfg(feature = "predicates")]
fn predicate_point<B: Backend>(point: &Point2<B>) -> hyperlimit::Point2<Scalar<B>> {
    hyperlimit::Point2::new(point.x().clone(), point.y().clone())
}
