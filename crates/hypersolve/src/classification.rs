//! Decided-or-uncertain outcomes of exact computations.
//!
//! Branching code consults a value only after its sign or order relation is
//! decided; an undecided step keeps the reason it could not be decided.

/// Unwraps a decided [`Classification`], or returns its uncertainty, unchanged,
/// from the enclosing function.
///
/// - `decided!(value)` returns `Classification::Uncertain(reason)`;
/// - `decided!(value, Ok)` returns `Ok(Classification::Uncertain(reason))`.
///
/// This is an exact pass-through and expands to the same `match` written by
/// hand. The uncertainty reason is neither mapped nor inspected, and nothing
/// runs between evaluating `value` and returning. An arm that does any of
/// these, such as one that treats `BudgetExhausted` specially, stays a
/// hand-written `match`.
///
/// ```
/// use hypersolve::classification::{Classification, UncertaintyReason};
///
/// fn halved(value: Classification<i32>) -> Result<Classification<i32>, ()> {
///     let value = hypersolve::decided!(value, Ok);
///     Ok(Classification::Decided(value / 2))
/// }
///
/// assert_eq!(halved(Classification::Decided(8)), Ok(Classification::Decided(4)));
/// assert_eq!(
///     halved(Classification::Uncertain(UncertaintyReason::RealSign)),
///     Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
/// );
/// ```
#[macro_export]
macro_rules! decided {
    ($classification:expr $(,)?) => {
        match $classification {
            $crate::classification::Classification::Decided(value) => value,
            $crate::classification::Classification::Uncertain(reason) => {
                return $crate::classification::Classification::Uncertain(reason);
            }
        }
    };
    ($classification:expr, $wrap:path $(,)?) => {
        match $classification {
            $crate::classification::Classification::Decided(value) => value,
            $crate::classification::Classification::Uncertain(reason) => {
                return $wrap($crate::classification::Classification::Uncertain(reason));
            }
        }
    };
}

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

/// Reason an operation could not decide a branch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UncertaintyReason {
    /// A Real sign could not be proven under the active policy.
    RealSign,
    /// Predicate policy could not decide the branch.
    Predicate,
    /// Parameter ordering could not be decided.
    Ordering,
    /// The query lies on a boundary where the requested Real result is
    /// undefined, such as a denominator certified to vanish.
    Boundary,
    /// The requested operation is not supported by this slice.
    Unsupported,
    /// The caller's explicit operation work budget stopped a required proof.
    BudgetExhausted,
    /// Checked storage dimensions or a fallible allocation were unavailable.
    /// This does not assert that a configured work limit was reached.
    StorageUnavailable,
}

impl UncertaintyReason {
    /// Whether this stop must remain retryable rather than becoming lasting
    /// evidence of undecidability or permission for an approximate terminal.
    pub const fn is_resource_failure(self) -> bool {
        matches!(self, Self::BudgetExhausted | Self::StorageUnavailable)
    }

    /// Distinguishes an undefined scalar quotient from an unresolved nonzero proof.
    pub fn from_real_division(error: hyperreal::Problem) -> Self {
        match error {
            hyperreal::Problem::DivideByZero => Self::Boundary,
            hyperreal::Problem::UnknownZero => Self::RealSign,
            _ => Self::Unsupported,
        }
    }
}

/// The sign of a product of two values with the given signs.
pub const fn product_sign(
    first: hyperreal::RealSign,
    second: hyperreal::RealSign,
) -> hyperreal::RealSign {
    use hyperreal::RealSign;
    match (first, second) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            RealSign::Positive
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            RealSign::Negative
        }
    }
}
