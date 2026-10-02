//! Decided-or-uncertain outcomes of exact computations.
//!
//! Branching code consults a value only after its sign or order relation is
//! decided; an undecided step keeps the reason it could not be decided.

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
}

impl UncertaintyReason {
    /// Distinguishes an undefined scalar quotient from an unresolved nonzero proof.
    pub fn from_real_division(error: hyperreal::Problem) -> Self {
        match error {
            hyperreal::Problem::DivideByZero => Self::Boundary,
            hyperreal::Problem::UnknownZero => Self::RealSign,
            _ => Self::Unsupported,
        }
    }
}
