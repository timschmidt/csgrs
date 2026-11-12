use crate::{IntervalError, SignSet};

// Private generic machinery: downstream types cannot use it to assert
// certified endpoint semantics. Public backends seal their endpoint traits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Bounds<T> {
    lower: T,
    upper: T,
}

impl<T: Copy + Ord> Bounds<T> {
    pub(crate) fn new(lower: T, upper: T) -> Result<Self, IntervalError> {
        if lower > upper {
            Err(IntervalError::ReversedEndpoints)
        } else {
            Ok(Self { lower, upper })
        }
    }

    pub(crate) fn point(value: T) -> Self {
        Self {
            lower: value,
            upper: value,
        }
    }

    pub(crate) fn endpoints(self) -> (T, T) {
        (self.lower, self.upper)
    }

    pub(crate) fn is_singleton(self) -> bool {
        self.lower == self.upper
    }

    pub(crate) fn contains(self, value: T) -> bool {
        self.lower <= value && value <= self.upper
    }

    pub(crate) fn subset(self, other: Self) -> bool {
        other.lower <= self.lower && self.upper <= other.upper
    }

    pub(crate) fn interior_subset(self, other: Self) -> bool {
        other.lower < self.lower && self.upper < other.upper
    }

    pub(crate) fn left_of(self, other: Self) -> bool {
        self.upper < other.lower
    }

    pub(crate) fn hull(self, other: Self) -> Self {
        Self {
            lower: self.lower.min(other.lower),
            upper: self.upper.max(other.upper),
        }
    }

    pub(crate) fn intersection(self, other: Self) -> Option<Self> {
        Self::new(self.lower.max(other.lower), self.upper.min(other.upper)).ok()
    }

    pub(crate) fn signs(self, zero: T) -> SignSet {
        let mut signs = SignSet::Empty;
        if self.lower < zero {
            signs = signs.union(SignSet::Negative);
        }
        if self.contains(zero) {
            signs = signs.union(SignSet::Zero);
        }
        if self.upper > zero {
            signs = signs.union(SignSet::Positive);
        }
        signs
    }
}

// The two concrete families share set relations, not arithmetic or scalar
// decisions. All constructors and serialization remain family-specific.
macro_rules! set_relations {
    () => {
        /// Whether this set is empty.
        pub fn is_empty(&self) -> bool {
            self.bounds.is_none()
        }

        /// Whether the set has exactly one member.
        pub fn is_singleton(&self) -> bool {
            self.bounds.is_some_and(|bounds| bounds.is_singleton())
        }

        /// Exact endpoint pair, or `None` for the empty set.
        pub fn bounds(&self) -> Option<(I, I)> {
            self.bounds.map(|b| b.endpoints())
        }

        /// Exact lower endpoint, or `None` for the empty set.
        pub fn lower(&self) -> Option<I> {
            self.bounds().map(|b| b.0)
        }

        /// Exact upper endpoint, or `None` for the empty set.
        pub fn upper(&self) -> Option<I> {
            self.bounds().map(|b| b.1)
        }

        /// Whether the set contains the given endpoint value (lattice bits for fixed).
        pub fn contains_point(&self, point: I) -> bool {
            self.bounds.is_some_and(|b| b.contains(point))
        }

        /// Set inclusion. Empty is a subset of every set.
        pub fn subset_of(&self, other: &Self) -> bool {
            match (self.bounds, other.bounds) {
                (None, _) => true,
                (Some(_), None) => false,
                (Some(a), Some(b)) => a.subset(b),
            }
        }

        /// Strict endpoint inclusion in the real interior of `other`.
        ///
        /// Empty is included in every interior. For integer ranges this is
        /// the real endpoint interior, not the discrete-topology interior.
        pub fn interior_subset_of(&self, other: &Self) -> bool {
            match (self.bounds, other.bounds) {
                (None, _) => true,
                (Some(_), None) => false,
                (Some(a), Some(b)) => a.interior_subset(b),
            }
        }

        /// Whether every member lies strictly left of `other`, with both nonempty.
        pub fn strictly_left_of(&self, other: &Self) -> bool {
            match (self.bounds, other.bounds) {
                (Some(a), Some(b)) => a.left_of(b),
                _ => false,
            }
        }

        /// Whether the sets share no point. Empty is disjoint from itself.
        pub fn disjoint(&self, other: &Self) -> bool {
            match (self.bounds, other.bounds) {
                (Some(a), Some(b)) => a.left_of(b) || b.left_of(a),
                _ => true,
            }
        }

        /// Whether the sets share at least one point, including a common endpoint.
        pub fn overlaps(&self, other: &Self) -> bool {
            !self.disjoint(other)
        }

        /// Smallest interval of this family containing both sets.
        pub fn hull(&self, other: &Self) -> Self {
            Self {
                bounds: match (self.bounds, other.bounds) {
                    (Some(a), Some(b)) => Some(a.hull(b)),
                    (a, b) => a.or(b),
                },
            }
        }

        /// Exact intersection; disjoint intervals produce explicit empty.
        pub fn intersection(&self, other: &Self) -> Self {
            Self {
                bounds: match (self.bounds, other.bounds) {
                    (Some(a), Some(b)) => a.intersection(b),
                    _ => None,
                },
            }
        }

        /// Signs of all represented members; no scalar predicate is decided.
        pub fn signs(&self) -> crate::SignSet {
            self.bounds
                .map_or(crate::SignSet::Empty, |b| b.signs(I::ZERO))
        }
    };
}

pub(crate) use set_relations;
