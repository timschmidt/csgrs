#[cfg(any(
    feature = "integer",
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
use crate::{IntervalError, SignSet};

// Private generic machinery: downstream types cannot use it to assert
// certified endpoint semantics. Public backends seal their endpoint traits.
// All admitted endpoints are totally ordered; Rational exposes that exact
// ordering through PartialOrd rather than implementing Rust's Ord trait.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Bounds<T> {
    lower: T,
    upper: T,
}

impl<T: PartialOrd> Bounds<T> {
    #[cfg(any(
        feature = "integer",
        feature = "fixed",
        feature = "rational",
        feature = "dyadic",
        feature = "float-enclosure"
    ))]
    pub(crate) fn new(lower: T, upper: T) -> Result<Self, IntervalError> {
        if lower > upper {
            Err(IntervalError::ReversedEndpoints)
        } else {
            Ok(Self { lower, upper })
        }
    }

    #[cfg(any(
        feature = "integer",
        feature = "fixed",
        feature = "rational",
        feature = "dyadic",
        feature = "float-enclosure"
    ))]
    pub(crate) fn point(value: T) -> Self
    where
        T: Clone,
    {
        Self {
            lower: value.clone(),
            upper: value,
        }
    }

    pub(crate) fn endpoints(self) -> (T, T) {
        (self.lower, self.upper)
    }

    #[cfg(any(feature = "rational", feature = "dyadic"))]
    pub(crate) fn as_refs(&self) -> Bounds<&T> {
        Bounds {
            lower: &self.lower,
            upper: &self.upper,
        }
    }

    // Callers must establish order algebraically before using this private
    // constructor. Public ingress always uses new().
    #[cfg(any(
        feature = "rational",
        feature = "float-proposal",
        feature = "float-enclosure"
    ))]
    pub(crate) fn ordered(lower: T, upper: T) -> Self {
        Self { lower, upper }
    }
}

#[cfg(feature = "rational")]
impl<T: Clone> Bounds<&T> {
    pub(crate) fn cloned(self) -> Bounds<T> {
        Bounds {
            lower: self.lower.clone(),
            upper: self.upper.clone(),
        }
    }
}

#[cfg(any(
    feature = "integer",
    feature = "fixed",
    feature = "rational",
    feature = "dyadic",
    feature = "float-enclosure"
))]
impl<T: Copy + PartialOrd> Bounds<T> {
    pub(crate) fn is_singleton(self) -> bool {
        self.lower == self.upper
    }

    // Monotonicity in each argument gives the exact image of min/max over
    // independent closed intervals. These are images, not intersection/hull.
    pub(crate) fn min_image(self, other: Self) -> Self {
        Self {
            lower: if self.lower <= other.lower {
                self.lower
            } else {
                other.lower
            },
            upper: if self.upper <= other.upper {
                self.upper
            } else {
                other.upper
            },
        }
    }

    pub(crate) fn max_image(self, other: Self) -> Self {
        Self {
            lower: if self.lower >= other.lower {
                self.lower
            } else {
                other.lower
            },
            upper: if self.upper >= other.upper {
                self.upper
            } else {
                other.upper
            },
        }
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
            lower: if self.lower <= other.lower {
                self.lower
            } else {
                other.lower
            },
            upper: if self.upper >= other.upper {
                self.upper
            } else {
                other.upper
            },
        }
    }

    pub(crate) fn intersection(self, other: Self) -> Option<Self> {
        Self::new(
            if self.lower >= other.lower {
                self.lower
            } else {
                other.lower
            },
            if self.upper <= other.upper {
                self.upper
            } else {
                other.upper
            },
        )
        .ok()
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
#[cfg(any(feature = "integer", feature = "fixed"))]
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

        /// Exact image of `min(x, y)` for independently chosen members.
        /// Either empty operand gives empty; no domain points are discarded.
        pub fn min_image(&self, other: &Self) -> Self {
            Self {
                bounds: match (self.bounds, other.bounds) {
                    (Some(a), Some(b)) => Some(a.min_image(b)),
                    _ => None,
                },
            }
        }

        /// Exact image of `max(x, y)` for independently chosen members.
        /// Either empty operand gives empty; this differs from hull.
        pub fn max_image(&self, other: &Self) -> Self {
            Self {
                bounds: match (self.bounds, other.bounds) {
                    (Some(a), Some(b)) => Some(a.max_image(b)),
                    _ => None,
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

#[cfg(any(feature = "integer", feature = "fixed"))]
pub(crate) use set_relations;
