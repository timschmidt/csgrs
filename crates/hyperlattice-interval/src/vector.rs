use crate::{IntervalArithmetic, IntervalLatticeError, array_result};

/// A Cartesian product of `N` finite real intervals, or the canonical empty set.
///
/// Any empty lane makes the whole product empty. With zero lanes, `new([])` is
/// the singleton unit product, distinct from `empty()`. Coordinates are private
/// and immutable; deserialization validates scalar values and canonicalizes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntervalVector<I: IntervalArithmetic, const N: usize> {
    pub(crate) lanes: Option<[I; N]>,
}

impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Constructs a product from validated scalar intervals (singletons lift points).
    /// Type-level constraints are checked even when `N == 0`.
    pub fn new(lanes: [I; N]) -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self {
            lanes: if lanes.iter().any(I::is_empty) {
                None
            } else {
                Some(lanes)
            },
        })
    }

    /// Constructs the empty set after validating the scalar family.
    pub fn empty() -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self { lanes: None })
    }

    /// Whether the represented Cartesian set is empty.
    pub const fn is_empty(&self) -> bool {
        self.lanes.is_none()
    }

    /// Whether the Cartesian set contains exactly one vector (including `()`).
    pub fn is_singleton(&self) -> bool {
        self.lanes
            .as_ref()
            .is_some_and(|lanes| lanes.iter().all(I::is_singleton))
    }

    /// Borrowed lanes, or `None` for the canonical empty set.
    pub const fn lanes(&self) -> Option<&[I; N]> {
        self.lanes.as_ref()
    }

    /// Transfers owned lanes without copying scalar data.
    pub fn into_lanes(self) -> Option<[I; N]> {
        self.lanes
    }

    /// Smallest Cartesian box containing both inputs; not their general union.
    pub fn hull(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        match (&self.lanes, &other.lanes) {
            (None, None) => Self::empty(),
            (Some(lanes), None) | (None, Some(lanes)) => {
                Self::new(array_result(|i| lanes[i].copy(context))?)
            }
            (Some(a), Some(b)) => Self::new(array_result(|i| a[i].hull(&b[i], context))?),
        }
    }

    /// Exact Cartesian intersection, canonicalizing any empty component.
    pub fn intersection(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::intersection)
    }

    /// Whether this set is included in the other set. Empty is always included.
    pub fn subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::subset_of)
    }

    /// Inclusion in the other's real interior. In dimension zero the unit
    /// product is open; in positive dimensions every lane needs strict inclusion.
    pub fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::interior_subset_of)
    }

    /// Coordinate sum enclosure. Empty operands have empty images.
    pub fn add(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::add)
    }

    /// Coordinate difference enclosure, treating the two inputs independently.
    pub fn sub(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::sub)
    }

    /// Dot-product enclosure with left-to-right multiplication then addition.
    /// The same caller budget is used for every scalar operation. The empty
    /// Cartesian set has empty image; a zero-dimensional unit dot product is zero.
    pub fn dot(&self, other: &Self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        match (&self.lanes, &other.lanes) {
            (Some(a), Some(b)) => Ok(crate::dot(a.iter().zip(b), context)?),
            _ => Ok(I::empty()?),
        }
    }

    fn zip(
        &self,
        other: &Self,
        context: &mut I::Context,
        mut op: impl FnMut(&I, &I, &mut I::Context) -> Result<I, hyperinterval::IntervalError>,
    ) -> Result<Self, IntervalLatticeError> {
        match (&self.lanes, &other.lanes) {
            (Some(a), Some(b)) => Self::new(array_result(|i| op(&a[i], &b[i], context))?),
            _ => Self::empty(),
        }
    }

    fn relation(
        &self,
        other: &Self,
        context: &mut I::Context,
        mut op: impl FnMut(&I, &I, &mut I::Context) -> Result<bool, hyperinterval::IntervalError>,
    ) -> Result<bool, IntervalLatticeError> {
        match (&self.lanes, &other.lanes) {
            (None, _) => Ok(true),
            (Some(_), None) => Ok(false),
            (Some(a), Some(b)) => {
                for (a, b) in a.iter().zip(b) {
                    if !op(a, b, context)? {
                        return Ok(false);
                    }
                }
                Ok(true)
            }
        }
    }
}
