//! Cartesian vector sets with canonical emptiness and ordered scalar delegation.
//!
//! This carrier stores coordinate intervals, not dependencies between coordinates.
//! Set relations and arithmetic use the sealed scalar backend; the shared helpers
//! preserve first-error and first-false behavior without adding a predicate policy.

use crate::{IntervalArithmetic, IntervalLatticeError, array_result};

/// A Cartesian product of `N` finite real intervals, or the canonical empty set.
///
/// Any empty lane makes the whole product empty. With zero lanes, `new([])` is
/// the singleton unit product, distinct from `empty()`. Coordinates are private
/// and immutable; deserialization validates scalar values and canonicalizes.
///
/// Arithmetic treats operands independently, even when both references identify
/// the same vector. Correlations lost by an enclosure are not reconstructed by a
/// later operation. Derived equality compares stored backend values; it does not
/// compare the source definitions from which two boxes might have been projected.
///
/// Storage is inline in N. Scalar work and allocation depend on the backend;
/// dyadic operations share the supplied context, while explicit Clone has no
/// context. Fallible operations return no partial vector and do not refund work
/// already charged. Borrowed inputs remain available for a fresh-budget retry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IntervalVector<I: IntervalArithmetic, const N: usize> {
    /// None is the canonical empty set; Some contains exactly N nonempty lanes.
    /// Some([]) retains the zero-dimensional unit product. Internal users must
    /// preserve this constructor invariant; public access does not expose mutation.
    pub(crate) lanes: Option<[I; N]>,
}

/// Construction, set relations and coordinate arithmetic on fixed-dimensional products.
impl<I: IntervalArithmetic, const N: usize> IntervalVector<I, N> {
    /// Constructs a product from validated scalar intervals (singletons lift points).
    /// Type-level constraints are checked even when `N == 0`.
    /// Takes ownership without scalar copies or arithmetic. After family validation,
    /// any empty lane discards the array and produces canonical empty storage.
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
    /// This performs no per-coordinate arithmetic or caller-context reservation.
    pub fn empty() -> Result<Self, IntervalLatticeError> {
        I::validate_family()?;
        Ok(Self { lanes: None })
    }

    /// Whether the represented Cartesian set is empty.
    /// Reads the canonical storage tag without scalar predicates or arithmetic.
    pub const fn is_empty(&self) -> bool {
        self.lanes.is_none()
    }

    /// Whether the Cartesian set contains exactly one vector (including `()`).
    /// Empty is false; nonempty lanes are inspected in order until a nonsingleton.
    pub fn is_singleton(&self) -> bool {
        self.lanes
            .as_ref()
            .is_some_and(|lanes| lanes.iter().all(I::is_singleton))
    }

    /// Borrowed lanes, or `None` for the canonical empty set.
    /// A unit product yields Some of an empty array; no scalar copy or projection occurs.
    pub const fn lanes(&self) -> Option<&[I; N]> {
        self.lanes.as_ref()
    }

    /// Transfers owned lanes without copying scalar data.
    /// Consumes the carrier; None and Some([]) keep empty and unit products distinct.
    pub fn into_lanes(self) -> Option<[I; N]> {
        self.lanes
    }

    /// Smallest Cartesian box containing both inputs; not their general union.
    ///
    /// One empty operand causes an accounted copy of every nonempty operand lane,
    /// so a geometric identity can still exhaust dyadic resources. Two empty inputs
    /// return empty without scalar copies. Otherwise hulls run in increasing index
    /// order and return the first error; no work is refunded.
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
    ///
    /// An already-empty operand skips coordinate work. For two nonempty operands,
    /// all lanes are intersected in order before successful canonicalization; an
    /// empty intermediate lane does not suppress a later scalar resource error.
    pub fn intersection(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::intersection)
    }

    /// Whether this set is included in the other set. Empty is always included.
    /// Nonempty lanes are checked in index order, stopping at the first false or
    /// scalar error. Empty-set shortcuts and the unit/unit case consume no context.
    pub fn subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::subset_of)
    }

    /// Inclusion in the other's real interior. In dimension zero the unit
    /// product is open; in positive dimensions every lane needs strict inclusion.
    /// Empty is included even in empty. Checks stop at the first false or error;
    /// a scalar failure is never interpreted as non-inclusion.
    pub fn interior_subset_of(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<bool, IntervalLatticeError> {
        self.relation(other, context, I::interior_subset_of)
    }

    /// Coordinate sum enclosure. Empty operands have empty images.
    /// Successful nonempty evaluation visits every lane in order, with one shared
    /// context and backend rounding. It does not retain relationships between inputs.
    pub fn add(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::add)
    }

    /// Coordinate difference enclosure, treating the two inputs independently.
    /// In particular, subtracting a nonsingleton box from itself need not give zero.
    /// Empty operands give empty; lane order and the first scalar error are preserved.
    pub fn sub(
        &self,
        other: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.zip(other, context, I::sub)
    }

    /// Enclose `{ s*x : s in scalar, x in self }` coordinate by coordinate.
    ///
    /// A shared scalar can correlate coordinates; this returns their Cartesian
    /// enclosure, not an exact correlated image. Empty operands give empty,
    /// including dimension zero. A nonempty scalar times the unit product is
    /// the unit product. Every multiplication shares the caller's context.
    /// Lanes are multiplied in increasing index order; the first error aborts
    /// construction without changing the source or returning partial coordinates.
    pub fn scale(
        &self,
        scalar: &I,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        let Some(lanes) = self.lanes() else {
            return Self::empty();
        };
        if scalar.is_empty() {
            return Self::empty();
        }
        Self::new(array_result(|i| lanes[i].mul(scalar, context))?)
    }

    /// Dot-product enclosure with left-to-right multiplication then addition.
    /// The same caller budget is used for every scalar operation. The empty
    /// Cartesian set has empty image; a zero-dimensional unit dot product is zero.
    /// Repeated references do not turn this into a dependency-aware quadratic form:
    /// each interval pair is treated independently, including when self == other.
    pub fn dot(&self, other: &Self, context: &mut I::Context) -> Result<I, IntervalLatticeError> {
        match (&self.lanes, &other.lanes) {
            (Some(a), Some(b)) => Ok(crate::dot(a.iter().zip(b), context)?),
            _ => Ok(I::empty()?),
        }
    }

    /// Lift a scalar binary operation over matching nonempty lanes in index order.
    ///
    /// An empty operand bypasses the callback and constructs validated empty storage.
    /// Otherwise array_result returns the first callback error, and Self::new runs
    /// only after all lanes succeed. A callback's empty result does not short-circuit
    /// later lanes. The unit/unit case invokes no callback. All calls share context;
    /// callback effects and consumed work survive failure, but no partial array escapes.
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

    /// Conjoin scalar relations with the Cartesian empty-set rules and index order.
    ///
    /// Empty self is included in any operand; nonempty self is not included in empty.
    /// Neither shortcut calls op. Nonempty products stop at the first false or error,
    /// so skipped lanes neither fail nor spend context work. No lanes means true.
    /// This is shared by closed inclusion and real-interior inclusion, not equality.
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
