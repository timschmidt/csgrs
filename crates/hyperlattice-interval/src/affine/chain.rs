//! Runtime affine composition retaining a bounded inline sequence of point maps.
//!
//! Storage follows application order; projection traverses that sequence backward
//! to compose coefficients before evaluating the original parameter source. Map
//! capacity is independent of scalar size/work limits and source nesting depth.

use super::{AffineMap, AffineSource, sealed};
use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector};

/// An exact affine image with a runtime map count and inline capacity `CAP`.
///
/// Maps apply in insertion order and retain the source's original parameters.
/// All maps in this chain have dimension `N`; rectangular maps can precede or
/// follow it through [`AffineMap::image`]. Zero maps denote the source itself,
/// without constructing identity coefficients (which some scales cannot hold).
/// Structural equality compares definitions, not geometric images.
///
/// Capacity limits retained maps, not scalar complexity, heap bytes or time.
/// Primitive storage is inline and queries allocate nothing. Exact backends
/// keep their scalar work/size contract; explicit clones and serde are caller
/// work. The number of matrix/vector scratch carriers does not grow with this
/// chain's map count, but their exact scalar storage can grow. Nested source
/// chains still add traversal stack frames. No cache is retained.
///
/// ```
/// # #[cfg(feature = "fixed")] {
/// use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};
/// use hyperlattice_interval::hyperinterval::FixedInterval;
/// type I = FixedInterval<i32, 0>;
/// let p = |x| I::point(x).unwrap();
/// let source = IntervalVector::new([I::new(-1, 1).unwrap()]).unwrap();
/// let mut image = AffineChain::<_, _, 1, 8>::new(source);
/// for _ in 0..3 {
///     image.push(AffineMap::new(
///         IntervalMatrix::new([[p(2)]]).unwrap(),
///         IntervalVector::new([p(1)]).unwrap(),
///     ).unwrap()).unwrap();
/// }
/// assert_eq!(image.enclosure(&mut ()).unwrap().lanes(), Some(&[I::new(-1, 15).unwrap()]));
/// # }
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AffineChain<
    I: IntervalArithmetic,
    S: AffineSource<I, N>,
    const N: usize,
    const CAP: usize,
> {
    /// Exact source before the first retained map, including original parameters.
    source: S,
    /// Occupied prefix followed by None slots; validated point maps stay in application order.
    maps: [Option<AffineMap<I, N, N>>; CAP],
    /// Occupied prefix length, always at most CAP; zero does not mean an empty image.
    count: usize,
}

/// Retain maps, inspect ownership and explicitly project or restrict original parameters.
impl<I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>
    AffineChain<I, S, N, CAP>
{
    /// Moves an exact source into an initially zero-map chain without arithmetic.
    /// Initializes all CAP slots to None; this bookkeeping has no scalar context
    /// and makes no new identity coefficients or source projection.
    pub fn new(source: S) -> Self {
        Self {
            source,
            maps: core::array::from_fn(|_| None),
            count: 0,
        }
    }

    /// Appends an exact point map without arithmetic or copying its scalars.
    /// On capacity exhaustion the chain is unchanged and the moved map is dropped.
    /// Use [`Self::into_parts`] to rebuild a full chain with a larger capacity.
    pub fn push(&mut self, map: AffineMap<I, N, N>) -> Result<(), IntervalLatticeError> {
        if self.count == CAP {
            return Err(IntervalLatticeError::AffineCapacityExhausted { capacity: CAP });
        }
        self.maps[self.count] = Some(map);
        self.count += 1;
        Ok(())
    }

    /// Maximum retained map count, fixed by this type.
    pub const fn capacity(&self) -> usize {
        CAP
    }

    /// Number of retained maps; zero is distinct from an empty represented set.
    pub const fn map_count(&self) -> usize {
        self.count
    }

    /// Immediate exact source before this chain's maps.
    pub const fn source(&self) -> &S {
        &self.source
    }

    /// Exact immutable point maps in application order.
    /// Only the occupied prefix is exposed. The iterator also supports reverse
    /// traversal and exact length; construction/push preserve its expect invariant.
    pub fn maps(&self) -> impl DoubleEndedIterator<Item = &AffineMap<I, N, N>> + ExactSizeIterator {
        self.maps[..self.count]
            .iter()
            .map(|map| map.as_ref().expect("occupied prefix"))
    }

    /// Transfers the source and maps in application order, without scalar copies.
    /// The returned iterator owns the inline slot array and its remaining maps.
    /// Dropping it drops unconsumed maps; no compact heap sequence is constructed.
    pub fn into_parts(
        self,
    ) -> (
        S,
        impl DoubleEndedIterator<Item = AffineMap<I, N, N>> + ExactSizeIterator,
    ) {
        (
            self.source,
            self.maps
                .into_iter()
                .take(self.count)
                .map(|map| map.expect("occupied prefix")),
        )
    }

    /// Original source parameter domain, including applied restrictions.
    pub fn parameter_domain(&self) -> &S::Parameters {
        self.source.parameter_domain()
    }

    /// Whether the represented set is empty, independently of its map count.
    pub fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Intersects the original parameters and retains all maps. Scalar copies
    /// share the context; any failure leaves this chain reusable and unchanged.
    /// Restrict the source first, then copy maps in application order even if
    /// the restricted domain is empty. Each map copies its matrix before its
    /// translation. The result has the same capacity, sufficient for the original
    /// occupied prefix; previous scalar charges are not refunded on failure.
    pub fn restrict_parameters(
        &self,
        restriction: &S::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        let mut result = Self::new(self.source.restrict_parameters(restriction, context)?);
        for map in self.maps() {
            result.push(map.copy(context)?)?;
        }
        Ok(result)
    }

    /// Explicit Cartesian projection, with the same rounding/overflow contract
    /// as [`super::AffineImage::enclosure`]. Exact maps and correlations survive
    /// failed queries. A zero-map chain delegates to its source's enclosure; a
    /// Cartesian source is copied through the arithmetic context. No artificial
    /// identity or unbudgeted clone is used. Otherwise borrow the last map as
    /// initial coefficients and pull it through the preceding maps in reverse order.
    pub fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match self.maps().next_back() {
            None => self.source.enclosure(context),
            Some(last) => {
                self.enclose_prefix(self.count - 1, last.linear(), last.translation(), context)
            }
        }
    }

    /// Pull external coefficient bounds through a prefix, then evaluate the source.
    ///
    /// Internal callers supply count at most map_count; take selects that prefix
    /// without separate range validation. Empty sources return empty first. Visit
    /// maps backward, computing each combined matrix before its translation with
    /// one shared context. The first step borrows incoming bounds; later steps
    /// retain only the current pair while constructing the next, not all prefixes.
    /// With no selected maps, pass the incoming bounds directly to the source.
    /// Scalar errors discard temporaries without changing stored maps or refunding work.
    fn enclose_prefix<const R: usize>(
        &self,
        count: usize,
        linear: &IntervalMatrix<I, R, N>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        if self.is_empty() {
            return IntervalVector::empty();
        }
        let mut combined = None;
        for map in self.maps().take(count).rev() {
            let (linear, translation) = match &combined {
                None => (linear, translation),
                Some((a, b)) => (a, b),
            };
            // Same pullback as a static image: matrix first, then translation.
            // Temporary bounds never replace any retained exact map.
            let next_linear = linear.mul_matrix(map.linear(), context)?;
            let next_translation =
                linear.affine_enclosure(map.translation(), translation, context)?;
            combined = Some((next_linear, next_translation));
        }
        let (linear, translation) = match &combined {
            None => (linear, translation),
            Some((a, b)) => (a, b),
        };
        self.source.enclose_affine(linear, translation, context)
    }
}

/// Expose runtime chains through the same exact-source contract as static images.
impl<I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>
    AffineSource<I, N> for AffineChain<I, S, N, CAP>
{
    /// Original source parameter type, unchanged by this chain's maps.
    type Parameters = S::Parameters;
    /// Delegate immutable parameter access to the retained source.
    fn parameter_domain(&self) -> &Self::Parameters {
        self.parameter_domain()
    }
    /// Delegate represented-set emptiness, independently of capacity or occupied length.
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    /// Delegate projection with the same reverse traversal and context accounting.
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.enclosure(context)
    }
    /// Delegate parameter intersection and application-order map copies.
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.restrict_parameters(restriction, context)
    }
}

/// Allow outer static or runtime maps to consume the retained chain before projection.
impl<I: IntervalArithmetic, S: AffineSource<I, N>, const N: usize, const CAP: usize>
    sealed::Pullback<I, N> for AffineChain<I, S, N, CAP>
{
    /// Pull the incoming bounds through every occupied map, preserving source parameters.
    /// Rectangular outer maps are allowed even though this chain's stored maps are square.
    fn enclose_affine<const R: usize>(
        &self,
        linear: &IntervalMatrix<I, R, N>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        self.enclose_prefix(self.count, linear, translation, context)
    }
}
