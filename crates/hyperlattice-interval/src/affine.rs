//! Retained point maps and statically nested affine images of exact sources.
//!
//! Construction keeps source parameters and map coefficients without projecting
//! intermediate images. Enclosure composes coefficient bounds from the outer map
//! toward the source, then delegates evaluation to the existing Cartesian layer.
//! Restrictions act on the source parameters, preserving every retained map.
//! Backend contexts account for scalar operations and explicit owned copies, not
//! a general bound on nesting depth, traversal stack, cloning or serialization.

use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector};

/// Runtime-length affine composition with a fixed inline map capacity.
mod chain;
mod translated;
pub use chain::AffineChain;
pub use translated::TranslatedBox;

// One internal traversal. Coefficients may acquire outward bounds during
// pullback, but those temporary bounds never replace the stored point maps.
/// Internal traversal contract that seals source implementations and shares pullback.
mod sealed {
    use super::*;
    /// Enclose an outer affine map by transporting its coefficient bounds to a source.
    ///
    /// Callers supply bounds arising from validated point-map composition. These
    /// temporary bounds may be nonsingleton after rounding; they are not admitted
    /// as a new exact family of authored maps. Implementations retain the source
    /// definition and use the same context throughout recursive evaluation.
    pub trait Pullback<I: IntervalArithmetic, const N: usize> {
        /// Return an R-coordinate enclosure of `linear * self + translation`.
        /// Preserve source parameter dependence through composition before final
        /// Cartesian projection. Backend errors propagate without partial output;
        /// no midpoint, sampled point or intermediate image box replaces the source.
        fn enclose_affine<const R: usize>(
            &self,
            linear: &IntervalMatrix<I, R, N>,
            translation: &IntervalVector<I, R>,
            context: &mut I::Context,
        ) -> Result<IntervalVector<I, R>, IntervalLatticeError>;
    }
}

/// A sealed source for exact affine images: a box, image or bounded chain.
///
/// `Parameters` is always the original source box's type. Restriction acts in
/// that parameter space; it does not intersect the output with an output box.
/// For [`TranslatedBox`], that domain contains offsets from the retained center.
/// The private pullback supertrait prevents downstream source implementations
/// from introducing a different projection or composition authority.
pub trait AffineSource<I: IntervalArithmetic, const N: usize>: sealed::Pullback<I, N> {
    /// Original parameter-box type, unchanged by subsequent maps.
    type Parameters;
    /// Original parameter domain, including any applied restrictions.
    fn parameter_domain(&self) -> &Self::Parameters;
    /// Whether the exact image is empty. A total affine map of a nonempty set
    /// stays nonempty, including singular maps and zero-dimensional outputs.
    fn is_empty(&self) -> bool;
    /// Explicit Cartesian projection, preserving the exact source definition.
    /// The context accounts for scalar work and owned copies across the query.
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError>;
    /// Intersects the original parameter domain with `restriction`, retaining
    /// all maps. Owned scalar copies use the same arithmetic context. Failure
    /// leaves this source unchanged and available for a larger-budget retry.
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError>
    where
        Self: Sized;
}

/// The exact point map `x -> A*x+b`, with finite singleton coefficients.
///
/// Singleton validation distinguishes a point map from an uncertain family of
/// maps. No arithmetic, rounding or coefficient multiplication occurs on image
/// construction. Fixed/float singleton entries denote their exact binary values.
/// Empty or nonsingleton coefficient carriers are invalid even for empty sources.
///
/// ```
/// # #[cfg(feature = "fixed")] {
/// use hyperlattice_interval::{AffineMap, IntervalMatrix, IntervalVector};
/// use hyperlattice_interval::hyperinterval::FixedInterval;
/// type I = FixedInterval<i32, 0>;
/// let p = |n| I::point(n).unwrap();
/// let duplicate = AffineMap::new(
///     IntervalMatrix::new([[p(1)], [p(1)]]).unwrap(),
///     IntervalVector::new([p(0), p(0)]).unwrap(),
/// ).unwrap();
/// let difference = AffineMap::new(
///     IntervalMatrix::new([[p(1), p(-1)]]).unwrap(),
///     IntervalVector::new([p(0)]).unwrap(),
/// ).unwrap();
/// let source = IntervalVector::new([I::new(-1, 1).unwrap()]).unwrap();
/// let image = difference.image(duplicate.image(source));
/// assert_eq!(image.enclosure(&mut ()).unwrap().lanes(), Some(&[p(0)]));
/// # }
/// ```
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AffineMap<I: IntervalArithmetic, const R: usize, const C: usize> {
    /// Validated singleton R-by-C coefficient matrix, retained in its scalar family.
    linear: IntervalMatrix<I, R, C>,
    /// Validated singleton R-coordinate translation, independent of any later source.
    translation: IntervalVector<I, R>,
}

/// Validate and retain point coefficients, inspect them, or attach an exact source.
impl<I: IntervalArithmetic, const R: usize, const C: usize> AffineMap<I, R, C> {
    /// Validates that both carriers contain exactly one point matrix/vector.
    /// Zero-sized unit products are valid; explicit empty carriers are not.
    /// Matrix validation precedes translation validation. Inputs are consumed,
    /// including on failure; no scalar arithmetic or source-dependent shortcut
    /// participates in admission.
    pub fn new(
        linear: IntervalMatrix<I, R, C>,
        translation: IntervalVector<I, R>,
    ) -> Result<Self, IntervalLatticeError> {
        if !linear.is_singleton() {
            return Err(IntervalLatticeError::NonPointAffineMatrix);
        }
        if !translation.is_singleton() {
            return Err(IntervalLatticeError::NonPointAffineTranslation);
        }
        Ok(Self {
            linear,
            translation,
        })
    }

    /// Exact immutable point matrix.
    pub const fn linear(&self) -> &IntervalMatrix<I, R, C> {
        &self.linear
    }

    /// Exact immutable translation.
    pub const fn translation(&self) -> &IntervalVector<I, R> {
        &self.translation
    }

    /// Transfers both point coefficient carriers without scalar copies.
    pub fn into_parts(self) -> (IntervalMatrix<I, R, C>, IntervalVector<I, R>) {
        (self.linear, self.translation)
    }

    /// Retains the exact affine image of `source`. Applying this to an existing
    /// image preserves its parameters and correlations; no box is substituted.
    /// Storage is inline and ownership is moved, without allocation or arithmetic.
    pub fn image<S: AffineSource<I, C>>(self, source: S) -> AffineImage<I, S, R, C> {
        AffineImage { source, map: self }
    }

    /// Make a budget-accounted owned copy, matrix first and translation second.
    ///
    /// Each hull against empty delegates to carrier copying in its established
    /// entry order. The immutable source already proves singleton invariants,
    /// so no new point-map validation is needed. First-error return drops any
    /// completed prefix without refunding work or changing the source map.
    fn copy(&self, context: &mut I::Context) -> Result<Self, IntervalLatticeError> {
        // Hull with empty is the existing exact, budget-accounted owned copy.
        Ok(Self {
            linear: self.linear.hull(&IntervalMatrix::empty()?, context)?,
            translation: self.translation.hull(&IntervalVector::empty()?, context)?,
        })
    }
}

/// An exact retained image of a finite parameter box under affine maps.
///
/// The represented set is `{ map(x) | x in source }`, with the source's shared
/// parameters retained recursively. This definition can denote off-grid points
/// even when `I` cannot materialize them as singleton intervals. [`Self::enclosure`]
/// is an explicit projection, not a replacement for the exact object.
///
/// Structural equality compares definitions, not geometric image equality.
/// Empty sources retain their maps/provenance; their enclosure is canonical empty.
/// Finite nesting is carried in the Rust type. Stack/storage grow with map sizes
/// and depth; this is not a dynamically sized expression arena. Explicit clones
/// are caller-owned work, while restrictions and queries share scalar accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AffineImage<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
{
    /// Immediate exact source, retaining original parameters through nested images.
    source: S,
    /// Last applied point map; it is not overwritten by temporary composed bounds.
    map: AffineMap<I, R, C>,
}

/// Exact affine image of a Cartesian box, before any further retained maps.
pub type AffineBox<I, const R: usize, const C: usize> = AffineImage<I, IntervalVector<I, C>, R, C>;

/// Immutable definition access, parameter restriction and explicit Cartesian enclosure.
impl<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
    AffineImage<I, S, R, C>
{
    /// Immediate exact source, which may itself retain an affine definition.
    pub const fn source(&self) -> &S {
        &self.source
    }

    /// The last retained point map.
    pub const fn map(&self) -> &AffineMap<I, R, C> {
        &self.map
    }

    /// Transfers the immediate source and map without scalar copies.
    pub fn into_parts(self) -> (S, AffineMap<I, R, C>) {
        (self.source, self.map)
    }

    /// Original source parameter domain, including restrictions.
    pub fn parameter_domain(&self) -> &S::Parameters {
        self.source.parameter_domain()
    }

    /// Whether the exact represented image is empty.
    pub fn is_empty(&self) -> bool {
        self.source.is_empty()
    }

    /// Intersects in original parameter space, retaining the complete definition.
    /// Budgeted copies are made only for the returned object; failure does not
    /// mutate this image or cache exhaustion as a permanent result.
    /// Restrict the immediate source first, then copy this last map. Recursion
    /// therefore copies inner maps before outer maps. Copies still occur when
    /// the restricted domain is empty, so empty results can require resources.
    pub fn restrict_parameters(
        &self,
        restriction: &S::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        Ok(Self {
            source: self.source.restrict_parameters(restriction, context)?,
            map: self.map.copy(context)?,
        })
    }

    /// Encloses the exact image by composing coefficient bounds before applying
    /// them to the original parameter box. No intermediate source-image box is
    /// used. Rational/dyadic success gives the exact coordinate hull. Primitive
    /// scalar rounding can widen it, and primitive overflow remains typed.
    ///
    /// The caller's context covers the entire pullback and final evaluation.
    /// No partial output is returned on exhaustion. The retained definition is
    /// unchanged even if its current projection overflows or exhausts resources.
    /// Even an exact coordinate hull can lose dependence between output lanes;
    /// further maps on the retained image continue to use its shared parameters.
    pub fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        self.source
            .enclose_affine(&self.map.linear, &self.map.translation, context)
    }
}

/// Treat a Cartesian vector product as its own parameterized affine source.
impl<I: IntervalArithmetic, const N: usize> AffineSource<I, N> for IntervalVector<I, N> {
    /// The original vector box, with the same scalar family and dimension.
    type Parameters = Self;
    /// Borrow this box directly as the parameter domain.
    fn parameter_domain(&self) -> &Self {
        self
    }
    /// Use the carrier's canonical empty state without numerical evaluation.
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    /// Copy the box through hull-against-empty with backend accounting.
    /// This identity projection can require dyadic work for nonempty coordinates;
    /// it is not an unbudgeted Clone and does not change the represented box.
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.hull(&Self::empty()?, context)
    }
    /// Delegate parameter restriction to native Cartesian intersection.
    fn restrict_parameters(
        &self,
        restriction: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.intersection(restriction, context)
    }
}

/// End coefficient pullback at a Cartesian parameter domain.
impl<I: IntervalArithmetic, const N: usize> sealed::Pullback<I, N> for IntervalVector<I, N> {
    /// Delegate the final affine enclosure to the existing matrix/vector kernel.
    /// Its empty handling, ordered products/additions and scalar errors remain intact.
    fn enclose_affine<const R: usize>(
        &self,
        linear: &IntervalMatrix<I, R, N>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        linear.affine_enclosure(self, translation, context)
    }
}

/// Preserve the original parameter type while exposing a nested image as a source.
impl<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
    AffineSource<I, R> for AffineImage<I, S, R, C>
{
    /// Source parameters, whose dimension need not equal the image dimension R.
    type Parameters = S::Parameters;
    /// Delegate to the retained source's parameter domain through the inherent API.
    fn parameter_domain(&self) -> &Self::Parameters {
        self.parameter_domain()
    }
    /// Delegate source emptiness; a singular total map does not create an empty set.
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    /// Delegate the same explicit projection and shared-context traversal.
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        self.enclosure(context)
    }
    /// Delegate source restriction and ordered map copying without altering semantics.
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.restrict_parameters(restriction, context)
    }
}

/// Transport an outer affine map through one retained point map toward the source.
impl<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
    sealed::Pullback<I, R> for AffineImage<I, S, R, C>
{
    /// Apply `B*(A*x+b)+c = (B*A)*x+(B*b+c)` to temporary coefficient bounds.
    ///
    /// Empty source returns empty before arithmetic. Otherwise compute the entire
    /// combined matrix first, then the combined translation, then recurse into
    /// the immediate source. Both coefficients and all nested calls share context.
    /// Failure drops temporaries without refunding charges; stored point maps are
    /// never replaced by rounded bounds or by a failed projection.
    fn enclose_affine<const K: usize>(
        &self,
        linear: &IntervalMatrix<I, K, R>,
        translation: &IntervalVector<I, K>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, K>, IntervalLatticeError> {
        if self.is_empty() {
            return IntervalVector::empty();
        }
        // B(Ax+b)+c = (BA)x+(Bb+c). Only the temporary coefficient bounds
        // change; the exact point maps and the shared source remain immutable.
        let combined_linear = linear.mul_matrix(&self.map.linear, context)?;
        let combined_translation =
            linear.affine_enclosure(&self.map.translation, translation, context)?;
        self.source
            .enclose_affine(&combined_linear, &combined_translation, context)
    }
}
