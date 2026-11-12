use crate::{IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector};

// One internal traversal. Coefficients may acquire outward bounds during
// pullback, but those temporary bounds never replace the stored point maps.
mod sealed {
    use super::*;
    pub trait Pullback<I: IntervalArithmetic, const N: usize> {
        fn enclose_affine<const R: usize>(
            &self,
            linear: &IntervalMatrix<I, R, N>,
            translation: &IntervalVector<I, R>,
            context: &mut I::Context,
        ) -> Result<IntervalVector<I, R>, IntervalLatticeError>;
    }
}

/// A sealed source for exact affine images: a box or another affine image.
///
/// `Parameters` is always the original source box's type. Restriction acts in
/// that parameter space; it does not intersect the output with an output box.
pub trait AffineSource<I: IntervalArithmetic, const N: usize>: sealed::Pullback<I, N> {
    /// Original parameter-box type, unchanged by subsequent maps.
    type Parameters;
    /// Original parameter domain, including any applied restrictions.
    fn parameter_domain(&self) -> &Self::Parameters;
    /// Whether the exact image is empty. A total affine map of a nonempty set
    /// stays nonempty, including singular maps and zero-dimensional outputs.
    fn is_empty(&self) -> bool;
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
    linear: IntervalMatrix<I, R, C>,
    translation: IntervalVector<I, R>,
}

impl<I: IntervalArithmetic, const R: usize, const C: usize> AffineMap<I, R, C> {
    /// Validates that both carriers contain exactly one point matrix/vector.
    /// Zero-sized unit products are valid; explicit empty carriers are not.
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
    source: S,
    map: AffineMap<I, R, C>,
}

/// Exact affine image of a Cartesian box, before any further retained maps.
pub type AffineBox<I, const R: usize, const C: usize> = AffineImage<I, IntervalVector<I, C>, R, C>;

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
    pub fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        self.source
            .enclose_affine(&self.map.linear, &self.map.translation, context)
    }
}

impl<I: IntervalArithmetic, const N: usize> AffineSource<I, N> for IntervalVector<I, N> {
    type Parameters = Self;
    fn parameter_domain(&self) -> &Self {
        self
    }
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn restrict_parameters(
        &self,
        restriction: &Self,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.intersection(restriction, context)
    }
}

impl<I: IntervalArithmetic, const N: usize> sealed::Pullback<I, N> for IntervalVector<I, N> {
    fn enclose_affine<const R: usize>(
        &self,
        linear: &IntervalMatrix<I, R, N>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        linear.affine_enclosure(self, translation, context)
    }
}

impl<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
    AffineSource<I, R> for AffineImage<I, S, R, C>
{
    type Parameters = S::Parameters;
    fn parameter_domain(&self) -> &Self::Parameters {
        self.parameter_domain()
    }
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.restrict_parameters(restriction, context)
    }
}

impl<I: IntervalArithmetic, S: AffineSource<I, C>, const R: usize, const C: usize>
    sealed::Pullback<I, R> for AffineImage<I, S, R, C>
{
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
