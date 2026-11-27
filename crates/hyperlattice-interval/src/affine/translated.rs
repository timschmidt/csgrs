//! Retained translation plus an offset box, avoiding absolute-bound projection.
//!
//! This source lets affine composition cancel or rescale an otherwise overflowing
//! authored definition. Restrictions select offset parameters while retaining the
//! center; projection and pullback leave that exact definition authoritative.

use super::{AffineSource, sealed};
use crate::{
    CenteredVector, IntervalArithmetic, IntervalLatticeError, IntervalMatrix, IntervalVector,
    array_result,
};

/// Exact translated parameter box `{ center + u | u in parameter_domain }`.
///
/// Constructed from an authored [`CenteredVector`] without projecting its
/// absolute bounds. Initially the offsets are `[-radius, radius]`. Restrictions
/// act on these offsets, not absolute coordinates. The center is retained even
/// when restriction removes all parameters; a canonical empty source may have
/// no center. Equality compares definitions, not geometric image equality.
///
/// Static affine images and bounded chains compose before evaluating this
/// source. No identity matrix or representable scalar one is required.
/// Replay stores the current center and parameter domain, not an original radius
/// or restriction history. It validates this definition without requiring symmetry
/// of the parameter box. Cloning follows the carriers without context accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TranslatedBox<I: IntervalArithmetic, const N: usize> {
    /// Singleton authored translation; absence requires an empty parameter domain.
    /// A retained center is allowed even after restriction empties that domain.
    center: Option<IntervalVector<I, N>>,
    /// Current independent signed offsets, which determine emptiness of the image.
    parameters: IntervalVector<I, N>,
}

/// Convert an authored center/radius definition into a reusable affine source.
impl<I: IntervalArithmetic, const N: usize> CenteredVector<I, N> {
    /// Converts this exact definition to an affine source with signed offset
    /// parameters, preserving this object on failure. Copies and arithmetic
    /// share the context. No absolute endpoint `center +/- radius` is computed.
    /// First copy the center with a hull against empty, then negate each radius
    /// in lane order, then hull the negative and positive radius products.
    /// The first error stops construction without refunding prior charges.
    /// Empty and zero-dimensional unit sources require no scalar operations.
    pub fn affine_source(
        &self,
        context: &mut I::Context,
    ) -> Result<TranslatedBox<I, N>, IntervalLatticeError> {
        let Some(center) = self.center() else {
            return TranslatedBox::new(None, IntervalVector::empty()?);
        };
        let center = center.hull(&IntervalVector::empty()?, context)?;
        let radius = self.radius().expect("nonempty authored definition");
        let radii = radius.lanes().expect("singleton radius");
        let negative = IntervalVector::new(array_result(|i| radii[i].neg(context))?)?;
        let parameters = negative.hull(radius, context)?;
        TranslatedBox::new(Some(center), parameters)
    }
}

/// Validate, inspect, restrict and explicitly project retained translated boxes.
impl<I: IntervalArithmetic, const N: usize> TranslatedBox<I, N> {
    /// Accept a singleton center with any valid parameter box, or no center with empty.
    ///
    /// Products already validate scalar families and dimensions. A present empty
    /// or nonsingleton center fails even when parameters are empty; absent center
    /// with nonempty parameters also reports NonPointCenter. No symmetry, original
    /// radius, endpoint materialization or restriction-history proof is required.
    fn new(
        center: Option<IntervalVector<I, N>>,
        parameters: IntervalVector<I, N>,
    ) -> Result<Self, IntervalLatticeError> {
        match &center {
            Some(center) if !center.is_singleton() => {
                return Err(IntervalLatticeError::NonPointCenter);
            }
            None if !parameters.is_empty() => return Err(IntervalLatticeError::NonPointCenter),
            _ => {}
        }
        Ok(Self { center, parameters })
    }

    /// Exact authored translation, absent only for a canonical empty source.
    pub const fn center(&self) -> Option<&IntervalVector<I, N>> {
        self.center.as_ref()
    }

    /// Current signed offset parameters, including any applied restrictions.
    pub const fn parameter_domain(&self) -> &IntervalVector<I, N> {
        &self.parameters
    }

    /// Whether this exact translated set is empty.
    pub const fn is_empty(&self) -> bool {
        self.parameters.is_empty()
    }

    /// Intersects the signed offsets, retaining the exact center. This is not
    /// an intersection with a box in absolute image coordinates.
    /// Parameter intersection precedes the budgeted center copy, which is still
    /// performed for a retained center when the intersection is empty. Thus a
    /// mathematically empty result can fail on copy resources. One shared context
    /// covers both phases; failure leaves this source unchanged without a refund.
    pub fn restrict_parameters(
        &self,
        restriction: &IntervalVector<I, N>,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        let parameters = self.parameters.intersection(restriction, context)?;
        let center = match &self.center {
            Some(center) => Some(center.hull(&IntervalVector::empty()?, context)?),
            None => None,
        };
        Self::new(center, parameters)
    }

    /// Explicit outward projection of the translated set. Failure leaves the
    /// exact definition intact; subsequent maps can still consume it.
    /// Adds the center to current offsets in lane order with the shared context.
    /// Both empty forms give canonical empty without scalar work, and a nonempty
    /// zero-dimensional source gives the unit product. Backend rounding and typed
    /// scalar failures remain visible; no new midpoint/radius definition is inferred.
    pub fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        match &self.center {
            Some(center) => center.add(&self.parameters, context),
            None => IntervalVector::empty(),
        }
    }
}

/// Expose the same offset domain and projection through the sealed source interface.
impl<I: IntervalArithmetic, const N: usize> AffineSource<I, N> for TranslatedBox<I, N> {
    /// Signed offset box, unchanged in type by later affine maps.
    type Parameters = IntervalVector<I, N>;
    /// Borrow the current offset domain through the inherent accessor.
    fn parameter_domain(&self) -> &Self::Parameters {
        self.parameter_domain()
    }
    /// Delegate to parameter-domain emptiness, independently of retained center presence.
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    /// Delegate explicit center-plus-offset enclosure without changing its resource order.
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.enclosure(context)
    }
    /// Delegate offset intersection and subsequent center copy with the shared context.
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.restrict_parameters(restriction, context)
    }
}

/// Terminate affine pullback at the retained translation and original offset domain.
impl<I: IntervalArithmetic, const N: usize> sealed::Pullback<I, N> for TranslatedBox<I, N> {
    /// Enclose `L*(center + u) + t` as `L*u + (L*center + t)`.
    ///
    /// Empty parameters return empty before either affine evaluation. Otherwise
    /// finish the shifted translation first, then apply the same coefficient
    /// bounds to offsets with that shift. These are enclosure computations, so
    /// intermediate rounded coefficient bounds may widen the result. One context
    /// covers both phases; neither temporary bounds nor failures replace the source.
    fn enclose_affine<const R: usize>(
        &self,
        linear: &IntervalMatrix<I, R, N>,
        translation: &IntervalVector<I, R>,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, R>, IntervalLatticeError> {
        if self.is_empty() {
            return IntervalVector::empty();
        }
        let center = self.center.as_ref().expect("nonempty translated source");
        let shifted = linear.affine_enclosure(center, translation, context)?;
        linear.affine_enclosure(&self.parameters, &shifted, context)
    }
}

#[cfg(feature = "serde")]
/// Native replay of current center and offset domain, with constructor validation.
mod serialization {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    /// Serialize the two retained fields without endpoint materialization.
    impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for TranslatedBox<I, N> {
        /// Encode a borrowed pair; both centerless and center-retaining empties persist.
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            (&self.center, &self.parameters).serialize(serializer)
        }
    }
    /// Decode native products and restore translated-source invariants.
    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
        for TranslatedBox<I, N>
    {
        /// Validate scalar bounds/dimensions during decoding, then center/domain consistency.
        /// Constructor failures become Serde errors. No arithmetic context, projection,
        /// original radius or serialized validity flag participates in admission.
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            let (center, parameters) = Deserialize::deserialize(deserializer)?;
            Self::new(center, parameters).map_err(serde::de::Error::custom)
        }
    }
}
