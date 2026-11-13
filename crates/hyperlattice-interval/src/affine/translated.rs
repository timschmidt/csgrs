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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TranslatedBox<I: IntervalArithmetic, const N: usize> {
    center: Option<IntervalVector<I, N>>,
    parameters: IntervalVector<I, N>,
}

impl<I: IntervalArithmetic, const N: usize> CenteredVector<I, N> {
    /// Converts this exact definition to an affine source with signed offset
    /// parameters, preserving this object on failure. Copies and arithmetic
    /// share the context. No absolute endpoint `center +/- radius` is computed.
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

impl<I: IntervalArithmetic, const N: usize> TranslatedBox<I, N> {
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

impl<I: IntervalArithmetic, const N: usize> AffineSource<I, N> for TranslatedBox<I, N> {
    type Parameters = IntervalVector<I, N>;
    fn parameter_domain(&self) -> &Self::Parameters {
        self.parameter_domain()
    }
    fn is_empty(&self) -> bool {
        self.is_empty()
    }
    fn enclosure(
        &self,
        context: &mut I::Context,
    ) -> Result<IntervalVector<I, N>, IntervalLatticeError> {
        self.enclosure(context)
    }
    fn restrict_parameters(
        &self,
        restriction: &Self::Parameters,
        context: &mut I::Context,
    ) -> Result<Self, IntervalLatticeError> {
        self.restrict_parameters(restriction, context)
    }
}

impl<I: IntervalArithmetic, const N: usize> sealed::Pullback<I, N> for TranslatedBox<I, N> {
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
mod serialization {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    impl<I: IntervalArithmetic + Serialize, const N: usize> Serialize for TranslatedBox<I, N> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            (&self.center, &self.parameters).serialize(serializer)
        }
    }
    impl<'de, I: IntervalArithmetic + Deserialize<'de>, const N: usize> Deserialize<'de>
        for TranslatedBox<I, N>
    {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            let (center, parameters) = Deserialize::deserialize(deserializer)?;
            Self::new(center, parameters).map_err(serde::de::Error::custom)
        }
    }
}
