//! Exact direction certificates shared by metric curve constructions.

use hyperreal::{Real, RealSign};

use crate::classify::real_sign;
use crate::{Classification, CurveContext, CurveError, CurveResult, UncertaintyReason};

/// Unit length follows from the normalization construction. Consumers reuse
/// that proof instead of expanding radical coordinates to prove it again.
pub(crate) struct UnitDirection2 {
    components: (Real, Real),
}

impl UnitDirection2 {
    pub(crate) fn from_direction(direction: &(Real, Real)) -> CurveResult<Classification<Self>> {
        let length_squared = &direction.0 * &direction.0 + &direction.1 * &direction.1;
        match real_sign(&length_squared, &CurveContext::STRICT) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an exact direction had negative squared length".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let inverse_length = length_squared.sqrt()?.inverse_ref_assuming_nonzero()?;
        Ok(Classification::Decided(Self {
            components: (
                &direction.0 * &inverse_length,
                &direction.1 * inverse_length,
            ),
        }))
    }

    pub(crate) fn components(&self) -> &(Real, Real) {
        &self.components
    }
}
