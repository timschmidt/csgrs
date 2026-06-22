//! Scalar import policy for exact mesh data.
//!
//! Primitive floats are accepted only at named lossy input edges. The imported
//! value is stored as an exact dyadic [`hyperreal::Real`], preserving the
//! exact predicate scheduling.

use super::error::{ExactMeshBlocker, ExactMeshBlockerKind};
use hyperreal::Real;

/// A checked primitive-float import into [`hyperreal::Real`].
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LossyF64Import {
    /// Exact dyadic value stored by `hyperreal`.
    pub(crate) value: Real,
}

impl LossyF64Import {
    /// Import one finite `f64` coordinate as an exact dyadic `Real`.
    ///
    /// This is intentionally named "lossy" because the caller's input channel
    /// was a primitive float. Once accepted, the stored `Real` exactly
    /// represents that dyadic value and predicate code must not re-consult
    /// primitive-float tolerances.
    pub(crate) fn new(value: f64, coordinate_index: usize) -> Result<Self, ExactMeshBlocker> {
        if !value.is_finite() {
            return Err(ExactMeshBlocker::new(
                ExactMeshBlockerKind::NonFiniteCoordinate,
                format!("coordinate {coordinate_index} is not finite"),
            )
            .with_coordinate(coordinate_index));
        }

        let real = Real::try_from(value).map_err(|problem| {
            ExactMeshBlocker::new(
                ExactMeshBlockerKind::CoordinateImportFailed,
                format!("coordinate {coordinate_index} could not be imported: {problem}"),
            )
            .with_coordinate(coordinate_index)
        })?;

        Ok(Self { value: real })
    }
}
