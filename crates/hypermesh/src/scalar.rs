//! Scalar import policy for exact mesh data.
//!
//! Primitive floats are accepted only at named lossy input edges. The imported
//! value is stored as an exact dyadic [`hyperreal::Real`], preserving the
//! exact predicate scheduling.

use super::error::{ExactMeshBlockerKind, ExactMeshBlocker, Severity};
use hyperreal::Real;

/// A checked primitive-float import into [`hyperreal::Real`].
#[derive(Clone, Debug, PartialEq)]
pub struct LossyF64Import {
    /// Exact dyadic value stored by `hyperreal`.
    pub value: Real,
    /// Original primitive-float bit pattern supplied by the caller.
    pub original_bits: u64,
}

impl LossyF64Import {
    /// Import one finite `f64` coordinate as an exact dyadic `Real`.
    ///
    /// This is intentionally named "lossy" because the caller's input channel
    /// was a primitive float. Once accepted, the stored `Real` exactly
    /// represents that dyadic value and predicate code must not re-consult
    /// primitive-float tolerances.
    pub fn new(value: f64, coordinate_index: usize) -> Result<Self, ExactMeshBlocker> {
        if !value.is_finite() {
            return Err(ExactMeshBlocker::new(
                Severity::Error,
                ExactMeshBlockerKind::NonFiniteCoordinate,
                format!("coordinate {coordinate_index} is not finite"),
            )
            .with_coordinate(coordinate_index));
        }

        let real = Real::try_from(value).map_err(|problem| {
            ExactMeshBlocker::new(
                Severity::Error,
                ExactMeshBlockerKind::CoordinateImportFailed,
                format!("coordinate {coordinate_index} could not be imported: {problem}"),
            )
            .with_coordinate(coordinate_index)
        })?;

        Ok(Self {
            value: real,
            original_bits: value.to_bits(),
        })
    }
}
