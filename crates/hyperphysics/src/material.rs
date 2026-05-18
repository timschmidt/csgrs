//! Exact physical material carriers.

use hyperreal::{Real, RealSign};

use crate::{PhysicsError, PhysicsResult};

/// Stable material identifier.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct MaterialId(String);

/// Uniform material data used by exact mass-property reports.
///
/// Density is stored as [`Real`] so catalog decimals, exact rationals, and
/// symbolic values are not collapsed into primitive floats at the physics
/// boundary. A strictly positive sign must be certified before construction;
/// this mirrors Yap's exact-geometric-computation requirement that decisions
/// use certified facts rather than hidden tolerances.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactMaterial {
    id: MaterialId,
    name: String,
    density: Real,
}

impl MaterialId {
    /// Creates a non-empty material id.
    pub fn new(id: impl Into<String>) -> PhysicsResult<Self> {
        let id = id.into();
        if id.is_empty() {
            return Err(PhysicsError::EmptyIdentifier);
        }
        Ok(Self(id))
    }

    /// Returns the identifier text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl ExactMaterial {
    /// Creates a material with certified positive density.
    pub fn new(id: MaterialId, name: impl Into<String>, density: Real) -> PhysicsResult<Self> {
        match density.refine_sign_until(-64) {
            Some(RealSign::Positive) => Ok(Self {
                id,
                name: name.into(),
                density,
            }),
            Some(RealSign::Negative | RealSign::Zero) | None => {
                Err(PhysicsError::NonPositiveDensity)
            }
        }
    }

    /// Returns this material's stable id.
    pub const fn id(&self) -> &MaterialId {
        &self.id
    }

    /// Returns the display name supplied by the caller.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the exact density.
    pub const fn density(&self) -> &Real {
        &self.density
    }
}
