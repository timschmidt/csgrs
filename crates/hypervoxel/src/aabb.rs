//! Exact axis-aligned bounding-box handoff records.
//!
//! These records preserve geometric structure until a predicate or
//! construction selects the required arithmetic. `ExactAabb3` therefore
//! carries exact bounds instead of silently flattening voxel cells into
//! display coordinates.

use hyperlattice::Vector3;
use hyperreal::Real;

use crate::{CellBounds, GridFrame, HypervoxelResult, VoxelAddress};

/// Exact three-dimensional axis-aligned bounding box.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactAabb3 {
    /// Minimum exact corner.
    pub min: [Real; 3],
    /// Maximum exact corner.
    pub max: [Real; 3],
}

impl From<CellBounds> for ExactAabb3 {
    fn from(bounds: CellBounds) -> Self {
        Self {
            min: bounds.min,
            max: bounds.max,
        }
    }
}

impl From<&CellBounds> for ExactAabb3 {
    fn from(bounds: &CellBounds) -> Self {
        Self {
            min: bounds.min.clone(),
            max: bounds.max.clone(),
        }
    }
}

impl ExactAabb3 {
    /// Returns the minimum corner as a [`hyperlattice::Vector3`].
    ///
    /// This is an exact object handoff, not a numeric lowering. Exposing a
    /// lattice vector lets downstream predicates and transforms consume exact
    /// coordinates without inventing a primitive-float AABB adapter.
    pub fn min_vector(&self) -> Vector3 {
        Vector3::new(self.min.clone())
    }

    /// Returns the maximum corner as a [`hyperlattice::Vector3`].
    ///
    /// Like [`Self::min_vector`], this preserves exact `Real` components so
    /// later predicates can decide which arithmetic package to use.
    pub fn max_vector(&self) -> Vector3 {
        Vector3::new(self.max.clone())
    }

    /// Returns the exact center point.
    pub fn center(&self) -> [Real; 3] {
        CellBounds {
            min: self.min.clone(),
            max: self.max.clone(),
        }
        .center()
    }

    /// Returns the exact extent along one axis.
    pub fn extent(&self, axis: usize) -> Real {
        self.max[axis].clone() - self.min[axis].clone()
    }
}

/// Exact AABB represented with `hyperlattice` vectors.
#[derive(Clone, Debug, PartialEq)]
pub struct LatticeAabbHandoff {
    /// Minimum exact corner.
    pub min: Vector3,
    /// Maximum exact corner.
    pub max: Vector3,
}

impl LatticeAabbHandoff {
    /// Builds a vector-backed handoff record for one cell in a grid frame.
    pub fn from_address(frame: &GridFrame, address: VoxelAddress) -> HypervoxelResult<Self> {
        GridAabbHandoff::from_address(frame, address).map(Self::from)
    }

    /// Returns exact structural facts for the minimum and maximum vectors.
    ///
    /// These facts are non-certifying scheduling metadata. They help exact
    /// kernels select sparse or common-scale routes but do not replace
    /// predicate certificates.
    pub fn vector_facts(&self) -> (hyperlattice::Vector3Facts, hyperlattice::Vector3Facts) {
        (self.min.structural_facts(), self.max.structural_facts())
    }
}

/// Exact AABB plus its grid address for inter-crate geometry handoff.
#[derive(Clone, Debug, PartialEq)]
pub struct GridAabbHandoff {
    /// Source grid address.
    pub address: VoxelAddress,
    /// Exact AABB in the source frame.
    pub bounds: ExactAabb3,
}

impl GridAabbHandoff {
    /// Builds a handoff record for one cell in a grid frame.
    pub fn from_address(frame: &GridFrame, address: VoxelAddress) -> HypervoxelResult<Self> {
        Ok(Self {
            address,
            bounds: address.bounds(frame)?.into(),
        })
    }

    /// Converts this record into a `hyperlattice` vector-backed handoff.
    pub fn into_lattice(self) -> LatticeAabbHandoff {
        self.into()
    }
}

impl From<GridAabbHandoff> for LatticeAabbHandoff {
    fn from(handoff: GridAabbHandoff) -> Self {
        Self {
            min: handoff.bounds.min_vector(),
            max: handoff.bounds.max_vector(),
        }
    }
}
