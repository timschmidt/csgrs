//! Exact signed-axis grid transforms.
//!
//! Full affine geometry belongs in the shape and lattice crates, but voxel
//! handoff frequently needs the common exact case: axis swaps, axis flips, and
//! exact translations between grid-aligned frames. Keeping this case explicit
//! preserves object structure in the sense of Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997, and avoids pretending
//! that a primitive-float matrix is the source of truth for a transformed
//! voxel AABB.

use std::cmp::Ordering;

use hyperreal::{CertifiedRealOrdering, Real};

use crate::{CellBounds, ExactAabb3, HypervoxelError, HypervoxelResult, VoxelAddress};

/// One output axis mapped from one source axis with a sign.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SignedAxis {
    /// Source axis index in `0..3`.
    pub source_axis: usize,
    /// Axis sign, either `1` or `-1`.
    pub sign: i8,
}

impl SignedAxis {
    /// Creates a validated signed axis mapping.
    pub fn new(source_axis: usize, sign: i8) -> HypervoxelResult<Self> {
        if source_axis >= 3 || !matches!(sign, -1 | 1) {
            return Err(HypervoxelError::InvalidAxisPermutation);
        }
        Ok(Self { source_axis, sign })
    }
}

/// Exact signed-axis permutation plus translation.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisPermutationTransform {
    axes: [SignedAxis; 3],
    translation: [Real; 3],
}

impl AxisPermutationTransform {
    /// Creates a validated signed-axis permutation transform.
    pub fn new(axes: [SignedAxis; 3], translation: [Real; 3]) -> HypervoxelResult<Self> {
        let mut seen = [false; 3];
        for axis in axes {
            if axis.source_axis >= 3 || seen[axis.source_axis] || !matches!(axis.sign, -1 | 1) {
                return Err(HypervoxelError::InvalidAxisPermutation);
            }
            seen[axis.source_axis] = true;
        }
        Ok(Self { axes, translation })
    }

    /// Returns the identity transform.
    pub fn identity() -> Self {
        Self {
            axes: [
                SignedAxis {
                    source_axis: 0,
                    sign: 1,
                },
                SignedAxis {
                    source_axis: 1,
                    sign: 1,
                },
                SignedAxis {
                    source_axis: 2,
                    sign: 1,
                },
            ],
            translation: [0.into(), 0.into(), 0.into()],
        }
    }

    /// Returns the axis mapping.
    pub fn axes(&self) -> &[SignedAxis; 3] {
        &self.axes
    }

    /// Returns the exact translation.
    pub fn translation(&self) -> &[Real; 3] {
        &self.translation
    }

    /// Maps one exact point.
    pub fn map_point(&self, point: &[Real; 3]) -> [Real; 3] {
        [
            self.map_axis(0, point),
            self.map_axis(1, point),
            self.map_axis(2, point),
        ]
    }

    /// Maps exact cell bounds to an exact AABB by transforming all corners.
    pub fn map_bounds(&self, bounds: &CellBounds) -> HypervoxelResult<ExactAabb3> {
        let mut corners = bounds
            .corners()
            .into_iter()
            .map(|point| self.map_point(&point));
        let first = corners
            .next()
            .expect("cell bounds always provide eight corners");
        let mut min = first.clone();
        let mut max = first;
        for corner in corners {
            for axis in 0..3 {
                if certified_cmp(&corner[axis], &min[axis], "transform min")? == Ordering::Less {
                    min[axis] = corner[axis].clone();
                }
                if certified_cmp(&corner[axis], &max[axis], "transform max")? == Ordering::Greater {
                    max[axis] = corner[axis].clone();
                }
            }
        }
        Ok(ExactAabb3 { min, max })
    }

    /// Maps one address through its exact bounds in a source frame.
    pub fn map_address_bounds(
        &self,
        frame: &crate::GridFrame,
        address: VoxelAddress,
    ) -> HypervoxelResult<ExactAabb3> {
        self.map_bounds(&address.bounds(frame)?)
    }

    fn map_axis(&self, output_axis: usize, point: &[Real; 3]) -> Real {
        let axis = self.axes[output_axis];
        let value = point[axis.source_axis].clone();
        if axis.sign < 0 {
            self.translation[output_axis].clone() - value
        } else {
            self.translation[output_axis].clone() + value
        }
    }
}

fn certified_cmp(left: &Real, right: &Real, field: &'static str) -> HypervoxelResult<Ordering> {
    match left.certified_cmp_until(right, -128) {
        CertifiedRealOrdering::Known { ordering, .. } => Ok(ordering),
        CertifiedRealOrdering::Unknown { .. } => {
            Err(HypervoxelError::UnknownScalarOrdering { field })
        }
    }
}
