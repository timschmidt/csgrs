//! Exact affine transform handoff.
//!
//! General affine transforms are still object-level constructions here: the
//! matrix and translation are exact [`hyperreal::Real`] values, and transformed
//! bounds are produced by mapping exact corners and certifying the resulting
//! min/max comparisons. This follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997, by keeping the
//! geometric construction exact until an explicit adapter lowers it.

use std::cmp::Ordering;

use hyperreal::{CertifiedRealOrdering, Real};

use crate::{CellBounds, ExactAabb3, HypervoxelError, HypervoxelResult};

/// Exact 3D affine transform `matrix * p + translation`.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactAffineTransform {
    /// Exact row-major linear matrix.
    pub matrix: [[Real; 3]; 3],
    /// Exact translation.
    pub translation: [Real; 3],
}

impl ExactAffineTransform {
    /// Creates a new exact affine transform.
    pub fn new(matrix: [[Real; 3]; 3], translation: [Real; 3]) -> Self {
        Self {
            matrix,
            translation,
        }
    }

    /// Returns the identity affine transform.
    pub fn identity() -> Self {
        Self {
            matrix: [
                [1.into(), 0.into(), 0.into()],
                [0.into(), 1.into(), 0.into()],
                [0.into(), 0.into(), 1.into()],
            ],
            translation: [0.into(), 0.into(), 0.into()],
        }
    }

    /// Maps one exact point.
    pub fn map_point(&self, point: &[Real; 3]) -> [Real; 3] {
        [
            self.map_axis(0, point),
            self.map_axis(1, point),
            self.map_axis(2, point),
        ]
    }

    /// Maps exact cell bounds to the exact AABB of the transformed corners.
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
                if certified_cmp(&corner[axis], &min[axis], "affine min")? == Ordering::Less {
                    min[axis] = corner[axis].clone();
                }
                if certified_cmp(&corner[axis], &max[axis], "affine max")? == Ordering::Greater {
                    max[axis] = corner[axis].clone();
                }
            }
        }
        Ok(ExactAabb3 { min, max })
    }

    fn map_axis(&self, axis: usize, point: &[Real; 3]) -> Real {
        self.translation[axis].clone()
            + self.matrix[axis][0].clone() * point[0].clone()
            + self.matrix[axis][1].clone() * point[1].clone()
            + self.matrix[axis][2].clone() * point[2].clone()
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
