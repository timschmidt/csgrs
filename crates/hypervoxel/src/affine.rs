//! Exact affine transform handoff.
//!
//! General affine transforms are still object-level constructions here: the
//! matrix and translation are exact [`hyperreal::Real`] values, and transformed
//! bounds are produced by accumulating the exact per-axis term intervals. This
//! follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7(1-2), 1997, by keeping the geometric construction exact until an
//! explicit adapter lowers it.

use std::cmp::Ordering;

use crate::{CellBounds, ExactAabb3, HypervoxelError, HypervoxelResult};
use hyperlattice::{Matrix3, Vector3};
use hyperreal::Real;

/// Exact 3D affine transform `matrix * p + translation`.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactAffineTransform {
    /// Exact row-major linear matrix.
    pub matrix: Matrix3,
    /// Exact translation.
    pub translation: Vector3,
}

impl ExactAffineTransform {
    /// Creates a new exact affine transform.
    pub fn new(matrix: [[Real; 3]; 3], translation: [Real; 3]) -> Self {
        Self {
            matrix: Matrix3::new(matrix),
            translation: Vector3::new(translation),
        }
    }

    /// Creates a new exact affine transform from lattice-owned primitives.
    pub const fn from_lattice(matrix: Matrix3, translation: Vector3) -> Self {
        Self {
            matrix,
            translation,
        }
    }

    /// Returns the identity affine transform.
    pub fn identity() -> Self {
        Self {
            matrix: Matrix3::new([
                [1.into(), 0.into(), 0.into()],
                [0.into(), 1.into(), 0.into()],
                [0.into(), 0.into(), 1.into()],
            ]),
            translation: Vector3::new([0.into(), 0.into(), 0.into()]),
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
    ///
    /// This uses Arvo's transform-box construction from Graphics Gems: for
    /// each output coordinate, add the smaller/larger contribution of every
    /// source-axis endpoint instead of transforming all eight corners.
    pub fn map_bounds(&self, bounds: &CellBounds) -> HypervoxelResult<ExactAabb3> {
        let mut min = self.translation.0.clone();
        let mut max = self.translation.0.clone();
        for target_axis in 0..3 {
            for source_axis in 0..3 {
                let lower_term =
                    self.matrix[target_axis][source_axis].clone() * bounds.min[source_axis].clone();
                let upper_term =
                    self.matrix[target_axis][source_axis].clone() * bounds.max[source_axis].clone();
                match certified_cmp(&lower_term, &upper_term, "affine term order")? {
                    Ordering::Less | Ordering::Equal => {
                        min[target_axis] = min[target_axis].clone() + lower_term;
                        max[target_axis] = max[target_axis].clone() + upper_term;
                    }
                    Ordering::Greater => {
                        min[target_axis] = min[target_axis].clone() + upper_term;
                        max[target_axis] = max[target_axis].clone() + lower_term;
                    }
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
    match hyperlimit::compare_reals(left, right).value() {
        Some(ordering) => Ok(ordering),
        None => Err(HypervoxelError::UnknownScalarOrdering { field }),
    }
}
