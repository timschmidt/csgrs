//! Signed Distance Field (SDF) meshing
//!
//! This module provides SDF meshing operations with dependency inversion,
//! allowing for different algorithm implementations (serial/parallel).

pub mod grid;
pub mod traits;

#[cfg(not(feature = "parallel"))]
pub mod serial;

#[cfg(feature = "parallel")]
pub mod parallel;

// Re-export core types
pub use grid::GridShape;
pub use traits::SdfOps;

#[cfg(not(feature = "parallel"))]
pub use serial::SerialSdfOps;

#[cfg(feature = "parallel")]
pub use parallel::ParallelSdfOps;

use crate::float_types::Real;
use crate::mesh::Mesh;
use nalgebra::Point3;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Return a Mesh created by meshing a signed distance field within a bounding box
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S>
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        #[cfg(not(feature = "parallel"))]
        let ops = SerialSdfOps::new();
        #[cfg(feature = "parallel")]
        let ops = ParallelSdfOps::new();

        ops.mesh(sdf, resolution, min_pt, max_pt, iso_value, metadata)
    }
}
