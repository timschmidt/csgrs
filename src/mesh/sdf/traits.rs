//! Traits defining SDF meshing operations for dependency inversion

use crate::float_types::Real;
use crate::mesh::Mesh;
use nalgebra::Point3;
use std::fmt::Debug;

/// Core SDF meshing operations trait
pub trait SdfOps<S: Clone + Debug + Send + Sync> {
    /// Create a mesh from an SDF
    fn mesh<F>(
        &self,
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S>
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send;
}
