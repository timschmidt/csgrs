/// A trait for any shape which can be represented by triangles
use crate::float_types::Real;
use nalgebra::{Point3, Vector3};

/// Generic vertex (position + normal) used by IO backends.
#[derive(Clone, Copy, Debug)]
pub struct TriVertex {
    pub position: Point3<Real>,
    pub normal:   Vector3<Real>,
}

/// A triangulated 3D surface.
///
/// Anything that can present itself as a bunch of triangles in 3D
/// can automatically use all the triangle-based IO backends.
pub trait Triangulated3D {
    /// Call `f` for each triangle.
    ///
    /// The triangle is `[v0, v1, v2]` with positions+normals.
    fn visit_triangles<F>(&self, f: F)
    where
        F: FnMut([TriVertex; 3]);
}
