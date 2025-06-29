//! Struct and functions for working with `Vertex`s from which `Polygon`s are composed.

use crate::float_types::Real;
use nalgebra::{Point3, Vector3};

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Vertex {
    /// Create a new [`Vertex`].
    ///
    /// * `pos`    – the position in model space  
    /// * `normal` – (optionally non‑unit) normal; it will be **copied
    ///              verbatim**, so make sure it is oriented the way
    ///              you need it for lighting / BSP tests.
    pub const fn new(pos: Point3<Real>, normal: Vector3<Real>) -> Self {
        Vertex { pos, normal }
    }

    /// Flip vertex normal
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// Return the barycentric linear interpolation between `self` (`t = 0`) and `other` (`t = 1`).
    ///
    /// Normals are linearlly interpolated as well.
    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // For positions (Point3): p(t) = p0 + t * (p1 - p0)
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }
}
