use nalgebra::{
    Point3, Vector3,
};

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub pos: Point3<f64>,
    pub normal: Vector3<f64>,
}

impl Vertex {
    pub fn new(pos: Point3<f64>, normal: Vector3<f64>) -> Self {
        Vertex { pos, normal }
    }

    /// Flip orientation-specific data (like normals)
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// Linearly interpolate between `self` and `other` by parameter `t`.
    pub fn interpolate(&self, other: &Vertex, t: f64) -> Vertex {
        // For positions (Point3): p(t) = p0 + t * (p1 - p0)
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }
}
