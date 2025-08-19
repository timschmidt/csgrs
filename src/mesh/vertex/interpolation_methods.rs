use nalgebra::Point3;

use crate::{float_types::Real, mesh::vertex::Vertex};

impl Vertex {
    /// **Mathematical Foundation: Barycentric Linear Interpolation**
    ///
    /// Compute the barycentric linear interpolation between `self` (`t = 0`) and `other` (`t = 1`).
    /// This implements the fundamental linear interpolation formula:
    ///
    /// ## **Interpolation Formula**
    /// For parameter t ∈ [0,1]:
    /// - **Position**: p(t) = (1-t)·p₀ + t·p₁ = p₀ + t·(p₁ - p₀)
    /// - **Normal**: n(t) = (1-t)·n₀ + t·n₁ = n₀ + t·(n₁ - n₀)
    ///
    /// ## **Mathematical Properties**
    /// - **Affine Combination**: Coefficients sum to 1: (1-t) + t = 1
    /// - **Endpoint Preservation**: p(0) = p₀, p(1) = p₁
    /// - **Linearity**: Second derivatives are zero (straight line in parameter space)
    /// - **Convexity**: Result lies on line segment between endpoints
    ///
    /// ## **Geometric Interpretation**
    /// The interpolated vertex represents a point on the edge connecting the two vertices,
    /// with both position and normal vectors smoothly blended. This is fundamental for:
    /// - **Polygon Splitting**: Creating intersection vertices during BSP operations
    /// - **Triangle Subdivision**: Generating midpoints for mesh refinement
    /// - **Smooth Shading**: Interpolating normals across polygon edges
    ///
    /// **Note**: Normals are linearly interpolated (not spherically), which is appropriate
    /// for most geometric operations but may require renormalization for lighting calculations.
    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // For positions (Point3): p(t) = p0 + t * (p1 - p0)
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }

    /// **Mathematical Foundation: Spherical Linear Interpolation (SLERP) for Normals**
    ///
    /// Compute spherical linear interpolation for normal vectors, preserving unit length:
    ///
    /// ## **SLERP Formula**
    /// For unit vectors n₀, n₁ and parameter t ∈ [0,1]:
    /// ```text
    /// slerp(n₀, n₁, t) = (sin((1-t)·Ω) · n₀ + sin(t·Ω) · n₁) / sin(Ω)
    /// ```
    /// Where Ω = arccos(n₀ · n₁) is the angle between vectors.
    ///
    /// ## **Mathematical Properties**
    /// - **Arc Interpolation**: Follows great circle on unit sphere
    /// - **Constant Speed**: Angular velocity is constant
    /// - **Unit Preservation**: Result is always unit length
    /// - **Orientation**: Shortest path between normals
    ///
    /// This is preferred over linear interpolation for normal vectors in lighting
    /// calculations and smooth shading applications.
    pub fn slerp_interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // Linear interpolation for position
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // Spherical linear interpolation for normals
        let n0 = self.normal.normalize();
        let n1 = other.normal.normalize();

        let dot = n0.dot(&n1).clamp(-1.0, 1.0);

        // If normals are nearly parallel, use linear interpolation
        if (dot.abs() - 1.0).abs() < Real::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }

        let omega = dot.acos();
        let sin_omega = omega.sin();

        if sin_omega.abs() < Real::EPSILON {
            // Fallback to linear interpolation
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }

        let a = ((1.0 - t) * omega).sin() / sin_omega;
        let b = (t * omega).sin() / sin_omega;

        let new_normal = (a * n0 + b * n1).normalize();
        Vertex::new(new_pos, new_normal)
    }

    /// **Mathematical Foundation: Barycentric Coordinates Interpolation**
    ///
    /// Interpolate vertex using barycentric coordinates (u, v, w) with u + v + w = 1:
    /// ```text
    /// p = u·p₁ + v·p₂ + w·p₃
    /// n = normalize(u·n₁ + v·n₂ + w·n₃)
    /// ```
    ///
    /// This is fundamental for triangle interpolation and surface parameterization.
    pub fn barycentric_interpolate(
        v1: &Vertex,
        v2: &Vertex,
        v3: &Vertex,
        u: Real,
        v: Real,
        w: Real,
    ) -> Vertex {
        // Ensure barycentric coordinates sum to 1 (normalize if needed)
        let total = u + v + w;
        let (u, v, w) = if total.abs() > Real::EPSILON {
            (u / total, v / total, w / total)
        } else {
            (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0) // Fallback to centroid
        };

        let new_pos = Point3::from(u * v1.pos.coords + v * v2.pos.coords + w * v3.pos.coords);

        let new_normal = (u * v1.normal + v * v2.normal + w * v3.normal).normalize();

        Vertex::new(new_pos, new_normal)
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{Point3, Vector3};

    use crate::mesh::vertex::Vertex;

    fn create_vertices() -> [Vertex; 2] {
        [
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::y()),
        ]
    }

    #[test]
    fn linear() {
        let [v1, v2] = create_vertices();

        // Test linear interpolation
        let mid_linear = v1.interpolate(&v2, 0.5);
        assert!(
            (mid_linear.pos - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
            "Linear interpolation midpoint should be (1,1,1)"
        );
    }

    #[test]
    fn barycentric() {
        let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
        let v2 = Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y());
        let v3 = Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z());

        // Test centroid (equal weights)
        let centroid =
            Vertex::barycentric_interpolate(&v1, &v2, &v3, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
        let expected_pos = Point3::new(1.0 / 3.0, 1.0 / 3.0, 0.0);
        assert!(
            (centroid.pos - expected_pos).norm() < 1e-10,
            "Barycentric centroid should be at (1/3, 1/3, 0)"
        );

        // Test vertex recovery (weight=1 for one vertex)
        let recovered_v1 = Vertex::barycentric_interpolate(&v1, &v2, &v3, 1.0, 0.0, 0.0);
        assert!(
            (recovered_v1.pos - v1.pos).norm() < 1e-10,
            "Barycentric should recover original vertex"
        );
    }

    #[test]
    /// Test spherical interpolation
    fn slerp() {
        let [v1, v2] = create_vertices();

        let mid_slerp = v1.slerp_interpolate(&v2, 0.5);
        assert!(
            (mid_slerp.pos - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-10,
            "SLERP position should match linear for positions"
        );

        // Normal should be normalized and between the two normals
        assert!(
            (mid_slerp.normal.norm() - 1.0).abs() < 1e-10,
            "SLERP normal should be unit length"
        );
    }
}
