//! Struct and functions for working with `Vertex`s from which `Polygon`s are composed.

use crate::float_types::{PI, Real};
use hashbrown::HashMap;
use nalgebra::{Point3, Vector3};

mod vertex_cluster;
pub use vertex_cluster::*;

mod interpolation_methods;

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone, PartialEq, Copy)]
pub struct Vertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Vertex {
    /// Create a new [`Vertex`].
    ///
    /// * `pos`    – the position in model space  
    /// * `normal` – (optionally non‑unit) normal; it will be **copied verbatim**, so make sure it is oriented the way you need it for lighting / BSP tests.
    ///
    #[inline]
    pub fn new(mut pos: Point3<Real>, mut normal: Vector3<Real>) -> Self {
        // Sanitise position
        for c in pos.coords.iter_mut() {
            if !c.is_finite() {
                *c = 0.0;
            }
        }

        // Sanitise normal
        for c in normal.iter_mut() {
            if !c.is_finite() {
                *c = 0.0;
            }
        }

        Vertex { pos, normal }
    }

    /// Flip vertex normal in place.
    ///
    ///
    /// # Example
    /// ```rust
    /// # use nalgebra::{Point3, Vector3};
    /// # use csgrs::mesh::vertex::Vertex;
    /// let mut v = Vertex::new(Point3::new(1.0, 2.0, 3.0), Vector3::x());
    /// v.flip();
    /// assert_eq!(v.pos, Point3::new(1.0, 2.0, 3.0), "position remains the same");
    /// assert_eq!(v.normal, -Vector3::x(), "the normal is negated");
    /// ```
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// **Mathematical Foundation: Distance Metrics**
    ///
    /// Compute Euclidean distance between vertex positions:
    /// ```text
    /// d(v₁, v₂) = |p₁ - p₂| = √((x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²)
    /// ```
    pub fn distance_to(&self, other: &Vertex) -> Real {
        (self.pos - other.pos).norm()
    }

    /// **Mathematical Foundation: Squared Distance Optimization**
    ///
    /// Compute squared Euclidean distance (avoiding sqrt for performance):
    /// ```text
    /// d²(v₁, v₂) = (x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²
    /// ```
    ///
    /// Useful for distance comparisons without expensive square root operation.
    pub fn distance_squared_to(&self, other: &Vertex) -> Real {
        (self.pos - other.pos).norm_squared()
    }

    /// **Mathematical Foundation: Normal Vector Angular Difference**
    ///
    /// Compute angle between normal vectors using dot product:
    /// ```text
    /// θ = arccos(n₁ · n₂ / (|n₁| · |n₂|))
    /// ```
    ///
    /// Returns angle in radians [0, π].
    pub fn normal_angle_to(&self, other: &Vertex) -> Real {
        let n1 = self.normal.normalize();
        let n2 = other.normal.normalize();
        let cos_angle = n1.dot(&n2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }

    /// **Mathematical Foundation: Weighted Average for Mesh Smoothing**
    ///
    /// Compute weighted average of vertex positions and normals:
    /// ```text
    /// p_avg = Σᵢ(wᵢ · pᵢ) / Σᵢ(wᵢ)
    /// n_avg = normalize(Σᵢ(wᵢ · nᵢ))
    /// ```
    ///
    /// This is fundamental for Laplacian smoothing and normal averaging.
    pub fn weighted_average(vertices: &[(Vertex, Real)]) -> Option<Vertex> {
        if vertices.is_empty() {
            return None;
        }

        let total_weight: Real = vertices.iter().map(|(_, w)| *w).sum();
        if total_weight < Real::EPSILON {
            return None;
        }

        let weighted_pos = vertices
            .iter()
            .fold(Point3::origin(), |acc, (v, w)| acc + v.pos.coords * (*w))
            / total_weight;

        let weighted_normal = vertices
            .iter()
            .fold(Vector3::zeros(), |acc, (v, w)| acc + v.normal * (*w));

        let normalized_normal = if weighted_normal.norm() > Real::EPSILON {
            weighted_normal.normalize()
        } else {
            Vector3::z() // Fallback normal
        };

        Some(Vertex::new(Point3::from(weighted_pos), normalized_normal))
    }

    /// **Mathematical Foundation: Edge-Length-Based Weighting**
    ///
    /// Compute cotangent weights for discrete Laplacian operators:
    /// ```text
    /// w_ij = (cot(α) + cot(β)) / 2
    /// ```
    /// Where α and β are the angles opposite to edge ij in adjacent triangles.
    ///
    /// This provides a better approximation to the continuous Laplacian operator
    /// compared to uniform weights.
    pub fn compute_cotangent_weight(
        center: &Vertex,
        neighbor: &Vertex,
        triangle_vertices: &[&Vertex],
    ) -> Real {
        if triangle_vertices.len() < 3 {
            return 1.0; // Fallback to uniform weight
        }

        // Find the third vertex in the triangle
        let mut cot_sum = 0.0;
        let mut weight_count = 0;

        for i in 0..triangle_vertices.len() {
            let v1 = triangle_vertices[i];
            let v2 = triangle_vertices[(i + 1) % triangle_vertices.len()];
            let v3 = triangle_vertices[(i + 2) % triangle_vertices.len()];

            // Check if this triangle contains our edge
            let contains_edge = (v1.pos == center.pos && v2.pos == neighbor.pos)
                || (v2.pos == center.pos && v3.pos == neighbor.pos)
                || (v3.pos == center.pos && v1.pos == neighbor.pos)
                || (v1.pos == neighbor.pos && v2.pos == center.pos)
                || (v2.pos == neighbor.pos && v3.pos == center.pos)
                || (v3.pos == neighbor.pos && v1.pos == center.pos);

            if contains_edge {
                // Find the vertex opposite to the edge
                let opposite = if v1.pos != center.pos && v1.pos != neighbor.pos {
                    v1
                } else if v2.pos != center.pos && v2.pos != neighbor.pos {
                    v2
                } else {
                    v3
                };

                // Compute cotangent of angle at opposite vertex
                let edge1 = center.pos - opposite.pos;
                let edge2 = neighbor.pos - opposite.pos;
                let cos_angle = edge1.normalize().dot(&edge2.normalize());
                let sin_angle = edge1.normalize().cross(&edge2.normalize()).norm();

                if sin_angle > Real::EPSILON {
                    cot_sum += cos_angle / sin_angle;
                    weight_count += 1;
                }
            }
        }

        if weight_count > 0 {
            cot_sum / (2.0 * weight_count as Real)
        } else {
            1.0 // Fallback to uniform weight
        }
    }

    /// **Mathematical Foundation: Vertex Valence and Regularity Analysis**
    ///
    /// Analyze vertex connectivity in mesh topology using actual adjacency data:
    /// - **Valence**: Number of edges incident to vertex (from adjacency map)
    /// - **Regularity**: Measure of how close valence is to optimal (6 for interior vertices)
    ///
    /// ## **Vertex Index Lookup**
    /// This function requires the vertex's global index in the mesh adjacency graph.
    /// The caller should provide the correct index from the mesh connectivity analysis.
    ///
    /// ## **Regularity Scoring**
    /// ```text
    /// regularity = 1 / (1 + |valence - target| / target)
    /// ```
    /// Where target = 6 for triangular meshes (optimal valence for interior vertices).
    ///
    /// Returns (valence, regularity_score) where regularity ∈ [0,1], 1 = optimal.
    pub fn analyze_connectivity_with_index(
        vertex_index: usize,
        adjacency_map: &HashMap<usize, Vec<usize>>,
    ) -> (usize, Real) {
        let valence = adjacency_map
            .get(&vertex_index)
            .map(|neighbors| neighbors.len())
            .unwrap_or(0);

        // Optimal valence is 6 for interior vertices in triangular meshes
        let target_valence = 6;
        let regularity: Real = if valence > 0 {
            let deviation = (valence as Real - target_valence as Real).abs();
            (1.0 / (1.0 + deviation / target_valence as Real)).max(0.0)
        } else {
            0.0
        };

        (valence, regularity)
    }

    /// **Mathematical Foundation: Position-Based Vertex Lookup**
    ///
    /// Simplified connectivity analysis that searches for the vertex in the adjacency map
    /// by position matching (with epsilon tolerance). This is slower but more convenient
    /// when you don't have the global vertex index readily available.
    ///
    /// **Note**: This is a convenience method. For performance-critical applications,
    /// use `analyze_connectivity_with_index` with pre-computed vertex indices.
    pub fn analyze_connectivity_by_position(
        &self,
        adjacency_map: &HashMap<usize, Vec<usize>>,
        vertex_positions: &HashMap<usize, Point3<Real>>,
        epsilon: Real,
    ) -> (usize, Real) {
        // Find the vertex index by position matching
        let mut vertex_index = None;
        for (&idx, &pos) in vertex_positions {
            if (self.pos - pos).norm() < epsilon {
                vertex_index = Some(idx);
                break;
            }
        }

        if let Some(idx) = vertex_index {
            Self::analyze_connectivity_with_index(idx, adjacency_map)
        } else {
            // Vertex not found in adjacency map
            (0, 0.0)
        }
    }

    /// **Mathematical Foundation: Curvature Estimation**
    ///
    /// Estimate discrete mean curvature using the angle deficit method:
    /// ```text
    /// H ≈ (2π - Σθᵢ) / A_mixed
    /// ```
    /// Where θᵢ are angles around the vertex and A_mixed is the mixed area.
    ///
    /// This provides a discrete approximation to the mean curvature at a vertex.
    pub fn estimate_mean_curvature(&self, neighbors: &[Vertex], face_areas: &[Real]) -> Real {
        if neighbors.len() < 3 {
            return 0.0;
        }

        // Compute angle sum around vertex
        let mut angle_sum = 0.0;
        for i in 0..neighbors.len() {
            let prev = &neighbors[(i + neighbors.len() - 1) % neighbors.len()];
            let next = &neighbors[(i + 1) % neighbors.len()];

            let v1 = (prev.pos - self.pos).normalize();
            let v2 = (next.pos - self.pos).normalize();

            let dot = v1.dot(&v2).clamp(-1.0, 1.0);
            angle_sum += dot.acos();
        }

        // Compute mixed area (average of face areas)
        let mixed_area = if !face_areas.is_empty() {
            face_areas.iter().sum::<Real>() / face_areas.len() as Real
        } else {
            1.0 // Fallback to avoid division by zero
        };

        // Discrete mean curvature
        let angle_deficit = 2.0 * PI - angle_sum;
        if mixed_area > Real::EPSILON {
            angle_deficit / mixed_area
        } else {
            0.0
        }
    }

    /// **Mathematical Foundation: Advanced Mesh Quality Analysis**
    ///
    /// Comprehensive vertex quality assessment using multiple metrics:
    ///
    /// ## **Quality Metrics**
    /// - **Regularity**: How close vertex valence is to optimal (6 for triangular meshes)
    /// - **Curvature**: Discrete mean curvature estimation
    /// - **Edge Uniformity**: Standard deviation of incident edge lengths
    /// - **Normal Variation**: Consistency of adjacent face normals
    ///
    /// ## **Applications**
    /// - **Adaptive Refinement**: Identify vertices needing subdivision
    /// - **Quality Scoring**: Overall mesh quality assessment
    /// - **Feature Detection**: Identify sharp features and boundaries
    ///
    /// Returns (regularity, curvature, edge_uniformity, normal_variation)
    pub fn comprehensive_quality_analysis(
        &self,
        vertex_index: usize,
        adjacency_map: &HashMap<usize, Vec<usize>>,
        vertex_positions: &HashMap<usize, Point3<Real>>,
        vertex_normals: &HashMap<usize, Vector3<Real>>,
    ) -> (Real, Real, Real, Real) {
        // Get connectivity information
        let (valence, regularity) =
            Self::analyze_connectivity_with_index(vertex_index, adjacency_map);

        if valence == 0 {
            return (0.0, 0.0, 0.0, 0.0);
        }

        // Get neighbor positions for edge length analysis
        let neighbors = adjacency_map.get(&vertex_index).unwrap();
        let mut edge_lengths = Vec::new();
        let mut neighbor_normals = Vec::new();

        for &neighbor_idx in neighbors {
            if let Some(&neighbor_pos) = vertex_positions.get(&neighbor_idx) {
                let edge_length = (self.pos - neighbor_pos).norm();
                edge_lengths.push(edge_length);

                if let Some(&neighbor_normal) = vertex_normals.get(&neighbor_idx) {
                    neighbor_normals.push(neighbor_normal);
                }
            }
        }

        // Edge uniformity (lower standard deviation = more uniform)
        let edge_uniformity = if edge_lengths.len() > 1 {
            let mean_edge = edge_lengths.iter().sum::<Real>() / edge_lengths.len() as Real;
            let variance = edge_lengths
                .iter()
                .map(|&len| (len - mean_edge).powi(2))
                .sum::<Real>()
                / edge_lengths.len() as Real;
            let std_dev = variance.sqrt();

            // Normalize to [0,1] where 1 = perfectly uniform
            1.0 / (1.0 + std_dev / mean_edge)
        } else {
            1.0
        };

        // Normal variation (lower = more consistent normals)
        let normal_variation = if neighbor_normals.len() > 1 {
            let mut max_angle: Real = 0.0;
            for &neighbor_normal in &neighbor_normals {
                let angle = self
                    .normal
                    .normalize()
                    .dot(&neighbor_normal.normalize())
                    .acos();
                max_angle = max_angle.max(angle);
            }

            // Normalize to [0,1] where 1 = perfectly consistent
            1.0 - (max_angle / PI).min(1.0)
        } else {
            1.0
        };

        // Simple curvature estimation based on normal variation
        let curvature = if !neighbor_normals.is_empty() {
            let avg_normal = neighbor_normals
                .iter()
                .fold(Vector3::zeros(), |acc, &n| acc + n)
                / neighbor_normals.len() as Real;
            (self.normal - avg_normal).norm() // normal deviation
        } else {
            0.0
        };

        (regularity, curvature, edge_uniformity, normal_variation)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct VertexEpsilon {
    pub position: <Point3<Real> as approx::AbsDiffEq>::Epsilon,
    pub normal: <Vector3<Real> as approx::AbsDiffEq>::Epsilon,
}

impl approx::AbsDiffEq for Vertex {
    type Epsilon = VertexEpsilon;

    fn default_epsilon() -> Self::Epsilon {
        Self::Epsilon {
            position: Point3::<Real>::default_epsilon(),
            normal: Vector3::<Real>::default_epsilon(),
        }
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        approx::AbsDiffEq::abs_diff_eq(&self.pos, &other.pos, epsilon.position)
            && approx::AbsDiffEq::abs_diff_eq(&self.normal, &other.normal, epsilon.normal)
    }
}

impl approx::RelativeEq for Vertex {
    fn default_max_relative() -> Self::Epsilon {
        Self::Epsilon {
            position: Point3::<Real>::default_max_relative(),
            normal: Vector3::<Real>::default_max_relative(),
        }
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        approx::RelativeEq::relative_eq(
            &self.pos,
            &other.pos,
            epsilon.position,
            max_relative.position,
        ) && approx::RelativeEq::relative_eq(
            &self.normal,
            &other.normal,
            epsilon.normal,
            max_relative.normal,
        )
    }
}

impl approx::UlpsEq for Vertex {
    fn default_max_ulps() -> u32 {
        debug_assert_eq!(
            Point3::<Real>::default_max_ulps(),
            Vector3::<Real>::default_max_ulps()
        );

        Point3::<Real>::default_max_ulps()
    }

    fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
        approx::UlpsEq::ulps_eq(&self.pos, &other.pos, epsilon.position, max_ulps)
            && approx::UlpsEq::ulps_eq(&self.normal, &other.normal, epsilon.normal, max_ulps)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_vertex_new() {
        let pos = Point3::new(1.0, 2.0, 3.0);
        let normal = Vector3::new(0.0, 1.0, 0.0);
        let v = Vertex::new(pos, normal);
        assert_eq!(v.pos, pos);
        assert_eq!(v.normal, normal);
    }

    #[test]
    fn test_vertex_interpolate() {
        let v1 = Vertex::new(Point3::origin(), Vector3::x());
        let v2 = Vertex::new(Point3::new(2.0, 2.0, 2.0), Vector3::y());
        let v_mid = v1.interpolate(&v2, 0.5);

        approx::assert_relative_eq!(
            v_mid,
            Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.5, 0.5, 0.0))
        );
    }

    #[test]
    fn distance() {
        let v1 = Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::x());
        let v2 = Vertex::new(Point3::new(3.0, 4.0, 0.0), Vector3::y());

        // Test distance calculation
        let distance = v1.distance_to(&v2);
        assert!(
            (distance - 5.0).abs() < 1e-10,
            "Distance should be 5.0 (3-4-5 triangle)"
        );

        // Test squared distance (should be 25.0)
        let distance_sq = v1.distance_squared_to(&v2);
        assert!(
            (distance_sq - 25.0).abs() < 1e-10,
            "Squared distance should be 25.0"
        );

        // Test normal angle
        let angle = v1.normal_angle_to(&v2);
        assert!(
            (angle - PI / 2.0).abs() < 1e-10,
            "Angle between x and y normals should be π/2"
        );
    }
}
