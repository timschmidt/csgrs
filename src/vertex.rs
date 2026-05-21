//! Struct and functions for working with `Vertex`s.

use crate::float_types::{
    PI, Real, hangle_between_vectors, hangle_sin_cos, hpoint_centroid, hpoint_distance,
    hpoint_lerp, hpoint_weighted_sum, hpoints_within_epsilon, hreal_clamp_f64, hreal_div,
    hreal_f64s_within_epsilon, hreal_from_f64, hreal_gt_f64, hreal_lt_f64,
    hreal_max_report_value, hreal_mean, hreal_mul, hreal_sample_stddev, hreal_sub, hreal_sum,
    hreal_to_f64, hunit_vector3, hvector3_distance, hvector3_dot, hvector3_from_point3,
    hvector3_mean, hvector3_weighted_sum, tolerance,
};
use hashbrown::HashMap;
use nalgebra::{Point3, Vector3};

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone, PartialEq, Copy)]
pub struct Vertex {
    pub position: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Default for Vertex {
    fn default() -> Self {
        Self::new(Point3::origin(), Vector3::z())
    }
}

impl Vertex {
    /// Create a new [`Vertex`].
    ///
    /// * `position`    – the position in model space  
    /// * `normal` – (optionally non‑unit) normal; it will be **copied verbatim**, so make sure it is oriented the way you need it for lighting / BSP tests.
    #[inline]
    pub const fn new(mut position: Point3<Real>, mut normal: Vector3<Real>) -> Self {
        // Sanitise position
        // Nasty loop unrolling to allow for const-context evaluations.
        // Can be replaced with proper for _ in _ {} loops once
        // https://github.com/rust-lang/rust/issues/87575 is merged
        let [[x, y, z]]: &mut [[_; 3]; 1] = &mut position.coords.data.0;

        if !x.is_finite() {
            *x = 0.0;
        }
        if !y.is_finite() {
            *y = 0.0;
        }
        if !z.is_finite() {
            *z = 0.0;
        }

        // Sanitise normal
        let [[nx, ny, nz]]: &mut [[_; 3]; 1] = &mut normal.data.0;

        if !nx.is_finite() {
            *nx = 0.0;
        }
        if !ny.is_finite() {
            *ny = 0.0;
        }
        if !nz.is_finite() {
            *nz = 0.0;
        }

        Vertex { position, normal }
    }

    /// Flip vertex normal
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// **Mathematical Foundation: Barycentric Linear Interpolation**
    ///
    /// Compute the barycentric linear interpolation between `self` (`t = 0`) and `other` (`t = 1`).
    /// This implements the fundamental linear interpolation formula:
    ///
    /// ## **Interpolation Formula**
    /// For parameter `t` in the closed interval `[0, 1]`:
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
        let new_position = self.position + (other.position - self.position) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_position, new_normal)
    }

    /// **Mathematical Foundation: Spherical Linear Interpolation (SLERP) for Normals**
    ///
    /// Compute spherical linear interpolation for normal vectors, preserving unit length:
    ///
    /// ## **SLERP Formula**
    /// For unit vectors n₀, n₁ and parameter `t` in `[0, 1]`:
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
    /// This is preferred over linear interpolation for normal vectors in
    /// lighting and smooth shading applications. Input normal normalization
    /// and the dot-product angle decision are routed through
    /// `hyperlattice::Vector3`/`hyperreal::Real`, while the final blended
    /// vector is exported back to the finite [`Vertex`] boundary. This follows
    /// Yap's exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn slerp_interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        let new_position = hpoint_lerp(&self.position, &other.position, t)
            .unwrap_or_else(|| self.interpolate(other, t).position);

        let Some(n0) = hunit_vector3(&self.normal) else {
            return self.interpolate(other, t);
        };
        let Some(n1) = hunit_vector3(&other.normal) else {
            return self.interpolate(other, t);
        };

        let Some(dot) = hvector3_dot(&n0, &n1).and_then(|dot| hreal_clamp_f64(dot, -1.0, 1.0))
        else {
            return self.interpolate(other, t);
        };

        // If normals are nearly parallel, use linear interpolation
        if hreal_f64s_within_epsilon(dot, 1.0, tolerance())
            || hreal_f64s_within_epsilon(dot, -1.0, tolerance())
        {
            let new_normal = hvector3_weighted_sum(
                &[self.normal, other.normal],
                &[hreal_sub(1.0, t).unwrap_or(0.0), t],
            )
            .and_then(|normal| hunit_vector3(&normal))
            .unwrap_or_else(Vector3::z);
            return Vertex::new(new_position, new_normal);
        }

        let Some(omega) = hangle_between_vectors(&n0, &n1) else {
            return self.interpolate(other, t);
        };
        let Some((sin_omega, _)) = hangle_sin_cos(omega) else {
            return self.interpolate(other, t);
        };

        if hreal_f64s_within_epsilon(sin_omega, 0.0, tolerance()) {
            // Fallback to linear interpolation
            let new_normal = hvector3_weighted_sum(
                &[self.normal, other.normal],
                &[hreal_sub(1.0, t).unwrap_or(0.0), t],
            )
            .and_then(|normal| hunit_vector3(&normal))
            .unwrap_or_else(Vector3::z);
            return Vertex::new(new_position, new_normal);
        }

        let Some(one_minus_t) = hreal_sub(1.0, t) else {
            return self.interpolate(other, t);
        };
        let Some(a_angle) = hreal_mul(one_minus_t, omega) else {
            return self.interpolate(other, t);
        };
        let Some(b_angle) = hreal_mul(t, omega) else {
            return self.interpolate(other, t);
        };
        let Some((a_sin, _)) = hangle_sin_cos(a_angle) else {
            return self.interpolate(other, t);
        };
        let Some((b_sin, _)) = hangle_sin_cos(b_angle) else {
            return self.interpolate(other, t);
        };
        let Some(a) = hreal_div(a_sin, sin_omega) else {
            return self.interpolate(other, t);
        };
        let Some(b) = hreal_div(b_sin, sin_omega) else {
            return self.interpolate(other, t);
        };

        let Some(blended) = hvector3_weighted_sum(&[n0, n1], &[a, b]) else {
            return self.interpolate(other, t);
        };
        let new_normal = hunit_vector3(&blended).unwrap_or_else(Vector3::z);
        Vertex::new(new_position, new_normal)
    }

    /// **Mathematical Foundation: Distance Metrics**
    ///
    /// Compute Euclidean distance between vertex positions:
    /// ```text
    /// d(v₁, v₂) = |p₁ - p₂| = √((x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²)
    /// ```
    ///
    /// The squared-distance expression is evaluated through
    /// `hyperlattice::Vector3` and `hyperreal::Real`, then exported to the
    /// finite `f64` mesh API. This keeps distance algebra in the hyper geometry
    /// layer and follows Yap's exact-geometric-computation boundary discipline
    /// for separating primitive coordinates from exact-aware objects:
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub fn distance_to(&self, other: &Vertex) -> Real {
        hyper_vertex_distance_squared(self, other)
            .and_then(|distance_squared| hreal_to_f64(&distance_squared.sqrt().ok()?))
            .unwrap_or(Real::INFINITY)
    }

    /// **Mathematical Foundation: Squared Distance Optimization**
    ///
    /// Compute squared Euclidean distance (avoiding sqrt for performance):
    /// ```text
    /// d²(v₁, v₂) = (x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²
    /// ```
    ///
    /// Useful for distance comparisons without expensive square root operation.
    /// The algebra lives in `hyperlattice::Vector3::squared_distance` so this
    /// helper remains an API-boundary finite export rather than a local f64
    /// predicate.
    pub fn distance_squared_to(&self, other: &Vertex) -> Real {
        hyper_vertex_distance_squared(self, other)
            .and_then(|distance_squared| hreal_to_f64(&distance_squared))
            .unwrap_or(Real::INFINITY)
    }

    /// **Mathematical Foundation: Normal Vector Angular Difference**
    ///
    /// Compute angle between normal vectors using dot product:
    /// ```text
    /// θ = arccos(n₁ · n₂ / (|n₁| · |n₂|))
    /// ```
    ///
    /// Returns angle in radians [0, π].
    ///
    /// Normalization, dot product, and inverse cosine are routed through
    /// `hyperlattice`/`hyperreal`, with `f64` retained only as this legacy mesh
    /// API's return type. The checked-normalization step rejects zero and
    /// unknown-zero normals before the angle branch, following Yap's exact
    /// geometric computation model for making geometric decisions on
    /// exact-aware objects rather than primitive floats:
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub fn normal_angle_to(&self, other: &Vertex) -> Real {
        hangle_between_vectors(&self.normal, &other.normal).unwrap_or(0.0)
    }

    /// **Mathematical Foundation: Weighted Average for Mesh Smoothing**
    ///
    /// Compute weighted average of vertex positions and normals:
    /// ```text
    /// p_avg = Σᵢ(wᵢ · pᵢ) / Σᵢ(wᵢ)
    /// n_avg = normalize(Σᵢ(wᵢ · nᵢ))
    /// ```
    ///
    /// This is fundamental for Laplacian smoothing and normal averaging. The
    /// incoming scalar weights are normalized through `hyperreal::Real` before
    /// the affine combination is evaluated at the current f64 vertex boundary.
    /// Keeping the normalization decision in the hyperreal path follows Yap's
    /// exact-geometric-computation separation between geometric decisions and
    /// primitive floating-point boundaries (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// The averaging itself is the local linear operation used by Laplacian
    /// mesh smoothing, while final normal normalization is checked by
    /// hyperlattice; see Taubin, "A Signal Processing Approach to Fair Surface
    /// Design," SIGGRAPH 1995
    /// (<https://doi.org/10.1145/218380.218473>).
    pub fn weighted_average(vertices: &[(Vertex, Real)]) -> Option<Vertex> {
        if vertices.is_empty() {
            return None;
        }

        let normalized_weights = hyper_normalized_weights(vertices.iter().map(|(_, w)| *w))?;

        let positions = vertices
            .iter()
            .map(|(vertex, _)| vertex.position)
            .collect::<Vec<_>>();
        let normals = vertices
            .iter()
            .map(|(vertex, _)| vertex.normal)
            .collect::<Vec<_>>();
        let weighted_position = hpoint_weighted_sum(&positions, &normalized_weights)?;
        let weighted_normal = hvector3_weighted_sum(&normals, &normalized_weights)?;

        let normalized_normal = hunit_vector3(&weighted_normal).unwrap_or_else(Vector3::z);

        Some(Vertex::new(weighted_position, normalized_normal))
    }

    /// **Mathematical Foundation: Barycentric Coordinates Interpolation**
    ///
    /// Interpolate vertex using barycentric coordinates (u, v, w) with u + v + w = 1:
    /// ```text
    /// p = u·p₁ + v·p₂ + w·p₃
    /// n = normalize(u·n₁ + v·n₂ + w·n₃)
    /// ```
    ///
    /// This is fundamental for triangle interpolation and surface
    /// parameterization. The incoming f64 weights are promoted to
    /// `hyperreal::Real` for the normalization decision and division, then
    /// exported back to the current f64 vertex boundary. That keeps the
    /// barycentric affine-combination predicate in the hyperreal path while
    /// this crate is still storing mesh vertices as boundary floats. For the
    /// mathematical role of barycentric coordinates in interpolation and
    /// geometry processing, see Floater, "Generalized barycentric coordinates
    /// and applications," *Acta Numerica* 24, 2015
    /// (<https://doi.org/10.1017/S0962492914000129>).
    pub fn barycentric_interpolate(
        v1: &Vertex,
        v2: &Vertex,
        v3: &Vertex,
        u: Real,
        v: Real,
        w: Real,
    ) -> Vertex {
        let weights = hyper_normalized_barycentric_weights(u, v, w).unwrap_or((
            1.0 / 3.0,
            1.0 / 3.0,
            1.0 / 3.0,
        ));
        let (u, v, w) = weights;

        let weights = [u, v, w];
        let new_position =
            hpoint_weighted_sum(&[v1.position, v2.position, v3.position], &weights)
                .unwrap_or_else(|| Point3::origin());

        let blended_normal =
            hvector3_weighted_sum(&[v1.normal, v2.normal, v3.normal], &weights)
                .unwrap_or_else(Vector3::z);
        let new_normal = hunit_vector3(&blended_normal).unwrap_or_else(Vector3::z);

        Vertex::new(new_position, new_normal)
    }
}

fn hyper_normalized_barycentric_weights(
    u: Real,
    v: Real,
    w: Real,
) -> Option<(Real, Real, Real)> {
    let u_h = hreal_from_f64(u).ok()?;
    let v_h = hreal_from_f64(v).ok()?;
    let w_h = hreal_from_f64(w).ok()?;
    let total = u_h.clone() + v_h.clone() + w_h.clone();

    if !hreal_gt_f64(&total, tolerance()) && !hreal_lt_f64(&total, -tolerance()) {
        return None;
    }

    Some((
        hreal_to_f64(&(u_h / total.clone()).ok()?)?,
        hreal_to_f64(&(v_h / total.clone()).ok()?)?,
        hreal_to_f64(&(w_h / total).ok()?)?,
    ))
}

fn hyper_normalized_weights(weights: impl IntoIterator<Item = Real>) -> Option<Vec<Real>> {
    let weights_h: Vec<_> = weights
        .into_iter()
        .map(hreal_from_f64)
        .collect::<Result<_, _>>()
        .ok()?;
    if weights_h.is_empty() {
        return None;
    }

    let total = weights_h
        .iter()
        .cloned()
        .fold(hyperreal::Real::zero(), |acc, weight| acc + weight);
    if !hreal_gt_f64(&total, tolerance()) {
        return None;
    }

    weights_h
        .into_iter()
        .map(|weight| hreal_to_f64(&(weight / total.clone()).ok()?))
        .collect()
}

fn hyper_vertex_distance_squared(lhs: &Vertex, rhs: &Vertex) -> Option<hyperreal::Real> {
    let lhs = hvector3_from_point3(&lhs.position)?;
    let rhs = hvector3_from_point3(&rhs.position)?;
    Some(lhs.squared_distance(&rhs))
}

fn hyper_cotangent_at_opposite(
    center: &Vertex,
    neighbor: &Vertex,
    opposite: &Vertex,
) -> Option<Real> {
    let center = hvector3_from_point3(&center.position)?;
    let neighbor = hvector3_from_point3(&neighbor.position)?;
    let opposite = hvector3_from_point3(&opposite.position)?;
    let edge1 = center - &opposite;
    let edge2 = neighbor - opposite;
    let cross = edge1.cross(&edge2);
    let cross_len = cross.dot(&cross).sqrt().ok()?;
    if !hreal_gt_f64(&cross_len, tolerance()) {
        return None;
    }
    hreal_to_f64(&(edge1.dot(&edge2) / cross_len).ok()?)
}

impl Vertex {
    /// **Mathematical Foundation: Edge-Length-Based Weighting**
    ///
    /// Compute cotangent weights for discrete Laplacian operators:
    /// ```text
    /// w_ij = (cot(α) + cot(β)) / 2
    /// ```
    /// Where α and β are the angles opposite to edge ij in adjacent triangles.
    ///
    /// This provides a better approximation to the continuous Laplacian
    /// operator compared to uniform weights. The cotangent ratio is evaluated
    /// as `(e1 · e2) / |e1 × e2|` with `hyperlattice::Vector3` and
    /// `hyperreal::Real`, avoiding local normalized f64 vectors for the
    /// degenerate-triangle decision. Cotangent weights are the standard local
    /// stencil used by discrete differential-geometry operators on triangle
    /// meshes; see Meyer, Desbrun, Schröder, and Barr, "Discrete
    /// Differential-Geometry Operators for Triangulated 2-Manifolds,"
    /// *Visualization and Mathematics III*, 2003
    /// (<https://doi.org/10.1007/978-3-662-05105-4_2>).
    pub fn compute_cotangent_weight(
        center: &Vertex,
        neighbor: &Vertex,
        triangle_vertices: &[&Vertex],
    ) -> Real {
        if triangle_vertices.len() < 3 {
            return 1.0; // Fallback to uniform weight
        }

        // Find the third vertex in the triangle
        let mut cotangents = Vec::new();

        for i in 0..triangle_vertices.len() {
            let v1 = triangle_vertices[i];
            let v2 = triangle_vertices[(i + 1) % triangle_vertices.len()];
            let v3 = triangle_vertices[(i + 2) % triangle_vertices.len()];

            // Check if this triangle contains our edge
            let contains_edge = (v1.position == center.position
                && v2.position == neighbor.position)
                || (v2.position == center.position && v3.position == neighbor.position)
                || (v3.position == center.position && v1.position == neighbor.position)
                || (v1.position == neighbor.position && v2.position == center.position)
                || (v2.position == neighbor.position && v3.position == center.position)
                || (v3.position == neighbor.position && v1.position == center.position);

            if contains_edge {
                // Find the vertex opposite to the edge
                let opposite = if v1.position != center.position
                    && v1.position != neighbor.position
                {
                    v1
                } else if v2.position != center.position && v2.position != neighbor.position {
                    v2
                } else {
                    v3
                };

                if let Some(cotangent) =
                    hyper_cotangent_at_opposite(center, neighbor, opposite)
                {
                    cotangents.push(cotangent);
                }
            }
        }

        if !cotangents.is_empty() {
            let cot_sum = hreal_sum(&cotangents).unwrap_or(0.0);
            hreal_div(cot_sum, 2.0 * cotangents.len() as Real).unwrap_or(1.0)
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
    /// Returns `(valence, regularity_score)` where regularity is in `[0, 1]`
    /// and `1` is optimal.
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
            let deviation = hreal_div(
                (valence as Real - target_valence as Real).abs(),
                target_valence as Real,
            )
            .unwrap_or(Real::INFINITY);
            hreal_div(1.0, 1.0 + deviation)
                .and_then(|regularity| hreal_clamp_f64(regularity, 0.0, 1.0))
                .unwrap_or(0.0)
        } else {
            0.0
        };

        (valence, regularity)
    }

    /// **Mathematical Foundation: Position-Based Vertex Lookup**
    ///
    /// Simplified connectivity analysis that searches for the vertex in the adjacency map
    /// by position matching (with tolerance). This is slower but more convenient
    /// when you don't have the global vertex index readily available.
    ///
    /// The position equality test promotes both candidate points into
    /// `hyperlattice::Vector3` and compares squared distance in
    /// `hyperreal::Real`, so the topological lookup avoids a local f64 square
    /// root and fails closed for non-finite boundary coordinates. This follows
    /// Yap's exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
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
        for (&idx, &position) in vertex_positions {
            if hpoints_within_epsilon(&self.position, &position, epsilon) {
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
    /// Direction normalization and angle extraction are delegated to
    /// `hyperlattice::Vector3`/`hyperreal::Real`, while the returned curvature
    /// remains a finite analysis metric. This follows the exact-geometric-
    /// computation boundary model of Yap, *Computational Geometry* 7(1-2),
    /// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the
    /// discrete differential-geometry context of Meyer et al.,
    /// "Discrete Differential-Geometry Operators for Triangulated
    /// 2-Manifolds" (<https://doi.org/10.1007/978-3-662-05105-4_2>).
    pub fn estimate_mean_curvature(&self, neighbors: &[Vertex], face_areas: &[Real]) -> Real {
        if neighbors.len() < 3 {
            return 0.0;
        }

        // Compute angle sum around vertex
        let mut angles = Vec::with_capacity(neighbors.len());
        for i in 0..neighbors.len() {
            let prev = &neighbors[(i + neighbors.len() - 1) % neighbors.len()];
            let next = &neighbors[(i + 1) % neighbors.len()];

            let v1 = prev.position - self.position;
            let v2 = next.position - self.position;
            if let Some(angle) = hangle_between_vectors(&v1, &v2) {
                angles.push(angle);
            }
        }
        let angle_sum = hreal_sum(&angles).unwrap_or(0.0);

        // Compute mixed area (average of face areas)
        let finite_face_areas: Vec<_> = face_areas
            .iter()
            .copied()
            .filter(|area| {
                hreal_from_f64(*area)
                    .ok()
                    .is_some_and(|area| hreal_gt_f64(&area, 0.0))
            })
            .collect();
        let mixed_area = hreal_mean(&finite_face_areas).unwrap_or(1.0);

        // Discrete mean curvature
        hreal_mul(2.0, PI)
            .and_then(|full_turn| hreal_sub(full_turn, angle_sum))
            .filter(|_| {
                hreal_from_f64(mixed_area)
                    .ok()
                    .is_some_and(|area| hreal_gt_f64(&area, tolerance()))
            })
            .and_then(|angle_deficit| hreal_div(angle_deficit, mixed_area))
            .unwrap_or(0.0)
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
    /// Incident edge distances, normal-angle variation, and normal deviation
    /// are measured through `hyperlattice::Vector3` adapters before exporting
    /// finite scores. Keeping these analysis predicates out of local f64
    /// normalization follows Yap's exact-geometric-computation boundary
    /// discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
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
            if let Some(&neighbor_position) = vertex_positions.get(&neighbor_idx) {
                if let Some(edge_length) = hpoint_distance(&self.position, &neighbor_position)
                {
                    edge_lengths.push(edge_length);
                }

                if let Some(&neighbor_normal) = vertex_normals.get(&neighbor_idx) {
                    neighbor_normals.push(neighbor_normal);
                }
            }
        }

        // Edge uniformity (lower standard deviation = more uniform)
        let edge_uniformity = if edge_lengths.len() > 1 {
            let mean_edge = hreal_mean(&edge_lengths).unwrap_or(0.0);
            let std_dev = hreal_sample_stddev(&edge_lengths).unwrap_or(0.0);

            // Normalize to [0,1] where 1 = perfectly uniform
            hreal_from_f64(mean_edge)
                .ok()
                .filter(|mean| hreal_gt_f64(mean, tolerance()))
                .and_then(|_| hreal_div(std_dev, mean_edge))
                .and_then(|ratio| hreal_div(1.0, 1.0 + ratio))
                .and_then(|uniformity| hreal_clamp_f64(uniformity, 0.0, 1.0))
                .unwrap_or(0.0)
        } else {
            1.0
        };

        // Normal variation (lower = more consistent normals)
        let normal_variation = if neighbor_normals.len() > 1 {
            let mut max_angle: Option<Real> = None;
            for &neighbor_normal in &neighbor_normals {
                if let Some(angle) = hangle_between_vectors(&self.normal, &neighbor_normal) {
                    if let Ok(angle_h) = hreal_from_f64(angle) {
                        max_angle = hreal_max_report_value(max_angle, &angle_h);
                    }
                }
            }

            // Normalize to [0,1] where 1 = perfectly consistent
            max_angle
                .and_then(|angle| hreal_div(angle, PI))
                .and_then(|ratio| hreal_clamp_f64(ratio, 0.0, 1.0))
                .and_then(|ratio| hreal_sub(1.0, ratio))
                .unwrap_or(0.0)
        } else {
            1.0
        };

        // Simple curvature estimation based on normal variation
        let curvature = if !neighbor_normals.is_empty() {
            let avg_normal = hvector3_mean(&neighbor_normals).unwrap_or_else(Vector3::zeros);
            // Fail closed instead of re-entering primitive nalgebra norms when
            // hostile public normals cannot be promoted to hyperlattice.
            hvector3_distance(&self.normal, &avg_normal).unwrap_or(0.0)
        } else {
            0.0
        };

        (regularity, curvature, edge_uniformity, normal_variation)
    }
}

/// **Mathematical Foundation: Vertex Clustering for Mesh Simplification**
///
/// Advanced vertex operations for mesh processing and optimization.
pub struct VertexCluster {
    /// Representative position (typically centroid)
    pub position: Point3<Real>,
    /// Averaged normal vector
    pub normal: Vector3<Real>,
    /// Number of vertices in cluster
    pub count: usize,
    /// Bounding radius of cluster
    pub radius: Real,
}

impl VertexCluster {
    /// Create a new vertex cluster from a collection of vertices.
    ///
    /// Normal normalization and radius distances are computed through
    /// hyperlattice/hyperreal helpers before finite export. The cluster itself
    /// is still a mesh-analysis boundary type, but the geometric predicates
    /// follow Yap's exact-geometric-computation split between primitive
    /// boundary coordinates and exact-aware objects
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_vertices(vertices: &[Vertex]) -> Option<Self> {
        if vertices.is_empty() {
            return None;
        }

        // Compute centroid position
        let positions = vertices
            .iter()
            .map(|vertex| vertex.position)
            .collect::<Vec<_>>();
        let centroid = hpoint_centroid(&positions)?;

        // Compute average normal
        let normals = vertices
            .iter()
            .map(|vertex| vertex.normal)
            .collect::<Vec<_>>();
        let avg_normal = hvector3_mean(&normals).unwrap_or_else(Vector3::z);
        let normalized_normal = hunit_vector3(&avg_normal).unwrap_or_else(Vector3::z);

        // Compute bounding radius
        let radius = vertices
            .iter()
            .filter_map(|v| hpoint_distance(&v.position, &centroid))
            .fold(None, |max_radius, radius| {
                hreal_from_f64(radius)
                    .ok()
                    .and_then(|radius| hreal_max_report_value(max_radius, &radius))
            })
            .unwrap_or(0.0);

        Some(VertexCluster {
            position: centroid,
            normal: normalized_normal,
            count: vertices.len(),
            radius,
        })
    }

    /// Convert cluster back to a representative vertex
    pub const fn to_vertex(&self) -> Vertex {
        Vertex::new(self.position, self.normal)
    }
}

#[cfg(test)]
mod test {

    use nalgebra::{Const, OPoint};

    use super::*;

    #[test]
    pub fn test_sanitise_vertices() {
        let vertex = Vertex::new(
            OPoint::<Real, Const<3>>::new(Real::INFINITY, Real::INFINITY, Real::INFINITY),
            Vector3::new(Real::INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY),
        );

        assert!(vertex.position.iter().copied().all(Real::is_finite));
        assert!(vertex.normal.iter().copied().all(Real::is_finite));
    }
}
