//! **IndexedMesh Vertex Operations**
//!
//! Optimized vertex operations specifically designed for IndexedMesh's indexed connectivity model.
//! This module provides index-aware vertex operations that leverage shared vertex storage
//! for maximum performance and memory efficiency.

use crate::IndexedMesh::IndexedMesh;
use crate::float_types::{PI, Real};
use nalgebra::{Point3, Vector3};

/// **IndexedVertex: Optimized Vertex for Indexed Connectivity**
///
/// Enhanced vertex structure optimized for IndexedMesh operations.
/// Maintains the same core data as regular Vertex but with additional
/// index-aware functionality.
#[derive(Debug, Clone, PartialEq, Copy)]
pub struct IndexedVertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl IndexedVertex {
    /// Create a new IndexedVertex with sanitized coordinates
    #[inline]
    pub const fn new(mut pos: Point3<Real>, mut normal: Vector3<Real>) -> Self {
        // Sanitise position - const-compatible loop unrolling
        let [[x, y, z]]: &mut [[_; 3]; 1] = &mut pos.coords.data.0;
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

        IndexedVertex { pos, normal }
    }

    /// Flip vertex normal
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// **Index-Aware Linear Interpolation**
    ///
    /// Optimized interpolation that can be used for creating new vertices
    /// during edge splitting operations in IndexedMesh.
    pub fn interpolate(&self, other: &IndexedVertex, t: Real) -> IndexedVertex {
        let new_pos = self.pos + (other.pos - self.pos) * t;
        let new_normal = self.normal + (other.normal - self.normal) * t;
        IndexedVertex::new(new_pos, new_normal)
    }

    /// **Spherical Linear Interpolation for Normals**
    ///
    /// High-quality normal interpolation preserving unit length.
    /// Ideal for smooth shading in indexed meshes.
    pub fn slerp_interpolate(&self, other: &IndexedVertex, t: Real) -> IndexedVertex {
        let new_pos = self.pos + (other.pos - self.pos) * t;

        let n0 = self.normal.normalize();
        let n1 = other.normal.normalize();
        let dot = n0.dot(&n1).clamp(-1.0, 1.0);

        // Handle nearly parallel normals
        if (dot.abs() - 1.0).abs() < Real::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return IndexedVertex::new(new_pos, new_normal);
        }

        let omega = dot.acos();
        let sin_omega = omega.sin();

        if sin_omega.abs() < Real::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return IndexedVertex::new(new_pos, new_normal);
        }

        let a = ((1.0 - t) * omega).sin() / sin_omega;
        let b = (t * omega).sin() / sin_omega;
        let new_normal = (a * n0 + b * n1).normalize();

        IndexedVertex::new(new_pos, new_normal)
    }

    /// Distance between vertex positions
    pub fn distance_to(&self, other: &IndexedVertex) -> Real {
        (self.pos - other.pos).norm()
    }

    /// Squared distance (avoids sqrt for performance)
    pub fn distance_squared_to(&self, other: &IndexedVertex) -> Real {
        (self.pos - other.pos).norm_squared()
    }

    /// Angle between normal vectors
    pub fn normal_angle_to(&self, other: &IndexedVertex) -> Real {
        let n1 = self.normal.normalize();
        let n2 = other.normal.normalize();
        let cos_angle = n1.dot(&n2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }
}

/// **IndexedVertexOperations: Advanced Index-Aware Vertex Operations**
///
/// Collection of static methods for performing advanced vertex operations
/// on IndexedMesh structures using index-based algorithms.
pub struct IndexedVertexOperations;

impl IndexedVertexOperations {
    /// **Index-Based Weighted Average**
    ///
    /// Compute weighted average of vertices using their indices in the mesh.
    /// This is more efficient than copying vertex data.
    pub fn weighted_average_by_indices<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        vertex_weights: &[(usize, Real)],
    ) -> Option<IndexedVertex> {
        if vertex_weights.is_empty() {
            return None;
        }

        let total_weight: Real = vertex_weights.iter().map(|(_, w)| *w).sum();
        if total_weight < Real::EPSILON {
            return None;
        }

        let mut weighted_pos = Point3::origin();
        let mut weighted_normal = Vector3::zeros();

        for &(vertex_idx, weight) in vertex_weights {
            if vertex_idx < mesh.vertices.len() {
                let vertex = &mesh.vertices[vertex_idx];
                weighted_pos += vertex.pos.coords * weight;
                weighted_normal += vertex.normal * weight;
            }
        }

        weighted_pos /= total_weight;
        let normalized_normal = if weighted_normal.norm() > Real::EPSILON {
            weighted_normal.normalize()
        } else {
            Vector3::z()
        };

        Some(IndexedVertex::new(
            Point3::from(weighted_pos),
            normalized_normal,
        ))
    }

    /// **Barycentric Interpolation by Indices**
    ///
    /// Interpolate vertex using barycentric coordinates and vertex indices.
    /// Optimized for IndexedMesh triangle operations.
    pub fn barycentric_interpolate_by_indices<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        v1_idx: usize,
        v2_idx: usize,
        v3_idx: usize,
        u: Real,
        v: Real,
        w: Real,
    ) -> Option<IndexedVertex> {
        if v1_idx >= mesh.vertices.len()
            || v2_idx >= mesh.vertices.len()
            || v3_idx >= mesh.vertices.len()
        {
            return None;
        }

        let v1 = &mesh.vertices[v1_idx];
        let v2 = &mesh.vertices[v2_idx];
        let v3 = &mesh.vertices[v3_idx];

        // Normalize barycentric coordinates
        let total = u + v + w;
        let (u, v, w) = if total.abs() > Real::EPSILON {
            (u / total, v / total, w / total)
        } else {
            (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
        };

        let new_pos = Point3::from(u * v1.pos.coords + v * v2.pos.coords + w * v3.pos.coords);
        let new_normal = (u * v1.normal + v * v2.normal + w * v3.normal).normalize();

        Some(IndexedVertex::new(new_pos, new_normal))
    }

    /// **Index-Based Connectivity Analysis**
    ///
    /// Analyze vertex connectivity using the mesh's indexed structure.
    /// Returns (valence, regularity_score) for the specified vertex.
    pub fn analyze_connectivity<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        vertex_idx: usize,
    ) -> (usize, Real) {
        if vertex_idx >= mesh.vertices.len() {
            return (0, 0.0);
        }

        // Count adjacent faces (valence approximation)
        let mut adjacent_faces = 0;
        for polygon in &mesh.polygons {
            if polygon.indices.contains(&vertex_idx) {
                adjacent_faces += 1;
            }
        }

        // Estimate valence (each face contributes ~2 edges on average for triangular meshes)
        let estimated_valence = adjacent_faces * 2 / 3;

        // Regularity score (optimal valence is 6 for interior vertices)
        let target_valence = 6;
        let regularity = if estimated_valence > 0 {
            let deviation = (estimated_valence as Real - target_valence as Real).abs();
            (1.0 / (1.0 + deviation / target_valence as Real)).max(0.0)
        } else {
            0.0
        };

        (estimated_valence, regularity)
    }

    /// **Index-Based Curvature Estimation**
    ///
    /// Estimate discrete mean curvature at a vertex using adjacent face information.
    /// Uses the angle deficit method optimized for indexed connectivity.
    pub fn estimate_mean_curvature<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        vertex_idx: usize,
    ) -> Real {
        if vertex_idx >= mesh.vertices.len() {
            return 0.0;
        }

        let vertex = &mesh.vertices[vertex_idx];
        let mut angle_sum = 0.0;
        let mut face_count = 0;

        // Find all faces containing this vertex and compute angles
        for polygon in &mesh.polygons {
            if let Some(pos) = polygon.indices.iter().position(|&idx| idx == vertex_idx) {
                let n = polygon.indices.len();
                if n >= 3 {
                    let prev_idx = polygon.indices[(pos + n - 1) % n];
                    let next_idx = polygon.indices[(pos + 1) % n];

                    if prev_idx < mesh.vertices.len() && next_idx < mesh.vertices.len() {
                        let prev_pos = mesh.vertices[prev_idx].pos;
                        let next_pos = mesh.vertices[next_idx].pos;

                        let v1 = (prev_pos - vertex.pos).normalize();
                        let v2 = (next_pos - vertex.pos).normalize();
                        let dot = v1.dot(&v2).clamp(-1.0, 1.0);
                        angle_sum += dot.acos();
                        face_count += 1;
                    }
                }
            }
        }

        if face_count > 0 {
            let angle_deficit = 2.0 * PI - angle_sum;
            // Approximate mixed area using average face area
            let mixed_area = 1.0; // Simplified - could be computed more accurately
            angle_deficit / mixed_area
        } else {
            0.0
        }
    }
}

/// **Conversion between regular Vertex and IndexedVertex**
impl From<crate::mesh::vertex::Vertex> for IndexedVertex {
    fn from(vertex: crate::mesh::vertex::Vertex) -> Self {
        IndexedVertex::new(vertex.pos, vertex.normal)
    }
}

impl From<IndexedVertex> for crate::mesh::vertex::Vertex {
    fn from(vertex: IndexedVertex) -> Self {
        crate::mesh::vertex::Vertex::new(vertex.pos, vertex.normal)
    }
}

/// **IndexedVertexCluster: Advanced Vertex Clustering for IndexedMesh**
///
/// Optimized vertex clustering specifically designed for IndexedMesh operations.
/// Provides efficient vertex grouping and representative selection.
#[derive(Debug, Clone)]
pub struct IndexedVertexCluster {
    /// Representative vertex indices in the mesh
    pub vertex_indices: Vec<usize>,
    /// Cluster centroid position
    pub centroid: Point3<Real>,
    /// Average normal vector
    pub normal: Vector3<Real>,
    /// Bounding radius of cluster
    pub radius: Real,
}

impl IndexedVertexCluster {
    /// Create cluster from vertex indices in a mesh
    pub fn from_indices<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        indices: Vec<usize>,
    ) -> Option<Self> {
        if indices.is_empty() {
            return None;
        }

        // Validate indices and collect valid vertices
        let valid_indices: Vec<usize> = indices
            .into_iter()
            .filter(|&idx| idx < mesh.vertices.len())
            .collect();

        if valid_indices.is_empty() {
            return None;
        }

        // Compute centroid
        let centroid = valid_indices.iter().fold(Point3::origin(), |acc, &idx| {
            acc + mesh.vertices[idx].pos.coords
        }) / valid_indices.len() as Real;

        // Compute average normal
        let avg_normal = valid_indices
            .iter()
            .fold(Vector3::zeros(), |acc, &idx| acc + mesh.vertices[idx].normal);
        let normalized_normal = if avg_normal.norm() > Real::EPSILON {
            avg_normal.normalize()
        } else {
            Vector3::z()
        };

        // Compute bounding radius
        let radius = valid_indices
            .iter()
            .map(|&idx| (mesh.vertices[idx].pos - Point3::from(centroid)).norm())
            .fold(0.0, |a: Real, b| a.max(b));

        Some(IndexedVertexCluster {
            vertex_indices: valid_indices,
            centroid: Point3::from(centroid),
            normal: normalized_normal,
            radius,
        })
    }

    /// Convert cluster to a representative IndexedVertex
    pub const fn to_indexed_vertex(&self) -> IndexedVertex {
        IndexedVertex::new(self.centroid, self.normal)
    }

    /// Get cluster quality metrics
    pub fn quality_metrics<S: Clone + Send + Sync + std::fmt::Debug>(
        &self,
        mesh: &IndexedMesh<S>,
    ) -> (Real, Real, Real) {
        if self.vertex_indices.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        // Compactness: ratio of average distance to radius
        let avg_distance = self
            .vertex_indices
            .iter()
            .map(|&idx| (mesh.vertices[idx].pos - self.centroid).norm())
            .sum::<Real>()
            / self.vertex_indices.len() as Real;
        let compactness = if self.radius > Real::EPSILON {
            avg_distance / self.radius
        } else {
            1.0
        };

        // Normal consistency: how aligned are the normals
        let normal_consistency = if self.vertex_indices.len() > 1 {
            let mut min_dot: Real = 1.0;
            for &idx in &self.vertex_indices {
                let dot = self.normal.dot(&mesh.vertices[idx].normal.normalize());
                min_dot = min_dot.min(dot);
            }
            min_dot.max(0.0)
        } else {
            1.0
        };

        // Density: vertices per unit volume
        let volume = if self.radius > Real::EPSILON {
            (4.0 / 3.0) * PI * self.radius.powi(3)
        } else {
            Real::EPSILON
        };
        let density = self.vertex_indices.len() as Real / volume;

        (compactness, normal_consistency, density)
    }
}

/// **Advanced Vertex Clustering Operations**
pub struct IndexedVertexClustering;

impl IndexedVertexClustering {
    /// **K-Means Clustering for Vertices**
    ///
    /// Perform k-means clustering on mesh vertices using position and normal.
    /// Returns cluster assignments for each vertex.
    pub fn k_means_clustering<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        k: usize,
        max_iterations: usize,
        position_weight: Real,
        normal_weight: Real,
    ) -> Vec<usize> {
        if mesh.vertices.is_empty() || k == 0 {
            return Vec::new();
        }

        let n_vertices = mesh.vertices.len();
        let k = k.min(n_vertices);

        // Initialize cluster centers randomly
        let mut cluster_centers = Vec::with_capacity(k);
        for i in 0..k {
            let idx = (i * n_vertices) / k;
            cluster_centers.push(mesh.vertices[idx]);
        }

        let mut assignments = vec![0; n_vertices];

        for _ in 0..max_iterations {
            let mut changed = false;

            // Assign vertices to nearest cluster
            for (vertex_idx, vertex) in mesh.vertices.iter().enumerate() {
                let mut best_cluster = 0;
                let mut best_distance = Real::MAX;

                for (cluster_idx, center) in cluster_centers.iter().enumerate() {
                    let pos_dist = (vertex.pos - center.pos).norm_squared();
                    let normal_dist = (vertex.normal - center.normal).norm_squared();
                    let distance = position_weight * pos_dist + normal_weight * normal_dist;

                    if distance < best_distance {
                        best_distance = distance;
                        best_cluster = cluster_idx;
                    }
                }

                if assignments[vertex_idx] != best_cluster {
                    assignments[vertex_idx] = best_cluster;
                    changed = true;
                }
            }

            if !changed {
                break;
            }

            // Update cluster centers
            let mut cluster_sums = vec![(Point3::origin(), Vector3::zeros(), 0); k];

            for (vertex_idx, &cluster_idx) in assignments.iter().enumerate() {
                let vertex = &mesh.vertices[vertex_idx];
                cluster_sums[cluster_idx].0 += vertex.pos.coords;
                cluster_sums[cluster_idx].1 += vertex.normal;
                cluster_sums[cluster_idx].2 += 1;
            }

            for (cluster_idx, (pos_sum, normal_sum, count)) in cluster_sums.iter().enumerate()
            {
                if *count > 0 {
                    let avg_pos = Point3::from(pos_sum.coords / *count as Real);
                    let avg_normal = if normal_sum.norm() > Real::EPSILON {
                        normal_sum.normalize()
                    } else {
                        Vector3::z()
                    };
                    cluster_centers[cluster_idx] =
                        IndexedVertex::new(avg_pos, avg_normal).into();
                }
            }
        }

        assignments
    }

    /// **Hierarchical Clustering**
    ///
    /// Perform hierarchical clustering using single linkage.
    /// Returns cluster tree as nested vectors.
    pub fn hierarchical_clustering<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        distance_threshold: Real,
    ) -> Vec<Vec<usize>> {
        if mesh.vertices.is_empty() {
            return Vec::new();
        }

        let n_vertices = mesh.vertices.len();
        let mut clusters: Vec<Vec<usize>> = (0..n_vertices).map(|i| vec![i]).collect();

        while clusters.len() > 1 {
            let mut min_distance = Real::MAX;
            let mut merge_indices = (0, 1);

            // Find closest pair of clusters
            for i in 0..clusters.len() {
                for j in (i + 1)..clusters.len() {
                    let distance = Self::cluster_distance(mesh, &clusters[i], &clusters[j]);
                    if distance < min_distance {
                        min_distance = distance;
                        merge_indices = (i, j);
                    }
                }
            }

            // Stop if minimum distance exceeds threshold
            if min_distance > distance_threshold {
                break;
            }

            // Merge closest clusters
            let (i, j) = merge_indices;
            let mut merged = clusters[i].clone();
            merged.extend(&clusters[j]);

            // Remove old clusters and add merged one
            clusters.remove(j); // Remove j first (higher index)
            clusters.remove(i);
            clusters.push(merged);
        }

        clusters
    }

    /// Compute distance between two clusters (single linkage)
    fn cluster_distance<S: Clone + Send + Sync + std::fmt::Debug>(
        mesh: &IndexedMesh<S>,
        cluster1: &[usize],
        cluster2: &[usize],
    ) -> Real {
        let mut min_distance = Real::MAX;

        for &idx1 in cluster1 {
            for &idx2 in cluster2 {
                if idx1 < mesh.vertices.len() && idx2 < mesh.vertices.len() {
                    let distance = (mesh.vertices[idx1].pos - mesh.vertices[idx2].pos).norm();
                    min_distance = min_distance.min(distance);
                }
            }
        }

        min_distance
    }
}
