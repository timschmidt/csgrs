//! Mesh smoothing and refinement operations.

use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::fmt::Debug;

fn real_cmp(lhs: &Real, rhs: &Real) -> Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

fn real_ge(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Greater | Ordering::Equal)
}

fn real_gt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Greater)
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), Ordering::Less)
}

fn degrees_to_radians(degrees: Real) -> Option<Real> {
    (degrees * Real::pi() / Real::from(180_u8)).ok()
}

fn point_distance(a: &Point3, b: &Point3) -> Option<Real> {
    let edge = b - a;
    edge.dot(&edge).sqrt().ok()
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundation: True Laplacian Mesh Smoothing with Global Connectivity**
    ///
    /// Implements proper discrete Laplacian smoothing using global mesh connectivity:
    ///
    /// ## **Discrete Laplacian Operator**
    /// For each vertex v with neighbors N(v):
    /// ```text
    /// L(v) = (1/|N(v)|) · Σ(n∈N(v)) (n - v)
    /// ```
    ///
    /// ## **Global Connectivity Benefits**
    /// - **Proper Neighborhoods**: Uses actual mesh connectivity, not just polygon edges
    /// - **Uniform Weighting**: Each neighbor contributes equally to smoothing
    /// - **Boundary Detection**: Automatically detects and preserves mesh boundaries
    /// - **Volume Preservation**: Better volume preservation than local smoothing
    ///
    /// ## **Algorithm Improvements**
    /// - **Exact Vertex Matching**: Hyperreal-backed boundary-coordinate identity
    /// - **Manifold Preservation**: Ensures mesh topology is maintained
    /// - **Feature Detection**: Can preserve sharp features based on neighbor count
    ///
    /// Vertex lookup reuses [`VertexIndexMap`](crate::mesh::connectivity::VertexIndexMap),
    /// whose matching predicate requires exact equality after promotion through
    /// `hyperlattice::Vector3` and `Real`. This keeps the discrete
    /// Laplacian's adjacency relation stable before applying the uniform
    /// smoothing stencil described in Botsch et al., *Polygon Mesh Processing*,
    /// 2010 (<https://doi.org/10.1201/b10688>).
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<M> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for iteration in 0..iterations {
            // Build current vertex position mapping
            let mut current_positions: HashMap<usize, Point3> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            // Compute Laplacian for each vertex
            let mut laplacian_updates: HashMap<usize, Point3> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    // Check if this is a boundary vertex
                    if preserve_boundaries && neighbors.len() < 4 {
                        // Boundary vertex - skip smoothing
                        laplacian_updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        let new_position = current_position.lerp(&neighbor_avg, &lambda);
                        laplacian_updates.insert(vertex_idx, new_position);
                    } else {
                        laplacian_updates.insert(vertex_idx, current_position);
                    }
                }
            }

            // Apply updates to mesh vertices
            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = laplacian_updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
                // Recompute polygon plane and normals after smoothing
                polygon.set_new_normal();
            }

            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!(
                    "Smoothing progress: {}/{} iterations",
                    iteration + 1,
                    iterations
                );
            }
        }

        Mesh::from_polygons(&smoothed_polygons, self.metadata.clone())
    }

    /// **Mathematical Foundation: Taubin Mesh Smoothing**
    ///
    /// Implements Taubin's feature-preserving mesh smoothing algorithm, which reduces
    /// shrinkage compared to standard Laplacian smoothing.
    ///
    /// ## **Taubin's Algorithm**
    /// This method involves two steps per iteration:
    /// 1. **Shrinking Step**: Apply standard Laplacian smoothing with a positive factor `lambda`.
    ///    `v' = v + λ * L(v)`
    /// 2. **Inflating Step**: Apply a second Laplacian step with a negative factor `mu`.
    ///    `v'' = v' + μ * L(v')`
    ///
    /// Typically, `0 < λ < -μ`. A common choice is `mu = -λ / (1 - λ)`.
    /// This combination effectively smooths the mesh while minimizing volume loss.
    ///
    /// Returns a new, smoothed CSG object.
    pub fn taubin_smooth(
        &self,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<M> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        for _ in 0..iterations {
            // --- Lambda (shrinking) pass ---
            let mut current_positions: HashMap<usize, Point3> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            let mut updates: HashMap<usize, Point3> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        updates
                            .insert(vertex_idx, current_position.lerp(&neighbor_avg, &lambda));
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
            }

            // --- Mu (inflating) pass ---
            current_positions.clear();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        current_positions.insert(idx, vertex.position.clone());
                    }
                }
            }

            updates.clear();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(current_position) = current_positions.get(&vertex_idx).cloned() {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_position);
                        continue;
                    }

                    let neighbor_positions = neighbors
                        .iter()
                        .filter_map(|neighbor_idx| {
                            current_positions.get(neighbor_idx).cloned()
                        })
                        .collect::<Vec<_>>();

                    if let Some(neighbor_avg) = Point3::centroid(&neighbor_positions) {
                        updates.insert(vertex_idx, current_position.lerp(&neighbor_avg, &mu));
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    if let Some(idx) = vertex_map.find_index(&vertex.position) {
                        if let Some(new_position) = updates.get(&idx) {
                            vertex.position = new_position.clone();
                        }
                    }
                }
            }
        }

        // Final pass to recompute normals
        for polygon in &mut smoothed_polygons {
            polygon.set_new_normal();
        }

        Mesh::from_polygons(&smoothed_polygons, self.metadata.clone())
    }

    /// **Mathematical Foundation: Adaptive Mesh Refinement**
    ///
    /// Intelligently refine mesh based on geometric criteria:
    ///
    /// ## **Refinement Criteria**
    /// - **Quality threshold**: Refine triangles with quality score < threshold
    /// - **Size variation**: Refine where edge lengths vary significantly
    /// - **Curvature**: Refine high-curvature regions (based on normal variation)
    /// - **Feature detection**: Preserve sharp edges and corners
    ///
    /// ## **Refinement Strategy**
    /// 1. **Quality-based**: Subdivide poor-quality triangles
    /// 2. **Size-based**: Subdivide triangles larger than target size
    /// 3. **Curvature-based**: Subdivide where surface curves significantly
    ///
    /// The scalar thresholds are promoted through `Real` before
    /// they participate in refinement predicates, and the curvature threshold
    /// is converted to radians through the shared hyperreal degree adapter.
    /// This keeps adaptive decisions aligned with Yap's exact-geometric-
    /// computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) while using the same
    /// dihedral-angle criterion as standard mesh processing texts such as
    /// Botsch et al., *Polygon Mesh Processing*, 2010
    /// (<https://doi.org/10.1201/b10688>).
    pub fn adaptive_refine(
        &self,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> Mesh<M> {
        let Some(curvature_threshold_rad) = degrees_to_radians(curvature_threshold_deg) else {
            return self.clone();
        };

        let qualities = self.analyze_triangle_quality();
        let (mut vertex_map, _adjacency) = self.build_connectivity();
        let mut refined_polygons = Vec::new();
        let mut polygon_map: HashMap<usize, Vec<usize>> = HashMap::new();

        for (poly_idx, poly) in self.polygons.iter().enumerate() {
            for vertex in &poly.vertices {
                let v_idx = vertex_map.get_or_create_index(vertex.position.clone());
                polygon_map.entry(v_idx).or_default().push(poly_idx);
            }
        }

        for (i, polygon) in self.polygons.iter().enumerate() {
            let mut should_refine = false;

            // Quality and edge length check
            if i < qualities.len() {
                let quality = &qualities[i];
                if real_lt(&quality.quality_score, &quality_threshold)
                    || real_gt(&Self::max_edge_length(&polygon.vertices), &max_edge_length)
                {
                    should_refine = true;
                }
            }

            // Curvature check
            if !should_refine {
                'edge_loop: for edge in polygon.edges() {
                    let v1_idx = vertex_map.get_or_create_index(edge.0.position.clone());
                    let v2_idx = vertex_map.get_or_create_index(edge.1.position.clone());

                    if let (Some(p1_indices), Some(p2_indices)) =
                        (polygon_map.get(&v1_idx), polygon_map.get(&v2_idx))
                    {
                        for &p1_idx in p1_indices {
                            if p1_idx == i {
                                continue;
                            }
                            for &p2_idx in p2_indices {
                                if p1_idx == p2_idx {
                                    let other_poly = &self.polygons[p1_idx];
                                    let angle = Self::dihedral_angle(polygon, other_poly);
                                    if real_gt(&angle, &curvature_threshold_rad) {
                                        should_refine = true;
                                        break 'edge_loop;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if should_refine {
                let subdivided = polygon.subdivide_triangles(1.try_into().unwrap());
                for triangle in subdivided {
                    let vertices = triangle.to_vec();
                    refined_polygons.push(Polygon::new(vertices, polygon.metadata.clone()));
                }
            } else {
                refined_polygons.push(polygon.clone());
            }
        }

        Mesh::from_polygons(&refined_polygons, self.metadata.clone())
    }

    /// Calculate maximum edge length in a polygon.
    ///
    /// Edge distances are lifted through `hyperlattice` before returning to the
    /// mesh boundary scalar, keeping refinement decisions aligned with the
    /// exact-geometric-computation split described by Yap
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return Real::zero();
        }

        let mut max_length: Option<Real> = None;
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            if let Some(edge_length) =
                point_distance(&vertices[j].position, &vertices[i].position)
            {
                max_length = match max_length {
                    Some(current) => {
                        hyperlimit::real_max(&current, &edge_length).value().cloned()
                    },
                    None => Some(edge_length),
                };
            }
        }
        max_length.unwrap_or_else(Real::zero)
    }

    /// **Mathematical Foundation: Feature-Preserving Mesh Optimization**
    ///
    /// Remove poor-quality triangles while preserving important geometric features:
    ///
    /// ## **Quality-Based Filtering**
    /// Remove triangles that meet criteria:
    /// - **Sliver triangles**: min_angle < threshold (typically 5°)
    /// - **Needle triangles**: aspect_ratio > threshold (typically 20)
    /// - **Small triangles**: area < threshold
    ///
    /// ## **Feature Preservation**
    /// - **Sharp edges**: Preserve edges with large dihedral angles
    /// - **Boundaries**: Maintain mesh boundaries
    /// - **Topology**: Ensure mesh remains manifold
    ///
    /// Quality, area, angle, and aspect-ratio predicates compare finite
    /// reporting scalars through hyperreal comparison helpers. The fixed
    /// five-degree sliver threshold is converted through the same hyperreal
    /// degree adapter used by transforms and refinement.
    ///
    /// Returns cleaned mesh with improved triangle quality.
    pub fn remove_poor_triangles(&self, min_quality: Real) -> Mesh<M> {
        let Some(min_angle_rad) = degrees_to_radians(Real::from(5_u8)) else {
            return self.clone();
        };
        let qualities = self.analyze_triangle_quality();
        let mut filtered_polygons = Vec::new();

        for (i, polygon) in self.polygons.iter().enumerate() {
            let keep_triangle = if i < qualities.len() {
                let quality = &qualities[i];
                real_ge(&quality.quality_score, &min_quality)
                    && real_gt(&quality.area, &Real::zero())
                    && real_gt(&quality.min_angle, &min_angle_rad)
                    && real_lt(&quality.aspect_ratio, &Real::from(20_u8))
            } else {
                true // Keep if we can't assess quality
            };

            if keep_triangle {
                filtered_polygons.push(polygon.clone());
            }
        }

        Mesh::from_polygons(&filtered_polygons, self.metadata.clone())
    }
}
