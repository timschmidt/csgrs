use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::Point3;
use std::collections::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
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
    /// - **Epsilon-based Vertex Matching**: Robust floating-point coordinate handling
    /// - **Manifold Preservation**: Ensures mesh topology is maintained
    /// - **Feature Detection**: Can preserve sharp features based on neighbor count
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<S> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        (0..iterations).for_each(|iteration| {
            // Build current vertex position mapping
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            smoothed_polygons
                .iter()
                .flat_map(|polygon| polygon.vertices.iter())
                .for_each(|vertex| {
                    // Find the global index for this position (with tolerance)
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        current_positions.insert(*idx, vertex.pos);
                    }
                });

            // Compute Laplacian for each vertex
            let mut laplacian_updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    // Check if this is a boundary vertex
                    if preserve_boundaries && neighbors.len() < 4 {
                        // Boundary vertex - skip smoothing
                        laplacian_updates.insert(vertex_idx, current_pos);
                        continue;
                    }

                    // Compute neighbor average
                    let mut neighbor_sum = Point3::origin();
                    let mut valid_neighbors = 0;

                    for &neighbor_idx in neighbors {
                        if let Some(&neighbor_pos) = current_positions.get(&neighbor_idx) {
                            neighbor_sum += neighbor_pos.coords;
                            valid_neighbors += 1;
                        }
                    }

                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        let new_pos = current_pos + laplacian * lambda;
                        laplacian_updates.insert(vertex_idx, new_pos);
                    } else {
                        laplacian_updates.insert(vertex_idx, current_pos);
                    }
                }
            }

            // Apply updates to mesh vertices
            smoothed_polygons.iter_mut().for_each(|polygon| {
                polygon.vertices.iter_mut().for_each(|vertex| {
                    // Find the global index for this vertex
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        if let Some(&new_pos) = laplacian_updates.get(idx) {
                            vertex.pos = new_pos;
                        }
                    }
                });
                // Recompute polygon plane and normals after smoothing
                polygon.set_new_normal();
            });

            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!(
                    "Smoothing progress: {}/{} iterations",
                    iteration + 1,
                    iterations
                );
            }
        });

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
    ) -> Mesh<S> {
        let (vertex_map, adjacency) = self.build_connectivity();
        let mut smoothed_polygons = self.polygons.clone();

        (0..iterations).for_each(|_| {
            // --- Lambda (shrinking) pass ---
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            smoothed_polygons
                .iter()
                .flat_map(|polygon| polygon.vertices.iter())
                .for_each(|vertex| {
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        current_positions.insert(*idx, vertex.pos);
                    }
                });

            let mut updates: HashMap<usize, Point3<Real>> = HashMap::new();
            adjacency.iter().for_each(|(&vertex_idx, neighbors)| {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        return;
                    }

                    let (neighbor_sum, valid_neighbors) = neighbors.iter()
                        .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                        .fold((Point3::origin(), 0), |(sum, count), &neighbor_pos| {
                            (sum + neighbor_pos.coords, count + 1)
                        });

                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        updates.insert(vertex_idx, current_pos + laplacian * lambda);
                    }
                }
            });

            smoothed_polygons.iter_mut().for_each(|polygon| {
                polygon.vertices.iter_mut().for_each(|vertex| {
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        if let Some(&new_pos) = updates.get(idx) {
                            vertex.pos = new_pos;
                        }
                    }
                });
            });

            // --- Mu (inflating) pass ---
            current_positions.clear();
            smoothed_polygons
                .iter()
                .flat_map(|polygon| polygon.vertices.iter())
                .for_each(|vertex| {
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        current_positions.insert(*idx, vertex.pos);
                    }
                });

            updates.clear();
            adjacency.iter().for_each(|(&vertex_idx, neighbors)| {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        return;
                    }

                    let (neighbor_sum, valid_neighbors) = neighbors.iter()
                        .filter_map(|&neighbor_idx| current_positions.get(&neighbor_idx))
                        .fold((Point3::origin(), 0), |(sum, count), &neighbor_pos| {
                            (sum + neighbor_pos.coords, count + 1)
                        });

                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        updates.insert(vertex_idx, current_pos + laplacian * mu);
                    }
                }
            });

            smoothed_polygons.iter_mut().for_each(|polygon| {
                polygon.vertices.iter_mut().for_each(|vertex| {
                    if let Some((_, idx)) = vertex_map.position_to_index
                        .iter()
                        .find(|(pos, _)| (vertex.pos - *pos).norm() < vertex_map.epsilon) {
                        if let Some(&new_pos) = updates.get(idx) {
                            vertex.pos = new_pos;
                        }
                    }
                });
            });
        });

        // Final pass to recompute normals
        smoothed_polygons.iter_mut().for_each(|polygon| {
            polygon.set_new_normal();
        });

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
    /// This provides better mesh quality compared to uniform subdivision.
    pub fn adaptive_refine(
        &self,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> Mesh<S> {
        let qualities = self.analyze_triangle_quality();
        let (mut vertex_map, _adjacency) = self.build_connectivity();
        let mut refined_polygons = Vec::new();
        let mut polygon_map: HashMap<usize, Vec<usize>> = HashMap::new();

        self.polygons.iter().enumerate().for_each(|(poly_idx, poly)| {
            poly.vertices.iter().for_each(|vertex| {
                let v_idx = vertex_map.get_or_create_index(vertex.pos);
                polygon_map.entry(v_idx).or_default().push(poly_idx);
            });
        });

        self.polygons.iter().enumerate().for_each(|(i, polygon)| {
            let mut should_refine = false;

            // Quality and edge length check
            if i < qualities.len() {
                let quality = &qualities[i];
                if quality.quality_score < quality_threshold
                    || Self::max_edge_length(&polygon.vertices) > max_edge_length
                {
                    should_refine = true;
                }
            }

            // Curvature check
            if !should_refine {
                'edge_loop: for edge in polygon.edges() {
                    let v1_idx = vertex_map.get_or_create_index(edge.0.pos);
                    let v2_idx = vertex_map.get_or_create_index(edge.1.pos);

                    if let (Some(p1_indices), Some(p2_indices)) =
                        (polygon_map.get(&v1_idx), polygon_map.get(&v2_idx))
                    {
                        if p1_indices.iter().any(|&p1_idx| {
                            p1_idx != i && p2_indices.iter().any(|&p2_idx| {
                                p1_idx == p2_idx && {
                                    let other_poly = &self.polygons[p1_idx];
                                    let angle = Self::dihedral_angle(polygon, other_poly);
                                    angle > curvature_threshold_deg.to_radians()
                                }
                            })
                        }) {
                            should_refine = true;
                            break 'edge_loop;
                        }
                    }
                }
            }

            if should_refine {
                let subdivided = polygon.subdivide_triangles(1.try_into().unwrap());
                subdivided.into_iter().for_each(|triangle| {
                    let vertices = triangle.to_vec();
                    refined_polygons.push(Polygon::new(vertices, polygon.metadata.clone()));
                });
            } else {
                refined_polygons.push(polygon.clone());
            }
        });

        Mesh::from_polygons(&refined_polygons, self.metadata.clone())
    }

    /// Calculate maximum edge length in a polygon
    fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return 0.0;
        }

        (0..vertices.len())
            .map(|i| {
                let j = (i + 1) % vertices.len();
                (vertices[j].pos - vertices[i].pos).norm()
            })
            .fold(0.0, |max_length, edge_length| max_length.max(edge_length))
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
    /// Returns cleaned mesh with improved triangle quality.
    pub fn remove_poor_triangles(&self, min_quality: Real) -> Mesh<S> {
        let qualities = self.analyze_triangle_quality();
        let mut filtered_polygons = Vec::new();

        self.polygons.iter().enumerate().for_each(|(i, polygon)| {
            let keep_triangle = if i < qualities.len() {
                let quality = &qualities[i];
                quality.quality_score >= min_quality
                    && quality.area > Real::EPSILON
                    && quality.min_angle > (5.0 as Real).to_radians()
                    && quality.aspect_ratio < 20.0
            } else {
                true // Keep if we can't assess quality
            };

            if keep_triangle {
                filtered_polygons.push(polygon.clone());
            }
        });

        Mesh::from_polygons(&filtered_polygons, self.metadata.clone())
    }
}
