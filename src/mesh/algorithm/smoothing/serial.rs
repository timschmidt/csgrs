//! Serial implementations of mesh smoothing operations.

use super::traits::SmoothingOps;
use crate::float_types::Real;
use crate::mesh::{Mesh, polygon::Polygon, vertex::Vertex};
use nalgebra::Point3;
use std::collections::HashMap;
use std::fmt::Debug;

/// Serial implementation of `SmoothingOps`.
pub struct SerialSmoothingOps;

impl Default for SerialSmoothingOps {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialSmoothingOps {
    pub const fn new() -> Self {
        Self
    }
}

impl<S: Clone + Debug + Send + Sync> SmoothingOps<S> for SerialSmoothingOps {
    fn laplacian_smooth(
        &self,
        mesh: &Mesh<S>,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<S> {
        let (vertex_map, adjacency) = mesh.build_connectivity();
        let mut smoothed_polygons = mesh.polygons.clone();

        for iteration in 0..iterations {
            // Build current vertex position mapping
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    // Find the global index for this position (with tolerance)
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }

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
            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    // Find the global index for this vertex
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = laplacian_updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
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

        Mesh::from_polygons(&smoothed_polygons, mesh.metadata.clone())
    }

    fn taubin_smooth(
        &self,
        mesh: &Mesh<S>,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Mesh<S> {
        let (vertex_map, adjacency) = mesh.build_connectivity();
        let mut smoothed_polygons = mesh.polygons.clone();

        for _ in 0..iterations {
            // --- Lambda (shrinking) pass ---
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }

            let mut updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        continue;
                    }

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
                        updates.insert(vertex_idx, current_pos + laplacian * lambda);
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
                        }
                    }
                }
            }

            // --- Mu (inflating) pass ---
            current_positions.clear();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }

            updates.clear();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    if preserve_boundaries && neighbors.len() < 4 {
                        updates.insert(vertex_idx, current_pos);
                        continue;
                    }

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
                        updates.insert(vertex_idx, current_pos + laplacian * mu);
                    }
                }
            }

            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
                        }
                    }
                }
            }
        }

        // Final pass to recompute normals
        for polygon in &mut smoothed_polygons {
            polygon.set_new_normal();
        }

        Mesh::from_polygons(&smoothed_polygons, mesh.metadata.clone())
    }

    fn adaptive_refine(
        &self,
        mesh: &Mesh<S>,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> Mesh<S> {
        let qualities = mesh.analyze_triangle_quality();
        let (mut vertex_map, _adjacency) = mesh.build_connectivity();
        let mut refined_polygons = Vec::new();
        let mut polygon_map: HashMap<usize, Vec<usize>> = HashMap::new();

        for (poly_idx, poly) in mesh.polygons.iter().enumerate() {
            for vertex in &poly.vertices {
                let v_idx = vertex_map.get_or_create_index(vertex.pos);
                polygon_map.entry(v_idx).or_default().push(poly_idx);
            }
        }

        for (i, polygon) in mesh.polygons.iter().enumerate() {
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
                        for &p1_idx in p1_indices {
                            if p1_idx == i {
                                continue;
                            }
                            for &p2_idx in p2_indices {
                                if p1_idx == p2_idx {
                                    let other_poly = &mesh.polygons[p1_idx];
                                    let angle = Mesh::dihedral_angle(polygon, other_poly);
                                    if angle > curvature_threshold_deg.to_radians() {
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

        Mesh::from_polygons(&refined_polygons, mesh.metadata.clone())
    }

    fn remove_poor_triangles(&self, mesh: &Mesh<S>, min_quality: Real) -> Mesh<S> {
        let qualities = mesh.analyze_triangle_quality();
        let mut filtered_polygons = Vec::new();

        for (i, polygon) in mesh.polygons.iter().enumerate() {
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
        }

        Mesh::from_polygons(&filtered_polygons, mesh.metadata.clone())
    }
}

impl SerialSmoothingOps {
    /// Calculate maximum edge length in a polygon
    pub fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return 0.0;
        }

        let mut max_length: Real = 0.0;
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let edge_length = (vertices[j].pos - vertices[i].pos).norm();
            max_length = max_length.max(edge_length);
        }
        max_length
    }
}
