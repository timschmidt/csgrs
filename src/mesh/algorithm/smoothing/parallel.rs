//! Parallel implementations of mesh smoothing operations.

use super::traits::SmoothingOps;
use crate::float_types::Real;
use crate::mesh::{Mesh, polygon::Polygon, vertex::Vertex};
use nalgebra::{Point3, Vector3};
use rayon::prelude::*;
use std::collections::HashMap;
use std::fmt::Debug;

/// Parallel implementation of `SmoothingOps`.
pub struct ParallelSmoothingOps;

impl ParallelSmoothingOps {
    pub fn new() -> Self {
        Self
    }
}

impl<S: Clone + Debug + Send + Sync> SmoothingOps<S> for ParallelSmoothingOps {
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
            // Build current vertex position mapping in parallel
            let current_positions: HashMap<usize, Point3<Real>> = smoothed_polygons
                .par_iter()
                .flat_map(|polygon| {
                    polygon.vertices.par_iter().filter_map(|vertex| {
                        for (pos, idx) in &vertex_map.position_to_index {
                            if (vertex.pos - pos).norm() < vertex_map.epsilon {
                                return Some((*idx, vertex.pos));
                            }
                        }
                        None
                    })
                })
                .collect();

            // Compute Laplacian for each vertex in parallel
            let laplacian_updates: HashMap<usize, Point3<Real>> = adjacency
                .par_iter()
                .map(|(&vertex_idx, neighbors)| {
                    if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                        // Check if this is a boundary vertex
                        if preserve_boundaries && neighbors.len() < 4 {
                            // Boundary vertex - skip smoothing
                            return (vertex_idx, current_pos);
                        }

                        // Compute neighbor average
                        let (neighbor_sum, valid_neighbors) = neighbors
                            .par_iter()
                            .filter_map(|&neighbor_idx| {
                                current_positions.get(&neighbor_idx).map(|&p| (p.coords, 1))
                            })
                            .reduce(
                                || (Vector3::zeros(), 0),
                                |(sum_a, count_a), (coords_b, count_b)| {
                                    (sum_a + coords_b, count_a + count_b)
                                },
                            );

                        if valid_neighbors > 0 {
                            let neighbor_avg =
                                Point3::from(neighbor_sum / valid_neighbors as Real);
                            let laplacian = neighbor_avg - current_pos;
                            let new_pos = current_pos + laplacian * lambda;
                            (vertex_idx, new_pos)
                        } else {
                            (vertex_idx, current_pos)
                        }
                    } else {
                        (vertex_idx, Point3::origin()) // Should not happen if connectivity is correct
                    }
                })
                .collect();

            // Apply updates to mesh vertices in parallel
            smoothed_polygons.par_iter_mut().for_each(|polygon| {
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
            });

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
            let current_positions: HashMap<usize, Point3<Real>> = smoothed_polygons
                .par_iter()
                .flat_map(|polygon| {
                    polygon.vertices.par_iter().filter_map(|vertex| {
                        for (pos, idx) in &vertex_map.position_to_index {
                            if (vertex.pos - pos).norm() < vertex_map.epsilon {
                                return Some((*idx, vertex.pos));
                            }
                        }
                        None
                    })
                })
                .collect();

            let updates: HashMap<usize, Point3<Real>> = adjacency
                .par_iter()
                .map(|(&vertex_idx, neighbors)| {
                    if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                        if preserve_boundaries && neighbors.len() < 4 {
                            return (vertex_idx, current_pos);
                        }

                        let (neighbor_sum, valid_neighbors) = neighbors
                            .par_iter()
                            .filter_map(|&neighbor_idx| {
                                current_positions.get(&neighbor_idx).map(|&p| (p.coords, 1))
                            })
                            .reduce(
                                || (Vector3::zeros(), 0),
                                |(sum_a, count_a), (coords_b, count_b)| {
                                    (sum_a + coords_b, count_a + count_b)
                                },
                            );

                        if valid_neighbors > 0 {
                            let neighbor_avg =
                                Point3::from(neighbor_sum / valid_neighbors as Real);
                            let laplacian = neighbor_avg - current_pos;
                            (vertex_idx, current_pos + laplacian * lambda)
                        } else {
                            (vertex_idx, current_pos)
                        }
                    } else {
                        (vertex_idx, Point3::origin())
                    }
                })
                .collect();

            smoothed_polygons.par_iter_mut().for_each(|polygon| {
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
            });

            // --- Mu (inflating) pass ---
            let current_positions: HashMap<usize, Point3<Real>> = smoothed_polygons
                .par_iter()
                .flat_map(|polygon| {
                    polygon.vertices.par_iter().filter_map(|vertex| {
                        for (pos, idx) in &vertex_map.position_to_index {
                            if (vertex.pos - pos).norm() < vertex_map.epsilon {
                                return Some((*idx, vertex.pos));
                            }
                        }
                        None
                    })
                })
                .collect();

            let updates: HashMap<usize, Point3<Real>> = adjacency
                .par_iter()
                .map(|(&vertex_idx, neighbors)| {
                    if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                        if preserve_boundaries && neighbors.len() < 4 {
                            return (vertex_idx, current_pos);
                        }

                        let (neighbor_sum, valid_neighbors) = neighbors
                            .par_iter()
                            .filter_map(|&neighbor_idx| {
                                current_positions.get(&neighbor_idx).map(|&p| (p.coords, 1))
                            })
                            .reduce(
                                || (Vector3::zeros(), 0),
                                |(sum_a, count_a), (coords_b, count_b)| {
                                    (sum_a + coords_b, count_a + count_b)
                                },
                            );

                        if valid_neighbors > 0 {
                            let neighbor_avg =
                                Point3::from(neighbor_sum / valid_neighbors as Real);
                            let laplacian = neighbor_avg - current_pos;
                            (vertex_idx, current_pos + laplacian * mu)
                        } else {
                            (vertex_idx, current_pos)
                        }
                    } else {
                        (vertex_idx, Point3::origin())
                    }
                })
                .collect();

            smoothed_polygons.par_iter_mut().for_each(|polygon| {
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
            });
        }

        // Final pass to recompute normals
        smoothed_polygons.par_iter_mut().for_each(|polygon| {
            polygon.set_new_normal();
        });

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
        let mut polygon_map: HashMap<usize, Vec<usize>> = HashMap::new();

        for (poly_idx, poly) in mesh.polygons.iter().enumerate() {
            for vertex in &poly.vertices {
                let v_idx = vertex_map.get_or_create_index(vertex.pos);
                polygon_map.entry(v_idx).or_default().push(poly_idx);
            }
        }

        // Pre-populate the vertex map with all edge vertices
        let edge_vertices: Vec<Point3<Real>> = mesh
            .polygons
            .par_iter()
            .flat_map(|polygon| {
                polygon
                    .edges()
                    .collect::<Vec<_>>()
                    .into_par_iter()
                    .flat_map(|edge| vec![edge.0.pos, edge.1.pos])
                    .collect::<Vec<_>>()
            })
            .collect();

        for pos in edge_vertices {
            vertex_map.get_or_create_index(pos);
        }

        let refined_polygons: Vec<Polygon<S>> = mesh
            .polygons
            .par_iter()
            .enumerate()
            .flat_map(|(i, polygon)| {
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
                        let v1_idx = vertex_map.get_index(&edge.0.pos).unwrap();
                        let v2_idx = vertex_map.get_index(&edge.1.pos).unwrap();

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
                    subdivided
                        .into_par_iter()
                        .map(move |triangle| {
                            let vertices = triangle.to_vec();
                            Polygon::new(vertices, polygon.metadata.clone())
                        })
                        .collect::<Vec<_>>()
                } else {
                    vec![polygon.clone()]
                }
            })
            .collect();

        Mesh::from_polygons(&refined_polygons, mesh.metadata.clone())
    }

    fn remove_poor_triangles(&self, mesh: &Mesh<S>, min_quality: Real) -> Mesh<S> {
        let qualities = mesh.analyze_triangle_quality();
        let filtered_polygons: Vec<Polygon<S>> = mesh
            .polygons
            .par_iter()
            .enumerate()
            .filter_map(|(i, polygon)| {
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
                    Some(polygon.clone())
                } else {
                    None
                }
            })
            .collect();

        Mesh::from_polygons(&filtered_polygons, mesh.metadata.clone())
    }
}

impl ParallelSmoothingOps {
    /// Calculate maximum edge length in a polygon
    pub fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return 0.0;
        }

        vertices
            .par_iter()
            .enumerate()
            .map(|(i, vertex)| {
                let next_vertex = &vertices[(i + 1) % vertices.len()];
                (next_vertex.pos - vertex.pos).norm()
            })
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0)
    }
}
