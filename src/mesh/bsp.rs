//! [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node structure and operations

#[cfg(not(feature = "parallel"))]
use crate::float_types::EPSILON;

#[cfg(not(feature = "parallel"))]
use crate::mesh::vertex::Vertex;

use crate::float_types::Real;
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::mesh::polygon::Polygon;
use std::fmt::Debug;

/// A [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Splitting plane for this node *or* **None** for a leaf that
    /// only stores polygons.
    pub plane: Option<Plane>,

    /// Polygons in *front* half‑spaces.
    pub front: Option<Box<Node<S>>>,

    /// Polygons in *back* half‑spaces.
    pub back: Option<Box<Node<S>>>,

    /// Polygons that lie *exactly* on `plane`
    /// (after the node has been built).
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    #[allow(unused)]
    const fn default() -> Self {
        Self::new()
    }

    /// Create a new empty BSP node
    pub const fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        }
    }

    /// Creates a new BSP node from polygons
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() {
            node.build(polygons);
        }
        node
    }

    /// Invert all polygons in the BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn invert(&mut self) {
        // Flip all polygons and plane in this node
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        if let Some(ref mut front) = self.front {
            front.invert();
        }
        if let Some(ref mut back) = self.back {
            back.invert();
        }

        std::mem::swap(&mut self.front, &mut self.back);
    }

    pub fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        const K_SPANS: Real = 8.0; // Weight for spanning polygons
        const K_BALANCE: Real = 1.0; // Weight for front/back balance

        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Take a sample of polygons as candidate planes
        let sample_size = polygons.len().min(20);
        polygons.iter().take(sample_size).for_each(|p| {
            let plane = &p.plane;
            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            polygons.iter().for_each(|poly| {
                match plane.classify_polygon(poly) {
                    COPLANAR => {}, // Not counted for balance
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING => num_spanning += 1,
                    _ => num_spanning += 1, // Treat any other combination as spanning
                }
            });

            let score = K_SPANS * num_spanning as Real
                + K_BALANCE * ((num_front - num_back) as Real).abs();

            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        });
        best_plane
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    /// **Mathematical Foundation**: Uses plane classification to determine polygon visibility.
    /// Polygons entirely in BACK half-space are clipped (removed).
    /// **Algorithm**: O(n log d) where n is polygon count, d is tree depth.
    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![(self, polygons.to_vec())];

        while let Some((node, polys)) = stack.pop() {
            if node.plane.is_none() {
                result.extend(polys);
                continue;
            }
            let plane = node.plane.as_ref().unwrap();

            let mut front_polys = Vec::with_capacity(polys.len());
            let mut back_polys = Vec::with_capacity(polys.len());

            polys.iter().for_each(|polygon| {
                let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                    plane.split_polygon(polygon);

                coplanar_front.into_iter().chain(coplanar_back.into_iter()).for_each(|coplanar_poly| {
                    if plane.orient_plane(&coplanar_poly.plane) == FRONT {
                        front_parts.push(coplanar_poly);
                    } else {
                        back_parts.push(coplanar_poly);
                    }
                });

                front_polys.append(&mut front_parts);
                back_polys.append(&mut back_parts);
            });

            if let Some(front_node) = &node.front {
                if !front_polys.is_empty() {
                    stack.push((front_node, front_polys));
                }
            } else {
                result.extend(front_polys);
            }

            if let Some(back_node) = &node.back {
                if !back_polys.is_empty() {
                    stack.push((back_node, back_polys));
                }
            }
        }
        result
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        let mut stack = vec![self];
        while let Some(node) = stack.pop() {
            node.polygons = bsp.clip_polygons(&node.polygons);
            if let Some(front) = node.front.as_mut() {
                stack.push(front);
            }
            if let Some(back) = node.back.as_mut() {
                stack.push(back);
            }
        }
    }

    /// Return all polygons in this BSP tree using an iterative approach,
    /// avoiding potential stack overflow of recursive approach
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Use iterator to add child nodes more efficiently
            stack.extend(
                [&node.front, &node.back]
                    .iter()
                    .filter_map(|child| child.as_ref().map(|boxed| boxed.as_ref())),
            );
        }
        result
    }

    /// Build a BSP tree from the given polygons
    #[cfg(not(feature = "parallel"))]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        let mut stack = vec![(self, polygons.to_vec())];

        while let Some((node, polys)) = stack.pop() {
            if polys.is_empty() {
                continue;
            }

            if node.plane.is_none() {
                node.plane = Some(node.pick_best_splitting_plane(&polys));
            }
            let plane = node.plane.as_ref().unwrap();

            let mut front = Vec::with_capacity(polys.len() / 2);
            let mut back = Vec::with_capacity(polys.len() / 2);

            polys.iter().for_each(|polygon| {
                let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                    plane.split_polygon(polygon);

                node.polygons.extend(coplanar_front);
                node.polygons.extend(coplanar_back);
                front.append(&mut front_parts);
                back.append(&mut back_parts);
            });

            if !front.is_empty() {
                let front_node = node.front.get_or_insert_with(|| Box::new(Node::new()));
                stack.push((front_node, front));
            }

            if !back.is_empty() {
                let back_node = node.back.get_or_insert_with(|| Box::new(Node::new()));
                stack.push((back_node, back));
            }
        }
    }

    /// Slices this BSP node with `slicing_plane`, returning:
    /// - All polygons that are coplanar with the plane (within EPSILON),
    /// - A list of line‐segment intersections (each a [Vertex; 2]) from polygons that span the plane.
    #[cfg(not(feature = "parallel"))]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons();

        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();

        all_polys.iter().for_each(|poly| {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                return; // degenerate polygon => skip
            }

            // Use iterator chain to compute vertex types more efficiently
            let types: Vec<_> = poly
                .vertices
                .iter()
                .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                .collect();

            let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

            // Based on the combined classification of its vertices:
            match polygon_type {
                COPLANAR => {
                    // The entire polygon is in the plane, so push it to the coplanar list.
                    coplanar_polygons.push(poly.clone());
                },

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                },

                SPANNING => {
                    // The polygon crosses the plane. We'll gather the intersection points
                    // (the new vertices introduced on edges that cross the plane).
                    let crossing_points: Vec<_> = (0..vcount)
                        .filter_map(|i| {
                            let j = (i + 1) % vcount;
                            let ti = types[i];
                            let tj = types[j];
                            let vi = &poly.vertices[i];
                            let vj = &poly.vertices[j];

                            if (ti | tj) == SPANNING {
                                let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                if denom.abs() > EPSILON {
                                    let intersection = (slicing_plane.offset()
                                        - slicing_plane.normal().dot(&vi.pos.coords))
                                        / denom;
                                    Some(vi.interpolate(vj, intersection))
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        })
                        .collect();

                    // Convert crossing points to intersection edges
                    intersection_edges.extend(
                        crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()]),
                    );
                },

                _ => {
                    // Shouldn't happen in a typical classification, but we can ignore
                },
            }
        });

        (coplanar_polygons, intersection_edges)
    }
}

#[cfg(test)]
mod tests {
    use crate::mesh::bsp::Node;
    use crate::mesh::polygon::Polygon;
    use crate::mesh::vertex::Vertex;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_bsp_basic_functionality() {
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        let polygon: Polygon<i32> = Polygon::new(vertices, None);
        let polygons = vec![polygon];

        let node = Node::from_polygons(&polygons);
        assert!(!node.all_polygons().is_empty());
    }
}
