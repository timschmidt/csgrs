//! [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node structure and operations

use crate::float_types::{Real, EPSILON};
use crate::IndexedMesh::IndexedPolygon;
use crate::IndexedMesh::plane::{Plane, FRONT, BACK, COPLANAR, SPANNING};
use crate::IndexedMesh::vertex::IndexedVertex;
use std::fmt::Debug;
use std::collections::HashMap;

/// A [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct IndexedNode<S: Clone> {
    /// Splitting plane for this node *or* **None** for a leaf that
    /// only stores polygons.
    pub plane: Option<Plane>,

    /// Polygons in *front* half‑spaces.
    pub front: Option<Box<IndexedNode<S>>>,

    /// Polygons in *back* half‑spaces.
    pub back: Option<Box<IndexedNode<S>>>,

    /// Polygons that lie *exactly* on `plane`
    /// (after the node has been built).
    pub polygons: Vec<IndexedPolygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Default for IndexedNode<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    pub const fn new() -> Self {
        Self {
            plane: None,
            polygons: Vec::new(),
            front: None,
            back: None,
        }
    }

    /// Creates a new BSP node from polygons
    /// Builds BSP tree immediately for consistency with Mesh implementation
    pub fn from_polygons(polygons: &[IndexedPolygon<S>], vertices: &mut Vec<IndexedVertex>) -> Self {
        let mut node = Self::new();
        if !polygons.is_empty() {
            node.build(polygons, vertices);
        }
        node
    }





    /// Pick the best splitting plane from a set of polygons using a heuristic
    pub fn pick_best_splitting_plane(&self, polygons: &[IndexedPolygon<S>], vertices: &[IndexedVertex]) -> Plane {
        const K_SPANS: Real = 8.0; // Weight for spanning polygons
        const K_BALANCE: Real = 1.0; // Weight for front/back balance

        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Take a sample of polygons as candidate planes
        let sample_size = polygons.len().min(20);
        for p in polygons.iter().take(sample_size) {
            let plane = &p.plane;
            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            for poly in polygons {
                match plane.classify_polygon(poly, vertices) {
                    COPLANAR => {}, // Not counted for balance
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING => num_spanning += 1,
                    _ => num_spanning += 1, // Treat any other combination as spanning
                }
            }

            let score = K_SPANS * num_spanning as Real
                + K_BALANCE * ((num_front - num_back) as Real).abs();

            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        }
        best_plane
    }

    /// Return all polygons in this BSP tree
    pub fn all_polygons(&self) -> Vec<IndexedPolygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Add child nodes to stack
            if let Some(ref front) = node.front {
                stack.push(front.as_ref());
            }
            if let Some(ref back) = node.back {
                stack.push(back.as_ref());
            }
        }

        result
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    /// **Mathematical Foundation**: Uses plane classification to determine polygon visibility.
    /// Polygons entirely in BACK half-space are clipped (removed).
    /// **Algorithm**: O(n log d) where n is polygon count, d is tree depth.
    pub fn clip_polygons(&self, polygons: &[IndexedPolygon<S>], vertices: &mut Vec<IndexedVertex>) -> Vec<IndexedPolygon<S>> {
        // If this node has no plane, just return the original set
        if self.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        // Process each polygon individually (like regular Mesh)
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());

        // Ensure consistent edge splits across all polygons for this plane
        let mut edge_cache: HashMap<(usize, usize), usize> = HashMap::new();

        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_indexed_polygon_with_cache(polygon, vertices, &mut edge_cache);

            // Handle coplanar polygons like regular Mesh
            for cp in coplanar_front.into_iter().chain(coplanar_back.into_iter()) {
                if plane.orient_plane(&cp.plane) == FRONT {
                    front_parts.push(cp);
                } else {
                    back_parts.push(cp);
                }
            }

            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }

        // Process front and back recursively
        let mut result = if let Some(ref f) = self.front {
            f.clip_polygons(&front_polys, vertices)
        } else {
            front_polys
        };

        if let Some(ref b) = self.back {
            result.extend(b.clip_polygons(&back_polys, vertices));
        }

        result
    }

    /// Clip this BSP tree to another BSP tree
    pub fn clip_to(&mut self, bsp: &IndexedNode<S>, vertices: &mut Vec<IndexedVertex>) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Clip polygons at this node
            node.polygons = bsp.clip_polygons(&node.polygons, vertices);

            // Add children to stack for processing
            if let Some(ref mut front) = node.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = node.back {
                stack.push(back.as_mut());
            }
        }
    }

    /// Invert all polygons in the BSP tree
    pub fn invert(&mut self) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Flip all polygons and plane in this node
            for p in &mut node.polygons {
                p.flip();
            }
            if let Some(ref mut plane) = node.plane {
                plane.flip();
            }

            // Swap front and back children
            std::mem::swap(&mut node.front, &mut node.back);

            // Add children to stack for processing
            if let Some(ref mut front) = node.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = node.back {
                stack.push(back.as_mut());
            }
        }
    }

    /// Invert all polygons in the BSP tree and flip vertex normals
    pub fn invert_with_vertices(&mut self, vertices: &mut Vec<IndexedVertex>) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Flip all polygons and their vertex normals in this node
            for p in &mut node.polygons {
                p.flip_with_vertices(vertices);
            }
            if let Some(ref mut plane) = node.plane {
                plane.flip();
            }

            // Swap front and back children
            std::mem::swap(&mut node.front, &mut node.back);

            // Add children to stack for processing
            if let Some(ref mut front) = node.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = node.back {
                stack.push(back.as_mut());
            }
        }
    }

    /// Build BSP tree from polygons (matches regular Mesh implementation)
    pub fn build(&mut self, polygons: &[IndexedPolygon<S>], vertices: &mut Vec<IndexedVertex>) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons, vertices));
        }
        let plane = self.plane.as_ref().unwrap();

        // Split polygons using a shared edge split cache for this plane
        let mut coplanar_front: Vec<IndexedPolygon<S>> = Vec::new();
        let mut coplanar_back: Vec<IndexedPolygon<S>> = Vec::new();
        let mut front: Vec<IndexedPolygon<S>> = Vec::new();
        let mut back: Vec<IndexedPolygon<S>> = Vec::new();
        let mut edge_cache: HashMap<(usize, usize), usize> = HashMap::new();
        for p in polygons {
            let (cf, cb, mut fr, mut bk) = plane.split_indexed_polygon_with_cache(p, vertices, &mut edge_cache);
            coplanar_front.extend(cf);
            coplanar_back.extend(cb);
            front.append(&mut fr);
            back.append(&mut bk);
        }

        // Append coplanar fronts/backs to self.polygons
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);

        // Build child nodes using lazy initialization pattern for memory efficiency
        if !front.is_empty() {
            self.front
                .get_or_insert_with(|| Box::new(IndexedNode::new()))
                .build(&front, vertices);
        }

        if !back.is_empty() {
            self.back
                .get_or_insert_with(|| Box::new(IndexedNode::new()))
                .build(&back, vertices);
        }
    }

    /// Slices this BSP node with `slicing_plane`, returning:
    /// - All polygons that are coplanar with the plane (within EPSILON),
    /// - A list of line‐segment intersections (each a [IndexedVertex; 2]) from polygons that span the plane.
    /// Note: This method requires access to the mesh vertices to resolve indices
    pub fn slice(&self, slicing_plane: &Plane, vertices: &[IndexedVertex]) -> (Vec<IndexedPolygon<S>>, Vec<[IndexedVertex; 2]>) {
        let all_polys = self.all_polygons();

        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();

        for poly in &all_polys {
            let vcount = poly.indices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }

            // Use iterator chain to compute vertex types more efficiently
            let types: Vec<i8> = poly
                .indices
                .iter()
                .map(|&idx| slicing_plane.orient_point(&vertices[idx].pos))
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
                    let crossing_points: Vec<_> = (0..poly.indices.len())
                        .filter_map(|i| {
                            let j = (i + 1) % poly.indices.len();
                            let ti = types[i];
                            let tj = types[j];
                            let vi = &vertices[poly.indices[i]];
                            let vj = &vertices[poly.indices[j]];

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
                            .map(|chunk| [chunk[0], chunk[1]]),
                    );
                },

                _ => {
                    // Shouldn't happen in a typical classification, but we can ignore
                },
            }
        }

        (coplanar_polygons, intersection_edges)
    }
}

#[cfg(test)]
mod tests {
    use crate::IndexedMesh::bsp::IndexedNode;
    use crate::IndexedMesh::IndexedPolygon;
    use nalgebra::Vector3;

    #[test]
    fn test_indexed_bsp_basic_functionality() {
        use crate::IndexedMesh::vertex::IndexedVertex;
        use nalgebra::Point3;

        // Create vertices first
        let mut vertices = vec![
            IndexedVertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            IndexedVertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            IndexedVertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
        ];

        let indices = vec![0, 1, 2];
        let plane = crate::IndexedMesh::plane::Plane::from_normal(Vector3::z(), 0.0);
        let polygon: IndexedPolygon<i32> = IndexedPolygon::new(indices, plane, None);
        let polygons = vec![polygon];

        let node = IndexedNode::from_polygons(polygons.as_slice(), &mut vertices);
        assert!(!node.all_polygons().is_empty());
    }
}
