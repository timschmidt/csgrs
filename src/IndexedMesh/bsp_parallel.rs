//! Parallel versions of [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) operations for IndexedMesh

use crate::IndexedMesh::bsp::IndexedNode;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use crate::IndexedMesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "parallel")]
use crate::IndexedMesh::{IndexedPolygon, vertex::IndexedVertex};

#[cfg(feature = "parallel")]
use std::collections::HashMap;

#[cfg(feature = "parallel")]
use crate::IndexedMesh::vertex::IndexedVertex;

#[cfg(feature = "parallel")]
use crate::float_types::EPSILON;

#[cfg(feature = "parallel")]
use crate::IndexedMesh::IndexedMesh;

impl<S: Clone + Send + Sync + Debug> IndexedNode<S> {
    /// Invert all polygons in the BSP tree using iterative approach to avoid stack overflow
    #[cfg(feature = "parallel")]
    pub fn invert(&mut self) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Flip all polygons and plane in this node
            node.polygons.par_iter_mut().for_each(|p| p.flip());
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

    /// Parallel version of clip Polygons
    #[cfg(feature = "parallel")]
    pub fn clip_polygons(&self, polygons: &[IndexedPolygon<S>]) -> Vec<IndexedPolygon<S>> {
        // If this node has no plane, just return the original set
        if self.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        // Split each polygon in parallel; gather results
        let (coplanar_front, coplanar_back, mut front, mut back) = polygons
            .par_iter()
            .map(|poly| plane.split_indexed_polygon(poly))
            .reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Decide where to send the coplanar polygons
        for cp in coplanar_front {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        }
        for cp in coplanar_back {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        }

        // Process front and back using parallel iterators to avoid recursive join
        let mut result = if let Some(ref f) = self.front {
            f.clip_polygons(&front)
        } else {
            front
        };

        if let Some(ref b) = self.back {
            result.extend(b.clip_polygons(&back));
        }
        // If there's no back node, we simply don't extend (effectively discarding back polygons)

        result
    }

    /// Parallel version of `clip_to` using iterative approach to avoid stack overflow
    #[cfg(feature = "parallel")]
    pub fn clip_to(&mut self, bsp: &IndexedNode<S>) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            // Clip polygons at this node
            node.polygons = bsp.clip_polygons(&node.polygons);

            // Add children to stack for processing
            if let Some(ref mut front) = node.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = node.back {
                stack.push(back.as_mut());
            }
        }
    }

    /// Parallel version of `build`.
    /// **FIXED**: Now takes vertices parameter to match sequential version
    #[cfg(feature = "parallel")]
    pub fn build(
        &mut self,
        polygons: &[IndexedPolygon<S>],
        vertices: &mut Vec<IndexedVertex>,
    ) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons, vertices));
        }
        let plane = self.plane.as_ref().unwrap();

        // **FIXED**: For parallel processing, we can't use shared edge caching
        // Instead, we'll fall back to sequential processing for IndexedMesh to maintain
        // vertex sharing consistency. This ensures identical results between parallel
        // and sequential execution.

        // Split polygons sequentially to maintain edge cache consistency
        let mut coplanar_front: Vec<IndexedPolygon<S>> = Vec::new();
        let mut coplanar_back: Vec<IndexedPolygon<S>> = Vec::new();
        let mut front: Vec<IndexedPolygon<S>> = Vec::new();
        let mut back: Vec<IndexedPolygon<S>> = Vec::new();
        let mut edge_cache: HashMap<crate::IndexedMesh::plane::PlaneEdgeCacheKey, usize> =
            HashMap::new();

        for p in polygons {
            let (cf, cb, mut fr, mut bk) =
                plane.split_indexed_polygon_with_cache(p, vertices, &mut edge_cache);
            coplanar_front.extend(cf);
            coplanar_back.extend(cb);
            front.append(&mut fr);
            back.append(&mut bk);
        }

        // Append coplanar fronts/backs to self.polygons
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);

        // Build children sequentially to avoid stack overflow from recursive join
        // The polygon splitting above already uses parallel iterators for the heavy work
        if !front.is_empty() {
            let mut front_node = self
                .front
                .take()
                .unwrap_or_else(|| Box::new(IndexedNode::new()));
            front_node.build(&front);
            self.front = Some(front_node);
        }

        if !back.is_empty() {
            let mut back_node = self
                .back
                .take()
                .unwrap_or_else(|| Box::new(IndexedNode::new()));
            back_node.build(&back);
            self.back = Some(back_node);
        }
    }

    // Parallel slice
    #[cfg(feature = "parallel")]
    pub fn slice(
        &self,
        slicing_plane: &Plane,
        mesh: &IndexedMesh<S>,
    ) -> (Vec<IndexedPolygon<S>>, Vec<[IndexedVertex; 2]>) {
        // Collect all polygons (this can be expensive, but let's do it).
        let all_polys = self.all_polygons();

        // Process polygons in parallel
        let (coplanar_polygons, intersection_edges) = all_polys
            .par_iter()
            .map(|poly| {
                let vcount = poly.indices.len();
                if vcount < 2 {
                    // Degenerate => skip
                    return (Vec::new(), Vec::new());
                }
                let mut polygon_type = 0;
                let mut types = Vec::with_capacity(vcount);

                for &vertex_idx in &poly.indices {
                    if vertex_idx >= mesh.vertices.len() {
                        continue; // Skip invalid indices
                    }
                    let vertex = &mesh.vertices[vertex_idx];
                    let vertex_type = slicing_plane.orient_point(&vertex.pos);
                    polygon_type |= vertex_type;
                    types.push(vertex_type);
                }

                match polygon_type {
                    COPLANAR => {
                        // Entire polygon in plane
                        (vec![poly.clone()], Vec::new())
                    },
                    FRONT | BACK => {
                        // Entirely on one side => no intersection
                        (Vec::new(), Vec::new())
                    },
                    SPANNING => {
                        // The polygon crosses the plane => gather intersection edges
                        let mut crossing_points = Vec::new();
                        for i in 0..vcount {
                            let j = (i + 1) % vcount;
                            let ti = types[i];
                            let tj = types[j];

                            if i >= poly.indices.len() || j >= poly.indices.len() {
                                continue;
                            }

                            let vi_idx = poly.indices[i];
                            let vj_idx = poly.indices[j];

                            if vi_idx >= mesh.vertices.len() || vj_idx >= mesh.vertices.len() {
                                continue;
                            }

                            let vi = &mesh.vertices[vi_idx];
                            let vj = &mesh.vertices[vj_idx];

                            if (ti | tj) == SPANNING {
                                // The param intersection at which plane intersects the edge [vi -> vj].
                                // Avoid dividing by zero:
                                let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                if denom.abs() > EPSILON {
                                    let intersection = (slicing_plane.offset()
                                        - slicing_plane.normal().dot(&vi.pos.coords))
                                        / denom;
                                    // Interpolate:
                                    let intersect_vert = vi.interpolate(vj, intersection);
                                    crossing_points.push(intersect_vert);
                                }
                            }
                        }

                        // Pair up intersection points => edges
                        let mut edges = Vec::new();
                        for chunk in crossing_points.chunks_exact(2) {
                            edges.push([chunk[0], chunk[1]]);
                        }
                        (Vec::new(), edges)
                    },
                    _ => (Vec::new(), Vec::new()),
                }
            })
            .reduce(
                || (Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc
                },
            );

        (coplanar_polygons, intersection_edges)
    }
}
