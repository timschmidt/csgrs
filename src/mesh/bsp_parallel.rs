//! Parallel versions of [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) operations

use crate::mesh::bsp::Node;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "parallel")]
use crate::mesh::Polygon;

#[cfg(feature = "parallel")]
use crate::mesh::Vertex;

#[cfg(feature = "parallel")]
use crate::float_types::EPSILON;

impl<S: Clone + Send + Sync + Debug> Node<S> {
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
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane, just return the original set
        if self.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = self.plane.as_ref().unwrap();

        // Split each polygon in parallel; gather results
        let (coplanar_front, coplanar_back, mut front, mut back) = polygons
            .par_iter()
            .map(|poly| plane.split_polygon(poly)) // <-- just pass poly
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
    pub fn clip_to(&mut self, bsp: &Node<S>) {
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
    #[cfg(feature = "parallel")]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons));
        }
        let plane = self.plane.as_ref().unwrap();

        // Split polygons in parallel
        let (mut coplanar_front, mut coplanar_back, front, back) =
            polygons.par_iter().map(|p| plane.split_polygon(p)).reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Append coplanar fronts/backs to self.polygons
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);

        // Build children sequentially to avoid stack overflow from recursive join
        // The polygon splitting above already uses parallel iterators for the heavy work
        if !front.is_empty() {
            let mut front_node = self.front.take().unwrap_or_else(|| Box::new(Node::new()));
            front_node.build(&front);
            self.front = Some(front_node);
        }

        if !back.is_empty() {
            let mut back_node = self.back.take().unwrap_or_else(|| Box::new(Node::new()));
            back_node.build(&back);
            self.back = Some(back_node);
        }
    }

    // Parallel slice
    #[cfg(feature = "parallel")]
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        // Collect all polygons (this can be expensive, but let's do it).
        let all_polys = self.all_polygons();

        // Process polygons in parallel
        let (coplanar_polygons, intersection_edges) = all_polys
            .par_iter()
            .map(|poly| {
                let vcount = poly.vertices.len();
                if vcount < 2 {
                    // Degenerate => skip
                    return (Vec::new(), Vec::new());
                }
                let mut polygon_type = 0;
                let mut types = Vec::with_capacity(vcount);

                for vertex in &poly.vertices {
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
                            let vi = &poly.vertices[i];
                            let vj = &poly.vertices[j];

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
