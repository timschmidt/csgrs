//! Parallel versions of [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) operations

use crate::mesh::bsp::Node;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};

#[cfg(feature = "parallel")]
use rayon::{join, prelude::*};

#[cfg(feature = "parallel")]
use crate::mesh::Polygon;

#[cfg(feature = "parallel")]
use crate::mesh::Vertex;

#[cfg(feature = "parallel")]
use crate::float_types::EPSILON;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Invert all polygons in the BSP tree
    #[cfg(feature = "parallel")]
    pub fn invert(&mut self) {
        // Flip all polygons and plane in this node
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        // Recursively invert children in parallel, if both exist
        match (&mut self.front, &mut self.back) {
            (Some(front_node), Some(back_node)) => {
                join(|| front_node.invert(), || back_node.invert());
            },
            (Some(front_node), None) => front_node.invert(),
            (None, Some(back_node)) => back_node.invert(),
            (None, None) => {},
        }

        std::mem::swap(&mut self.front, &mut self.back);
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

        // Recursively clip front & back in parallel
        let (front_clipped, back_clipped) = join(
            || {
                if let Some(ref f) = self.front {
                    f.clip_polygons(&front)
                } else {
                    front
                }
            },
            || {
                if let Some(ref b) = self.back {
                    b.clip_polygons(&back)
                } else {
                    // If there's no back node, discard these polygons
                    Vec::new()
                }
            },
        );

        // Combine front and back
        let mut result = front_clipped;
        result.extend(back_clipped);
        result
    }

    /// Parallel version of `clip_to`
    #[cfg(feature = "parallel")]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        // The clipping of polygons can be done in parallel for different nodes.
        let (polygons, front_opt, back_opt) = (
            std::mem::take(&mut self.polygons),
            self.front.take(),
            self.back.take(),
        );

        let (clipped_polygons, (clipped_front, clipped_back)) = rayon::join(
            || bsp.clip_polygons(&polygons),
            || {
                rayon::join(
                    || {
                        if let Some(mut front) = front_opt {
                            front.clip_to(bsp);
                            Some(front)
                        } else {
                            None
                        }
                    },
                    || {
                        if let Some(mut back) = back_opt {
                            back.clip_to(bsp);
                            Some(back)
                        } else {
                            None
                        }
                    },
                )
            },
        );

        self.polygons = clipped_polygons;
        self.front = clipped_front;
        self.back = clipped_back;
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
        let (mut coplanar_front, mut coplanar_back, front, back) = polygons
            .par_iter()
            .map(|p| plane.split_polygon(p)) // <-- just pass p
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

        // Append coplanar fronts/backs to self.polygons
        self.polygons.append(&mut coplanar_front);
        self.polygons.append(&mut coplanar_back);

        // Parallelize the recursive building of child nodes
        rayon::join(
            || {
                if !front.is_empty() {
                    let mut front_node = Box::new(Node::new());
                    front_node.build(&front);
                    self.front = Some(front_node);
                }
            },
            || {
                if !back.is_empty() {
                    let mut back_node = Box::new(Node::new());
                    back_node.build(&back);
                    self.back = Some(back_node);
                }
            },
        );
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
                            edges.push([chunk[0].clone(), chunk[1].clone()]);
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
