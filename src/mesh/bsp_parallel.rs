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

        // Take the sub-trees out of `self` so that we can move them.
        let (front_opt, back_opt) = (self.front.take(), self.back.take());

        // run the two inversions in parallel
        let (front_done, back_done) = rayon::join(
            || {
                front_opt.map(|mut n| {
                    n.invert();
                    n
                })
            },
            || {
                back_opt.map(|mut n| {
                    n.invert();
                    n
                })
            },
        );

        // and swap them afterwards
        self.front = back_done;
        self.back = front_done;
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
        coplanar_front.into_iter().for_each(|cp| {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        });
        coplanar_back.into_iter().for_each(|cp| {
            if plane.orient_plane(&cp.plane) == FRONT {
                front.push(cp);
            } else {
                back.push(cp);
            }
        });

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
        let polygons = std::mem::take(&mut self.polygons);
        let front_child = self.front.take();
        let back_child = self.back.take();

        // parallel work – *no* mutation of `self` inside
        let (clipped_polys, (new_front, new_back)) = rayon::join(
            || bsp.clip_polygons(&polygons),
            || {
                rayon::join(
                    || {
                        front_child.map(|mut n| {
                            n.clip_to(bsp);
                            n
                        })
                    },
                    || {
                        back_child.map(|mut n| {
                            n.clip_to(bsp);
                            n
                        })
                    },
                )
            },
        );

        // single-thread commit
        self.polygons = clipped_polys;
        self.front = new_front;
        self.back = new_back;
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

        // take ownership of the existing children (if any)
        let front_opt = self.front.take();
        let back_opt = self.back.take();

        let (new_front, new_back) = rayon::join(
            // front branch
            || {
                if front.is_empty() {
                    None
                } else {
                    // reuse the node if it was already there
                    let mut node = front_opt.unwrap_or_else(|| Box::new(Node::new()));
                    node.build(&front);
                    Some(node)
                }
            },
            // back branch
            || {
                if back.is_empty() {
                    None
                } else {
                    let mut node = back_opt.unwrap_or_else(|| Box::new(Node::new()));
                    node.build(&back);
                    Some(node)
                }
            },
        );

        self.front = new_front;
        self.back = new_back;
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
                let mut types = Vec::with_capacity(vcount);

                let polygon_type = poly.vertices.iter().fold(0, |acc, vertex| {
                    let vertex_type = slicing_plane.orient_point(&vertex.pos);
                    types.push(vertex_type);
                    acc | vertex_type
                });

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
                        let crossing_points: Vec<_> = (0..vcount)
                            .filter_map(|i| {
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
                                        Some(intersect_vert)
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            })
                            .collect();

                        // Pair up intersection points => edges
                        let edges = crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()])
                            .collect();
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
