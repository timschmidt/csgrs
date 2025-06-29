//! [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node structure and operations

use crate::float_types::{EPSILON, Real};
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
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
    pub fn new(polygons: &[Polygon<S>]) -> Self {
        let mut node = Node {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        };
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
        for p in polygons.iter().take(sample_size) {
            let plane = &p.plane;
            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            for poly in polygons {
                match plane.classify_polygon(poly) {
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

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane (i.e. it’s empty), just return
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        let mut front: Vec<Polygon<S>> = Vec::new();
        let mut back: Vec<Polygon<S>> = Vec::new();
        let mut coplanar_front: Vec<Polygon<S>> = Vec::new();
        let mut coplanar_back: Vec<Polygon<S>> = Vec::new();

        // For each polygon, split it by the node's plane.
        for poly in polygons {
            let (cf, cb, f, b) = plane.split_polygon(poly);
            coplanar_front.extend(cf);
            coplanar_back.extend(cb);
            front.extend(f);
            back.extend(b);
        }

        // Now decide where to send the coplanar polygons.  If the polygon’s normal
        // aligns with this node’s plane.normal, treat it as “front,” else treat as “back.”
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

        // Recursively clip the front polygons.
        if let Some(ref f) = self.front {
            front = f.clip_polygons(&front);
        }

        // Recursively clip the back polygons.
        if let Some(ref b) = self.back {
            back = b.clip_polygons(&back);
        } else {
            back.clear();
        }

        // Now combine front and back
        front.extend(back);
        front
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    #[cfg(not(feature = "parallel"))]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

	/// Return all polygons in this BSP tree using an iterative approach, avoiding potential stack overflow of recursive approach
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

        // Choose the first polygon's plane as the splitting plane if not already set.
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();

        let mut front: Vec<Polygon<S>> = Vec::new();
        let mut back: Vec<Polygon<S>> = Vec::new();

        // For each polygon, split it relative to the current node's plane.
        for p in polygons {
            let (coplanar_front, coplanar_back, f, b) = plane.split_polygon(p);

            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);

            front.extend(f);
            back.extend(b);
        }

        // Recursively build the front subtree.
        if !front.is_empty() {
            if self.front.is_none() {
                self.front = Some(Box::new(Node::new(&[])));
            }
            self.front.as_mut().unwrap().build(&front);
        }

        // Recursively build the back subtree.
        if !back.is_empty() {
            if self.back.is_none() {
                self.back = Some(Box::new(Node::new(&[])));
            }
            self.back.as_mut().unwrap().build(&back);
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

        for poly in &all_polys {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }
            let mut polygon_type = 0;
            let mut types = Vec::with_capacity(vcount);

            for vertex in &poly.vertices {
                let vertex_type = slicing_plane.orient_point(&vertex.pos);
                polygon_type |= vertex_type;
                types.push(vertex_type);
            }

            // Based on the combined classification of its vertices:
            match polygon_type {
                COPLANAR => {
                    // The entire polygon is in the plane, so push it to the coplanar list.
                    // Depending on normal alignment, it may be “coplanar_front” or “coplanar_back.”
                    // Usually we don’t care — we just return them as “in the plane.”
                    coplanar_polygons.push(poly.clone());
                },

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                },

                SPANNING => {
                    // The polygon crosses the plane. We'll gather the intersection points
                    // (the new vertices introduced on edges that cross the plane).
                    let mut crossing_points = Vec::new();

                    for i in 0..vcount {
                        let j = (i + 1) % vcount;
                        let ti = types[i];
                        let tj = types[j];
                        let vi = &poly.vertices[i];
                        let vj = &poly.vertices[j];

                        // If this vertex is on the "back" side, and the next vertex is on the
                        // "front" side (or vice versa), that edge crosses the plane.
                        // (Also if exactly one is COPLANAR and the other is FRONT or BACK, etc.)
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

                    // Typical convex polygons crossing a plane get exactly 2 intersection points.
                    // Concave polygons might produce 2 or more. We pair them up in consecutive pairs:
                    // e.g. if crossing_points = [p0, p1, p2, p3], we'll produce 2 edges: [p0,p1], [p2,p3].
                    // This is one simple heuristic. If you have an odd number, something degenerate happened.
                    for chunk in crossing_points.chunks_exact(2) {
                        intersection_edges.push([chunk[0].clone(), chunk[1].clone()]);
                    }
                    // If crossing_points.len() was not a multiple of 2, you can handle leftover
                    // points or flag them as errors, etc. We'll skip that detail here.
                },

                _ => {
                    // Shouldn't happen in a typical classification, but we can ignore
                },
            }
        }

        (coplanar_polygons, intersection_edges)
    }
}
