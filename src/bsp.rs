use crate::float_types::EPSILON;
use crate::plane::{Plane, BACK, FRONT, COPLANAR, SPANNING};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "parallel")]
use rayon::join;

/// A BSP tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    pub plane: Option<Plane>,
    pub front: Option<Box<Node<S>>>,
    pub back: Option<Box<Node<S>>>,
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
    pub fn invert(&mut self) {
        for p in &mut self.polygons {
            p.flip();
        }
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        // Recursively invert children in parallel, if both exist
        #[cfg(feature = "parallel")]
        match (&mut self.front, &mut self.back) {
            (Some(front_node), Some(back_node)) => {
                join(|| front_node.invert(), || back_node.invert());
            }
            (Some(front_node), None) => front_node.invert(),
            (None, Some(back_node)) => back_node.invert(),
            (None, None) => {}
        }

        #[cfg(not(feature = "parallel"))]
        if let Some(ref mut front) = self.front {
            front.invert();
        }
        #[cfg(not(feature = "parallel"))]
        if let Some(ref mut back) = self.back {
            back.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
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

    // ------------------------------------------------------------------------
    // Clip Polygons (parallel version)
    // ------------------------------------------------------------------------
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

    /// Parallel remove all polygons in this BSP tree that are inside the other BSP tree
    #[cfg(feature = "parallel")]
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        // clip self.polygons in parallel
        let new_polygons = bsp.clip_polygons(&self.polygons);
        self.polygons = new_polygons;

        // Recurse in parallel over front/back
        match (&mut self.front, &mut self.back) {
            (Some(front_node), Some(back_node)) => {
                join(|| front_node.clip_to(bsp), || back_node.clip_to(bsp));
            }
            (Some(front_node), None) => front_node.clip_to(bsp),
            (None, Some(back_node)) => back_node.clip_to(bsp),
            (None, None) => {}
        }
    }

    /// Return all polygons in this BSP tree
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = self.polygons.clone();
        if let Some(ref front) = self.front {
            result.extend(front.all_polygons());
        }
        if let Some(ref back) = self.back {
            result.extend(back.all_polygons());
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

    // ------------------------------------------------------------------------
    // Build (parallel version)
    // ------------------------------------------------------------------------
    #[cfg(feature = "parallel")]
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();

        // Split polygons in parallel
        let (mut coplanar_front, mut coplanar_back, mut front, mut back) = polygons
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

        // Recursively build front/back in parallel
        match (!front.is_empty(), !back.is_empty()) {
            (true, true) => {
                if self.front.is_none() {
                    self.front = Some(Box::new(Node::new(&[])));
                }
                if self.back.is_none() {
                    self.back = Some(Box::new(Node::new(&[])));
                }

                let front_node = self.front.as_mut().unwrap();
                let back_node = self.back.as_mut().unwrap();
                join(|| front_node.build(&front), || back_node.build(&back));
            }
            (true, false) => {
                if self.front.is_none() {
                    self.front = Some(Box::new(Node::new(&[])));
                }
                self.front.as_mut().unwrap().build(&front);
            }
            (false, true) => {
                if self.back.is_none() {
                    self.back = Some(Box::new(Node::new(&[])));
                }
                self.back.as_mut().unwrap().build(&back);
            }
            (false, false) => {}
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
                }

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                }

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
                }

                _ => {
                    // Shouldn't happen in a typical classification, but we can ignore
                }
            }
        }

        (coplanar_polygons, intersection_edges)
    }

    // ------------------------------------------------------------------------
    // Slice (parallel version)
    // ------------------------------------------------------------------------
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
                    }
                    FRONT | BACK => {
                        // Entirely on one side => no intersection
                        (Vec::new(), Vec::new())
                    }
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
                    }
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
