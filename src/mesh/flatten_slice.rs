//! Provides functions for flattening a `Mesh` against the Z=0 `Plane`
//! or slicing a `Mesh` with an arbitrary `Plane` into a `Sketch`

use crate::float_types::{Real, tolerance};
use crate::mesh::Mesh;
use crate::mesh::bsp::Node;
use crate::mesh::plane::Plane;
use crate::mesh::vertex::Vertex;
use crate::sketch::Sketch;
use geo::{
    BooleanOps, Geometry, GeometryCollection, LineString, MultiPolygon, Orient,
    Polygon as GeoPolygon, coord, orient::Direction,
};
use hashbrown::HashMap;
use nalgebra::Point3;
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Flattens any 3D polygons by projecting them onto the XY plane (z=0),
    /// unifies them into one or more 2D polygons, and returns a purely 2D Sketch.
    ///
    /// - All `polygons` in the Mesh are tessellated, projected into XY, and unioned.
    /// - The output is a Sketch containing the final 2D shape.
    pub fn flatten(&self) -> Sketch<S> {
        // Convert all 3D polygons into a collection of 2D polygons
        let mut flattened_3d = Vec::new(); // will store geo::Polygon<Real>

        for poly in &self.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let ring = vec![
                    (tri[0].pos.x, tri[0].pos.y),
                    (tri[1].pos.x, tri[1].pos.y),
                    (tri[2].pos.x, tri[2].pos.y),
                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                ];
                let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
                flattened_3d.push(polygon_2d);
            }
        }

        // Union all these polygons together into one MultiPolygon
        // (We could chain them in a fold-based union.)
        let unioned_from_3d = if flattened_3d.is_empty() {
            MultiPolygon::new(Vec::new())
        } else {
            // Start with the first polygon as a MultiPolygon
            let mut mp_acc = MultiPolygon(vec![flattened_3d[0].clone()]);
            // Union in the rest
            for p in flattened_3d.iter().skip(1) {
                mp_acc = mp_acc.union(&MultiPolygon(vec![p.clone()]));
            }
            mp_acc
        };

        // Ensure consistent orientation (CCW for exteriors):
        let oriented = unioned_from_3d.orient(Direction::Default);

        // Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));

        // Return a Sketch: polygons empty, geometry has the final shape
        Sketch {
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: Sketch::<S>::prepare_origin_vec_and_quat(self.origin),
        }
    }

    /// Slice this solid by a given `plane`, returning a new `Sketch` whose polygons
    /// are either:
    /// - The polygons that lie exactly in the slicing plane (coplanar), or
    /// - Polygons formed by the intersection edges (each a line, possibly open or closed).
    ///
    /// The returned `Sketch` can contain:
    /// - **Closed polygons** that are coplanar,
    /// - **Open polygons** (poly-lines) if the plane cuts through edges,
    /// - Potentially **closed loops** if the intersection lines form a cycle.
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use csgrs::mesh::plane::Plane;
    /// use csgrs::sketch::Sketch;
    /// use nalgebra::Vector3;
    /// let cylinder = Mesh::<()>::cylinder(1.0, 2.0, 32, None);
    /// let plane_z0 = Plane::from_normal(Vector3::z(), 0.0);
    /// let cross_section = cylinder.slice(plane_z0);
    /// // `cross_section` will contain:
    /// //   - Possibly an open or closed polygon(s) at z=0
    /// //   - Or empty if no intersection
    /// ```
    pub fn slice(&self, plane: Plane) -> Sketch<S> {
        // Build a BSP from all of our polygons:
        let node = Node::from_polygons(&self.polygons.clone());

        // Ask the BSP for coplanar polygons + intersection edges:
        let (coplanar_polys, intersection_edges) = node.slice(&plane);

        // "Knit" those intersection edges into polylines. Each edge is [vA, vB].
        let polylines_3d = unify_intersection_edges(&intersection_edges);

        // Convert each polyline of vertices into a Polygon<S>
        let mut result_polygons = Vec::new();

        // Add the coplanar polygons. We can re‐assign their plane to `plane` to ensure
        // they share the exact plane definition (in case of numeric drift).
        for p in coplanar_polys {
            result_polygons.push(p);
        }

        let mut new_gc = GeometryCollection::default();

        // Convert the "chains" or loops into open/closed polygons
        for mut chain in polylines_3d {
            let n = chain.len();
            if n < 2 {
                // degenerate
                continue;
            }

            // check if first and last point are within tolerance of each other
            let dist_sq = (chain[0].pos - chain[n - 1].pos).norm_squared();
            if dist_sq < tolerance() * tolerance() {
                // Force them to be exactly the same, closing the line
                chain[n - 1] = chain[0];
            }

            let polyline = LineString::new(
                chain
                    .iter()
                    .map(|vertex| {
                        coord! {x: vertex.pos.x, y: vertex.pos.y}
                    })
                    .collect(),
            );

            if polyline.is_closed() {
                let polygon = GeoPolygon::new(polyline, vec![]);
                let oriented = polygon.orient(Direction::Default);
                new_gc.0.push(Geometry::Polygon(oriented));
            } else {
                new_gc.0.push(Geometry::LineString(polyline));
            }
        }

        // Return Sketch
        Sketch {
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: Sketch::<S>::prepare_origin_vec_and_quat(self.origin),
        }
    }
}

// Build a small helper for hashing endpoints:
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndKey(i64, i64, i64);

/// Round a floating to a grid for hashing
fn quantize(x: Real) -> i64 {
    // For example, scale by 1e8
    (x * 1e8).round() as i64
}

/// Convert a Vertex's position to an EndKey
fn make_key(pos: &Point3<Real>) -> EndKey {
    EndKey(quantize(pos.x), quantize(pos.y), quantize(pos.z))
}

/// Take a list of intersection edges `[Vertex;2]` and merge them into polylines.
/// Each edge is a line segment between two 3D points.  We want to "knit" them together by
/// matching endpoints that lie within tolerance of each other, forming either open or closed chains.
///
/// This returns a `Vec` of polylines, where each polyline is a `Vec<Vertex>`.
fn unify_intersection_edges(edges: &[[Vertex; 2]]) -> Vec<Vec<Vertex>> {
    // We will store adjacency by a "key" that identifies an endpoint up to tolerance,
    // then link edges that share the same key.

    // Adjacency map: key -> list of (edge_index, is_start_or_end)
    // We'll store "(edge_idx, which_end)" as which_end = 0 or 1 for edges[edge_idx][0/1].
    let mut adjacency: HashMap<EndKey, Vec<(usize, usize)>> = HashMap::new();

    // Collect all endpoints
    for (i, edge) in edges.iter().enumerate() {
        for (end_idx, v) in edge.iter().enumerate() {
            let k = make_key(&v.pos);
            adjacency.entry(k).or_default().push((i, end_idx));
        }
    }

    // We'll keep track of which edges have been “visited” in the final polylines.
    let mut visited = vec![false; edges.len()];

    let mut chains: Vec<Vec<Vertex>> = Vec::new();

    // For each edge not yet visited, we "walk" outward from one end, building a chain
    for start_edge_idx in 0..edges.len() {
        if visited[start_edge_idx] {
            continue;
        }
        // Mark it visited
        visited[start_edge_idx] = true;

        // Our chain starts with `edges[start_edge_idx]`. We can build a small function to “walk”:
        // We'll store it in the direction edge[0] -> edge[1]
        let e = &edges[start_edge_idx];
        let mut chain = vec![e[0], e[1]];

        // We walk "forward" from edge[1] if possible
        extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);

        // We also might walk "backward" from edge[0], but
        // we can do that by reversing the chain at the end if needed. Alternatively,
        // we can do a separate pass.  Let's do it in place for clarity:
        chain.reverse();
        extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);
        // Then reverse back so it goes in the original direction
        chain.reverse();

        chains.push(chain);
    }

    chains
}

/// Extends a chain "forward" by repeatedly finding any unvisited edge that starts
/// at the chain's current end vertex.
fn extend_chain_forward(
    chain: &mut Vec<Vertex>,
    adjacency: &HashMap<EndKey, Vec<(usize, usize)>>,
    visited: &mut [bool],
    edges: &[[Vertex; 2]],
) {
    loop {
        // The chain's current end point:
        let last_v = chain.last().unwrap();
        let key = make_key(&last_v.pos);

        // Find candidate edges that share this endpoint
        let Some(candidates) = adjacency.get(&key) else {
            break;
        };

        // Among these candidates, we want one whose "other endpoint" we can follow
        // and is not visited yet.
        let mut found_next = None;
        for &(edge_idx, end_idx) in candidates {
            if visited[edge_idx] {
                continue;
            }
            // If this is edges[edge_idx][end_idx], the "other" end is edges[edge_idx][1-end_idx].
            // We want that other end to continue the chain.
            let other_end_idx = 1 - end_idx;
            let next_vertex = &edges[edge_idx][other_end_idx];

            // But we must also confirm that the last_v is indeed edges[edge_idx][end_idx]
            // (within tolerance) which we have checked via the key, so likely yes.

            // Mark visited
            visited[edge_idx] = true;
            found_next = Some(*next_vertex);
            break;
        }

        match found_next {
            Some(v) => {
                chain.push(v);
            },
            None => {
                break;
            },
        }
    }
}
