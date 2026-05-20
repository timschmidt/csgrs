//! Provides functions for flattening a `Mesh` against the Z=0 `Plane`
//! or slicing a `Mesh` with an arbitrary `Plane` into a `Profile`

use crate::float_types::{Real, hpoints_within_epsilon, tolerance};
use crate::mesh::Mesh;
use crate::mesh::bsp::Node;
use crate::mesh::plane::Plane;
use crate::sketch::Profile;
use crate::vertex::Vertex;
use hashbrown::HashMap;
use hypercurve::{
    BooleanOp, Classification, Contour2, CurvePolicy, CurveString2, FillRule, Region2,
    finite_ring_signed_area,
};
use nalgebra::Point3;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Flattens any 3D polygons by projecting them onto the XY plane (z=0),
    /// unifies them into one or more 2D polygons, and returns a hypercurve-backed
    /// 2D Profile.
    ///
    /// - All `polygons` in the Mesh are tessellated, projected into XY, promoted
    ///   to hypercurve contours, and unioned as `Region2` topology.
    /// - The XY coordinates are still finite mesh-boundary samples, but the
    ///   planar set operations stay in hypercurve. This follows Yap, "Towards Exact Geometric
    ///   Computation," *Computational Geometry* 7(1-2), 1997
    ///   (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
    ///   discipline in Hobby, "Practical Segment Intersection with Finite
    ///   Precision Output," *Computational Geometry* 13(4), 1999
    ///   (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn flatten(&self) -> Profile<M> {
        let policy = CurvePolicy::certified();
        let mut flattened_region = Region2::empty();
        let mut material_contours = Vec::new();

        for poly in &self.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let mut ring = [
                    [tri[0].position.x, tri[0].position.y],
                    [tri[1].position.x, tri[1].position.y],
                    [tri[2].position.x, tri[2].position.y],
                    [tri[0].position.x, tri[0].position.y],
                ];
                if finite_ring_signed_area(&ring) < 0.0 {
                    ring.swap(1, 2);
                    ring[3] = ring[0];
                }
                let Ok(contour) = Contour2::from_finite_ring(&ring) else {
                    continue;
                };
                let triangle_region = Region2::from_material_contours(vec![contour.clone()]);
                flattened_region = if flattened_region.is_empty() {
                    triangle_region
                } else {
                    match flattened_region.boolean_region(
                        &triangle_region,
                        BooleanOp::Union,
                        FillRule::NonZero,
                        &policy,
                    ) {
                        Ok(Classification::Decided(region)) => region,
                        Ok(Classification::Uncertain(_)) | Err(_) => {
                            material_contours.push(contour);
                            Region2::from_material_contours(material_contours.clone())
                        },
                    }
                };
                material_contours = flattened_region.material_contours().to_vec();
            }
        }

        Profile::from_region_and_wires_with_origin(
            flattened_region,
            Vec::new(),
            self.metadata.clone(),
            Vertex::default(),
            Profile::<M>::prepare_origin_transform(Vertex::default()),
        )
    }

    /// Slice this solid by a given `plane`, returning a new `Profile` whose polygons
    /// are either:
    /// - The polygons that lie exactly in the slicing plane (coplanar), or
    /// - Polygons formed by the intersection edges (each a line, possibly open or closed).
    ///
    /// The returned `Profile` can contain:
    /// - **Closed polygons** that are coplanar,
    /// - **Open polygons** (poly-lines) if the plane cuts through edges,
    /// - Potentially **closed loops** if the intersection lines form a cycle.
    ///
    /// Loop closure uses the shared hyperreal point predicate, promoting
    /// boundary `Point3<f64>` endpoints into `hyperlattice::Vector3` before
    /// testing squared distance. This keeps slice topology decisions aligned
    /// with Yap's exact-geometric-computation model, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// Open intersection chains are promoted directly to native
    /// `hypercurve::CurveString2` wires, so finite projected points are no
    /// longer the source of slice path topology.
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use csgrs::mesh::plane::Plane;
    /// use csgrs::sketch::Profile;
    /// use nalgebra::Vector3;
    /// let cylinder = Mesh::<()>::cylinder(1.0, 2.0, 32, ());
    /// let plane_z0 = Plane::from_normal(Vector3::z(), 0.0);
    /// let cross_section = cylinder.slice(plane_z0);
    /// // `cross_section` will contain:
    /// //   - Possibly an open or closed polygon(s) at z=0
    /// //   - Or empty if no intersection
    /// ```
    pub fn slice(&self, plane: Plane) -> Profile<M> {
        // Build a BSP from all of our polygons:
        let node = Node::from_polygons(&self.polygons.clone());

        // Ask the BSP for coplanar polygons + intersection edges:
        let (coplanar_polys, intersection_edges) = node.slice(&plane);

        // "Knit" those intersection edges into polylines. Each edge is [vA, vB].
        let polylines_3d = unify_intersection_edges(&intersection_edges);

        // Convert each polyline of vertices into a Polygon<M>
        let mut result_polygons = Vec::new();

        // Add the coplanar polygons. We can re‐assign their plane to `plane` to ensure
        // they share the exact plane definition (in case of numeric drift).
        for p in coplanar_polys {
            result_polygons.push(p);
        }

        let mut open_wires = Vec::new();
        let mut material_contours = Vec::new();

        // Convert the "chains" or loops into open/closed polygons
        for mut chain in polylines_3d {
            let n = chain.len();
            if n < 2 {
                // degenerate
                continue;
            }

            // Check loop closure with the shared hyperreal point predicate.
            if hpoints_within_epsilon(
                &chain[0].position,
                &chain[n - 1].position,
                crate::float_types::tolerance(),
            ) {
                // Force them to be exactly the same, closing the line
                chain[n - 1] = chain[0];
            }

            let points = chain
                .iter()
                .map(|vertex| [vertex.position.x, vertex.position.y])
                .collect::<Vec<_>>();

            let closed = points
                .first()
                .zip(points.last())
                .is_some_and(|(first, last)| {
                    (first[0] - last[0]).abs() <= crate::float_types::tolerance()
                        && (first[1] - last[1]).abs() <= crate::float_types::tolerance()
                });

            if closed {
                if let Ok(contour) = Contour2::from_finite_ring(&points) {
                    material_contours.push(contour);
                }
            } else if let Ok(wire) =
                CurveString2::from_finite_point_iter(points.iter().copied())
            {
                open_wires.push(wire);
            }
        }

        let region = if material_contours.is_empty() {
            Region2::empty()
        } else {
            Region2::from_material_contours(material_contours)
        };

        Profile::from_region_and_wires_with_origin(
            region,
            open_wires,
            self.metadata.clone(),
            Vertex::default(),
            Profile::<M>::prepare_origin_transform(Vertex::default()),
        )
    }
}

// Build a small helper for hashing endpoints:
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndKey(i64, i64, i64);

/// Round a floating coordinate to a tolerance-scaled grid for endpoint hashing.
///
/// The hash is only a candidate bucket: actual endpoint identity is still
/// verified by [`hpoints_within_epsilon`], which promotes coordinates into
/// `hyperlattice::Vector3`/`hyperreal::Real`. Using the crate tolerance for the
/// coarse grid keeps this acceleration structure aligned with the exact-aware
/// topology predicate, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn quantize(x: Real) -> i64 {
    (x / tolerance()).round() as i64
}

/// Convert a Vertex's position to an EndKey
fn make_key(position: &Point3<Real>) -> EndKey {
    EndKey(
        quantize(position.x),
        quantize(position.y),
        quantize(position.z),
    )
}

fn neighboring_keys(key: EndKey) -> impl Iterator<Item = EndKey> {
    (-1..=1).flat_map(move |dx| {
        (-1..=1).flat_map(move |dy| {
            (-1..=1).map(move |dz| EndKey(key.0 + dx, key.1 + dy, key.2 + dz))
        })
    })
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
            let k = make_key(&v.position);
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
        let key = make_key(&last_v.position);

        // Among these candidates, we want one whose "other endpoint" we can follow
        // and is not visited yet.
        let mut found_next = None;
        'candidate_search: for candidate_key in neighboring_keys(key) {
            let Some(candidates) = adjacency.get(&candidate_key) else {
                continue;
            };

            for &(edge_idx, end_idx) in candidates {
                if visited[edge_idx] {
                    continue;
                }
                if !hpoints_within_epsilon(
                    &last_v.position,
                    &edges[edge_idx][end_idx].position,
                    tolerance(),
                ) {
                    continue;
                }

                // If this is edges[edge_idx][end_idx], the "other" end is
                // edges[edge_idx][1-end_idx]. We want that other end to
                // continue the chain.
                let other_end_idx = 1 - end_idx;
                let next_vertex = &edges[edge_idx][other_end_idx];

                // Mark visited
                visited[edge_idx] = true;
                found_next = Some(*next_vertex);
                break 'candidate_search;
            }
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn intersection_edge_knitting_uses_hyperreal_endpoint_predicate() {
        let z = Vector3::z();
        let edge_a = [
            Vertex::new(Point3::new(0.0, 0.0, 0.0), z),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), z),
        ];
        let edge_b = [
            Vertex::new(Point3::new(1.0 + tolerance() * 0.25, 0.0, 0.0), z),
            Vertex::new(Point3::new(2.0, 0.0, 0.0), z),
        ];

        let chains = unify_intersection_edges(&[edge_a, edge_b]);

        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].len(), 3);
        assert!(hpoints_within_epsilon(
            &chains[0][1].position,
            &edge_a[1].position,
            tolerance()
        ));
        assert_eq!(chains[0][2].position, edge_b[1].position);
    }
}
