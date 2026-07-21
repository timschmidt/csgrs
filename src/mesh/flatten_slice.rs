//! Provides functions for flattening a `Mesh` against the Z=0 `Plane`
//! or slicing a `Mesh` with an arbitrary `Plane` into a `Profile`

use crate::mesh::Mesh;
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::sketch::Profile;
use crate::vertex::Vertex;
use hypercurve::{BooleanOp, Contour2, CurvePolicy, CurveRegion2, CurveString2};
use hyperlattice::Real;
use std::cmp::Ordering;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Flattens any 3D polygons by projecting them onto the XY plane (z=0),
    /// unifies them into one or more 2D polygons, and returns a hypercurve-backed
    /// 2D Profile.
    ///
    /// - All `polygons` in the Mesh are tessellated, projected into XY, promoted
    ///   to hypercurve contours, and unioned as `CurveRegion2` topology.
    /// - The XY coordinates are still finite mesh-boundary samples, but the
    ///   planar set operations stay in hypercurve. This follows Yap, "Towards Exact Geometric
    ///   Computation," *Computational Geometry* 7(1-2), 1997
    ///   (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
    ///   discipline in Hobby, "Practical Segment Intersection with Finite
    ///   Precision Output," *Computational Geometry* 13(4), 1999
    ///   (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    pub fn flatten(&self) -> Profile {
        let policy = CurvePolicy::certified();
        if self.is_certified_convex_triangle_surface()
            && let Some(flattened_region) = convex_projected_region(self)
        {
            return Profile::from_curve_topology(flattened_region, Vec::new(), Vec::new());
        }
        let mut flattened_region = CurveRegion2::empty();
        let mut material_contours = Vec::new();

        for poly in &self.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let mut ring = [
                    [tri[0].position.x.clone(), tri[0].position.y.clone()],
                    [tri[1].position.x.clone(), tri[1].position.y.clone()],
                    [tri[2].position.x.clone(), tri[2].position.y.clone()],
                    [tri[0].position.x.clone(), tri[0].position.y.clone()],
                ];
                let ring_points = ring
                    .iter()
                    .map(|point| hyperlimit::Point2::new(point[0].clone(), point[1].clone()))
                    .collect::<Vec<_>>();
                if matches!(
                    hyperlimit::ring_area_sign(&ring_points).value(),
                    Some(hyperlimit::Sign::Negative)
                ) {
                    ring.swap(1, 2);
                    ring[3] = ring[0].clone();
                }
                let Ok(contour) = Contour2::from_real_ring(&ring) else {
                    continue;
                };
                let Ok(triangle_region) = CurveRegion2::try_from_native_material_contours(
                    vec![contour.clone()],
                    &policy,
                ) else {
                    continue;
                };
                material_contours.push(contour);
                flattened_region = if flattened_region.is_empty() {
                    triangle_region
                } else {
                    match flattened_region.boolean_region(
                        &triangle_region,
                        BooleanOp::Union,
                        &policy,
                    ) {
                        Ok(region) => region,
                        Err(_) => CurveRegion2::try_from_native_material_contours(
                            material_contours.clone(),
                            &policy,
                        )
                        .unwrap_or_else(|_| CurveRegion2::empty()),
                    }
                };
            }
        }

        Profile::from_curve_topology(flattened_region, Vec::new(), Vec::new())
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
    /// Loop closure uses the shared exact hyperreal point predicate, promoting
    /// boundary `Point3<f64>` endpoints into `hyperlattice::Vector3` before
    /// testing squared distance for exact zero. This keeps slice topology
    /// decisions aligned with Yap's exact-geometric-computation model,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// Open intersection chains are promoted directly to native
    /// `hypercurve::CurveString2` wires, so finite projected points are no
    /// longer the source of slice path topology.
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use csgrs::mesh::plane::Plane;
    /// use csgrs::profile::Profile;
    /// use hyperlattice::Vector3;
    /// let cylinder = Mesh::<()>::cylinder(1.0, 2.0, 32, ());
    /// let plane_z0 = Plane::from_normal(Vector3::z(), 0.0);
    /// let cross_section = cylinder.slice(plane_z0);
    /// // `cross_section` will contain:
    /// //   - Possibly an open or closed polygon(s) at z=0
    /// //   - Or empty if no intersection
    /// ```
    pub fn slice(&self, plane: Plane) -> Profile {
        let (coplanar_polys, intersection_edges) =
            slice_polygons_by_plane(&self.polygons, &plane);

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

            // Check loop closure with Hyper's exact point predicate.
            let first = hyperlimit::Point3::new(
                chain[0].position.x.clone(),
                chain[0].position.y.clone(),
                chain[0].position.z.clone(),
            );
            let last = hyperlimit::Point3::new(
                chain[n - 1].position.x.clone(),
                chain[n - 1].position.y.clone(),
                chain[n - 1].position.z.clone(),
            );
            if matches!(hyperlimit::point3_equal(&first, &last).value(), Some(true)) {
                // Force them to be exactly the same, closing the line
                chain[n - 1] = chain[0].clone();
            }

            let points = chain
                .iter()
                .map(|vertex| [vertex.position.x.clone(), vertex.position.y.clone()])
                .collect::<Vec<_>>();

            let closed = points
                .first()
                .zip(points.last())
                .is_some_and(|(first, last)| {
                    let first = hyperlimit::Point2::new(first[0].clone(), first[1].clone());
                    let last = hyperlimit::Point2::new(last[0].clone(), last[1].clone());
                    matches!(hyperlimit::point2_equal(&first, &last).value(), Some(true))
                });

            if closed {
                if let Ok(contour) = Contour2::from_real_ring(&points) {
                    material_contours.push(contour);
                }
            } else if let Ok(wire) = CurveString2::from_real_point_iter(points.iter().cloned())
            {
                open_wires.push(wire);
            }
        }

        let region = CurveRegion2::try_from_native_material_contours(
            material_contours,
            &CurvePolicy::certified(),
        )
        .unwrap_or_else(|_| CurveRegion2::empty());

        Profile::from_curve_topology(region, open_wires, Vec::new())
    }
}

fn convex_projected_region<M: Clone + Debug + Send + Sync>(
    mesh: &Mesh<M>,
) -> Option<CurveRegion2> {
    let mut points = mesh
        .polygons
        .iter()
        .flat_map(|polygon| &polygon.vertices)
        .enumerate()
        .map(|(index, vertex)| (index, [vertex.position.x.clone(), vertex.position.y.clone()]))
        .collect::<Vec<_>>();
    points.sort_by(|(left_index, left), (right_index, right)| {
        exact_point_order(left, right)
            .unwrap_or(Ordering::Equal)
            .then_with(|| left_index.cmp(right_index))
    });
    if points.windows(2).any(|pair| {
        !matches!(
            exact_point_order(&pair[0].1, &pair[1].1),
            Some(Ordering::Less | Ordering::Equal)
        )
    }) {
        return None;
    }

    let mut unique = Vec::<[Real; 2]>::with_capacity(points.len());
    for (_, point) in points {
        match unique
            .last()
            .map(|previous| exact_point_order(previous, &point))
        {
            Some(Some(Ordering::Equal)) => {},
            Some(Some(Ordering::Less)) | None => unique.push(point),
            Some(Some(Ordering::Greater) | None) => return None,
        }
    }
    if unique.len() < 3 {
        return None;
    }

    let mut lower = Vec::with_capacity(unique.len());
    for point in &unique {
        push_convex_hull_point(&mut lower, point.clone())?;
    }
    let mut upper = Vec::with_capacity(unique.len());
    for point in unique.iter().rev() {
        push_convex_hull_point(&mut upper, point.clone())?;
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    if lower.len() < 3 {
        return None;
    }
    lower.push(lower[0].clone());
    let contour = Contour2::from_real_ring(&lower).ok()?;
    CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::certified())
        .ok()
}

fn exact_point_order(left: &[Real; 2], right: &[Real; 2]) -> Option<Ordering> {
    match hyperlimit::compare_reals(&left[0], &right[0]).value()? {
        Ordering::Equal => hyperlimit::compare_reals(&left[1], &right[1]).value(),
        order => Some(order),
    }
}

fn push_convex_hull_point(hull: &mut Vec<[Real; 2]>, point: [Real; 2]) -> Option<()> {
    while hull.len() >= 2 {
        let origin = hyperlimit::Point2::new(
            hull[hull.len() - 2][0].clone(),
            hull[hull.len() - 2][1].clone(),
        );
        let middle = hyperlimit::Point2::new(
            hull[hull.len() - 1][0].clone(),
            hull[hull.len() - 1][1].clone(),
        );
        let end = hyperlimit::Point2::new(point[0].clone(), point[1].clone());
        match hyperlimit::orient2d(&origin, &middle, &end).value()? {
            hyperlimit::Sign::Positive => break,
            hyperlimit::Sign::Negative | hyperlimit::Sign::Zero => {
                hull.pop();
            },
        }
    }
    hull.push(point);
    Some(())
}

#[cfg(test)]
mod convex_projection_tests {
    use super::*;

    #[test]
    fn sampled_sphere_certifies_convex_projection_hull() {
        let sphere = Mesh::sphere(Real::from(8_u8), 24, 12, ());
        assert!(sphere.is_certified_convex_triangle_surface());
        assert!(convex_projected_region(&sphere).is_some());
    }
}

/// Slice polygons directly against a plane without constructing a partition tree.
///
/// Each polygon is classified by the existing hyperreal plane predicate and
/// spanning edges are intersected in polygon order. This keeps slicing as
/// direct plane/hypercurve composition instead of routing through a tree owned
/// by `csgrs`, following Yap's exact-geometric-computation boundary discipline
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>) and the finite segment
/// output guidance in Hobby, "Practical Segment Intersection with Finite
/// Precision Output," *Computational Geometry* 13(4), 1999
/// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
fn slice_polygons_by_plane<M: Clone + Send + Sync>(
    polygons: &[crate::mesh::Polygon<M>],
    slicing_plane: &Plane,
) -> (Vec<crate::mesh::Polygon<M>>, Vec<[Vertex; 2]>) {
    let mut coplanar_polygons = Vec::new();
    let mut intersection_edges = Vec::new();

    for poly in polygons {
        let vcount = poly.vertices.len();
        if vcount < 2 {
            continue;
        }

        let types = poly
            .vertices
            .iter()
            .map(|vertex| slicing_plane.orient_point(&vertex.position))
            .collect::<Vec<_>>();
        let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

        match polygon_type {
            COPLANAR => coplanar_polygons.push(poly.clone()),
            FRONT | BACK => {},
            SPANNING => {
                let crossing_points = (0..vcount)
                    .filter_map(|i| {
                        let j = (i + 1) % vcount;
                        ((types[i] | types[j]) == SPANNING)
                            .then(|| {
                                slicing_plane
                                    .intersect_edge(&poly.vertices[i], &poly.vertices[j])
                            })
                            .flatten()
                    })
                    .collect::<Vec<_>>();

                intersection_edges.extend(
                    crossing_points
                        .chunks_exact(2)
                        .map(|chunk| [chunk[0].clone(), chunk[1].clone()]),
                );
            },
            _ => {},
        }
    }

    (coplanar_polygons, intersection_edges)
}

/// Take a list of intersection edges `[Vertex;2]` and merge them into polylines.
/// Each edge is a line segment between two 3D points.  We want to "knit" them together by
/// matching endpoints whose promoted coordinates are exactly equal, forming
/// either open or closed chains.
///
/// The direct scan is intentionally simple while the mesh carrier is still a
/// finite boundary type: endpoint identity is owned by `hyperlattice` /
/// `hyperreal`, not by a tolerance grid or `f64` hash. This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
///
/// This returns a `Vec` of polylines, where each polyline is a `Vec<Vertex>`.
fn unify_intersection_edges(edges: &[[Vertex; 2]]) -> Vec<Vec<Vertex>> {
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
        let mut chain = vec![e[0].clone(), e[1].clone()];

        // We walk "forward" from edge[1] if possible
        extend_chain_forward(&mut chain, &mut visited, edges);

        // We also might walk "backward" from edge[0], but
        // we can do that by reversing the chain at the end if needed. Alternatively,
        // we can do a separate pass.  Let's do it in place for clarity:
        chain.reverse();
        extend_chain_forward(&mut chain, &mut visited, edges);
        // Then reverse back so it goes in the original direction
        chain.reverse();

        chains.push(chain);
    }

    chains
}

/// Extends a chain "forward" by repeatedly finding any unvisited edge that starts
/// at the chain's current end vertex.
fn extend_chain_forward(chain: &mut Vec<Vertex>, visited: &mut [bool], edges: &[[Vertex; 2]]) {
    loop {
        // The chain's current end point:
        let last_v = chain.last().unwrap();

        // Among these candidates, we want one whose "other endpoint" we can follow
        // and is not visited yet.
        let mut found_next = None;
        'candidate_search: for (edge_idx, edge) in edges.iter().enumerate() {
            if visited[edge_idx] {
                continue;
            }
            for end_idx in 0..=1 {
                let last_point = hyperlimit::Point3::new(
                    last_v.position.x.clone(),
                    last_v.position.y.clone(),
                    last_v.position.z.clone(),
                );
                let edge_point = hyperlimit::Point3::new(
                    edge[end_idx].position.x.clone(),
                    edge[end_idx].position.y.clone(),
                    edge[end_idx].position.z.clone(),
                );
                if !matches!(
                    hyperlimit::point3_equal(&last_point, &edge_point).value(),
                    Some(true)
                ) {
                    continue;
                }

                // If this is edges[edge_idx][end_idx], the "other" end is
                // edges[edge_idx][1-end_idx]. We want that other end to
                // continue the chain.
                let other_end_idx = 1 - end_idx;
                let next_vertex = &edge[other_end_idx];

                // Mark visited
                visited[edge_idx] = true;
                found_next = Some(next_vertex.clone());
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
    use crate::hyper_math::{Real, hreal_from_f64};
    use hyperlattice::{Point3, Vector3};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn intersection_edge_knitting_uses_exact_hyperreal_endpoint_predicate() {
        let z = Vector3::z();
        let edge_a = [
            Vertex::new(p3(0.0, 0.0, 0.0), z.clone()),
            Vertex::new(p3(1.0, 0.0, 0.0), z.clone()),
        ];
        let exact_edge_b = [
            Vertex::new(p3(1.0, 0.0, 0.0), z.clone()),
            Vertex::new(p3(2.0, 0.0, 0.0), z.clone()),
        ];
        let near_edge_b = [
            Vertex::new(p3(1.0 + 1.0e-12, 0.0, 0.0), z.clone()),
            Vertex::new(p3(2.0, 0.0, 0.0), z),
        ];

        let exact_chains = unify_intersection_edges(&[edge_a.clone(), exact_edge_b.clone()]);
        assert_eq!(exact_chains.len(), 1);
        assert_eq!(exact_chains[0].len(), 3);
        assert_eq!(exact_chains[0][1].position, edge_a[1].position);
        assert_eq!(exact_chains[0][2].position, exact_edge_b[1].position);

        let near_chains = unify_intersection_edges(&[edge_a, near_edge_b]);
        assert_eq!(near_chains.len(), 2);
        assert!(near_chains.iter().all(|chain| chain.len() == 2));
    }
}
