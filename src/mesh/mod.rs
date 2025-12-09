//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::float_types::{
    parry3d::{
        bounding_volume::{Aabb, BoundingVolume},
        query::RayCast,
        shape::Shape,
    },
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape, TriMesh, Triangle,
    },
    {Real, tolerance},
};
use crate::mesh::{bsp::Node, plane::Plane, polygon::Polygon, vertex::Vertex};
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{CoordsIter, Geometry, Polygon as GeoPolygon};
use hashbrown::HashMap;
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3, partial_max, partial_min,
};
use std::{
    cmp::{Ordering, PartialEq},
    fmt::Debug,
    num::NonZeroU32,
    sync::OnceLock,
};
use crate::bmesh::BMesh;

#[cfg(feature = "parallel")]
use rayon::{iter::IntoParallelRefIterator, prelude::*};

pub mod bsp;
pub mod bsp_parallel;

#[cfg(feature = "chull")]
pub mod convex_hull;
pub mod flatten_slice;

#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub mod polygon;

pub mod connectivity;
pub mod manifold;
pub mod quality;
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod vertex;

fn point_in_aabb_with_tolerance(p: &Point3<Real>, bb: &Aabb, eps: Real) -> bool {
    p.x >= bb.mins.x - eps
        && p.x <= bb.maxs.x + eps
        && p.y >= bb.mins.y - eps
        && p.y <= bb.maxs.y + eps
        && p.z >= bb.mins.z - eps
        && p.z <= bb.maxs.z + eps
}

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug + PartialEq> Mesh<S> {
    /// Compare just the `metadata` fields of two meshes
    #[inline]
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }

    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &S) -> Mesh<S> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| p.metadata.as_ref() == Some(needle))
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Build a Mesh from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>], metadata: Option<S>) -> Self {
        let mut mesh = Mesh::new();
        mesh.polygons = polygons.to_vec();
        mesh.metadata = metadata;
        mesh
    }

    /// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
    fn partition_polys(
        polys: &[Polygon<S>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        polys
            .iter()
            .cloned()
            .partition(|p| p.bounding_box().intersects(other_bb))
    }

    /// Helper to collect all vertices from the CSG.
    #[cfg(not(feature = "parallel"))]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Parallel helper to collect all vertices from the CSG.
    #[cfg(feature = "parallel")]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .par_iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Pre-pass to remove T-junctions between polygons by inserting
    /// missing vertices on shared / colinear overlapping edges.
    ///
    /// For each vertex of each polygon, we look for edges of *other*
    /// polygons on which that vertex lies (within a tolerance). If we
    /// find such an edge and the vertex is strictly inside that edge,
    /// we schedule a split of that edge and insert the vertex there.
    ///
    /// This works even when two polygons' edges do not share endpoints
    /// but are colinear and overlapping, because the overlapping
    /// endpoints lie strictly in the interior of the opposite segment.
    #[cfg(not(feature = "parallel"))]
    fn fix_t_junctions_on_shared_edges(polygons: &mut [Polygon<S>]) {
        let eps = tolerance();
        let eps2 = eps * eps;

        let poly_count = polygons.len();
        if poly_count < 2 {
            return;
        }

        // Precompute per-polygon AABBs.
        let poly_aabbs: Vec<Aabb> = polygons.iter().map(|p| p.bounding_box()).collect();

        // Precompute polygon neighbors using AABB intersection.
        //
        // neighbors[i] = all j != i such that Aabb(i) intersects Aabb(j).
        let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); poly_count];
        for i in 0..poly_count {
            for j in (i + 1)..poly_count {
                if poly_aabbs[i].intersects(&poly_aabbs[j]) {
                    neighbors[i].push(j);
                    neighbors[j].push(i);
                }
            }
        }

        // edge_splits[poly_index][edge_start_index] = Vec<(t along edge, Vertex)>
        let mut edge_splits: Vec<HashMap<usize, Vec<(Real, Vertex)>>> =
            vec![HashMap::new(); poly_count];

        // Detection pass: find vertices that lie on other polygons' edges.
        for i in 0..poly_count {
            let poly_i = &polygons[i];
            if poly_i.vertices.len() < 2 {
                continue;
            }

            // If this polygon has no AABB neighbors, it can't form T-junctions
            // with any other polygon.
            if neighbors[i].is_empty() {
                continue;
            }

            for vert in &poly_i.vertices {
                // Only test against polygons whose AABBs intersect poly_i’s AABB.
                for &j in &neighbors[i] {
                    let poly_j = &polygons[j];
                    if poly_j.vertices.len() < 2 {
                        continue;
                    }

                    // Fast reject: if the vertex lies outside polygon j’s AABB
                    // (with tolerance), it cannot lie on any edge of j.
                    if !point_in_aabb_with_tolerance(&vert.pos, &poly_aabbs[j], eps) {
                        continue;
                    }

                    let verts_j = &poly_j.vertices;
                    let n_j = verts_j.len();

                    for edge_start in 0..n_j {
                        let a = &verts_j[edge_start];
                        let b = &verts_j[(edge_start + 1) % n_j];

                        // Edge vector AB
                        let ab = b.pos - a.pos;
                        let ab_len_sq = ab.norm_squared();
                        if ab_len_sq < eps2 {
                            // Degenerate edge
                            continue;
                        }

                        // Vectors from endpoints to the candidate vertex
                        let av = vert.pos - a.pos;
                        let bv = vert.pos - b.pos;

                        // Skip if vertex is basically at one of the endpoints
                        if av.norm_squared() < eps2 || bv.norm_squared() < eps2 {
                            continue;
                        }

                        // Parametric coordinate of the projection of vert onto AB
                        let t = ab.dot(&av) / ab_len_sq;

                        // Only consider points strictly inside the segment (avoid ends)
                        if t <= eps || t >= 1.0 - eps {
                            // Too close to edge endpoints
                            continue;
                        }

                        // Closest point on AB to vert
                        let projected = a.pos + ab * t;

                        // Check that the vertex actually lies on the segment (within eps)
                        if (vert.pos - projected).norm_squared() > eps2 {
                            // Not actually on the edge
                            continue;
                        }

                        // We now know vert lies on edge (a, b) of polygon j.
                        // Create a vertex consistent with polygon j's plane.
                        let new_vertex = Vertex::new(vert.pos, poly_j.plane.normal());

                        // Register the split
                        let entry = edge_splits[j].entry(edge_start).or_insert_with(Vec::new);

                        // Avoid duplicate splits (same t / same position)
                        let mut already_present = false;
                        for (existing_t, existing_v) in entry.iter() {
                            if (existing_t - t).abs() < eps {
                                already_present = true;
                                break;
                            }

                            if (existing_v.pos - new_vertex.pos).norm_squared() < eps2 {
                                already_present = true;
                                break;
                            }
                        }

                        if !already_present {
                            entry.push((t, new_vertex));
                        }
                    }
                }
            }
        }

        // Application pass: actually split edges and rebuild polygon vertex lists
        for (poly_index, poly) in polygons.iter_mut().enumerate() {
            let splits_map = &edge_splits[poly_index];
            if splits_map.is_empty() {
                continue;
            }

            let original = poly.vertices.clone();
            let n = original.len();
            if n < 2 {
                continue;
            }

            let extra_vertices: usize = splits_map.values().map(|v| v.len()).sum();

            let mut new_vertices = Vec::with_capacity(n + extra_vertices);

            for (edge_start, vertex) in original.iter().enumerate() {
                // Always keep the original starting vertex of the edge
                new_vertices.push(*vertex);

                if let Some(splits) = splits_map.get(&edge_start) {
                    // Sort new points along the edge
                    let mut splits_sorted = splits.clone();
                    splits_sorted.sort_by(|(t_a, _), (t_b, _)| {
                        t_a.partial_cmp(t_b).unwrap_or(Ordering::Equal)
                    });

                    // Insert them in parametric order between edge_start and edge_start+1
                    for (_, v) in splits_sorted {
                        new_vertices.push(v);
                    }
                }
            }

            poly.vertices = new_vertices;
            // Inserted vertices lie on existing edges, so the polygon AABB
            // should remain valid and we don't need to reset poly.bounding_box.
        }
    }

    #[cfg(feature = "parallel")]
    fn fix_t_junctions_on_shared_edges(polygons: &mut [Polygon<S>]) {
        use rayon::prelude::*;

        let eps = tolerance();
        let eps2 = eps * eps;

        let poly_count = polygons.len();
        if poly_count < 2 {
            return;
        }

        // Immutable view of polygons for detection.
        let polys: &[Polygon<S>] = &*polygons;

        // Precompute per-polygon AABBs.
        let poly_aabbs: Vec<Aabb> = polys.iter().map(|p| p.bounding_box()).collect();

        // Precompute polygon neighbors using AABB intersection.
        //
        // neighbors[i] = all j != i such that Aabb(i) intersects Aabb(j).
        let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); poly_count];
        for i in 0..poly_count {
            for j in (i + 1)..poly_count {
                if poly_aabbs[i].intersects(&poly_aabbs[j]) {
                    neighbors[i].push(j);
                    neighbors[j].push(i);
                }
            }
        }

        // --- Detection pass (parallel) ---
        let edge_splits: Vec<HashMap<usize, Vec<(Real, Vertex)>>> = (0..poly_count)
            .into_par_iter()
            .map(|j| {
                let poly_j = &polys[j];
                let mut splits_map: HashMap<usize, Vec<(Real, Vertex)>> = HashMap::new();

                let verts_j = &poly_j.vertices;
                let n_j = verts_j.len();
                if n_j < 2 {
                    return splits_map;
                }

                let bb_j = &poly_aabbs[j];

                for edge_start in 0..n_j {
                    let a = &verts_j[edge_start];
                    let b = &verts_j[(edge_start + 1) % n_j];

                    // Edge vector AB
                    let ab = b.pos - a.pos;
                    let ab_len_sq = ab.norm_squared();
                    if ab_len_sq < eps2 {
                        // Degenerate edge
                        continue;
                    }

                    // Test all vertices of AABB-neighboring polygons only.
                    for &i in &neighbors[j] {
                        let poly_i = &polys[i];
                        if poly_i.vertices.len() < 2 {
                            continue;
                        }

                        for vert in &poly_i.vertices {
                            // Fast reject: vertex must at least lie inside poly_j’s AABB
                            // (with tolerance), otherwise it cannot be on any edge of j.
                            if !point_in_aabb_with_tolerance(&vert.pos, bb_j, eps) {
                                continue;
                            }

                            // Vectors from endpoints to the candidate vertex
                            let av = vert.pos - a.pos;
                            let bv = vert.pos - b.pos;

                            // Skip if vertex is basically at one of the endpoints
                            if av.norm_squared() < eps2 || bv.norm_squared() < eps2 {
                                continue;
                            }

                            // Parametric coordinate of the projection of vert onto AB
                            let t = ab.dot(&av) / ab_len_sq;

                            // Only consider points strictly inside the segment (avoid ends)
                            if t <= eps || t >= 1.0 - eps {
                                // Too close to edge endpoints
                                continue;
                            }

                            // Closest point on AB to vert
                            let projected = a.pos + ab * t;

                            // Check that the vertex actually lies on the segment (within eps)
                            if (vert.pos - projected).norm_squared() > eps2 {
                                // Not actually on the edge
                                continue;
                            }

                            // We now know vert lies on edge (a, b) of polygon j.
                            // Create a vertex consistent with polygon j's plane.
                            let new_vertex = Vertex::new(vert.pos, poly_j.plane.normal());

                            // Register the split
                            let entry = splits_map.entry(edge_start).or_insert_with(Vec::new);

                            // Avoid duplicate splits (same t / same position)
                            let mut already_present = false;
                            for (existing_t, existing_v) in entry.iter() {
                                if (existing_t - t).abs() < eps {
                                    already_present = true;
                                    break;
                                }

                                if (existing_v.pos - new_vertex.pos).norm_squared() < eps2 {
                                    already_present = true;
                                    break;
                                }
                            }

                            if !already_present {
                                entry.push((t, new_vertex));
                            }
                        }
                    }
                }

                splits_map
            })
            .collect();

        // --- Application pass (parallel) ---
        polygons
            .par_iter_mut()
            .enumerate()
            .for_each(|(poly_index, poly)| {
                let splits_map = &edge_splits[poly_index];
                if splits_map.is_empty() {
                    return;
                }

                let original = poly.vertices.clone();
                let n = original.len();
                if n < 2 {
                    return;
                }

                let extra_vertices: usize = splits_map.values().map(|v| v.len()).sum();

                let mut new_vertices = Vec::with_capacity(n + extra_vertices);

                for (edge_start, vertex) in original.iter().enumerate() {
                    // Always keep the original starting vertex of the edge
                    new_vertices.push(*vertex);

                    if let Some(splits) = splits_map.get(&edge_start) {
                        // Sort new points along the edge
                        let mut splits_sorted = splits.clone();
                        splits_sorted.sort_by(|(t_a, _), (t_b, _)| {
                            t_a.partial_cmp(t_b).unwrap_or(Ordering::Equal)
                        });

                        // Insert them in parametric order between edge_start and edge_start+1
                        for (_, v) in splits_sorted {
                            new_vertices.push(v);
                        }
                    }
                }

                poly.vertices = new_vertices;
                // Inserted vertices lie on existing edges, so the polygon AABB
                // remains valid and we don't need to reset poly.bounding_box.
            });
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<S> {
        // Work on a local copy so we do not mutate the original mesh.
        let mut polygons = self.polygons.clone();

        // Fix T-junctions by inserting shared-edge vertices.
        Self::fix_t_junctions_on_shared_edges(&mut polygons);

        let triangles = polygons
            .iter()
            .flat_map(|poly| {
                poly.triangulate().into_iter().map(move |triangle| {
                    Polygon::new(triangle.to_vec(), poly.metadata.clone())
                })
            })
            .collect::<Vec<_>>();

        Mesh::from_polygons(&triangles, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<S> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(vec![tri[0], tri[1], tri[2]], poly.metadata.clone())
                })
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(vec![tri[0], tri[1], tri[2]], poly.metadata.clone())
                })
            })
            .collect();

        Mesh::from_polygons(&new_polygons, self.metadata.clone())
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use core::num::NonZeroU32;
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None);
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, None);
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            self.polygons = self
                .polygons
                .par_iter_mut()
                .flat_map(|poly| {
                    let sub_tris = poly.subdivide_triangles(levels);
                    // Convert each small tri back to a Polygon
                    sub_tris
                        .into_par_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.polygons = self
                .polygons
                .iter()
                .flat_map(|poly| {
                    let polytri = poly.subdivide_triangles(levels);
                    polytri
                        .into_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    pub fn dihedral_angle(p1: &Polygon<S>, p2: &Polygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }

    /// Extracts vertices and indices from the Mesh's tessellated polygons.
    pub fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        let tri_csg = self.triangulate();
        let vertices = tri_csg
            .polygons
            .iter()
            .flat_map(|p| [p.vertices[0].pos, p.vertices[1].pos, p.vertices[2].pos])
            .collect();

        let indices = (0..tri_csg.polygons.len())
            .map(|i| {
                let offset = i as u32 * 3;
                [offset, offset + 1, offset + 2]
            })
            .collect();

        (vertices, indices)
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        let mut hits: Vec<_> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .filter_map(|tri| {
                let a = tri[0].pos;
                let b = tri[1].pos;
                let c = tri[2].pos;
                let triangle = Triangle::new(a, b, c);
                triangle
                    .cast_ray_and_get_normal(&iso, &ray, Real::MAX, true)
                    .map(|hit| {
                        let point_on_ray = ray.point_at(hit.time_of_impact);
                        (Point3::from(point_on_ray.coords), hit.time_of_impact)
                    })
            })
            .collect();

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < tolerance());

        hits
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.\
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> SharedShape {
        let (vertices, indices) = self.get_vertices_and_indices();
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`.\
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Uses Parry to check if a point is inside a `Mesh`'s as a `TriMesh`.\
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    /// # use nalgebra::Point3;
    /// # use nalgebra::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, None);
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3<Real>) -> bool {
        self.ray_intersections(point, &Vector3::new(1.0, 1.0, 1.0))
            .len()
            % 2
            == 1
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> (Real, Point3<Real>, Unit<Quaternion<Real>>) {
        let trimesh = self.to_trimesh().unwrap();
        let mp = trimesh.mass_properties(density);

        (
            mp.mass(),
            mp.local_com,                     // a Point3<Real>
            mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
        )
    }

    /// Create a Rapier rigid body + collider from this Mesh, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<Real>,
        rotation: Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
    ) -> RigidBodyHandle {
        let shape = self.to_rapier_shape();

        // Build a Rapier RigidBody
        let rb = RigidBodyBuilder::dynamic()
            .translation(translation)
            // Now `rotation(...)` expects an axis-angle Vector3.
            .rotation(rotation)
            .build();
        let rb_handle = rb_set.insert(rb);

        // Build the collider
        let coll = ColliderBuilder::new(shape).density(density).build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);

        rb_handle
    }

    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh, PrimitiveTopology};

        let triangulated_mesh = &self.triangulate();
        let polygons = &triangulated_mesh.polygons;

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::with_capacity(polygons.len() * 3);

        let mut index_start = 0u32;

        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }

            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }

            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for Mesh<S> {
    /// Returns a new empty Mesh
    fn new() -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Return a new Mesh representing union of the two Meshes.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        final_polys.extend(b_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        // propagate self.metadata to new polygons by overwriting intersecting
        // polygon.metadata in other.
        let b_clip_retagged: Vec<Polygon<S>> = b_clip
            .iter()
            .map(|poly| {
                let mut p = poly.clone();
                p.metadata = self.metadata.clone();
                p
            })
            .collect();

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip_retagged);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Mesh<S>) -> Mesh<S> {
        let mut a = Node::from_polygons(&self.polygons);
        let mut b = Node::from_polygons(&other.polygons);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        Mesh {
            polygons: a.all_polygons(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Mesh<S>) -> Mesh<S> {
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    fn transform(&self, mat: &Matrix4<Real>) -> Mesh<S> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.try_inverse() {
            Some(inv) => inv.transpose(),
            None => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };

        let mut mesh = self.clone();

        for poly in &mut mesh.polygons {
            for vert in &mut poly.vertices {
                // Transform position using homogeneous coordinates
                let hom_pos = mat * vert.pos.to_homogeneous();
                match Point3::from_homogeneous(hom_pos) {
                    Some(transformed_pos) => vert.pos = transformed_pos,
                    None => {
                        eprintln!(
                            "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                        );
                        continue;
                    },
                }

                // Transform normal using inverse transpose rule
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }

            // Reconstruct plane from transformed vertices for consistency
            poly.plane = Plane::from_vertices(poly.vertices.clone());

            // Invalidate the polygon's bounding box
            poly.bounding_box = OnceLock::new();
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        mesh
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                    min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                    min_z = *partial_min(&min_z, &v.pos.z).unwrap();

                    max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                    max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                    max_z = *partial_max(&max_z, &v.pos.z).unwrap();
                }
            }

            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<S> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> From<Sketch<S>> for Mesh<S> {
    /// Convert a Sketch into a Mesh.
    fn from(sketch: Sketch<S>) -> Self {
        /// Helper function to convert a geo::Polygon to a Vec<crate::mesh::polygon::Polygon>
        fn geo_poly_to_csg_polys<S: Clone + Debug + Send + Sync>(
            poly2d: &GeoPolygon<Real>,
            metadata: &Option<S>,
        ) -> Vec<Polygon<S>> {
            let mut all_polygons = Vec::new();

            // Handle the exterior ring
            let outer_vertices_3d: Vec<_> = poly2d
                .exterior()
                .coords_iter()
                .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                .collect();

            if outer_vertices_3d.len() >= 3 {
                all_polygons.push(Polygon::new(outer_vertices_3d, metadata.clone()));
            }

            // Handle interior rings (holes)
            for ring in poly2d.interiors() {
                let hole_vertices_3d: Vec<_> = ring
                    .coords_iter()
                    .map(|c| Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()))
                    .collect();
                if hole_vertices_3d.len() >= 3 {
                    all_polygons.push(Polygon::new(hole_vertices_3d, metadata.clone()));
                }
            }
            all_polygons
        }

        let final_polygons = sketch
            .geometry
            .iter()
            .flat_map(|geom| -> Vec<Polygon<S>> {
                match geom {
                    Geometry::Polygon(poly2d) => {
                        geo_poly_to_csg_polys(poly2d, &sketch.metadata)
                    },
                    Geometry::MultiPolygon(multipoly) => multipoly
                        .iter()
                        .flat_map(|poly2d| geo_poly_to_csg_polys(poly2d, &sketch.metadata))
                        .collect(),
                    _ => vec![],
                }
            })
            .collect();

        Mesh {
            polygons: final_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<BMesh<S>> for Mesh<S> {
    fn from(bmesh: BMesh<S>) -> Self {
        // Empty BMesh -> empty Mesh
        let Some(manifold) = bmesh.manifold else {
            return Mesh::<S>::new();
        };

        // Convert boolmesh vertices (glam) into nalgebra points
        let mut points: Vec<Point3<Real>> = Vec::with_capacity(manifold.ps.len());
        for p in &manifold.ps {
            points.push(Point3::new(p.x as Real, p.y as Real, p.z as Real));
        }

        // Each 3 half-edges in hs correspond to 1 triangle face
        let mut polygons: Vec<Polygon<S>> = Vec::with_capacity(manifold.hs.len() / 3);

        for halfs in manifold.hs.chunks_exact(3) {
            let i0 = halfs[0].tail;
            let i1 = halfs[1].tail;
            let i2 = halfs[2].tail;

            // Safeguard against invalid indices
            if i0 >= points.len() || i1 >= points.len() || i2 >= points.len() {
                continue;
            }

            let p0 = points[i0];
            let p1 = points[i1];
            let p2 = points[i2];

            // Compute triangle normal
            let e1: Vector3<Real> = p1 - p0;
            let e2: Vector3<Real> = p2 - p0;
            let mut n = e1.cross(&e2);
            if let Some(unit) = n.try_normalize(1e-12) {
                n = unit;
            }

            let v0 = Vertex::new(p0, n);
            let v1 = Vertex::new(p1, n);
            let v2 = Vertex::new(p2, n);

            polygons.push(Polygon::new(vec![v0, v1, v2], bmesh.metadata.clone()));
        }

        Mesh::from_polygons(&polygons, bmesh.metadata.clone())
    }
}
