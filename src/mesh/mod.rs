//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::errors::ValidationError;
use crate::float_types::{
    hpoints_within_epsilon, hreal_gt_f64, hreal_lt_f64, hreal_to_f64, hvector3_from_point3,
    parry3d::{bounding_volume::Aabb, query::RayCast, shape::Shape},
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape, TriMesh, Triangle,
    },
    {Real, tolerance},
};

#[cfg(feature = "mesh-bbopt")]
use crate::float_types::parry3d::bounding_volume::BoundingVolume;

use crate::mesh::{bsp::Node, plane::Plane};
use crate::polygon::Polygon;
use crate::vertex::Vertex;

#[cfg(feature = "sketch")]
use crate::sketch::Sketch;

#[cfg(feature = "bmesh")]
use crate::bmesh::BMesh;
use crate::csg::CSG;
use hashbrown::HashMap;
#[cfg(feature = "sketch")]
use hypercurve::{Classification, FiniteProjectionOptions};
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3, partial_max, partial_min,
};
use std::{
    cmp::{Ordering, PartialEq},
    fmt::Debug,
    num::NonZeroU32,
    sync::OnceLock,
};

#[cfg(feature = "parallel")]
use rayon::{iter::IntoParallelRefIterator, prelude::*};

pub mod bsp;

#[cfg(feature = "parallel")]
pub mod bsp_parallel;

#[cfg(feature = "chull")]
pub mod convex_hull;
pub mod flatten_slice;

pub mod connectivity;
pub mod manifold;
#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub mod quality;
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod triangulated;

/// Stored as `(position: [f32; 3], normal: [f32; 3])`.
pub type GraphicsMeshVertex = ([f32; 3], [f32; 3]);

/// Mesh data laid out for renderers that want packed f32 vertices plus u32 indices.
#[derive(Debug, Clone)]
pub struct GraphicsMesh {
    pub vertices: Vec<GraphicsMeshVertex>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct Mesh<M: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<M>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Lazily built Parry TriMesh reused by query operations.
    pub query_trimesh: OnceLock<Option<TriMesh>>,

    /// Whole-mesh metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub metadata: M,
}

fn hyper_edge_projection_parameter(
    point: &Point3<Real>,
    edge_start: &Point3<Real>,
    edge_end: &Point3<Real>,
    epsilon: Real,
) -> Option<Real> {
    let a = hvector3_from_point3(edge_start)?;
    let b = hvector3_from_point3(edge_end)?;
    let p = hvector3_from_point3(point)?;
    let ab = &b - &a;
    let av = &p - &a;
    let bv = &p - &b;

    let eps2 = epsilon * epsilon;
    let ab_len_sq = ab.dot(&ab);
    if !hreal_gt_f64(&ab_len_sq, eps2) {
        return None;
    }

    if !hreal_gt_f64(&av.dot(&av), eps2) || !hreal_gt_f64(&bv.dot(&bv), eps2) {
        return None;
    }

    let t_h = (ab.dot(&av) / ab_len_sq).ok()?;
    if !hreal_gt_f64(&t_h, epsilon) || !hreal_lt_f64(&t_h, 1.0 - epsilon) {
        return None;
    }

    let projected = a + ab * &t_h;
    let delta = p - projected;
    if hreal_gt_f64(&delta.dot(&delta), eps2) {
        return None;
    }

    hreal_to_f64(&t_h)
}

fn hyperphysics_vector3_from_point3(
    point: &Point3<Real>,
) -> Result<HyperVector3, ValidationError> {
    Ok(HyperVector3::new([
        hyperphysics::Real::try_from(point.x)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(*point)))?,
        hyperphysics::Real::try_from(point.y)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(*point)))?,
        hyperphysics::Real::try_from(point.z)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(*point)))?,
    ]))
}

impl<M: Clone + Send + Sync + Debug + PartialEq> Mesh<M> {
    /// Compare just the `metadata` fields of two meshes
    #[inline]
    pub fn same_metadata(&self, other: &Self) -> bool {
        self.metadata == other.metadata
    }

    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &M) -> Mesh<M> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| &p.metadata == needle)
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
            query_trimesh: std::sync::OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Return a new empty mesh with explicit metadata.
    pub fn empty(metadata: M) -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata,
        }
    }

    /// Build a Mesh from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<M>], metadata: M) -> Self {
        Mesh {
            polygons: polygons.to_vec(),
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata,
        }
    }

    /// Return this mesh with replacement metadata on the mesh and every polygon.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Mesh<NewM> {
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.with_metadata(metadata.clone()))
            .collect();

        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata,
        }
    }

    /// Map metadata on the mesh and every polygon while preserving geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, mut f: F) -> Mesh<NewM>
    where
        F: FnMut(M) -> NewM,
    {
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.map_metadata(&mut f))
            .collect();
        let metadata = f(self.metadata);

        Mesh {
            polygons,
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata,
        }
    }

    /// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
    #[cfg(feature = "mesh-bbopt")]
    fn partition_polys(
        polys: &[Polygon<M>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<M>>, Vec<Polygon<M>>) {
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

    /// Pre-pass to remove T-junctions between polygons by inserting missing
    /// vertices on shared or colinear overlapping edges before triangulation.
    ///
    /// Edge projection and point-edge distance checks are evaluated in
    /// `hyperreal::Real` through `hyperlattice::Vector3`; only the accepted
    /// insertion parameter is exported back to `f64` so existing mesh vertices
    /// can still be stored at the public API boundary. Retaining the point and
    /// edge as hyper geometry objects before scalar fallback follows Yap's
    /// exact-geometric-computation model, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The repair addresses
    /// T-vertices before triangulation because mesh cracks violate the
    /// watertightness assumptions behind triangle mesh processing; see
    /// Botsch et al., *Polygon Mesh Processing*, 2010
    /// (<https://doi.org/10.1201/b10688>).
    fn fix_t_junctions_on_shared_edges(polygons: &mut [Polygon<M>]) {
        let eps = tolerance();
        let eps2 = eps * eps;

        if polygons.len() < 2 {
            return;
        }

        let poly_count = polygons.len();
        let mut edge_splits: Vec<HashMap<usize, Vec<(Real, Vertex)>>> =
            vec![HashMap::new(); poly_count];

        for (i, poly_i) in polygons.iter().enumerate() {
            if poly_i.vertices.len() < 2 {
                continue;
            }

            for vert in &poly_i.vertices {
                for (j, poly_j) in polygons.iter().enumerate() {
                    if i == j || poly_j.vertices.len() < 2 {
                        continue;
                    }

                    let verts_j = &poly_j.vertices;
                    let n_j = verts_j.len();
                    for edge_start in 0..n_j {
                        let a = &verts_j[edge_start];
                        let b = &verts_j[(edge_start + 1) % n_j];
                        let Some(t) = hyper_edge_projection_parameter(
                            &vert.position,
                            &a.position,
                            &b.position,
                            eps,
                        ) else {
                            continue;
                        };

                        let new_vertex = Vertex::new(vert.position, poly_j.plane.normal());
                        let entry = edge_splits[j].entry(edge_start).or_default();
                        let already_present = entry.iter().any(|(existing_t, existing_v)| {
                            (existing_t - t).abs() < eps
                                || (existing_v.position - new_vertex.position).norm_squared()
                                    < eps2
                        });

                        if !already_present {
                            entry.push((t, new_vertex));
                        }
                    }
                }
            }
        }

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

            let extra_vertices: usize = splits_map.values().map(Vec::len).sum();
            let mut new_vertices = Vec::with_capacity(n + extra_vertices);

            for edge_start in 0..n {
                new_vertices.push(original[edge_start]);

                if let Some(splits) = splits_map.get(&edge_start) {
                    let mut splits_sorted = splits.clone();
                    splits_sorted.sort_by(|(t_a, _), (t_b, _)| {
                        t_a.partial_cmp(t_b).unwrap_or(Ordering::Equal)
                    });

                    for (_, vertex) in splits_sorted {
                        new_vertices.push(vertex);
                    }
                }
            }

            poly.vertices = new_vertices;
        }
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<M> {
        let mut polygons = self.polygons.clone();
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
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<M> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<M>> = self
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
        let new_polygons: Vec<Polygon<M>> = self
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
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
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
    pub fn dihedral_angle(p1: &Polygon<M>, p2: &Polygon<M>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }

    /// Converts this mesh into f32 vertex/index buffers suitable for rendering.
    ///
    /// Vertices are deduplicated by exact f32 position and normal bit pattern.
    pub fn build_graphics_mesh(&self) -> GraphicsMesh {
        let triangles = self.triangulate().polygons;
        let triangle_count = triangles.len();

        let mut indices = Vec::with_capacity(triangle_count * 3);
        let mut vertices = Vec::with_capacity(triangle_count * 3);
        let mut vertices_hash: HashMap<([u32; 3], [u32; 3]), u32> =
            HashMap::with_capacity(triangle_count * 3);

        for triangle in triangles {
            for vertex in triangle.vertices {
                let position = [
                    vertex.position.x as f32,
                    vertex.position.y as f32,
                    vertex.position.z as f32,
                ];
                let normal = [
                    vertex.normal.x as f32,
                    vertex.normal.y as f32,
                    vertex.normal.z as f32,
                ];
                let key = (position.map(f32::to_bits), normal.map(f32::to_bits));

                let index = *vertices_hash.entry(key).or_insert_with(|| {
                    let new_index = vertices.len() as u32;
                    vertices.push((position, normal));
                    new_index
                });
                indices.push(index);
            }
        }

        vertices.shrink_to_fit();
        GraphicsMesh { vertices, indices }
    }

    /// Extracts vertices and indices from the Mesh's tessellated polygons.
    ///
    /// This intentionally does not remove duplicate vertices.
    pub fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        let tri_csg = self.triangulate();
        let vertices = tri_csg
            .polygons
            .iter()
            .flat_map(|p| {
                [
                    p.vertices[0].position,
                    p.vertices[1].position,
                    p.vertices[2].position,
                ]
            })
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
                let a = tri[0].position;
                let b = tri[1].position;
                let c = tri[2].position;
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

    /// Find all intersection points between a polyline and this mesh's
    /// triangulated surface.
    ///
    /// Each consecutive pair of points defines one segment. Hits are deduplicated
    /// locally and returned in polyline order. Deduplication compares squared
    /// hit-point distance through `hyperlattice::Vector3` and
    /// `hyperreal::Real`, keeping this topology-affecting equality decision out
    /// of local f64 square-root arithmetic. This follows Yap's
    /// exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn intersect_polyline(&self, polyline: &[Point3<Real>]) -> Vec<Point3<Real>> {
        if polyline.len() < 2 {
            return Vec::new();
        }

        let iso = Isometry3::identity();
        let tol = tolerance();
        let triangles: Vec<[Vertex; 3]> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .collect();

        let mut hits: Vec<Point3<Real>> = Vec::new();

        for seg in polyline.windows(2) {
            let seg_start = seg[0];
            let seg_end = seg[1];
            let seg_dir = seg_end - seg_start;
            let seg_len = seg_dir.norm();
            if seg_len < tol {
                continue;
            }

            let ray = Ray::new(seg_start, seg_dir / seg_len);
            let mut seg_hits: Vec<(Point3<Real>, Real)> = Vec::new();

            for tri in &triangles {
                let triangle =
                    Triangle::new(tri[0].position, tri[1].position, tri[2].position);
                if let Some(hit) =
                    triangle.cast_ray_and_get_normal(&iso, &ray, seg_len + tol, true)
                {
                    let t = hit.time_of_impact;
                    if t >= -tol && t <= seg_len + tol {
                        seg_hits.push((Point3::from(ray.point_at(t).coords), t));
                    }
                }
            }

            seg_hits
                .sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

            for (point, _) in seg_hits {
                if let Some(last) = hits.last() {
                    if hpoints_within_epsilon(&point, last, tol) {
                        continue;
                    }
                }
                hits.push(point);
            }
        }

        hits
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.\
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> Result<SharedShape, ValidationError> {
        let (vertices, indices) = self.get_vertices_and_indices();
        let trimesh = TriMesh::new(vertices, indices)
            .map_err(|err| ValidationError::TriMeshError(format!("{err:?}")))?;
        Ok(SharedShape::new(trimesh))
    }

    /// Convert the polygons in this Mesh to a Parry `TriMesh`.\
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Result<TriMesh, ValidationError> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices)
            .map_err(|err| ValidationError::TriMeshError(format!("{err:?}")))
    }

    fn cached_trimesh(&self) -> Result<&TriMesh, ValidationError> {
        self.query_trimesh
            .get_or_init(|| {
                let (vertices, indices) = self.get_vertices_and_indices();
                TriMesh::new(vertices, indices).ok()
            })
            .as_ref()
            .ok_or_else(|| {
                ValidationError::TriMeshError("failed to build triangle mesh".into())
            })
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
    /// let csg_cube = Mesh::<()>::cube(6.0, ());
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
    ) -> Result<(Real, Point3<Real>, Unit<Quaternion<Real>>), ValidationError> {
        let trimesh = self.cached_trimesh()?;
        let mp = trimesh.mass_properties(density);

        Ok((
            mp.mass(),
            mp.local_com,                     // a Point3<Real>
            mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
        ))
    }

    /// Exact uniform-density mass properties through `hyperphysics`.
    ///
    /// This is the Hyper-native counterpart to [`Mesh::mass_properties`]. Mesh
    /// coordinates are finite `csgrs` adapter scalars at this boundary; each
    /// coordinate and the density are lifted into [`hyperphysics::Real`] before
    /// volume, center of mass, and inertia are accumulated. The report-bearing
    /// path follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) by keeping the lossy
    /// `f64` CSG/engine surface explicit and moving physical integrals into the
    /// exact object layer before any Rapier/Parry adapter is used.
    pub fn exact_mass_properties(
        &self,
        density: Real,
    ) -> Result<HyperMassPropertyReport3, ValidationError> {
        let density = hyperphysics::Real::try_from(density)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))?;
        let mesh = self.to_hyperphysics_closed_triangle_mesh()?;
        mesh.uniform_density_mass_properties(density)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))
    }

    /// Converts triangulated mesh polygons into a `hyperphysics` closed mesh
    /// carrier without going through Parry or Rapier.
    pub fn to_hyperphysics_closed_triangle_mesh(
        &self,
    ) -> Result<HyperClosedTriangleMesh3, ValidationError> {
        let tri_mesh = self.triangulate();
        let triangles = tri_mesh
            .polygons
            .iter()
            .map(|polygon| {
                if polygon.vertices.len() != 3 {
                    return Err(ValidationError::TooFewPoints(Point3::origin()));
                }
                Ok(HyperTriangle3::new([
                    hyperphysics_vector3_from_point3(&polygon.vertices[0].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[1].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[2].position)?,
                ]))
            })
            .collect::<Result<Vec<_>, _>>()?;
        HyperClosedTriangleMesh3::new(triangles)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))
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
    ) -> Result<RigidBodyHandle, ValidationError> {
        let shape = self.to_rapier_shape()?;

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

        Ok(rb_handle)
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
                positions_32.push([
                    v.position.x as f32,
                    v.position.y as f32,
                    v.position.z as f32,
                ]);
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

impl Mesh<()> {
    /// Return a new empty mesh with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Mesh<M> {
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
    fn union(&self, other: &Mesh<M>) -> Mesh<M> {
        // avoid splitting obvious non‑intersecting faces
        #[cfg(feature = "mesh-bbopt")]
        let final_polys = {
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
            final_polys
        };

        #[cfg(not(feature = "mesh-bbopt"))]
        let final_polys = {
            let mut a = Node::from_polygons(&self.polygons);
            let mut b = Node::from_polygons(&other.polygons);

            a.clip_to(&b);
            b.clip_to(&a);
            b.invert();
            b.clip_to(&a);
            b.invert();
            a.build(&b.all_polygons());

            a.all_polygons()
        };

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
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
    fn difference(&self, other: &Mesh<M>) -> Mesh<M> {
        // avoid splitting obvious non‑intersecting faces
        #[cfg(feature = "mesh-bbopt")]
        let final_polys = {
            let (a_clip, a_passthru) =
                Self::partition_polys(&self.polygons, &other.bounding_box());
            let (b_clip, _b_passthru) =
                Self::partition_polys(&other.polygons, &self.bounding_box());

            let mut a = Node::from_polygons(&a_clip);
            let mut b = Node::from_polygons(&b_clip);

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
            final_polys
        };

        #[cfg(not(feature = "mesh-bbopt"))]
        let final_polys = {
            let mut a = Node::from_polygons(&self.polygons);
            let mut b = Node::from_polygons(&other.polygons);

            a.invert();
            a.clip_to(&b);
            b.clip_to(&a);
            b.invert();
            b.clip_to(&a);
            b.invert();
            a.build(&b.all_polygons());
            a.invert();

            a.all_polygons()
        };

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
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
    fn intersection(&self, other: &Mesh<M>) -> Mesh<M> {
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
            query_trimesh: OnceLock::new(),
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
    fn xor(&self, other: &Mesh<M>) -> Mesh<M> {
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
    fn transform(&self, mat: &Matrix4<Real>) -> Mesh<M> {
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
                let hom_position = mat * vert.position.to_homogeneous();
                match Point3::from_homogeneous(hom_position) {
                    Some(transformed_position) => vert.position = transformed_position,
                    None => {
                        eprintln!(
                            "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                        );
                        continue;
                    },
                }

                // Transform normal using inverse transpose rule
                let transformed_normal = mat_inv_transpose.transform_vector(&vert.normal);
                if transformed_normal.iter().all(|coord| coord.is_finite()) {
                    if let Some(normal) = transformed_normal.try_normalize(tolerance()) {
                        vert.normal = normal;
                    }
                }
            }

            // Reconstruct plane from transformed vertices for consistency
            poly.plane = Plane::from_vertices(poly.vertices.clone());

            // Invalidate the polygon's bounding box
            poly.bounding_box = OnceLock::new();
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();
        mesh.query_trimesh = OnceLock::new();

        mesh
    }

    /// Returns an axis-aligned bounding box indicating the 3D bounds of all
    /// `polygons`.
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
                    min_x = *partial_min(&min_x, &v.position.x).unwrap();
                    min_y = *partial_min(&min_y, &v.position.y).unwrap();
                    min_z = *partial_min(&min_z, &v.position.z).unwrap();

                    max_x = *partial_max(&max_x, &v.position.x).unwrap();
                    max_y = *partial_max(&max_y, &v.position.y).unwrap();
                    max_z = *partial_max(&max_z, &v.position.z).unwrap();
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
        self.query_trimesh = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<M> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Send + Sync + Debug> From<Sketch<M>> for Mesh<M> {
    /// Convert a Sketch into a Mesh.
    ///
    /// Closed area sketches are consumed only from hypercurve-owned finite
    /// [`hypercurve::FiniteRegionProfile2`] projections. The temporary `geo`
    /// compatibility cache is deliberately ignored here: once a Sketch has been
    /// imported, `Region2`/`CurveString2` are the CAD source of truth. The
    /// grouping follows the
    /// point-in-polygon ownership structure surveyed by Hormann and Agathos,
    /// "The point in polygon problem for arbitrary polygons," *Computational
    /// Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>). This keeps the
    /// conversion aligned with the exact-geometric-computation boundary
    /// discipline from Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>): topology lives in
    /// hyper geometry, while mesh vertices are the finite API-boundary
    /// realization.
    fn from(sketch: Sketch<M>) -> Self {
        fn ring_to_vertices(ring: &[[Real; 2]]) -> Vec<Vertex> {
            let mut vertices: Vec<_> = ring
                .iter()
                .map(|p| Vertex::new(Point3::new(p[0], p[1], 0.0), Vector3::z()))
                .collect();
            if vertices.first() == vertices.last() {
                vertices.pop();
            }
            vertices
        }

        fn ring_to_polygon<M: Clone + Send + Sync>(
            ring: &[[Real; 2]],
            metadata: &M,
        ) -> Option<Polygon<M>> {
            let vertices = ring_to_vertices(ring);
            (vertices.len() >= 3).then(|| Polygon::new(vertices, metadata.clone()))
        }

        let projection_options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let region_profiles = match sketch.project_region_profiles(&projection_options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        if !region_profiles.is_empty() {
            let final_polygons = region_profiles
                .iter()
                .flat_map(|profile| {
                    std::iter::once(profile.material().points())
                        .chain(profile.holes().iter().map(|hole| hole.points()))
                })
                .filter_map(|ring| ring_to_polygon(ring, &sketch.metadata))
                .collect();

            return Mesh {
                polygons: final_polygons,
                bounding_box: OnceLock::new(),
                query_trimesh: OnceLock::new(),
                metadata: sketch.metadata.clone(),
            };
        }

        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata: sketch.metadata.clone(),
        }
    }
}

#[cfg(feature = "bmesh")]
impl<M: Clone + Send + Sync + Debug> From<BMesh<M>> for Mesh<M> {
    fn from(bmesh: BMesh<M>) -> Self {
        // Empty BMesh -> empty Mesh
        let Some(manifold) = bmesh.manifold else {
            return Mesh::empty(bmesh.metadata);
        };

        // Convert boolmesh vertices (glam) into nalgebra points
        let mut points: Vec<Point3<Real>> = Vec::with_capacity(manifold.ps.len());
        for p in &manifold.ps {
            points.push(Point3::new(p.x as Real, p.y as Real, p.z as Real));
        }

        // Each 3 half-edges in hs correspond to 1 triangle face
        let mut polygons: Vec<Polygon<M>> = Vec::with_capacity(manifold.hs.len() / 3);

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hyper_edge_projection_parameter_accepts_exact_t_vertex() {
        let point = Point3::new(1.0, 0.0, 0.0);
        let edge_start = Point3::new(0.0, 0.0, 0.0);
        let edge_end = Point3::new(2.0, 0.0, 0.0);

        let t = hyper_edge_projection_parameter(&point, &edge_start, &edge_end, tolerance())
            .expect("point lies strictly inside the edge");

        assert!((t - 0.5).abs() < tolerance());
    }

    #[test]
    fn hyper_edge_projection_parameter_rejects_off_edge_point() {
        let point = Point3::new(1.0, tolerance() * 4.0, 0.0);
        let edge_start = Point3::new(0.0, 0.0, 0.0);
        let edge_end = Point3::new(2.0, 0.0, 0.0);

        assert!(
            hyper_edge_projection_parameter(&point, &edge_start, &edge_end, tolerance())
                .is_none()
        );
    }
}
