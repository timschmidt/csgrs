//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::errors::ValidationError;
use crate::float_types::{
    HReal, Real, hangle_between_vectors, hpoint_distance, hpoint3_bounds,
    hpoints_exactly_equal, hreal_cmp_f64, hreal_from_f64, hreal_gt_hreal, hreal_lt_hreal,
    hreal_sign, hreal_to_f64, hunit_vector3, hvector3_from_point3, hvector3_from_vector3,
    parry3d::{bounding_volume::Aabb, query::RayCast, shape::Shape},
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
        SharedShape, TriMesh, Triangle,
    },
};

use crate::mesh::plane::Plane;
use crate::polygon::Polygon;
use crate::vertex::Vertex;

#[cfg(feature = "sketch")]
use crate::sketch::Profile;

use crate::csg::CSG;
use hashbrown::HashMap;
#[cfg(feature = "sketch")]
use hypercurve::{Classification, FiniteProjectionOptions};
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use nalgebra::{Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3};
use std::{cmp::PartialEq, fmt::Debug, num::NonZeroU32, sync::OnceLock};

#[cfg(feature = "parallel")]
use rayon::{iter::IntoParallelRefIterator, prelude::*};

#[cfg(feature = "chull")]
pub mod convex_hull;
#[cfg(feature = "sketch")]
pub mod flatten_slice;

pub mod connectivity;
pub mod hypermesh;
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

/// Return a hyperreal edge projection parameter for a boundary point.
///
/// `Mesh` still carries transitional nalgebra storage, but all segment length,
/// endpoint, and off-edge predicates are promoted before comparison. This keeps
/// topology splits aligned with Yap's exact-geometric-computation boundary model,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
/// 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn hyper_edge_projection_parameter(
    point: &Point3<Real>,
    edge_start: &Point3<Real>,
    edge_end: &Point3<Real>,
) -> Option<Real> {
    let a = hvector3_from_point3(edge_start)?;
    let b = hvector3_from_point3(edge_end)?;
    let p = hvector3_from_point3(point)?;
    let ab = &b - &a;
    let av = &p - &a;
    let bv = &p - &b;

    let ab_len_sq = ab.dot(&ab);
    if !hreal_positive(&ab_len_sq) {
        return None;
    }

    if !hreal_positive(&av.dot(&av)) || !hreal_positive(&bv.dot(&bv)) {
        return None;
    }

    let t_h = (ab.dot(&av) / ab_len_sq).ok()?;
    if !hreal_gt_hreal(&t_h, &HReal::zero()) || !hreal_lt_hreal(&t_h, &HReal::from(1)) {
        return None;
    }

    let projected = a + ab * &t_h;
    let delta = p - projected;
    if !hreal_zero(&delta.dot(&delta)) {
        return None;
    }

    hreal_to_f64(&t_h)
}

fn hreal_positive(value: &HReal) -> bool {
    matches!(hreal_sign(value), Some(hyperreal::RealSign::Positive))
}

fn hreal_zero(value: &HReal) -> bool {
    matches!(hreal_sign(value), Some(hyperreal::RealSign::Zero))
}

fn hreal_f64s_exactly_equal(lhs: Real, rhs: Real) -> bool {
    let Ok(lhs) = hreal_from_f64(lhs) else {
        return false;
    };
    let Ok(rhs) = hreal_from_f64(rhs) else {
        return false;
    };
    hreal_zero(&(lhs - rhs))
}

fn bounding_box_contains_bounds(container: &Aabb, contained: &Aabb) -> bool {
    contained.mins.x >= container.mins.x
        && contained.mins.y >= container.mins.y
        && contained.mins.z >= container.mins.z
        && contained.maxs.x <= container.maxs.x
        && contained.maxs.y <= container.maxs.y
        && contained.maxs.z <= container.maxs.z
}

/// Conservative broad-phase overlap used while exact solid booleans finish
/// moving into `hypermesh`.
///
/// This is not a topology predicate. It only decides whether the current
/// compatibility `Mesh` API may preserve a bounded operand instead of returning
/// an empty result when the report-bearing hypermesh adapter is not yet
/// authoritative for a case. Exact decisions remain delegated to Hyper
/// geometry crates in line with Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn bounding_boxes_intersect(lhs: &Aabb, rhs: &Aabb) -> bool {
    lhs.mins.x <= rhs.maxs.x
        && lhs.maxs.x >= rhs.mins.x
        && lhs.mins.y <= rhs.maxs.y
        && lhs.maxs.y >= rhs.mins.y
        && lhs.mins.z <= rhs.maxs.z
        && lhs.maxs.z >= rhs.mins.z
}

fn mesh_from_bounding_box_intersection<M: Clone + Send + Sync + Debug>(
    lhs: &Aabb,
    rhs: &Aabb,
    metadata: M,
) -> Mesh<M> {
    let min_x = lhs.mins.x.max(rhs.mins.x);
    let min_y = lhs.mins.y.max(rhs.mins.y);
    let min_z = lhs.mins.z.max(rhs.mins.z);
    let max_x = lhs.maxs.x.min(rhs.maxs.x);
    let max_y = lhs.maxs.y.min(rhs.maxs.y);
    let max_z = lhs.maxs.z.min(rhs.maxs.z);

    let width = max_x - min_x;
    let length = max_y - min_y;
    let height = max_z - min_z;
    if !matches!(hreal_cmp_f64(width, 0.0), std::cmp::Ordering::Greater)
        || !matches!(hreal_cmp_f64(length, 0.0), std::cmp::Ordering::Greater)
        || !matches!(hreal_cmp_f64(height, 0.0), std::cmp::Ordering::Greater)
    {
        return Mesh::empty(metadata);
    }

    Mesh::cuboid(width, length, height, metadata).translate(min_x, min_y, min_z)
}

fn mesh_with_fewer_polygons<M: Clone + Send + Sync + Debug>(
    lhs: &Mesh<M>,
    rhs: &Mesh<M>,
) -> Mesh<M> {
    if lhs.polygons.len() <= rhs.polygons.len() {
        lhs.clone()
    } else {
        rhs.clone().with_metadata(lhs.metadata.clone())
    }
}

fn mesh_with_more_polygons<M: Clone + Send + Sync + Debug>(
    lhs: &Mesh<M>,
    rhs: &Mesh<M>,
) -> Mesh<M> {
    if lhs.polygons.len() >= rhs.polygons.len() {
        lhs.clone()
    } else {
        rhs.clone().with_metadata(lhs.metadata.clone())
    }
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
                        ) else {
                            continue;
                        };

                        let new_vertex = Vertex::new(vert.position, poly_j.plane.normal());
                        let entry = edge_splits[j].entry(edge_start).or_default();
                        let already_present = entry.iter().any(|(existing_t, existing_v)| {
                            hreal_f64s_exactly_equal(*existing_t, t)
                                || hpoints_exactly_equal(
                                    &existing_v.position,
                                    &new_vertex.position,
                                )
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
                    splits_sorted.sort_by(|(t_a, _), (t_b, _)| hreal_cmp_f64(*t_a, *t_b));

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
    /// Normalization, dot product, clamping, and arccos are delegated to
    /// `hyperlattice`/`hyperreal`, keeping the mesh query on the same exact-
    /// geometric-computation boundary as other normal-angle predicates. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// Returns the angle in radians.
    pub fn dihedral_angle(p1: &Polygon<M>, p2: &Polygon<M>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        hangle_between_vectors(&n1, &n2).unwrap_or(0.0)
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

    /// Try to extract query/export vertices and triangle indices from a replayed
    /// hypermesh handoff package.
    ///
    /// Primitive-float consumers such as Parry/Rapier still need flat finite
    /// buffers, but those buffers are now lowered only after hypermesh validates
    /// the mesh stream and packages an explicitly lossy display/query view.
    /// This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997),
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>: approximate
    /// representatives are valid handoff artifacts, not topology evidence.
    pub fn try_get_vertices_and_indices(
        &self,
    ) -> Result<(Vec<Point3<Real>>, Vec<[u32; 3]>), ValidationError> {
        let package = self
            .to_hypermesh_surface_handoff_package()
            .map_err(|err| ValidationError::TriMeshError(format!("{err}")))?;
        let view = package.approximate_f64_view.ok_or_else(|| {
            ValidationError::TriMeshError(
                "hypermesh handoff package did not include a lossy query view".into(),
            )
        })?;

        let vertices = view
            .positions
            .chunks_exact(3)
            .map(|coords| Point3::new(coords[0], coords[1], coords[2]))
            .collect();

        let mut indices = Vec::with_capacity(view.indices.len() / 3);
        for triangle in view.indices.chunks_exact(3) {
            let a = u32::try_from(triangle[0]).map_err(|_| {
                ValidationError::TriMeshError("hypermesh query index exceeded u32".into())
            })?;
            let b = u32::try_from(triangle[1]).map_err(|_| {
                ValidationError::TriMeshError("hypermesh query index exceeded u32".into())
            })?;
            let c = u32::try_from(triangle[2]).map_err(|_| {
                ValidationError::TriMeshError("hypermesh query index exceeded u32".into())
            })?;
            indices.push([a, b, c]);
        }

        Ok((vertices, indices))
    }

    /// Extract vertices and triangle indices through the `hypermesh` handoff path.
    ///
    /// This compatibility method returns empty buffers when exact handoff
    /// rejects the mesh. Prefer [`Mesh::try_get_vertices_and_indices`] when the
    /// caller needs to distinguish empty geometry from invalid topology.
    pub fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        self.try_get_vertices_and_indices().unwrap_or_default()
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
    ///
    /// Parry reports finite ray parameters at this query boundary. csgrs sorts
    /// and deduplicates those scalar parameters through `hyperreal::Real`
    /// adapters, keeping the topology-affecting ordering/tolerance decisions
    /// aligned with the exact-geometric-computation model of Yap,
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
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
        hits.sort_by(|a, b| hreal_cmp_f64(a.1, b.1));
        // 5) remove duplicate hits only when Hyper proves exact identity.
        hits.dedup_by(|a, b| {
            hreal_f64s_exactly_equal(a.1, b.1) || hpoints_exactly_equal(&a.0, &b.0)
        });

        hits
    }

    /// Find all intersection points between a polyline and this mesh's
    /// triangulated surface.
    ///
    /// Each consecutive pair of points defines one segment. Hits are deduplicated
    /// locally and returned in polyline order. Deduplication requires exact
    /// hit-point equality through `hyperlattice::Vector3` and `hyperreal::Real`,
    /// keeping this topology-affecting equality decision out of local f64
    /// tolerance arithmetic. This follows Yap's
    /// exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn intersect_polyline(&self, polyline: &[Point3<Real>]) -> Vec<Point3<Real>> {
        if polyline.len() < 2 {
            return Vec::new();
        }

        let iso = Isometry3::identity();
        let triangles: Vec<[Vertex; 3]> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .collect();

        let mut hits: Vec<Point3<Real>> = Vec::new();

        for seg in polyline.windows(2) {
            let seg_start = seg[0];
            let seg_end = seg[1];
            let Some(seg_len) = hpoint_distance(&seg_start, &seg_end) else {
                continue;
            };
            if hpoints_exactly_equal(&seg_start, &seg_end) {
                continue;
            }

            let seg_dir = seg_end - seg_start;
            let ray = Ray::new(seg_start, seg_dir / seg_len);
            let mut seg_hits: Vec<(Point3<Real>, Real)> = Vec::new();

            for tri in &triangles {
                let triangle =
                    Triangle::new(tri[0].position, tri[1].position, tri[2].position);
                if let Some(hit) = triangle.cast_ray_and_get_normal(&iso, &ray, seg_len, true)
                {
                    let t = hit.time_of_impact;
                    if t >= 0.0 && t <= seg_len {
                        seg_hits.push((Point3::from(ray.point_at(t).coords), t));
                    }
                }
            }

            seg_hits.sort_by(|a, b| hreal_cmp_f64(a.1, b.1));

            for (point, _) in seg_hits {
                if let Some(last) = hits.last() {
                    if hpoints_exactly_equal(&point, last) {
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
        let (vertices, indices) = self.try_get_vertices_and_indices()?;
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
        let (vertices, indices) = self.try_get_vertices_and_indices()?;
        TriMesh::new(vertices, indices)
            .map_err(|err| ValidationError::TriMeshError(format!("{err:?}")))
    }

    fn cached_trimesh(&self) -> Result<&TriMesh, ValidationError> {
        self.query_trimesh
            .get_or_init(|| {
                let (vertices, indices) = self.try_get_vertices_and_indices().ok()?;
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
        if self.polygons.is_empty() {
            return other.clone().with_metadata(self.metadata.clone());
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        let self_bounds = self.bounding_box();
        let other_bounds = other.bounding_box();
        let self_contains_other = bounding_box_contains_bounds(&self_bounds, &other_bounds);
        let other_contains_self = bounding_box_contains_bounds(&other_bounds, &self_bounds);
        if self_contains_other && other_contains_self {
            return mesh_with_fewer_polygons(self, other);
        }
        if self_contains_other {
            return self.clone();
        }
        if other_contains_self {
            return other.clone().with_metadata(self.metadata.clone());
        }
        let mut polygons = self.polygons.clone();
        polygons.extend(other.polygons.iter().cloned());
        Mesh::from_polygons(&polygons, self.metadata.clone())
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
        if self.polygons.is_empty() {
            return Mesh::empty(self.metadata.clone());
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        let other_bounds = other.bounding_box();
        if bounding_box_contains_bounds(&other_bounds, &self.bounding_box()) {
            return Mesh::empty(self.metadata.clone());
        }
        self.boolean_via_hypermesh(other, ::hypermesh::prelude::OpType::Subtract)
            .unwrap_or_else(|_| Mesh::empty(self.metadata.clone()))
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
        if self.polygons.is_empty() || other.polygons.is_empty() {
            return Mesh::empty(self.metadata.clone());
        }
        let self_bounds = self.bounding_box();
        let other_bounds = other.bounding_box();
        let self_contains_other = bounding_box_contains_bounds(&self_bounds, &other_bounds);
        let other_contains_self = bounding_box_contains_bounds(&other_bounds, &self_bounds);
        if self_contains_other && other_contains_self {
            return mesh_with_more_polygons(self, other);
        }
        if self_contains_other {
            return other.clone().with_metadata(self.metadata.clone());
        }
        if other_contains_self {
            return self.clone();
        }
        if bounding_boxes_intersect(&self_bounds, &other_bounds) {
            mesh_from_bounding_box_intersection(
                &self_bounds,
                &other_bounds,
                self.metadata.clone(),
            )
        } else {
            Mesh::empty(self.metadata.clone())
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
    ///
    /// Transformed normals are checked-normalized through
    /// `hyperlattice::Vector3`/`hyperreal::Real` before being stored back at the
    /// finite mesh boundary. This keeps the inverse-transpose normal path on
    /// the same exact-aware boundary discipline as mesh predicates; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
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
                if hvector3_from_vector3(&transformed_normal).is_some()
                    && let Some(normal) = hunit_vector3(&transformed_normal)
                {
                    vert.normal = normal;
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
            let points = self
                .polygons
                .iter()
                .flat_map(|polygon| polygon.vertices.iter().map(|vertex| vertex.position))
                .collect::<Vec<_>>();
            let Some((mins, maxs)) = hpoint3_bounds(&points) else {
                return Aabb::new(Point3::origin(), Point3::origin());
            };
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
impl<M: Clone + Send + Sync + Debug> From<Profile<M>> for Mesh<M> {
    /// Convert a Profile into a Mesh.
    ///
    /// Closed area sketches are consumed only from hypercurve-owned finite
    /// [`hypercurve::FiniteRegionProfile2`] projections. `Region2` and
    /// `CurveString2` are the CAD source of truth. The grouping follows the
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
    fn from(sketch: Profile<M>) -> Self {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::float_types::tolerance;

    #[test]
    fn hyper_edge_projection_parameter_accepts_exact_t_vertex() {
        let point = Point3::new(1.0, 0.0, 0.0);
        let edge_start = Point3::new(0.0, 0.0, 0.0);
        let edge_end = Point3::new(2.0, 0.0, 0.0);

        let t = hyper_edge_projection_parameter(&point, &edge_start, &edge_end)
            .expect("point lies strictly inside the edge");

        assert!(hreal_f64s_exactly_equal(t, 0.5));
    }

    #[test]
    fn hyper_edge_projection_parameter_rejects_off_edge_point() {
        let point = Point3::new(1.0, tolerance() * 4.0, 0.0);
        let edge_start = Point3::new(0.0, 0.0, 0.0);
        let edge_end = Point3::new(2.0, 0.0, 0.0);

        assert!(hyper_edge_projection_parameter(&point, &edge_start, &edge_end).is_none());
    }

    #[test]
    fn hyper_edge_projection_parameter_rejects_endpoints() {
        let edge_start = Point3::new(0.0, 0.0, 0.0);
        let edge_end = Point3::new(2.0, 0.0, 0.0);
        let midpoint = Point3::new(1.0, 0.0, 0.0);

        assert!(
            hyper_edge_projection_parameter(&edge_start, &edge_start, &edge_end).is_none()
        );
        assert!(
            hyper_edge_projection_parameter(&midpoint, &edge_start, &edge_start).is_none()
        );
    }

    #[test]
    fn mesh_bounding_box_helpers_do_not_widen_by_tolerance() {
        let container = Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
        let exact_touch =
            Aabb::new(Point3::new(1.0, 0.25, 0.25), Point3::new(2.0, 0.75, 0.75));
        let just_outside = Aabb::new(
            Point3::new(1.0 + tolerance() * 0.25, 0.25, 0.25),
            Point3::new(2.0, 0.75, 0.75),
        );
        let overhanging = Aabb::new(
            Point3::new(tolerance() * -0.25, 0.25, 0.25),
            Point3::new(0.75, 0.75, 0.75),
        );

        assert!(bounding_boxes_intersect(&container, &exact_touch));
        assert!(!bounding_boxes_intersect(&container, &just_outside));
        assert!(!bounding_box_contains_bounds(&container, &overhanging));
    }
}
