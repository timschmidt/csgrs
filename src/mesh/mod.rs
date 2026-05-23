//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::errors::ValidationError;

use crate::mesh::plane::Plane;
use crate::polygon::Polygon;
use crate::vertex::Vertex;

#[cfg(feature = "sketch")]
use crate::sketch::Profile;

use crate::csg::CSG;
#[cfg(feature = "sketch")]
use hypercurve::{Classification, FiniteProjectionOptions};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hyperlimit::{PreparedAabb3, aabb3s_intersect, real_max, real_min};
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use hyperreal::RealSign;
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

/// Stored as `(position: [Real; 3], normal: [Real; 3])`.
pub type GraphicsMeshVertex = ([Real; 3], [Real; 3]);

/// Mesh data laid out for renderers that want vertex buffers plus u32 indices.
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

    /// Whole-mesh metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub metadata: M,
}

fn bounding_box_contains_bounds(container: &Aabb, contained: &Aabb) -> bool {
    let container_min = hyperlimit::Point3::new(
        container.mins.x.clone(),
        container.mins.y.clone(),
        container.mins.z.clone(),
    );
    let container_max = hyperlimit::Point3::new(
        container.maxs.x.clone(),
        container.maxs.y.clone(),
        container.maxs.z.clone(),
    );
    let contained_min = hyperlimit::Point3::new(
        contained.mins.x.clone(),
        contained.mins.y.clone(),
        contained.mins.z.clone(),
    );
    let contained_max = hyperlimit::Point3::new(
        contained.maxs.x.clone(),
        contained.maxs.y.clone(),
        contained.maxs.z.clone(),
    );

    let container = PreparedAabb3::new(&container_min, &container_max);
    matches!(container.contains_point(&contained_min).value(), Some(true))
        && matches!(container.contains_point(&contained_max).value(), Some(true))
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
    let lhs_min =
        hyperlimit::Point3::new(lhs.mins.x.clone(), lhs.mins.y.clone(), lhs.mins.z.clone());
    let lhs_max =
        hyperlimit::Point3::new(lhs.maxs.x.clone(), lhs.maxs.y.clone(), lhs.maxs.z.clone());
    let rhs_min =
        hyperlimit::Point3::new(rhs.mins.x.clone(), rhs.mins.y.clone(), rhs.mins.z.clone());
    let rhs_max =
        hyperlimit::Point3::new(rhs.maxs.x.clone(), rhs.maxs.y.clone(), rhs.maxs.z.clone());

    matches!(
        aabb3s_intersect(&lhs_min, &lhs_max, &rhs_min, &rhs_max).value(),
        Some(true)
    )
}

fn real_cmp(lhs: &Real, rhs: &Real) -> std::cmp::Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(128) {
            Some(RealSign::Positive) => std::cmp::Ordering::Greater,
            Some(RealSign::Negative) => std::cmp::Ordering::Less,
            Some(RealSign::Zero) | None => std::cmp::Ordering::Equal,
        })
}

fn real_gt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Greater)
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Less)
}

fn real_zero(value: &Real) -> bool {
    matches!(value.refine_sign_until(128), Some(RealSign::Zero))
}

fn point3_bounds(points: &[Point3]) -> Option<(Point3, Point3)> {
    let first = points.first()?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut min_z = first.z.clone();
    let mut max_x = min_x.clone();
    let mut max_y = min_y.clone();
    let mut max_z = min_z.clone();

    for point in &points[1..] {
        min_x = real_min(&min_x, &point.x).value()?.clone();
        min_y = real_min(&min_y, &point.y).value()?.clone();
        min_z = real_min(&min_z, &point.z).value()?.clone();
        max_x = real_max(&max_x, &point.x).value()?.clone();
        max_y = real_max(&max_y, &point.y).value()?.clone();
        max_z = real_max(&max_z, &point.z).value()?.clone();
    }

    Some((
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
}

fn mesh_from_bounding_box_intersection<M: Clone + Send + Sync + Debug>(
    lhs: &Aabb,
    rhs: &Aabb,
    metadata: M,
) -> Mesh<M> {
    let Some(min_x) = real_max(&lhs.mins.x, &rhs.mins.x).value().cloned() else {
        return Mesh::empty(metadata);
    };
    let Some(min_y) = real_max(&lhs.mins.y, &rhs.mins.y).value().cloned() else {
        return Mesh::empty(metadata);
    };
    let Some(min_z) = real_max(&lhs.mins.z, &rhs.mins.z).value().cloned() else {
        return Mesh::empty(metadata);
    };
    let Some(max_x) = real_min(&lhs.maxs.x, &rhs.maxs.x).value().cloned() else {
        return Mesh::empty(metadata);
    };
    let Some(max_y) = real_min(&lhs.maxs.y, &rhs.maxs.y).value().cloned() else {
        return Mesh::empty(metadata);
    };
    let Some(max_z) = real_min(&lhs.maxs.z, &rhs.maxs.z).value().cloned() else {
        return Mesh::empty(metadata);
    };

    let width = max_x.clone() - min_x.clone();
    let length = max_y.clone() - min_y.clone();
    let height = max_z.clone() - min_z.clone();
    if !real_gt(&width, &Real::zero())
        || !real_gt(&length, &Real::zero())
        || !real_gt(&height, &Real::zero())
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

fn hyperphysics_vector3_from_point3(point: &Point3) -> Result<HyperVector3, ValidationError> {
    Ok(HyperVector3::new([
        hyperphysics::Real::try_from(point.x.clone())
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(point.clone())))?,
        hyperphysics::Real::try_from(point.y.clone())
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(point.clone())))?,
        hyperphysics::Real::try_from(point.z.clone())
            .map_err(|err| ValidationError::Other(format!("{err:?}"), Some(point.clone())))?,
    ]))
}

fn ray_triangle_intersection(
    origin: &Point3,
    direction: &Vector3,
    tri: &[Vertex; 3],
) -> Option<(Point3, Real)> {
    let edge1 = &tri[1].position - &tri[0].position;
    let edge2 = &tri[2].position - &tri[0].position;
    let h = direction.cross(&edge2);
    let det = edge1.dot(&h);
    if real_zero(&det) {
        return None;
    }

    let inv_det = (Real::one() / det).ok()?;
    let s = origin - &tri[0].position;
    let u = inv_det.clone() * s.dot(&h);
    let zero = Real::zero();
    let one = Real::one();
    if real_lt(&u, &zero) || real_gt(&u, &one) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = inv_det.clone() * direction.dot(&q);
    if real_lt(&v, &zero) || real_gt(&(u.clone() + v.clone()), &one) {
        return None;
    }

    let t = inv_det * edge2.dot(&q);
    if real_lt(&t, &zero) {
        return None;
    }

    let point = origin.clone() + direction.clone() * t.clone();
    Some((point, t))
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
            metadata,
        }
    }

    /// Build a Mesh from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<M>], metadata: M) -> Self {
        Mesh {
            polygons: polygons.to_vec(),
            bounding_box: OnceLock::new(),
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

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<M> {
        let triangles = self
            .polygons
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
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
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
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
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
        n1.angle_to(&n2).unwrap_or_else(|_| Real::zero())
    }

    /// Converts this mesh into exact vertex/index buffers suitable for rendering adapters.
    pub fn build_graphics_mesh(&self) -> GraphicsMesh {
        let triangles = self.triangulate().polygons;
        let triangle_count = triangles.len();

        let mut indices = Vec::with_capacity(triangle_count * 3);
        let mut vertices = Vec::with_capacity(triangle_count * 3);

        for triangle in triangles {
            for vertex in triangle.vertices {
                let position = [
                    vertex.position.x.clone(),
                    vertex.position.y.clone(),
                    vertex.position.z.clone(),
                ];
                let normal = [
                    vertex.normal.0[0].clone(),
                    vertex.normal.0[1].clone(),
                    vertex.normal.0[2].clone(),
                ];

                let index = vertices.len() as u32;
                vertices.push((position, normal));
                indices.push(index);
            }
        }

        vertices.shrink_to_fit();
        GraphicsMesh { vertices, indices }
    }

    /// Try to extract hyperreal vertices and triangle indices.
    ///
    /// This is the native mesh-buffer view. It does not cross a primitive-float
    /// boundary; coordinates stay in [`Real`] and only the
    /// topological index carrier is lowered to `u32`.
    pub fn try_get_vertices_and_indices(
        &self,
    ) -> Result<(Vec<Point3>, Vec<[u32; 3]>), ValidationError> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|polygon| polygon.triangulate())
            .collect::<Vec<_>>();
        let mut vertices = Vec::with_capacity(triangles.len() * 3);
        let mut indices = Vec::with_capacity(triangles.len());

        for triangle in triangles {
            let base = u32::try_from(vertices.len()).map_err(|_| {
                ValidationError::MeshBufferError("mesh vertex count exceeded u32".into())
            })?;
            vertices.push(triangle[0].position.clone());
            vertices.push(triangle[1].position.clone());
            vertices.push(triangle[2].position.clone());
            indices.push([base, base + 1, base + 2]);
        }

        Ok((vertices, indices))
    }

    /// Extract vertices and triangle indices through the `hypermesh` handoff path.
    ///
    /// This compatibility method returns empty buffers when exact handoff
    /// rejects the mesh. Prefer [`Mesh::try_get_vertices_and_indices`] when the
    /// caller needs to distinguish empty geometry from invalid topology.
    pub fn get_vertices_and_indices(&self) -> (Vec<Point3>, Vec<[u32; 3]>) {
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
    /// A `Vec` of `(Point3, Real)` where:
    /// - `Point3` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    ///
    /// Triangle intersections are evaluated with hyperreal vector operations,
    /// keeping hit ordering and deduplication in the same scalar domain as the
    /// mesh coordinates.
    pub fn ray_intersections(
        &self,
        origin: &Point3,
        direction: &Vector3,
    ) -> Vec<(Point3, Real)> {
        let mut hits: Vec<_> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .filter_map(|tri| ray_triangle_intersection(origin, direction, &tri))
            .collect();

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| real_cmp(&a.1, &b.1));
        // 5) remove duplicate hits only when Hyper proves exact identity.
        hits.dedup_by(|a, b| {
            if real_zero(&(a.1.clone() - b.1.clone())) {
                return true;
            }
            let a_point = hyperlimit::Point3::new(a.0.x.clone(), a.0.y.clone(), a.0.z.clone());
            let b_point = hyperlimit::Point3::new(b.0.x.clone(), b.0.y.clone(), b.0.z.clone());
            matches!(
                hyperlimit::point3_equal(&a_point, &b_point).value(),
                Some(true)
            )
        });

        hits
    }

    /// Find all intersection points between a polyline and this mesh's
    /// triangulated surface.
    ///
    /// Each consecutive pair of points defines one segment. Hits are deduplicated
    /// locally and returned in polyline order. Deduplication requires exact
    /// hit-point equality through `hyperlattice::Vector3` and `Real`,
    /// keeping this topology-affecting equality decision out of local f64
    /// tolerance arithmetic. This follows Yap's
    /// exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn intersect_polyline(&self, polyline: &[Point3]) -> Vec<Point3> {
        if polyline.len() < 2 {
            return Vec::new();
        }

        let triangles: Vec<[Vertex; 3]> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.triangulate())
            .collect();

        let mut hits: Vec<Point3> = Vec::new();

        for seg in polyline.windows(2) {
            let seg_start = seg[0].clone();
            let seg_end = seg[1].clone();
            let seg_dir = &seg_end - &seg_start;
            if real_zero(&seg_dir.dot(&seg_dir)) {
                continue;
            }

            let mut seg_hits: Vec<(Point3, Real)> = Vec::new();

            for tri in &triangles {
                if let Some((point, t)) = ray_triangle_intersection(&seg_start, &seg_dir, tri)
                {
                    if !real_lt(&t, &Real::zero()) && !real_gt(&t, &Real::one()) {
                        seg_hits.push((point, t));
                    }
                }
            }

            seg_hits.sort_by(|a, b| real_cmp(&a.1, &b.1));

            for (point, _) in seg_hits {
                if let Some(last) = hits.last() {
                    let point_h = hyperlimit::Point3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                    );
                    let last_h = hyperlimit::Point3::new(
                        last.x.clone(),
                        last.y.clone(),
                        last.z.clone(),
                    );
                    if matches!(
                        hyperlimit::point3_equal(&point_h, &last_h).value(),
                        Some(true)
                    ) {
                        continue;
                    }
                }
                hits.push(point);
            }
        }

        hits
    }

    /// Uses hyperreal triangle ray intersections to check if a point is inside a `Mesh`.
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    /// # use hyperlattice::Point3;
    /// # use hyperlattice::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, ());
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3) -> bool {
        self.ray_intersections(
            point,
            &Vector3::from_xyz(Real::one(), Real::one(), Real::one()),
        )
        .len()
            % 2
            == 1
    }

    /// Mass properties through the Hyper physics stack.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> Result<(Real, Point3, Matrix4), ValidationError> {
        let mp = self.exact_mass_properties(density)?;
        let center = Point3::new(
            mp.center_of_mass[0].clone(),
            mp.center_of_mass[1].clone(),
            mp.center_of_mass[2].clone(),
        );
        Ok((mp.mass, center, Matrix4::identity()))
    }

    /// Exact uniform-density mass properties through `hyperphysics`.
    ///
    /// This is the report-bearing counterpart to [`Mesh::mass_properties`]. Mesh
    /// coordinates are finite `csgrs` adapter scalars at this boundary; each
    /// coordinate and the density are lifted into [`hyperphysics::Real`] before
    /// volume, center of mass, and inertia are accumulated. The report-bearing
    /// path follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) by keeping the lossy
    /// exact object layer.
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

    /// Converts triangulated mesh polygons into a `hyperphysics` closed mesh carrier.
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
                positions_32.push(v.position.to_f32_array_lossy().unwrap_or([0.0; 3]));
                normals_32.push([
                    v.normal.0[0].to_f32_lossy().unwrap_or(0.0),
                    v.normal.0[1].to_f32_lossy().unwrap_or(0.0),
                    v.normal.0[2].to_f32_lossy().unwrap_or(0.0),
                ]);
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
    /// `hyperlattice::Vector3`/`Real` before being stored back at the
    /// finite mesh boundary. This keeps the inverse-transpose normal path on
    /// the same exact-aware boundary discipline as mesh predicates; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn transform(&self, mat: &Matrix4) -> Mesh<M> {
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match mat.clone().inverse() {
            Ok(inv) => inv.transpose(),
            Err(_) => {
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
                match mat.transform_point3(&vert.position) {
                    Ok(transformed_position) => vert.position = transformed_position,
                    Err(_) => {
                        eprintln!(
                            "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                        );
                        continue;
                    },
                }

                // Transform normal using inverse transpose rule
                let transformed_normal = mat_inv_transpose.transform_direction3(&vert.normal);
                if let Ok(normal) = transformed_normal.normalize_checked() {
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

        mesh
    }

    /// Returns an axis-aligned bounding box indicating the 3D bounds of all
    /// `polygons`.
    fn bounding_box(&self) -> Aabb {
        self.bounding_box
            .get_or_init(|| {
                let points = self
                    .polygons
                    .iter()
                    .flat_map(|polygon| {
                        polygon.vertices.iter().map(|vertex| vertex.position.clone())
                    })
                    .collect::<Vec<_>>();
                let Some((mins, maxs)) = point3_bounds(&points) else {
                    return Aabb::origin();
                };
                Aabb::new(mins, maxs)
            })
            .clone()
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
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
        fn ring_to_vertices(ring: &[[f64; 2]]) -> Vec<Vertex> {
            let mut vertices: Vec<_> = ring
                .iter()
                .map(|p| {
                    Vertex::new(
                        Point3::new(
                            Real::try_from(p[0]).unwrap_or_else(|_| Real::zero()),
                            Real::try_from(p[1]).unwrap_or_else(|_| Real::zero()),
                            Real::zero(),
                        ),
                        Vector3::z(),
                    )
                })
                .collect();
            if vertices.first() == vertices.last() {
                vertices.pop();
            }
            vertices
        }

        fn ring_to_polygon<M: Clone + Send + Sync>(
            ring: &[[f64; 2]],
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
                metadata: sketch.metadata.clone(),
            };
        }

        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: sketch.metadata.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64, tolerance};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn mesh_bounding_box_helpers_do_not_widen_by_tolerance() {
        let container = Aabb::new(Point3::origin(), p3(1.0, 1.0, 1.0));
        let exact_touch = Aabb::new(p3(1.0, 0.25, 0.25), p3(2.0, 0.75, 0.75));
        let just_outside = Aabb::new(
            Point3::new(r(1.0) + tolerance() * r(0.25), r(0.25), r(0.25)),
            p3(2.0, 0.75, 0.75),
        );
        let overhanging = Aabb::new(
            Point3::new(tolerance() * r(-0.25), r(0.25), r(0.25)),
            p3(0.75, 0.75, 0.75),
        );

        assert!(bounding_boxes_intersect(&container, &exact_touch));
        assert!(!bounding_boxes_intersect(&container, &just_outside));
        assert!(!bounding_box_contains_bounds(&container, &overhanging));
    }

    #[test]
    fn mesh_triangulate_does_not_repair_cross_polygon_t_junctions() {
        let normal = Vector3::z();
        let polygons = vec![
            Polygon::new(
                vec![
                    Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 1.0, 0.0), normal.clone()),
                    Vertex::new(p3(0.0, 1.0, 0.0), normal.clone()),
                ],
                (),
            ),
            Polygon::new(
                vec![
                    Vertex::new(p3(1.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(1.5, -0.5, 0.0), normal.clone()),
                    Vertex::new(p3(0.5, -0.5, 0.0), normal),
                ],
                (),
            ),
        ];

        let triangulated = Mesh::from_polygons(&polygons, ()).triangulate();

        assert_eq!(triangulated.polygons.len(), 3);
    }
}
