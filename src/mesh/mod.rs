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
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use hyperreal::RealSign;
use std::{cmp::PartialEq, fmt::Debug, num::NonZeroU32, sync::OnceLock};

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

fn real_cmp(lhs: &Real, rhs: &Real) -> std::cmp::Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(128) {
            Some(RealSign::Positive) => std::cmp::Ordering::Greater,
            Some(RealSign::Negative) => std::cmp::Ordering::Less,
            Some(RealSign::Zero) | None => std::cmp::Ordering::Equal,
        })
}

fn real_cmp_for_finite_bounds(lhs: &Real, rhs: &Real) -> std::cmp::Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .or_else(|| {
            let lhs = lhs.to_f64_lossy()?;
            let rhs = rhs.to_f64_lossy()?;
            lhs.partial_cmp(&rhs)
        })
        .unwrap_or(std::cmp::Ordering::Equal)
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
        if matches!(
            real_cmp_for_finite_bounds(&point.x, &min_x),
            std::cmp::Ordering::Less
        ) {
            min_x = point.x.clone();
        }
        if matches!(
            real_cmp_for_finite_bounds(&point.y, &min_y),
            std::cmp::Ordering::Less
        ) {
            min_y = point.y.clone();
        }
        if matches!(
            real_cmp_for_finite_bounds(&point.z, &min_z),
            std::cmp::Ordering::Less
        ) {
            min_z = point.z.clone();
        }
        if matches!(
            real_cmp_for_finite_bounds(&point.x, &max_x),
            std::cmp::Ordering::Greater
        ) {
            max_x = point.x.clone();
        }
        if matches!(
            real_cmp_for_finite_bounds(&point.y, &max_y),
            std::cmp::Ordering::Greater
        ) {
            max_y = point.y.clone();
        }
        if matches!(
            real_cmp_for_finite_bounds(&point.z, &max_z),
            std::cmp::Ordering::Greater
        ) {
            max_z = point.z.clone();
        }
    }

    Some((
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
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
    let exact_origin = hyperlimit_point3(origin);
    let exact_direction = hyperlimit::Point3::new(
        direction.0[0].clone(),
        direction.0[1].clone(),
        direction.0[2].clone(),
    );
    let a = hyperlimit_point3(&tri[0].position);
    let b = hyperlimit_point3(&tri[1].position);
    let c = hyperlimit_point3(&tri[2].position);
    match hyperlimit::classify_ray_triangle3_intersection_report(
        &exact_origin,
        &exact_direction,
        &a,
        &b,
        &c,
    ) {
        hyperlimit::PredicateOutcome::Decided { value: report, .. } => {
            let Some(point) = report.point else {
                return ray_triangle_intersection_lossy(origin, direction, tri);
            };
            let Some(t) = report.parameter else {
                return ray_triangle_intersection_lossy(origin, direction, tri);
            };
            if matches!(
                report.relation,
                hyperlimit::RayTriangleIntersection::Disjoint
                    | hyperlimit::RayTriangleIntersection::Coplanar
            ) {
                ray_triangle_intersection_lossy(origin, direction, tri)
            } else {
                Some((Point3::new(point.x, point.y, point.z), t))
            }
        },
        hyperlimit::PredicateOutcome::Unknown { .. } => {
            ray_triangle_intersection_lossy(origin, direction, tri)
        },
    }
}

fn hyperlimit_point3(point: &Point3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn point3_to_f64(point: &Point3) -> Option<[f64; 3]> {
    Some([
        point.x.to_f64_lossy()?,
        point.y.to_f64_lossy()?,
        point.z.to_f64_lossy()?,
    ])
}

fn vector3_to_f64(vector: &Vector3) -> Option<[f64; 3]> {
    Some([
        vector.0[0].to_f64_lossy()?,
        vector.0[1].to_f64_lossy()?,
        vector.0[2].to_f64_lossy()?,
    ])
}

fn ray_triangle_intersection_lossy(
    origin: &Point3,
    direction: &Vector3,
    tri: &[Vertex; 3],
) -> Option<(Point3, Real)> {
    let origin = point3_to_f64(origin)?;
    let direction = vector3_to_f64(direction)?;
    let a = point3_to_f64(&tri[0].position)?;
    let b = point3_to_f64(&tri[1].position)?;
    let c = point3_to_f64(&tri[2].position)?;

    let edge1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let edge2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let h = cross3(direction, edge2);
    let det = dot3(edge1, h);
    if det.abs() < 1e-12 {
        return None;
    }

    let inv_det = 1.0 / det;
    let s = [origin[0] - a[0], origin[1] - a[1], origin[2] - a[2]];
    let u = inv_det * dot3(s, h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = cross3(s, edge1);
    let v = inv_det * dot3(direction, q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = inv_det * dot3(edge2, q);
    if t < 0.0 {
        return None;
    }

    Some((
        Point3::new(
            Real::try_from(origin[0] + direction[0] * t).ok()?,
            Real::try_from(origin[1] + direction[1] * t).ok()?,
            Real::try_from(origin[2] + direction[2] * t).ok()?,
        ),
        Real::try_from(t).ok()?,
    ))
}

fn point_triangle_plane_distance_lossy(point: &Point3, tri: &[Vertex; 3]) -> Option<f64> {
    let point = point3_to_f64(point)?;
    let a = point3_to_f64(&tri[0].position)?;
    let b = point3_to_f64(&tri[1].position)?;
    let c = point3_to_f64(&tri[2].position)?;
    let normal = cross3(subtract3(b, a), subtract3(c, a));
    let normal_len = norm3(normal);
    if normal_len <= f64::EPSILON {
        return None;
    }
    Some((dot3(subtract3(point, a), normal) / normal_len).abs())
}

fn dot3(lhs: [f64; 3], rhs: [f64; 3]) -> f64 {
    lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]
}

fn norm3(value: [f64; 3]) -> f64 {
    dot3(value, value).sqrt()
}

fn subtract3(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]]
}

fn cross3(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]
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
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
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
    /// ```ignore
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

    /// Extract vertices and triangle indices through the native mesh path.
    ///
    /// This legacy convenience method returns empty buffers when validation
    /// rejects the mesh. Prefer [`Mesh::try_get_vertices_and_indices`] when the
    /// caller needs to distinguish empty geometry from invalid topology.
    #[deprecated(
        since = "0.23.0",
        note = "use try_get_vertices_and_indices to preserve validation failures"
    )]
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
            if let (Some(lhs), Some(rhs)) = (a.1.to_f64_lossy(), b.1.to_f64_lossy())
                && (lhs - rhs).abs() <= 1e-10
            {
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
    /// ```ignore
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
        if self.polygons.is_empty() {
            return false;
        }

        let bounds = self.bounding_box();
        if real_lt(&point.x, &bounds.mins.x)
            || real_gt(&point.x, &bounds.maxs.x)
            || real_lt(&point.y, &bounds.mins.y)
            || real_gt(&point.y, &bounds.maxs.y)
            || real_lt(&point.z, &bounds.mins.z)
            || real_gt(&point.z, &bounds.maxs.z)
        {
            return false;
        }

        let query = hyperlimit_point3(point);
        for tri in self.polygons.iter().flat_map(|polygon| polygon.triangulate()) {
            let a = hyperlimit_point3(&tri[0].position);
            let b = hyperlimit_point3(&tri[1].position);
            let c = hyperlimit_point3(&tri[2].position);
            let on_triangle = matches!(
                hyperlimit::classify_point_triangle3(&a, &b, &c, &query).value(),
                Some(
                    hyperlimit::Triangle3Location::Inside
                        | hyperlimit::Triangle3Location::OnEdge
                        | hyperlimit::Triangle3Location::OnVertex
                )
            );
            if on_triangle
                && point_triangle_plane_distance_lossy(point, &tri)
                    .is_some_and(|distance| distance <= 1e-8)
            {
                return false;
            }
        }

        if let Some(inside) = self.contains_vertex_winding_lossy(point) {
            return inside;
        }

        let one = Real::one();
        let three = Real::from(3_u8);
        let seven = Real::from(7_u8);
        let direction = Vector3::from_xyz(
            one.clone(),
            (one.clone() / three).unwrap_or_else(|_| one.clone()),
            (one.clone() / seven).unwrap_or(one),
        );

        self.ray_intersections(point, &direction).len() % 2 == 1
    }

    fn contains_vertex_winding_lossy(&self, point: &Point3) -> Option<bool> {
        let point = point3_to_f64(point)?;
        let mut solid_angle = 0.0_f64;

        for tri in self.polygons.iter().flat_map(|polygon| polygon.triangulate()) {
            let a = subtract3(point3_to_f64(&tri[0].position)?, point);
            let b = subtract3(point3_to_f64(&tri[1].position)?, point);
            let c = subtract3(point3_to_f64(&tri[2].position)?, point);
            let la = norm3(a);
            let lb = norm3(b);
            let lc = norm3(c);
            if la <= f64::EPSILON || lb <= f64::EPSILON || lc <= f64::EPSILON {
                return Some(false);
            }

            let numerator = dot3(a, cross3(b, c));
            let denominator =
                la * lb * lc + dot3(a, b) * lc + dot3(b, c) * la + dot3(c, a) * lb;
            solid_angle += (2.0 * numerator.atan2(denominator)).abs();
        }

        Some(solid_angle.abs() > std::f64::consts::PI * 2.0)
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

    /// Return a new mesh representing the exact hypermesh union, or the reason
    /// hypermesh could not import, validate, or materialize the result.
    pub fn try_union(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        let self_buffers = self.to_hypermesh_buffers();
        let other_buffers = other.to_hypermesh_buffers();
        if self.polygons.is_empty() || self_buffers.indices.is_empty() {
            return Ok(other.clone().with_metadata(self.metadata.clone()));
        }
        if other.polygons.is_empty() || other_buffers.indices.is_empty() {
            return Ok(self.clone());
        }
        if self_buffers == other_buffers {
            return Ok(self.clone());
        }
        self.boolean_via_hypermesh(other, hypermesh::HypermeshBooleanOp::Union)
    }

    /// Return a new mesh representing the exact hypermesh difference, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_difference(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        let self_buffers = self.to_hypermesh_buffers();
        let other_buffers = other.to_hypermesh_buffers();
        if self.polygons.is_empty() || self_buffers.indices.is_empty() {
            return Ok(Mesh::empty(self.metadata.clone()));
        }
        if other.polygons.is_empty() || other_buffers.indices.is_empty() {
            return Ok(self.clone());
        }
        if self_buffers == other_buffers {
            return Ok(Mesh::empty(self.metadata.clone()));
        }
        self.boolean_via_hypermesh(other, hypermesh::HypermeshBooleanOp::Difference)
    }

    /// Return a new mesh representing the exact hypermesh intersection, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        let self_buffers = self.to_hypermesh_buffers();
        let other_buffers = other.to_hypermesh_buffers();
        if self.polygons.is_empty()
            || other.polygons.is_empty()
            || self_buffers.indices.is_empty()
            || other_buffers.indices.is_empty()
        {
            return Ok(Mesh::empty(self.metadata.clone()));
        }
        if self_buffers == other_buffers {
            return Ok(self.clone());
        }
        self.boolean_via_hypermesh(other, hypermesh::HypermeshBooleanOp::Intersection)
    }

    /// Return a new mesh representing the exact hypermesh symmetric
    /// difference, or the typed reason hypermesh could not produce it.
    pub fn try_xor(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        let self_buffers = self.to_hypermesh_buffers();
        let other_buffers = other.to_hypermesh_buffers();
        if self.polygons.is_empty() || self_buffers.indices.is_empty() {
            return Ok(other.clone().with_metadata(self.metadata.clone()));
        }
        if other.polygons.is_empty() || other_buffers.indices.is_empty() {
            return Ok(self.clone());
        }
        if self_buffers == other_buffers {
            return Ok(Mesh::empty(self.metadata.clone()));
        }
        self.boolean_via_hypermesh(other, hypermesh::HypermeshBooleanOp::Xor)
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
        self.try_union(other)
            .unwrap_or_else(|error| panic!("hypermesh union failed: {error}"))
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
        self.try_difference(other)
            .unwrap_or_else(|error| panic!("hypermesh difference failed: {error}"))
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
        self.try_intersection(other)
            .unwrap_or_else(|error| panic!("hypermesh intersection failed: {error}"))
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
        self.try_xor(other)
            .unwrap_or_else(|error| panic!("hypermesh xor failed: {error}"))
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

        let intersecting = |lhs: &Aabb, rhs: &Aabb| {
            let lhs_min = hyperlimit::Point3::new(
                lhs.mins.x.clone(),
                lhs.mins.y.clone(),
                lhs.mins.z.clone(),
            );
            let lhs_max = hyperlimit::Point3::new(
                lhs.maxs.x.clone(),
                lhs.maxs.y.clone(),
                lhs.maxs.z.clone(),
            );
            let rhs_min = hyperlimit::Point3::new(
                rhs.mins.x.clone(),
                rhs.mins.y.clone(),
                rhs.mins.z.clone(),
            );
            let rhs_max = hyperlimit::Point3::new(
                rhs.maxs.x.clone(),
                rhs.maxs.y.clone(),
                rhs.maxs.z.clone(),
            );

            matches!(
                hyperlimit::aabb3s_intersect(&lhs_min, &lhs_max, &rhs_min, &rhs_max).value(),
                Some(true)
            )
        };
        let contains = |container: &Aabb, contained: &Aabb| {
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

            let container = hyperlimit::PreparedAabb3::new(&container_min, &container_max);
            matches!(container.contains_point(&contained_min).value(), Some(true))
                && matches!(container.contains_point(&contained_max).value(), Some(true))
        };

        assert!(intersecting(&container, &exact_touch));
        assert!(!intersecting(&container, &just_outside));
        assert!(!contains(&container, &overhanging));
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
