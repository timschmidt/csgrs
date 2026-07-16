//! Planar n-gon face storage and derived geometry.

use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use hashbrown::HashMap;
use hyperlattice::{Aabb, Point3, Real, Vector3};
use hyperreal::Rational;
use std::cell::RefCell;
use std::ops::{Deref, DerefMut};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_PLANE_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Clone, Debug)]
struct CachedPolygonNormal {
    vertex_ids: Vec<u64>,
    normal: Vector3,
}

thread_local! {
    static POLYGON_NORMALS: RefCell<HashMap<u64, CachedPolygonNormal>> =
        RefCell::new(HashMap::new());
}

const POLYGON_NORMAL_CACHE_CAPACITY: usize = 8_192;

pub(crate) fn fresh_plane_id() -> u64 {
    NEXT_PLANE_ID.fetch_add(1, Ordering::Relaxed)
}

mod triangulation;

/// A polygon, defined by a list of vertices.
/// - `M` is the generic metadata type stored directly on the polygon. Use
///   `M = ()` for no metadata, or `M = Option<YourMetadata>` for optional metadata.
#[derive(Debug, Clone)]
pub struct Polygon<M: Clone> {
    pub(crate) vertices: Vec<Vertex>,
    pub(crate) plane: Plane,
    pub(crate) plane_id: u64,
    bounding_box: OnceLock<Aabb>,
    certified_f64_bounds: OnceLock<Option<CertifiedF64Bounds>>,
    prepared_triangle_query: OnceLock<Option<PreparedTriangleQuery>>,
    certified_nondegenerate: OnceLock<Option<bool>>,
    pub(crate) metadata: M,
}

/// Outward-rounded binary bounds certified to contain a polygon's exact
/// coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CertifiedF64Bounds {
    pub(crate) min: [f64; 3],
    pub(crate) max: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedTriangleQuery {
    pub(crate) edge1: Vector3,
    pub(crate) edge2: Vector3,
    pub(crate) exact_x_axis: OnceLock<Option<PreparedExactXAxisTriangle>>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedExactXAxisTriangle {
    pub(crate) first: [Rational; 3],
    pub(crate) edge1: [Rational; 3],
    pub(crate) edge2: [Rational; 3],
    pub(crate) determinant_unit: Rational,
    pub(crate) normal: [Rational; 3],
    pub(crate) first_dot_normal: Rational,
}

/// Mutable access to a polygon's existing vertices.
///
/// Dropping this guard refreshes geometry derived from vertex positions.
pub struct PolygonVerticesMut<'a, M: Clone + Send + Sync> {
    polygon: &'a mut Polygon<M>,
    original_geometry: Option<Vec<(Point3, Vector3)>>,
}

impl<M: Clone + Send + Sync> Deref for PolygonVerticesMut<'_, M> {
    type Target = [Vertex];

    fn deref(&self) -> &Self::Target {
        &self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> DerefMut for PolygonVerticesMut<'_, M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.polygon.vertices
    }
}

impl<M: Clone + Send + Sync> Drop for PolygonVerticesMut<'_, M> {
    fn drop(&mut self) {
        let mut position_changed = false;
        if let Some(original_geometry) = &self.original_geometry {
            for (vertex, (original_position, original_normal)) in
                self.polygon.vertices.iter_mut().zip(original_geometry)
            {
                let vertex_position_changed = vertex.position != *original_position;
                if vertex_position_changed || vertex.normal != *original_normal {
                    vertex.refresh_position_identity();
                }
                if vertex_position_changed {
                    position_changed = true;
                }
            }
        }
        if position_changed {
            self.polygon.plane_id = fresh_plane_id();
        }
        self.polygon.refresh_geometry();
    }
}

impl<M: Clone + PartialEq> PartialEq for Polygon<M> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

impl<M: Clone + Send + Sync> Polygon<M> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: M) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");

        let plane = Plane::from_vertices(&vertices);
        Polygon {
            vertices,
            plane,
            plane_id: fresh_plane_id(),
            bounding_box: OnceLock::new(),
            certified_f64_bounds: OnceLock::new(),
            prepared_triangle_query: OnceLock::new(),
            certified_nondegenerate: OnceLock::new(),
            metadata,
        }
    }

    /// Return this polygon with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync>(self, metadata: NewM) -> Polygon<NewM> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            certified_f64_bounds: self.certified_f64_bounds,
            prepared_triangle_query: self.prepared_triangle_query,
            certified_nondegenerate: self.certified_nondegenerate,
            metadata,
        }
    }

    pub(crate) const fn with_plane_id(mut self, plane_id: u64) -> Self {
        self.plane_id = plane_id;
        self
    }

    /// Map this polygon's metadata while preserving its geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync, F>(self, f: F) -> Polygon<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            plane_id: self.plane_id,
            bounding_box: self.bounding_box,
            certified_f64_bounds: self.certified_f64_bounds,
            prepared_triangle_query: self.prepared_triangle_query,
            certified_nondegenerate: self.certified_nondegenerate,
            metadata: f(self.metadata),
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
    pub fn bounding_box(&self) -> Aabb {
        self.bounding_box_ref().clone()
    }

    pub(crate) fn bounding_box_ref(&self) -> &Aabb {
        self.bounding_box.get_or_init(|| {
            let Some((mins, maxs)) =
                point3_bounds(self.vertices.iter().map(|vertex| &vertex.position))
            else {
                return Aabb::origin();
            };
            Aabb::new(mins, maxs)
        })
    }

    pub(crate) fn cached_bounding_box_ref(&self) -> Option<&Aabb> {
        self.bounding_box.get()
    }

    pub(crate) fn certified_f64_bounds_ref(&self) -> Option<&CertifiedF64Bounds> {
        self.certified_f64_bounds
            .get_or_init(|| certified_f64_bounds(&self.vertices))
            .as_ref()
    }

    pub(crate) fn prepared_triangle_query_ref(&self) -> Option<&PreparedTriangleQuery> {
        self.prepared_triangle_query
            .get_or_init(|| prepared_triangle_query(&self.vertices))
            .as_ref()
    }

    pub(crate) fn prepare_spatial_query_caches(&self) {
        let _ = self.certified_f64_bounds_ref();
        let _ = self.prepared_triangle_query_ref();
    }

    #[cfg(feature = "stl-io")]
    pub(crate) fn certified_nondegenerate(&self) -> Option<bool> {
        *self
            .certified_nondegenerate
            .get_or_init(|| self.compute_certified_nondegenerate())
    }

    pub(crate) fn certify_nondegenerate(&self) {
        let _ = self.certified_nondegenerate.set(Some(true));
    }

    #[cfg(feature = "stl-io")]
    fn compute_certified_nondegenerate(&self) -> Option<bool> {
        let normal = if let Some(prepared) = self.prepared_triangle_query_ref() {
            prepared.edge1.cross(&prepared.edge2)
        } else {
            let first = self.plane.point_b.clone() - self.plane.point_a.clone();
            let second = self.plane.point_c.clone() - self.plane.point_a.clone();
            first.cross(&second)
        };
        let mut all_exact = true;
        for component in &normal.0 {
            match component.exact_rational_ref() {
                Some(rational) if !rational.is_zero() => return Some(true),
                Some(_) => {},
                None => all_exact = false,
            }
        }
        all_exact.then_some(false)
    }

    #[cfg(test)]
    pub(crate) fn has_cached_bounding_box(&self) -> bool {
        self.bounding_box.get().is_some()
    }

    /// Vertices in winding order.
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    /// Mutably access vertex values while preserving derived geometry.
    ///
    /// The slice has a fixed length, so the polygon cannot become degenerate
    /// through this API. Its plane and cached bounds are refreshed on drop.
    pub fn vertices_mut(&mut self) -> PolygonVerticesMut<'_, M> {
        let original_geometry = self
            .vertices
            .iter()
            .map(|vertex| (vertex.position.clone(), vertex.normal.clone()))
            .collect();
        PolygonVerticesMut {
            polygon: self,
            original_geometry: Some(original_geometry),
        }
    }

    pub(crate) const fn vertices_mut_with_managed_identity(
        &mut self,
    ) -> PolygonVerticesMut<'_, M> {
        PolygonVerticesMut {
            polygon: self,
            original_geometry: None,
        }
    }

    /// Plane derived from the current vertices.
    pub const fn plane(&self) -> &Plane {
        &self.plane
    }

    fn refresh_geometry(&mut self) {
        self.plane = Plane::from_vertices(&self.vertices);
        self.bounding_box = OnceLock::new();
        self.certified_f64_bounds = OnceLock::new();
        self.prepared_triangle_query = OnceLock::new();
        self.certified_nondegenerate = OnceLock::new();
    }

    pub(crate) fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
        self.certified_f64_bounds = OnceLock::new();
        self.prepared_triangle_query = OnceLock::new();
    }

    /// Reverses winding order, flips vertices normals, and flips the plane normal
    pub fn flip(&mut self) {
        // 1) reverse vertices
        self.vertices.reverse();
        // 2) flip all vertex normals
        for v in &mut self.vertices {
            v.flip();
        }
        // 3) flip the cached plane too
        self.plane.flip();
        self.prepared_triangle_query = OnceLock::new();
    }

    /// Return an iterator over paired vertices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// Return a normal calculated from all polygon vertices.
    ///
    /// The oriented area normal is evaluated with `hyperlattice::Vector3`.
    /// Exact-rational triangle crosses use a finite normalized view for the
    /// mesh's shading-normal attribute; symbolic inputs retain the exact
    /// checked normalization path. Topology predicates never consume this
    /// output attribute.
    pub fn calculate_new_normal(&self) -> Vector3 {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        if let Some(normal) = POLYGON_NORMALS.with_borrow(|normals| {
            normals.get(&self.plane_id).and_then(|cached| {
                (cached.vertex_ids.len() == self.vertices.len()
                    && cached
                        .vertex_ids
                        .iter()
                        .zip(&self.vertices)
                        .all(|(id, vertex)| *id == vertex.position_id))
                .then(|| cached.normal.clone())
            })
        }) {
            return normal;
        }

        let normal = if let Some(prepared) = self.prepared_triangle_query_ref() {
            let area_normal = prepared.edge1.cross(&prepared.edge2);
            finite_normalized_exact_rational(&area_normal)
                .or_else(|| area_normal.normalize().ok())
                .unwrap_or_else(Vector3::z)
        } else {
            // Newell's oriented area vector already follows the polygon
            // winding. Recomputing and normalizing the support plane solely to
            // compare its direction duplicates the same exact
            // cross/square-root work.
            hyper_polygon_newell_normal(&self.vertices).unwrap_or_else(Vector3::z)
        };

        POLYGON_NORMALS.with_borrow_mut(|normals| {
            if normals.len() == POLYGON_NORMAL_CACHE_CAPACITY {
                normals.clear();
            }
            normals.insert(
                self.plane_id,
                CachedPolygonNormal {
                    vertex_ids: self
                        .vertices
                        .iter()
                        .map(|vertex| vertex.position_id)
                        .collect(),
                    normal: normal.clone(),
                },
            );
        });
        normal
    }

    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        // Assign each vertex's normal to match the plane
        let new_normal = self.calculate_new_normal();
        for v in &mut self.vertices {
            v.normal = new_normal.clone();
        }
    }

    /// Returns a reference to the metadata.
    pub const fn metadata(&self) -> &M {
        &self.metadata
    }

    /// Returns a mutable reference to the metadata.
    pub const fn metadata_mut(&mut self) -> &mut M {
        &mut self.metadata
    }

    /// Sets the metadata to the given value.
    pub fn set_metadata(&mut self, data: M) {
        self.metadata = data;
    }
}

fn point3_bounds<'a>(
    points: impl IntoIterator<Item = &'a Point3>,
) -> Option<(Point3, Point3)> {
    let mut points = points.into_iter();
    let first = points.next()?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut min_z = first.z.clone();
    let mut max_x = min_x.clone();
    let mut max_y = min_y.clone();
    let mut max_z = min_z.clone();

    for point in points {
        if super::real_lt(&point.x, &min_x) {
            min_x = point.x.clone();
        }
        if super::real_lt(&point.y, &min_y) {
            min_y = point.y.clone();
        }
        if super::real_lt(&point.z, &min_z) {
            min_z = point.z.clone();
        }
        if super::real_gt(&point.x, &max_x) {
            max_x = point.x.clone();
        }
        if super::real_gt(&point.y, &max_y) {
            max_y = point.y.clone();
        }
        if super::real_gt(&point.z, &max_z) {
            max_z = point.z.clone();
        }
    }

    Some((
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
}

fn prepared_triangle_query(vertices: &[Vertex]) -> Option<PreparedTriangleQuery> {
    if vertices.len() != 3 {
        return None;
    }
    let edge1 = vertices[1].position.clone() - vertices[0].position.clone();
    let edge2 = vertices[2].position.clone() - vertices[0].position.clone();
    Some(PreparedTriangleQuery {
        edge1,
        edge2,
        exact_x_axis: OnceLock::new(),
    })
}

fn certified_f64_bounds(vertices: &[Vertex]) -> Option<CertifiedF64Bounds> {
    let mut minimum = [f64::INFINITY; 3];
    let mut maximum = [f64::NEG_INFINITY; 3];
    for vertex in vertices {
        for (axis, coordinate) in [&vertex.position.x, &vertex.position.y, &vertex.position.z]
            .into_iter()
            .enumerate()
        {
            let [lower, upper] = coordinate.certified_dyadic_interval(-20)?;
            let lower = Real::from(lower).to_f64_lossy()?;
            let upper = Real::from(upper).to_f64_lossy()?;
            if !lower.is_finite() || !upper.is_finite() {
                return None;
            }
            minimum[axis] = minimum[axis].min(lower.next_down());
            maximum[axis] = maximum[axis].max(upper.next_up());
        }
    }
    Some(CertifiedF64Bounds {
        min: minimum,
        max: maximum,
    })
}

fn hyper_polygon_newell_normal(vertices: &[Vertex]) -> Option<Vector3> {
    let points = vertices
        .iter()
        .map(|vertex| vertex.position.to_vector())
        .collect::<Vec<_>>();
    let normal = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .fold(Vector3::zero(), |acc, (current, next)| {
            acc + current.cross(next)
        });
    normal.normalize().ok()
}

pub(crate) fn finite_normalized_exact_rational(vector: &Vector3) -> Option<Vector3> {
    if vector
        .0
        .iter()
        .any(|component| component.exact_rational_ref().is_none())
    {
        return None;
    }
    let components = [
        vector.0[0].to_f64_lossy()?,
        vector.0[1].to_f64_lossy()?,
        vector.0[2].to_f64_lossy()?,
    ];
    let magnitude = (components[0] * components[0]
        + components[1] * components[1]
        + components[2] * components[2])
        .sqrt();
    if !magnitude.is_finite() || magnitude == 0.0 {
        return None;
    }
    Some(Vector3::new([
        Real::try_from(components[0] / magnitude).ok()?,
        Real::try_from(components[1] / magnitude).ok()?,
        Real::try_from(components[2] / magnitude).ok()?,
    ]))
}
