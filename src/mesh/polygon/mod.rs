//! Planar n-gon face storage and derived geometry.

use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use hyperlattice::{Aabb, Point3, Vector3};
use hyperreal::RealSign;
use std::ops::{Deref, DerefMut};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_PLANE_ID: AtomicU64 = AtomicU64::new(1);

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
    pub(crate) metadata: M,
}

/// Mutable access to a polygon's existing vertices.
///
/// Dropping this guard refreshes geometry derived from vertex positions.
pub struct PolygonVerticesMut<'a, M: Clone + Send + Sync> {
    polygon: &'a mut Polygon<M>,
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

        let plane = Plane::from_vertices(vertices.clone());

        Polygon {
            vertices,
            plane,
            plane_id: fresh_plane_id(),
            bounding_box: OnceLock::new(),
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
            metadata: f(self.metadata),
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
    pub fn bounding_box(&self) -> Aabb {
        self.bounding_box
            .get_or_init(|| {
                let points = self
                    .vertices
                    .iter()
                    .map(|vertex| vertex.position.clone())
                    .collect::<Vec<_>>();
                let Some((mins, maxs)) = point3_bounds(&points) else {
                    return Aabb::origin();
                };
                Aabb::new(mins, maxs)
            })
            .clone()
    }

    /// Vertices in winding order.
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    /// Mutably access vertex values while preserving derived geometry.
    ///
    /// The slice has a fixed length, so the polygon cannot become degenerate
    /// through this API. Its plane and cached bounds are refreshed on drop.
    pub const fn vertices_mut(&mut self) -> PolygonVerticesMut<'_, M> {
        PolygonVerticesMut { polygon: self }
    }

    /// Plane derived from the current vertices.
    pub const fn plane(&self) -> &Plane {
        &self.plane
    }

    fn refresh_geometry(&mut self) {
        self.plane = Plane::from_vertices(self.vertices.clone());
        self.bounding_box = OnceLock::new();
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
    }

    /// Return an iterator over paired vertices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// Return a normal calculated from all polygon vertices.
    ///
    /// The Newell-style accumulated area normal is evaluated with
    /// `hyperlattice::Vector3` and checked-normalized before
    /// export to the current finite mesh boundary type. This keeps polygon
    /// normal recomputation on the same exact-geometric-computation boundary
    /// model used by plane predicates (Yap, *Computational Geometry* 7(1-2),
    /// 1997, <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn calculate_new_normal(&self) -> Vector3 {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        if let Some(mut poly_normal) = hyper_polygon_newell_normal(&self.vertices) {
            let plane_normal = self.plane.normal();
            if matches!(
                poly_normal.dot(&plane_normal).refine_sign_until(-128),
                Some(RealSign::Negative)
            ) {
                poly_normal = -poly_normal;
            }
            return poly_normal;
        }

        Vector3::z()
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

fn point3_bounds(points: &[Point3]) -> Option<(Point3, Point3)> {
    let first = points.first()?;
    let mut min_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut min_z = first.z.clone();
    let mut max_x = min_x.clone();
    let mut max_y = min_y.clone();
    let mut max_z = min_z.clone();

    for point in &points[1..] {
        min_x = hyperlimit::real_min(&min_x, &point.x).value()?.clone();
        min_y = hyperlimit::real_min(&min_y, &point.y).value()?.clone();
        min_z = hyperlimit::real_min(&min_z, &point.z).value()?.clone();
        max_x = hyperlimit::real_max(&max_x, &point.x).value()?.clone();
        max_y = hyperlimit::real_max(&max_y, &point.y).value()?.clone();
        max_z = hyperlimit::real_max(&max_z, &point.z).value()?.clone();
    }

    Some((
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    ))
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
    normal.normalize_checked().ok()
}
