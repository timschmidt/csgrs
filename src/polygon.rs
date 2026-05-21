//! Struct and functions for working with planar `Polygon`s without holes

use crate::float_types::{
    Real, hperpendicular_basis, hpoint3_bounds, hreal_from_f64, hreal_gt_f64, hreal_lt_f64,
    hunit_vector3, hvector3_dot, hvector3_from_point3, hvector3_from_vector3,
    parry3d::bounding_volume::Aabb, tolerance,
};
use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use hyperlattice::Vector3 as HVector3;
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

/// A polygon, defined by a list of vertices.
/// - `M` is the generic metadata type stored directly on the polygon. Use
///   `M = ()` for no metadata, or `M = Option<YourMetadata>` for optional metadata.
#[derive(Debug, Clone)]
pub struct Polygon<M: Clone> {
    /// Vertices defining the Polygon's shape
    pub vertices: Vec<Vertex>,

    /// The plane on which this Polygon lies, used for splitting
    pub plane: Plane,

    /// Lazily‑computed axis‑aligned bounding box of the Polygon
    pub bounding_box: OnceLock<Aabb>,

    /// Generic metadata associated with the Polygon
    pub metadata: M,
}

impl<M: Clone + PartialEq> PartialEq for Polygon<M> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

#[allow(unused)]
impl<M: Clone + Send + Sync + PartialEq> Polygon<M> {
    fn same_metadata(&self, metadata: &M) -> bool {
        &self.metadata == metadata
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
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Return this polygon with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync>(self, metadata: NewM) -> Polygon<NewM> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Map this polygon's metadata while preserving its geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync, F>(self, f: F) -> Polygon<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
    pub fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let points = self
                .vertices
                .iter()
                .map(|vertex| vertex.position)
                .collect::<Vec<_>>();
            let Some((mins, maxs)) = hpoint3_bounds(&points) else {
                return Aabb::new(Point3::origin(), Point3::origin());
            };
            Aabb::new(mins, maxs)
        })
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

    /// Triangulate this polygon into `[v0, v1, v2]` triangles with hypertri.
    ///
    /// The polygon is projected from its 3D support plane to 2D, each finite
    /// projected coordinate is promoted to `hyperreal::Real`, and hypertri's
    /// exact ear-clipping path decides topology. The projection is a boundary
    /// conversion; all irreversible 2D orientation and containment predicates
    /// are then made in hyperreal space. This follows Yap's exact geometric
    /// computation contract (Computational Geometry 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>) and the ear theorem
    /// foundation from Meisters, "Polygons Have Ears" (American Mathematical
    /// Monthly 82(6), 1975, <https://doi.org/10.2307/2319703>).
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to tessellate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        // A polygon that is already a triangle: no need to call hypertri.
        // Returning it directly avoids robustness problems with very thin
        // triangles and makes the fast-path cheaper.
        if self.vertices.len() == 3 {
            let a = self.vertices[0];
            let b = self.vertices[1];
            let c = self.vertices[2];

            return vec![[a, b, c]];
        }

        let Some(normal_3d) = hunit_vector3(&self.plane.normal()) else {
            return Vec::new();
        };
        let Some((u, v)) = hperpendicular_basis(&normal_3d) else {
            return Vec::new();
        };
        let origin_3d = self.vertices[0].position;

        let n_verts = self.vertices.len();
        let mut points_2d = Vec::with_capacity(n_verts);
        for vert in &self.vertices {
            let offset = vert.position.coords - origin_3d.coords;
            let Some(x) = hvector3_dot(&offset, &u) else {
                return Vec::new();
            };
            let Some(y) = hvector3_dot(&offset, &v) else {
                return Vec::new();
            };
            let Ok(x) = hreal_from_f64(x) else {
                return Vec::new();
            };
            let Ok(y) = hreal_from_f64(y) else {
                return Vec::new();
            };
            points_2d.push(hypertri::Point2::new(x, y));
        }

        let indices = match hypertri::earcut(&points_2d, &[]) {
            Ok(indices) => indices,
            Err(_) => return Vec::new(),
        };

        let mut triangles = Vec::with_capacity(indices.len() / 3);
        for tri in indices.chunks_exact(3) {
            let [i0, i1, i2] = [tri[0], tri[1], tri[2]];
            if i0 >= n_verts || i1 >= n_verts || i2 >= n_verts {
                continue;
            }

            let mut tri_vertices = [self.vertices[i0], self.vertices[i1], self.vertices[i2]];
            let Some(orientation) = hyper_triangle_orientation(&tri_vertices, &normal_3d)
            else {
                continue;
            };
            if orientation == TriangleOrientation::Reversed {
                tri_vertices.swap(1, 2);
            }
            triangles.push(tri_vertices);
        }

        triangles
    }

    /// **Mathematical Foundation: Triangle Subdivision for Mesh Refinement**
    ///
    /// Subdivide this polygon into smaller triangles using recursive triangle splitting.
    /// This implements the mathematical theory of uniform mesh refinement:
    ///
    /// ## **Subdivision Algorithm**
    ///
    /// ### **Base Triangulation**
    /// 1. **Initial Tessellation**: Convert polygon to base triangles using tessellate()
    /// 2. **Triangle Count**: n base triangles from polygon
    ///
    /// ### **Recursive Subdivision**
    /// For each subdivision level, each triangle T is split into 4 smaller triangles:
    /// ```text
    /// Original Triangle:     Subdivided Triangle:
    ///        A                        A
    ///       /\                      /\ \
    ///      /  \                    /  \ \
    ///     /____\                  M₁___M₂ \
    ///    B      C                /\    /\ \
    ///                           /  \  /  \ \
    ///                          /____\/____\
    ///                         B     M₃     C
    /// ```
    ///
    /// ### **Midpoint Calculation**
    /// For triangle vertices (A, B, C):
    /// - **M₁ = midpoint(A,B)**: Linear interpolation at t=0.5
    /// - **M₂ = midpoint(A,C)**: Linear interpolation at t=0.5  
    /// - **M₃ = midpoint(B,C)**: Linear interpolation at t=0.5
    ///
    /// ### **Subdivision Pattern**
    /// Creates 4 congruent triangles:
    /// 1. **Corner triangles**: (A,M₁,M₂), (M₁,B,M₃), (M₂,M₃,C)
    /// 2. **Center triangle**: (M₁,M₂,M₃)
    ///
    /// ## **Mathematical Properties**
    /// - **Area Preservation**: Total area remains constant
    /// - **Similarity**: All subtriangles are similar to original
    /// - **Scaling Factor**: Each subtriangle has 1/4 the area
    /// - **Growth Rate**: Triangle count × 4ᵏ after k subdivisions
    /// - **Smoothness**: C¹ continuity maintained across edges
    ///
    /// ## **Applications**
    /// - **Level of Detail**: Adaptive mesh resolution
    /// - **Smooth Surfaces**: Approximating curved surfaces with flat triangles
    /// - **Numerical Methods**: Finite element mesh refinement
    /// - **Rendering**: Progressive mesh detail for distance-based LOD
    ///
    /// Returns a list of refined triangles (each is a [Vertex; 3]).
    /// For polygon applications, these can be converted back to triangular polygons.
    pub fn subdivide_triangles(
        &self,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<[Vertex; 3]> {
        // 1) Triangulate the polygon as it is.
        let base_tris = self.triangulate();

        // 2) For each triangle, subdivide 'subdivisions' times.
        let mut result = Vec::new();
        for tri in base_tris {
            // We'll keep a queue of triangles to process
            let mut queue = vec![tri];
            for _ in 0..subdivisions.get() {
                let mut next_level = Vec::with_capacity(queue.len() * 4);
                for t in queue {
                    let subs = subdivide_triangle(t);
                    next_level.extend(subs);
                }
                queue = next_level;
            }
            result.extend(queue);
        }

        result // todo: return polygons
    }

    /// Convert subdivision triangles back to polygons for CSG operations
    /// Each triangle becomes a triangular polygon with the same metadata
    pub fn subdivide_to_polygons(
        &self,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<Polygon<M>> {
        self.subdivide_triangles(subdivisions)
            .into_iter()
            .map(|tri| {
                let vertices = tri.to_vec();
                Polygon::new(vertices, self.metadata.clone())
            })
            .collect()
    }

    /// Return a normal calculated from all polygon vertices.
    ///
    /// The Newell-style accumulated area normal is evaluated with
    /// `hyperlattice::Vector3<hyperreal::Real>` and checked-normalized before
    /// export to the current finite mesh boundary type. This keeps polygon
    /// normal recomputation on the same exact-geometric-computation boundary
    /// model used by plane predicates (Yap, *Computational Geometry* 7(1-2),
    /// 1997, <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn calculate_new_normal(&self) -> Vector3<Real> {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        if let Some(mut poly_normal) = hyper_polygon_newell_normal(&self.vertices) {
            let plane_normal = self.plane.normal();
            if let (Some(poly_h), Some(plane_h)) = (
                hvector3_from_vector3(&poly_normal),
                hvector3_from_vector3(&plane_normal),
            ) {
                if hreal_lt_f64(&poly_h.dot(&plane_h), 0.0) {
                    poly_normal = -poly_normal;
                }
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
            v.normal = new_normal;
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TriangleOrientation {
    Aligned,
    Reversed,
}

/// Classify a triangulated face against the polygon normal with hyperreal predicates.
///
/// The winding normal is evaluated as `(b-a) x (c-a)` in
/// `hyperlattice::Vector3`; degenerate-area and orientation-sign decisions are
/// then made in `hyperreal::Real`. This keeps the post-`hypertri` topology
/// filter out of local f64 dot/cross comparisons, following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The triangulation context
/// follows Meisters' ear decomposition result, "Polygons Have Ears,"
/// *American Mathematical Monthly* 82(6), 1975
/// (<https://doi.org/10.2307/2319703>).
fn hyper_triangle_orientation(
    tri_vertices: &[Vertex; 3],
    polygon_normal: &Vector3<Real>,
) -> Option<TriangleOrientation> {
    let a = hvector3_from_point3(&tri_vertices[0].position)?;
    let b = hvector3_from_point3(&tri_vertices[1].position)?;
    let c = hvector3_from_point3(&tri_vertices[2].position)?;

    let edge1 = b - &a;
    let edge2 = c - a;
    let winding_normal = edge1.cross(&edge2);
    if !hreal_gt_f64(
        &winding_normal.dot(&winding_normal),
        tolerance() * tolerance(),
    ) {
        return None;
    }

    let a = hyperlimit::Point3::try_from_f64_array([
        tri_vertices[0].position.x,
        tri_vertices[0].position.y,
        tri_vertices[0].position.z,
    ])
    .ok()?;
    let b = hyperlimit::Point3::try_from_f64_array([
        tri_vertices[1].position.x,
        tri_vertices[1].position.y,
        tri_vertices[1].position.z,
    ])
    .ok()?;
    let c = hyperlimit::Point3::try_from_f64_array([
        tri_vertices[2].position.x,
        tri_vertices[2].position.y,
        tri_vertices[2].position.z,
    ])
    .ok()?;
    let normal = hyperlimit::Point3::try_from_f64_array([
        polygon_normal.x,
        polygon_normal.y,
        polygon_normal.z,
    ])
    .ok()?;

    if matches!(
        hyperlimit::triangle3_winding_normal_sign(&a, &b, &c, &normal).value(),
        Some(hyperlimit::Sign::Negative)
    ) {
        Some(TriangleOrientation::Reversed)
    } else {
        Some(TriangleOrientation::Aligned)
    }
}

fn hyper_polygon_newell_normal(vertices: &[Vertex]) -> Option<Vector3<Real>> {
    let points = vertices
        .iter()
        .map(|vertex| hvector3_from_point3(&vertex.position))
        .collect::<Option<Vec<_>>>()?;
    let normal = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .fold(HVector3::zero(), |acc, (current, next)| {
            acc + current.cross(next)
        });
    let normal = normal.normalize_checked().ok()?;
    let [x, y, z] = normal.to_f64_array_lossy()?;
    Some(Vector3::new(x, y, z))
}

/// Given a normal vector `n`, build two perpendicular unit vectors `u` and `v`
/// so that {u, v, n} forms an orthonormal basis.
///
/// The checked normalization and cross products are delegated to
/// `hyperlattice::Vector3`; the returned vectors are finite mesh-boundary
/// values used by projection code. This follows Yap's exact-geometric-
/// computation boundary discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub fn build_orthonormal_basis(n: Vector3<Real>) -> (Vector3<Real>, Vector3<Real>) {
    if let Some(basis) = hperpendicular_basis(&n) {
        return basis;
    }

    (Vector3::x(), Vector3::y())
}

/// Helper function to subdivide a triangle into four smaller triangles.
pub fn subdivide_triangle(tri: [Vertex; 3]) -> [[Vertex; 3]; 4] {
    let v01 = tri[0].interpolate(&tri[1], 0.5);
    let v12 = tri[1].interpolate(&tri[2], 0.5);
    let v20 = tri[2].interpolate(&tri[0], 0.5);

    [
        [tri[0], v01, v20],
        [v01, tri[1], v12],
        [v20, v12, tri[2]],
        [v01, v12, v20],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hyper_triangle_orientation_classifies_aligned_and_reversed() {
        let normal = Vector3::z();
        let aligned = [
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), normal),
        ];
        let reversed = [aligned[0], aligned[2], aligned[1]];

        assert_eq!(
            hyper_triangle_orientation(&aligned, &normal),
            Some(TriangleOrientation::Aligned)
        );
        assert_eq!(
            hyper_triangle_orientation(&reversed, &normal),
            Some(TriangleOrientation::Reversed)
        );
    }

    #[test]
    fn hyper_triangle_orientation_rejects_near_degenerate_triangle() {
        let normal = Vector3::z();
        let degenerate = [
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(tolerance() * 0.25, 0.0, 0.0), normal),
            Vertex::new(Point3::new(0.0, tolerance() * 0.25, 0.0), normal),
        ];

        assert_eq!(hyper_triangle_orientation(&degenerate, &normal), None);
    }
}
