//! Struct and functions for working with planar `Polygon`s without holes

use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use hyperlattice::{Aabb, Point3, Real, Vector3};
use hyperreal::RealSign;
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
    /// projected coordinate is promoted to `Real`, and hypertri's
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
            let a = self.vertices[0].clone();
            let b = self.vertices[1].clone();
            let c = self.vertices[2].clone();

            return vec![[a, b, c]];
        }

        let Ok(normal_3d) = self.plane.normal().normalize_checked() else {
            return Vec::new();
        };
        let Ok((u, v)) = normal_3d.orthonormal_basis_checked() else {
            return Vec::new();
        };
        let origin_3d = self.vertices[0].position.clone();

        let n_verts = self.vertices.len();
        let mut points_2d = Vec::with_capacity(n_verts);
        for vert in &self.vertices {
            let offset = &vert.position - &origin_3d;
            let x = offset.dot(&u);
            let y = offset.dot(&v);
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

            let mut tri_vertices = [
                self.vertices[i0].clone(),
                self.vertices[i1].clone(),
                self.vertices[i2].clone(),
            ];
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
                poly_normal.dot(&plane_normal).refine_sign_until(128),
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TriangleOrientation {
    Aligned,
    Reversed,
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

/// Classify a triangulated face against the polygon normal with hyperreal predicates.
///
/// The winding normal is evaluated as `(b-a) x (c-a)` in
/// `hyperlattice::Vector3`; degenerate-area and orientation-sign decisions are
/// then made in `Real`. This keeps the post-`hypertri` topology
/// filter out of local f64 dot/cross comparisons, following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The triangulation context
/// follows Meisters' ear decomposition result, "Polygons Have Ears,"
/// *American Mathematical Monthly* 82(6), 1975
/// (<https://doi.org/10.2307/2319703>).
fn hyper_triangle_orientation(
    tri_vertices: &[Vertex; 3],
    polygon_normal: &Vector3,
) -> Option<TriangleOrientation> {
    let area2 = (&tri_vertices[1].position - &tri_vertices[0].position)
        .cross(&(&tri_vertices[2].position - &tri_vertices[0].position));
    if !matches!(
        area2.dot(&area2).refine_sign_until(128),
        Some(RealSign::Positive | RealSign::Negative)
    ) {
        return None;
    }

    let a = hyperlimit::Point3::new(
        tri_vertices[0].position.x.clone(),
        tri_vertices[0].position.y.clone(),
        tri_vertices[0].position.z.clone(),
    );
    let b = hyperlimit::Point3::new(
        tri_vertices[1].position.x.clone(),
        tri_vertices[1].position.y.clone(),
        tri_vertices[1].position.z.clone(),
    );
    let c = hyperlimit::Point3::new(
        tri_vertices[2].position.x.clone(),
        tri_vertices[2].position.y.clone(),
        tri_vertices[2].position.z.clone(),
    );
    let normal = hyperlimit::Point3::new(
        polygon_normal.0[0].clone(),
        polygon_normal.0[1].clone(),
        polygon_normal.0[2].clone(),
    );

    if matches!(
        hyperlimit::triangle3_winding_normal_sign(&a, &b, &c, &normal).value(),
        Some(hyperlimit::Sign::Negative)
    ) {
        Some(TriangleOrientation::Reversed)
    } else {
        Some(TriangleOrientation::Aligned)
    }
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

/// Given a normal vector `n`, build two perpendicular unit vectors `u` and `v`
/// so that {u, v, n} forms an orthonormal basis.
///
/// The checked normalization and cross products are delegated to
/// `hyperlattice::Vector3`; the returned vectors are finite mesh-boundary
/// values used by projection code. This follows Yap's exact-geometric-
/// computation boundary discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
pub fn build_orthonormal_basis(n: Vector3) -> (Vector3, Vector3) {
    if let Ok(basis) = n.orthonormal_basis_checked() {
        return basis;
    }

    (Vector3::x(), Vector3::y())
}

/// Helper function to subdivide a triangle into four smaller triangles.
pub fn subdivide_triangle(tri: [Vertex; 3]) -> [[Vertex; 3]; 4] {
    let half = (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
    let v01 = tri[0].interpolate(&tri[1], half.clone());
    let v12 = tri[1].interpolate(&tri[2], half.clone());
    let v20 = tri[2].interpolate(&tri[0], half);

    [
        [tri[0].clone(), v01.clone(), v20.clone()],
        [v01.clone(), tri[1].clone(), v12.clone()],
        [v20.clone(), v12.clone(), tri[2].clone()],
        [v01, v12, v20],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn hyper_triangle_orientation_classifies_aligned_and_reversed() {
        let normal = Vector3::z();
        let aligned = [
            Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(1.0, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(0.0, 1.0, 0.0), normal.clone()),
        ];
        let reversed = [aligned[0].clone(), aligned[2].clone(), aligned[1].clone()];

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
    fn hyper_triangle_orientation_accepts_tiny_nonzero_triangle() {
        let normal = Vector3::z();
        let tiny = [
            Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(1.0e-15, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(0.0, 1.0e-15, 0.0), normal.clone()),
        ];

        assert_eq!(
            hyper_triangle_orientation(&tiny, &normal),
            Some(TriangleOrientation::Aligned)
        );
    }

    #[test]
    fn hyper_triangle_orientation_rejects_exact_zero_area_triangle() {
        let normal = Vector3::z();
        let degenerate = [
            Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(1.0, 0.0, 0.0), normal.clone()),
            Vertex::new(p3(2.0, 0.0, 0.0), normal.clone()),
        ];

        assert_eq!(hyper_triangle_orientation(&degenerate, &normal), None);
    }
}
