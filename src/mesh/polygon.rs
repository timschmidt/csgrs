//! Struct and functions for working with planar `Polygon`s without holes

use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
use crate::mesh::plane::Plane;
use crate::mesh::vertex::Vertex;
use geo::{LineString, Polygon as GeoPolygon, coord};
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

/// A polygon, defined by a list of vertices.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    /// Vertices defining the Polygon's shape
    pub vertices: Vec<Vertex>,

    /// The plane on which this Polygon lies, used for splitting
    pub plane: Plane,

    /// Lazily‑computed axis‑aligned bounding box of the Polygon
    pub bounding_box: OnceLock<Aabb>,

    /// Generic metadata associated with the Polygon
    pub metadata: Option<S>,
}

impl<S: Clone + PartialEq> PartialEq for Polygon<S> {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
            && self.plane == other.plane
            && self.metadata == other.metadata
    }
}

#[allow(unused)]
impl<S: Clone + Send + Sync + PartialEq> Polygon<S> {
    fn same_metadata(&self, metadata: Option<S>) -> bool {
        self.metadata == metadata
    }
}

impl<S: Clone + Send + Sync> Polygon<S> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");

        let plane = Plane::from_vertices(vertices.clone());

        Polygon {
            vertices,
            plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Axis aligned bounding box of this Polygon (cached after first call)
    pub fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let (mins, maxs) = self.vertices.iter().fold(
                (Point3::new(Real::MAX, Real::MAX, Real::MAX), Point3::new(-Real::MAX, -Real::MAX, -Real::MAX)),
                |(mut mins, mut maxs), v| {
                    mins.x = mins.x.min(v.pos.x);
                    mins.y = mins.y.min(v.pos.y);
                    mins.z = mins.z.min(v.pos.z);
                    maxs.x = maxs.x.max(v.pos.x);
                    maxs.y = maxs.y.max(v.pos.y);
                    maxs.z = maxs.z.max(v.pos.z);
                    (mins, maxs)
                }
            );
            Aabb::new(mins, maxs)
        })
    }

    /// Reverses winding order, flips vertices normals, and flips the plane normal
    pub fn flip(&mut self) {
        // 1) reverse vertices
        self.vertices.reverse();
        // 2) flip all vertex normals
        self.vertices.iter_mut().for_each(|v| v.flip());
        // 3) flip the cached plane too
        self.plane.flip();
    }

    /// Return an iterator over paired vertices each forming an edge of the polygon
    pub fn edges(&self) -> impl Iterator<Item = (&Vertex, &Vertex)> {
        self.vertices.iter().zip(self.vertices.iter().cycle().skip(1))
    }

    /// **Mathematical Foundation: Polygon Triangulation**
    ///
    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    /// This implements robust 2D triangulation algorithms for 3D planar polygons.
    ///
    /// ## **Algorithmic Approaches**
    ///
    /// ### **Ear Clipping (Earcut)**
    /// **Algorithm**: Based on the "ear removal" theorem:
    /// - **Ear Definition**: A triangle formed by three consecutive vertices with no other vertices inside
    /// - **Theorem**: Every simple polygon with n > 3 vertices has at least two ears
    /// - **Complexity**: O(n²) worst case, O(n) for most practical polygons
    /// - **Robustness**: Handles arbitrary simple polygons including concave shapes
    ///
    /// ### **Delaunay Triangulation (Spade)**
    /// **Algorithm**: Based on maximizing minimum angles:
    /// - **Delaunay Property**: No vertex lies inside circumcircle of any triangle
    /// - **Complexity**: O(n log n) expected time
    /// - **Quality**: Produces well-shaped triangles, avoids slivers
    /// - **Constraints**: Maintains polygon boundary as constraint edges
    ///
    /// ## **3D to 2D Projection**
    /// The algorithm projects the 3D planar polygon to 2D:
    /// 1. **Orthonormal Basis**: Compute basis vectors {u⃗, v⃗} in the plane
    /// 2. **Projection**: For each vertex pᵢ: (x,y) = ((pᵢ-p₀)·u⃗, (pᵢ-p₀)·v⃗)
    /// 3. **Triangulation**: Apply 2D algorithm to projected coordinates
    /// 4. **Reconstruction**: Map 2D triangles back to 3D using inverse projection
    ///
    /// ## **Numerical Considerations**
    /// - **Degeneracy Handling**: Filters out near-zero coordinates for stability
    /// - **Precision Limits**: Spade enforces minimum coordinate values
    /// - **Normal Preservation**: All output triangles maintain original plane normal
    ///
    /// The choice between algorithms depends on build features:
    /// - `earcut`: Fast for simple polygons, handles concave shapes
    /// - `delaunay`: Better triangle quality, more robust for complex geometry
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to tessellate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        // A polygon that is already a triangle: no need to call earcut/spade.
        // Returning it directly avoids robustness problems with very thin
        // triangles and makes the fast-path cheaper.
        if self.vertices.len() == 3 {
            return vec![[
                self.vertices[0].clone(),
                self.vertices[1].clone(),
                self.vertices[2].clone(),
            ]];
        }

        let normal_3d = self.plane.normal().normalize();
        let (u, v) = build_orthonormal_basis(normal_3d);
        let origin_3d = self.vertices[0].pos;

        #[cfg(feature = "earcut")]
        {
            // Flatten each vertex to 2D
            let all_vertices_2d: Vec<_> = self.vertices.iter().map(|vert| {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                coord! {x: x, y: y}
            }).collect();

            use geo::TriangulateEarcut;
            let triangulation = GeoPolygon::new(LineString::new(all_vertices_2d), Vec::new())
                .earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert back into 3D triangles
            triangle_indices.chunks_exact(3).map(|tri_chunk| {
                let tri_vertices: [Vertex; 3] = tri_chunk.iter().enumerate().map(|(k, &idx)| {
                    let base = idx * 2;
                    let x = vertices[base];
                    let y = vertices[base + 1];
                    let pos_3d = origin_3d.coords + (x * u) + (y * v);
                    Vertex::new(Point3::from(pos_3d), normal_3d)
                }).collect::<Vec<_>>().try_into().unwrap();
                tri_vertices
            }).collect()
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;

            // Flatten each vertex to 2D
            // Here we clamp values within spade's minimum allowed value of  0.0 to 0.0
            // because spade refuses to triangulate with values within it's minimum:
            #[allow(clippy::excessive_precision)]
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43; // 1.0 * 2^-142
            let all_vertices_2d: Vec<_> = self.vertices.iter().filter_map(|vert| {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                let y = offset.dot(&v);
                let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };

                // test for NaN/±∞
                if x.is_finite()
                    && y.is_finite()
                    && x_clamped.is_finite()
                    && y_clamped.is_finite()
                {
                    Some(coord! {x: x_clamped, y: y_clamped})
                } else {
                    // at least one coordinate was NaN/±∞ – ignore this triangle
                    None
                }
            }).collect();

            let polygon_2d = GeoPolygon::new(
                LineString::new(all_vertices_2d),
                // no holes if your polygon is always simple
                Vec::new(),
            );
            let Ok(tris) = polygon_2d.constrained_triangulation(Default::default()) else {
                return Vec::new();
            };

            tris.into_iter().map(|tri2d| {
                // tri2d is a geo::Triangle in 2D
                // Convert each corner from (x,y) to 3D again
                let [coord_a, coord_b, coord_c] = [tri2d.0, tri2d.1, tri2d.2];
                let pos_a_3d = origin_3d.coords + coord_a.x * u + coord_a.y * v;
                let pos_b_3d = origin_3d.coords + coord_b.x * u + coord_b.y * v;
                let pos_c_3d = origin_3d.coords + coord_c.x * u + coord_c.y * v;

                [
                    Vertex::new(Point3::from(pos_a_3d), normal_3d),
                    Vertex::new(Point3::from(pos_b_3d), normal_3d),
                    Vertex::new(Point3::from(pos_c_3d), normal_3d),
                ]
            }).collect()
        }
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
        base_tris.into_iter().flat_map(|tri| {
            // We'll keep a queue of triangles to process
            (0..subdivisions.get()).fold(vec![tri], |queue, _| {
                queue.into_iter().flat_map(|t| {
                    subdivide_triangle(t)
                }).collect()
            })
        }).collect() // todo: return polygons
    }

    /// Convert subdivision triangles back to polygons for CSG operations
    /// Each triangle becomes a triangular polygon with the same metadata
    pub fn subdivide_to_polygons(
        &self,
        subdivisions: core::num::NonZeroU32,
    ) -> Vec<Polygon<S>> {
        self.subdivide_triangles(subdivisions)
            .into_iter()
            .map(|tri| {
                let vertices = tri.to_vec();
                Polygon::new(vertices, self.metadata.clone())
            })
            .collect()
    }

    /// return a normal calculated from all polygon vertices
    pub fn calculate_new_normal(&self) -> Vector3<Real> {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        let points: Vec<_> = self.vertices.iter().map(|vertex| vertex.pos).collect();
        let mut normal = Vector3::zeros();

        // Loop over each edge of the polygon.
        normal = (0..n).fold(normal, |mut acc, i| {
            let current = points[i];
            let next = points[(i + 1) % n]; // wrap around using modulo
            acc.x += (current.y - next.y) * (current.z + next.z);
            acc.y += (current.z - next.z) * (current.x + next.x);
            acc.z += (current.x - next.x) * (current.y + next.y);
            acc
        });

        // Normalize the computed normal.
        let mut poly_normal = normal.normalize();

        // Ensure the computed normal is in the same direction as the given normal.
        if poly_normal.dot(&self.plane.normal()) < 0.0 {
            poly_normal = -poly_normal;
        }

        poly_normal
    }

    /// Recompute this polygon's normal from all vertices, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        // Assign each vertex's normal to match the plane
        let new_normal = self.calculate_new_normal();
        self.vertices.iter_mut().for_each(|v| {
            v.normal = new_normal;
        });
    }

    /// Returns a reference to the metadata, if any.
    pub const fn metadata(&self) -> Option<&S> {
        self.metadata.as_ref()
    }

    /// Returns a mutable reference to the metadata, if any.
    pub const fn metadata_mut(&mut self) -> Option<&mut S> {
        self.metadata.as_mut()
    }

    /// Sets the metadata to the given value.
    pub fn set_metadata(&mut self, data: S) {
        self.metadata = Some(data);
    }
}

/// Given a normal vector `n`, build two perpendicular unit vectors `u` and `v` so that
/// {u, v, n} forms an orthonormal basis. `n` is assumed non‐zero.
pub fn build_orthonormal_basis(n: Vector3<Real>) -> (Vector3<Real>, Vector3<Real>) {
    // Normalize the given normal
    let n = n.normalize();

    // Pick a vector that is not parallel to `n`. For instance, pick the axis
    // which has the smallest absolute component in `n`, and cross from there.
    // Because crossing with that is least likely to cause numeric issues.
    let other = if n.x.abs() < n.y.abs() && n.x.abs() < n.z.abs() {
        Vector3::x()
    } else if n.y.abs() < n.z.abs() {
        Vector3::y()
    } else {
        Vector3::z()
    };

    // v = n × other
    let v = n.cross(&other).normalize();
    // u = v × n
    let u = v.cross(&n).normalize();

    (u, v)
}

// Helper function to subdivide a triangle
pub fn subdivide_triangle(tri: [Vertex; 3]) -> Vec<[Vertex; 3]> {
    let v01 = tri[0].interpolate(&tri[1], 0.5);
    let v12 = tri[1].interpolate(&tri[2], 0.5);
    let v20 = tri[2].interpolate(&tri[0], 0.5);

    vec![
        [tri[0].clone(), v01.clone(), v20.clone()],
        [v01.clone(), tri[1].clone(), v12.clone()],
        [v20.clone(), v12.clone(), tri[2].clone()],
        [v01, v12, v20],
    ]
}
