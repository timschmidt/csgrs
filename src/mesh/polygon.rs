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
            let mut mins = Point3::new(Real::MAX, Real::MAX, Real::MAX);
            let mut maxs = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);
            for v in &self.vertices {
                mins.x = mins.x.min(v.pos.x);
                mins.y = mins.y.min(v.pos.y);
                mins.z = mins.z.min(v.pos.z);
                maxs.x = maxs.x.max(v.pos.x);
                maxs.y = maxs.y.max(v.pos.y);
                maxs.z = maxs.z.max(v.pos.z);
            }
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

    /// Rotate polygons into 2D to perform triangulation, then rotate triangles back to original 3D position
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to tessellate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        let normal_3d = self.plane.normal().normalize();
        let (u, v) = build_orthonormal_basis(normal_3d);
        let origin_3d = self.vertices[0].pos;

        #[cfg(feature = "earcut")]
        {
            // Flatten each vertex to 2D
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                all_vertices_2d.push(coord! {x: x, y: y});
            }

            use geo::TriangulateEarcut;
            let triangulation = GeoPolygon::new(LineString::new(all_vertices_2d), Vec::new())
                .earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert back into 3D triangles
            let mut triangles = Vec::with_capacity(triangle_indices.len() / 3);
            for tri_chunk in triangle_indices.chunks_exact(3) {
                let mut tri_vertices = [const {
                    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0))
                }; 3];
                for (k, &idx) in tri_chunk.iter().enumerate() {
                    let base = idx * 2;
                    let x = vertices[base];
                    let y = vertices[base + 1];
                    let pos_3d = origin_3d.coords + (x * u) + (y * v);
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }
                triangles.push(tri_vertices);
            }
            triangles
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;

            // Flatten each vertex to 2D
            // Here we clamp values within spade's minimum allowed value of  0.0 to 0.0
            // because spade refuses to triangulate with values within it's minimum:
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43; // 1.0 * 2^-142
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                let y = offset.dot(&v);
                let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };

                // test for NaN/±∞
                if !(x.is_finite()
                    && y.is_finite()
                    && x_clamped.is_finite()
                    && y_clamped.is_finite())
                {
                    // at least one coordinate was NaN/±∞ – ignore this triangle
                    continue;
                }
                all_vertices_2d.push(coord! {x: x_clamped, y: y_clamped});
            }

            let polygon_2d = GeoPolygon::new(
                LineString::new(all_vertices_2d),
                // no holes if your polygon is always simple
                Vec::new(),
            );
            let Ok(tris) = polygon_2d.constrained_triangulation(Default::default()) else {
                return Vec::new();
            };

            let mut final_triangles = Vec::with_capacity(tris.len());
            for tri2d in tris {
                // tri2d is a geo::Triangle in 2D
                // Convert each corner from (x,y) to 3D again
                let [coord_a, coord_b, coord_c] = [tri2d.0, tri2d.1, tri2d.2];
                let pos_a_3d = origin_3d.coords + coord_a.x * u + coord_a.y * v;
                let pos_b_3d = origin_3d.coords + coord_b.x * u + coord_b.y * v;
                let pos_c_3d = origin_3d.coords + coord_c.x * u + coord_c.y * v;

                final_triangles.push([
                    Vertex::new(Point3::from(pos_a_3d), normal_3d),
                    Vertex::new(Point3::from(pos_b_3d), normal_3d),
                    Vertex::new(Point3::from(pos_c_3d), normal_3d),
                ]);
            }
            final_triangles
        }
    }

    /// Subdivide this polygon into smaller triangles.
    /// Returns a list of new triangles (each is a [Vertex; 3]).
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
                let mut next_level = Vec::new();
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

    /// return a normal calculated from all polygon vertices
    pub fn calculate_new_normal(&self) -> Vector3<Real> {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        let mut points = Vec::new();
        for vertex in &self.vertices {
            points.push(vertex.pos);
        }
        let mut normal = Vector3::zeros();

        // Loop over each edge of the polygon.
        for i in 0..n {
            let current = points[i];
            let next = points[(i + 1) % n]; // wrap around using modulo
            normal.x += (current.y - next.y) * (current.z + next.z);
            normal.y += (current.z - next.z) * (current.x + next.x);
            normal.z += (current.x - next.x) * (current.y + next.y);
        }

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
        // Assign each vertex’s normal to match the plane
        let new_normal = self.calculate_new_normal();
        for v in &mut self.vertices {
            v.normal = new_normal;
        }
    }

    /// Returns a reference to the metadata, if any.
    pub fn metadata(&self) -> Option<&S> {
        self.metadata.as_ref()
    }

    /// Returns a mutable reference to the metadata, if any.
    pub fn metadata_mut(&mut self) -> Option<&mut S> {
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
