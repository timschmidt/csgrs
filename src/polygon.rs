//! Struct and functions for working with planar `Polygon`s without holes

use crate::float_types::{Real, parry3d::bounding_volume::Aabb, tolerance};
use crate::mesh::plane::Plane;
use crate::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::sync::OnceLock;

/// A polygon, defined by a list of vertices.
/// - `M` is the generic metadata type stored directly on the polygon. Use
///   `M = ()` for no metadata, or `M = Option<T>` for optional metadata.
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
    pub fn with_metadata<T: Clone + Send + Sync>(self, metadata: T) -> Polygon<T> {
        Polygon {
            vertices: self.vertices,
            plane: self.plane,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Map this polygon's metadata while preserving its geometry.
    pub fn map_metadata<T: Clone + Send + Sync, F>(self, f: F) -> Polygon<T>
    where
        F: FnOnce(M) -> T,
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
            let mut mins = Point3::new(Real::MAX, Real::MAX, Real::MAX);
            let mut maxs = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);
            for v in &self.vertices {
                mins.x = mins.x.min(v.position.x);
                mins.y = mins.y.min(v.position.y);
                mins.z = mins.z.min(v.position.z);
                maxs.x = maxs.x.max(v.position.x);
                maxs.y = maxs.y.max(v.position.y);
                maxs.z = maxs.z.max(v.position.z);
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
    /// - `delaunay-rs`: Point-set Delaunay triangulation filtered to the polygon interior
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to tessellate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        // A polygon that is already a triangle: no need to call earcut/spade.
        // Returning it directly avoids robustness problems with very thin
        // triangles and makes the fast-path cheaper.
        if self.vertices.len() == 3 {
            let a = self.vertices[0];
            let b = self.vertices[1];
            let c = self.vertices[2];

            return vec![[a, b, c]];
        }

        #[cfg(feature = "delaunay-rs")]
        if self.vertices.len() == 4 {
            let a = self.vertices[0];
            let b = self.vertices[1];
            let c = self.vertices[2];
            let d = self.vertices[3];

            return vec![[a, b, c], [a, c, d]];
        }

        let raw_normal = self.plane.normal();
        let normal_len = raw_normal.norm();
        if normal_len < tolerance() || !normal_len.is_finite() {
            return Vec::new();
        }

        let normal_3d = raw_normal / normal_len;
        if !normal_3d.x.is_finite() || !normal_3d.y.is_finite() || !normal_3d.z.is_finite() {
            return Vec::new();
        }

        let (u, v) = build_orthonormal_basis(normal_3d);
        let origin_3d = self.vertices[0].position;

        #[cfg(feature = "earcut")]
        {
            use earcutr::earcut;

            let n_verts = self.vertices.len();

            // 1. Build flattened 2D coordinates, in the same order as `self.vertices`.
            let mut flat_2d = Vec::with_capacity(n_verts * 2);
            for vert in &self.vertices {
                let offset = vert.position.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                if !x.is_finite() || !y.is_finite() {
                    return Vec::new();
                }
                flat_2d.push(x);
                flat_2d.push(y);
            }

            // 2. Run earcut: indices are into `flat_2d` as (x0, y0, x1, y1, …).
            let holes: Vec<usize> = Vec::new(); // you said: no holes
            let indices = match earcut(&flat_2d, &holes, 2) {
                Ok(indices) => indices,
                Err(_) => return Vec::new(),
            };

            // 3. Build triangles using *original* vertices.
            let mut triangles = Vec::with_capacity(indices.len() / 3);
            for tri in indices.chunks_exact(3) {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;

                if i0 >= n_verts || i1 >= n_verts || i2 >= n_verts {
                    continue;
                }

                let a = self.vertices[i0];
                let b = self.vertices[i1];
                let c = self.vertices[i2];

                let edge1 = b.position - a.position;
                let edge2 = c.position - a.position;
                if edge1.cross(&edge2).norm_squared() < tolerance() * tolerance() {
                    continue;
                }

                triangles.push([a, b, c]);
            }

            triangles
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::coord;
            use spade::{
                AngleLimit, ConstrainedDelaunayTriangulation, Point2 as SpadePoint2,
                RefinementParameters, Triangulation as SpadeTriangulation,
            };

            // Flatten each vertex to 2D
            // Here we clamp values within spade's minimum allowed value of  0.0 to 0.0
            // because spade refuses to triangulate with values within it's minimum:
            #[allow(clippy::excessive_precision)]
            const MIN_ALLOWED_VALUE: Real = 1.793662034335766e-43; // 1.0 * 2^-142
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.position.coords - origin_3d.coords;
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
                    // at least one coordinate was NaN/±∞ – skip this vertex
                    continue;
                }

                all_vertices_2d.push(coord! { x: x_clamped, y: y_clamped });
            }

            if all_vertices_2d.len() < 3 {
                return Vec::new();
            }

            {
                let eps2 = tolerance() * tolerance();
                let mut deduped = Vec::with_capacity(all_vertices_2d.len());
                for c in &all_vertices_2d {
                    if let Some(prev) = deduped.last() {
                        let prev: &geo::Coord<Real> = prev;
                        let dx = c.x - prev.x;
                        let dy = c.y - prev.y;
                        if dx * dx + dy * dy < eps2 {
                            continue;
                        }
                    }
                    deduped.push(*c);
                }
                if deduped.len() > 1 {
                    let first = deduped[0];
                    let last = *deduped.last().unwrap();
                    let dx = first.x - last.x;
                    let dy = first.y - last.y;
                    if dx * dx + dy * dy < eps2 {
                        deduped.pop();
                    }
                }
                all_vertices_2d = deduped;
            }

            if all_vertices_2d.len() < 3 {
                return Vec::new();
            }

            let signed_area_2x = polygon_signed_area_2x(&all_vertices_2d);
            if signed_area_2x.abs() < tolerance() {
                return Vec::new();
            }

            // Build constrained Delaunay triangulation in Spade

            // Spade vertices
            let vertices_spade: Vec<SpadePoint2<Real>> = all_vertices_2d
                .iter()
                .map(|c| SpadePoint2::new(c.x, c.y))
                .collect();

            let n = vertices_spade.len();
            if n < 3 {
                return Vec::new();
            }

            let has_self_intersection = {
                let mut found = false;
                'outer: for i in 0..n {
                    let a1 = &all_vertices_2d[i];
                    let b1 = &all_vertices_2d[(i + 1) % n];
                    for j in (i + 2)..n {
                        if i == 0 && j == n - 1 {
                            continue;
                        }
                        let a2 = &all_vertices_2d[j];
                        let b2 = &all_vertices_2d[(j + 1) % n];
                        if segments_intersect_2d(
                            a1.x, a1.y, b1.x, b1.y, a2.x, a2.y, b2.x, b2.y,
                        ) {
                            found = true;
                            break 'outer;
                        }
                    }
                }
                found
            };
            if has_self_intersection {
                return Vec::new();
            }

            // Constraint edges: closed ring along the polygon boundary
            let edges: Vec<[usize; 2]> = (0..n).map(|i| [i, (i + 1) % n]).collect();

            // Build CDT with constraints in one go
            let mut cdt =
                match ConstrainedDelaunayTriangulation::<SpadePoint2<Real>>::bulk_load_cdt(
                    vertices_spade,
                    edges,
                ) {
                    Ok(cdt) => cdt,
                    Err(_) => {
                        // Invalid coordinates / constraints – nothing we can do here
                        return Vec::new();
                    },
                };

            // Refine only large, non-sliver polygons. For typical outlines,
            // refinement can create excessive triangle counts and hurt exports.
            let perimeter: Real = {
                let mut p: Real = 0.0;
                for i in 0..all_vertices_2d.len() {
                    let j = (i + 1) % all_vertices_2d.len();
                    let dx = all_vertices_2d[j].x - all_vertices_2d[i].x;
                    let dy = all_vertices_2d[j].y - all_vertices_2d[i].y;
                    p += (dx * dx + dy * dy).sqrt();
                }
                p
            };
            let compactness = signed_area_2x.abs() / (perimeter * perimeter + tolerance());
            const REFINE_MIN_VERTS: usize = 64;
            if all_vertices_2d.len() >= REFINE_MIN_VERTS && compactness > 1e-6 {
                let refinement = RefinementParameters::<Real>::new()
                    .with_angle_limit(AngleLimit::from_deg(5.0))
                    .keep_constraint_edges()
                    .exclude_outer_faces(true);

                let _result = cdt.refine(refinement);
            }

            // Extract triangles back out of Spade and lift to 3D
            let mut final_triangles = Vec::new();

            for face in cdt.inner_faces() {
                // Each face is a triangle; get its three vertices
                let verts = face.vertices(); // [VertexHandle; 3]

                // Keep only faces whose centroid lies inside the constrained
                // ring; this avoids leaking triangles outside the polygon.
                let p0 = verts[0].position();
                let p1 = verts[1].position();
                let p2 = verts[2].position();
                let cx = (p0.x + p1.x + p2.x) / 3.0;
                let cy = (p0.y + p1.y + p2.y) / 3.0;
                if !point_in_ring_2d(cx, cy, &all_vertices_2d) {
                    continue;
                }

                let mut tri_vertices =
                    [Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); 3];

                // Map each 2D vertex back to 3D via the plane basis
                for (k, v_handle) in verts.iter().enumerate() {
                    let p = v_handle.position(); // &Point2<Real>
                    let pos_3d = origin_3d.coords + p.x * u + p.y * v;
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }

                let edge1 = tri_vertices[1].position - tri_vertices[0].position;
                let edge2 = tri_vertices[2].position - tri_vertices[0].position;
                if edge1.cross(&edge2).norm_squared() < tolerance() * tolerance() {
                    continue;
                }

                final_triangles.push(tri_vertices);
            }

            final_triangles
        }

        #[cfg(feature = "delaunay-rs")]
        {
            use delaunay_triangulator::core::vertex::Vertex as DelaunayVertex;
            use delaunay_triangulator::triangulation::delaunay::DelaunayTriangulation;
            use geo::{Intersects, LineString, Point as GeoPoint, Polygon as GeoPolygon};

            let mut vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.position.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                if x.is_finite() && y.is_finite() {
                    vertices_2d.push([x, y]);
                }
            }

            if vertices_2d.len() < 3 {
                return Vec::new();
            }

            let polygon_2d = GeoPolygon::new(
                LineString::from(vertices_2d.iter().map(|&[x, y]| (x, y)).collect::<Vec<_>>()),
                Vec::new(),
            );

            vertices_2d.dedup_by(|a, b| a[0] == b[0] && a[1] == b[1]);
            if vertices_2d.len() > 1
                && vertices_2d.first().copied() == vertices_2d.last().copied()
            {
                vertices_2d.pop();
            }
            if vertices_2d.len() < 3 {
                return Vec::new();
            }

            let delaunay_vertices: Vec<DelaunayVertex<f64, (), 2>> = vertices_2d
                .iter()
                .map(|&[x, y]| delaunay_triangulator::vertex!([x as f64, y as f64]))
                .collect();
            let Ok(dt) = DelaunayTriangulation::<_, (), (), 2>::new(&delaunay_vertices) else {
                return Vec::new();
            };

            let mut triangles = Vec::with_capacity(dt.number_of_cells());
            for (cell_key, _) in dt.cells() {
                let Some(verts) = dt.cell_vertices(cell_key) else {
                    continue;
                };
                if verts.len() != 3 {
                    continue;
                }

                let Some(coords) = verts
                    .iter()
                    .map(|&key| dt.vertex_coords(key))
                    .collect::<Option<Vec<_>>>()
                else {
                    continue;
                };

                let sample_points = [
                    GeoPoint::new(
                        ((coords[0][0] + coords[1][0] + coords[2][0]) / 3.0) as Real,
                        ((coords[0][1] + coords[1][1] + coords[2][1]) / 3.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[0][0] + coords[1][0]) / 2.0) as Real,
                        ((coords[0][1] + coords[1][1]) / 2.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[1][0] + coords[2][0]) / 2.0) as Real,
                        ((coords[1][1] + coords[2][1]) / 2.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[2][0] + coords[0][0]) / 2.0) as Real,
                        ((coords[2][1] + coords[0][1]) / 2.0) as Real,
                    ),
                ];
                if !sample_points.iter().all(|point| polygon_2d.intersects(point)) {
                    continue;
                }

                let mut tri_vertices =
                    [Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)); 3];

                for (k, coord) in coords.iter().enumerate() {
                    let pos_3d =
                        origin_3d.coords + (coord[0] as Real) * u + (coord[1] as Real) * v;
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }

                let winding_normal = (tri_vertices[1].position - tri_vertices[0].position)
                    .cross(&(tri_vertices[2].position - tri_vertices[0].position));
                if winding_normal.dot(&normal_3d) < 0.0 {
                    tri_vertices.swap(1, 2);
                }

                triangles.push(tri_vertices);
            }

            triangles
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

    /// return a normal calculated from all polygon vertices
    pub fn calculate_new_normal(&self) -> Vector3<Real> {
        let n = self.vertices.len();
        if n < 3 {
            return Vector3::z(); // degenerate or empty
        }

        let mut points = Vec::new();
        for vertex in &self.vertices {
            points.push(vertex.position);
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

#[cfg(feature = "delaunay")]
fn polygon_signed_area_2x(ring: &[geo::Coord<Real>]) -> Real {
    let mut area = 0.0;
    for i in 0..ring.len() {
        let j = (i + 1) % ring.len();
        area += ring[i].x * ring[j].y - ring[j].x * ring[i].y;
    }
    area
}

/// Point-in-polygon test for a simple 2D ring using ray casting.
/// Returns true for inside points and points very close to the boundary.
#[cfg(feature = "delaunay")]
fn point_in_ring_2d(px: Real, py: Real, ring: &[geo::Coord<Real>]) -> bool {
    if ring.len() < 3 {
        return false;
    }

    let eps = tolerance();

    for i in 0..ring.len() {
        let j = (i + 1) % ring.len();
        let x1 = ring[i].x;
        let y1 = ring[i].y;
        let x2 = ring[j].x;
        let y2 = ring[j].y;

        let cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
        if cross.abs() <= eps {
            let min_x = x1.min(x2) - eps;
            let max_x = x1.max(x2) + eps;
            let min_y = y1.min(y2) - eps;
            let max_y = y1.max(y2) + eps;
            if px >= min_x && px <= max_x && py >= min_y && py <= max_y {
                return true;
            }
        }
    }

    let mut inside = false;
    for i in 0..ring.len() {
        let j = (i + 1) % ring.len();
        let xi = ring[i].x;
        let yi = ring[i].y;
        let xj = ring[j].x;
        let yj = ring[j].y;

        let denom = yj - yi;
        let denom_safe = if denom.abs() < eps {
            if denom >= 0.0 { eps } else { -eps }
        } else {
            denom
        };
        let x_intersect = (xj - xi) * (py - yi) / denom_safe + xi;
        let intersects = ((yi > py) != (yj > py)) && (px < x_intersect);

        if intersects {
            inside = !inside;
        }
    }

    inside
}

/// Test whether two 2D line segments properly intersect, ignoring shared endpoints.
#[cfg(feature = "delaunay")]
fn segments_intersect_2d(
    ax1: Real,
    ay1: Real,
    bx1: Real,
    by1: Real,
    ax2: Real,
    ay2: Real,
    bx2: Real,
    by2: Real,
) -> bool {
    let d1x = bx1 - ax1;
    let d1y = by1 - ay1;
    let d2x = bx2 - ax2;
    let d2y = by2 - ay2;

    let denom = d1x * d2y - d1y * d2x;
    if denom.abs() < 1e-12 {
        return false;
    }

    let dx = ax2 - ax1;
    let dy = ay2 - ay1;
    let t = (dx * d2y - dy * d2x) / denom;
    let u = (dx * d1y - dy * d1x) / denom;

    let eps = 1e-10;
    t > eps && t < 1.0 - eps && u > eps && u < 1.0 - eps
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
