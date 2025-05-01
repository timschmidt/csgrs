use crate::float_types::{PI, Real};
use crate::plane::Plane;
use crate::vertex::Vertex;
use geo::{LineString, Polygon as GeoPolygon, coord};
use nalgebra::{Point2, Point3, Vector3};

/// A polygon, defined by a list of vertices.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
    pub metadata: Option<S>,
}

impl<S: Clone> Polygon<S>
where S: Clone + Send + Sync {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        assert!(vertices.len() >= 3, "degenerate polygon");
        
        let plane = Plane::from_vertices(vertices.clone());
        
        Polygon {
            vertices,
            plane,
            metadata,
        }
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
        self.vertices
            .iter()
            .zip(self.vertices.iter().cycle().skip(1))
    }

    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    pub fn tessellate(&self) -> Vec<[Vertex; 3]> {
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
            const MIN_ALLOWED_VALUE: f64 = 1.793662034335766e-43; // 1.0 * 2^-142
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let x_clamped = if x.abs() < MIN_ALLOWED_VALUE { 0.0 } else { x };
                let y = offset.dot(&v);
                let y_clamped = if y.abs() < MIN_ALLOWED_VALUE { 0.0 } else { y };
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
    pub fn subdivide_triangles(&self, subdivisions: u32) -> Vec<[Vertex; 3]> {
        // 1) Triangulate the polygon as it is.
        let base_tris = self.tessellate();

        // 2) For each triangle, subdivide 'subdivisions' times.
        let mut result = Vec::new();
        for tri in base_tris {
            // We'll keep a queue of triangles to process
            let mut queue = vec![tri];
            for _ in 0..subdivisions {
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

/// Helper to normalize angles into (-π, π].
const fn normalize_angle(mut a: Real) -> Real {
    while a <= -PI {
        a += 2.0 * PI;
    }
    while a > PI {
        a -= 2.0 * PI;
    }
    a
}

/// Compute an initial guess of the circle center through three points p1, p2, p3
/// (this is used purely as an initial guess).
///
/// This is a direct port of your snippet’s `centre(p1, p2, p3)`, but
/// returning a `Point2<Real>` from nalgebra.
fn naive_circle_center(p1: &Point2<Real>, p2: &Point2<Real>, p3: &Point2<Real>) -> Point2<Real> {
    // Coordinates
    let (x1, y1) = (p1.x, p1.y);
    let (x2, y2) = (p2.x, p2.y);
    let (x3, y3) = (p3.x, p3.y);

    let x12 = x1 - x2;
    let x13 = x1 - x3;
    let y12 = y1 - y2;
    let y13 = y1 - y3;

    let y31 = y3 - y1;
    let y21 = y2 - y1;
    let x31 = x3 - x1;
    let x21 = x2 - x1;

    let sx13 = x1.powi(2) - x3.powi(2);
    let sy13 = y1.powi(2) - y3.powi(2);
    let sx21 = x2.powi(2) - x1.powi(2);
    let sy21 = y2.powi(2) - y1.powi(2);

    let xden = 2.0 * (x31 * y12 - x21 * y13);
    let yden = 2.0 * (y31 * x12 - y21 * x13);

    if xden.abs() < 1e-14 || yden.abs() < 1e-14 {
        // fallback => just average the points
        let cx = (x1 + x2 + x3) / 3.0;
        let cy = (y1 + y2 + y3) / 3.0;
        return Point2::new(cx, cy);
    }

    let g = (sx13 * y12 + sy13 * y12 + sx21 * y13 + sy21 * y13) / xden;
    let f = (sx13 * x12 + sy13 * x12 + sx21 * x13 + sy21 * x13) / yden;

    // Return the center as a Point2
    Point2::new(-g, -f)
}

/// Fit a circle to the points `[pt_c, intermediates..., pt_n]` by adjusting an offset `d` from
/// the midpoint. This uses nalgebra’s `Point2<Real>`.
///
/// # Returns
///
/// `(center, radius, cw, rms)`:
/// - `center`: fitted circle center (Point2),
/// - `radius`: circle radius,
/// - `cw`: `true` if the arc is clockwise, `false` if ccw,
/// - `rms`: root‐mean‐square error of the fit.
pub fn fit_circle_arcfinder(
    pt_c: &Point2<Real>,
    pt_n: &Point2<Real>,
    intermediates: &[Point2<Real>],
) -> (Point2<Real>, Real, bool, Real) {
    // 1) Distance between pt_c and pt_n, plus midpoint
    let k = (pt_c - pt_n).norm();
    if k < 1e-14 {
        // Degenerate case => no unique circle
        let center = *pt_c;
        return (center, 0.0, false, 9999.0);
    }
    let mid = Point2::new(0.5 * (pt_c.x + pt_n.x), 0.5 * (pt_c.y + pt_n.y));

    // 2) Pre‐compute the direction used for the offset:
    //    This is the 2D +90 rotation of (pt_n - pt_c).
    //    i.e. rotate( dx, dy ) => (dy, -dx ) or similar.
    let vec_cn = pt_n - pt_c; // a Vector2
    let rx = vec_cn.y; // +90 deg
    let ry = -vec_cn.x; // ...

    // collect all points in one array for the mismatch
    let mut all_points = Vec::with_capacity(intermediates.len() + 2);
    all_points.push(*pt_c);
    all_points.extend_from_slice(intermediates);
    all_points.push(*pt_n);

    // The mismatch function g(d)
    let g = |d: Real| -> Real {
        let r_desired = (d * d + 0.25 * k * k).sqrt();
        // circle center
        let cx = mid.x + (d / k) * rx;
        let cy = mid.y + (d / k) * ry;
        let mut sum_sq = 0.0;
        for p in &all_points {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dist = (dx * dx + dy * dy).sqrt();
            let diff = dist - r_desired;
            sum_sq += diff * diff;
        }
        sum_sq
    };

    // derivative dg(d) => we’ll do a small finite difference
    let dg = |d: Real| -> Real {
        let h = 1e-6;
        let g_p = g(d + h);
        let g_m = g(d - h);
        (g_p - g_m) / (2.0 * h)
    };

    // 3) choose an initial guess for d
    let mut d_est = 0.0; // fallback
    if !intermediates.is_empty() {
        // pick p3 ~ the middle of intermediates
        let mididx = intermediates.len() / 2;
        let p3 = intermediates[mididx];
        let c_est = naive_circle_center(pt_c, pt_n, &p3);
        // project c_est - mid onto (rx, ry)/k => that is d
        let dx = c_est.x - mid.x;
        let dy = c_est.y - mid.y;
        let dot = dx * (rx / k) + dy * (ry / k);
        d_est = dot;
    }

    // 4) small secant iteration for ~10 steps
    let mut d0 = d_est - 0.1 * k;
    let mut d1 = d_est;
    let mut dg0 = dg(d0);
    let mut dg1 = dg(d1);

    for _ in 0..10 {
        if (dg1 - dg0).abs() < 1e-14 {
            break;
        }
        let temp = d1;
        d1 = d1 - dg1 * (d1 - d0) / (dg1 - dg0);
        d0 = temp;
        dg0 = dg1;
        dg1 = dg(d1);
    }

    let d_opt = d1;
    let cx = mid.x + (d_opt / k) * rx;
    let cy = mid.y + (d_opt / k) * ry;
    let center = Point2::new(cx, cy);
    let radius_opt = (d_opt * d_opt + 0.25 * k * k).sqrt();

    // sum of squares at d_opt
    let sum_sq = g(d_opt);
    let n_pts = all_points.len() as Real;
    let rms = (sum_sq / n_pts).sqrt();

    // 5) determine cw vs ccw
    let dx0 = pt_c.x - cx;
    let dy0 = pt_c.y - cy;
    let dx1 = pt_n.x - cx;
    let dy1 = pt_n.y - cy;
    let angle0 = dy0.atan2(dx0);
    let angle1 = dy1.atan2(dx1);
    let total_sweep = normalize_angle(angle1 - angle0);
    let cw = total_sweep < 0.0;

    (center, radius_opt, cw, rms)
}
