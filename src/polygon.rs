use crate::float_types::{Real, PI};
use crate::errors::ValidationError;
use crate::vertex::Vertex;
use crate::plane::Plane;
use nalgebra::{
    Matrix4, Point2, Point3, Rotation3, Translation3, Unit, Vector3, Vector4,
};
use cavalier_contours::polyline::{
    PlineSource, Polyline,
};

/// A convex polygon, defined by a list of vertices and a plane.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    pub vertices: Vec<Vertex>,
    pub plane: Plane,
    pub metadata: Option<S>,
}

impl<S: Clone> Polygon<S> where S: Clone + Send + Sync {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        let plane = if vertices.len() < 3 {
            panic!(); // todo: return error
        } else {
            Plane::from_points(&vertices[0].pos, &vertices[1].pos, &vertices[2].pos)
        };
       
        Polygon {
            vertices,
            plane,
            metadata,
        }
    }

    /// Reverses winding order, flips vertices normals, and flips the plane normal
    pub fn flip(&mut self) {
        self.vertices.reverse();
        for v in &mut self.vertices {
            v.flip();
        }
        self.plane.flip();
    }

    /// Triangulate this polygon into a list of triangles, each triangle is [v0, v1, v2].
    pub fn triangulate(&self) -> Vec<[Vertex; 3]> {
        // If polygon has fewer than 3 vertices, nothing to triangulate
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        // ----- Option 1: Use earclip if "earclip-io" feature is enabled -----
        #[cfg(feature = "earclip-io")]
        {
            // We will flatten the polygon onto a local 2D plane and call earclip::earcut
            let normal_3d = self.plane.normal.normalize();
            let (u, v) = build_orthonormal_basis(normal_3d);
            let origin_3d = self.vertices[0].pos;

            // Collect 2D coords by projecting each vertex onto (u,v) plane
            let mut coords_2d = Vec::with_capacity(self.vertices.len() * 2);
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                coords_2d.push(x);
                coords_2d.push(y);
            }

            // Use earclip
            let indices = earclip::earcut::<Real, usize>(&coords_2d, &[], 2);
            let mut triangles = Vec::with_capacity(indices.len() / 3);

            // Rebuild final 3D triangles
            for tri_chunk in indices.chunks_exact(3) {
                let mut tri_vertices = [Vertex::new(Point3::origin(), Vector3::zeros()); 3];
                for (k, &idx) in tri_chunk.iter().enumerate() {
                    let x = coords_2d[2 * idx];
                    let y = coords_2d[2 * idx + 1];
                    let pos_3d = origin_3d.coords + (x * u) + (y * v);
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }
                triangles.push(tri_vertices);
            }
            return triangles;
        }

        // ----- Option 2: Use earcut if "earcut-io" feature is enabled -----
        #[cfg(feature = "earcut-io")]
        {
            use earcut::Earcut;

            let normal_3d = self.plane.normal.normalize();
            let (u, v) = build_orthonormal_basis(normal_3d);
            let origin_3d = self.vertices[0].pos;

            // Flatten each vertex to 2D
            let mut all_vertices_2d = Vec::with_capacity(self.vertices.len());
            for vert in &self.vertices {
                let offset = vert.pos.coords - origin_3d.coords;
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                all_vertices_2d.push([x, y]);
            }

            // No holes, so hole_indices = []
            let hole_indices: Vec<usize> = Vec::new();

            // Run earcut
            let mut earcut = Earcut::new();
            let mut triangle_indices = Vec::new();
            earcut.earcut(all_vertices_2d.clone(), &hole_indices, &mut triangle_indices);

            // Convert back into 3D triangles
            let mut triangles = Vec::with_capacity(triangle_indices.len() / 3);
            for tri_chunk in triangle_indices.chunks_exact(3) {
                let mut tri_vertices = [Vertex::new(Point3::origin(), Vector3::zeros()); 3];
                for (k, &idx) in tri_chunk.iter().enumerate() {
                    let [x, y] = all_vertices_2d[idx];
                    let pos_3d = origin_3d.coords + (x * u) + (y * v);
                    tri_vertices[k] = Vertex::new(Point3::from(pos_3d), normal_3d);
                }
                triangles.push(tri_vertices);
            }
            return triangles;
        }

        // ----- Fallback / default if neither earclip-io nor earcut-io is enabled, known to fail for non-convex polygons and polygons with holes -----
        #[cfg(not(any(feature = "earclip-io", feature = "earcut-io")))]
        {
            // Naive fan triangulation from vertex[0]
            let mut triangles = Vec::new();
            if self.vertices.len() < 3 {
                return triangles;
            }
            let v0 = self.vertices[0].clone();
            for i in 1..(self.vertices.len() - 1) {
                triangles.push([
                    v0.clone(),
                    self.vertices[i].clone(),
                    self.vertices[i + 1].clone(),
                ]);
            }
            triangles
        }
    }

    /// Subdivide this polygon into smaller triangles.
    /// Returns a list of new triangles (each is a [Vertex; 3]).
    pub fn subdivide_triangles(&self, subdivisions: u32) -> Vec<[Vertex; 3]> {
        // 1) Triangulate the polygon as it is.
        let base_tris = self.triangulate();

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
        if poly_normal.dot(&self.plane.normal) < 0.0 {
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
    
    /// Returns a new Polygon translated by t.
    pub fn translate(&self, x: Real, y: Real, z: Real) -> Self {     // todo: modify for Vector2 in-plane translation
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new Polygon translated by t.
    pub fn translate_vector(&self, t: Vector3<Real>) -> Self {     // todo: modify for Vector2 in-plane translation
        let new_vertices = self
            .vertices
            .iter()
            .map(|v| Vertex::new(v.pos + t, v.normal))
            .collect();
        let new_plane = Plane {
            normal: self.plane.normal,
            w: self.plane.w + self.plane.normal.dot(&t),
        };
        Self {
            vertices: new_vertices,
            plane: new_plane,
            metadata: self.metadata.clone(),
        }
    }
    
    /// Build a new Polygon from a set of 2D polylines in XY. Each polyline
    /// is turned into one polygon at z=0.
    pub fn from_polyline(polyline: &Polyline<Real>, metadata: Option<S>) -> Polygon<S> {
        if polyline.vertex_count() < 3 {
            // degenerate polygon
        }

        let plane_normal = Vector3::z();
        let mut poly_verts = Vec::with_capacity(polyline.vertex_count());
        for i in 0..polyline.vertex_count() {
            let v = polyline.at(i);
            poly_verts.push(Vertex::new(
                Point3::new(v.x, v.y, 0.0),
                plane_normal,
            ));
        }
        return Polygon::new(poly_verts, metadata);
    }
    
    /// Returns an error if any coordinate is not finite (NaN or ±∞).
    fn check_coordinates_finite(&self) -> Result<(), ValidationError> {
        for v in &self.vertices {
            let p = &v.pos;
            if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
                return Err(ValidationError::InvalidCoordinate(p.clone()));
            }
        }
        Ok(())
    }

    /// Check for repeated adjacent points. Return the first repeated coordinate if found.
    fn check_repeated_points(&self) -> Result<(), ValidationError> {
        // If there's only 2 or fewer points, skip
        if self.vertices.len() <= 2 {
            return Ok(());
        }
        for i in 0..self.vertices.len() - 1 {
            let cur = &self.vertices[i].pos;
            let nxt = &self.vertices[i + 1].pos;
            if (cur.x - nxt.x).abs() < 1e-12
                && (cur.y - nxt.y).abs() < 1e-12
                && (cur.z - nxt.z).abs() < 1e-12
            {
                return Err(ValidationError::RepeatedPoint(cur.clone()));
            }
        }
        Ok(())
    }

    /// Check ring closure: first and last vertex must coincide if polygon is meant to be closed.
    fn check_ring_closed(&self) -> Result<(), ValidationError> {
        if self.vertices.len() < 3 {
            // Not enough points to be meaningful, skip or return error
            return Err(ValidationError::TooFewPoints(
                self.vertices.get(0).map(|v| v.pos).unwrap_or_else(|| Point3::origin()),
            ));
        }
        let first = &self.vertices[0].pos;
        let last = &self.vertices[self.vertices.len() - 1].pos;
        let dist_sq = (first - last).norm_squared();
        // Adjust tolerance as needed
        if dist_sq > 1e-12 {
            return Err(ValidationError::RingNotClosed(first.clone()));
        }
        Ok(())
    }

    /// Check that the ring has at least 3 distinct points.
    fn check_minimum_ring_size(&self) -> Result<(), ValidationError> {
        // A ring should have at least 3 unique coordinates (not counting the repeated last == first).
        // If the user’s code always pushes a repeated last point, effective count = vertices.len() - 1.
        if self.vertices.len() < 4 {
            return Err(ValidationError::TooFewPoints(
                self.vertices[0].pos.clone(),
            ));
        }
        Ok(())
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
    let v0 = tri[0].clone();
    let v1 = tri[1].clone();
    let v2 = tri[2].clone();

    let v01 = v0.interpolate(&v1, 0.5);
    let v12 = v1.interpolate(&v2, 0.5);
    let v20 = v2.interpolate(&v0, 0.5);

    vec![
        [v0.clone(), v01.clone(), v20.clone()],
        [v01.clone(), v1.clone(), v12.clone()],
        [v20.clone(), v12.clone(), v2.clone()],
        [v01, v12, v20],
    ]
}

/// Helper to normalize angles into (-π, π].
fn normalize_angle(mut a: Real) -> Real {
    while a <= -PI {
        a += 2.0 * PI;
    }
    while a > PI {
        a -= 2.0 * PI;
    }
    a
}

/// Returns `true` if the line segments p1->p2 and p3->p4 intersect, otherwise `false`.
fn segments_intersect_2d(
    p1x: Real, p1y: Real,
    p2x: Real, p2y: Real,
    p3x: Real, p3y: Real,
    p4x: Real, p4y: Real,
) -> bool {
    // A helper function to get the orientation of the triplet (p, q, r).
    // Returns:
    // 0 -> p, q, r are collinear
    // 1 -> Clockwise
    // 2 -> Counterclockwise
    fn orientation(px: Real, py: Real, qx: Real, qy: Real, rx: Real, ry: Real) -> i32 {
        let val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
        if val.abs() < 1e-12 {
            0
        } else if val > 0.0 {
            1
        } else {
            2
        }
    }

    // A helper function to check if point q lies on line segment pr
    fn on_segment(px: Real, py: Real, qx: Real, qy: Real, rx: Real, ry: Real) -> bool {
        qx >= px.min(rx) && qx <= px.max(rx) &&
        qy >= py.min(ry) && qy <= py.max(ry)
    }

    // Find the 4 orientations needed for the general and special cases
    let o1 = orientation(p1x, p1y, p2x, p2y, p3x, p3y);
    let o2 = orientation(p1x, p1y, p2x, p2y, p4x, p4y);
    let o3 = orientation(p3x, p3y, p4x, p4y, p1x, p1y);
    let o4 = orientation(p3x, p3y, p4x, p4y, p2x, p2y);

    // General case: If the two line segments strictly intersect
    if o1 != o2 && o3 != o4 {
        return true;
    }

    // Special cases: check for collinearity and overlap
    // p1, p2, p3 are collinear and p3 lies on segment p1p2
    if o1 == 0 && on_segment(p1x, p1y, p3x, p3y, p2x, p2y) {
        return true;
    }
    // p1, p2, p4 are collinear and p4 lies on segment p1p2
    if o2 == 0 && on_segment(p1x, p1y, p4x, p4y, p2x, p2y) {
        return true;
    }
    // p3, p4, p1 are collinear and p1 lies on segment p3p4
    if o3 == 0 && on_segment(p3x, p3y, p1x, p1y, p4x, p4y) {
        return true;
    }
    // p3, p4, p2 are collinear and p2 lies on segment p3p4
    if o4 == 0 && on_segment(p3x, p3y, p2x, p2y, p4x, p4y) {
        return true;
    }

    // Otherwise, they do not intersect
    false
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
/// the midpoint. This reproduces your “arcfinder” approach in a version that uses nalgebra’s
/// `Point2<Real>`.
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

/// Helper to produce the "best fit arc" for the points from `pt_c` through `pt_n`, plus
/// any in `intermediates`. This is basically your old “best_arc” logic but now returning
/// `None` if it fails or `Some((cw, radius, center, rms))` if success.
fn best_arc_fit(
    pt_c: Point2<Real>,
    pt_n: Point2<Real>,
    intermediates: &[Point2<Real>],
    rms_limit: Real,
    angle_limit_degs: Real,
    _offset_limit: Real,
) -> Option<(bool, Real, Point2<Real>, Real)> {
    // 1) Call your circle-fitting routine:
    let (center, radius, cw, rms) = fit_circle_arcfinder(&pt_c, &pt_n, intermediates);

    // 2) Check RMS error vs. limit
    if rms > rms_limit {
        return None;
    }
    // 3) measure the total arc sweep
    //    We'll compute angle0, angle1 from the center
    //    v0 = pt_c - center, v1 = pt_n - center
    let v0 = pt_c - center;
    let v1 = pt_n - center;
    let angle0 = v0.y.atan2(v0.x);
    let angle1 = v1.y.atan2(v1.x);
    let sweep = normalize_angle(angle1 - angle0).abs();
    let sweep_degs = sweep.to_degrees();
    if sweep_degs > angle_limit_degs {
        return None;
    }
    // 4) Possibly check some "offset" or chord–offset constraints
    // e.g. if your logic says “if radius < ??? or if something with offset_limit”
    if radius < 1e-9 {
        return None;
    }
    // offset constraint is left to your specific arcs logic:
    // if something > offset_limit {...}

    // If all is well:
    Some((cw, radius, center, rms))
}
