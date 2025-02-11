use crate::float_types::{Real, PI, CLOSED};
use crate::errors::ValidationError;
use crate::vertex::Vertex;
use crate::plane::Plane;
use nalgebra::{
    Matrix4, Point2, Point3, Rotation3, Translation3, Unit, Vector3,
};
use cavalier_contours::polyline::{
    BooleanOp, PlineCreation, PlineSource, PlineSourceMut, Polyline,
};
use chull::ConvexHullWrapper;

/// A convex polygon, defined by a list of vertices and a plane.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    pub vertices: Vec<Vertex>,
    pub open: bool,
    pub metadata: Option<S>,
    pub plane: Plane,
}

impl<S: Clone> Polygon<S> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, mut open: bool, metadata: Option<S>) -> Self {
        if vertices.len() < 3 {
            open = true;
        };

        let plane = Plane::from_points(&vertices[0].pos, &vertices[1].pos, &vertices[2].pos);
        Polygon {
            vertices,
            open,
            metadata,
            plane,
        }
    }

    /// Build a new Polygon in 3D from a 2D polyline in *this* polygon’s plane.
    /// i.e. we treat that 2D polyline as lying in the same plane as `self`.
    pub fn from_2d(&self, polyline: &Polyline<Real>) -> Polygon<S> {
        let open = !polyline.is_closed();
        let (_to_xy, from_xy) = self.plane.to_xy_transform();

        let mut poly_verts = Vec::with_capacity(polyline.vertex_count());
        for i in 0..polyline.vertex_count() {
            let v = polyline.at(i);

            // (x, y, 0, 1)
            let p4_local = nalgebra::Vector4::new(v.x, v.y, 0.0, 1.0);
            let p4_world = from_xy * p4_local;

            let vx = p4_world[0];
            let vy = p4_world[1];
            let vz = p4_world[2];

            poly_verts.push(Vertex::new(
                Point3::new(vx, vy, vz),
                self.plane.normal, // We will recalc plane anyway
            ));
        }
        let mut poly3d = Polygon::new(poly_verts, open, self.metadata.clone());
        poly3d.set_new_normal();
        poly3d
    }

    /// Project this polygon into its own plane’s local XY coordinates,
    /// producing a 2D cavalier_contours Polyline<Real>.
    pub fn to_2d(&self) -> Polyline<Real> {
        if self.vertices.len() < 2 {
            // Degenerate polygon, return empty polyline
            return Polyline::new();
        }

        // Get transforms
        let (to_xy, _from_xy) = self.plane.to_xy_transform();

        // Transform each vertex.
        // Then we only keep (x, y) and ignore the new z (should be near zero).
        let mut polyline = Polyline::with_capacity(self.vertices.len(), !self.open);
        for v in &self.vertices {
            let p4 = v.pos.to_homogeneous();
            let xyz = to_xy * p4; // Matrix4 × Vector4
            let x2 = xyz[0];
            let y2 = xyz[1];
            let bulge = 0.0; // todo: support arcs
            polyline.add(x2, y2, bulge);
        }
        polyline
    }

    /// Project this polygon into its own plane’s local XY coordinates,
    /// producing a 2D cavalier_contours Polyline<Real>.
    pub fn to_polyline(&self) -> Polyline<Real> {
        if self.vertices.len() < 2 {
            // Degenerate polygon, return empty polyline
            return Polyline::new();
        }

        // We flatten the polygon into the XY plane (z ~ 0).
        // If our polygons might have arcs, we'll need more logic to detect + store bulge, etc.
        let mut polyline = Polyline::with_capacity(self.vertices.len(), !self.open);
        for v in &self.vertices {
            let bulge = 0.0; // ignoring arcs
            polyline.add(v.pos.coords.x, v.pos.coords.y, bulge);
        }
        polyline
    }

    /// Build a new Polygon from a set of 2D polylines in XY. Each polyline
    /// is turned into one polygon at z=0.
    pub fn from_polyline(polyline: &Polyline<Real>, metadata: Option<S>) -> Polygon<S> {
        if polyline.vertex_count() < 2 {
            // degenerate polygon
        }
        
        let open = !polyline.is_closed();

        let plane_normal = nalgebra::Vector3::z();
        let mut poly_verts = Vec::with_capacity(polyline.vertex_count());
        for i in 0..polyline.vertex_count() {
            let v = polyline.at(i);
            poly_verts.push(Vertex::new(
                nalgebra::Point3::new(v.x, v.y, 0.0),
                plane_normal,
            ));
        }
        return Polygon::new(poly_verts, open, metadata);
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

    /// Recompute this polygon's normal, then set all vertices' normals to match (flat shading).
    pub fn set_new_normal(&mut self) {
        // Assign each vertex’s normal to match the plane
        let new_normal = self.calculate_new_normal();
        for v in &mut self.vertices {
            v.normal = new_normal;
        }
    }

    /// Return all resulting polygons from the union.
    /// If the union has disjoint pieces, you'll get multiple polygons.
    pub fn union(&self, other: &Polygon<S>) -> Vec<Polygon<S>> {
        let self_cc = self.to_2d();
        let other_cc = other.to_2d();

        // Use cavalier_contours boolean op OR
        // union_result is a `BooleanResult<Polyline>`
        let union_result = self_cc.boolean(&other_cc, BooleanOp::Or);

        let mut polygons_out = Vec::new();

        // union_result.pos_plines has the union outlines
        // union_result.neg_plines might be empty for `Or`.
        for outline in union_result.pos_plines {
            let pl = outline.pline; // a Polyline<Real>
            if pl.vertex_count() < 3 {
                continue; // skip degenerate
            }
            // Convert to a 3D Polygon<S> in the XY plane
            polygons_out.push(self.from_2d(&pl));
        }

        polygons_out
    }

    /// Perform 2D boolean intersection with `other` and return resulting polygons.
    pub fn intersection(&self, other: &Polygon<S>) -> Vec<Polygon<S>> {
        let self_cc = self.to_2d();
        let other_cc = other.to_2d();

        // Use cavalier_contours boolean op AND
        let result = self_cc.boolean(&other_cc, cavalier_contours::polyline::BooleanOp::And);

        let mut polygons_out = Vec::new();

        // For intersection, result.pos_plines has the “kept” intersection loops
        for outline in result.pos_plines {
            let pl = outline.pline;
            if pl.vertex_count() < 3 {
                continue;
            }
            polygons_out.push(self.from_2d(&pl));
        }
        polygons_out
    }

    /// Perform 2D boolean difference (this minus other) and return resulting polygons.
    pub fn difference(&self, other: &Polygon<S>) -> Vec<Polygon<S>> {
        let self_cc = self.to_2d();
        let other_cc = other.to_2d();

        // Use cavalier_contours boolean op NOT
        let result = self_cc.boolean(&other_cc, cavalier_contours::polyline::BooleanOp::Not);

        let mut polygons_out = Vec::new();

        // For difference, result.pos_plines is what remains of self after subtracting `other`.
        for outline in result.pos_plines {
            let pl = outline.pline;
            if pl.vertex_count() < 3 {
                continue;
            }
            polygons_out.push(self.from_2d(&pl));
        }
        polygons_out
    }

    /// Perform 2D boolean exclusive‐or (symmetric difference) and return resulting polygons.
    pub fn xor(&self, other: &Polygon<S>) -> Vec<Polygon<S>> {
        let self_cc = self.to_2d();
        let other_cc = other.to_2d();

        // Use cavalier_contours boolean op XOR
        let result = self_cc.boolean(&other_cc, cavalier_contours::polyline::BooleanOp::Xor);

        let mut polygons_out = Vec::new();

        // For XOR, result.pos_plines is the symmetrical difference
        for outline in result.pos_plines {
            let pl = outline.pline;
            if pl.vertex_count() < 3 {
                continue;
            }
            polygons_out.push(self.from_2d(&pl));
        }
        polygons_out
    }

    /// Returns a new Polygon translated by t.
    pub fn translate(&self, t: Vector3<Real>) -> Self {     // todo: modify for Vector2 in-plane translation
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
            open: self.open.clone(),
            metadata: self.metadata.clone(),
            plane: new_plane,
        }
    }

    /// Applies the affine transform given by mat to all vertices and normals.
    pub fn transform(&self, mat: &Matrix4<Real>) -> Self {
        let new_vertices: Vec<Vertex> = self
            .vertices
            .iter()
            .map(|v| {
                // Transform the position:
                let p_hom = v.pos.to_homogeneous();
                let new_hom = mat * p_hom;
                let new_pos = Point3::from_homogeneous(new_hom).unwrap();
                // Transform the normal using the inverse–transpose:
                let mat_inv_trans = mat.try_inverse().unwrap().transpose();
                // Treat the normal as a direction (w=0)
                let normal_hom = v.normal.push(0.0);
                let new_normal = (mat_inv_trans * normal_hom).xyz().normalize();
                Vertex::new(new_pos, new_normal)
            })
            .collect();
        // Recompute the plane from the first three vertices.
        let new_plane = if new_vertices.len() >= 3 {
            Plane::from_points(
                &new_vertices[0].pos,
                &new_vertices[1].pos,
                &new_vertices[2].pos,
            )
        } else {
            self.plane.clone()
        };
        Self {
            vertices: new_vertices,
            open: self.open.clone(),
            metadata: self.metadata.clone(),
            plane: new_plane,
        }
    }

    /// Rotates the polygon by a given angle (radians) about the given axis.
    /// If a center is provided the rotation is performed about that point;
    /// otherwise rotation is about the origin.
    pub fn rotate(&self, axis: Vector3<Real>, angle: Real, center: Option<Point3<Real>>) -> Self {
        let rotation = Rotation3::from_axis_angle(&Unit::new_normalize(axis), angle);
        let t = if let Some(c) = center {
            // Translate so that c goes to the origin, rotate, then translate back.
            let trans_to_origin = Translation3::from(-c.coords);
            let trans_back = Translation3::from(c.coords);
            trans_back.to_homogeneous()
                * rotation.to_homogeneous()
                * trans_to_origin.to_homogeneous()
        } else {
            rotation.to_homogeneous()
        };
        self.transform(&t)
    }

    /// Uniformly scales the polygon by the given factor.
    pub fn scale(&self, factor: Real) -> Self {
        let scaling = Matrix4::new_nonuniform_scaling(&Vector3::new(factor, factor, factor));
        self.transform(&scaling)
    }

    /// Mirrors the polygon about the given x axis
    pub fn mirror_x(&self) -> Self {
        let mirror_mat = Matrix4::new_nonuniform_scaling(&Vector3::new(-1.0, 1.0, 1.0));
        self.transform(&mirror_mat)
    }
    
    /// Mirrors the polygon about the given y axis
    pub fn mirror_y(&self) -> Self {
        let mirror_mat = Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, -1.0, 1.0));
        self.transform(&mirror_mat)
    }
    
    /// Mirrors the polygon about the given z axis
    pub fn mirror_z(&self) -> Self {
        let mirror_mat = Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0));
        self.transform(&mirror_mat)
    }

    /// Returns a new Polygon that is the convex hull of the current polygon’s vertices.
    /// (It projects the vertices to 2D in the polygon’s plane, computes the convex hull, and lifts back.)
    pub fn convex_hull(&self) -> Self {
        let (to_xy, from_xy) = self.plane.to_xy_transform();
        let pts_2d: Vec<Vec<Real>> = self
            .vertices
            .iter()
            .map(|v| {
                let p2 = to_xy * v.pos.to_homogeneous();
                vec![p2[0], p2[1]]
            })
            .collect();
        let chull = ConvexHullWrapper::try_new(&pts_2d, None).expect("convex hull failed");
        let (hull_verts, _hull_indices) = chull.vertices_indices();
        let new_vertices = hull_verts
            .iter()
            .map(|p| {
                // Make sure to tell Rust the type explicitly so that the multiplication produces
                // a Vector4<Real>.
                let p4: nalgebra::Vector4<Real> = nalgebra::Vector4::new(p[0], p[1], 0.0, 1.0);
                let p3 = from_xy * p4;
                Vertex::new(Point3::from_homogeneous(p3).unwrap(), self.plane.normal)
            })
            .collect();
        Polygon::new(new_vertices, CLOSED, self.metadata.clone())
    }

    /// Returns the Minkowski sum of this polygon and another.
    /// (For each vertex in self and other, we add their coordinates, and then take the convex hull.)
    pub fn minkowski_sum(&self, other: &Self) -> Self {
        let mut sum_pts = Vec::new();
        for v in &self.vertices {
            for w in &other.vertices {
                sum_pts.push(Point3::from(v.pos.coords + w.pos.coords));
            }
        }
        let (to_xy, from_xy) = self.plane.to_xy_transform();
        let pts_2d: Vec<Vec<Real>> = sum_pts
            .iter()
            .map(|p| {
                let p_hom = p.to_homogeneous();
                let p2 = to_xy * p_hom;
                vec![p2[0], p2[1]]
            })
            .collect();
        let chull =
            ConvexHullWrapper::try_new(&pts_2d, None).expect("Minkowski sum convex hull failed");
        let (hull_verts, _hull_indices) = chull.vertices_indices();
        let new_vertices = hull_verts
            .iter()
            .map(|p| {
                // Make sure to tell Rust the type explicitly so that the multiplication produces
                // a Vector4<Real>.
                let p4: nalgebra::Vector4<Real> = nalgebra::Vector4::new(p[0], p[1], 0.0, 1.0);
                let p3 = from_xy * p4;
                Vertex::new(Point3::from_homogeneous(p3).unwrap(), self.plane.normal)
            })
            .collect();
        Polygon::new(new_vertices, CLOSED, self.metadata.clone())
    }

    /// Parallel offset of this polygon (interpreted as an open polyline) by distance `d`.
    /// Uses cavalier_contours offset on the underlying 2D polyline representation.
    /// Returns a new Polygon or possibly multiple polygons.
    pub fn offset(&self, distance: Real) -> Vec<Polygon<S>> {
        if self.vertices.len() < 2 {
            return vec![];
        }
        // Convert to a single 2D open polyline
        let pline_2d = self.to_2d();
        // Perform offset
        let offset_result = pline_2d.parallel_offset(distance);
        let plane_normal = nalgebra::Vector3::z();

        // Convert each offset polyline back to a 3D Polygon
        let mut new_polygons = Vec::new();
        for off_pl in offset_result {
            if off_pl.vertex_count() >= 2 {
                let open = !off_pl.is_closed();
                let mut poly_verts = Vec::with_capacity(off_pl.vertex_count());
                for i in 0..off_pl.vertex_count() {
                    let v = off_pl.at(i);
                    poly_verts.push(Vertex::new(
                        nalgebra::Point3::new(v.x, v.y, 0.0),
                        plane_normal,
                    ));
                }
                new_polygons.push(Polygon::new(poly_verts, open, self.metadata.clone()));
            }
        }
        new_polygons
    }

    /// Attempt to reconstruct arcs of constant radius in the 2D projection of this polygon,
    /// storing them as bulge arcs in the returned `Polyline<Real>`.
    ///
    /// # Parameters
    /// - `min_match`: minimal number of consecutive edges needed to consider forming an arc
    /// - `rms_limit`: max RMS fitting error (like `arcfinder`’s `options.rms_limit`)
    /// - `angle_limit_degs`: max total arc sweep in degrees
    /// - `offset_limit`: additional limit used by `arcfinder` for chord offsets, etc.
    ///
    /// # Returns
    /// A single `Polyline<Real>` with arcs (encoded via bulge) or lines if no arcs found.
    ///
    pub fn reconstruct_arcs(
        &self,
        min_match: usize,
        rms_limit: Real,
        angle_limit_degs: Real,
        offset_limit: Real,
    ) -> Polyline<Real> {
        // 1) Flatten or project to 2D. Suppose `to_2d()` returns a Polyline<Real> with .x, .y, .bulge:
        let poly_2d = self.to_2d();
        // If too few vertices, or degenerate
        if poly_2d.vertex_count() < 2 {
            return poly_2d;
        }

        // 2) Collect all points in a Vec<Point2<Real>>
        //    If polygon is closed, the polyline might be closed. We can handle it accordingly:
        let mut all_pts: Vec<Point2<Real>> = Vec::with_capacity(poly_2d.vertex_count());
        for i in 0..poly_2d.vertex_count() {
            let v = poly_2d.at(i);
            all_pts.push(Point2::new(v.x, v.y));
        }

        // 3) We'll build a new output polyline with arcs.
        //    For demonstration, let's replicate an approach like your code snippet:
        let mut result = Polyline::with_capacity(all_pts.len(), poly_2d.is_closed());
        if !all_pts.is_empty() {
            // add the first point as a line start
            let pt_c = all_pts[0];
            result.add(pt_c.x, pt_c.y, 0.0);
        }

        let mut i = 0;
        let n = all_pts.len();

        while i < n - 1 {
            // Attempt to form an arc from i..some j≥i+min_match
            let start_pt = all_pts[i];
            let mut found_arc = false;
            let mut best_j = i + 1;
            let mut best_arc_data: Option<(bool, Real, Point2<Real>)> = None;

            let mut j = i + min_match;
            while j < n {
                let pt_j = all_pts[j];
                let midslice = &all_pts[i + 1..j];
                if let Some((cw, r, ctr, _rms)) = best_arc_fit(
                    start_pt,
                    pt_j,
                    midslice,
                    rms_limit,
                    angle_limit_degs,
                    offset_limit,
                ) {
                    found_arc = true;
                    best_arc_data = Some((cw, r, ctr));
                    best_j = j;
                    j += 1; // try extending more
                } else {
                    break;
                }
            }

            if found_arc {
                // we have an arc from i..best_j
                let end_pt = all_pts[best_j];
                let (cw, _r, c) = best_arc_data.unwrap();

                // compute angle from center => to find bulge
                let v0 = start_pt - c; // v0 is a Vector2<Real>
                let v1 = end_pt - c; // v1 is a Vector2<Real>
                let ang0 = v0.y.atan2(v0.x);
                let ang1 = v1.y.atan2(v1.x);
                let total_sweep = normalize_angle(ang1 - ang0);
                let arc_sweep = if cw {
                    -total_sweep.abs()
                } else {
                    total_sweep.abs()
                };
                // bulge = tan(sweep/4)
                let bulge = (arc_sweep * 0.25).tan();

                // set bulge on the last vertex in `result` (the arc start):
                let last_idx = result.vertex_count() - 1;
                let mut last_v = result[last_idx];
                last_v.bulge = bulge;
                result.set_vertex(last_idx, last_v);

                // then add end vertex with bulge=0
                result.add(end_pt.x, end_pt.y, 0.0);

                i = best_j;
            } else {
                // no arc => just line from i->i+1
                let next_pt = all_pts[i + 1];
                // set bulge=0 on the last output vertex
                let last_idx = result.vertex_count() - 1;
                let mut lv = result[last_idx];
                lv.bulge = 0.0;
                result.set_vertex(last_idx, lv);

                result.add(next_pt.x, next_pt.y, 0.0);

                i += 1;
            }
        }

        result
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

    /// Very basic ring self‐intersection check by naive line–line intersection
    fn check_ring_self_intersection(&self) -> Result<(), ValidationError> {
        // We can flatten to 2D or do full 3D line‐segment intersection logic
        // but typically “self” is in the plane, so let's do a quick 2D approach:
        let poly_2d = self.to_2d(); // produce a Polyline in XY
        
        // Check if the ring is simple by scanning edges pairwise
        // A more robust approach is to use a line sweep or an noding approach.

        // We’ll do naive O(n^2)
        let n = poly_2d.vertex_count();
        // skip if < 4
        if n < 4 {
            return Ok(()); // or already caught by min size
        }

        for i in 0..(n - 1) {
            let p1 = poly_2d.at(i);
            let p2 = poly_2d.at(i + 1);
            for j in (i + 2)..(n - 1) {
                // skip adjacent edges sharing a vertex
                if j == i + 1 {
                    continue;
                }
                let p3 = poly_2d.at(j);
                let p4 = poly_2d.at(j + 1);

                if segments_intersect_2d(p1.x, p1.y, p2.x, p2.y,
                                         p3.x, p3.y, p4.x, p4.y)
                {
                    // Return an error
                    // We might pick the intersection coords or just p1, etc.
                    let pt = Point3::new(p1.x, p1.y, 0.0);
                    return Err(ValidationError::RingSelfIntersection(pt));
                }
            }
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

/// Computes the signed area of a closed 2D polyline via the shoelace formula.
/// We assume `pline.is_closed() == true` and it has at least 2 vertices.
/// Returns positive area if CCW, negative if CW. Near-zero => degenerate.
pub fn pline_area(pline: &Polyline<Real>) -> Real {
    if pline.vertex_count() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    let n = pline.vertex_count();
    for i in 0..n {
        let j = (i + 1) % n;
        let (x_i, y_i) = (pline.at(i).x, pline.at(i).y);
        let (x_j, y_j) = (pline.at(j).x, pline.at(j).y);
        area += x_i * y_j - y_i * x_j;
    }
    0.5 * area
}

/// Given a normal vector `n`, build two perpendicular unit vectors `u` and `v` so that
/// {u, v, n} forms an orthonormal basis. `n` is assumed non‐zero.
pub fn build_orthonormal_basis(
    n: nalgebra::Vector3<Real>,
) -> (nalgebra::Vector3<Real>, nalgebra::Vector3<Real>) {
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

/// Perform a 2D union of an entire slice of polygons (all assumed in the XY plane).
/// Because `Polygon::union` returns a `Vec<Polygon<S>>` (it can split or merge),
/// we accumulate and re‐union until everything is combined.
pub fn union_all_2d<S: Clone>(polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
    if polygons.is_empty() {
        return vec![];
    }
    // Start with the first polygon
    let mut result = vec![polygons[0].clone()];

    // Union successively with each subsequent polygon
    for poly in &polygons[1..] {
        let mut new_result = Vec::new();
        for r in result {
            // `r.union(poly)` can return multiple disjoint polygons.
            let merged = r.union(poly);
            new_result.extend(merged);
        }
        result = new_result;
    }
    result
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
