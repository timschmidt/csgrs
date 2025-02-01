#![allow(dead_code)]
#![forbid(unsafe_code)]

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use nalgebra::{Point2, Point3, Vector3, Translation3, Rotation3, Isometry3, Matrix4, Unit, Quaternion};
use chull::ConvexHullWrapper;
use parry3d_f64::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::{Triangle, TriMesh, SharedShape},
    };
use rapier3d_f64::prelude::*;
use meshtext::{Glyph, MeshGenerator, MeshText};
use stl_io;
use std::io::Cursor;
use std::f64::consts::PI;
use std::error::Error;
use std::collections::HashMap;
use cavalier_contours::polyline::{
    Polyline, PlineSource, PlineCreation, PlineSourceMut, BooleanOp
};
use earclip::{earcut, flatten};
use dxf::Drawing;
use dxf::entities::*;

#[cfg(test)]
mod tests;

const EPSILON: f64 = 1e-5;

pub enum Axis {
    X,
    Y,
    Z,
}

/// Computes the signed area of a closed 2D polyline via the shoelace formula.
/// We assume `pline.is_closed() == true` and it has at least 2 vertices.
/// Returns positive area if CCW, negative if CW. Near-zero => degenerate.
fn pline_area(pline: &Polyline<f64>) -> f64 {
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
fn build_orthonormal_basis(n: nalgebra::Vector3<f64>) -> (nalgebra::Vector3<f64>, nalgebra::Vector3<f64>) {
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
        [v0.clone(),  v01.clone(), v20.clone()],
        [v01.clone(), v1.clone(),  v12.clone()],
        [v20.clone(), v12.clone(), v2.clone()],
        [v01,         v12,         v20],
    ]
}

/// Perform a 2D union of an entire slice of polygons (all assumed in the XY plane).
/// Because `Polygon::union` returns a `Vec<Polygon<S>>` (it can split or merge),
/// we accumulate and re‐union until everything is combined.
fn union_all_2d<S: Clone>(polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
    if polygons.is_empty() {
        return vec![];
    }
    // Start with the first polygon
    let mut result = vec![polygons[0].clone()];

    // Union successively with each subsequent polygon
    for poly in &polygons[1..] {
        let mut new_result = Vec::new();
        for r in result {
            // `r.union(poly)` is the new 2D union call. It can return multiple disjoint polygons.
            let merged = r.union(poly);
            new_result.extend(merged);
        }
        result = new_result;
    }
    result
}

/// Helper to normalize angles into (-π, π].
#[inline]
fn normalize_angle(mut a: f64) -> f64 {
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
/// returning a `Point2<f64>` from nalgebra.
fn naive_circle_center(
    p1: &Point2<f64>,
    p2: &Point2<f64>,
    p3: &Point2<f64>,
) -> Point2<f64> {
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
/// `Point2<f64>`.
///
/// # Returns
///
/// `(center, radius, cw, rms)`:
/// - `center`: fitted circle center (Point2),
/// - `radius`: circle radius,
/// - `cw`: `true` if the arc is clockwise, `false` if ccw,
/// - `rms`: root‐mean‐square error of the fit.
pub fn fit_circle_arcfinder(
    pt_c: &Point2<f64>,
    pt_n: &Point2<f64>,
    intermediates: &[Point2<f64>]
) -> (Point2<f64>, f64, bool, f64)
{
    // 1) Distance between pt_c and pt_n, plus midpoint
    let k = (pt_c - pt_n).norm();
    if k < 1e-14 {
        // Degenerate case => no unique circle
        let center = *pt_c;
        return (center, 0.0, false, 9999.0);
    }
    let mid = Point2::new(
        0.5 * (pt_c.x + pt_n.x),
        0.5 * (pt_c.y + pt_n.y),
    );

    // 2) Pre‐compute the direction used for the offset:
    //    This is the 2D +90 rotation of (pt_n - pt_c).
    //    i.e. rotate( dx, dy ) => (dy, -dx ) or similar.
    let vec_cn = pt_n - pt_c;  // a Vector2
    let rx = vec_cn.y;   // +90 deg
    let ry = -vec_cn.x;  // ...
    
    // collect all points in one array for the mismatch
    let mut all_points = Vec::with_capacity(intermediates.len() + 2);
    all_points.push(*pt_c);
    all_points.extend_from_slice(intermediates);
    all_points.push(*pt_n);

    // The mismatch function g(d)
    let g = |d: f64| -> f64 {
        let r_desired = (d*d + 0.25 * k*k).sqrt();
        // circle center
        let cx = mid.x + (d/k)*rx;
        let cy = mid.y + (d/k)*ry;
        let mut sum_sq = 0.0;
        for p in &all_points {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dist = (dx*dx + dy*dy).sqrt();
            let diff = dist - r_desired;
            sum_sq += diff*diff;
        }
        sum_sq
    };

    // derivative dg(d) => we’ll do a small finite difference
    let dg = |d: f64| -> f64 {
        let h = 1e-6;
        let g_p = g(d + h);
        let g_m = g(d - h);
        (g_p - g_m) / (2.0 * h)
    };

    // 3) choose an initial guess for d
    let mut d_est = 0.0;  // fallback
    if !intermediates.is_empty() {
        // pick p3 ~ the middle of intermediates
        let mididx = intermediates.len()/2;
        let p3 = intermediates[mididx];
        let c_est = naive_circle_center(pt_c, pt_n, &p3);
        // project c_est - mid onto (rx, ry)/k => that is d
        let dx = c_est.x - mid.x;
        let dy = c_est.y - mid.y;
        let dot = dx*(rx/k) + dy*(ry/k);
        d_est = dot;
    }

    // 4) small secant iteration for ~10 steps
    let mut d0 = d_est - 0.1*k;
    let mut d1 = d_est;
    let mut dg0 = dg(d0);
    let mut dg1 = dg(d1);

    for _ in 0..10 {
        if (dg1 - dg0).abs() < 1e-14 {
            break;
        }
        let temp = d1;
        d1 = d1 - dg1*(d1 - d0)/(dg1 - dg0);
        d0 = temp;
        dg0 = dg1;
        dg1 = dg(d1);
    }

    let d_opt = d1;
    let cx = mid.x + (d_opt/k)*rx;
    let cy = mid.y + (d_opt/k)*ry;
    let center = Point2::new(cx, cy);
    let radius_opt = (d_opt*d_opt + 0.25*k*k).sqrt();

    // sum of squares at d_opt
    let sum_sq = g(d_opt);
    let n_pts = all_points.len() as f64;
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
    pt_c: Point2<f64>,
    pt_n: Point2<f64>,
    intermediates: &[Point2<f64>],
    rms_limit: f64,
    angle_limit_degs: f64,
    _offset_limit: f64
) -> Option<(bool, f64, Point2<f64>, f64)>
{
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

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub pos: Point3<f64>,
    pub normal: Vector3<f64>,
}

impl Vertex {
    pub fn new(pos: Point3<f64>, normal: Vector3<f64>) -> Self {
        Vertex { pos, normal }
    }

    /// Flip orientation-specific data (like normals)
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// Linearly interpolate between `self` and `other` by parameter `t`.
    pub fn interpolate(&self, other: &Vertex, t: f64) -> Vertex {
        // For positions (Point3): p(t) = p0 + t * (p1 - p0)
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // For normals (Vector3): n(t) = n0 + t * (n1 - n0)
        let new_normal = self.normal + (other.normal - self.normal) * t;
        Vertex::new(new_pos, new_normal)
    }
}

/// A plane in 3D space defined by a normal and a w-value
#[derive(Debug, Clone)]
pub struct Plane {
    pub normal: Vector3<f64>,
    pub w: f64,
}

impl Plane {
    /// Create a plane from three points
    pub fn from_points(a: &Point3<f64>, b: &Point3<f64>, c: &Point3<f64>) -> Plane {
        let n = (b - a).cross(&(c - a)).normalize();
        if n.magnitude() < EPSILON {
            panic!("Degenerate polygon: vertices do not define a plane");
        }
        Plane {
            normal: n,
            w: n.dot(&a.coords),
        }
    }

    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }

    /// Split `polygon` by this plane if needed, distributing the results into
    /// `coplanar_front`, `coplanar_back`, `front`, and `back`.
    pub fn split_polygon<S: Clone>(
        &self,
        polygon: &Polygon<S>,
        coplanar_front: &mut Vec<Polygon<S>>,
        coplanar_back: &mut Vec<Polygon<S>>,
        front: &mut Vec<Polygon<S>>,
        back: &mut Vec<Polygon<S>>,
    ) {
        const COPLANAR: i32 = 0;
        const FRONT: i32 = 1;
        const BACK: i32 = 2;
        const SPANNING: i32 = 3;

        let mut polygon_type = 0;
        let mut types = Vec::with_capacity(polygon.vertices.len());

        // Classify each vertex
        for v in &polygon.vertices {
            let t = self.normal.dot(&v.pos.coords) - self.w;
            let vertex_type = if t < -EPSILON {
                BACK
            } else if t > EPSILON {
                FRONT
            } else {
                COPLANAR
            };
            polygon_type |= vertex_type;
            types.push(vertex_type);
        }

        match polygon_type {
            COPLANAR => {
                // Coincident normals => belongs in front vs. back
                if self.normal.dot(&polygon.plane.normal) > 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            }
            FRONT => {
                front.push(polygon.clone());
            }
            BACK => {
                back.push(polygon.clone());
            }
            _ => {
                // SPANNING
                let mut f: Vec<Vertex> = Vec::new();
                let mut b: Vec<Vertex> = Vec::new();
                let vcount = polygon.vertices.len();

                for i in 0..vcount {
                    let j = (i + 1) % vcount;
                    let ti = types[i];
                    let tj = types[j];
                    let vi = &polygon.vertices[i];
                    let vj = &polygon.vertices[j];

                    if ti != BACK {
                        f.push(vi.clone());
                    }
                    if ti != FRONT {
                        b.push(vi.clone());
                    }

                    if (ti | tj) == SPANNING {
                        let denom = self.normal.dot(&(vj.pos - vi.pos));
                        // Avoid dividing by zero
                        if denom.abs() > EPSILON {
                            let t = (self.w - self.normal.dot(&vi.pos.coords)) / denom;
                            let v = vi.interpolate(vj, t);
                            f.push(v.clone());
                            b.push(v);
                        }
                    }
                }

                if f.len() >= 3 {
                    front.push(Polygon::new(f, polygon.metadata.clone()));
                }
                if b.len() >= 3 {
                    back.push(Polygon::new(b, polygon.metadata.clone()));
                }
            }
        }
    }
    
    /// Returns (T, T_inv), where:
    /// - `T`   maps a point on this plane into XY plane (z=0) 
    ///   with the plane’s normal going to +Z,
    /// - `T_inv` is the inverse transform, mapping back.
    pub fn to_xy_transform(&self) -> (Matrix4<f64>, Matrix4<f64>) {
        // Normal
        let n = self.normal;
        let n_len = n.norm();
        if n_len < 1e-12 {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        }

        // Normalize
        let norm_dir = n / n_len;

        // Rotate plane.normal -> +Z
        let rot = Rotation3::rotation_between(&norm_dir, &Vector3::z())
            .unwrap_or_else(|| Rotation3::identity());
        let iso_rot = Isometry3::from_parts(Translation3::identity(), rot.into());

        // We want to translate so that the plane’s reference point 
        //    (some point p0 with n·p0 = w) lands at z=0 in the new coords.
        // p0 = (plane.w / (n·n)) * n
        let denom = n.dot(&n);
        let p0_3d = norm_dir * (self.w / denom);
        let p0_rot = iso_rot.transform_point(&Point3::from(p0_3d));

        // We want p0_rot.z = 0, so we shift by -p0_rot.z
        let shift_z = -p0_rot.z;
        let iso_trans = Translation3::new(0.0, 0.0, shift_z);

        let transform_to_xy = iso_trans.to_homogeneous() * iso_rot.to_homogeneous();

        // Inverse for going back
        let transform_from_xy = transform_to_xy.try_inverse().unwrap_or_else(|| Matrix4::identity());

        (transform_to_xy, transform_from_xy)
    }
}

/// A convex polygon, defined by a list of vertices and a plane.
/// - `S` is the generic metadata type, stored as `Option<S>`.
#[derive(Debug, Clone)]
pub struct Polygon<S: Clone> {
    pub vertices: Vec<Vertex>,
    pub metadata: Option<S>,
    pub plane: Plane,
}

impl<S: Clone> Polygon<S> {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, metadata: Option<S>) -> Self {
        assert!(
            vertices.len() >= 3,
            "Polygon::new requires at least 3 vertices"
        );
    
        let plane = Plane::from_points(
            &vertices[0].pos,
            &vertices[1].pos,
            &vertices[2].pos,
        );
        Polygon { vertices, metadata, plane }
    }
    
    /// Build a new Polygon in 3D from a 2D polyline in *this* polygon’s plane.
    /// i.e. we treat that 2D polyline as lying in the same plane as `self`.
    pub fn from_2d(&self, polyline: Polyline<f64>) -> Polygon<S> {
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
                self.plane.normal  // We will recalc plane anyway
            ));
        }
        let mut poly3d = Polygon::new(poly_verts, self.metadata.clone());
        poly3d.recalc_plane_and_normals();
        poly3d
    }
    
    /// Project this polygon into its own plane’s local XY coordinates,
    /// producing a 2D cavalier_contours Polyline<f64>.
    pub fn to_2d(&self) -> Polyline<f64> {
        if self.vertices.len() < 2 {
            // Degenerate polygon, return empty polyline
            return Polyline::new();
        }
        
        // Get transforms
        let (to_xy, _from_xy) = self.plane.to_xy_transform();

        // Transform each vertex. 
        // Then we only keep (x, y) and ignore the new z (should be near zero).
        let mut polyline = Polyline::with_capacity(self.vertices.len(), true);
        for v in &self.vertices {
            let p4 = v.pos.to_homogeneous();
            let xyz = to_xy * p4; // Matrix4 × Vector4
            let x2 = xyz[0];
            let y2 = xyz[1];
            let bulge = 0.0;  // ignoring arcs
            polyline.add(x2, y2, bulge);
        }
        polyline
    }
    
    /// Project this polygon into its own plane’s local XY coordinates,
    /// producing a 2D cavalier_contours Polyline<f64>.
    pub fn to_xy(&self) -> Polyline<f64> {
        if self.vertices.len() < 2 {
            // Degenerate polygon, return empty polyline
            return Polyline::new();
        }

        // We flatten the polygon into the XY plane (z ~ 0).
        // If our polygons might have arcs, we'll need more logic to detect + store bulge, etc.
        let mut polyline = Polyline::with_capacity(self.vertices.len(), true);
        for v in &self.vertices {
            let bulge = 0.0;  // ignoring arcs
            polyline.add(v.pos.coords.x, v.pos.coords.y, bulge);
        }
        polyline
    }
    
    /// Build a new Polygon from a set of 2D polylines in XY. Each polyline
    /// is turned into one polygon at z=0.
    pub fn from_xy(polyline: Polyline<f64>) -> Polygon<S> {
        if polyline.vertex_count() < 3 {
            // degenerate polygon
        }
        
        let plane_normal = nalgebra::Vector3::z();
        let mut poly_verts = Vec::with_capacity(polyline.vertex_count());
        for i in 0..polyline.vertex_count() {
            let v = polyline.at(i);
            poly_verts.push(Vertex::new(
                nalgebra::Point3::new(v.x, v.y, 0.0),
                plane_normal
            ));
        }
        return Polygon::new(poly_verts, None);
    }

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
    pub fn subdivide_triangles(&self, levels: u32) -> Vec<[Vertex; 3]> {
        // 1) Triangulate the polygon as it is.
        let base_tris = self.triangulate();

        // 2) For each triangle, subdivide 'levels' times.
        let mut result = Vec::new();
        for tri in base_tris {
            // We'll keep a queue of triangles to process
            let mut queue = vec![tri];
            for _ in 0..levels {
                let mut next_level = Vec::new();
                for t in queue {
                    let subs = subdivide_triangle(t);
                    next_level.extend(subs);
                }
                queue = next_level;
            }
            result.extend(queue);
        }

        result
    }

    /// Recompute this polygon's plane from the first 3 vertices,
    /// then set all vertices' normals to match that plane (flat shading).
    pub fn recalc_plane_and_normals(&mut self) {
        if self.vertices.len() < 3 {
            return; // degenerate or empty
        }
        // Recompute the plane from the first 3 vertices
        self.plane = Plane::from_points(
            &self.vertices[0].pos,
            &self.vertices[1].pos,
            &self.vertices[2].pos,
        );

        // Assign each vertex’s normal to match the plane
        let new_normal = self.plane.normal;
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
            let pl = outline.pline; // a Polyline<f64>
            if pl.vertex_count() < 3 {
                continue; // skip degenerate
            }
            // Convert to a 3D Polygon<S> in the XY plane
            polygons_out.push(self.from_2d(pl));
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
            polygons_out.push(self.from_2d(pl));
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
            polygons_out.push(self.from_2d(pl));
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
            polygons_out.push(self.from_2d(pl));
        }
        polygons_out
    }
    
    /// Returns a new Polygon translated by t.
    pub fn translate(&self, t: Vector3<f64>) -> Self {
        let new_vertices = self.vertices.iter()
            .map(|v| Vertex::new(v.pos + t, v.normal))
            .collect();
        let new_plane = Plane {
            normal: self.plane.normal,
            w: self.plane.w + self.plane.normal.dot(&t),
        };
        Self {
            vertices: new_vertices,
            metadata: self.metadata.clone(),
            plane: new_plane,
        }
    }

    /// Applies the affine transform given by mat to all vertices and normals.
    pub fn transform(&self, mat: &Matrix4<f64>) -> Self {
        let new_vertices: Vec<Vertex> = self.vertices.iter()
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
            Plane::from_points(&new_vertices[0].pos, &new_vertices[1].pos, &new_vertices[2].pos)
        } else {
            self.plane.clone()
        };
        Self {
            vertices: new_vertices,
            metadata: self.metadata.clone(),
            plane: new_plane,
        }
    }

    /// Rotates the polygon by a given angle (radians) about the given axis.
    /// If a center is provided the rotation is performed about that point;
    /// otherwise rotation is about the origin.
    pub fn rotate(&self, axis: Vector3<f64>, angle: f64, center: Option<Point3<f64>>) -> Self {
        let rotation = Rotation3::from_axis_angle(&Unit::new_normalize(axis), angle);
        let t = if let Some(c) = center {
            // Translate so that c goes to the origin, rotate, then translate back.
            let trans_to_origin = Translation3::from(-c.coords);
            let trans_back = Translation3::from(c.coords);
            trans_back.to_homogeneous() * rotation.to_homogeneous() * trans_to_origin.to_homogeneous()
        } else {
            rotation.to_homogeneous()
        };
        self.transform(&t)
    }

    /// Uniformly scales the polygon by the given factor.
    pub fn scale(&self, factor: f64) -> Self {
        let scaling = Matrix4::new_nonuniform_scaling(&Vector3::new(factor, factor, factor));
        self.transform(&scaling)
    }

    /// Mirrors the polygon about the given axis (X, Y, or Z).
    pub fn mirror(&self, axis: Axis) -> Self {
        let (sx, sy, sz) = match axis {
            Axis::X => (-1.0, 1.0, 1.0),
            Axis::Y => (1.0, -1.0, 1.0),
            Axis::Z => (1.0, 1.0, -1.0),
        };
        let mirror_mat = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mirror_mat)
    }

    /// Returns a new Polygon with its orientation reversed (i.e. flipped).
    /// (This is equivalent to “inverse” in CSG.)
    pub fn inverse(&self) -> Self {
        let mut new_poly = self.clone();
        new_poly.flip();
        new_poly
    }

    /// Returns a new Polygon that is the convex hull of the current polygon’s vertices.
    /// (It projects the vertices to 2D in the polygon’s plane, computes the convex hull, and lifts back.)
    pub fn convex_hull(&self) -> Self {
        let (to_xy, from_xy) = self.plane.to_xy_transform();
        let pts_2d: Vec<Vec<f64>> = self.vertices.iter().map(|v| {
            let p2 = to_xy * v.pos.to_homogeneous();
            vec![p2[0], p2[1]]
        }).collect();
        let chull = ConvexHullWrapper::try_new(&pts_2d, None)
            .expect("convex hull failed");
        let (hull_verts, _hull_indices) = chull.vertices_indices();
        let new_vertices = hull_verts.iter().map(|p| {
            // Make sure to tell Rust the type explicitly so that the multiplication produces
            // a Vector4<f64>.
            let p4: nalgebra::Vector4<f64> = nalgebra::Vector4::new(p[0], p[1], 0.0, 1.0);
            let p3 = from_xy * p4;
            Vertex::new(Point3::from_homogeneous(p3).unwrap(), self.plane.normal)
        }).collect();
        Polygon::new(new_vertices, self.metadata.clone())
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
        let pts_2d: Vec<Vec<f64>> = sum_pts.iter().map(|p| {
            let p_hom = p.to_homogeneous();
            let p2 = to_xy * p_hom;
            vec![p2[0], p2[1]]
        }).collect();
        let chull = ConvexHullWrapper::try_new(&pts_2d, None)
            .expect("Minkowski sum convex hull failed");
        let (hull_verts, _hull_indices) = chull.vertices_indices();
        let new_vertices = hull_verts.iter().map(|p| {
            // Make sure to tell Rust the type explicitly so that the multiplication produces
            // a Vector4<f64>.
            let p4: nalgebra::Vector4<f64> = nalgebra::Vector4::new(p[0], p[1], 0.0, 1.0);
            let p3 = from_xy * p4;
            Vertex::new(Point3::from_homogeneous(p3).unwrap(), self.plane.normal)
        }).collect();
        Polygon::new(new_vertices, self.metadata.clone())
    }
    
    /// Attempt to reconstruct arcs of constant radius in the 2D projection of this polygon,
    /// storing them as bulge arcs in the returned `Polyline<f64>`.
    ///
    /// # Parameters
    /// - `min_match`: minimal number of consecutive edges needed to consider forming an arc
    /// - `rms_limit`: max RMS fitting error (like `arcfinder`’s `options.rms_limit`)
    /// - `angle_limit_degs`: max total arc sweep in degrees 
    /// - `offset_limit`: additional limit used by `arcfinder` for chord offsets, etc.
    ///
    /// # Returns
    /// A single `Polyline<f64>` with arcs (encoded via bulge) or lines if no arcs found.
    ///
    pub fn reconstruct_arcs(
        &self,
        min_match: usize,
        rms_limit: f64,
        angle_limit_degs: f64,
        offset_limit: f64
    ) -> Polyline<f64>
    {
        // 1) Flatten or project to 2D. Suppose `to_2d()` returns a Polyline<f64> with .x, .y, .bulge:
        let poly_2d = self.to_2d(); 
        // If too few vertices, or degenerate
        if poly_2d.vertex_count() < 2 {
            return poly_2d;
        }
    
        // 2) Collect all points in a Vec<Point2<f64>>
        //    If polygon is closed, the polyline might be closed. We can handle it accordingly:
        let mut all_pts: Vec<Point2<f64>> = Vec::with_capacity(poly_2d.vertex_count());
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
    
        while i < n-1 {
            // Attempt to form an arc from i..some j≥i+min_match
            let start_pt = all_pts[i];
            let mut found_arc = false;
            let mut best_j = i+1;
            let mut best_arc_data: Option<(bool, f64, Point2<f64>)> = None;
    
            let mut j = i + min_match;
            while j < n {
                let pt_j = all_pts[j];
                let midslice = &all_pts[i+1 .. j];
                if let Some((cw, r, ctr, _rms)) = best_arc_fit(
                    start_pt, pt_j, midslice,
                    rms_limit, angle_limit_degs, offset_limit
                ) {
                    found_arc = true;
                    best_arc_data = Some((cw, r, ctr));
                    best_j = j;
                    j += 1;  // try extending more
                } else {
                    break;
                }
            }
    
            if found_arc {
                // we have an arc from i..best_j
                let end_pt = all_pts[best_j];
                let (cw, _r, c) = best_arc_data.unwrap();
    
                // compute angle from center => to find bulge
                let v0 = start_pt - c;  // v0 is a Vector2<f64>
                let v1 = end_pt - c;    // v1 is a Vector2<f64>
                let ang0 = v0.y.atan2(v0.x);
                let ang1 = v1.y.atan2(v1.x);
                let total_sweep = normalize_angle(ang1 - ang0);
                let arc_sweep = if cw { -total_sweep.abs() } else { total_sweep.abs() };
                // bulge = tan(sweep/4)
                let bulge = (arc_sweep * 0.25).tan();
    
                // set bulge on the last vertex in `result` (the arc start):
                let last_idx = result.vertex_count()-1;
                let mut last_v = result[last_idx];
                last_v.bulge = bulge;
                result.set_vertex(last_idx, last_v);
    
                // then add end vertex with bulge=0
                result.add(end_pt.x, end_pt.y, 0.0);
    
                i = best_j;
            } else {
                // no arc => just line from i->i+1
                let next_pt = all_pts[i+1];
                // set bulge=0 on the last output vertex
                let last_idx = result.vertex_count()-1;
                let mut lv = result[last_idx];
                lv.bulge = 0.0;
                result.set_vertex(last_idx, lv);
    
                result.add(next_pt.x, next_pt.y, 0.0);
    
                i += 1;
            }
            
        }
    
        result
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

/// A BSP tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    pub plane: Option<Plane>,
    pub front: Option<Box<Node<S>>>,
    pub back: Option<Box<Node<S>>>,
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone> Node<S> {
    pub fn new(polygons: Vec<Polygon<S>>) -> Self {
        let mut node = Node {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        };
        if !polygons.is_empty() {
            node.build(&polygons);
        }
        node
    }

    /// Invert all polygons in the BSP tree
    pub fn invert(&mut self) {
        for p in &mut self.polygons {
            p.flip();
        }
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }
        if let Some(ref mut front) = self.front {
            front.invert();
        }
        if let Some(ref mut back) = self.back {
            back.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        let mut front: Vec<Polygon<S>> = Vec::new();
        let mut back: Vec<Polygon<S>> = Vec::new();

        for poly in polygons {
            plane.split_polygon(
                poly,
                &mut Vec::new(), // coplanar_front
                &mut Vec::new(), // coplanar_back
                &mut front,
                &mut back,
            );
        }

        if let Some(ref f) = self.front {
            front = f.clip_polygons(&front);
        }
        if let Some(ref b) = self.back {
            back = b.clip_polygons(&back);
        } else {
            back.clear();
        }

        front.extend(back);
        front
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

    /// Return all polygons in this BSP tree
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = self.polygons.clone();
        if let Some(ref front) = self.front {
            result.extend(front.all_polygons());
        }
        if let Some(ref back) = self.back {
            result.extend(back.all_polygons());
        }
        result
    }

    /// Build a BSP tree from the given polygons
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();

        let mut front: Vec<Polygon<S>> = Vec::new();
        let mut back: Vec<Polygon<S>> = Vec::new();

        for p in polygons {
            let mut coplanar_front = Vec::new();
            let mut coplanar_back = Vec::new();

            plane.split_polygon(
                p,
                &mut coplanar_front,
                &mut coplanar_back,
                &mut front,
                &mut back,
            );

            self.polygons.append(&mut coplanar_front);
            self.polygons.append(&mut coplanar_back);
        }

        if !front.is_empty() {
            if self.front.is_none() {
                self.front = Some(Box::new(Node::new(vec![])));
            }
            self.front.as_mut().unwrap().build(&front);
        }

        if !back.is_empty() {
            if self.back.is_none() {
                self.back = Some(Box::new(Node::new(vec![])));
            }
            self.back.as_mut().unwrap().build(&back);
        }
    }
}

/// The main CSG solid structure. Contains a list of polygons.
#[derive(Debug, Clone)]
pub struct CSG<S: Clone> {
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone> CSG<S> {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG { polygons: Vec::new() }
    }

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: Vec<Polygon<S>>) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons;
        csg
    }

    /// Return the internal polygons
    pub fn to_polygons(&self) -> &[Polygon<S>] {
        &self.polygons
    }
    
    /// Group polygons by their metadata.
    ///
    /// Returns a map from the metadata (as `Option<S>`) to a
    /// list of references to all polygons that have that metadata.
    ///
    /// # Example
    /// ```
    /// let mut csg = CSG::new();
    /// // ... fill `csg.polygons` with some that share metadata, some that have None, etc.
    ///
    /// let grouped = csg.polygons_by_metadata();
    /// for (meta, polys) in &grouped {
    ///     println!("Metadata = {:?}, #polygons = {}", meta, polys.len());
    /// }
    /// ```
    /// requires impl<S: Clone + Eq + Hash> CSG<S> { and use std::collections::HashMap; use std::hash::Hash;
    ///pub fn polygons_by_metadata(&self) -> HashMap<Option<S>, Vec<&Polygon<S>>> {
    ///    let mut map: HashMap<Option<S>, Vec<&Polygon<S>>> = HashMap::new();
    ///    
    ///    for poly in &self.polygons {
    ///        // Clone the `Option<S>` so we can use it as the key
    ///        let key = poly.metadata.clone();
    ///        map.entry(key).or_default().push(poly);
    ///    }
    ///    
    ///    map
    ///}

    /// Return polygons grouped by metadata
    /// requires impl<S: Clone + std::cmp::PartialEq> CSG<S> {
    ///pub fn polygons_by_metadata_partialeq(&self) -> Vec<(Option<S>, Vec<&Polygon<S>>)> {
    ///    let mut groups: Vec<(Option<S>, Vec<&Polygon<S>>)> = Vec::new();
    ///    'outer: for poly in &self.polygons {
    ///        let meta = poly.metadata.clone();
    ///        // Try to find an existing group with the same metadata (requires a way to compare!)
    ///        for (existing_meta, polys) in &mut groups {
    ///            // For this to work, you need some form of comparison on `S`.
    ///            // If S does not implement Eq, you might do partial compare or pointer compare, etc.
    ///            if *existing_meta == meta {
    ///                polys.push(poly);
    ///                continue 'outer;
    ///            }
    ///        }
    ///        // Otherwise, start a new group
    ///        groups.push((meta, vec![poly]));
    ///    }
    ///    groups
    ///}
    
    /// Build a new CSG from a set of 2D polylines in XY. Each polyline
    /// is turned into one polygon at z=0. If a union produced multiple
    /// loops, you will get multiple polygons in the final CSG.
    pub fn from_cc_polylines(loops: Vec<Polyline<f64>>) -> CSG<S> {
        let mut all_polygons = Vec::new();
        let plane_normal = Vector3::new(0.0, 0.0, 1.0);
    
        for pl in loops {
            // Convert each Polyline into a single polygon in z=0.
            // For arcs, we could subdivide by bulge, etc. This ignores arcs for simplicity.
            if pl.vertex_count() >= 3 {
                let mut poly_verts = Vec::with_capacity(pl.vertex_count());
                for i in 0..pl.vertex_count() {
                    let v = pl.at(i);
                    poly_verts.push(Vertex::new(
                        nalgebra::Point3::new(v.x, v.y, 0.0),
                        plane_normal
                    ));
                }
                all_polygons.push(Polygon::new(poly_verts, None));
            }
        }
    
        CSG::from_polygons(all_polygons)
    }
    
    /// Constructs a new CSG solid from “complex” (possibly non‐triangulated)
    /// polygons provided in the same format that earclip accepts:
    /// a slice of polygons, each a Vec of points (each point a Vec<f64> of length 2 or 3).
    ///
    /// The routine “flattens” the input into a flat list of coordinates and a list of hole indices,
    /// then uses earcut to tessellate the outline into triangles.
    ///
    /// Each triangle is then converted into a Polygon (with three vertices) with a default normal
    /// (here assumed to be pointing along +Z if the polygon is 2D) and no metadata.
    ///
    /// # Example
    ///
    /// ```rust
    /// // A square with a square hole:
    /// let outer = vec![
    ///     vec![0.0, 0.0],
    ///     vec![10.0, 0.0],
    ///     vec![10.0, 10.0],
    ///     vec![0.0, 10.0],
    /// ];
    /// let hole = vec![
    ///     vec![3.0, 3.0],
    ///     vec![7.0, 3.0],
    ///     vec![7.0, 7.0],
    ///     vec![3.0, 7.0],
    /// ];
    ///
    /// // In earclip’s expected format, the first polygon is the outer loop,
    /// // and any subsequent ones are holes:
    /// let polys = vec![outer, hole];
    ///
    /// let csg = CSG::<()>::from_complex_polygons(&polys);
    /// // Now csg.polygons contains the triangulated version.
    /// ```
    pub fn from_complex_polygons(polys: &[Vec<Vec<f64>>]) -> CSG<S> {
        // Flatten the input. (The earclip::flatten function returns a tuple:
        // (flat_vertices, hole_indices, dim). For example, if the input is 2D,
        // dim will be 2.)
        let (vertices, hole_indices, dim) = flatten(polys);
        // Tessellate the flattened polygon using earcut.
        let indices: Vec<usize> = earcut(&vertices, &hole_indices, dim);
        
        let mut new_polygons = Vec::new();
        // Each consecutive triple in the output indices defines a triangle.
        for tri in indices.chunks_exact(3) {
            let mut tri_vertices = Vec::with_capacity(3);
            for &i in tri {
                let start = i * dim;
                // Build a 3D point.
                let p = if dim == 2 {
                    // If 2D, assume z = 0.
                    Point3::new(vertices[start], vertices[start + 1], 0.0)
                } else {
                    // If 3D (or higher) use the first three coordinates.
                    Point3::new(vertices[start], vertices[start + 1], vertices[start + 2])
                };
                // Here we simply assign a default normal pointing up.
                // (In a more advanced implementation you might compute
                // the true face normal from the triangle vertices.)
                let normal = Vector3::new(0.0, 0.0, 1.0);
                tri_vertices.push(Vertex::new(p, normal));
            }
            // Create a polygon (triangle) with no metadata.
            new_polygons.push(Polygon::new(tri_vertices, None));
        }
        CSG::from_polygons(new_polygons)
    }

    /// CSG union: this ∪ other
    pub fn union(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(self.polygons.clone());
        let mut b = Node::new(other.polygons.clone());

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        CSG::from_polygons(a.all_polygons())
    }

    /// CSG subtract: this \ other
    pub fn subtract(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(self.polygons.clone());
        let mut b = Node::new(other.polygons.clone());

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        CSG::from_polygons(a.all_polygons())
    }

    /// CSG intersect: this ∩ other
    pub fn intersect(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(self.polygons.clone());
        let mut b = Node::new(other.polygons.clone());

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        CSG::from_polygons(a.all_polygons())
    }

    /// Invert this CSG (flip inside vs. outside)
    pub fn inverse(&self) -> CSG<S> {
        let mut csg = self.clone();
        for p in &mut csg.polygons {
            p.flip();
        }
        csg
    }
    
    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `size`: the width and height of the square (default [1.0, 1.0])
    /// - `center`: if `true`, center the square about (0,0); otherwise bottom-left is at (0,0).
    ///
    /// # Example
    /// let sq = CSG::square(None);
    /// // or with custom params:
    /// let sq2 = CSG::square(Some(([2.0, 3.0], true)));
    pub fn square(params: Option<([f64; 2], bool)>) -> CSG<S> {
        let (size, center) = match params {
            Some((sz, c)) => (sz, c),
            None => ([1.0, 1.0], false),
        };

        let (w, h) = (size[0], size[1]);
        let (x0, y0, x1, y1) = if center {
            (-w / 2.0, -h / 2.0, w / 2.0, h / 2.0)
        } else {
            (0.0, 0.0, w, h)
        };

        // Single 2D polygon, normal = +Z
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let vertices = vec![
            Vertex::new(Point3::new(x0, y0, 0.0), normal),
            Vertex::new(Point3::new(x1, y0, 0.0), normal),
            Vertex::new(Point3::new(x1, y1, 0.0), normal),
            Vertex::new(Point3::new(x0, y1, 0.0), normal),
        ];
        CSG::from_polygons(vec![Polygon::new(vertices, None)])
    }

    /// Creates a 2D circle in the XY plane.
    pub fn circle(params: Option<(f64, usize)>) -> CSG<S> {
        let (r, segments) = match params {
            Some((radius, segs)) => (radius, segs),
            None => (1.0, 32),
        };

        let mut verts = Vec::with_capacity(segments);
        let normal = Vector3::new(0.0, 0.0, 1.0);

        for i in 0..segments {
            let theta = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
            let x = r * theta.cos();
            let y = r * theta.sin();
            verts.push(Vertex::new(Point3::new(x, y, 0.0), normal));
        }

        CSG::from_polygons(vec![Polygon::new(verts, None)])
    }

    /// Creates a 2D polygon in the XY plane from a list of `[x, y]` points.
    ///
    /// # Parameters
    ///
    /// - `points`: a sequence of 2D points (e.g. `[[0.0,0.0], [1.0,0.0], [0.5,1.0]]`)
    ///   describing the polygon boundary in order.
    ///
    /// # Example
    /// let pts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]];
    /// let poly2d = CSG::polygon_2d(&pts);
    pub fn polygon_2d(points: &[[f64; 2]]) -> CSG<S> {
        assert!(points.len() >= 3, "polygon_2d requires at least 3 points");
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let mut verts = Vec::with_capacity(points.len());
        for p in points {
            verts.push(Vertex::new(Point3::new(p[0], p[1], 0.0), normal));
        }
        CSG::from_polygons(vec![Polygon::new(verts, None)])
    }

    /// Construct an axis-aligned cube by creating a 2D square and then
    /// extruding it between two parallel planes (using `extrude_between`).
    /// We translate the final shape so that its center is at `center`,
    /// and each axis extends ± its corresponding `radius`.
    pub fn cube(options: Option<(&[f64; 3], &[f64; 3])>) -> CSG<S> {
        let (center, radius) = match options {
            Some((c, r)) => (*c, *r),
            None => ([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
        };
        let c = Vector3::new(center[0], center[1], center[2]);
        let r = Vector3::new(radius[0], radius[1], radius[2]);

        let indices_and_normals = vec![
            (vec![0, 4, 6, 2], Vector3::new(-1.0, 0.0, 0.0)),
            (vec![1, 3, 7, 5], Vector3::new(1.0, 0.0, 0.0)),
            (vec![0, 1, 5, 4], Vector3::new(0.0, -1.0, 0.0)),
            (vec![2, 6, 7, 3], Vector3::new(0.0, 1.0, 0.0)),
            (vec![0, 2, 3, 1], Vector3::new(0.0, 0.0, -1.0)),
            (vec![4, 5, 7, 6], Vector3::new(0.0, 0.0, 1.0)),
        ];

        let mut polygons = Vec::new();
        for (idxs, n) in indices_and_normals {
            let mut verts = Vec::new();
            for i in idxs {
                // The bits of `i` pick +/- for x,y,z
                let vx = c.x + r.x * ((i & 1) as f64 * 2.0 - 1.0);
                let vy = c.y + r.y * (((i & 2) >> 1) as f64 * 2.0 - 1.0);
                let vz = c.z + r.z * (((i & 4) >> 2) as f64 * 2.0 - 1.0);
                verts.push(Vertex::new(Point3::new(vx, vy, vz), n));
            }
            polygons.push(Polygon::new(verts, None));
        }

        CSG::from_polygons(polygons)
    }

    /// Construct a sphere with optional center, radius, slices, stacks
    pub fn sphere(options: Option<(&[f64; 3], f64, usize, usize)>) -> CSG<S> {
        let (center, radius, slices, stacks) = match options {
            Some((c, r, sl, st)) => (*c, r, sl, st),
            None => ([0.0, 0.0, 0.0], 1.0, 16, 8),
        };

        let c = Vector3::new(center[0], center[1], center[2]);
        let mut polygons = Vec::new();

        for i in 0..slices {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: f64, phi: f64| {
                    let dir = Vector3::new(
                        theta.cos() * phi.sin(),
                        phi.cos(),
                        theta.sin() * phi.sin(),
                    );
                    Vertex::new(Point3::new(c.x + dir.x * radius,
                                            c.y + dir.y * radius,
                                            c.z + dir.z * radius),
                                dir)
                };

                let t0 = i as f64 / slices as f64;
                let t1 = (i + 1) as f64 / slices as f64;
                let p0 = j as f64 / stacks as f64;
                let p1 = (j + 1) as f64 / stacks as f64;

                let theta0 = t0 * std::f64::consts::TAU;
                let theta1 = t1 * std::f64::consts::TAU;
                let phi0 = p0 * std::f64::consts::PI;
                let phi1 = p1 * std::f64::consts::PI;

                vertices.push(vertex(theta0, phi0));
                if j > 0 {
                    vertices.push(vertex(theta1, phi0));
                }
                if j < stacks - 1 {
                    vertices.push(vertex(theta1, phi1));
                }
                vertices.push(vertex(theta0, phi1));

                polygons.push(Polygon::new(vertices, None));
            }
        }

        CSG::from_polygons(polygons)
    }

    /// Construct a cylinder whose centerline goes from `start` to `end`,
    /// with a circular cross-section of given `radius`. We first build a
    /// 2D circle in the XY plane, then extrude two copies of it in the Z
    /// direction (via `extrude_between`), and finally rotate/translate
    /// so that the cylinder aligns with `start -> end`.
    pub fn cylinder(options: Option<(&[f64; 3], &[f64; 3], f64, usize)>) -> CSG<S> {
        let (start, end, radius, slices) = match options {
            Some((s, e, r, sl)) => (*s, *e, r, sl),
            None => ([0.0, -1.0, 0.0], [0.0, 1.0, 0.0], 1.0, 16),
        };

        let s = Vector3::new(start[0], start[1], start[2]);
        let e = Vector3::new(end[0], end[1], end[2]);
        let ray = e - s;
        let axis_z = ray.normalize();
        let is_y = axis_z.y.abs() > 0.5;

        // If axis_z is mostly aligned with Y, pick X; otherwise pick Y.
        let mut axis_x = if is_y {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        axis_x = axis_x.cross(&axis_z).normalize();
        let axis_y = axis_x.cross(&axis_z).normalize();

        let start_v = Vertex::new(Point3::from(s), -axis_z);
        let end_v = Vertex::new(Point3::from(e), axis_z);

        let mut polygons = Vec::new();

        let point = |stack: f64, slice: f64, normal_blend: f64| {
            let angle = slice * std::f64::consts::TAU;
            let out = axis_x * angle.cos() + axis_y * angle.sin();
            let pos = s + ray * stack + out * radius;
            // Blend outward normal with axis_z for the cap edges
            let normal = out * (1.0 - normal_blend.abs()) + axis_z * normal_blend;
            Vertex::new(Point3::from(pos), normal)
        };

        for i in 0..slices {
            let t0 = i as f64 / slices as f64;
            let t1 = (i + 1) as f64 / slices as f64;

            // bottom cap
            polygons.push(Polygon::new(
                vec![
                    start_v.clone(),
                    point(0.0, t0, -1.0),
                    point(0.0, t1, -1.0),
                ],
                None,
            ));

            // tube
            polygons.push(Polygon::new(
                vec![
                    point(0.0, t1, 0.0),
                    point(0.0, t0, 0.0),
                    point(1.0, t0, 0.0),
                    point(1.0, t1, 0.0),
                ],
                None,
            ));

            // top cap
            polygons.push(Polygon::new(
                vec![
                    end_v.clone(),
                    point(1.0, t1, 1.0),
                    point(1.0, t0, 1.0),
                ],
                None,
            ));
        }

        CSG::from_polygons(polygons)
    }

    /// Creates a CSG polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// # Parameters
    ///
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and a pyramid side [0,1,4]
    /// let fcs = vec![
    ///     vec![0, 1, 2, 3],
    ///     vec![0, 1, 4],
    ///     vec![1, 2, 4],
    ///     vec![2, 3, 4],
    ///     vec![3, 0, 4],
    /// ];
    ///
    /// let csg_poly = CSG::polyhedron(pts, &fcs);
    /// ```
    pub fn polyhedron(points: &[[f64; 3]], faces: &[Vec<usize>]) -> CSG<S> {
        let mut polygons = Vec::new();

        for face in faces {
            // Skip degenerate faces
            if face.len() < 3 {
                continue;
            }

            // Gather the vertices for this face
            let mut face_vertices = Vec::with_capacity(face.len());
            for &idx in face {
                // Ensure the index is valid
                if idx >= points.len() {
                    panic!("Face index {} is out of range (points.len = {}).", idx, points.len());
                }
                let [x, y, z] = points[idx];
                face_vertices.push(Vertex::new(
                    Point3::new(x, y, z),
                    Vector3::zeros(), // we'll set this later
                ));
            }

            // Build the polygon (plane is auto-computed from first 3 vertices).
            let mut poly = Polygon::new(face_vertices, None);

            // Optionally, set each vertex normal to match the polygon’s plane normal,
            // so that shading in many 3D viewers looks correct.
            let plane_normal = poly.plane.normal;
            for v in &mut poly.vertices {
                v.normal = plane_normal;
            }
            polygons.push(poly);
        }

        CSG::from_polygons(polygons)
    }

    /// Transform all vertices in this CSG by a given 4×4 matrix.
    pub fn transform(&self, mat: &Matrix4<f64>) -> CSG<S> {
        let mat_inv_transpose = mat.try_inverse().unwrap().transpose();
        let mut csg = self.clone();

        for poly in &mut csg.polygons {
            for vert in &mut poly.vertices {
                // Position
                let hom_pos = mat * vert.pos.to_homogeneous();
                vert.pos = Point3::from_homogeneous(hom_pos).unwrap();
    
                // Normal
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }
    
            // Plane normal
            poly.plane.normal = mat_inv_transpose.transform_vector(&poly.plane.normal).normalize();
    
            // Plane w
            if let Some(first_vert) = poly.vertices.get(0) {
                poly.plane.w = poly.plane.normal.dot(&first_vert.pos.coords);
            }
        }

        csg
    }

    pub fn translate(&self, v: Vector3<f64>) -> CSG<S> {
        let translation = Translation3::from(v);
        // Convert to a Matrix4
        let mat4 = translation.to_homogeneous();
        self.transform(&mat4)
    }

    pub fn rotate(&self, x_deg: f64, y_deg: f64, z_deg: f64) -> CSG<S> {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());
        
        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> CSG<S> {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    /// Mirror across X=0, Y=0, or Z=0 plane
    pub fn mirror(&self, axis: Axis) -> CSG<S> {
        let (sx, sy, sz) = match axis {
            Axis::X => (-1.0, 1.0, 1.0),
            Axis::Y => (1.0, -1.0, 1.0),
            Axis::Z => (1.0, 1.0, -1.0),
        };

        // We can just use a "non-uniform scaling" matrix that
        // flips exactly one axis:
        let mat = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat)
    }

    /// Compute the convex hull of all vertices in this CSG.
    pub fn convex_hull(&self) -> CSG<S> {
	// Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Vec<f64>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.vertices.iter().map(|v| vec![v.pos.x, v.pos.y, v.pos.z])
            })
            .collect();

        // Compute convex hull using the robust wrapper
        let hull = ConvexHullWrapper::try_new(&points, None)
            .expect("Failed to compute convex hull");

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons as triangles
        // todo: replace with filter / iterator
        let mut polygons = Vec::new();
        for tri in indices.chunks(3) {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];
            let vv0 = Vertex::new(Point3::new(v0[0], v0[1], v0[2]), Vector3::zeros());
            let vv1 = Vertex::new(Point3::new(v1[0], v1[1], v1[2]), Vector3::zeros());
            let vv2 = Vertex::new(Point3::new(v2[0], v2[1], v2[2]), Vector3::zeros());
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
        }

        CSG::from_polygons(polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// Naive approach: Take every vertex in `self`, add it to every vertex in `other`,
    /// then compute the convex hull of all resulting points.
    pub fn minkowski_sum(&self, other: &CSG<S>) -> CSG<S> {
        // Collect all vertices (x, y, z) from self
        let verts_a: Vec<Point3<f64>> = self.polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        // Collect all vertices from other
        let verts_b: Vec<Point3<f64>> = other.polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();
            
        if verts_a.is_empty() || verts_b.is_empty() {
            // Empty input to minkowski sum
        }

        // For Minkowski, add every point in A to every point in B
        let sum_points: Vec<_> = verts_a.iter()
            .flat_map(|a| verts_b.iter().map(move |b| a + b.coords))
            .map(|v| vec![v.x, v.y, v.z])
            .collect();

        // Compute the hull of these Minkowski-sum points
        let hull = ConvexHullWrapper::try_new(&sum_points, None)
            .expect("Failed to compute Minkowski sum hull");
        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons
        let mut polygons = Vec::new();
        for tri in indices.chunks(3) {
            let v0 = &verts[tri[0]];
            let v1 = &verts[tri[1]];
            let v2 = &verts[tri[2]];
            let vv0 = Vertex::new(Point3::new(v0[0], v0[1], v0[2]), Vector3::zeros());
            let vv1 = Vertex::new(Point3::new(v1[0], v1[1], v1[2]), Vector3::zeros());
            let vv2 = Vertex::new(Point3::new(v2[0], v2[1], v2[2]), Vector3::zeros());
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
        }

        CSG::from_polygons(polygons)
    }

    /// Subdivide all polygons in this CSG 'levels' times, returning a new CSG.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: u32) -> CSG<S> {
        if levels == 0 {
            return self.clone();
        }

        let mut new_polygons = Vec::new();
        for poly in &self.polygons {
            // Subdivide the polygon into many smaller triangles
            let sub_tris = poly.subdivide_triangles(levels);
            // Convert each small tri back into a Polygon with 3 vertices
            // (you can keep the same metadata or None).
            for tri in sub_tris {
                new_polygons.push(
                    Polygon::new(vec![tri[0].clone(), tri[1].clone(), tri[2].clone()], poly.metadata.clone())
                );
            }
        }

        CSG::from_polygons(new_polygons)
    }

    /// Renormalize all polygons in this CSG by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.recalc_plane_and_normals();
        }
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this CSG and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<f64>, f64)` where:
    /// - `Point3<f64>` is the intersection coordinate in 3D,
    /// - `f64` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<f64>,
        direction: &Vector3<f64>,
    ) -> Vec<(Point3<f64>, f64)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        let mut hits = Vec::new();

        // 1) For each polygon in the CSG:
        for poly in &self.polygons {
            // 2) Triangulate it if necessary:
            let triangles = poly.triangulate();

            // 3) For each triangle, do a ray–triangle intersection test:
            for tri in triangles {
                let a = tri[0].pos;
                let b = tri[1].pos;
                let c = tri[2].pos;

                // Construct a parry Triangle shape from the 3 vertices:
                let triangle = Triangle::new(a, b, c);

                // Ray-cast against the triangle:
                if let Some(hit) = triangle.cast_ray_and_get_normal(&iso, &ray, f64::MAX, true) {
                    let point_on_ray = ray.point_at(hit.time_of_impact);
                    hits.push((
                        Point3::from(point_on_ray.coords),
                        hit.time_of_impact,
                    ));
                }
            }
        }

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < EPSILON);

        hits
    }
    
    /// Linearly extrude this (2D) shape in the +Z direction by `height`.
    ///
    /// This is just a convenience wrapper around extrude_vector using Vector3::new(0.0, 0.0, height)
    pub fn extrude(&self, height: f64) -> CSG<S> {
        self.extrude_vector(Vector3::new(0.0, 0.0, height))
    }
    
    /// Linearly extrude this (2D) shape along an arbitrary 3D direction vector.
    ///
    /// - The shape is “swept” from its original location to a translated “top” copy
    ///   offset by `direction`.
    /// - Side walls are formed between the bottom and top edges.
    /// - The shape is assumed to be “2D” in the sense that each polygon typically
    ///   lies in a single plane (e.g. XY). For best results, your polygons’ normals
    ///   should be consistent.
    pub fn extrude_vector(&self, direction: Vector3<f64>) -> CSG<S> {
        // Collect all new polygons here
        let mut new_polygons = Vec::new();
    
        // 1) Bottom polygons = original polygons
        //    (assuming they are in some plane, e.g. XY). We just clone them.
        for poly in &self.polygons {
            new_polygons.push(poly.clone());
        }
    
        // 2) Top polygons = translate each original polygon by `direction`.
        //    The orientation of their normals will remain the same unless you decide to flip them.
        let top_polygons = self.translate(direction).polygons;
        new_polygons.extend(top_polygons.iter().cloned());
    
        // 3) Side polygons = For each polygon in `self`, connect its edges
        //    from the original to the corresponding edges in the translated version.
        //
        //    We'll iterate over each polygon’s vertices. For each edge (v[i], v[i+1]),
        //    we form a rectangular side quad with (v[i]+direction, v[i+1]+direction).
        //    That is, a quad [b_i, b_j, t_j, t_i].
        let bottom_polys = &self.polygons;
        let top_polys = &top_polygons;
    
        for (poly_bottom, poly_top) in bottom_polys.iter().zip(top_polys.iter()) {
            let vcount = poly_bottom.vertices.len();
            if vcount < 3 {
                continue; // skip degenerate or empty polygons
            }
            for i in 0..vcount {
                let j = (i + 1) % vcount; // next index, wrapping around
                let b_i = &poly_bottom.vertices[i];
                let b_j = &poly_bottom.vertices[j];
                let t_i = &poly_top.vertices[i];
                let t_j = &poly_top.vertices[j];
    
                // Build a side quad [b_i, b_j, t_j, t_i].
                // Then push it as a new polygon.
                let side_poly = Polygon::new(
                    vec![b_i.clone(), b_j.clone(), t_j.clone(), t_i.clone()],
                    None
                );
                new_polygons.push(side_poly);
            }
        }
    
        // Combine into a new CSG
        CSG::from_polygons(new_polygons)
    }
    
    /// Extrudes (or "lofts") a closed 3D volume between two polygons in space.
    /// - `bottom` and `top` each have the same number of vertices `n`, in matching order.
    /// - Returns a new CSG whose faces are:
    ///   - The `bottom` polygon,
    ///   - The `top` polygon,
    ///   - `n` rectangular side polygons bridging each edge of `bottom` to the corresponding edge of `top`.
    pub fn extrude_between(bottom: &Polygon<S>, top: &Polygon<S>, flip_bottom_polygon: bool) -> CSG<S> {
        let n = bottom.vertices.len();
        assert_eq!(
            n,
            top.vertices.len(),
            "extrude_between: both polygons must have the same number of vertices"
        );
    
        // Conditionally flip the bottom polygon if requested.
        let bottom_poly = if flip_bottom_polygon {
            let mut flipped = bottom.clone();
            flipped.flip();
            flipped
        } else {
            bottom.clone()
        };
    
        // 1) Gather polygons: bottom + top
        //    (Depending on the orientation, you might want to flip one of them.)

        let mut polygons = vec![bottom_poly.clone(), top.clone()];
    
        // 2) For each edge (i -> i+1) in bottom, connect to the corresponding edge in top.
        for i in 0..n {
            let j = (i + 1) % n;
            let b_i = &bottom.vertices[i];
            let b_j = &bottom.vertices[j];
            let t_i = &top.vertices[i];
            let t_j = &top.vertices[j];
    
            // Build the side face as a 4-vertex polygon (quad).
            // Winding order here is chosen so that the polygon's normal faces outward 
            // (depending on the orientation of bottom vs. top).
            let side_poly = Polygon::new(
                vec![
                    b_i.clone(), // bottom[i]
                    b_j.clone(), // bottom[i+1]
                    t_j.clone(), // top[i+1]
                    t_i.clone(), // top[i]
                ],
                None, // or carry over some metadata if you wish
            );
            polygons.push(side_poly);
        }
    
        CSG::from_polygons(polygons)
    }

    /// Rotate-extrude (revolve) this 2D shape around the Z-axis from 0..`angle_degs`
    /// by replicating the original polygon(s) at each step and calling `extrude_between`.
    /// Caps are added automatically if the revolve is partial (angle < 360°).
    pub fn rotate_extrude(&self, angle_degs: f64, segments: usize) -> CSG<S> {
        let angle_radians = angle_degs.to_radians();
        if segments < 2 {
            panic!("rotate_extrude requires at least 2 segments");
        }
    
        // We'll consider the revolve "closed" if the angle is effectively 360°
        let closed = (angle_degs - 360.0).abs() < EPSILON;
    
        // Collect all newly formed polygons here
        let mut result_polygons = Vec::new();
    
        // For each polygon in our original 2D shape:
        for original_poly in &self.polygons {
            let n_verts = original_poly.vertices.len();
            if n_verts < 3 {
                // Skip degenerate or empty polygons
                continue;
            }
    
            // 1) Create a list of rotated copies ("slices") of `original_poly`.
            //    We'll generate `segments+1` slices if it's a partial revolve,
            //    so that slices[0] = 0° and slices[segments] = angle_degs,
            //    giving us "segments" intervals to extrude_between.
            //    If `angle_degs == 360`, slices[segments] ends up co-located with slices[0].
            let mut slices = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let frac = i as f64 / segments as f64;
                let theta = frac * angle_radians;
    
                // Build a rotation around Z by `theta`
                let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), theta).to_homogeneous();
    
                // Transform this single polygon by that rotation
                let rotated_poly = CSG::from_polygons(vec![original_poly.clone()])
                    .transform(&rot)
                    .polygons[0]
                    .clone();
                slices.push(rotated_poly);
            }
    
            // 2) "Loft" between successive slices using `extrude_between`.
            //    - If it's a full 360 revolve, we do 0..(segments) and wrap around 
            //      from slices[segments-1] => slices[0].
            //    - If it's partial, we just do 0..(segments), which covers 
            //      slices[i] -> slices[i+1] for i=0..(segments-1).
            for i in 0..(segments) {
                let bottom = &slices[i];
                let top = if closed {
                    &slices[(i + 1) % slices.len()] // Wrap around if `closed` is true
                } else {
                    &slices[i + 1] // Direct access if `closed` is false
                };
                let mut side_solid = CSG::extrude_between(bottom, top, true).polygons;
                result_polygons.append(&mut side_solid);
            }

            // Add "cap" for the last slice so the revolve is closed at end.
            // The end cap is slices[segments] as-is:
            if !closed {
                let end_cap = slices[segments].clone();
                result_polygons.push(end_cap);
            }
        }
    
        // Gather everything into a new CSG
        CSG::from_polygons(result_polygons) // todo: figure out why rotate_extrude results in inverted solids
    }

    /// Returns a `parry3d::bounding_volume::Aabb`.
    pub fn bounding_box(&self) -> Aabb {
        // Gather all points from all polygons.
        // parry expects a slice of `&Point3<f64>` or a slice of `na::Point3<f64>`.
        let mut all_points = Vec::new();
        for poly in &self.polygons {
            for v in &poly.vertices {
                all_points.push(v.pos);
            }
        }

        // If empty, return a degenerate AABB at origin or handle accordingly
        if all_points.is_empty() {
            return Aabb::new_invalid(); // or AABB::new(Point3::origin(), Point3::origin());
        }

        // Construct the parry AABB from points
        Aabb::from_points(&all_points)
    }

    /// Helper to collect all vertices from the CSG.
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Approximate growing (outward offset) of the shape by a given distance (3D).
    /// This method unions translated copies of the shape along a sphere.
    pub fn grow(&self, distance: f64) -> CSG<S> {
        let resolution = 32;
        let sphere: CSG<S> = CSG::sphere(Some((&[0.0, 0.0, 0.0], distance, resolution, resolution / 2)));
        let sphere_vertices = sphere.vertices();
        let mut result = CSG::new();

        // Union the shape translated by each vertex of the sphere.
        for v in sphere_vertices {
            result = result.union(&self.translate(v.pos.coords));
        }
        result
    }

    /// Approximate shrinking (inward offset) of the shape by a given distance (3D).
    /// This method unions translated copies of the complement of the shape along a sphere,
    /// then inverts the result.
    pub fn shrink(&self, distance: f64) -> CSG<S> {
        let resolution = 32;
        let sphere: CSG<S> = CSG::sphere(Some((&[0.0, 0.0, 0.0], distance, resolution, resolution / 2)));
        let sphere_vertices = sphere.vertices();
        let complement = self.inverse();
        let mut result = CSG::new();

        for v in sphere_vertices {
            result = result.union(&complement.translate(v.pos.coords));
        }
        result.inverse()
    }

    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using cavalier_contours parallel_offset.
    /// for each Polygon we convert to a cavalier_contours Polyline<f64> and call parallel_offset
    pub fn offset_2d(&self, distance: f64) -> CSG<S> {
        let mut offset_loops = Vec::new(); // each "loop" is a cavalier_contours polyline

        for poly in &self.polygons {
            // Convert to cavalier_contours Polyline (closed by default):
            let cpoly = poly.to_xy();

            // Remove any degenerate or redundant vertices:
            cpoly.remove_redundant(1e-5);

            // Perform the actual offset:
            let result_plines = cpoly.parallel_offset(-distance);

            // Collect this loop
            offset_loops.extend(result_plines);
        }

        // Build a new CSG from those offset loops in XY:
        CSG::from_cc_polylines(offset_loops)
    }

    /// Flatten a `CSG` into the XY plane and union all polygons' outlines,
    /// returning a new `CSG` that may contain multiple polygons (loops) if disjoint.
    ///
    /// We skip "degenerate" loops whose area is near zero, both before
    /// and after performing the union. This helps avoid collinear or
    /// duplicate edges that can cause issues in `cavalier_contours`.
    pub fn flatten(&self) -> CSG<S> {
        let eps_area = 1e-9;

        // Convert each 3D polygon to a 2D polygon in XY (z=0).
        // Filter out degenerate polygons (area ~ 0).
        let mut polys_2d = Vec::new();
        for poly in &self.polygons {
            // Convert to a cavalier_contours::Polyline first (same as .to_cc_polyline())
            let cc = poly.to_xy();
            // Optional: remove redundant points
            cc.remove_redundant(EPSILON);

            // Check area (shoelace). If above threshold, turn it into a 2D Polygon
            let area = crate::pline_area(&cc).abs();
            if area > eps_area {
                polys_2d.push(Polygon::from_xy(cc));
            }
        }
        if polys_2d.is_empty() {
            return CSG::new();
        }

        // Use our `union_all_2d` helper
        let merged_2d = union_all_2d(&polys_2d);

        // Return them as a new CSG in z=0
        CSG::from_polygons(merged_2d)
    }
    
    /// Slice this CSG by a plane, keeping only cross-sections on that plane.
    /// If `plane` is None, defaults to the plane z=0.
    pub fn cut(&self, plane: Option<Plane>) -> CSG<S> {

        let plane = plane.unwrap_or_else(|| Plane {
            normal: nalgebra::Vector3::new(0.0, 0.0, 1.0),
            w: 0.0,
        });
    
        let mut result_polygons = Vec::new();
    
        // For each polygon in the original CSG:
        for poly in &self.polygons {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // skip degenerate
            }
    
            // Classify each vertex against plane
            // side[i] = +1 if above plane, -1 if below plane, 0 if on plane
            let mut sides = Vec::with_capacity(vcount);
            for v in &poly.vertices {
                let dist = plane.normal.dot(&v.pos.coords) - plane.w;
                if dist.abs() < EPSILON {
                    sides.push(0); // on plane
                } else if dist > 0.0 {
                    sides.push(1); // above
                } else {
                    sides.push(-1); // below
                }
            }
    
            // Collect the points where the polygon intersects the plane
            let mut intersect_points = Vec::new();
    
            for i in 0..vcount {
                let j = (i + 1) % vcount;
                let side_i = sides[i];
                let side_j = sides[j];
                let vi = &poly.vertices[i];
                let vj = &poly.vertices[j];
    
                // If a vertex lies exactly on the plane, include it
                if side_i == 0 {
                    intersect_points.push(vi.pos);
                }
    
                // If edges cross the plane, find intersection
                if side_i != side_j && side_i != 0 && side_j != 0 {
                    let denom = plane.normal.dot(&(vj.pos - vi.pos));
                    if denom.abs() > EPSILON {
                        let t = (plane.w - plane.normal.dot(&vi.pos.coords)) / denom;
                        let new_v = vi.interpolate(vj, t).pos;
                        intersect_points.push(new_v);
                    }
                }
            }
    
            // If fewer than 3 intersection points, no valid cross-section
            if intersect_points.len() < 3 {
                continue;
            }
    
            // Sort intersection points around their average center so they form a proper polygon
            let mut avg = nalgebra::Vector3::zeros();
            for p in &intersect_points {
                avg += p.coords;
            }
            avg /= intersect_points.len() as f64;
    
            intersect_points.sort_by(|a, b| {
                let ax = a.x - avg.x;
                let ay = a.y - avg.y;
                let bx = b.x - avg.x;
                let by = b.y - avg.y;
                let angle_a = ay.atan2(ax);
                let angle_b = by.atan2(bx);
                angle_a
                    .partial_cmp(&angle_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
    
            // Build a new Polygon (in z=0, normal = +Z)
            let normal_2d = nalgebra::Vector3::new(0.0, 0.0, 1.0);
            let new_verts: Vec<Vertex> = intersect_points
                .into_iter()
                .map(|p| Vertex::new(nalgebra::Point3::new(p.x, p.y, 0.0), normal_2d))
                .collect();
    
            result_polygons.push(Polygon::new(new_verts, poly.metadata.clone()));
        }
    
        CSG::from_polygons(result_polygons)
    }
    
    /// Slice this CSG by `plane`, returning the cross‐section(s) in that plane
    /// as a new CSG of 2D polygons. If `plane` is `None`, slices at z=0.
    pub fn cut_slab(&self, plane: Option<Plane>) -> CSG<S> {
        // 1) Pick the plane or default to z=0
        let plane = plane.unwrap_or_else(|| Plane {
            normal: nalgebra::Vector3::new(0.0, 0.0, 1.0),
            w: 0.0,
        });
        
        // 2) Build a transform T that maps our plane → the XY plane at z=0.
        //
        //    - We want plane.normal to become +Z
        //    - We want the plane’s “offset” (the point where plane.normal·p = w) to lie at z=0
        //
        //    The easiest way is:
        //      A) Rotate plane.normal to +Z
        //      B) Translate so that the plane’s reference point is at z=0
        //
        //    Let’s find a small point on that plane: p0 = plane.normal * (plane.w / normal.len²).
        let n = plane.normal;
        let n_len = n.magnitude();
        if n_len < 1e-12 {
            // Degenerate plane? Just bail or return empty
            return CSG::new();
        }
        let norm_dir = n / n_len;    // normalized
        let p0 = norm_dir * plane.w; // a point on the plane in 3D
        
        // A) Rotate `norm_dir` to +Z
        //    We can do so by e.g. Rotation3::rotation_between(&norm_dir, &Vector3::z())
        let to_z = Rotation3::rotation_between(&norm_dir, &Vector3::z())
            .unwrap_or_else(|| Rotation3::identity());
        
        // B) Then translate so that p0 goes to z=0
        //    After rotation, p0 is somewhere. We want its z to become 0 => so we shift by -p0.z
        //    We can do it by building an isometry that does the rotation, then find the new p0,
        //    then do a translation that sets the new p0.z = 0.
        let iso_rot = Isometry3::from_parts(Translation3::identity(), to_z.into());
        let p0_rot = iso_rot.transform_point(&Point3::from(p0));
        let shift_z = -p0_rot.z;
        let iso_trans = Translation3::new(0.0, 0.0, shift_z);

        // Combined transform T = translate * rotate
        let transform_to_xy = iso_trans.to_homogeneous() * iso_rot.to_homogeneous();
        
        // Transform shape into the plane’s coordinate system so that the plane is now z=0.
        let csg_in_plane_coords = self.transform(&transform_to_xy);

        // 3) Build a big “slab” in z=[-ε, +ε], with a big bounding square in XY.
        //    First find bounding box of csg_in_plane_coords to decide how big the slab must be.
        let aabb = csg_in_plane_coords.bounding_box();
        let mins = aabb.mins;
        let maxs = aabb.maxs;
        
        // We can consider the Aabb "invalid" if mins is not <= maxs in each component.
        // Alternatively, you could check for degenerate bounding boxes if you like.
        let valid = mins.x <= maxs.x && mins.y <= maxs.y && mins.z <= maxs.z;
        
        if !valid {
            // handle the invalid case, e.g. return empty
            return CSG::new();
        }
        
        // The "diagonal" is just (maxs - mins):
        let diag_vec = maxs - mins;
        let diag_len = diag_vec.norm();
        
        // some “big enough” dimension
        let diag = (diag_len * 2.0).max(1.0);
        let epsilon = 1e-5;
        
        // Let’s build a big square in XY from -diag..+diag, then extrude from z = -ε..+ε.
        // That “square” is effectively the top‐down bounding shape for the slab in XY.
        //
        // We'll just use CSG::square, then scale it up, and extrude ±ε.
        let big_xy = CSG::square(None)
            .scale(diag, diag, 1.0)
            .translate(Vector3::new(-diag / 2.0, -diag / 2.0, 0.0));
        
        // Now extrude it ± ε around z=0: easiest is extrude +ε, then shift -ε/2
        // or do something like:
        let slab = big_xy
            .extrude(2.0 * epsilon) // extrude from z=0 up to z=+2ε
            .translate(Vector3::new(0.0, 0.0, -epsilon)); // shift down so range is [-ε, +ε]
        
        // 4) Intersect the shape with that slab in plane‐coords
        let cross_section_3d = csg_in_plane_coords.intersect(&slab);

        // 5) Flatten that intersection down to z=0 using your existing `flatten()`,
        //    which merges everything in XY.  This uses your new 2D boolean ops under the hood.
        let section_2d = cross_section_3d.flatten();

        // 6) (Optional) transform the cross‐section polygons back to the *original* 3D coordinate system
        //    if you want them placed in the actual slicing plane in 3D.  That’s typically helpful
        //    if the user wants to see the cross‐section “in place.”
        //
        //    The inverse is T⁻¹ (the inverse of `transform_to_xy`).
        //    But note that your `flatten()` forced polygons to z=0, so re‐lifting them onto
        //    the real plane means you have to set their z‐coordinate to plane.w along the normal.
        //    In short, you may or may not want this step. 
        //
        //    If you do want them in the original 3D:
        //      let invT = transform_to_xy.try_inverse().unwrap();
        //      section_2d = section_2d.transform(&invT);

        section_2d
    }

    /// Convert a `MeshText` (from meshtext) into a list of `Polygon` in the XY plane.
    /// - `scale` allows you to resize the glyph (e.g. matching a desired font size).
    /// - By default, the glyph’s normal is set to +Z.
    fn meshtext_to_polygons(glyph_mesh: &meshtext::MeshText, scale: f64) -> Vec<Polygon<S>> {
        let mut polygons = Vec::new();
        let verts = &glyph_mesh.vertices;

        // Each set of 9 floats = one triangle: (x1,y1,z1, x2,y2,z2, x3,y3,z3)
        for tri_chunk in verts.chunks_exact(9) {
            let x1 = tri_chunk[0] as f64;
            let y1 = tri_chunk[1] as f64;
            let z1 = tri_chunk[2] as f64;
            let x2 = tri_chunk[3] as f64;
            let y2 = tri_chunk[4] as f64;
            let z2 = tri_chunk[5] as f64;
            let x3 = tri_chunk[6] as f64;
            let y3 = tri_chunk[7] as f64;
            let z3 = tri_chunk[8] as f64;

            // Scale them
            let px1 = x1 * scale;
            let py1 = y1 * scale;
            let pz1 = z1 * scale;
    
            let px2 = x2 * scale;
            let py2 = y2 * scale;
            let pz2 = z2 * scale;
    
            let px3 = x3 * scale;
            let py3 = y3 * scale;
            let pz3 = z3 * scale;
    
            // Normal = +Z
            let normal = nalgebra::Vector3::new(0.0, 0.0, 1.0);
    
            polygons.push(Polygon::new(
                vec![
                    Vertex::new(Point3::new(px1, py1, pz1), normal),
                    Vertex::new(Point3::new(px2, py2, pz2), normal),
                    Vertex::new(Point3::new(px3, py3, pz3), normal),
                ],
                None,
            ));
        }

        polygons
    }

    /// Creates 2D text in the XY plane using the `meshtext` crate to generate glyph meshes.
    ///
    /// - `text_str`: the text to render
    /// - `font_data`: TTF font file bytes (e.g. `include_bytes!("../assets/FiraMono-Regular.ttf")`)
    /// - `size`: optional scaling factor (e.g., a rough "font size"). 
    ///
    /// **Note**: Limitations:
    ///   - does not handle kerning or multi-line text,
    ///   - simply advances the cursor by each glyph’s width,
    ///   - places all characters along the X axis.
    pub fn text(text_str: &str, font_data: &[u8], size: Option<f64>) -> CSG<S> {
        let mut generator = MeshGenerator::new(font_data.to_vec());
        let scale = size.unwrap_or(20.0);

        let mut all_polygons = Vec::new();
        let mut cursor_x = 0.0f64;

        for ch in text_str.chars() {
            // Optionally skip control chars
            if ch.is_control() {
                continue;
            }
            // Generate glyph mesh
            let glyph_mesh: MeshText = match generator.generate_glyph(ch, true, None) {
                Ok(m) => m,
                Err(_) => {
                    // Missing glyph? Advance by some default
                    cursor_x += scale;
                    continue;
                }
            };
    
            // Convert to polygons
            let glyph_polygons = Self::meshtext_to_polygons(&glyph_mesh, scale);

            // Translate polygons by (cursor_x, 0.0)
            let glyph_csg = CSG::from_polygons(glyph_polygons)
                .translate(Vector3::new(cursor_x, 0.0, 0.0));
            // Accumulate
            all_polygons.extend(glyph_csg.polygons);

            // Advance cursor by the glyph’s bounding-box width
            let glyph_width = glyph_mesh.bbox.max.x - glyph_mesh.bbox.min.x;
            cursor_x += glyph_width as f64 * scale;
        }

        CSG::from_polygons(all_polygons)
    }
    
    /// Re‐triangulate each polygon in this CSG using the `earclip` library (earcut).
    /// Returns a new CSG whose polygons are all triangles.
    pub fn retriangulate(&self) -> CSG<S> {
        let mut new_polygons = Vec::new();

        // For each polygon in this CSG:
        for poly in &self.polygons {
            // Skip if fewer than 3 vertices or degenerate
            if poly.vertices.len() < 3 {
                continue;
            }
            // Optional: You may also want to check if the polygon is "nearly degenerate"
            // by measuring area or normal magnitude, etc. For brevity, we skip that here.

            // 1) Build an orthonormal basis from the polygon's plane normal
            let n = poly.plane.normal.normalize();
            let (u, v) = build_orthonormal_basis(n);

            // 2) Pick a reference point on the polygon. Typically the first vertex
            let p0 = poly.vertices[0].pos;

            // 3) Project each 3D vertex into 2D coordinates: (x_i, y_i)
            // We'll store them in a flat `Vec<f64>` of the form [x0, y0, x1, y1, x2, y2, ...]
            let mut coords_2d = Vec::with_capacity(poly.vertices.len() * 2);
            for vert in &poly.vertices {
                let offset = vert.pos.coords - p0.coords;  // vector from p0 to the vertex
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                coords_2d.push(x);
                coords_2d.push(y);
            }

            // 4) Call Earcut on that 2D outline. We assume no holes, so hole_indices = &[].
            //    earcut's signature is `earcut::<f64, usize>(data, hole_indices, dim)`
            //    with `dim = 2` for our XY data.
            let indices: Vec<usize> = earcut(&coords_2d, &[], 2);

            // 5) Map each returned triangle's (i0, i1, i2) back to 3D,
            //    constructing a new `Polygon` (with 3 vertices) for each tri.
            //
            //    The earcut indices are typed as `usize` adjust or cast as needed.
            for tri in indices.chunks_exact(3) {
                let mut tri_vertices = Vec::with_capacity(3);
                for &i in tri {
                    let x = coords_2d[2 * i];
                    let y = coords_2d[2 * i + 1];
                    // Inverse projection:
                    // Q_3D = p0 + x * u + y * v
                    let pos_vec = p0.coords + x * u + y * v;
                    let pos_3d  = Point3::from(pos_vec);
                    // We can store the normal = polygon's plane normal (or recalc).
                    // We'll recalc below, so for now just keep n or 0 as a placeholder:
                    tri_vertices.push(Vertex::new(pos_3d, n));
                }

                // Create a polygon from these 3 vertices. We preserve the metadata:
                let mut new_poly = Polygon::new(tri_vertices, poly.metadata.clone());

                // Recompute the plane/normal to ensure correct orientation/shading:
                new_poly.recalc_plane_and_normals();

                new_polygons.push(new_poly);
            }
        }

        // Combine all newly formed triangles into a new CSG:
        CSG::from_polygons(new_polygons)
    }

    /// Convert the polygons in this CSG to a Parry TriMesh.
    /// Useful for collision detection or physics simulations.
    pub fn to_trimesh(&self) -> SharedShape {
        // 1) Gather all the triangles from each polygon
        // 2) Build a TriMesh from points + triangle indices
        // 3) Wrap that in a SharedShape to be used in Rapier
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut index_offset = 0;

        for poly in &self.polygons {
            let tris = poly.triangulate();
            for tri in &tris {
                // Each tri is [Vertex; 3]
                //  push the positions into `vertices`
                //  build the index triplet for `indices`
                for v in tri {
                    vertices.push(Point3::new(v.pos.x, v.pos.y, v.pos.z));
                }
                indices.push([index_offset, index_offset+1, index_offset+2]);
                index_offset += 3;
            }
        }

        // TriMesh::new(Vec<[f64; 3]>, Vec<[u32; 3]>)
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(&self, density: f64) -> (f64, Point3<f64>, Unit<Quaternion<f64>>) {
        let shape = self.to_trimesh();
        if let Some(trimesh) = shape.as_trimesh() {
            let mp = trimesh.mass_properties(density);
            (
                mp.mass(),
                mp.local_com,                         // a Point3<f64>
                mp.principal_inertia_local_frame      // a Unit<Quaternion<f64>>
            )
        } else {
	    // fallback if not a TriMesh
            (0.0, Point3::origin(), Unit::<Quaternion<f64>>::identity())
        }
    }

    /// Create a Rapier rigid body + collider from this CSG, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<f64>,
        rotation: Vector3<f64>, // rotation axis scaled by angle (radians)
        density: f64,
    ) -> RigidBodyHandle {
        let shape = self.to_trimesh();
    
        // Build a Rapier RigidBody
        let rb = RigidBodyBuilder::dynamic()
            .translation(translation)
            // Now `rotation(...)` expects an axis-angle Vector3.
            .rotation(rotation)
            .build();
        let rb_handle = rb_set.insert(rb);
    
        // Build the collider
        let coll = ColliderBuilder::new(shape)
            .density(density)
            .build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);
    
        rb_handle
    }
    
    /// Checks if the CSG object is manifold.
    ///
    /// This function defines a comparison function which takes EPSILON into account
    /// for f64 coordinates, builds a hashmap key from the string representation of
    /// the coordinates, triangulates the CSG polygons, gathers each of their three edges,
    /// counts how many times each edge appears across all triangles,
    /// and returns true if every edge appears exactly 2 times, else false.
    ///
    /// We should also check that all faces have consistent orientation and no neighbors
    /// have flipped normals.
    ///
    /// We should also check for zero-area triangles
    ///
    /// # Returns
    ///
    /// - `true`: If the CSG object is manifold.
    /// - `false`: If the CSG object is not manifold.
    pub fn is_manifold(&self) -> bool {
        fn approx_lt(a: &Point3<f64>, b: &Point3<f64>) -> bool {
            // Compare x
            if (a.x - b.x).abs() > EPSILON {
                return a.x < b.x;
            }
            // If x is "close", compare y
            if (a.y - b.y).abs() > EPSILON {
                return a.y < b.y;
            }
            // If y is also close, compare z
            a.z < b.z
        }

        // Turn a 3D point into a string with limited decimal places
        fn point_key(p: &Point3<f64>) -> String {
            // Truncate/round to e.g. 6 decimals
            format!("{:.6},{:.6},{:.6}", p.x, p.y, p.z)
        }

        let mut edge_counts: HashMap<(String, String), u32> = HashMap::new();

        for poly in &self.polygons {
            // Triangulate each polygon
            for tri in poly.triangulate() {
                // Each tri is 3 vertices: [v0, v1, v2]
                // We'll look at edges (0->1, 1->2, 2->0).
                for &(i0, i1) in &[(0,1), (1,2), (2,0)] {
                    let p0 = tri[i0].pos;
                    let p1 = tri[i1].pos;
                    
                    // Order them so (p0, p1) and (p1, p0) become the same key
                    let (a_key, b_key) = if approx_lt(&p0, &p1) {
                        (point_key(&p0), point_key(&p1))
                    } else {
                        (point_key(&p1), point_key(&p0))
                    };

                    *edge_counts.entry((a_key, b_key)).or_insert(0) += 1;
                }
            }
        }

        // For a perfectly closed manifold surface (with no boundary),
        // each edge should appear exactly 2 times.
        edge_counts.values().all(|&count| count == 2)
    }

    /// Export to ASCII STL
    ///
    /// Convert this CSG to an **ASCII STL** string with the given `name`.
    ///
    /// ```
    /// let csg = CSG::cube(None);
    /// let stl_text = csg.to_stl("my_solid");
    /// println!("{}", stl_text);
    /// ```
    pub fn to_stl_ascii(&self, name: &str) -> String {
        let mut out = String::new();
        out.push_str(&format!("solid {}\n", name));

        for poly in &self.polygons {
            // Use the polygon plane's normal for the facet normal (normalized).
            let normal = poly.plane.normal.normalize();
            let triangles = poly.triangulate();

            for tri in triangles {
                out.push_str(&format!(
                    "  facet normal {:.6} {:.6} {:.6}\n",
                    normal.x, normal.y, normal.z
                ));
                out.push_str("    outer loop\n");
                for vertex in &tri {
                    out.push_str(&format!(
                        "      vertex {:.6} {:.6} {:.6}\n",
                        vertex.pos.x, vertex.pos.y, vertex.pos.z
                    ));
                }
                out.push_str("    endloop\n");
                out.push_str("  endfacet\n");
            }
        }

        out.push_str(&format!("endsolid {}\n", name));
        out
    }

    /// Export to BINARY STL (returns Vec<u8>)
    ///
    /// Convert this CSG to a **binary STL** byte vector with the given `name`.
    ///
    /// The resulting `Vec<u8>` can then be written to a file or handled in memory:
    ///
    /// ```
    /// let bytes = csg.to_stl_binary("my_solid")?;
    /// std::fs::write("my_solid.stl", bytes)?;
    /// ```
    pub fn to_stl_binary(&self, _name: &str) -> std::io::Result<Vec<u8>> {
        // `_name` could be embedded in the binary header if desired, but `stl_io` 
        // doesn't strictly require it. We skip storing it or store it in the 80-byte header.

        // Gather the triangles for stl_io
        let mut triangles = Vec::new();
        for poly in &self.polygons {
            let normal = poly.plane.normal.normalize();
            let tri_list = poly.triangulate();

            for tri in tri_list {
                triangles.push(stl_io::Triangle {
                    normal: stl_io::Normal::new([normal.x as f32, normal.y as f32, normal.z as f32]),
                    vertices: [
                        stl_io::Vertex::new([tri[0].pos.x as f32, tri[0].pos.y as f32, tri[0].pos.z as f32, ]),
                        stl_io::Vertex::new([tri[1].pos.x as f32, tri[1].pos.y as f32, tri[1].pos.z as f32, ]),
                        stl_io::Vertex::new([tri[2].pos.x as f32, tri[2].pos.y as f32, tri[2].pos.z as f32, ]),
                    ],
                });
            }
        }

        // Write to an in-memory buffer
        let mut cursor = Cursor::new(Vec::new());
        stl_io::write_stl(&mut cursor, triangles.iter())?;

        Ok(cursor.into_inner())
    }

    /// Create a CSG object from STL data using `stl_io`.
    pub fn from_stl(stl_data: &[u8]) -> Result<CSG<S>, std::io::Error> {
        // Create an in-memory cursor from the STL data
        let mut cursor = Cursor::new(stl_data);
    
        // Create an STL reader from the cursor
        let stl_reader = stl_io::create_stl_reader(&mut cursor)?;

        let mut polygons = Vec::new();

        for tri_result in stl_reader {
            // Handle potential errors from the STL reader
            let tri = match tri_result {
                Ok(t) => t,
                Err(e) => return Err(e), // Propagate the error
            };

            // Construct vertices and a polygon
            let vertices = vec![
                Vertex::new(
                    Point3::new(tri.vertices[0][0] as f64, tri.vertices[0][1] as f64, tri.vertices[0][2] as f64),
                    Vector3::new(tri.normal[0] as f64, tri.normal[1] as f64, tri.normal[2] as f64),
                ),
                Vertex::new(
                    Point3::new(tri.vertices[1][0] as f64, tri.vertices[1][1] as f64, tri.vertices[1][2] as f64),
                    Vector3::new(tri.normal[0] as f64, tri.normal[1] as f64, tri.normal[2] as f64),
                ),
                Vertex::new(
                    Point3::new(tri.vertices[2][0] as f64, tri.vertices[2][1] as f64, tri.vertices[2][2] as f64),
                    Vector3::new(tri.normal[0] as f64, tri.normal[1] as f64, tri.normal[2] as f64),
                ),
            ];
            polygons.push(Polygon::new(vertices, None));
        }

        Ok(CSG::from_polygons(polygons))
    }
    
    /// Import a CSG object from DXF data.
    ///
    /// # Parameters
    ///
    /// - `dxf_data`: A byte slice containing the DXF file data.
    ///
    /// # Returns
    ///
    /// A `Result` containing the CSG object or an error if parsing fails.
    pub fn from_dxf(dxf_data: &[u8]) -> Result<CSG<S>, Box<dyn Error>> {
        // Load the DXF drawing from the provided data
        let drawing = Drawing::load(&mut Cursor::new(dxf_data))?;

        let mut polygons = Vec::new();

        for entity in drawing.entities() {
            match &entity.specific {
                EntityType::Line(_line) => {
                    // Convert a line to a thin rectangular polygon (optional)
                    // Alternatively, skip lines if they don't form closed loops
                    // Here, we'll skip standalone lines
                    // To form polygons from lines, you'd need to group connected lines into loops
                }
                EntityType::Polyline(polyline) => {
                    // Handle POLYLINE entities (which can be 2D or 3D)
                    if polyline.is_closed() {
                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            verts.push(Vertex::new(
                                Point3::new(vertex.location.x, vertex.location.y, vertex.location.z),
                                Vector3::new(0.0, 0.0, 1.0), // Assuming flat in XY
                            ));
                        }
                        // Create a polygon from the polyline vertices
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, None));
                        }
                    }
                }
                EntityType::Circle(circle) => {
                    // Approximate circles with regular polygons
                    let center = Point3::new(circle.center.x, circle.center.y, circle.center.z);
                    let radius = circle.radius;
                    let segments = 32; // Number of segments to approximate the circle

                    let mut verts = Vec::new();
                    let normal = Vector3::new(0.0, 0.0, 1.0); // Assuming circle lies in XY plane

                    for i in 0..segments {
                        let theta = 2.0 * PI * (i as f64) / (segments as f64);
                        let x = center.x + radius * theta.cos();
                        let y = center.y + radius * theta.sin();
                        let z = center.z;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }

                    // Create a polygon from the approximated circle vertices
                    polygons.push(Polygon::new(verts, None));
                }
                // Handle other entity types as needed (e.g., Arc, Spline)
                _ => {
                    // Ignore unsupported entity types for now
                }
            }
        }

        Ok(CSG::from_polygons(polygons))
    }

    /// Export the CSG object to DXF format.
    ///
    /// # Returns
    ///
    /// A `Result` containing the DXF file as a byte vector or an error if exporting fails.
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut drawing = Drawing::new();

        for poly in &self.polygons {
            // Triangulate the polygon if it has more than 3 vertices
            let triangles = if poly.vertices.len() > 3 {
                poly.triangulate()
            } else {
                vec![[
                    poly.vertices[0].clone(),
                    poly.vertices[1].clone(),
                    poly.vertices[2].clone(),
                ]]
            };

            for tri in triangles {
                // Create a 3DFACE entity for each triangle
                let face = dxf::entities::Face3D::new(
                    // 3DFACE expects four vertices, but for triangles, the fourth is the same as the third
                    dxf::Point::new(tri[0].pos.x, tri[0].pos.y, tri[0].pos.z),
                    dxf::Point::new(tri[1].pos.x, tri[1].pos.y, tri[1].pos.z),
                    dxf::Point::new(tri[2].pos.x, tri[2].pos.y, tri[2].pos.z),
                    dxf::Point::new(tri[2].pos.x, tri[2].pos.y, tri[2].pos.z), // Duplicate for triangular face
                );
                
                let entity = dxf::entities::Entity::new(dxf::entities::EntityType::Face3D(face));

                // Add the 3DFACE entity to the drawing
                drawing.add_entity(entity);
            }
        }

        // Serialize the DXF drawing to bytes
        let mut buffer = Vec::new();
        drawing.save(&mut buffer)?;

        Ok(buffer)
    }
}
