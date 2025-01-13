#![allow(dead_code)]

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use nalgebra::{Matrix4, Vector3, Point3, Translation3, Rotation3, Isometry3};
use chull::ConvexHullWrapper;
use parry3d_f64::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::Triangle,
    };

#[cfg(test)]
mod tests;

const EPSILON: f64 = 1e-5;

pub enum Axis {
    X,
    Y,
    Z,
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
    pub fn split_polygon(
        &self,
        polygon: &Polygon,
        coplanar_front: &mut Vec<Polygon>,
        coplanar_back: &mut Vec<Polygon>,
        front: &mut Vec<Polygon>,
        back: &mut Vec<Polygon>,
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
                    front.push(Polygon::new(f, polygon.shared.clone()));
                }
                if b.len() >= 3 {
                    back.push(Polygon::new(b, polygon.shared.clone()));
                }
            }
        }
    }
}

/// A convex polygon, defined by a list of vertices and a plane
#[derive(Debug, Clone)]
pub struct Polygon {
    pub vertices: Vec<Vertex>,
    /// This can hold any “shared” data (color, surface ID, etc.).
    pub shared: Option<String>,
    pub plane: Plane,
}

impl Polygon {
    /// Create a polygon from vertices
    pub fn new(vertices: Vec<Vertex>, shared: Option<String>) -> Self {
        let plane = Plane::from_points(
            &vertices[0].pos,
            &vertices[1].pos,
            &vertices[2].pos,
        );
        Polygon { vertices, shared, plane }
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
}

/// A BSP tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node {
    pub plane: Option<Plane>,
    pub front: Option<Box<Node>>,
    pub back: Option<Box<Node>>,
    pub polygons: Vec<Polygon>,
}

impl Node {
    pub fn new(polygons: Vec<Polygon>) -> Self {
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
    pub fn clip_polygons(&self, polygons: &[Polygon]) -> Vec<Polygon> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();
        let mut front: Vec<Polygon> = Vec::new();
        let mut back: Vec<Polygon> = Vec::new();

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
    pub fn clip_to(&mut self, bsp: &Node) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

    /// Return all polygons in this BSP tree
    pub fn all_polygons(&self) -> Vec<Polygon> {
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
    pub fn build(&mut self, polygons: &[Polygon]) {
        if polygons.is_empty() {
            return;
        }

        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();

        let mut front: Vec<Polygon> = Vec::new();
        let mut back: Vec<Polygon> = Vec::new();

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
pub struct CSG {
    pub polygons: Vec<Polygon>,
}

impl CSG {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG { polygons: Vec::new() }
    }

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: Vec<Polygon>) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons;
        csg
    }

    /// Return the internal polygons
    pub fn to_polygons(&self) -> &[Polygon] {
        &self.polygons
    }

    /// CSG union: this ∪ other
    pub fn union(&self, other: &CSG) -> CSG {
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
    pub fn subtract(&self, other: &CSG) -> CSG {
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
    pub fn intersect(&self, other: &CSG) -> CSG {
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
    pub fn inverse(&self) -> CSG {
        let mut csg = self.clone();
        for p in &mut csg.polygons {
            p.flip();
        }
        csg
    }

    /// Construct an axis-aligned cube, with optional center and radius
    pub fn cube(options: Option<(&[f64; 3], &[f64; 3])>) -> CSG {
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
    pub fn sphere(options: Option<(&[f64; 3], f64, usize, usize)>) -> CSG {
        let (center, radius, slices, stacks) = match options {
            Some((c, r, sl, st)) => (*c, r, sl, st),
            None => ([0.0, 0.0, 0.0], 1.0, 16, 8),
        };

        let c = Vector3::new(center[0], center[1], center[2]);
        let mut polygons = Vec::new();

        // 2π is `std::f64::consts::TAU` in newer Rust,
        // but if that's not available, replace with `2.0 * std::f64::consts::PI`.
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

    /// Construct a cylinder with optional start, end, radius, slices
    pub fn cylinder(options: Option<(&[f64; 3], &[f64; 3], f64, usize)>) -> CSG {
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

    /// Transform all vertices in this CSG by a given 4×4 matrix.
    pub fn transform(&self, mat: &Matrix4<f64>) -> CSG {
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

    
    pub fn translate(&self, v: Vector3<f64>) -> CSG {
        let translation = Translation3::from(v);
        // Convert to a Matrix4
        let mat4 = translation.to_homogeneous();
        self.transform(&mat4)
    }

    pub fn rotate(&self, x_deg: f64, y_deg: f64, z_deg: f64) -> CSG {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());
        
        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> CSG {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    /// Mirror across X=0, Y=0, or Z=0 plane
    pub fn mirror(&self, axis: Axis) -> CSG {
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
    pub fn convex_hull(&self) -> CSG {
        // Gather all (x, y, z) coordinates from the polygons
        let points: Vec<Vec<f64>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.vertices.iter().map(|v| {
                    vec![v.pos.x, v.pos.y, v.pos.z]
                })
            })
            .collect();

        // Compute convex hull using the robust wrapper
        let hull = ConvexHullWrapper::try_new(&points, None)
            .expect("Failed to compute convex hull");

        let (verts, indices) = hull.vertices_indices();

        // Reconstruct polygons as triangles
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
    pub fn minkowski_sum(&self, other: &CSG) -> CSG {
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

        // For Minkowski, add every point in A to every point in B
        let mut sum_points = Vec::with_capacity(verts_a.len() * verts_b.len());
        for a in &verts_a {
            for b in &verts_b {
                sum_points.push(vec![a.x + b.x, a.y + b.y, a.z + b.z]);
            }
        }

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
        origin: &nalgebra::Point3<f64>,
        direction: &nalgebra::Vector3<f64>,
    ) -> Vec<(nalgebra::Point3<f64>, f64)> {
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
                        nalgebra::Point3::from(point_on_ray.coords),
                        hit.time_of_impact,
                    ));
                }
            }
        }

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        hits
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
    pub fn square(params: Option<([f64; 2], bool)>) -> CSG {
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
    pub fn circle(params: Option<(f64, usize)>) -> CSG {
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
    /// **Note:** This simple version ignores 'paths' and holes. For more complex
    /// polygons, we'll have to handle multiple paths, winding order, holes, etc.
    ///
    /// # Example
    /// let pts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]];
    /// let poly2d = CSG::polygon_2d(&pts);
    pub fn polygon_2d(points: &[[f64; 2]]) -> CSG {
        assert!(points.len() >= 3, "polygon_2d requires at least 3 points");

        let normal = Vector3::new(0.0, 0.0, 1.0);
        let mut verts = Vec::with_capacity(points.len());
        for p in points {
            verts.push(Vertex::new(Point3::new(p[0], p[1], 0.0), normal));
        }
        CSG::from_polygons(vec![Polygon::new(verts, None)])
    }

    /// Creates 2D text in the XY plane, returning one or more polygons.
    ///
    /// **Note**: In OpenSCAD, `text(...)` uses a font engine to generate outline polygons
    /// of each glyph. Implementing this fully in Rust would require a font library (e.g. `fontdue`,
    /// `rusttype`, or similar). Below is a *stub* that returns an empty shape or a placeholder.
    ///
    /// # Parameters
    ///
    /// - `text_str`: the text to render
    /// - `size`: approximate font size
    ///
    /// # Example
    /// let txt = CSG::text("Hello", None);
    /// // This stub returns an empty shape or placeholder polygons.
    pub fn text(text_str: &str, size: Option<f64>) -> CSG {
        // --- STUB implementation ---
        // In a real implementation, you’d use a font library
        // to generate outlines, then convert to polygons.

        let _font_size = size.unwrap_or(10.0);
        if text_str.is_empty() {
            return CSG::new();
        }

        // For illustration, we'll just return a small 'placeholder' square
        // that indicates "text bounding box", ignoring the actual glyph outlines:
        let w = text_str.len() as f64 * 0.6 * _font_size; // approximate width
        let h = _font_size;
        let placeholder = CSG::square(Some(([w, h], false)));
        
        // Translate so it's visible at (0,0):
        placeholder.translate(Vector3::new(0.0, 0.0, 0.0))
    }
    
    /// Linearly extrude this (2D) shape in the +Z direction by `height`.
    ///
    /// This is similar to OpenSCAD's `linear_extrude(height=...)` assuming
    /// the base 2D shape is in the XY plane with a +Z normal.
    ///
    /// - If your shape is centered around Z=0, the resulting extrusion
    ///   will go from Z=0 to Z=`height`.
    /// - The top polygons will be created at Z=`height`.
    /// - The bottom polygons remain at Z=0 (the original).
    /// - Side polygons will be formed around the perimeter.
    pub fn extrude(&self, height: f64) -> CSG {
        // Collect all new polygons here
        let mut new_polygons = Vec::new();

        // 1) Bottom polygons = original polygons
        //    (assuming they are already in the XY plane with some Z).
        //    We keep them as-is. If you want them “closed”, make sure the polygon
        //    normal is pointing down or up consistently.
        for poly in &self.polygons {
            new_polygons.push(poly.clone());
        }

        // 2) Top polygons = translate each original polygon by +Z=height,
        //    then *flip* if you want the normals to face outward (typically up).
        //    The simplest approach is to clone, then shift all vertices by (0,0,height).
        //    We can do that using the `translate(...)` method, but let's do it manually here:
        let top_polygons = self.translate(Vector3::new(0.0, 0.0, height)).polygons;
        // Typically for a "closed" shape, you'd want the top polygon normals
        // facing upward. If your original polygons had +Z normals, after
        // translation, they'd still have +Z normals, so you might not need flip.
        // But if you want them reversed, uncomment:
        // for p in &mut top_polygons {
        //     p.flip();
        // }

        // Add those top polygons to the list
        new_polygons.extend(top_polygons.iter().cloned());

        // 3) Side polygons = For each polygon in `self`, connect its edges
        //    from the original to the translated version.
        //
        //    We'll iterate over each polygon’s vertices. For each edge
        //    (v[i], v[i+1]), we form a rectangular side quad with the corresponding
        //    points in the top polygon.  That is:
        //
        //    bottom edge = (v[i], v[i+1])
        //    top edge    = (v[i]+(0,0,h), v[i+1]+(0,0,h))
        //
        //    We'll build those as two triangles or one quad polygon.
        //
        //    We can find the corresponding top vertex by the same index in
        //    the top_polygons list.  Because we simply did a single `translate(...)`
        //    for the entire shape, we need to match polygons by index.

        // We'll do this polygon-by-polygon. For each polygon, we retrieve
        // its "partner" in top_polygons by the same index. This assumes the order
        // of polygons hasn't changed between self.polygons and top_polygons.
        //
        // => If your shape has many polygons, be sure they line up. 
        //    If your shape is a single polygon, it's simpler. 
        //    If your shape is multiple polygons, be mindful.

        // We already have them in arrays: 
        let bottom_polys = &self.polygons;
        let top_polys = &top_polygons;

        for (poly_bottom, poly_top) in bottom_polys.iter().zip(top_polys.iter()) {
            let vcount = poly_bottom.vertices.len();
            if vcount < 3 {
                continue; // skip degenerate
            }

            for i in 0..vcount {
                let j = (i + 1) % vcount; // next index

                // Bottom edge: b_i -> b_j
                let b_i = &poly_bottom.vertices[i];
                let b_j = &poly_bottom.vertices[j];

                // Top edge: t_i -> t_j
                let t_i = &poly_top.vertices[i];
                let t_j = &poly_top.vertices[j];

                // We'll form a quad [b_i, b_j, t_j, t_i]
                // with outward-facing normals. 
                let side_poly = Polygon::new(
                    vec![
                        b_i.clone(),
                        b_j.clone(),
                        t_j.clone(),
                        t_i.clone(),
                    ],
                    None,
                );
                new_polygons.push(side_poly);
            }
        }

        // Combine into a new CSG
        CSG::from_polygons(new_polygons)
    }
    
    /// Rotate-extrude (revolve) this 2D shape around the Z-axis from 0..`angle_degs`.
    /// - `segments` determines how many steps to sample around the axis.
    /// - For a full revolve, pass `angle_degs = 360.0`.
    /// 
    /// This is similar to OpenSCAD's `rotate_extrude(angle=..., segments=...)`.
    pub fn rotate_extrude(&self, angle_degs: f64, segments: usize) -> CSG {
        let angle_radians = angle_degs.to_radians();
        if segments < 2 {
            panic!("rotate_extrude requires at least 2 segments");
        }

        // For each polygon in the original 2D shape, we'll build a "swept" surface.
        // We'll store the new polygons in here:
        let mut new_polygons = Vec::new();

        // Precompute rotation transforms for each step.
        // Step i in [0..segments]:
        //   θ_i = i * (angle_radians / (segments as f64))
        // We'll store these as 4×4 matrices for convenience.
        let mut transforms = Vec::with_capacity(segments);
        for i in 0..segments {
            let frac = i as f64 / segments as f64;
            let theta = frac * angle_radians;
            let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), theta);
            transforms.push(rot.to_homogeneous());
        }

        // Also consider the "closing" transform: if angle < 360, we do not close fully,
        // but if angle == 360, the last transform is the same as the 0th. 
        // We'll handle that logic by just iterating from i..(i+1).
        // We'll get the index of the next slice: (i+1)%segments if angle=360,
        // or i+1 if partial revolve. For partial revolve, the last segment = segments-1
        // won't connect forward if angle<360. 
        //
        // We'll define a small helper to get the next index safely:
        let closed = (angle_degs - 360.0).abs() < EPSILON; // if angle=360, we close
        let next_index = |i: usize| -> Option<usize> {
            if i == segments - 1 {
                if closed { Some(0) } else { None }
            } else {
                Some(i + 1)
            }
        };

        // For each polygon in our 2D shape...
        for poly in &self.polygons {
            let vcount = poly.vertices.len();
            if vcount < 3 {
                continue;
            }

            // Build a slice of transformed polygons
            // for i in 0..segments, polygon_i = transform(poly, transforms[i])
            let mut slices = Vec::with_capacity(segments);
            for i in 0..segments {
                let slice_csg = CSG::from_polygons(vec![poly.clone()]).transform(&transforms[i]);
                // We only have 1 polygon in slice_csg, so just grab it
                slices.push(slice_csg.polygons[0].clone());
            }

            // Now connect slice i -> slice i+1 for each edge in the original polygon
            // The bottom edge is from slices[i].vertices[edge], 
            // the top edge is from slices[i+1].vertices[edge] (in matching order).
            for i in 0..segments {
                let n_i = next_index(i);
                if n_i.is_none() {
                    continue; // partial revolve, we skip the last connection
                }
                let i2 = n_i.unwrap();

                // The two polygons we want to connect are slices[i] and slices[i2].
                let poly1 = &slices[i];
                let poly2 = &slices[i2];

                // Connect edges
                for e in 0..vcount {
                    let e_next = (e + 1) % vcount;

                    let b1 = &poly1.vertices[e];
                    let b2 = &poly1.vertices[e_next];
                    let t1 = &poly2.vertices[e];
                    let t2 = &poly2.vertices[e_next];

                    // We'll form a quad [b1, b2, t2, t1]
                    let side_poly = Polygon::new(
                        vec![b1.clone(), b2.clone(), t2.clone(), t1.clone()],
                        None,
                    );
                    new_polygons.push(side_poly);
                }
            }

            // Optionally, if you want the "ends" (like if angle<360), you could
            // add the very first slice polygon and the last slice polygon
            // as “caps” — but that’s more advanced logic:
            // - 0° cap = slices[0] (or a reversed copy)
            // - angle° cap = slices[segments-1] ...
            //
            // For a full revolve (360), these ends coincide and there's no extra capping needed.
        }

        // Combine everything into a new shape
        CSG::from_polygons(new_polygons)
    }

    /// Returns a `parry3d::bounding_volume::AABB`.
    pub fn bounding_box(&self) -> Aabb {
        // Gather all points from all polygons.
        // parry expects a slice of `&Point3<f64>` or a slice of `na::Point3<f64>`.
        let mut all_points = Vec::new();
        for poly in &self.polygons {
            for v in &poly.vertices {
                all_points.push(v.pos); // already an nalgebra Point3<f64>
            }
        }

        // If empty, return a degenerate AABB at origin or handle accordingly
        if all_points.is_empty() {
            return Aabb::new_invalid(); // or AABB::new(Point3::origin(), Point3::origin());
        }

        // Construct the parry AABB from points
        Aabb::from_points(&all_points)
    }
    
    // ----------------------------------------------------------
    //   Export to ASCII STL
    // ----------------------------------------------------------

    /// Convert this CSG to an **ASCII STL** string with the given `name`.
    ///
    /// ```
    /// let csg = CSG::cube(None);
    /// let stl_text = csg.to_stl("my_solid");
    /// println!("{}", stl_text);
    /// ```
    pub fn to_stl(&self, name: &str) -> String {
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
}
