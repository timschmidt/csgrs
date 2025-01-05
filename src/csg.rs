//! csg.rs
//!
//! A Rust translation of Evan Wallace's JavaScript CSG code. This uses a BSP tree
//! for constructive solid geometry (union, subtract, intersect).
//!
//! # Example
//!
//! ```rust
//! let cube = CSG::cube(None);
//! let sphere = CSG::sphere(None);
//! let result = cube.subtract(&sphere);
//! let polygons = result.to_polygons();
//! println!("{}", result.to_stl("my_solid"));
//! ```

/// Epsilon for floating-point comparisons
const EPSILON: f64 = 1e-5;

/// A 3D vector
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vector { x, y, z }
    }

    pub fn from_slice(slice: &[f64]) -> Self {
        Vector {
            x: slice[0],
            y: slice[1],
            z: slice[2],
        }
    }

    pub fn clone(&self) -> Self {
        *self
    }

    pub fn negated(&self) -> Self {
        Vector::new(-self.x, -self.y, -self.z)
    }

    pub fn plus(&self, a: &Vector) -> Self {
        Vector::new(self.x + a.x, self.y + a.y, self.z + a.z)
    }

    pub fn minus(&self, a: &Vector) -> Self {
        Vector::new(self.x - a.x, self.y - a.y, self.z - a.z)
    }

    pub fn times(&self, a: f64) -> Self {
        Vector::new(self.x * a, self.y * a, self.z * a)
    }

    pub fn divided_by(&self, a: f64) -> Self {
        Vector::new(self.x / a, self.y / a, self.z / a)
    }

    pub fn dot(&self, a: &Vector) -> f64 {
        self.x * a.x + self.y * a.y + self.z * a.z
    }

    pub fn lerp(&self, a: &Vector, t: f64) -> Self {
        self.plus(&a.minus(self).times(t))
    }

    pub fn length(&self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn unit(&self) -> Self {
        let len = self.length();
        if len < EPSILON {
            *self
        } else {
            self.divided_by(len)
        }
    }

    pub fn cross(&self, a: &Vector) -> Self {
        Vector::new(
            self.y * a.z - self.z * a.y,
            self.z * a.x - self.x * a.z,
            self.x * a.y - self.y * a.x,
        )
    }
}

/// A vertex of a polygon, holding position and normal.
/// Additional data (like UVs, colors, etc.) can be added if needed.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub pos: Vector,
    pub normal: Vector,
}

impl Vertex {
    pub fn new(pos: Vector, normal: Vector) -> Self {
        Vertex { pos, normal }
    }

    pub fn clone(&self) -> Self {
        Vertex {
            pos: self.pos.clone(),
            normal: self.normal.clone(),
        }
    }

    /// Flip orientation-specific data (like normals)
    pub fn flip(&mut self) {
        self.normal = self.normal.negated();
    }

    /// Linearly interpolate between self and `other` by parameter `t`
    pub fn interpolate(&self, other: &Vertex, t: f64) -> Vertex {
        Vertex {
            pos: self.pos.lerp(&other.pos, t),
            normal: self.normal.lerp(&other.normal, t),
        }
    }
}

/// A plane in 3D space defined by a normal and a w-value
#[derive(Debug, Clone)]
pub struct Plane {
    pub normal: Vector,
    pub w: f64,
}

impl Plane {
    /// Create a plane from three points
    pub fn from_points(a: &Vector, b: &Vector, c: &Vector) -> Plane {
        let n = b.minus(a).cross(&c.minus(a)).unit();
        Plane {
            normal: n,
            w: n.dot(a),
        }
    }

    pub fn clone(&self) -> Self {
        Plane {
            normal: self.normal.clone(),
            w: self.w,
        }
    }

    pub fn flip(&mut self) {
        self.normal = self.normal.negated();
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
        // Classification constants
        const COPLANAR: i32 = 0;
        const FRONT: i32 = 1;
        const BACK: i32 = 2;
        const SPANNING: i32 = 3;

        let mut polygon_type = 0;
        let mut types = Vec::with_capacity(polygon.vertices.len());

        // Classify each vertex
        for v in &polygon.vertices {
            let t = self.normal.dot(&v.pos) - self.w;
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
                        let t = (self.w - self.normal.dot(&vi.pos))
                            / self.normal.dot(&vj.pos.minus(&vi.pos));
                        let v = vi.interpolate(vj, t);
                        f.push(v.clone());
                        b.push(v);
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
        Polygon {
            vertices,
            shared,
            plane,
        }
    }

    pub fn clone(&self) -> Self {
        Polygon {
            vertices: self.vertices.iter().map(|v| v.clone()).collect(),
            shared: self.shared.clone(),
            plane: self.plane.clone(),
        }
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

    pub fn clone(&self) -> Self {
        Node {
            plane: self.plane.clone(),
            front: self.front.clone(),
            back: self.back.clone(),
            polygons: self.polygons.iter().map(|p| p.clone()).collect(),
        }
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

        // We'll accumulate front/back polygons separately
        let mut front: Vec<Polygon> = Vec::new();
        let mut back: Vec<Polygon> = Vec::new();

        for p in polygons {
            // Temporary vectors for coplanar polygons
            let mut coplanar_front = Vec::new();
            let mut coplanar_back = Vec::new();

            plane.split_polygon(
                p,
                &mut coplanar_front, // instead of &mut self.polygons
                &mut coplanar_back,  // instead of &mut self.polygons
                &mut front,
                &mut back,
            );

            // Now we can safely merge those coplanar polygons into self.polygons
            self.polygons.append(&mut coplanar_front);
            self.polygons.append(&mut coplanar_back);
        }

        if !front.is_empty() {
            // Build the front subtree
            if self.front.is_none() {
                self.front = Some(Box::new(Node::new(vec![])));
            }
            self.front.as_mut().unwrap().build(&front);
        }

        if !back.is_empty() {
            // Build the back subtree
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
        CSG {
            polygons: Vec::new(),
        }
    }

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: Vec<Polygon>) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons;
        csg
    }

    /// Clone this CSG
    pub fn clone(&self) -> Self {
        CSG {
            polygons: self.polygons.iter().map(|p| p.clone()).collect(),
        }
    }

    /// Return the internal polygons
    pub fn to_polygons(&self) -> &[Polygon] {
        &self.polygons
    }

    /// CSG union: this ∪ other
    pub fn union(&self, other: &CSG) -> CSG {
        let mut a = Node::new(self.clone().polygons);
        let mut b = Node::new(other.clone().polygons);

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
        let mut a = Node::new(self.clone().polygons);
        let mut b = Node::new(other.clone().polygons);

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
        let mut a = Node::new(self.clone().polygons);
        let mut b = Node::new(other.clone().polygons);

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

    /// Construct an axis-aligned cube, optional center and radius
    ///
    /// # Example
    ///
    /// ```
    /// let cube = CSG::cube(None);
    /// ```
    pub fn cube(options: Option<(&[f64; 3], &[f64; 3])>) -> CSG {
        // Default center = [0, 0, 0], radius = [1, 1, 1]
        let (center, radius) = match options {
            Some((c, r)) => (*c, *r),
            None => ([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
        };

        // Build the 6 faces
        let indices_and_normals = vec![
            (vec![0, 4, 6, 2], Vector::new(-1.0, 0.0, 0.0)),
            (vec![1, 3, 7, 5], Vector::new(1.0, 0.0, 0.0)),
            (vec![0, 1, 5, 4], Vector::new(0.0, -1.0, 0.0)),
            (vec![2, 6, 7, 3], Vector::new(0.0, 1.0, 0.0)),
            (vec![0, 2, 3, 1], Vector::new(0.0, 0.0, -1.0)),
            (vec![4, 5, 7, 6], Vector::new(0.0, 0.0, 1.0)),
        ];

        let c = Vector::from_slice(&center);
        let r = Vector::from_slice(&radius);

        let mut polygons = Vec::new();
        for (idxs, n) in indices_and_normals {
            let mut verts = Vec::new();
            for i in idxs {
                // The bits of `i` pick +/- for x,y,z
                let vx = c.x + r.x * ((i & 1) as f64 * 2.0 - 1.0);
                let vy = c.y + r.y * (((i & 2) >> 1) as f64 * 2.0 - 1.0);
                let vz = c.z + r.z * (((i & 4) >> 2) as f64 * 2.0 - 1.0);
                verts.push(Vertex::new(Vector::new(vx, vy, vz), n));
            }
            polygons.push(Polygon::new(verts, None));
        }

        CSG::from_polygons(polygons)
    }

    /// Construct a sphere with optional center, radius, slices, stacks
    ///
    /// # Example
    ///
    /// ```
    /// let sphere = CSG::sphere(None);
    /// ```
    pub fn sphere(options: Option<(&[f64; 3], f64, usize, usize)>) -> CSG {
        let (center, radius, slices, stacks) = match options {
            Some((c, r, sl, st)) => (*c, r, sl, st),
            None => ([0.0, 0.0, 0.0], 1.0, 16, 8),
        };

        let c = Vector::from_slice(&center);
        let mut polygons = Vec::new();

        // generate polygons
        for i in 0..slices {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: f64, phi: f64| {
                    let dir = Vector::new(
                        theta.cos() * phi.sin(),
                        phi.cos(),
                        theta.sin() * phi.sin(),
                    );
                    Vertex::new(c.plus(&dir.times(radius)), dir)
                };

                let t0 = i as f64 / slices as f64;
                let t1 = (i + 1) as f64 / slices as f64;
                let p0 = j as f64 / stacks as f64;
                let p1 = (j + 1) as f64 / stacks as f64;

                // angles
                let theta0 = t0 * std::f64::consts::TAU; // 2π
                let theta1 = t1 * std::f64::consts::TAU;
                let phi0 = p0 * std::f64::consts::PI;
                let phi1 = p1 * std::f64::consts::PI;

                // build up to 4 vertices
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
    ///
    /// # Example
    ///
    /// ```
    /// let cylinder = CSG::cylinder(None);
    /// ```
    pub fn cylinder(options: Option<(&[f64; 3], &[f64; 3], f64, usize)>) -> CSG {
        let (start, end, radius, slices) = match options {
            Some((s, e, r, sl)) => (*s, *e, r, sl),
            None => ([0.0, -1.0, 0.0], [0.0, 1.0, 0.0], 1.0, 16),
        };

        let s = Vector::from_slice(&start);
        let e = Vector::from_slice(&end);
        let ray = e.minus(&s);
        let axis_z = ray.unit();
        let is_y = axis_z.y.abs() > 0.5;
        let axis_x = Vector::new(is_y as i32 as f64, (!is_y) as i32 as f64, 0.0)
            .cross(&axis_z)
            .unit();
        let axis_y = axis_x.cross(&axis_z).unit();

        let start_v = Vertex::new(s, axis_z.negated());
        let end_v = Vertex::new(e, axis_z);

        let mut polygons = Vec::new();

        let point = |stack: f64, slice: f64, normal_blend: f64| {
            let angle = slice * std::f64::consts::TAU;
            let out = axis_x.times(angle.cos()).plus(&axis_y.times(angle.sin()));
            let pos = s.plus(&ray.times(stack)).plus(&out.times(radius));
            let normal = out.times(1.0 - normal_blend.abs()).plus(&axis_z.times(normal_blend));
            Vertex::new(pos, normal)
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

    // ----------------------------------------------------------
    //   NEW: Export to ASCII STL
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
        // STL header
        out.push_str(&format!("solid {}\n", name));

        // For each polygon, triangulate and then output each triangle
        for poly in &self.polygons {
            let normal = poly.plane.normal.unit();
            let triangles = poly.triangulate();

            for tri in triangles {
                out.push_str(&format!("  facet normal {:.6} {:.6} {:.6}\n", normal.x, normal.y, normal.z));
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

        // STL footer
        out.push_str(&format!("endsolid {}\n", name));
        out
    }
}

