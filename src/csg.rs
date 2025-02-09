use crate::float_types::{EPSILON, PI, TAU, CLOSED, Real};
use crate::enums::Axis;
use crate::bsp::Node;
use crate::vertex::Vertex;
use crate::plane::Plane;
use crate::polygon::{Polygon, pline_area, union_all_2d, build_orthonormal_basis};
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, Vector3,
};
use crate::float_types::parry3d::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::{Shape, SharedShape, TriMesh, Triangle},
};
use crate::float_types::rapier3d::prelude::*;
use std::collections::HashMap;
use cavalier_contours::polyline::{
    PlineSource, Polyline,
};
use chull::ConvexHullWrapper;
use meshtext::{Glyph, MeshGenerator, MeshText};
use std::io::Cursor;
use std::error::Error;
use dxf::entities::*;
use dxf::Drawing;
use stl_io;

/// The main CSG solid structure. Contains a list of polygons.
#[derive(Debug, Clone)]
pub struct CSG<S: Clone> {
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone> CSG<S> {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG {
            polygons: Vec::new(),
        }
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
    pub fn from_polylines(polylines: Vec<Polyline<Real>>, metadata: Option<S>) -> CSG<S> {
        let mut all_polygons = Vec::new();
        let plane_normal = Vector3::z();

        for pl in polylines {
            // Convert each Polyline into a single polygon in z=0.
            // todo: For arcs, subdivide by bulge, etc. This ignores arcs for simplicity.
            let open = !pl.is_closed();
            if pl.vertex_count() >= 2 {
                let mut poly_verts = Vec::with_capacity(pl.vertex_count());
                for i in 0..pl.vertex_count() {
                    let v = pl.at(i);
                    poly_verts.push(Vertex::new(
                        nalgebra::Point3::new(v.x, v.y, 0.0),
                        plane_normal,
                    ));
                }
                all_polygons.push(Polygon::new(poly_verts, open, metadata.clone()));
            }
        }

        CSG::from_polygons(all_polygons)
    }

    /// Constructs a new CSG solid polygons provided in the format that earclip accepts:
    /// a slice of polygons, each a Vec of points (each point a Vec<Real> of length 2 or 3).
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
    pub fn from_earclip(polys: &[Vec<Vec<Real>>]) -> CSG<S> {
        // Flatten (as in data, not geometry) the input. If the input is 2D, dim will be 2.
        let (vertices, hole_indices, dim) = earclip::flatten(polys);
        // Tessellate the polygon using earcut.
        let earcut_indices: Vec<usize> = earclip::earcut(&vertices, &hole_indices, dim);

        let mut new_polygons = Vec::new();
        // Each consecutive triple in the output indices defines a triangle.
        for tri in earcut_indices.chunks_exact(3) {
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
                // todo:  compute the true face normal from the triangle vertices.)
                let normal = Vector3::z();
                tri_vertices.push(Vertex::new(p, normal));
            }
            // Create a polygon (triangle) with no metadata.
            new_polygons.push(Polygon::new(tri_vertices, CLOSED, None));
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
    /// - `width`: the width of the square
    /// - `length`: the height of the square
    ///
    /// # Example
    /// let sq2 = CSG::square(2.0, 3.0, None);
    pub fn square(width: Real, length: Real, metadata: Option<S>) -> CSG<S> {
        // Single 2D polygon, normal = +Z
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), normal),
            Vertex::new(Point3::new(width, 0.0, 0.0), normal),
            Vertex::new(Point3::new(width, length, 0.0), normal),
            Vertex::new(Point3::new(0.0, length, 0.0), normal),
        ];
        CSG::from_polygons(vec![Polygon::new(vertices, CLOSED, metadata.clone())])
    }

    /// Creates a 2D circle in the XY plane.
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>)>) -> CSG<S> {
        let mut verts = Vec::with_capacity(segments);
        let normal = Vector3::new(0.0, 0.0, 1.0);

        for i in 0..segments {
            let theta = 2.0 * PI * (i as Real) / (segments as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            verts.push(Vertex::new(Point3::new(x, y, 0.0), normal));
        }

        CSG::from_polygons(vec![Polygon::new(verts, CLOSED, metadata)])
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
    /// let poly2d = CSG::polygon_2d(&pts, metadata);
    pub fn polygon_2d(points: &[[Real; 2]], metadata: Option<S>) -> CSG<S> {
        // todo: return error
        assert!(points.len() >= 3, "polygon_2d requires at least 3 points");

        let normal = Vector3::new(0.0, 0.0, 1.0);
        let mut verts = Vec::with_capacity(points.len());
        for p in points {
            verts.push(Vertex::new(Point3::new(p[0], p[1], 0.0), normal));
        }
        CSG::from_polygons(vec![Polygon::new(verts, CLOSED, metadata)])
    }
    
    /// Create a right prism (a box) that spans from (0, 0, 0) 
    /// to (width, length, height). All dimensions must be >= 0.
    pub fn cube(width: Real, length: Real, height: Real, metadata: Option<S>) -> CSG<S> {
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::new(0.0,      0.0,      0.0);
        let p100 = Point3::new(width,    0.0,      0.0);
        let p110 = Point3::new(width,    length,   0.0);
        let p010 = Point3::new(0.0,      length,   0.0);

        let p001 = Point3::new(0.0,      0.0,      height);
        let p101 = Point3::new(width,    0.0,      height);
        let p111 = Point3::new(width,    length,   height);
        let p011 = Point3::new(0.0,      length,   height);

        // We’ll define 6 faces (each a Polygon), in an order that keeps outward-facing normals 
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Bottom face (z=0, normal approx. -Z)
        // p000 -> p100 -> p110 -> p010
        let bottom_normal = Vector3::new(0.0, 0.0, -1.0);
        let bottom = Polygon::new(
            vec![
                Vertex::new(p000, bottom_normal),
                Vertex::new(p010, bottom_normal),
                Vertex::new(p110, bottom_normal),
                Vertex::new(p100, bottom_normal),
            ],
            CLOSED,
            metadata.clone(),
        );

        // Top face (z=depth, normal approx. +Z)
        // p001 -> p011 -> p111 -> p101
        let top_normal = Vector3::new(0.0, 0.0, 1.0);
        let top = Polygon::new(
            vec![
                Vertex::new(p001, top_normal),
                Vertex::new(p101, top_normal),
                Vertex::new(p111, top_normal),
                Vertex::new(p011, top_normal),                
            ],
            CLOSED,
            metadata.clone(),
        );

        // Front face (y=0, normal approx. -Y)
        // p000 -> p001 -> p101 -> p100
        let front_normal = Vector3::new(0.0, -1.0, 0.0);
        let front = Polygon::new(
            vec![
                Vertex::new(p000, front_normal),
                Vertex::new(p100, front_normal),
                Vertex::new(p101, front_normal),
                Vertex::new(p001, front_normal),
            ],
            CLOSED,
            metadata.clone(),
        );

        // Back face (y=height, normal approx. +Y)
        // p010 -> p110 -> p111 -> p011
        let back_normal = Vector3::new(0.0, 1.0, 0.0);
        let back = Polygon::new(
            vec![
                Vertex::new(p010, back_normal),
                Vertex::new(p011, back_normal),
                Vertex::new(p111, back_normal),
                Vertex::new(p110, back_normal),
            ],
            CLOSED,
            metadata.clone(),
        );

        // Left face (x=0, normal approx. -X)
        // p000 -> p010 -> p011 -> p001
        let left_normal = Vector3::new(-1.0, 0.0, 0.0);
        let left = Polygon::new(
            vec![
                Vertex::new(p000, left_normal),
                Vertex::new(p001, left_normal),
                Vertex::new(p011, left_normal),
                Vertex::new(p010, left_normal),
            ],
            CLOSED,
            metadata.clone(),
        );

        // Right face (x=width, normal approx. +X)
        // p100 -> p101 -> p111 -> p110
        let right_normal = Vector3::new(1.0, 0.0, 0.0);
        let right = Polygon::new(
            vec![
                Vertex::new(p100, right_normal),
                Vertex::new(p110, right_normal),
                Vertex::new(p111, right_normal),
                Vertex::new(p101, right_normal),
            ],
            CLOSED,
            metadata.clone(),
        );

        // Combine all faces into a CSG
        CSG::from_polygons(vec![bottom, top, front, back, left, right])
    }

    /// Construct a sphere with radius, segments, stacks
    pub fn sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>) -> CSG<S> {
        let mut polygons = Vec::new();

        for i in 0..segments {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: Real, phi: Real| {
                    let dir = Vector3::new(theta.cos() * phi.sin(), phi.cos(), theta.sin() * phi.sin());
                    Vertex::new(
                        Point3::new(
                            dir.x * radius,
                            dir.y * radius,
                            dir.z * radius,
                        ),
                        dir,
                    )
                };

                let t0 = i as Real / segments as Real;
                let t1 = (i + 1) as Real / segments as Real;
                let p0 = j as Real / stacks as Real;
                let p1 = (j + 1) as Real / stacks as Real;

                let theta0 = t0 * TAU;
                let theta1 = t1 * TAU;
                let phi0 = p0 * PI;
                let phi1 = p1 * PI;

                vertices.push(vertex(theta0, phi0));
                if j > 0 {
                    vertices.push(vertex(theta1, phi0));
                }
                if j < stacks - 1 {
                    vertices.push(vertex(theta1, phi1));
                }
                vertices.push(vertex(theta0, phi1));

                polygons.push(Polygon::new(vertices, CLOSED, metadata.clone()));
            }
        }
        CSG::from_polygons(polygons)
    }

    /// Construct a cone whose centerline goes from `start` to `end`,
    /// with a circular cross-section of given `radius`. 
    pub fn cylinder_ptp(start: Point3<Real>, end: Point3<Real>, radius: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        let s = start.coords;
        let e = end.coords;
        let ray = e - s;
        let axis_z = ray.normalize();

        // If axis_z is mostly aligned with Y, pick X; otherwise pick Y.
        let is_y = axis_z.y.abs() > 0.5;
        let mut axis_x: Vector3<Real> = if is_y {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        axis_x = axis_x.cross(&axis_z).normalize();
        let axis_y = axis_x.cross(&axis_z).normalize();

        let start_v = Vertex::new(start, -axis_z);
        let end_v = Vertex::new(end, axis_z);

        let mut polygons = Vec::new();

        // Helper to compute a vertex at a given "slice" (angle) and "stack" ([0..1])
        let point = |stack: Real, slice: Real, normal_blend: Real| {
            let angle = slice * TAU;
            let out   = axis_x * angle.cos() + axis_y * angle.sin();
            // Position: linear interpolation along the axis + radial offset
            let pos   = s + ray * stack + out * radius;
            // For the outer tube, normal is approximately `out`. For the caps,
            // we blend in ±axis_z to get a continuous normal around the rim.
            let normal = out * (1.0 - normal_blend.abs()) + axis_z * normal_blend;
            Vertex::new(Point3::from(pos), normal)
        };

        for i in 0..segments {
            let t0 = i as Real / segments as Real;
            let t1 = (i + 1) as Real / segments as Real;

            // bottom cap
            polygons.push(Polygon::new(
                vec![start_v.clone(), point(0.0, t0, -1.0), point(0.0, t1, -1.0)],
                CLOSED,
                metadata.clone(),
            ));

            // tube
            polygons.push(Polygon::new(
                vec![
                    point(0.0, t1, 0.0),
                    point(0.0, t0, 0.0),
                    point(1.0, t0, 0.0),
                    point(1.0, t1, 0.0),
                ],
                CLOSED,
                metadata.clone(),
            ));

            // top cap
            polygons.push(Polygon::new(
                vec![end_v.clone(), point(1.0, t1, 1.0), point(1.0, t0, 1.0)],
                CLOSED,
                metadata.clone(),
            ));
        }

        CSG::from_polygons(polygons)
    }
    
    // A helper to create a vertical cylinder along Z from z=0..z=height
    // with the specified radius (NOT diameter).
    pub fn cylinder(radius: f64, height: f64, segments: usize, metadata: Option<S>) -> CSG<S> {
        // csgrs::csg::cylinder_ptp takes a (Point3, Point3, Real, usize, Option(metadata))
        // (start, end, radius, segments, metadata).
        // We'll define the start at [0,0,0], the end at [0,0,height], ~32 segments:
        CSG::cylinder_ptp(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, height),
            radius,
            segments,
            metadata,
        )
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
    pub fn polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>) -> CSG<S> {
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
                    panic!(
                        "Face index {} is out of range (points.len = {}).",
                        idx,
                        points.len()
                    );
                }
                let [x, y, z] = points[idx];
                face_vertices.push(Vertex::new(
                    Point3::new(x, y, z),
                    Vector3::zeros(), // we'll set this later
                ));
            }

            // Build the polygon (plane is auto-computed from first 3 vertices).
            let mut poly = Polygon::new(face_vertices, CLOSED, metadata.clone());

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
    pub fn transform(&self, mat: &Matrix4<Real>) -> CSG<S> {
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
            poly.plane.normal = mat_inv_transpose
                .transform_vector(&poly.plane.normal)
                .normalize();

            // Plane w
            if let Some(first_vert) = poly.vertices.get(0) {
                poly.plane.w = poly.plane.normal.dot(&first_vert.pos.coords);
            }
        }

        csg
    }

    pub fn translate(&self, v: Vector3<Real>) -> CSG<S> {
        let translation = Translation3::from(v);
        // Convert to a Matrix4
        let mat4 = translation.to_homogeneous();
        self.transform(&mat4)
    }

    pub fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> CSG<S> {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    pub fn scale(&self, sx: Real, sy: Real, sz: Real) -> CSG<S> {
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
        let points: Vec<Vec<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.vertices
                    .iter()
                    .map(|v| vec![v.pos.x, v.pos.y, v.pos.z])
            })
            .collect();

        // Compute convex hull using the robust wrapper
        let hull =
            ConvexHullWrapper::try_new(&points, None).expect("Failed to compute convex hull");

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
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], CLOSED, None));
        }

        CSG::from_polygons(polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// Naive approach: Take every vertex in `self`, add it to every vertex in `other`,
    /// then compute the convex hull of all resulting points.
    pub fn minkowski_sum(&self, other: &CSG<S>) -> CSG<S> {
        // Collect all vertices (x, y, z) from self
        let verts_a: Vec<Point3<Real>> = self
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        // Collect all vertices from other
        let verts_b: Vec<Point3<Real>> = other
            .polygons
            .iter()
            .flat_map(|poly| poly.vertices.iter().map(|v| v.pos))
            .collect();

        if verts_a.is_empty() || verts_b.is_empty() {
            // Empty input to minkowski sum
        }

        // For Minkowski, add every point in A to every point in B
        let sum_points: Vec<_> = verts_a
            .iter()
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
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], CLOSED, None));
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
            for tri in sub_tris {
                new_polygons.push(Polygon::new(
                    vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                    CLOSED,
                    poly.metadata.clone(),
                ));
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
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
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
                if let Some(hit) = triangle.cast_ray_and_get_normal(&iso, &ray, Real::MAX, true) {
                    let point_on_ray = ray.point_at(hit.time_of_impact);
                    hits.push((Point3::from(point_on_ray.coords), hit.time_of_impact));
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
    pub fn extrude(&self, height: Real) -> CSG<S> {
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
    pub fn extrude_vector(&self, direction: Vector3<Real>) -> CSG<S> {    
        // Collect our polygons
        let mut result_polygons = Vec::new();
        let mut top_polygons = Vec::new();
        let mut bottom_polygons = Vec::new();

        //let unioned_polygons = &self.flatten().polygons;
        let unioned_polygons = &self.polygons; // todo

        // Bottom polygons = original polygons
        // (assuming they are in some plane, e.g. XY). We just clone them.
        for poly in unioned_polygons {
            let mut bottom = poly.clone();
            let top = poly.translate(direction);
            
            // Collect top and bottom polygons for stitching side walls in same winding orientation
            bottom_polygons.push(bottom.clone());
            top_polygons.push(top.clone());
            
            bottom.flip(); // flip winding of the bottom polygon for correct normals
            
            // Collect top and bottom polygons in result
            result_polygons.push(bottom);
            result_polygons.push(top);
        }

        // Side polygons = For each polygon in `self`, connect its edges
        // from the original to the corresponding edges in the translated version.
        //
        // We'll iterate over each polygon’s vertices. For each edge (v[i], v[i+1]),
        // we form a rectangular side quad with (v[i]+direction, v[i+1]+direction).
        // That is, a quad [b_i, b_j, t_j, t_i].
        for (poly_bottom, poly_top) in bottom_polygons.iter().zip(top_polygons.iter()) {
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
                    false,
                    None,
                );
                result_polygons.push(side_poly);
            }
        }

        // Combine into a new CSG
        CSG::from_polygons(result_polygons)
    }

    /// Extrudes (or "lofts") a closed 3D volume between two polygons in space.
    /// - `bottom` and `top` each have the same number of vertices `n`, in matching order.
    /// - Returns a new CSG whose faces are:
    ///   - The `bottom` polygon,
    ///   - The `top` polygon,
    ///   - `n` rectangular side polygons bridging each edge of `bottom` to the corresponding edge of `top`.
    pub fn extrude_between(
        bottom: &Polygon<S>,
        top: &Polygon<S>,
        flip_bottom_polygon: bool,
    ) -> CSG<S> {
        let n = bottom.vertices.len();
        assert_eq!(
            n,
            top.vertices.len(),
            "extrude_between: both polygons must have the same number of vertices" // todo: return error
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
                CLOSED,
                bottom.metadata.clone(), // carry over bottom polygon metadata
            );
            polygons.push(side_poly);
        }

        CSG::from_polygons(polygons)
    }

    /// Rotate-extrude (revolve) this 2D shape around the Z-axis from 0..`angle_degs`
    /// by replicating the original polygon(s) at each step and calling `extrude_between`.
    /// Caps are added automatically if the revolve is partial (angle < 360°).
    pub fn rotate_extrude(&self, angle_degs: Real, segments: usize) -> CSG<S> {
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
                let frac = i as Real / segments as Real;
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
    
    /// Extrude an open or closed 2D polyline (from cavalier_contours) along `direction`,
    /// returning a 3D `CSG` containing the resulting side walls plus top/bottom if it’s closed.
    /// For open polylines, no “caps” are added unless you do so manually.
    pub fn extrude_polyline(poly: Polyline<Real>, direction: Vector3<Real>, metadata: Option<S>) -> CSG<S> {
        if poly.vertex_count() < 2 {
            return CSG::new();
        }

        let open = !poly.is_closed();
        let polygon_bottom = Polygon::from_polyline(poly, metadata.clone());
        let mut result_polygons = Vec::new();

        // "bottom" polygon => keep it only if closed
        if !open {
            let mut bottom = polygon_bottom.clone();
            // The top polygon is just a translate
            let top = bottom.translate(direction);

            // Flip winding on the bottom
            bottom.flip();

            result_polygons.push(bottom);
            result_polygons.push(top);
        }

        // Build side walls
        let b_verts = &polygon_bottom.vertices;
        let t_verts: Vec<_> = b_verts.iter().map(|v| {
            let mut tv = v.clone();
            tv.pos += direction;
            tv
        }).collect();

        let vcount = b_verts.len();
        for i in 0..(vcount-0) {  // if closed, we wrap, if open, we do (vcount-1) for side segments
            let j = (i+1) % vcount; 
            // For open polyline, skip the last segment that is the "wrap around" if not closed:
            if open && j == 0 {
                break;
            }
            let b_i = b_verts[i].clone();
            let b_j = b_verts[j].clone();
            let t_i = t_verts[i].clone();
            let t_j = t_verts[j].clone();

            let side = Polygon::new(vec![b_i, b_j, t_j, t_i], CLOSED, metadata.clone());
            result_polygons.push(side);
        }
        CSG::from_polygons(result_polygons)
    }

    /// Given a list of Polygons that each represent a 2D open polyline (in XY, z=0),
    /// reconstruct a single 3D polyline by matching consecutive endpoints in 3D space.
    /// (If some polygons are closed, you can skip them or handle differently.)
    ///
    /// Returns a vector of 3D points (the polyline’s vertices). 
    /// If no matching is possible or the polygons are empty, returns an empty vector.
    pub fn reconstruct_polyline_3d(polylines: &[Polygon<S>]) -> Vec<nalgebra::Point3<Real>> {
        // Collect open polylines in 2D first:
        let mut all_points = Vec::new();
        for poly in polylines {
            if !poly.open {
                // skip or handle closed polygons differently
                continue;
            }
            // Convert to 2D
            let pline_2d = poly.to_2d();
            if pline_2d.vertex_count() < 2 {
                continue;
            }
            // gather all points
            let mut segment_points = Vec::with_capacity(pline_2d.vertex_count());
            for i in 0..pline_2d.vertex_count() {
                let v = pline_2d.at(i);
                segment_points.push(nalgebra::Point3::new(v.x, v.y, 0.0));
            }
            all_points.push(segment_points);
        }
        if all_points.is_empty() {
            return vec![];
        }

        // Simple approach: assume each open polyline’s end matches the next polyline’s start,
        // building one continuous chain. 
        // More sophisticated logic might do a tolerance-based matching, 
        // or unify them in a single chain, etc.
        let mut chain = Vec::new();
        // Start with the first polyline’s points
        chain.extend(all_points[0].clone());
        // Then see if the last point matches the first point of the next polyline, 
        // etc. (You can do any matching logic you prefer.)
        for i in 1..all_points.len() {
            let prev_end = chain.last().unwrap();
            let next_start = all_points[i][0];
            // If they match within some tolerance, skip the next start:
            if (prev_end.coords - next_start.coords).norm() < 1e-9 {
                // skip the next_start
                chain.extend(all_points[i].iter().skip(1).cloned());
            } else {
                // if no match, just connect them with a big jump
                chain.extend(all_points[i].iter().cloned());
            }
        }

        chain
    }

    /// Returns a `parry3d::bounding_volume::Aabb`.
    pub fn bounding_box(&self) -> Aabb {
        // Gather all points from all polygons.
        // parry expects a slice of `&Point3<Real>` or a slice of `na::Point3<Real>`.
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

    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using cavalier_contours parallel_offset.
    /// for each Polygon we convert to a cavalier_contours Polyline<Real> and call parallel_offset
    pub fn offset_2d(&self, distance: Real) -> CSG<S> {
        let mut result_polygons = Vec::new(); // each "loop" is a Polygon

        for poly in &self.polygons {
            // Convert to cavalier_contours Polyline (closed by default):
            let cpoly = poly.to_polyline();

            // Remove any degenerate or redundant vertices:
            cpoly.remove_redundant(EPSILON);

            // Perform the actual offset:
            let result_plines = cpoly.parallel_offset(-distance);

            // Collect polygons
            for pline in result_plines {
                result_polygons.push(Polygon::from_polyline(pline, poly.metadata.clone()));
            }
        }

        // Build a new CSG from those offset loops in XY:
        CSG::from_polygons(result_polygons)
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
            let cc = poly.to_polyline();
            // Optional: remove redundant points
            cc.remove_redundant(EPSILON);

            // Check area (shoelace). If above threshold, turn it into a 2D Polygon
            let area = pline_area(&cc).abs();
            if area > eps_area {
                polys_2d.push(Polygon::from_polyline(cc, poly.metadata.clone()));
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
    pub fn slice(&self, plane: Option<Plane>) -> CSG<S> {
        let _plane = plane.unwrap_or_else(|| Plane {
            normal: nalgebra::Vector3::new(0.0, 0.0, 1.0),
            w: 0.0,
        });

        let result_polygons = Vec::new();

        CSG::from_polygons(result_polygons)
    }

    /// Convert a `MeshText` (from meshtext) into a list of `Polygon` in the XY plane.
    /// - `scale` allows you to resize the glyph (e.g. matching a desired font size).
    /// - By default, the glyph’s normal is set to +Z.
    fn meshtext_to_polygons(glyph_mesh: &meshtext::MeshText, scale: Real, metadata: Option<S>) -> Vec<Polygon<S>> {
        let mut polygons = Vec::new();
        let verts = &glyph_mesh.vertices;

        // Each set of 9 floats = one triangle: (x1,y1,z1, x2,y2,z2, x3,y3,z3)
        for tri_chunk in verts.chunks_exact(9) {
            let x1 = tri_chunk[0] as Real;
            let y1 = tri_chunk[1] as Real;
            let z1 = tri_chunk[2] as Real;
            let x2 = tri_chunk[3] as Real;
            let y2 = tri_chunk[4] as Real;
            let z2 = tri_chunk[5] as Real;
            let x3 = tri_chunk[6] as Real;
            let y3 = tri_chunk[7] as Real;
            let z3 = tri_chunk[8] as Real;

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
                CLOSED,
                metadata.clone(),
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
    pub fn text(text_str: &str, font_data: &[u8], size: Option<Real>, metadata: Option<S>) -> CSG<S> {
        let mut generator = MeshGenerator::new(font_data.to_vec());
        let scale = size.unwrap_or(20.0);

        let mut all_polygons = Vec::new();
        let mut cursor_x: Real = 0.0;

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
            let glyph_polygons = Self::meshtext_to_polygons(&glyph_mesh, scale, metadata.clone());

            // Translate polygons by (cursor_x, 0.0)
            let glyph_csg =
                CSG::from_polygons(glyph_polygons).translate(Vector3::new(cursor_x, 0.0, 0.0));
            // Accumulate
            all_polygons.extend(glyph_csg.polygons);

            // Advance cursor by the glyph’s bounding-box width
            let glyph_width = glyph_mesh.bbox.max.x - glyph_mesh.bbox.min.x;
            cursor_x += glyph_width as Real * scale;
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
            // We'll store them in a flat `Vec<Real>` of the form [x0, y0, x1, y1, x2, y2, ...]
            let mut coords_2d = Vec::with_capacity(poly.vertices.len() * 2);
            for vert in &poly.vertices {
                let offset = vert.pos.coords - p0.coords; // vector from p0 to the vertex
                let x = offset.dot(&u);
                let y = offset.dot(&v);
                coords_2d.push(x);
                coords_2d.push(y);
            }

            // 4) Call Earcut on that 2D outline. We assume no holes, so hole_indices = &[].
            //    earcut's signature is `earcut::<Real, usize>(data, hole_indices, dim)`
            //    with `dim = 2` for our XY data.
            let indices: Vec<usize> = earclip::earcut(&coords_2d, &[], 2);

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
                    let pos_3d = Point3::from(pos_vec);
                    // We can store the normal = polygon's plane normal (or recalc).
                    // We'll recalc below, so for now just keep n or 0 as a placeholder:
                    tri_vertices.push(Vertex::new(pos_3d, n));
                }

                // Create a polygon from these 3 vertices. We preserve the metadata:
                let mut new_poly = Polygon::new(tri_vertices, CLOSED, poly.metadata.clone());

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
                indices.push([index_offset, index_offset + 1, index_offset + 2]);
                index_offset += 3;
            }
        }

        // TriMesh::new(Vec<[Real; 3]>, Vec<[u32; 3]>)
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(&self, density: Real) -> (Real, Point3<Real>, Unit<Quaternion<Real>>) {
        let shape = self.to_trimesh();
        if let Some(trimesh) = shape.as_trimesh() {
            let mp = trimesh.mass_properties(density);
            (
                mp.mass(),
                mp.local_com,                     // a Point3<Real>
                mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
            )
        } else {
            // fallback if not a TriMesh
            (0.0, Point3::origin(), Unit::<Quaternion<Real>>::identity())
        }
    }

    /// Create a Rapier rigid body + collider from this CSG, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<Real>,
        rotation: Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
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
        let coll = ColliderBuilder::new(shape).density(density).build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);

        rb_handle
    }

    /// Checks if the CSG object is manifold.
    ///
    /// This function defines a comparison function which takes EPSILON into account
    /// for Real coordinates, builds a hashmap key from the string representation of
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
        fn approx_lt(a: &Point3<Real>, b: &Point3<Real>) -> bool {
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
        fn point_key(p: &Point3<Real>) -> String {
            // Truncate/round to e.g. 6 decimals
            format!("{:.6},{:.6},{:.6}", p.x, p.y, p.z)
        }

        let mut edge_counts: HashMap<(String, String), u32> = HashMap::new();

        for poly in &self.polygons {
            // Triangulate each polygon
            for tri in poly.triangulate() {
                // Each tri is 3 vertices: [v0, v1, v2]
                // We'll look at edges (0->1, 1->2, 2->0).
                for &(i0, i1) in &[(0, 1), (1, 2), (2, 0)] {
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
                    normal: stl_io::Normal::new([
                        normal.x as f32,
                        normal.y as f32,
                        normal.z as f32,
                    ]),
                    vertices: [
                        stl_io::Vertex::new([
                            tri[0].pos.x as f32,
                            tri[0].pos.y as f32,
                            tri[0].pos.z as f32,
                        ]),
                        stl_io::Vertex::new([
                            tri[1].pos.x as f32,
                            tri[1].pos.y as f32,
                            tri[1].pos.z as f32,
                        ]),
                        stl_io::Vertex::new([
                            tri[2].pos.x as f32,
                            tri[2].pos.y as f32,
                            tri[2].pos.z as f32,
                        ]),
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
                    Point3::new(
                        tri.vertices[0][0] as Real,
                        tri.vertices[0][1] as Real,
                        tri.vertices[0][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[1][0] as Real,
                        tri.vertices[1][1] as Real,
                        tri.vertices[1][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
                Vertex::new(
                    Point3::new(
                        tri.vertices[2][0] as Real,
                        tri.vertices[2][1] as Real,
                        tri.vertices[2][2] as Real,
                    ),
                    Vector3::new(
                        tri.normal[0] as Real,
                        tri.normal[1] as Real,
                        tri.normal[2] as Real,
                    ),
                ),
            ];
            polygons.push(Polygon::new(vertices, CLOSED, None));
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
                                Point3::new(
                                    vertex.location.x as Real,
                                    vertex.location.y as Real,
                                    vertex.location.z as Real,
                                ),
                                Vector3::new(0.0, 0.0, 1.0), // Assuming flat in XY
                            ));
                        }
                        // Create a polygon from the polyline vertices
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, CLOSED, None));
                        }
                    }
                }
                EntityType::Circle(circle) => {
                    // Approximate circles with regular polygons
                    let center = Point3::new(circle.center.x as Real, circle.center.y as Real, circle.center.z as Real);
                    let radius = circle.radius as Real;
                    let segments = 32; // Number of segments to approximate the circle

                    let mut verts = Vec::new();
                    let normal = Vector3::new(0.0, 0.0, 1.0); // Assuming circle lies in XY plane

                    for i in 0..segments {
                        let theta = 2.0 * PI * (i as Real) / (segments as Real);
                        let x = center.x as Real + radius * theta.cos();
                        let y = center.y as Real + radius * theta.sin();
                        let z = center.z as Real;
                        verts.push(Vertex::new(Point3::new(x, y, z), normal));
                    }

                    // Create a polygon from the approximated circle vertices
                    polygons.push(Polygon::new(verts, CLOSED, None));
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
                    dxf::Point::new(tri[0].pos.x as f64, tri[0].pos.y as f64, tri[0].pos.z as f64),
                    dxf::Point::new(tri[1].pos.x as f64, tri[1].pos.y as f64, tri[1].pos.z as f64),
                    dxf::Point::new(tri[2].pos.x as f64, tri[2].pos.y as f64, tri[2].pos.z as f64),
                    dxf::Point::new(tri[2].pos.x as f64, tri[2].pos.y as f64, tri[2].pos.z as f64), // Duplicate for triangular face
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
