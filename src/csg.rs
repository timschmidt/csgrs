use crate::float_types::{EPSILON, PI, TAU, OPEN, CLOSED, Real};
use crate::bsp::Node;
use crate::vertex::Vertex;
use crate::plane::Plane;
use crate::polygon::{Polygon, polyline_area, union_all_2d, build_orthonormal_basis};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, Vector3,
};
use std::error::Error;
use cavalier_contours::polyline::{
    PlineSource, Polyline, PlineSourceMut,
};
use crate::float_types::parry3d::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::{Shape, SharedShape, TriMesh, Triangle},
};
use crate::float_types::rapier3d::prelude::*;

#[cfg(feature = "hashmap")]
use hashbrown::HashMap;

#[cfg(feature = "chull-io")]
use chull::ConvexHullWrapper;

#[cfg(feature = "earcut-io")]
use earcut::Earcut;

#[cfg(feature = "hershey-text")]
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "truetype-text")]
use meshtext::{Glyph, MeshGenerator, MeshText};

#[cfg(any(feature = "stl-io", feature = "dxf-io"))]
use core2::io::Cursor;

#[cfg(feature = "dxf-io")]
use dxf::entities::*;
#[cfg(feature = "dxf-io")]
use dxf::Drawing;

#[cfg(feature = "stl-io")]
use stl_io;

#[cfg(feature = "image-io")]
use image::GrayImage;

#[cfg(any(feature = "metaballs", feature = "sdf"))]
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};

#[derive(Debug, Clone)]
#[cfg(feature = "metaballs")]
pub struct MetaBall {
    pub center: Point3<Real>,
    pub radius: Real,
}

#[cfg(feature = "metaballs")]
impl MetaBall {
    pub fn new(center: Point3<Real>, radius: Real) -> Self {
        Self { center, radius }
    }

    /// “Influence” function used by the scalar field for metaballs
    pub fn influence(&self, p: &Point3<Real>) -> Real {
        let dist_sq = (p - self.center).norm_squared() + EPSILON;
        self.radius * self.radius / dist_sq
    }
}

/// Summation of influences from multiple metaballs.
#[cfg(feature = "metaballs")]
fn scalar_field_metaballs(balls: &[MetaBall], p: &Point3<Real>) -> Real {
    let mut value = 0.0;
    for ball in balls {
        value += ball.influence(p);
    }
    value
}

/// The main CSG solid structure. Contains a list of polygons.
#[derive(Debug, Clone)]
pub struct CSG<S: Clone> {
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone> CSG<S> where S: Clone + Send + Sync {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG {
            polygons: Vec::new(),
        }
    }

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons.to_vec();
        csg
    }

    /// Return the internal polygons
    pub fn to_polygons(&self) -> &[Polygon<S>] {
        &self.polygons
    }

    // Group polygons by their metadata.
    //
    // Returns a map from the metadata (as `Option<S>`) to a
    // list of references to all polygons that have that metadata.
    //
    // # Example
    // ```
    // let mut csg = CSG::new();
    // // ... fill `csg.polygons` with some that share metadata, some that have None, etc.
    //
    // let grouped = csg.polygons_by_metadata();
    // for (meta, polys) in &grouped {
    //     println!("Metadata = {:?}, #polygons = {}", meta, polys.len());
    // }
    // ```
    // requires impl<S: Clone + Eq + Hash> CSG<S> { and use std::collections::HashMap; use std::hash::Hash;
    // pub fn polygons_by_metadata(&self) -> HashMap<Option<S>, Vec<&Polygon<S>>> {
    //    let mut map: HashMap<Option<S>, Vec<&Polygon<S>>> = HashMap::new();
    //    
    //    for poly in &self.polygons {
    //        // Clone the `Option<S>` so we can use it as the key
    //        let key = poly.metadata.clone();
    //        map.entry(key).or_default().push(poly);
    //    }
    //    
    //    map
    // }

    // Return polygons grouped by metadata
    // requires impl<S: Clone + std::cmp::PartialEq> CSG<S> {
    // pub fn polygons_by_metadata_partialeq(&self) -> Vec<(Option<S>, Vec<&Polygon<S>>)> {
    //    let mut groups: Vec<(Option<S>, Vec<&Polygon<S>>)> = Vec::new();
    //    'outer: for poly in &self.polygons {
    //        let meta = poly.metadata.clone();
    //        // Try to find an existing group with the same metadata (requires a way to compare!)
    //        for (existing_meta, polys) in &mut groups {
    //            // For this to work, you need some form of comparison on `S`.
    //            // If S does not implement Eq, you might do partial compare or pointer compare, etc.
    //            if *existing_meta == meta {
    //                polys.push(poly);
    //                continue 'outer;
    //            }
    //        }
    //        // Otherwise, start a new group
    //        groups.push((meta, vec![poly]));
    //    }
    //    groups
    // }

    /// Build a new CSG from a set of 2D polylines in XY. Each polyline
    /// is turned into one polygon at z=0. If a union produced multiple
    /// loops, you will get multiple polygons in the final CSG.
    pub fn from_polylines(polylines: &[Polyline<Real>], metadata: Option<S>) -> CSG<S> {
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
                        Point3::new(v.x, v.y, 0.0),
                        plane_normal,
                    ));
                }
                all_polygons.push(Polygon::new(poly_verts, open, metadata.clone()));
            }
        }

        CSG::from_polygons(&all_polygons)
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
    #[cfg(feature = "earclip-io")]
    pub fn from_earclip(polys: &[Vec<Vec<Real>>], metadata: Option<S>) -> CSG<S> {
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
                let normal = Vector3::z();
                tri_vertices.push(Vertex::new(p, normal));
            }
            // Create a polygon (triangle)
            // todo:  compute the true face normal from the triangle vertices.): let normal = (b - a).cross(&(c - a)).normalize();
            new_polygons.push(Polygon::new(tri_vertices, CLOSED, metadata.clone()));
        }
        CSG::from_polygons(&new_polygons)
    }
    
    /// Constructs a new CSG solid by triangulating a complex polygon using the earcut algorithm.
    ///
    /// # Parameters
    ///
    /// - `polys`: A slice of polygons, where each polygon is represented as a Vec of points.
    ///            Each point is itself a Vec<Real> of length 2 or 3. The first polygon is taken as the outer loop
    ///            and any subsequent ones are considered holes.
    /// - `metadata`: Optional shared metadata to store in each resulting triangle.
    ///
    /// # Returns
    ///
    /// A new CSG whose polygons are the triangles produced by earcut.
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
    /// let polys = vec![outer, hole];
    ///
    /// let csg = CSG::<()>::from_earcut(&polys, None);
    /// // Now `csg.polygons` contains the triangulated version.
    /// ```
    #[cfg(feature = "earcut-io")]
    pub fn from_earcut(polys: &[Vec<Vec<Real>>], metadata: Option<S>) -> CSG<S> {
        // If no input is provided, return an empty CSG.
        if polys.is_empty() {
            return CSG::new();
        }

        // Build a flat list of 2D vertices and a list of hole indices.
        // (We always work in 2D for earcut; if points have 3 coordinates we ignore the third.)
        let mut vertices: Vec<[Real; 2]> = Vec::new();
        let mut hole_indices: Vec<usize> = Vec::new();

        for (i, poly) in polys.iter().enumerate() {
            // For each polygon after the first, record the current length as a hole start index.
            if i > 0 {
                hole_indices.push(vertices.len());
            }
            for point in poly {
                if point.len() < 2 {
                    // Skip degenerate points.
                    continue;
                }
                vertices.push([point[0], point[1]]);
            }
        }

        // Call earcut to triangulate.
        let mut earcut = Earcut::new();
        let mut triangle_indices: Vec<usize> = Vec::new();
        earcut.earcut(vertices.clone(), &hole_indices, &mut triangle_indices);

        // Each consecutive triplet in triangle_indices defines a triangle.
        let mut triangles = Vec::new();
        for tri in triangle_indices.chunks_exact(3) {
            let i0 = tri[0];
            let i1 = tri[1];
            let i2 = tri[2];

            let v0 = Vertex::new(
                Point3::new(vertices[i0][0], vertices[i0][1], 0.0),
                Vector3::z(),
            );
            let v1 = Vertex::new(
                Point3::new(vertices[i1][0], vertices[i1][1], 0.0),
                Vector3::z(),
            );
            let v2 = Vertex::new(
                Point3::new(vertices[i2][0], vertices[i2][1], 0.0),
                Vector3::z(),
            );

            // Create a triangle polygon (closed) from these three vertices.
            triangles.push(Polygon::new(vec![v0, v1, v2], CLOSED, metadata.clone()));
        }

        CSG::from_polygons(&triangles)
    }

    /// CSG union: this ∪ other
    pub fn union(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        CSG::from_polygons(&a.all_polygons())
    }

    /// CSG difference: this \ other
    pub fn difference(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        CSG::from_polygons(&a.all_polygons())
    }

    /// CSG intersection: this ∩ other
    pub fn intersection(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        CSG::from_polygons(&a.all_polygons())
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
        let normal = Vector3::z();
        let vertices = vec![
            Vertex::new(Point3::origin(), normal),
            Vertex::new(Point3::new(width, 0.0, 0.0), normal),
            Vertex::new(Point3::new(width, length, 0.0), normal),
            Vertex::new(Point3::new(0.0, length, 0.0), normal),
        ];
        CSG::from_polygons(&[Polygon::new(vertices, CLOSED, metadata)])
    }

    /// Creates a 2D circle in the XY plane.
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        let mut vertices = Vec::with_capacity(segments);
        let normal = Vector3::z();

        for i in 0..segments {
            let theta = 2.0 * PI * (i as Real) / (segments as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            vertices.push(Vertex::new(Point3::new(x, y, 0.0), normal));
        }

        CSG::from_polygons(&[Polygon::new(vertices, CLOSED, metadata)])
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
        // todo: return error "polygon_2d requires at least 3 points"
        if points.len() < 3 {
            return CSG::new();
        }

        let normal = Vector3::z();
        let mut vertices = Vec::with_capacity(points.len());
        for p in points {
            vertices.push(Vertex::new(Point3::new(p[0], p[1], 0.0), normal));
        }
        CSG::from_polygons(&[Polygon::new(vertices, CLOSED, metadata)])
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
        let bottom_normal = -Vector3::z();
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
        let top_normal = Vector3::z();
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
        let front_normal = -Vector3::y();
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
        let back_normal = Vector3::y();
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
        let left_normal = -Vector3::x();
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
        let right_normal = Vector3::x();
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
        CSG::from_polygons(&[bottom, top, front, back, left, right])
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
        CSG::from_polygons(&polygons)
    }
    
    /// Construct a frustum whose axis goes from `start` to `end`, with the start face having
    /// radius = `radius1` and the end face having radius = `radius2`.
    pub fn frustrum_ptp(
        start: Point3<Real>,
        end: Point3<Real>,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        let s = start.coords;
        let e = end.coords;
        let ray = e - s;
    
        // If the start and end coincide, return an empty CSG or handle gracefully
        if ray.norm_squared() < EPSILON {
            return CSG::new();
        }
    
        // We’ll choose an axis_z aligned with the start->end vector
        let axis_z = ray.normalize();
    
        // Pick an axis_x that is not parallel to axis_z
        let axis_x = if axis_z.y.abs() > 0.5 {
            Vector3::x()
        } else {
            Vector3::y()
        }
        .cross(&axis_z)
        .normalize();
    
        // Now define axis_y = axis_x × axis_z
        let axis_y = axis_x.cross(&axis_z).normalize();
    
        // For convenience, define "center" vertices for the caps
        let start_v = Vertex::new(start, -axis_z); // bottom cap center
        let end_v = Vertex::new(end, axis_z);      // top cap center
    
        // We’ll collect polygons for the bottom cap, top cap, and side walls
        let mut polygons = Vec::new();
    
        // Helper: given a "stack" (0.0 for bottom, 1.0 for top) and a "slice" in [0..1],
        // return a Vertex on the frustum surface. `normal_blend` controls how
        // we blend the purely radial normal vs. the cap normal for bottom/top.
        let point = |stack: Real, slice: Real, normal_blend: Real| {
            // Interpolate radius by stack: 0 => radius1, 1 => radius2
            let r = radius1 * (1.0 - stack) + radius2 * stack;
    
            // Convert the slice fraction into an angle around the axis
            let angle = slice * TAU;
            let radial_dir = axis_x * angle.cos() + axis_y * angle.sin();
    
            // Position in 3D
            let pos = s + ray * stack + radial_dir * r;
    
            // For a perfect cylinder, the side normal is radial. For the caps,
            // we blend in ±axis_z for smooth normals at the seam. You can
            // omit this blend if you prefer simpler “hard” edges.
            let normal = radial_dir * (1.0 - normal_blend.abs()) + axis_z * normal_blend;
            Vertex::new(Point3::from(pos), normal.normalize())
        };
    
        // Build polygons via "fan" for bottom cap, side quads, and "fan" for top cap
        for i in 0..segments {
            let slice0 = i as Real / segments as Real;
            let slice1 = (i + 1) as Real / segments as Real;
    
            //
            // Bottom cap triangle
            //  -- "fan" from start_v to ring edges at stack=0
            //
            polygons.push(Polygon::new(
                vec![
                    start_v.clone(),
                    point(0.0, slice0, -1.0),
                    point(0.0, slice1, -1.0),
                ],
                CLOSED,
                metadata.clone(),
            ));
    
            //
            // Side wall (a quad) bridging stack=0..1 at slice0..slice1
            // The four corners are:
            //   (0.0, slice1), (0.0, slice0), (1.0, slice0), (1.0, slice1)
            //
            polygons.push(Polygon::new(
                vec![
                    point(0.0, slice1, 0.0),
                    point(0.0, slice0, 0.0),
                    point(1.0, slice0, 0.0),
                    point(1.0, slice1, 0.0),
                ],
                CLOSED,
                metadata.clone(),
            ));
    
            //
            // Top cap triangle
            //  -- "fan" from end_v to ring edges at stack=1
            //
            polygons.push(Polygon::new(
                vec![
                    end_v.clone(),
                    point(1.0, slice1, 1.0),
                    point(1.0, slice0, 1.0),
                ],
                CLOSED,
                metadata.clone(),
            ));
        }
    
        CSG::from_polygons(&polygons)
    }
    
    // A helper to create a vertical cylinder along Z from z=0..z=height
    // with the specified radius (NOT diameter).
    pub fn frustrum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        CSG::frustrum_ptp(
            Point3::origin(),
            Point3::new(0.0, 0.0, height),
            radius1,
            radius2,
            segments,
            metadata,
        )
    }
    
    // A helper to create a vertical cylinder along Z from z=0..z=height
    // with the specified radius (NOT diameter).
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        CSG::frustrum_ptp(
            Point3::origin(),
            Point3::new(0.0, 0.0, height),
            radius.clone(),
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
                    panic!( // todo return error
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

        CSG::from_polygons(&polygons)
    }

    /// Transform all vertices in this CSG by a given 4×4 matrix.
    pub fn transform(&self, mat: &Matrix4<Real>) -> CSG<S> {
        let mat_inv_transpose = mat.try_inverse().unwrap().transpose(); // todo catch error
        let mut csg = self.clone();

        for poly in &mut csg.polygons {
            for vert in &mut poly.vertices {
                // Position
                let hom_pos = mat * vert.pos.to_homogeneous();
                vert.pos = Point3::from_homogeneous(hom_pos).unwrap();  // todo catch error

                // Normal
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }
                
            if poly.vertices.len() >= 3 {
                poly.plane = Plane::from_points(&poly.vertices[0].pos, &poly.vertices[1].pos, &poly.vertices[2].pos);
            }
        }

        csg
    }

    /// Returns a new CSG translated by x, y, and z.
    ///
    pub fn translate(&self, x: Real, y: Real, z: Real) -> CSG<S> {
        self.translate_vector(Vector3::new(x, y, z))
    }
    
    /// Returns a new CSG translated by vector.
    ///
    pub fn translate_vector(&self, vector: Vector3<Real>) -> CSG<S> {
        let translation = Translation3::from(vector);
        // Convert to a Matrix4
        let mat4 = translation.to_homogeneous();
        self.transform(&mat4)
    }
    
    /// Returns a new CSG translated so that its bounding-box center is at the origin (0,0,0).
    pub fn center(&self) -> Self {
        let aabb = self.bounding_box();
        
        // Compute the AABB center
        let center_x = (aabb.mins.x + aabb.maxs.x) * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) * 0.5;
        let center_z = (aabb.mins.z + aabb.maxs.z) * 0.5;

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center_x, -center_y, -center_z)
    }
    
    /// Translates the CSG so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// let csg = CSG::cube(1.0, 1.0, 3.0, None).translate(2.0, 1.0, -2.0);
    /// let floated = csg.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    pub fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    /// Rotates the CSG by x_degrees, y_degrees, z_degrees
    pub fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> CSG<S> {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    /// Scales the CSG by scale_x, scale_y, scale_z
    pub fn scale(&self, sx: Real, sy: Real, sz: Real) -> CSG<S> {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }
    
    /// Reflect (mirror) this CSG about an arbitrary plane `plane`.
    ///
    /// The plane is specified by:
    ///   `plane.normal` = the plane’s normal vector (need not be unit),
    ///   `plane.w`      = the dot-product with that normal for points on the plane (offset).
    ///
    /// Returns a new CSG whose geometry is mirrored accordingly.
    pub fn mirror(&self, plane: Plane) -> Self {
        // Normal might not be unit, so compute its length:
        let len = plane.normal.norm();
        if len.abs() < EPSILON {
            // Degenerate plane? Just return clone (no transform)
            return self.clone();
        }

        // Unit normal:
        let n = plane.normal / len;
        // Adjusted offset = w / ||n||
        let w = plane.w / len;

        // Step 1) Translate so the plane crosses the origin
        // The plane’s offset vector from origin is (w * n).
        let offset = n * w;
        let t1 = Translation3::from(-offset);  // push the plane to origin
        let t1_mat = t1.to_homogeneous();

        // Step 2) Build the reflection matrix about a plane normal n at the origin
        //   R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Step 3) Translate back
        let t2 = Translation3::from(offset);   // pull the plane back out
        let t2_mat = t2.to_homogeneous();

        // Combine into a single 4×4
        let mirror_mat = t2_mat * reflect_4 * t1_mat;

        // Apply to all polygons
        self.transform(&mirror_mat)
    }

    /// Compute the convex hull of all vertices in this CSG.
    #[cfg(feature = "chull-io")]
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

        CSG::from_polygons(&polygons)
    }

    /// Compute the Minkowski sum: self ⊕ other
    ///
    /// Naive approach: Take every vertex in `self`, add it to every vertex in `other`,
    /// then compute the convex hull of all resulting points.
    #[cfg(feature = "chull-io")]
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

        CSG::from_polygons(&polygons)
    }

    /// Subdivide all polygons in this CSG 'levels' times, returning a new CSG.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: u32) -> CSG<S> {
        if levels == 0 {
            return self.clone();
        }
    
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        CLOSED,
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();
    
        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        CLOSED,
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();
    
        CSG::from_polygons(&new_polygons)
    }

    /// Renormalize all polygons in this CSG by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
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

        // let unioned_polygons = &self.flatten().polygons;
        let unioned_polygons = &self.polygons; // todo

        // Bottom polygons = original polygons
        // (assuming they are in some plane, e.g. XY). We just clone them.
        for poly in unioned_polygons {
            let mut bottom = poly.clone();
            let top = poly.translate_vector(direction);
            
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
                    CLOSED,
                    poly_bottom.metadata.clone(),
                );
                result_polygons.push(side_poly);
            }
        }

        // Combine into a new CSG
        CSG::from_polygons(&result_polygons)
    }
    
    /// Perform a linear extrusion along some axis, with optional twist, center, slices, scale, etc.
    ///
    /// # Parameters
    /// - `direction`: Direction vector for the extrusion.
    /// - `twist`: Total twist in degrees around the extrusion axis from bottom to top.
    /// - `segments`: Number of intermediate subdivisions.
    /// - `scale`: A uniform scale factor to apply at the top slice (bottom is scale=1.0).
    ///
    /// # Assumptions
    /// - This CSG is assumed to represent one or more 2D polygons lying in or near the XY plane.
    /// - The resulting shape is extruded *initially* along +Z, then finally rotated if `v != [0,0,1]`.
    ///
    /// # Returns
    /// A new 3D CSG.
    ///
    /// # Example
    /// ```
    /// let shape_2d = CSG::square(2.0, 2.0, None); // a 2D square in XY
    /// let extruded = shape_2d.linear_extrude(
    ///     direction = Vector3::new(0.0, 0.0, 10.0),
    ///     twist = 360.0,
    ///     segments = 32,
    ///     scale = 1.2,
    /// );
    /// ```
    pub fn linear_extrude(&self, direction: Vector3<Real>, twist: Real, segments: usize, scale: Real) -> CSG<S> {
        // calculate height from direction vector
        let height = direction.norm();
        let (z_start, z_end) = (0.0, height);

        // ----------------------------------------------
        // For each segment i in [0..segments], compute:
        //    fraction f = i/n
        //    z_i = z_start + f*(z_end - z_start)
        //    scale_i = 1 + (scale - 1)*f
        //    twist_i = twist * f
        //
        // Then transform (scale -> rotate -> translate)
        // the original 2D polygons.
        // ----------------------------------------------
        let mut segments_polygons = Vec::with_capacity(segments + 1);

        for i in 0..=segments {
            let f = (i as Real) / (segments as Real);
            let z_i = z_start + f * (z_end - z_start);
            let sc_i = 1.0 + f * (scale - 1.0);
            let twist_i_deg = twist * f;
            let twist_i_rad = twist_i_deg.to_radians();

            // Build a transform: scale in XY, then rotate around Z, then translate in Z.
            // (1) scale
            let mat_scale = Matrix4::new_nonuniform_scaling(&Vector3::new(sc_i, sc_i, 1.0));
            // (2) rotate around Z by twist_i
            let rot = Rotation3::from_axis_angle(
                &Vector3::z_axis(),
                twist_i_rad,
            )
            .to_homogeneous();
            // (3) translate by z_i in Z
            let tr = Translation3::new(0.0, 0.0, z_i).to_homogeneous();

            let segment_mat = tr * rot * mat_scale;

            // Transform *this* shape by segment_mat
            // However, we only want each polygon individually, not the entire 3D union yet.
            // So let's flatten first to unify all 2D polygons, OR just use `self.polygons`.
            let segment_csg = CSG::from_polygons(&self.polygons).transform(&segment_mat);

            // We'll store all polygons from segment_csg as a "segment".
            // But for extrude_between, we want exactly one "merged" polygon per shape
            // if we want side walls. If your shape can have multiple polygons, we must keep them all.
            // We'll do a flatten() if you want to unify them. In simpler usage,
            // we might just store them as is. We'll store them as "the polygons for segment i."
            segments_polygons.push(segment_csg.polygons);
        }

        // ----------------------------------------------
        // Connect consecutive segments to form side walls.
        // For each polygon in segment[i], connect to polygon in segment[i+1].
        //
        // The typical assumption is that the number of polygons & vertex count
        // matches from segment to segment. If the shape has multiple polygons or holes,
        // more advanced matching is needed. The simplest approach is if the shape is
        // a single polygon with the same vertex count each segment. 
        // If not, you may need "lofting" logic or repeated triangulation.
        // ----------------------------------------------
        let mut result_polygons = Vec::new();

        // We also add the bottom polygons (with flipped winding) and top polygons as-is:
        if !segments_polygons.is_empty() {
            // BOTTOM set => flip winding
            let bottom_set = &segments_polygons[0];
            for botpoly in bottom_set {
                let mut bp = botpoly.clone();
                bp.flip();
                result_polygons.push(bp);
            }
            // TOP set => keep as-is
            let top_set = &segments_polygons[segments_polygons.len() - 1];
            for toppoly in top_set {
                result_polygons.push(toppoly.clone());
            }
        }

        // Build side walls:
        // We do a naive 1:1 polygon pairing for each segment pair.
        for i in 0..segments {
            let bottom_polys = &segments_polygons[i];
            let top_polys = &segments_polygons[i + 1];
            if bottom_polys.len() != top_polys.len() {
                // In complex shapes, you might need a more robust approach 
                // or skip unmatched polygons. We'll do a direct zip here.
                // For now, only iterate over the min length.
            }
            let pair_count = bottom_polys.len().min(top_polys.len());
            for k in 0..pair_count {
                let poly_bot = &bottom_polys[k];
                let poly_top = &top_polys[k];

                // `extrude_between` must have same vertex count in matching order:
                if poly_bot.vertices.len() == poly_top.vertices.len() && poly_bot.vertices.len() >= 3
                {
                    let side_solid = CSG::extrude_between(poly_bot, poly_top, true);
                    // Gather polygons
                    for sp in side_solid.polygons {
                        result_polygons.push(sp);
                    }
                } else {
                    // Mismatch in vertex count. 
                    // Optionally do something else, or skip.
                }
            }
        }

        // Combine them all
        let mut extruded_csg = CSG::from_polygons(&result_polygons);

        // ----------------------------------------------
        // Finally, the `direction` to extrude along,
        // rotate the entire shape from +Z to that direction 
        // and scale its length to `height`.
        //
        // If direction = Some([vx, vy, vz]), we do:
        //   - first check direction’s length. If non-zero, we’ll rotate +Z to match its direction
        //   - scale the shape so that bounding box extends exactly `length(v)` in that direction.
        //
        // In OpenSCAD, `direction` is required to be "positive Z" direction, but we can be more general.
        // ---------------------------------------------- 
        if height > EPSILON {
            // 1) rotate from +Z to final_dir
            let zaxis = Vector3::z();
            if (zaxis - direction.normalize()).norm() > EPSILON {
                // do the rotation transform
                let axis = zaxis.cross(&direction).normalize();
                let angle = zaxis.dot(&direction).acos(); // angle between
                let rot_mat = Rotation3::from_axis_angle(
                    &Unit::new_normalize(axis),
                    angle
                ).to_homogeneous();
                extruded_csg = extruded_csg.transform(&rot_mat);
            }
        }
        extruded_csg
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

        // Gather polygons: bottom + top
        // (Depending on the orientation, you might want to flip one of them.)

        let mut polygons = vec![bottom_poly.clone(), top.clone()];

        // For each edge (i -> i+1) in bottom, connect to the corresponding edge in top.
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

        CSG::from_polygons(&polygons)
    }

    /// Rotate-extrude (revolve) this 2D shape around the Z-axis from 0..`angle_degs`
    /// by replicating the original polygon(s) at each step and calling `extrude_between`.
    /// Caps are added automatically if the revolve is partial (angle < 360°).
    pub fn rotate_extrude(&self, angle_degs: Real, segments: usize) -> CSG<S> {
        let angle_radians = angle_degs.to_radians();
        if segments < 2 {
            panic!("rotate_extrude requires at least 2 segments"); // todo: return error
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
                let rotated_poly = CSG::from_polygons(&[original_poly.clone()])
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
        CSG::from_polygons(&result_polygons) // todo: figure out why rotate_extrude results in inverted solids
    }
    
    /// Extrude an open or closed 2D polyline (from cavalier_contours) along `direction`,
    /// returning a 3D `CSG` containing the resulting side walls plus top/bottom if it’s closed.
    /// For open polylines, no “caps” are added unless you do so manually.
    pub fn extrude_polyline(poly: &Polyline<Real>, direction: Vector3<Real>, metadata: Option<S>) -> CSG<S> {
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
            let top = bottom.translate_vector(direction);

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
        CSG::from_polygons(&result_polygons)
    }
    
    /// Sweep a 2D shape `shape_2d` (in XY plane, normal=+Z) along a 2D path `path_2d` (also in XY).
    /// Produces a 3D CSG whose cross-sections match `shape_2d` at each vertex of `path_2d`.
    ///
    /// - If `path_2d` is open, the shape is capped at the start/end.
    /// - If `path_2d` is closed, we connect the last copy back to the first, forming a loop without caps.
    ///
    /// # Assumptions
    /// - `shape_2d` is a single Polygon in the XY plane. Its normal should be +Z.
    /// - `path_2d` is a single Polygon (or open polyline) also in XY. If `path_2d.open==false`, it’s closed.
    /// - Both polygons have enough vertices to be meaningful (e.g. 3+ for the shape if closed, 2+ for the path).
    ///
    /// # Returns
    /// A new 3D `CSG` that is the swept volume.
    pub fn sweep(shape_2d: &Polygon<S>, path_2d: &Polygon<S>) -> CSG<S> {
        // Gather the path’s vertices in XY
        if path_2d.vertices.len() < 2 {
            // Degenerate path => no sweep
            return CSG::new();
        }
        let path_is_closed = !path_2d.open;  // If false => open path, if true => closed path
    
        // Extract path points (x,y,0) from path_2d
        let mut path_points = Vec::with_capacity(path_2d.vertices.len());
        for v in &path_2d.vertices {
            // We only take X & Y; Z is typically 0 for a 2D path
            path_points.push(Point3::new(v.pos.x, v.pos.y, 0.0));
        }
    
        // Convert the shape_2d into a list of its vertices in local coords (usually in XY).
        // We assume shape_2d is a single polygon (can also handle multiple if needed).
        let shape_is_closed = !shape_2d.open && shape_2d.vertices.len() >= 3;
        let shape_count = shape_2d.vertices.len();
    
        // For each path vertex, compute the orientation that aligns +Z to the path tangent.
        // Then transform the shape’s 2D vertices into 3D “slice[i]”.
        let n_path = path_points.len();
        let mut slices: Vec<Vec<Point3<Real>>> = Vec::with_capacity(n_path);
    
        for i in 0..n_path {
            // The path tangent is p[i+1] - p[i] (or wrap if path is closed)
            // If open and i == n_path-1 => we’ll copy the tangent from the last segment
            let next_i = if i == n_path - 1 {
                if path_is_closed { 0 } else { i - 1 } // if closed, wrap, else reuse the previous
            } else {
                i + 1
            };
    
            let mut dir = path_points[next_i] - path_points[i];
            if dir.norm_squared() < EPSILON {
                // Degenerate segment => fallback to the previous direction or just use +Z
                dir = Vector3::z();
            } else {
                dir.normalize_mut();
            }
    
            // Build a rotation that maps +Z to `dir`.
            let rot = rotation_from_z_to_dir(dir);
    
            // Build a translation that puts shape origin at path_points[i]
            let trans = Translation3::from(path_points[i].coords);
    
            // Combined transform = T * R
            let mat = trans.to_homogeneous() * rot;
    
            // Apply that transform to all shape_2d vertices => slice[i]
            let mut slice_i = Vec::with_capacity(shape_count);
            for sv in &shape_2d.vertices {
                let local_pt = sv.pos;  // (x, y, z=0)
                let p4 = local_pt.to_homogeneous();
                let p4_trans = mat * p4;
                slice_i.push(Point3::from_homogeneous(p4_trans).unwrap());
            }
            slices.push(slice_i);
        }
    
        // Build polygons for the new 3D swept solid.
        // - (A) “Cap” polygons at start & end if path is open.
        // - (B) “Side wall” quads between slice[i] and slice[i+1].
        //
        // We’ll gather them all into a Vec<Polygon<S>>, then make a CSG.
    
        let mut all_polygons = Vec::new();
    
        // Caps if path is open
        //  We replicate the shape_2d as polygons at slice[0] and slice[n_path-1].
        //  We flip the first one so its normal faces outward. The last we keep as is.
        if !path_is_closed {
            // “Bottom” cap = slice[0], but we flip its winding so outward normal is “down” the path
            if shape_is_closed {
                let bottom_poly = polygon_from_slice(
                    &slices[0],
                    true, // flip
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(bottom_poly);
            }
            // “Top” cap = slice[n_path-1] (no flip)
            if shape_is_closed {
                let top_poly = polygon_from_slice(
                    &slices[n_path - 1],
                    false, // no flip
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(top_poly);
            }
        }
    
        // Side walls: For i in [0..n_path-1], or [0..n_path] if closed
        let end_index = if path_is_closed { n_path } else { n_path - 1 };
    
        for i in 0..end_index {
            let i_next = (i + 1) % n_path;  // wraps if closed
            let slice_i = &slices[i];
            let slice_next = &slices[i_next];
    
            // For each edge in the shape, connect vertices k..k+1
            // shape_2d may be open or closed. If open, we do shape_count-1 edges; if closed, shape_count edges.
            let edge_count = if shape_is_closed {
                shape_count  // because last edge wraps
            } else {
                shape_count - 1
            };
    
            for k in 0..edge_count {
                let k_next = (k + 1) % shape_count;
    
                let v_i_k     = slice_i[k];
                let v_i_knext = slice_i[k_next];
                let v_next_k     = slice_next[k];
                let v_next_knext = slice_next[k_next];
    
                // Build a quad polygon in CCW order for outward normal
                // or you might choose a different ordering.  Typically:
                //   [v_i_k, v_i_knext, v_next_knext, v_next_k]
                // forms an outward-facing side wall if the shape_2d was originally CCW in XY.
                let side_poly = Polygon::new(
                    vec![
                        Vertex::new(v_i_k,     Vector3::zeros()),
                        Vertex::new(v_i_knext, Vector3::zeros()),
                        Vertex::new(v_next_knext, Vector3::zeros()),
                        Vertex::new(v_next_k,     Vector3::zeros()),
                    ],
                    CLOSED,
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(side_poly);
            }
        }
    
        // Combine into a final CSG
        CSG::from_polygons(&all_polygons)
    }

    /// Given a list of Polygons that each represent a 2D open polyline (in XY, z=0),
    /// reconstruct a single 3D polyline by matching consecutive endpoints in 3D space.
    /// (If some polygons are closed, you can skip them or handle differently.)
    ///
    /// Returns a vector of 3D points (the polyline’s vertices). 
    /// If no matching is possible or the polygons are empty, returns an empty vector.
    pub fn reconstruct_polyline_3d(polylines: &[Polygon<S>]) -> Vec<Point3<Real>> {
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
                segment_points.push(Point3::new(v.x, v.y, 0.0));
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
            return Aabb::new(Point3::origin(), Point3::origin()); // or Aabb::new_invalid();
        }

        // Construct the parry AABB from points
        Aabb::from_points(&all_points)
    }

    /// Helper to collect all vertices from the CSG.
    #[cfg(not(feature = "parallel"))]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }
    
    /// Parallel helper to collect all vertices from the CSG.
    #[cfg(feature = "parallel")]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .par_iter()
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
                result_polygons.push(Polygon::from_polyline(&pline, poly.metadata.clone()));
            }
        }

        // Build a new CSG from those offset loops in XY:
        CSG::from_polygons(&result_polygons)
    }

    /// Flatten a `CSG` into the XY plane and union all polygons' outlines,
    /// returning a new `CSG` that may contain multiple polygons (loops) if disjoint.
    ///
    /// We skip "degenerate" loops whose area is near zero, both before
    /// and after performing the union. This helps avoid collinear or
    /// duplicate edges that can cause issues in `cavalier_contours`.
    pub fn flatten(&self) -> CSG<S> {    
        #[cfg(feature = "parallel")]
        let polys_2d: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .filter_map(|poly| {
                let cc = poly.to_polyline();
                cc.remove_redundant(EPSILON);
                let area = polyline_area(&cc).abs();
                if area > EPSILON { // keep it
                    Some(Polygon::from_polyline(&cc, poly.metadata.clone()))
                } else {
                    None
                }
            })
            .collect();
    
        #[cfg(not(feature = "parallel"))]
        let polys_2d: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .filter_map(|poly| {
                let cc = poly.to_polyline();
                cc.remove_redundant(EPSILON);
                let area = polyline_area(&cc).abs();
                if area > EPSILON { // keep it
                    Some(Polygon::from_polyline(&cc, poly.metadata.clone()))
                } else {
                    None
                }
            })
            .collect();
    
        // --- 2) Union them (still a single-thread union_all_2d call for now)
        let merged_2d = union_all_2d(&polys_2d);
    
        // --- 3) Convert merged_2d polygons (still in XY) to a new CSG
        CSG::from_polygons(&merged_2d)
    }

    /// Slice this solid by a given `plane`, returning a new `CSG` whose polygons
    /// are either:
    /// - The polygons that lie exactly in the slicing plane (coplanar), or
    /// - Polygons formed by the intersection edges (each a line, possibly open or closed).
    ///
    /// The returned `CSG` can contain:
    /// - **Closed polygons** that are coplanar,
    /// - **Open polygons** (poly-lines) if the plane cuts through edges,
    /// - Potentially **closed loops** if the intersection lines form a cycle.
    ///
    /// # Example
    /// ```
    /// let cylinder = CSG::cylinder(1.0, 2.0, 32, None);
    /// let plane_z0 = Plane { normal: Vector3::z(), w: 0.0 };
    /// let cross_section = cylinder.slice(plane_z0);
    /// // `cross_section` will contain:
    /// //   - Possibly an open or closed polygon(s) at z=0
    /// //   - Or empty if no intersection
    /// ```
    #[cfg(feature = "hashmap")]
    pub fn slice(&self, plane: Plane) -> CSG<S> {
        // Build a BSP from all of our polygons:
        let node = Node::new(&self.polygons.clone());

        // Ask the BSP for coplanar polygons + intersection edges:
        let (coplanar_polys, intersection_edges) = node.slice(&plane);

        // “Knit” those intersection edges into polylines. Each edge is [vA, vB].
        let polylines_3d = unify_intersection_edges(&intersection_edges);

        // Convert each polyline of vertices into a Polygon<S> with `open = true` or false (if loop).
        let mut result_polygons = Vec::new();

        // Add the coplanar polygons. We can re‐assign their plane to `plane` to ensure
        // they share the exact plane definition (in case of numeric drift).
        for mut p in coplanar_polys {
            p.plane = plane.clone(); // unify plane data
            result_polygons.push(p);
        }

        // Convert the “chains” or loops into open/closed polygons
        for chain in polylines_3d {
            let n = chain.len();
            if n < 2 {
                // degenerate
                continue;
            }

            // Check if last point matches first (within EPSILON) => closed loop
            let first = &chain[0];
            let last = &chain[n - 1];
            let is_closed = (first.pos.coords - last.pos.coords).norm() < EPSILON;

            let poly = Polygon {
                vertices: chain,
                open: !is_closed,
                metadata: None, // you could choose to store something else
                plane: plane.clone(),
            };

            result_polygons.push(poly);
        }

        // Build a new CSG
        CSG::from_polygons(&result_polygons)
    }

    /// Convert a `MeshText` (from meshtext) into a list of `Polygon` in the XY plane.
    /// - `scale` allows you to resize the glyph (e.g. matching a desired font size).
    /// - By default, the glyph’s normal is set to +Z.
    #[cfg(feature = "truetype-text")]
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
            let normal = Vector3::z();

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
    /// - `text`: the text to render
    /// - `font_data`: TTF font file bytes (e.g. `include_bytes!("../assets/FiraMono-Regular.ttf")`)
    /// - `size`: optional scaling factor (e.g., a rough "font size").
    ///
    /// **Note**: Limitations:
    ///   - does not handle kerning or multi-line text,
    ///   - simply advances the cursor by each glyph’s width,
    ///   - places all characters along the X axis.
    #[cfg(feature = "truetype-text")]
    pub fn text(text: &str, font_data: &[u8], size: Real, metadata: Option<S>) -> CSG<S> {
        let mut generator = MeshGenerator::new(font_data.to_vec());

        let mut all_polygons = Vec::new();
        let mut cursor_x: Real = 0.0;

        for ch in text.chars() {
            // Optionally skip control chars
            if ch.is_control() {
                continue;
            }
            // Generate glyph mesh
            let glyph_mesh: MeshText = match generator.generate_glyph(ch, true, None) {
                Ok(m) => m,
                Err(_) => {
                    // Missing glyph? Advance by some default
                    cursor_x += size;
                    continue;
                }
            };

            // Convert to polygons
            let glyph_polygons = Self::meshtext_to_polygons(&glyph_mesh, size, metadata.clone());

            // Translate polygons by (cursor_x, 0.0)
            let glyph_csg = CSG::from_polygons(&glyph_polygons).translate(cursor_x, 0.0, 0.0);
            // Accumulate
            all_polygons.extend(glyph_csg.polygons);

            // Advance cursor by the glyph’s bounding-box width
            let glyph_width = glyph_mesh.bbox.max.x - glyph_mesh.bbox.min.x;
            cursor_x += glyph_width as Real * size;
        }

        CSG::from_polygons(&all_polygons)
    }

    /// Triangulate each polygon in the CSG returning a CSG containing triangles
    pub fn triangulate(&self) -> CSG<S> {
        let mut triangles = Vec::new();
    
        for poly in &self.polygons {
            let tris = poly.triangulate();
            for triangle in tris {
                triangles.push(Polygon::new(triangle.to_vec(), CLOSED, poly.metadata.clone()));
            }
        }
        
        CSG::from_polygons(&triangles)
    }

    /// Creates 2D text in the XY plane using a **Hershey** font.
    ///
    /// Each glyph is rendered as one or more *open* polygons (strokes).  If you need 
    /// “thick” or “filled” text, you could **offset** or **extrude** these strokes 
    /// afterward.
    ///
    /// # Parameters
    ///
    /// - `text`: The text to render.
    /// - `font`: A Hershey `Font` reference (from your hershey crate code).
    /// - `size`: Optional scaling factor (defaults to 20.0 if `None`).
    /// - `metadata`: Shared metadata to attach to each stroke polygon.
    ///
    /// # Returns
    ///
    /// A new 2D `CSG<S>` in the XY plane, composed of multiple open polygons 
    /// (one for each stroke).
    ///
    /// # Example
    /// ```
    /// let font = hershey::fonts::GOTHIC_ENG_SANS; // or whichever Font you have
    /// let csg_text = CSG::from_hershey("HELLO", &font, Some(15.0), None);
    /// // Now you can extrude or union, etc.
    /// ```
    #[cfg(feature = "hershey-text")]
    pub fn from_hershey(
        text: &str,
        font: &Font,
        size: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let mut all_polygons = Vec::new();

        // Simple left-to-right “pen” position
        let mut cursor_x: Real = 0.0;

        for ch in text.chars() {
            // Optionally skip controls, spaces, or handle them differently
            if ch.is_control() {
                continue;
            }
            // Attempt to get the glyph
            match font.glyph(ch) {
                Ok(g) => {
                    // Convert the Hershey glyph’s line segments into open polylines/polygons
                    let glyph_width = (g.max_x - g.min_x) as Real;

                    let strokes = build_hershey_glyph_polygons(
                        &g,
                        size,
                        cursor_x,
                        0.0,          // y offset
                        metadata.clone()
                    );
                    all_polygons.extend(strokes);

                    // Advance cursor in x by the glyph width (scaled).
                    // You might add spacing, or shift by g.min_x, etc.
                    cursor_x += glyph_width * size * 0.8; 
                    // ^ adjust to taste or add extra letter spacing
                }
                Err(_) => {
                    // Missing glyph => skip or move cursor
                    cursor_x += 6.0 * size;
                }
            }
        }

        // Combine everything
        CSG::from_polygons(&all_polygons)
    }

    /// Re‐triangulate each polygon in this CSG using the `earclip` library.
    /// Returns a new CSG whose polygons are all triangles.
    #[cfg(feature = "earclip-io")]
    pub fn triangulate_earclip(&self) -> CSG<S> {
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

            // 4) Call earclip on that 2D outline. We assume no holes, so hole_indices = &[].
            //    earclip's signature is `earcut::<Real, usize>(data, hole_indices, dim)`
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
                new_poly.set_new_normal();

                new_polygons.push(new_poly);
            }
        }

        // Combine all newly formed triangles into a new CSG:
        CSG::from_polygons(&new_polygons)
    }

    /// Convert the polygons in this CSG to a Parry TriMesh.
    /// Useful for collision detection or physics simulations.
    pub fn to_trimesh(&self) -> SharedShape {
        // 1) Gather all the triangles from each polygon
        // 2) Build a TriMesh from points + triangle indices
        // 3) Wrap that in a SharedShape to be used in Rapier
        let tri_csg = self.triangulate();
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut index_offset = 0;

        for poly in &tri_csg.polygons {
            let a = poly.vertices[0].pos;
            let b = poly.vertices[1].pos;
            let c = poly.vertices[2].pos;
    
            vertices.push(a);
            vertices.push(b);
            vertices.push(c);
    
            indices.push([index_offset, index_offset + 1, index_offset + 2]);
            index_offset += 3;
        }


        // TriMesh::new(Vec<[Real; 3]>, Vec<[u32; 3]>)
        let trimesh = TriMesh::new(vertices, indices).unwrap(); // todo: handle error
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
    #[cfg(feature = "hashmap")]
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

        // Triangulate the whole shape once
        let tri_csg = self.triangulate();
        let mut edge_counts: HashMap<(String, String), u32> = HashMap::new();

        for poly in &tri_csg.polygons {
            // Each tri is 3 vertices: [v0, v1, v2]
            // We'll look at edges (0->1, 1->2, 2->0).
            for &(i0, i1) in &[(0, 1), (1, 2), (2, 0)] {
                let p0 = &poly.vertices[i0].pos;
                let p1 = &poly.vertices[i1].pos;

                // Order them so (p0, p1) and (p1, p0) become the same key
                let (a_key, b_key) = if approx_lt(&p0, &p1) {
                    (point_key(&p0), point_key(&p1))
                } else {
                    (point_key(&p1), point_key(&p0))
                };

                *edge_counts.entry((a_key, b_key)).or_insert(0) += 1;
            }
        }

        // For a perfectly closed manifold surface (with no boundary),
        // each edge should appear exactly 2 times.
        edge_counts.values().all(|&count| count == 2)
    }
    
    /// Generate a Triply Periodic Minimal Surface (Gyroid) inside the volume of `self`.
    ///
    /// # Parameters
    ///
    /// - `resolution`: how many sampling steps along each axis (larger = finer mesh).
    /// - `period`: controls the spatial period of the gyroid function.  Larger = repeats more slowly.
    /// - `iso_value`: the implicit surface is f(x,y,z) = iso_value. Usually 0.0 for a "standard" gyroid.
    ///
    /// # Returns
    ///
    /// A new `CSG` whose polygons approximate the gyroid surface *inside* the volume of `self`.
    ///
    /// # Example
    /// ```
    /// // Suppose `shape` is a CSG volume, e.g. a box or sphere.
    /// let gyroid_csg = shape.tpms_gyroid(50, 2.0, 0.0);
    /// ```
    pub fn gyroid(&self, resolution: usize, period: Real, iso_value: Real) -> CSG<S> {
        // Get bounding box of `self`.
        let aabb = self.bounding_box();

        // Extract bounding box corners
        let min_pt = aabb.mins;
        let max_pt = aabb.maxs;

        // Discretize bounding box into a 3D grid of size `resolution × resolution × resolution`.
        // For each cell in the grid, we'll sample the Gyroid function at its corners and do
        // a simple "marching cubes" step.
        if resolution < 2 {
            // degenerate sampling => no real geometry
            return CSG::new();
        }

        // Cell size in each dimension
        let nx = resolution;
        let ny = resolution;
        let nz = resolution;

        let dx = (max_pt.x - min_pt.x) / (nx - 1) as Real;
        let dy = (max_pt.y - min_pt.y) / (ny - 1) as Real;
        let dz = (max_pt.z - min_pt.z) / (nz - 1) as Real;

        // A small helper to evaluate the gyroid function at a given (x, y, z).
        fn gyroid_f(x: Real, y: Real, z: Real, period: Real) -> Real {
            // If you prefer the standard "period ~ 2π" style, adjust accordingly.
            // Here we divide the coordinates by `period` to set the "wavelength".
            (x / period).sin() * (y / period).cos()
                + (y / period).sin() * (z / period).cos()
                + (z / period).sin() * (x / period).cos()
        }
        
        // A small helper to evaluate the Schwarz-P function at a given (x, y, z).
        fn schwarz_p_f(x: Real, y: Real, z: Real, period: Real) -> Real {
            let px = x / period;
            let py = y / period;
            let pz = z / period;
            (px).cos() + (py).cos() + (pz).cos()
        }


        // We’ll store sampled values in a 3D array, [nx * ny * nz].
        let mut grid_vals = vec![0.0; nx * ny * nz];

        // A small function to convert (i, j, k) => index in `grid_vals`.
        let idx = |i: usize, j: usize, k: usize| -> usize {
            (k * ny + j) * nx + i
        };

        // Evaluate the gyroid function at each grid point
        for k in 0..nz {
            let zf = min_pt.z + (k as Real) * dz;
            for j in 0..ny {
                let yf = min_pt.y + (j as Real) * dy;
                for i in 0..nx {
                    let xf = min_pt.x + (i as Real) * dx;
                    let val = gyroid_f(xf, yf, zf, period);
                    grid_vals[idx(i, j, k)] = val;
                }
            }
        }

        // Marching Cubes (naïve version).
        //
        // We'll do a simple variant that looks at each cube of 8 corner samples, checks
        // which corners are above/below the iso_value, and linearly interpolates edges.
        // For a full version with all 256 cases, see e.g.:
        //   - the "marching_cubes" crate, or
        //   - the classic lookup‐table approach from Paul Bourke / NVIDIA.
        //
        // Here, we’ll implement just enough to produce a surface, using the standard
        // approach in about ~8 steps.  For brevity, we skip the full 256-case edge table
        // and do a simpler approach that might produce more triangles than typical.

        let mut triangles = Vec::new(); // will store [ (p1, p2, p3), ... ]

        // Helper to get the (x,y,z) of a grid corner.
        let corner_xyz = |i: usize, j: usize, k: usize| -> Point3<Real> {
            Point3::new(
                min_pt.x + (i as Real) * dx,
                min_pt.y + (j as Real) * dy,
                min_pt.z + (k as Real) * dz,
            )
        };

        // Linear interpolate the position along an edge where the function crosses iso_value.
        fn interpolate_iso(
            p1: Point3<Real>,
            p2: Point3<Real>,
            v1: Real,
            v2: Real,
            iso: Real,
        ) -> Point3<Real> {
            if (v2 - v1).abs() < 1e-12 {
                return p1; // fallback
            }
            let t = (iso - v1) / (v2 - v1);
            Point3::new(
                p1.x + t * (p2.x - p1.x),
                p1.y + t * (p2.y - p1.y),
                p1.z + t * (p2.z - p1.z),
            )
        }

        // We'll iterate through each cell in x,y,z from [0..nx-1], [0..ny-1], [0..nz-1]
        // so that (i+1, j+1, k+1) is in range.  Each cell has 8 corners:
        //   c0 = (i, j, k)
        //   c1 = (i+1, j, k)
        //   c2 = (i+1, j, k+1)
        //   c3 = (i,   j, k+1)
        //   c4 = (i, j+1, k)
        //   c5 = (i+1, j+1, k)
        //   c6 = (i+1, j+1, k+1)
        //   c7 = (i,   j+1, k+1)
        //
        // For each cell, we gather which corners are above/below iso_value, and build triangles.

        for k in 0..(nz - 1) {
            for j in 0..(ny - 1) {
                for i in 0..(nx - 1) {
                    // The indices of the 8 corners:
                    let c_id = [
                        idx(i, j, k),
                        idx(i + 1, j, k),
                        idx(i + 1, j, k + 1),
                        idx(i, j, k + 1),
                        idx(i, j + 1, k),
                        idx(i + 1, j + 1, k),
                        idx(i + 1, j + 1, k + 1),
                        idx(i, j + 1, k + 1),
                    ];

                    let c_pos = [
                        corner_xyz(i, j, k),
                        corner_xyz(i + 1, j, k),
                        corner_xyz(i + 1, j, k + 1),
                        corner_xyz(i, j, k + 1),
                        corner_xyz(i, j + 1, k),
                        corner_xyz(i + 1, j + 1, k),
                        corner_xyz(i + 1, j + 1, k + 1),
                        corner_xyz(i, j + 1, k + 1),
                    ];

                    let c_val = [
                        grid_vals[c_id[0]],
                        grid_vals[c_id[1]],
                        grid_vals[c_id[2]],
                        grid_vals[c_id[3]],
                        grid_vals[c_id[4]],
                        grid_vals[c_id[5]],
                        grid_vals[c_id[6]],
                        grid_vals[c_id[7]],
                    ];

                    // Determine which corners are inside vs. outside:
                    // inside = c_val < iso_value
                    let mut cube_index = 0u8;
                    for (bit, &val) in c_val.iter().enumerate() {
                        if val < iso_value {
                            // We consider "inside" => set bit
                            cube_index |= 1 << bit;
                        }
                    }
                    // If all corners are inside or all corners are outside, skip
                    if cube_index == 0 || cube_index == 0xFF {
                        continue;
                    }

                    // We do a simplified approach: sample each of the 12 possible edges,
                    // see if the iso‐crossing occurs there, and if so, compute that point.
                    let mut edge_points = [None; 12];

                    // Helper macro to handle an edge from corner A to corner B, with indices eA, eB
                    macro_rules! check_edge {
                        ($edge_idx:expr, $cA:expr, $cB:expr) => {
                            let mask_a = 1 << $cA;
                            let mask_b = 1 << $cB;
                            // If corners differ across iso => there's an intersection on this edge
                            let inside_a = (cube_index & mask_a) != 0;
                            let inside_b = (cube_index & mask_b) != 0;
                            if inside_a != inside_b {
                                // Interpolate
                                edge_points[$edge_idx] = Some(interpolate_iso(
                                    c_pos[$cA],
                                    c_pos[$cB],
                                    c_val[$cA],
                                    c_val[$cB],
                                    iso_value,
                                ));
                            }
                        };
                    }

                    // The classic marching‐cubes edges:
                    check_edge!(0, 0, 1);
                    check_edge!(1, 1, 2);
                    check_edge!(2, 2, 3);
                    check_edge!(3, 3, 0);
                    check_edge!(4, 4, 5);
                    check_edge!(5, 5, 6);
                    check_edge!(6, 6, 7);
                    check_edge!(7, 7, 4);
                    check_edge!(8, 0, 4);
                    check_edge!(9, 1, 5);
                    check_edge!(10, 2, 6);
                    check_edge!(11, 3, 7);

                    // Now collect the intersection points in a small list (some MC code uses a lookup table).
                    // We’ll do a simple approach: gather all edge_points that are Some(..) into a polygon
                    // fan (which can cause more triangles than needed).
                    let verts: Vec<Point3<Real>> = edge_points
                        .iter()
                        .filter_map(|&pt| pt)
                        .collect();

                    // Triangulate them (fan from verts[0]) if we have >=3
                    if verts.len() >= 3 {
                        let anchor = verts[0];
                        for t in 1..(verts.len() - 1) {
                            triangles.push((anchor, verts[t], verts[t + 1]));
                        }
                    }
                }
            }
        }

        // Convert our triangle soup into a new CSG
        let mut surf_polygons = Vec::with_capacity(triangles.len());
        for (a, b, c) in triangles {
            // Create a 3‐vertex polygon
            let mut poly = Polygon::new(
                vec![
                    Vertex::new(a, Vector3::zeros()),
                    Vertex::new(b, Vector3::zeros()),
                    Vertex::new(c, Vector3::zeros()),
                ],
                true,
                None,
            );
            // Recompute plane & normals
            poly.set_new_normal();
            surf_polygons.push(poly);
        }
        let gyroid_surf = CSG::from_polygons(&surf_polygons);

        // Intersect with `self` to keep only the portion of the gyroid inside this volume.
        let clipped = gyroid_surf.intersection(self);

        clipped
    }

    /// **Creates a CSG from a list of metaballs** by sampling a 3D grid and using marching cubes.
    /// 
    /// - `balls`: slice of metaball definitions (center + radius).
    /// - `resolution`: (nx, ny, nz) defines how many steps along x, y, z.
    /// - `iso_value`: threshold at which the isosurface is extracted.
    /// - `padding`: extra margin around the bounding region (e.g. 0.5) so the surface doesn’t get truncated.
    #[cfg(feature = "metaballs")]    
    pub fn metaballs(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
    ) -> CSG<S> {
        if balls.is_empty() {
            return CSG::new();
        }
    
        // Determine bounding box of all metaballs (plus padding).
        let mut min_pt = Point3::new(Real::MAX, Real::MAX, Real::MAX);
        let mut max_pt = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);
    
        for mb in balls {
            let c = &mb.center;
            let r = mb.radius + padding;
    
            if c.x - r < min_pt.x {
                min_pt.x = c.x - r;
            }
            if c.y - r < min_pt.y {
                min_pt.y = c.y - r;
            }
            if c.z - r < min_pt.z {
                min_pt.z = c.z - r;
            }
    
            if c.x + r > max_pt.x {
                max_pt.x = c.x + r;
            }
            if c.y + r > max_pt.y {
                max_pt.y = c.y + r;
            }
            if c.z + r > max_pt.z {
                max_pt.z = c.z + r;
            }
        }
    
        // Resolution for X, Y, Z
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
    
        // Spacing in each axis
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);
    
        // Create and fill the scalar-field array with "field_value - iso_value"
        // so that the isosurface will be at 0.
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0 as f32; array_size];
    
        let index_3d = |ix: u32, iy: u32, iz: u32| -> usize {
            (iz * ny + iy) as usize * (nx as usize) + ix as usize
        };
    
        for iz in 0..nz {
            let zf = min_pt.z + (iz as Real) * dz;
            for iy in 0..ny {
                let yf = min_pt.y + (iy as Real) * dy;
                for ix in 0..nx {
                    let xf = min_pt.x + (ix as Real) * dx;
                    let p = Point3::new(xf, yf, zf);
    
                    let val = scalar_field_metaballs(balls, &p) - iso_value;
                    field_values[index_3d(ix, iy, iz)] = val as f32;
                }
            }
        }
    
        // Use fast-surface-nets to extract a mesh from this 3D scalar field.
        // We'll define a shape type for ndshape:
        #[allow(non_snake_case)]
        #[derive(Clone, Copy)]
        struct GridShape {
            nx: u32,
            ny: u32,
            nz: u32,
        }
        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;
            #[inline]
            fn as_array(&self) -> [Self::Coord; 3] {
                [self.nx, self.ny, self.nz]
            }
        
            fn size(&self) -> Self::Coord {
                self.nx * self.ny * self.nz
            }
        
            fn usize(&self) -> usize {
                (self.nx * self.ny * self.nz) as usize
            }
        
            fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
                let [x, y, z] = coords;
                (z * self.ny + y) * self.nx + x
            }
        
            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % (self.nx);
                let yz = i / (self.nx);
                let y = yz % (self.ny);
                let z = yz / (self.ny);
                [x, y, z]
            }
        }
    
        let shape = GridShape { nx, ny, nz };
    
        // We'll collect the output into a SurfaceNetsBuffer
        let mut sn_buffer = SurfaceNetsBuffer::default();
    
        // The region we pass to surface_nets is the entire 3D range [0..nx, 0..ny, 0..nz]
        // minus 1 in each dimension to avoid indexing past the boundary:
        let (max_x, max_y, max_z) = (nx - 1, ny - 1, nz - 1);
    
        surface_nets(
            &field_values,      // SDF array
            &shape,             // custom shape
            [0, 0, 0],          // minimum corner in lattice coords
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );
    
        // Convert the resulting surface net indices/positions into Polygons
        // for the csgrs data structures.
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);
    
        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;
    
            let p0_index = sn_buffer.positions[i0];
            let p1_index = sn_buffer.positions[i1];
            let p2_index = sn_buffer.positions[i2];
            
            // Convert from index space to real (world) space:
            let p0_real = Point3::new(
                min_pt.x + p0_index[0] as Real * dx,
                min_pt.y + p0_index[1] as Real * dy,
                min_pt.z + p0_index[2] as Real * dz
            );
            
            let p1_real = Point3::new(
                min_pt.x + p1_index[0] as Real * dx,
                min_pt.y + p1_index[1] as Real * dy,
                min_pt.z + p1_index[2] as Real * dz
            );
            
            let p2_real = Point3::new(
                min_pt.x + p2_index[0] as Real * dx,
                min_pt.y + p2_index[1] as Real * dy,
                min_pt.z + p2_index[2] as Real * dz
            );
            
            // Likewise for the normals if you want them in true world space. 
            // Usually you'd need to do an inverse-transpose transform if your 
            // scale is non-uniform. For uniform voxels, scaling is simpler:
            
            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];
            
            // Construct your vertices:
            let v0 = Vertex::new(p0_real, Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real));
            let v1 = Vertex::new(p1_real, Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real));
            let v2 = Vertex::new(p2_real, Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real));
    
            // Each tri is turned into a Polygon with 3 vertices
            let poly = Polygon::new(vec![v0, v2, v1], CLOSED, None);
            triangles.push(poly);
        }
    
        // Build and return a CSG from these polygons
        CSG::from_polygons(&triangles)
    }
    
    /// Return a CSG created by meshing a signed distance field within a bounding box
    ///
    ///    // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
    ///    let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;
    ///
    ///    let resolution = (60, 60, 60);
    ///    let min_pt = Point3::new(-2.0, -2.0, -2.0);
    ///    let max_pt = Point3::new( 2.0,  2.0,  2.0);
    ///    let iso_value = 0.0; // Typically zero for SDF-based surfaces
    ///
    ///    let csg_shape = CSG::from_sdf(my_sdf, resolution, min_pt, max_pt, iso_value);
    ///
    ///    // Now `csg_shape` is your polygon mesh as a CSG you can union, subtract, or export:
    ///    let _ = std::fs::write("stl/sdf_sphere.stl", csg_shape.to_stl_binary("sdf_sphere").unwrap());
    #[cfg(feature = "sdf")]
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
    ) -> CSG<S>
    where
        // F is a closure or function that takes a 3D point and returns the signed distance.
        // Must be `Sync`/`Send` if you want to parallelize the sampling.
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};
        use crate::float_types::Real;
    
        // Early return if resolution is degenerate
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
    
        // Determine grid spacing based on bounding box and resolution
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);
    
        // Allocate storage for field values:
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0_f32; array_size];
    
        // Helper to map (ix, iy, iz) to 1D index:
        let index_3d = |ix: u32, iy: u32, iz: u32| -> usize {
            (iz * ny + iy) as usize * (nx as usize) + ix as usize
        };
    
        // Sample the SDF at each grid cell:
        // Note that for an "isosurface" at iso_value, we store (sdf_value - iso_value)
        // so that `surface_nets` zero-crossing aligns with iso_value.
        for iz in 0..nz {
            let zf = min_pt.z + (iz as Real) * dz;
            for iy in 0..ny {
                let yf = min_pt.y + (iy as Real) * dy;
                for ix in 0..nx {
                    let xf = min_pt.x + (ix as Real) * dx;
                    let p = Point3::new(xf, yf, zf);
                    let sdf_val = sdf(&p);
                    // Shift by iso_value so that the zero-level is the surface we want:
                    field_values[index_3d(ix, iy, iz)] = (sdf_val - iso_value) as f32;
                }
            }
        }
    
        // The shape describing our discrete grid for Surface Nets:
        #[derive(Clone, Copy)]
        struct GridShape {
            nx: u32,
            ny: u32,
            nz: u32,
        }
    
        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;
    
            #[inline]
            fn as_array(&self) -> [Self::Coord; 3] {
                [self.nx, self.ny, self.nz]
            }
    
            fn size(&self) -> Self::Coord {
                self.nx * self.ny * self.nz
            }
    
            fn usize(&self) -> usize {
                (self.nx * self.ny * self.nz) as usize
            }
    
            fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
                let [x, y, z] = coords;
                (z * self.ny + y) * self.nx + x
            }
    
            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % self.nx;
                let yz = i / self.nx;
                let y = yz % self.ny;
                let z = yz / self.ny;
                [x, y, z]
            }
        }
    
        let shape = GridShape { nx, ny, nz };
    
        // `SurfaceNetsBuffer` collects the positions, normals, and triangle indices
        let mut sn_buffer = SurfaceNetsBuffer::default();
    
        // The max valid coordinate in each dimension
        let max_x = nx - 1;
        let max_y = ny - 1;
        let max_z = nz - 1;
    
        // Run surface nets
        surface_nets(
            &field_values,
            &shape,
            [0, 0, 0],
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );
    
        // Convert the resulting triangles into CSG polygons
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);
    
        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;
    
            let p0i = sn_buffer.positions[i0];
            let p1i = sn_buffer.positions[i1];
            let p2i = sn_buffer.positions[i2];
    
            // Convert from [u32; 3] to real coordinates:
            let p0 = Point3::new(
                min_pt.x + p0i[0] as Real * dx,
                min_pt.y + p0i[1] as Real * dy,
                min_pt.z + p0i[2] as Real * dz,
            );
            let p1 = Point3::new(
                min_pt.x + p1i[0] as Real * dx,
                min_pt.y + p1i[1] as Real * dy,
                min_pt.z + p1i[2] as Real * dz,
            );
            let p2 = Point3::new(
                min_pt.x + p2i[0] as Real * dx,
                min_pt.y + p2i[1] as Real * dy,
                min_pt.z + p2i[2] as Real * dz,
            );
    
            // Retrieve precomputed normal from Surface Nets:
            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];
    
            let v0 = Vertex::new(
                p0,
                Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real),
            );
            let v1 = Vertex::new(
                p1,
                Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real),
            );
            let v2 = Vertex::new(
                p2,
                Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real),
            );
    
            // Note: reverse v1, v2 if you need to fix winding
            let poly = Polygon::new(vec![v0, v1, v2], CLOSED, None);
            triangles.push(poly);
        }
    
        // Return as a CSG
        CSG::from_polygons(&triangles)
    }

    /// Builds a new CSG from the “on” pixels of a grayscale image,
    /// tracing connected outlines (and holes) via the `contour_tracing` code.
    ///
    /// - `img`: a reference to a GrayImage
    /// - `threshold`: pixels >= threshold are treated as "on/foreground", else off/background
    /// - `closepaths`: if true, each traced path is closed with 'Z' in the SVG path commands
    /// - `metadata`: optional metadata to attach to the resulting polygons
    ///
    /// # Returns
    /// A 2D shape in the XY plane (z=0) representing all traced contours. Each contour
    /// becomes a polygon. The polygons are *not* automatically unioned; they are simply
    /// collected in one `CSG`.
    ///
    /// # Example
    /// ```no_run
    /// # use csgrs::csg::CSG;
    /// # use image::{GrayImage, Luma};
    /// # fn main() {
    /// let img: GrayImage = image::open("my_binary.png").unwrap().to_luma8();
    /// let csg2d = CSG::from_image(&img, 128, true, None);
    /// // optionally extrude it:
    /// let shape3d = csg2d.extrude(5.0);
    /// # }
    /// ```
    #[cfg(feature = "image-io")]
    pub fn from_image(
        img: &GrayImage,
        threshold: u8,
        closepaths: bool,
        metadata: Option<S>,
    ) -> Self {        
        // Convert the image into a 2D array of bits for the contour_tracing::array::bits_to_paths function.
        // We treat pixels >= threshold as 1, else 0.
        let width = img.width() as usize;
        let height = img.height() as usize;
        let mut bits = Vec::with_capacity(height);
        for y in 0..height {
            let mut row = Vec::with_capacity(width);
            for x in 0..width {
                let px_val = img.get_pixel(x as u32, y as u32)[0];
                if px_val >= threshold {
                    row.push(1);
                } else {
                    row.push(0);
                }
            }
            bits.push(row);
        }

        // Use contour_tracing::array::bits_to_paths to get a single SVG path string
        // containing multiple “move” commands for each outline/hole.
        let svg_path = contour_tracing::array::bits_to_paths(bits, closepaths);
        // This might look like: "M1 0H4V1H1M6 0H11V5H6 ..." etc.

        // Parse the path string into one or more polylines. Each polyline
        // starts with an 'M x y' and then “H x” or “V y” commands until the next 'M' or end.
        let polylines = Self::parse_svg_path_into_polylines(&svg_path);

        // Convert each polyline into a Polygon in the XY plane at z=0,
        // storing them in a `Vec<Polygon<S>>`.
        let mut all_polygons = Vec::new();
        for pl in polylines {
            if pl.len() < 2 {
                continue;
            }
            // Build vertices with normal = +Z
            let normal = Vector3::z();
            let open = false; // We usually consider each contour closed (since it’s a shape outline).
            let mut verts = Vec::with_capacity(pl.len());
            for &(x, y) in &pl {
                verts.push(Vertex::new(
                    Point3::new(x as Real, y as Real, 0.0),
                    normal,
                ));
            }
            // If the path was not closed and we used closepaths == true, the path commands
            // do a 'Z', but we might need to ensure the first/last are the same. Up to you.
            // For safety, we can ensure a closed ring if distinct:
            if (verts.first().unwrap().pos - verts.last().unwrap().pos).norm() > EPSILON {
                // close it
                verts.push(verts.first().unwrap().clone());
            }
            let poly = Polygon::new(verts, open, metadata.clone());
            all_polygons.push(poly);
        }

        // Build a CSG from those polygons
        CSG::from_polygons(&all_polygons)
    }

    /// Internal helper to parse a minimal subset of SVG path commands:
    /// - M x y   => move absolute
    /// - H x     => horizontal line
    /// - V y     => vertical line
    /// - Z       => close path
    ///
    /// Returns a `Vec` of polylines, each polyline is a list of `(x, y)` in integer coords.
    fn parse_svg_path_into_polylines(path_str: &str) -> Vec<Vec<(f32, f32)>> {
        let mut polylines = Vec::new();
        let mut current_poly = Vec::new();

        let mut current_x = 0.0_f32;
        let mut current_y = 0.0_f32;
        let mut chars = path_str.trim().chars().peekable();

        // We'll read tokens that could be:
        //  - a letter (M/H/V/Z)
        //  - a number (which may be float, but from bits_to_paths it’s all integer steps)
        //  - whitespace or other
        //
        // This small scanner accumulates tokens so we can parse them easily.
        fn read_number<I: Iterator<Item = char>>(iter: &mut std::iter::Peekable<I>) -> Option<f32> {
            let mut buf = String::new();
            // skip leading spaces
            while let Some(&ch) = iter.peek() {
                if ch.is_whitespace() {
                    iter.next();
                } else {
                    break;
                }
            }
            // read sign or digits
            while let Some(&ch) = iter.peek() {
                if ch.is_ascii_digit() || ch == '.' || ch == '-' {
                    buf.push(ch);
                    iter.next();
                } else {
                    break;
                }
            }
            if buf.is_empty() {
                return None;
            }
            // parse as f32
            buf.parse().ok()
        }

        while let Some(ch) = chars.next() {
            match ch {
                'M' | 'm' => {
                    // Move command => read 2 numbers for x,y
                    if !current_poly.is_empty() {
                        // start a new polyline
                        polylines.push(current_poly);
                        current_poly = Vec::new();
                    }
                    let nx = read_number(&mut chars).unwrap_or(current_x);
                    let ny = read_number(&mut chars).unwrap_or(current_y);
                    current_x = nx;
                    current_y = ny;
                    current_poly.push((current_x, current_y));
                }
                'H' | 'h' => {
                    // Horizontal line => read 1 number for x
                    let nx = read_number(&mut chars).unwrap_or(current_x);
                    current_x = nx;
                    current_poly.push((current_x, current_y));
                }
                'V' | 'v' => {
                    // Vertical line => read 1 number for y
                    let ny = read_number(&mut chars).unwrap_or(current_y);
                    current_y = ny;
                    current_poly.push((current_x, current_y));
                }
                'Z' | 'z' => {
                    // Close path
                    // We'll let the calling code decide if it must explicitly connect back.
                    // For now, we just note that this polyline ends.
                    if !current_poly.is_empty() {
                        polylines.push(std::mem::take(&mut current_poly));
                    }
                }
                // Possibly other characters (digits) or spaces:
                c if c.is_whitespace() || c.is_ascii_digit() || c == '-' => {
                    // Could be an inlined number if the path commands had no letter.
                    // Typically bits_to_paths always has M/H/V so we might ignore it or handle gracefully.
                    // If you want robust parsing, you can push this char back and try read_number.
                }
                _ => {
                    // ignoring other
                }
            }
        }

        // If the last polyline is non‐empty, push it.
        if !current_poly.is_empty() {
            polylines.push(current_poly);
        }

        polylines
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
    #[cfg(feature = "stl-io")]
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
    #[cfg(feature = "stl-io")]
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

        Ok(CSG::from_polygons(&polygons))
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
    #[cfg(feature = "dxf-io")]
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
                                Vector3::z(), // Assuming flat in XY
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
                    let normal = Vector3::z(); // Assuming circle lies in XY plane

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

        Ok(CSG::from_polygons(&polygons))
    }

    /// Export the CSG object to DXF format.
    ///
    /// # Returns
    ///
    /// A `Result` containing the DXF file as a byte vector or an error if exporting fails.
    #[cfg(feature = "dxf-io")]
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

/// Build a rotation matrix that maps the global +Z axis to the specified `dir` in 3D.
///
/// - If `dir` is near zero length or near +Z, we produce the identity.
/// - Otherwise, we use an axis‐angle rotation from +Z to `dir`.
fn rotation_from_z_to_dir(dir: Vector3<Real>) -> Matrix4<Real> {
    // We'll rotate the z-axis (0,0,1) onto `dir`.
    let z = Vector3::z();
    let dot = z.dot(&dir);
    // If dir is basically the same as z, no rotation needed
    if (dot - 1.0).abs() < EPSILON {
        return Matrix4::identity();
    }
    // If dir is basically opposite z
    if (dot + 1.0).abs() < EPSILON {
        // 180 deg around X or Y axis
        let rot180 = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::x()), PI);
        return rot180.to_homogeneous();
    }
    // Otherwise, general axis = z × dir
    let axis = z.cross(&dir).normalize();
    let angle = z.dot(&dir).acos();
    let rot = Rotation3::from_axis_angle(&Unit::new_unchecked(axis), angle);
    rot.to_homogeneous()
}

/// Helper to build a single Polygon from a “slice” of 3D points.
///
/// If `flip_winding` is true, we reverse the vertex order (so the polygon’s normal flips).
fn polygon_from_slice<S: Clone + Send + Sync>(
    slice_pts: &[Point3<Real>],
    flip_winding: bool,
    metadata: Option<S>,
) -> Polygon<S> {
    if slice_pts.len() < 3 {
        // degenerate polygon
        return Polygon::new(vec![], OPEN, metadata);
    }
    // Build the vertex list
    let mut verts: Vec<Vertex> = slice_pts
        .iter()
        .map(|p| Vertex::new(*p, Vector3::zeros()))
        .collect();

    if flip_winding {
        verts.reverse();
        for v in &mut verts {
            v.flip();
        }
    }

    let mut poly = Polygon::new(verts, CLOSED, metadata);
    poly.set_new_normal(); // Recompute its plane & normal for consistency
    poly
}

/// Helper for building open polygons from a single Hershey `Glyph`.
#[cfg(feature = "hershey-text")]
fn build_hershey_glyph_polygons<S: Clone + Send + Sync>(
    glyph: &HersheyGlyph,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
    metadata: Option<S>,
) -> Vec<Polygon<S>> {
    let mut polygons = Vec::new();

    // We will collect line segments in a “current” Polyline 
    // each time we see `Vector::MoveTo` => start a new stroke.
    let mut current_pline = Polyline::new();
    let mut _pen_down = false;

    for vector_cmd in &glyph.vectors {
        match vector_cmd {
            HersheyVector::MoveTo { x, y } => {
                // The Hershey code sets pen-up or "hovering" here: start a new polyline
                // if the old polyline has 2+ vertices, push it into polygons
                if current_pline.vertex_count() >= 2 {
                    // Convert the existing stroke into an open polygon
                    let stroke_poly = Polygon::from_polyline(&current_pline, metadata.clone());
                    polygons.push(stroke_poly);
                }
                // Begin a fresh new stroke
                current_pline = Polyline::new();
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_pline.add(px, py, 0.0);

                _pen_down = false;
            }
            HersheyVector::LineTo { x, y } => {
                // If pen was up, effectively we’re continuing from last point
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_pline.add(px, py, 0.0);

                _pen_down = true;
            }
        }
    }

    // If our final polyline has >=2 vertices, store it
    if current_pline.vertex_count() >= 2 {
        let stroke_poly = Polygon::from_polyline(&current_pline, metadata.clone());
        polygons.push(stroke_poly);
    }

    polygons
}

// 1) Build a small helper for hashing endpoints:
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct EndKey(i64, i64, i64);

/// Round a floating to a grid for hashing
fn quantize(x: Real) -> i64 {
    // For example, scale by 1e8
    (x * 1e8).round() as i64
}

/// Convert a Vertex’s position to an EndKey
fn make_key(pos: &Point3<Real>) -> EndKey {
    EndKey(quantize(pos.x), quantize(pos.y), quantize(pos.z))
}

/// Take a list of intersection edges `[Vertex;2]` and merge them into polylines.
/// Each edge is a line segment between two 3D points.  We want to “knit” them together by
/// matching endpoints that lie within EPSILON of each other, forming either open or closed chains.
///
/// This returns a `Vec` of polylines, where each polyline is a `Vec<Vertex>`.
#[cfg(feature = "hashmap")]
fn unify_intersection_edges(edges: &[[Vertex; 2]]) -> Vec<Vec<Vertex>> {
    // We will store adjacency by a “key” that identifies an endpoint up to EPSILON,
    // then link edges that share the same key.

    // Adjacency map: key -> list of (edge_index, is_start_or_end)
    // We’ll store “(edge_idx, which_end)” as which_end = 0 or 1 for edges[edge_idx][0/1].
    let mut adjacency: HashMap<EndKey, Vec<(usize, usize)>> = HashMap::new();

    // Collect all endpoints
    for (i, edge) in edges.iter().enumerate() {
        for end_idx in 0..2 {
            let v = &edge[end_idx];
            let k = make_key(&v.pos);
            adjacency.entry(k).or_default().push((i, end_idx));
        }
    }

    // We’ll keep track of which edges have been “visited” in the final polylines.
    let mut visited = vec![false; edges.len()];

    let mut chains: Vec<Vec<Vertex>> = Vec::new();

    // For each edge not yet visited, we “walk” outward from one end, building a chain
    for start_edge_idx in 0..edges.len() {
        if visited[start_edge_idx] {
            continue;
        }
        // Mark it visited
        visited[start_edge_idx] = true;

        // Our chain starts with `edges[start_edge_idx]`. We can build a small function to “walk”:
        // We’ll store it in the direction edge[0] -> edge[1]
        let e = &edges[start_edge_idx];
        let mut chain = vec![e[0].clone(), e[1].clone()];

        // We walk “forward” from edge[1] if possible
        extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);

        // We also might walk “backward” from edge[0], but
        // we can do that by reversing the chain at the end if needed. Alternatively,
        // we can do a separate pass.  Let’s do it in place for clarity:
        chain.reverse();
        extend_chain_forward(&mut chain, &adjacency, &mut visited, edges);
        // Then reverse back so it goes in the original direction
        chain.reverse();

        chains.push(chain);
    }

    chains
}

/// Extends a chain “forward” by repeatedly finding any unvisited edge that starts
/// at the chain’s current end vertex.
#[cfg(feature = "hashmap")]
fn extend_chain_forward(
    chain: &mut Vec<Vertex>,
    adjacency: &HashMap<EndKey, Vec<(usize, usize)>>,
    visited: &mut [bool],
    edges: &[[Vertex; 2]],
) {
    loop {
        // The chain’s current end point:
        let last_v = chain.last().unwrap();
        let key = make_key(&last_v.pos);

        // Find candidate edges that share this endpoint
        let Some(candidates) = adjacency.get(&key) else {
            break;
        };

        // Among these candidates, we want one whose “other endpoint” we can follow
        // and is not visited yet.
        let mut found_next = None;
        for &(edge_idx, end_idx) in candidates {
            if visited[edge_idx] {
                continue;
            }
            // If this is edges[edge_idx][end_idx], the “other” end is edges[edge_idx][1-end_idx].
            // We want that other end to continue the chain.
            let other_end_idx = 1 - end_idx;
            let next_vertex = &edges[edge_idx][other_end_idx];

            // But we must also confirm that the last_v is indeed edges[edge_idx][end_idx]
            // (within EPSILON) which we have checked via the key, so likely yes.

            // Mark visited
            visited[edge_idx] = true;
            found_next = Some(next_vertex.clone());
            break;
        }

        match found_next {
            Some(v) => {
                chain.push(v);
            }
            None => {
                break;
            }
        }
    }
}
