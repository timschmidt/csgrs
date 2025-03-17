use crate::float_types::{EPSILON, PI, Real};
use crate::bsp::Node;
use crate::vertex::Vertex;
use crate::plane::Plane;
use crate::polygon::Polygon;
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, Vector3, partial_min, partial_max,
};
use geo::{
    Area, AffineTransform, AffineOps, BoundingRect, BooleanOps, coord, Coord, CoordsIter, Geometry, GeometryCollection, MultiPolygon, LineString, Orient, orient::Direction, Polygon as GeoPolygon, Rect, TriangulateEarcut,
};
//extern crate geo_booleanop;
//use geo_booleanop::boolean::BooleanOp;
use std::error::Error;
use std::fmt::Debug;
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

#[cfg(feature = "hershey-text")]
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "truetype-text")]
use ttf_utils::Outline;
#[cfg(feature = "truetype-text")]
use ttf_parser::{OutlineBuilder};

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

#[cfg(feature = "offset")]
use geo_buf::{ buffer_polygon, buffer_multi_polygon, };

// For flattening curves, how many segments per quad/cubic
const CURVE_STEPS: usize = 8;

/// The main CSG solid structure. Contains a list of 3D polygons, 2D polylines, and some metadata.
#[derive(Debug, Clone)]
pub struct CSG<S: Clone> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// 2D geometry
    pub geometry: GeometryCollection<Real>,

    /// Metadata
    pub metadata: Option<S>,
}


impl<S: Clone + Debug> CSG<S> where S: Clone + Send + Sync {
    /// Create an empty CSG
    pub fn new() -> Self {
        CSG {
            polygons: Vec::new(),
            geometry: GeometryCollection::default(),
            metadata: None,
        }
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

    /// Build a CSG from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut csg = CSG::new();
        csg.polygons = polygons.to_vec();
        csg
    }

    /// Convert internal polylines into polygons and return along with any existing internal polygons.
    pub fn to_polygons(&self) -> Vec<Polygon<S>> {
        let mut all_polygons = Vec::new();
    
        for geom in &self.geometry {
            if let Geometry::Polygon(poly2d) = geom {
                // 1. Convert the outer ring to 3D.
                let mut outer_vertices_3d = Vec::new();
                for c in poly2d.exterior().coords_iter() {
                    outer_vertices_3d.push(
                        Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z())
                    );
                }
                
                // Push as a new Polygon<S> if it has at least 3 vertices.
                if outer_vertices_3d.len() >= 3 {
                    all_polygons.push(Polygon::new(outer_vertices_3d, self.metadata.clone()));
                }
    
                // 2. Convert each interior ring (hole) into its own Polygon<S>.
                for ring in poly2d.interiors() {
                    let mut hole_vertices_3d = Vec::new();
                    for c in ring.coords_iter() {
                        hole_vertices_3d.push(
                            Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z())
                        );
                    }
    
                    if hole_vertices_3d.len() >= 3 {
                        // If your `Polygon<S>` type can represent holes internally,
                        // adjust this to store hole_vertices_3d as a hole rather
                        // than a new standalone polygon.
                        all_polygons.push(Polygon::new(hole_vertices_3d, self.metadata.clone()));
                    }
                }
            }
            // else if let Geometry::LineString(ls) = geom {
            //     // Example of how you might convert a linestring to a polygon,
            //     // if desired. Omitted for brevity.
            // }
        }
    
        all_polygons
    }
    
    /// Create a CSG that holds *only* 2D geometry in a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: Option<S>) -> Self {
        let mut csg = CSG::new();
        csg.geometry = geometry;
        csg.metadata = metadata;
        csg
    }
    
    pub fn tessellate_2d(outer: &[[Real; 2]], holes: &[&[[Real; 2]]]) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> = outer
            .iter()
            .map(|&[x, y]| Coord { x, y })
            .collect();
        
        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> = hole
                    .iter()
                    .map(|&[x, y]| Coord { x, y })
                    .collect();
                LineString::new(coords)
            })
            .collect();
    
        // Ear-cut triangulation on the polygon (outer + holes)
        let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);
        let triangulation = polygon.earcut_triangles_raw();
        let triangle_indices = triangulation.triangle_indices;
        let vertices = triangulation.vertices;
    
        // Convert the 2D result (x,y) into 3D triangles with z=0
        let mut result = Vec::with_capacity(triangle_indices.len() / 3);
        for tri in triangle_indices.chunks_exact(3) {
            let pts = [
                Point3::new(vertices[2 * tri[0]], vertices[2 * tri[0] + 1], 0.0),
                Point3::new(vertices[2 * tri[1]], vertices[2 * tri[1] + 1], 0.0),
                Point3::new(vertices[2 * tri[2]], vertices[2 * tri[2] + 1], 0.0),
            ];
            result.push(pts);
        }
        result
    }

    /// Return a new CSG representing union of the two CSG's.
    ///
    /// ```no_run
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    #[must_use = "Use new CSG representing space in both CSG's"]
    pub fn union(&self, other: &CSG<S>) -> CSG<S> {
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        
        // Extract polygons from geometry
        let polys1 = gc_to_polygons(&self.geometry);
        let polys2 = gc_to_polygons(&other.geometry);
    
        // Perform union on those polygons
        let unioned = polys1.union(&polys2); // This is valid if each is a MultiPolygon
        let oriented = unioned.orient(Direction::Default);
    
        // Wrap the unioned polygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));
        
        // re-insert lines & points from both sets:
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip polygons
                }
                _ => final_gc.0.push(g.clone())
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip polygons
                }
                _ => final_gc.0.push(g.clone())
            }
        }

        CSG {
            polygons: a.all_polygons(),
            geometry: final_gc,
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing diffarence of the two CSG's.
    ///
    /// ```no_run
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    #[must_use = "Use new CSG"]
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
        
        // -- 2D geometry-based approach --
        let polys1 = gc_to_polygons(&self.geometry);
        let polys2 = gc_to_polygons(&other.geometry);
    
        // Perform difference on those polygons
        let differenced = polys1.difference(&polys2);
        let oriented = differenced.orient(Direction::Default);
    
        // Wrap the differenced polygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));
    
        // Re-insert lines & points from self only
        // (If you need to exclude lines/points that lie inside other, you'd need more checks here.)
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
    
        CSG {
            polygons: a.all_polygons(),
            geometry: final_gc,
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```no_run
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
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
        
        // -- 2D geometry-based approach --
        let polys1 = gc_to_polygons(&self.geometry);
        let polys2 = gc_to_polygons(&other.geometry);
    
        // Perform intersection on those polygons
        let intersected = polys1.intersection(&polys2);
        let oriented = intersected.orient(Direction::Default);
    
        // Wrap the intersected polygons + lines/points into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));
    
        // For lines and points: keep them only if they intersect in both sets
        // todo: detect intersection of non-polygons
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
    
        CSG {
            polygons: a.all_polygons(),
            geometry: final_gc,
            metadata: self.metadata.clone(),
        }
    }
    
    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```no_run
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    pub fn xor(&self, other: &CSG<S>) -> CSG<S> {
        // A \ B
        let a_sub_b = self.difference(other);
    
        // B \ A
        let b_sub_a = other.difference(self);
    
        // Union those two
        a_sub_b.union(&b_sub_a)
        
        /* here in case 2D xor misbehaves as an alternate implementation
        // -- 2D geometry-based approach only (no polygon-based Node usage here) --
        let polys1 = gc_to_polygons(&self.geometry);
        let polys2 = gc_to_polygons(&other.geometry);
    
        // Perform symmetric difference (XOR)
        let xored = polys1.xor(&polys2);
        let oriented = xored.orient(Direction::Default);
    
        // Wrap in a new GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));
    
        // Re-insert lines & points from both sets
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
    
        CSG {
            // If you also want a polygon-based Node XOR, you'd need to implement that similarly
            polygons: self.polygons.clone(),
            geometry: final_gc,
            metadata: self.metadata.clone(),
        }
        */
    }

    /// Invert this CSG (flip inside vs. outside)
    pub fn inverse(&self) -> CSG<S> {
        let mut csg = self.clone();
        for p in &mut csg.polygons {
            p.flip();
        }
        csg
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    pub fn transform(&self, mat: &Matrix4<Real>) -> CSG<S> {
        let mat_inv_transpose = mat.try_inverse().expect("Matrix not invertible?").transpose(); // todo catch error
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
        
        // Convert the top-left 2×2 submatrix + translation of a 4×4 into a geo::AffineTransform
        // The 4x4 looks like:
        //  [ m11  m12  m13  m14 ]
        //  [ m21  m22  m23  m24 ]
        //  [ m31  m32  m33  m34 ]
        //  [ m41  m42  m43  m44 ]
        //
        // For 2D, we use the sub-block:
        //   a = m11,  b = m12,
        //   d = m21,  e = m22,
        //   xoff = m14,
        //   yoff = m24,
        // ignoring anything in z.
        //
        // So the final affine transform in 2D has matrix:
        //   [a   b   xoff]
        //   [d   e   yoff]
        //   [0   0    1  ]
        let a    = mat[(0, 0)];
        let b    = mat[(0, 1)];
        let xoff = mat[(0, 3)];
        let d    = mat[(1, 0)];
        let e    = mat[(1, 1)];
        let yoff = mat[(1, 3)];
    
        let affine2 = AffineTransform::new(a, b, xoff, d, e, yoff);

        // 4) Transform csg.geometry (the GeometryCollection) in 2D
        //    Using geo’s map-coords approach or the built-in AffineOps trait.
        //    Below we use the `AffineOps` trait if you have `use geo::AffineOps;`
        csg.geometry = csg.geometry.affine_transform(&affine2);

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
        let t1 = Translation3::from(-offset).to_homogeneous();  // push the plane to origin

        // Step 2) Build the reflection matrix about a plane normal n at the origin
        //   R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Step 3) Translate back
        let t2 = Translation3::from(offset).to_homogeneous();   // pull the plane back out

        // Combine into a single 4×4
        let mirror_mat = t2 * reflect_4 * t1;

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }
    
    /// Distribute this CSG `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new CSG with all copies (their polygons).
    pub fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
    ) -> CSG<S> {
        if count < 1 {
            return self.clone();
        }
        let start_rad = start_angle_deg.to_radians();
        let end_rad   = end_angle_deg.to_radians();
        let sweep     = end_rad - start_rad;

        // create a container to hold our unioned copies
        let mut all_csg = CSG::<S>::new();

        for i in 0..count {
            // pick an angle fraction
            let t = if count == 1 {
                0.5
            } else {
                i as Real / ((count - 1) as Real)
            };

            let angle = start_rad + t * sweep;
            let rot   = nalgebra::Rotation3::from_axis_angle(
                &nalgebra::Vector3::z_axis(),
                angle,
            )
            .to_homogeneous();

            // translate out to radius in x
            let trans = nalgebra::Translation3::new(radius, 0.0, 0.0).to_homogeneous();
            let mat   = rot * trans;

            // Transform a copy of self and union with other copies
            all_csg = all_csg.union(&self.transform(&mat));
        }

        // Put it in a new CSG
        CSG {
            polygons: all_csg.polygons,
            geometry: all_csg.geometry,
            metadata: self.metadata.clone(),
        }
    }
    
    /// Distribute this CSG `count` times along a straight line (vector),
    /// each copy spaced by `spacing`.
    /// E.g. if `dir=(1.0,0.0,0.0)` and `spacing=2.0`, you get copies at
    /// x=0, x=2, x=4, ... etc.
    pub fn distribute_linear(
        &self,
        count: usize,
        dir: nalgebra::Vector3<Real>,
        spacing: Real,
    ) -> CSG<S> {
        if count < 1 {
            return self.clone();
        }
        let step = dir.normalize() * spacing;
    
        // create a container to hold our unioned copies
        let mut all_csg = CSG::<S>::new();
    
        for i in 0..count {
            let offset  = step * (i as Real);
            let trans   = nalgebra::Translation3::from(offset).to_homogeneous();
    
            // Transform a copy of self and union with other copies
            all_csg = all_csg.union(&self.transform(&trans));
        }
    
        // Put it in a new CSG
        CSG {
            polygons: all_csg.polygons,
            geometry: all_csg.geometry,
            metadata: self.metadata.clone(),
        }
    }

    /// Distribute this CSG in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    pub fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> CSG<S> {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let step_x = nalgebra::Vector3::new(dx, 0.0, 0.0);
        let step_y = nalgebra::Vector3::new(0.0, dy, 0.0);
    
        // create a container to hold our unioned copies
        let mut all_csg = CSG::<S>::new();
    
        for r in 0..rows {
            for c in 0..cols {
                let offset = step_x * (c as Real) + step_y * (r as Real);
                let trans  = nalgebra::Translation3::from(offset).to_homogeneous();
    
                // Transform a copy of self and union with other copies
                all_csg = all_csg.union(&self.transform(&trans));
            }
        }
    
        // Put it in a new CSG
        CSG {
            polygons: all_csg.polygons,
            geometry: all_csg.geometry,
            metadata: self.metadata.clone(),
        }
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
    
        // Attempt to compute the convex hull using the robust wrapper
        let hull = match ConvexHullWrapper::try_new(&points, None) {
            Ok(h) => h,
            Err(_) => {
                // Fallback to an empty CSG if hull generation fails
                return CSG::new();
            }
        };
    
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
            polygons.push(Polygon::new(vec![vv0, vv1, vv2], None));
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
            let triangles = poly.tessellate();

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

    /// Returns a [`parry3d::bounding_volume::Aabb`] by merging:
    /// 1. The 3D bounds of all `polygons`.
    /// 2. The 2D bounding rectangle of `self.geometry`, interpreted at z=0.
    pub fn bounding_box(&self) -> Aabb {
        // Track overall min/max in x, y, z among all 3D polygons and the 2D geometry’s bounding_rect.
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut min_z = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;
        let mut max_z = -Real::MAX;

        // 1) Gather from the 3D polygons
        for poly in &self.polygons {
            for v in &poly.vertices {
                min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                min_z = *partial_min(&min_z, &v.pos.z).unwrap();

                max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                max_z = *partial_max(&max_z, &v.pos.z).unwrap();
            }
        }

        // 2) Gather from the 2D geometry using `geo::BoundingRect`
        //    This gives us (min_x, min_y) / (max_x, max_y) in 2D. For 3D, treat z=0.
        //    Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
        let maybe_rect: Option<Rect<Real>> = self.geometry.bounding_rect().into();
    
        if let Some(rect) = maybe_rect {
            let min_pt = rect.min();
            let max_pt = rect.max();

            // Merge the 2D bounds into our existing min/max, forcing z=0 for 2D geometry.
            min_x = *partial_min(&min_x, &min_pt.x).unwrap();
            min_y = *partial_min(&min_y, &min_pt.y).unwrap();
            min_z = *partial_min(&min_z, &0.0).unwrap();

            max_x = *partial_max(&max_x, &max_pt.x).unwrap();
            max_y = *partial_max(&max_y, &max_pt.y).unwrap();
            max_z = *partial_max(&max_z, &0.0).unwrap();
        }

        // If still uninitialized (e.g., no polygons or geometry), return a trivial AABB at origin
        if min_x > max_x {
            return Aabb::new(Point3::origin(), Point3::origin());
        }

        // Build a parry3d Aabb from these min/max corners
        let mins = Point3::new(min_x, min_y, min_z);
        let maxs = Point3::new(max_x, max_y, max_z);
        Aabb::new(mins, maxs)
    }

    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using cavalier_contours parallel_offset.
    /// for each Polygon we convert to a cavalier_contours Polyline<Real> and call parallel_offset
    #[cfg(feature = "offset")]
    pub fn offset(&self, distance: Real) -> CSG<S> {
        // For each Geometry in the collection:
        //   - If it's a Polygon, buffer it and store the result as a MultiPolygon
        //   - If it's a MultiPolygon, buffer it directly
        //   - Otherwise, ignore (exclude) it from the new collection
        let offset_geoms = self.geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let new_mpoly = buffer_polygon(poly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                }
                Geometry::MultiPolygon(mpoly) => {
                    let new_mpoly = buffer_multi_polygon(mpoly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                }
                _ => None, // ignore other geometry types
            })
            .collect();
    
        // Construct a new GeometryCollection from the offset geometries
        let new_collection = GeometryCollection(offset_geoms);
    
        // Return a new CSG using the offset geometry collection and the old polygons/metadata
        CSG {
            polygons: self.polygons.clone(),
            geometry: new_collection,
            metadata: self.metadata.clone(),
        }
    }

    /// Flattens any 3D polygons by projecting them onto the XY plane (z=0),
    /// unifies them into one or more 2D polygons, and returns a purely 2D CSG.
    ///
    /// - If this CSG is already 2D (`self.polygons` is empty), just returns `self.clone()`.
    /// - Otherwise, all `polygons` are tessellated, projected into XY, and unioned.
    /// - We also union any existing 2D geometry (`self.geometry`).
    /// - The output has `.polygons` empty and `.geometry` containing the final 2D shape.
    pub fn flatten(&self) -> CSG<S> {
        // 1) If there are no 3D polygons, this is already purely 2D => return as-is
        if self.polygons.is_empty() {
            return self.clone();
        }
    
        // 2) Convert all 3D polygons into a collection of 2D polygons
        let mut flattened_3d = Vec::new(); // will store geo::Polygon<Real>
    
        for poly in &self.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.tessellate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let ring = vec![
                    (tri[0].pos.x, tri[0].pos.y),
                    (tri[1].pos.x, tri[1].pos.y),
                    (tri[2].pos.x, tri[2].pos.y),
                    (tri[0].pos.x, tri[0].pos.y), // close ring explicitly
                ];
                let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
                flattened_3d.push(polygon_2d);
            }
        }
    
        // 3) Union all these polygons together into one MultiPolygon
        //    (We could chain them in a fold-based union.)
        let unioned_from_3d = if flattened_3d.is_empty() {
            MultiPolygon::new(Vec::new())
        } else {
            // Start with the first polygon as a MultiPolygon
            let mut mp_acc = MultiPolygon(vec![flattened_3d[0].clone()]);
            // Union in the rest
            for p in flattened_3d.iter().skip(1) {
                mp_acc = mp_acc.union(&MultiPolygon(vec![p.clone()]));
            }
            mp_acc
        };
    
        // 4) Union this with any existing 2D geometry (polygons) from self.geometry
        let existing_2d = gc_to_polygons(&self.geometry);  // turns geometry -> MultiPolygon
        let final_union = unioned_from_3d.union(&existing_2d);
        // Optionally ensure consistent orientation (CCW for exteriors):
        let oriented = final_union.orient(Direction::Default);
    
        // 5) Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));
    
        // 6) Return a purely 2D CSG: polygons empty, geometry has the final shape
        CSG {
            polygons: Vec::new(),
            geometry: new_gc,
            metadata: self.metadata.clone(),
        }
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

        // Convert each polyline of vertices into a Polygon<S>
        let mut result_polygons = Vec::new();

        // Add the coplanar polygons. We can re‐assign their plane to `plane` to ensure
        // they share the exact plane definition (in case of numeric drift).
        for mut p in coplanar_polys {
            p.plane = plane.clone(); // unify plane data
            result_polygons.push(p);
        }

        let mut new_gc = GeometryCollection::default();

        // Convert the “chains” or loops into open/closed polygons
        for mut chain in polylines_3d {
            let n = chain.len();
            if n < 2 {
                // degenerate
                continue;
            }

            // check if first and last point are within EPSILON of each other
            let dist_sq = (chain[0].pos - chain[n - 1].pos).norm_squared();
            if dist_sq < EPSILON * EPSILON {
                // Force them to be exactly the same, closing the line
                chain[n - 1] = chain[0].clone();
            }
            
            let polyline = LineString::new(chain.iter().map(|vertex| {coord! {x: vertex.pos.x, y: vertex.pos.y}}).collect());
            
            if polyline.is_closed() {
                let polygon = GeoPolygon::new(polyline, vec![]);
                let oriented = polygon.orient(Direction::Default);
                new_gc.0.push(Geometry::Polygon(oriented));
            } else {
                new_gc.0.push(Geometry::LineString(polyline));
            }
        }

        // Return a purely 2D CSG: polygons empty, geometry has the final shape
        CSG {
            polygons: Vec::new(),
            geometry: new_gc,
            metadata: self.metadata.clone(),
        }
    }

    /// Create **2D text** (outlines only) in the XY plane using ttf-utils + ttf-parser.
    /// 
    /// Each glyph’s closed contours become one or more `Polygon`s (with holes if needed), 
    /// and any open contours become `LineString`s.
    ///
    /// # Arguments
    /// - `text`: the text string (no multiline logic here)
    /// - `font_data`: raw bytes of a TTF file
    /// - `scale`: a uniform scale factor for glyphs
    /// - `metadata`: optional metadata for the resulting `CSG`
    ///
    /// # Returns
    /// A `CSG` whose `geometry` contains:
    /// - One or more `Polygon`s for each glyph, 
    /// - A set of `LineString`s for any open contours (rare in standard fonts), 
    /// all positioned in the XY plane at z=0.
    pub fn text(
        text: &str,
        font_data: &[u8],
        scale: Real,
        metadata: Option<S>,
    ) -> Self {
        // 1) Parse the TTF font
        let face = match ttf_parser::Face::from_slice(font_data, 0) {
            Ok(f) => f,
            Err(_) => {
                // If the font fails to parse, return an empty 2D CSG
                return CSG::new();
            }
        };
        
        // 1 font unit, 2048 font units / em, scale points / em, 0.352777 points / mm
        let font_scale = 1.0 / 2048.0 * scale * 0.3527777;

        // 2) We'll collect all glyph geometry into one GeometryCollection
        let mut geo_coll = GeometryCollection::default();

        // 3) A simple "pen" cursor for horizontal text layout
        let mut cursor_x = 0.0 as Real;

        for ch in text.chars() {
            // Skip control chars:
            if ch.is_control() {
                continue;
            }

            // Find glyph index in the font
            if let Some(gid) = face.glyph_index(ch) {
                // Extract the glyph outline (if any)
                if let Some(outline) = Outline::new(&face, gid) {
                    // Flatten the outline into line segments
                    let mut collector = OutlineFlattener::new(font_scale as Real, cursor_x as Real, 0.0);
                    outline.emit(&mut collector);

                    // Now `collector.contours` holds closed subpaths,
                    // and `collector.open_contours` holds open polylines.

                    // -------------------------
                    // Handle all CLOSED subpaths (which might be outer shapes or holes):
                    // -------------------------
                    if !collector.contours.is_empty() {
                        // We can have multiple outer loops and multiple inner loops (holes).
                        let mut outer_rings = Vec::new();
                        let mut hole_rings  = Vec::new();

                        for closed_pts in collector.contours {
                            if closed_pts.len() < 3 {
                                continue; // degenerate
                            }

                            let ring = LineString::from(closed_pts);

                            // We need to measure signed area.  The `signed_area` method works on a Polygon,
                            // so construct a temporary single-ring polygon:
                            let tmp_poly = GeoPolygon::new(ring.clone(), vec![]);
                            let area = tmp_poly.signed_area();

                            // ttf files store outer loops as CW and inner loops as CCW
                            if area < 0.0 {
                                // This is an outer ring
                                outer_rings.push(ring);
                            } else {
                                // This is a hole ring
                                hole_rings.push(ring);
                            }
                        }

                        // Typically, a TrueType glyph has exactly one outer ring and 0+ holes.
                        // But in some tricky glyphs, you might see multiple separate outer rings.
                        // We'll create one Polygon for the first outer ring with all holes,
                        // then if there are additional outer rings, each becomes its own separate Polygon.
                        if !outer_rings.is_empty() {
                            let first_outer = outer_rings.remove(0);

                            // The “primary” polygon: first outer + all holes
                            let polygon_2d = GeoPolygon::new(first_outer, hole_rings);
                            let oriented = polygon_2d.orient(Direction::Default);
                            geo_coll.0.push(Geometry::Polygon(oriented));

                            // If there are leftover outer rings, push them each as a separate polygon (no holes):
                            // todo: test bounding boxes and sort holes appropriately
                            for extra_outer in outer_rings {
                                let poly_2d = GeoPolygon::new(extra_outer, vec![]);
                                let oriented = poly_2d.orient(Direction::Default);
                                geo_coll.0.push(Geometry::Polygon(oriented));
                            }
                        }
                    }

                    // -------------------------
                    // Handle all OPEN subpaths => store as LineStrings:
                    // -------------------------
                    for open_pts in collector.open_contours {
                        if open_pts.len() >= 2 {
                            geo_coll.0.push(Geometry::LineString(LineString::from(open_pts)));
                        }
                    }

                    // Finally, advance our pen by the glyph's bounding-box width
                    let bbox = outline.bbox();
                    let glyph_width = bbox.width() as Real * font_scale;
                    cursor_x += glyph_width;
                } else {
                    // If there's no outline (e.g., space), just move a bit
                    cursor_x += font_scale as Real * 0.3;
                }
            } else {
                // Missing glyph => small blank advance
                cursor_x += font_scale as Real * 0.3;
            }
        }

        // Build a 2D CSG from the collected geometry
        CSG::from_geo(geo_coll, metadata)
    }

    /// Triangulate each polygon in the CSG returning a CSG containing triangles
    pub fn tessellate(&self) -> CSG<S> {
        let mut triangles = Vec::new();
    
        for poly in &self.polygons {
            let tris = poly.tessellate();
            for triangle in tris {
                triangles.push(Polygon::new(triangle.to_vec(), poly.metadata.clone()));
            }
        }
        
        CSG::from_polygons(&triangles)
    }

    /// Creates **2D line-stroke text** in the XY plane using a Hershey font.
    ///
    /// Each glyph’s strokes become one or more `LineString<Real>` entries in `geometry`.
    /// If you need them filled or thickened, you can later offset or extrude these lines.
    ///
    /// # Parameters
    /// - `text`: The text to render
    /// - `font`: The Hershey font (e.g., `hershey::fonts::GOTHIC_ENG_SANS`)
    /// - `size`: Scale factor for glyphs
    /// - `metadata`: Optional user data to store in the resulting CSG
    ///
    /// # Returns
    /// A new `CSG` where each glyph stroke is a `Geometry::LineString` in `geometry`.
    ///
    pub fn from_hershey(
        text: &str,
        font: &Font,
        size: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        use geo::{Geometry, GeometryCollection};

        let mut all_strokes = Vec::new();
        let mut cursor_x: Real = 0.0;

        for ch in text.chars() {
            // Skip control chars or spaces as needed
            if ch.is_control() {
                continue;
            }

            // Attempt to find a glyph in this font
            match font.glyph(ch) {
                Ok(glyph) => {
                    // Convert the Hershey lines to geo::LineString objects
                    let glyph_width = (glyph.max_x - glyph.min_x) as Real;
                    let strokes = build_hershey_glyph_lines(&glyph, size, cursor_x, 0.0);

                    // Collect them
                    all_strokes.extend(strokes);

                    // Advance the pen in X
                    cursor_x += glyph_width * size * 0.8;
                }
                Err(_) => {
                    // Missing glyph => skip or just advance
                    cursor_x += 6.0 * size;
                }
            }
        }

        // Insert each stroke as a separate LineString in the geometry
        let mut geo_coll = GeometryCollection::default();
        for line_str in all_strokes {
            geo_coll.0.push(Geometry::LineString(line_str));
        }

        // Return a new CSG that has no 3D polygons, but has these lines in geometry.
        CSG {
            polygons: Vec::new(),
            geometry: geo_coll,
            metadata: metadata,
        }
    }

    /// Convert the polygons in this CSG to a Parry TriMesh.
    /// Useful for collision detection or physics simulations.
    pub fn to_trimesh(&self) -> SharedShape {
        // 1) Gather all the triangles from each polygon
        // 2) Build a TriMesh from points + triangle indices
        // 3) Wrap that in a SharedShape to be used in Rapier
        let tri_csg = self.tessellate();
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
    /// the coordinates, tessellates the CSG polygons, gathers each of their three edges,
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
        let tri_csg = self.tessellate();
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
    pub fn gyroid(&self, resolution: usize, period: Real, iso_value: Real, metadata: Option<S>) -> CSG<S> {
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
        fn _schwarz_p_f(x: Real, y: Real, z: Real, period: Real) -> Real {
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
                metadata.clone(),
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
            let mut verts = Vec::with_capacity(pl.len());
            for &(x, y) in &pl {
                verts.push(Vertex::new(
                    Point3::new(x as Real, y as Real, 0.0),
                    normal,
                ));
            }
            // If the path was not closed and we used closepaths == true, we might need to ensure the first/last are the same.
            if (verts.first().unwrap().pos - verts.last().unwrap().pos).norm() > EPSILON {
                // close it
                verts.push(verts.first().unwrap().clone());
            }
            let poly = Polygon::new(verts, metadata.clone());
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
    /// 1) 3D polygons in `self.polygons`,
    /// 2) any 2D Polygons or MultiPolygons in `self.geometry` (tessellated in XY).
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
    
        //
        // (A) Write out all *3D* polygons
        //
        for poly in &self.polygons {
            // Ensure the polygon is tessellated, since STL is triangle-based.
            let triangles = poly.tessellate();
            // A typical STL uses the face normal; we can take the polygon’s plane normal:
            let normal = poly.plane.normal.normalize();
    
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
    
        //
        // (B) Write out all *2D* geometry from `self.geometry`
        //     We only handle Polygon and MultiPolygon.  We tessellate in XY, set z=0.
        //    
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Outer ring (in CCW for a typical “positive” polygon)
                    let outer = poly2d
                        .exterior()
                        .coords_iter()
                        .map(|c| [c.x, c.y])
                        .collect::<Vec<[Real; 2]>>();
    
                    // Collect holes
                    let holes_vec = poly2d
                        .interiors()
                        .into_iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>())
                        .collect::<Vec<_>>();
                    let hole_refs = holes_vec
                        .iter()
                        .map(|hole_coords| &hole_coords[..])
                        .collect::<Vec<_>>();
    
                    // Triangulate with our existing helper:
                    let triangles_2d = Self::tessellate_2d(&outer, &hole_refs);
    
                    // Write each tri as a facet in ASCII STL, with a normal of (0,0,1)
                    for tri in triangles_2d {
                        out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                        out.push_str("    outer loop\n");
                        for pt in &tri {
                            out.push_str(&format!(
                                "      vertex {:.6} {:.6} {:.6}\n",
                                pt.x, pt.y, pt.z
                            ));
                        }
                        out.push_str("    endloop\n");
                        out.push_str("  endfacet\n");
                    }
                }
    
                geo::Geometry::MultiPolygon(mp) => {
                    // Each polygon inside the MultiPolygon
                    for poly2d in &mp.0 {
                        let outer = poly2d
                            .exterior()
                            .coords_iter()
                            .map(|c| [c.x, c.y])
                            .collect::<Vec<[Real; 2]>>();
    
                        // Holes
                        let holes_vec = poly2d
                            .interiors()
                            .into_iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect::<Vec<_>>())
                            .collect::<Vec<_>>();
                        let hole_refs = holes_vec
                            .iter()
                            .map(|hole_coords| &hole_coords[..])
                            .collect::<Vec<_>>();
    
                        let triangles_2d = Self::tessellate_2d(&outer, &hole_refs);
    
                        for tri in triangles_2d {
                            out.push_str("  facet normal 0.000000 0.000000 1.000000\n");
                            out.push_str("    outer loop\n");
                            for pt in &tri {
                                out.push_str(&format!(
                                    "      vertex {:.6} {:.6} {:.6}\n",
                                    pt.x, pt.y, pt.z
                                ));
                            }
                            out.push_str("    endloop\n");
                            out.push_str("  endfacet\n");
                        }
                    }
                }
    
                // Skip all other geometry types (LineString, Point, etc.)
                // You can optionally handle them if you like, or ignore them.
                _ => {}
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
        use stl_io::{Normal, Vertex, Triangle, write_stl};
        use core2::io::Cursor;
    
        let mut triangles = Vec::new();
    
        // Triangulate all 3D polygons in self.polygons
        for poly in &self.polygons {
            let normal = poly.plane.normal.normalize();
            // Convert polygon to triangles
            let tri_list = poly.tessellate();
            for tri in tri_list {
                triangles.push(Triangle {
                    normal: Normal::new([normal.x as f32, normal.y as f32, normal.z as f32]),
                    vertices: [
                        Vertex::new([
                            tri[0].pos.x as f32,
                            tri[0].pos.y as f32,
                            tri[0].pos.z as f32
                        ]),
                        Vertex::new([
                            tri[1].pos.x as f32,
                            tri[1].pos.y as f32,
                            tri[1].pos.z as f32
                        ]),
                        Vertex::new([
                            tri[2].pos.x as f32,
                            tri[2].pos.y as f32,
                            tri[2].pos.z as f32
                        ]),
                    ],
                });
            }
        }
    
        //
        // (B) Triangulate any 2D geometry from self.geometry (Polygon, MultiPolygon).
        //     We treat these as lying in the XY plane, at Z=0, with a default normal of +Z.
        //
        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Gather outer ring as [x,y]
                    let outer: Vec<[Real; 2]> =
                        poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
    
                    // Gather holes
                    let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();
    
                    // Convert each hole to a slice-reference for triangulation
                    let hole_refs: Vec<&[[Real; 2]]> = holes_vec.iter().map(|h| &h[..]).collect();
    
                    // Triangulate using our geo-based helper
                    let tri_2d = Self::tessellate_2d(&outer, &hole_refs);
    
                    // Each triangle is in XY, so normal = (0,0,1)
                    for tri_pts in tri_2d {
                        triangles.push(Triangle {
                            normal: Normal::new([0.0, 0.0, 1.0]),
                            vertices: [
                                Vertex::new([
                                    tri_pts[0].x as f32,
                                    tri_pts[0].y as f32,
                                    tri_pts[0].z as f32
                                ]),
                                Vertex::new([
                                    tri_pts[1].x as f32,
                                    tri_pts[1].y as f32,
                                    tri_pts[1].z as f32
                                ]),
                                Vertex::new([
                                    tri_pts[2].x as f32,
                                    tri_pts[2].y as f32,
                                    tri_pts[2].z as f32
                                ]),
                            ],
                        });
                    }
                }
    
                geo::Geometry::MultiPolygon(mpoly) => {
                    // Same approach, but each Polygon in the MultiPolygon
                    for poly2d in &mpoly.0 {
                        let outer: Vec<[Real; 2]> =
                            poly2d.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
    
                        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                            .collect();
    
                        let hole_refs: Vec<&[[Real; 2]]> = holes_vec.iter().map(|h| &h[..]).collect();
                        let tri_2d = Self::tessellate_2d(&outer, &hole_refs);
    
                        for tri_pts in tri_2d {
                            triangles.push(Triangle {
                                normal: Normal::new([0.0, 0.0, 1.0]),
                                vertices: [
                                    Vertex::new([
                                        tri_pts[0].x as f32,
                                        tri_pts[0].y as f32,
                                        tri_pts[0].z as f32
                                    ]),
                                    Vertex::new([
                                        tri_pts[1].x as f32,
                                        tri_pts[1].y as f32,
                                        tri_pts[1].z as f32
                                    ]),
                                    Vertex::new([
                                        tri_pts[2].x as f32,
                                        tri_pts[2].y as f32,
                                        tri_pts[2].z as f32
                                    ]),
                                ],
                            });
                        }
                    }
                }
    
                // Skip other geometry types: lines, points, etc.
                _ => {}
            }
        }
    
        //
        // (C) Encode into a binary STL buffer
        //
        let mut cursor = Cursor::new(Vec::new());
        write_stl(&mut cursor, triangles.iter())?;
        Ok(cursor.into_inner())
    }

    /// Create a CSG object from STL data using `stl_io`.
    #[cfg(feature = "stl-io")]
    pub fn from_stl(stl_data: &[u8], metadata: Option<S>,) -> Result<CSG<S>, std::io::Error> {
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
            polygons.push(Polygon::new(vertices, metadata.clone()));
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
    pub fn from_dxf(dxf_data: &[u8], metadata: Option<S>,) -> Result<CSG<S>, Box<dyn Error>> {
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
                            polygons.push(Polygon::new(verts, None));
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
                    polygons.push(Polygon::new(verts, metadata.clone()));
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
                poly.tessellate()
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

/// Helper for building open polygons from a single Hershey `Glyph`.
#[cfg(feature = "hershey-text")]
fn build_hershey_glyph_lines(
    glyph: &HersheyGlyph,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
) -> Vec<geo::LineString<Real>> {
    use geo::{coord, LineString};

    let mut strokes = Vec::new();

    // We'll accumulate each stroke’s points in `current_coords`,
    // resetting whenever Hershey issues a "MoveTo"
    let mut current_coords = Vec::new();

    for vector_cmd in &glyph.vectors {
        match vector_cmd {
            HersheyVector::MoveTo { x, y } => {
                // If we already had 2+ points, that stroke is complete:
                if current_coords.len() >= 2 {
                    strokes.push(LineString::from(current_coords));
                }
                // Start a new stroke
                current_coords = Vec::new();
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push(coord! { x: px, y: py });
            }
            HersheyVector::LineTo { x, y } => {
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push(coord! { x: px, y: py });
            }
        }
    }

    // End-of-glyph: if our final stroke has 2+ points, convert to a line string
    if current_coords.len() >= 2 {
        strokes.push(LineString::from(current_coords));
    }

    strokes
}

// Extract only the polygons from a geometry collection
fn gc_to_polygons(gc: &GeometryCollection<Real>) -> MultiPolygon<Real> {
    let mut polygons = vec![];
    for geom in &gc.0 {
        match geom {
            Geometry::Polygon(poly) => polygons.push(poly.clone()),
            Geometry::MultiPolygon(mp) => polygons.extend(mp.0.clone()),
            // ignore lines, points, etc.
            _ => {}
        }
    }
    MultiPolygon(polygons)
}

/// A helper that implements `ttf_parser::OutlineBuilder`.
/// It receives MoveTo/LineTo/QuadTo/CurveTo calls from `outline.emit(self)`.
/// We flatten curves and accumulate polylines. 
///
/// - Whenever `close()` occurs, we finalize the current subpath as a closed polygon (`contours`).
/// - If we start a new MoveTo while the old subpath is open, that old subpath is treated as open (`open_contours`).
struct OutlineFlattener {
    // scale + offset
    scale: Real,
    offset_x: Real,
    offset_y: Real,

    // We gather shapes: each "subpath" can be closed or open
    contours: Vec<Vec<(Real, Real)>>,      // closed polygons
    open_contours: Vec<Vec<(Real, Real)>>, // open polylines

    current: Vec<(Real, Real)>, // points for the subpath
    last_pt: (Real, Real),      // current "cursor" in flattening
    subpath_open: bool,
}

impl OutlineFlattener {
    fn new(scale: Real, offset_x: Real, offset_y: Real) -> Self {
        Self {
            scale,
            offset_x,
            offset_y,
            contours: Vec::new(),
            open_contours: Vec::new(),
            current: Vec::new(),
            last_pt: (0.0, 0.0),
            subpath_open: false,
        }
    }

    /// Helper: transform TTF coordinates => final (x,y)
    #[inline]
    fn tx(&self, x: f32, y: f32) -> (Real, Real) {
        let sx = x as Real * self.scale + self.offset_x;
        let sy = y as Real * self.scale + self.offset_y;
        (sx, sy)
    }

    /// Start a fresh subpath
    fn begin_subpath(&mut self, x: f32, y: f32) {
        // If we already had an open subpath, push it as open_contours:
        if self.subpath_open && !self.current.is_empty() {
            self.open_contours.push(self.current.clone());
        }
        self.current.clear();

        self.subpath_open = true;
        self.last_pt = self.tx(x, y);
        self.current.push(self.last_pt);
    }

    /// Finish the current subpath as open (do not close).
    /// (We call this if a new `MoveTo` or the entire glyph ends.)
    fn _finish_open_subpath(&mut self) {
        if self.subpath_open && !self.current.is_empty() {
            self.open_contours.push(self.current.clone());
        }
        self.current.clear();
        self.subpath_open = false;
    }

    /// Flatten a line from `last_pt` to `(x,y)`.
    fn line_to_impl(&mut self, x: f32, y: f32) {
        let (xx, yy) = self.tx(x, y);
        self.current.push((xx, yy));
        self.last_pt = (xx, yy);
    }

    /// Flatten a quadratic Bézier from last_pt -> (x1,y1) -> (x2,y2)
    fn quad_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) {
        let steps = CURVE_STEPS;
        let (px0, py0) = self.last_pt;
        let (px1, py1) = self.tx(x1, y1);
        let (px2, py2) = self.tx(x2, y2);

        // B(t) = (1 - t)^2 * p0 + 2(1 - t)t * cp + t^2 * p2
        for i in 1..=steps {
            let t = i as Real / steps as Real;
            let mt = 1.0 - t;
            let bx = mt*mt*px0 + 2.0*mt*t*px1 + t*t*px2;
            let by = mt*mt*py0 + 2.0*mt*t*py1 + t*t*py2;
            self.current.push((bx, by));
        }
        self.last_pt = (px2, py2);
    }

    /// Flatten a cubic Bézier from last_pt -> (x1,y1) -> (x2,y2) -> (x3,y3)
    fn curve_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        let steps = CURVE_STEPS;
        let (px0, py0) = self.last_pt;
        let (cx1, cy1) = self.tx(x1, y1);
        let (cx2, cy2) = self.tx(x2, y2);
        let (px3, py3) = self.tx(x3, y3);

        // B(t) = (1-t)^3 p0 + 3(1-t)^2 t c1 + 3(1-t) t^2 c2 + t^3 p3
        for i in 1..=steps {
            let t = i as Real / steps as Real;
            let mt = 1.0 - t;
            let mt2 = mt*mt;
            let t2  = t*t;
            let bx = mt2*mt*px0
                + 3.0*mt2*t*cx1
                + 3.0*mt*t2*cx2
                + t2*t*px3;
            let by = mt2*mt*py0
                + 3.0*mt2*t*cy1
                + 3.0*mt*t2*cy2
                + t2*t*py3;
            self.current.push((bx, by));
        }
        self.last_pt = (px3, py3);
    }

    /// Called when `close()` is invoked => store as a closed polygon.
    fn close_impl(&mut self) {
        // We have a subpath that should be closed => replicate first point as last if needed.
        let n = self.current.len();
        if n > 2 {
            // If the last point != the first, close it.
            let first = self.current[0];
            let last  = self.current[n-1];
            if (first.0 - last.0).abs() > Real::EPSILON || (first.1 - last.1).abs() > Real::EPSILON {
                self.current.push(first);
            }
            // That becomes one closed contour
            self.contours.push(self.current.clone());
        } else {
            // If it's 2 or fewer points, ignore or treat as degenerate
        }

        self.current.clear();
        self.subpath_open = false;
    }
}

impl OutlineBuilder for OutlineFlattener {
    fn move_to(&mut self, x: f32, y: f32) {
        self.begin_subpath(x, y);
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.line_to_impl(x, y);
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) {
        self.quad_to_impl(x1, y1, x2, y2);
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        self.curve_to_impl(x1, y1, x2, y2, x3, y3);
    }

    fn close(&mut self) {
        self.close_impl();
    }
}


// Build a small helper for hashing endpoints:
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