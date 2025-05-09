use crate::bsp::Node;
use crate::float_types::{EPSILON, Real};
use crate::plane::Plane;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use geo::{
    AffineOps, AffineTransform, BooleanOps, BoundingRect, Coord, CoordsIter, Geometry,
    GeometryCollection, LineString, MultiPolygon, Orient, Polygon as GeoPolygon, Rect,
    orient::Direction,
};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, Vector3,
    partial_max, partial_min,
};
use crate::float_types::parry3d::{
    bounding_volume::Aabb,
    query::{Ray, RayCast},
    shape::{Shape, SharedShape, TriMesh, Triangle},
};
use crate::float_types::rapier3d::prelude::*;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "bevymesh")]
use bevy::{prelude::*, render::render_asset::RenderAssetUsages, render::mesh::{Indices, PrimitiveTopology}};

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

impl<S: Clone + Debug + Send + Sync> CSG<S> {
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
                    outer_vertices_3d.push(Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()));
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

    /// Take the [`geo::Polygon`]'s from the `CSG`'s geometry collection
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        // allocate vec to fit all polygons
        let mut polygons = Vec::with_capacity(self.geometry.0.iter().fold(0, |len, geom| {
            len + match geom {
                Geometry::Polygon(_) => len + 1,
                Geometry::MultiPolygon(mp) => len + mp.0.len(),
                // ignore lines, points, etc.
                _ => len,
            }
        }));

        for geom in &self.geometry.0 {
            match geom {
                Geometry::Polygon(poly) => polygons.push(poly.clone()),
                Geometry::MultiPolygon(mp) => polygons.extend(mp.0.clone()),
                // ignore lines, points, etc.
                _ => {}
            }
        }

        MultiPolygon(polygons)
    }

    pub fn tessellate_2d(outer: &[[Real; 2]], holes: &[&[[Real; 2]]]) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> = outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> = hole.iter().map(|&[x, y]| Coord { x, y }).collect();
                LineString::new(coords)
            })
            .collect();

        // Ear-cut triangulation on the polygon (outer + holes)
        let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);

        #[cfg(feature = "earcut")]
        {
            use geo::TriangulateEarcut;
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

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            // We want polygons with holes => constrained triangulation.
            // For safety, handle the Result the trait returns:
            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                // If a triangulation error is a possibility,
                // pick the error-handling you want here:
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                // Each `triangle` is a geo_types::Triangle whose `.0, .1, .2`
                // are the 2D coordinates. We'll embed them at z=0.
                let [a, b, c] = [triangle.0, triangle.1, triangle.2];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }
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
        // 3D union:
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        // 2D union:
        // Extract multipolygon from geometry
        let polys1 = self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform union on those multipolygons
        let unioned = polys1.union(polys2); // This is valid if each is a MultiPolygon
        let oriented = unioned.orient(Direction::Default);

        // Wrap the unioned multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // re-insert lines & points from both sets:
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                }
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                }
                _ => final_gc.0.push(g.clone()),
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
        // 3D difference:
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

        // 2D difference:
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform difference on those multipolygons
        let differenced = polys1.difference(polys2);
        let oriented = differenced.orient(Direction::Default);

        // Wrap the differenced multipolygons + lines/points back into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // Re-insert lines & points from self only
        // (If you need to exclude lines/points that lie inside other, you'd need more checks here.)
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
        // 3D intersection:
        let mut a = Node::new(&self.polygons);
        let mut b = Node::new(&other.polygons);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        // 2D intersection:
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform intersection on those multipolygons
        let intersected = polys1.intersection(polys2);
        let oriented = intersected.orient(Direction::Default);

        // Wrap the intersected multipolygons + lines/points into one GeometryCollection
        let mut final_gc = GeometryCollection::default();
        final_gc.0.push(Geometry::MultiPolygon(oriented));

        // For lines and points: keep them only if they intersect in both sets
        // todo: detect intersection of non-polygons
        for g in &self.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {} // skip
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
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)

        /* here in case 2D xor misbehaves as an alternate implementation
        // 2D xor:
        let polys1 = &self.to_multipolygon();
        let polys2 = &other.to_multipolygon();

        // Perform symmetric difference (XOR)
        let xored = polys1.xor(polys2);
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
        let mat_inv_transpose = mat
            .try_inverse().expect("Matrix not invertible?")
            .transpose(); // todo catch error
        let mut csg = self.clone();

        for poly in &mut csg.polygons {
            for vert in &mut poly.vertices {
                // Position
                let hom_pos = mat * vert.pos.to_homogeneous();
                vert.pos = Point3::from_homogeneous(hom_pos).unwrap(); // todo catch error

                // Normal
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }
            
            // keep the cached plane consistent with the new vertex positions
            poly.plane = Plane::from_vertices(poly.vertices.clone());
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
        let a = mat[(0, 0)];
        let b = mat[(0, 1)];
        let xoff = mat[(0, 3)];
        let d = mat[(1, 0)];
        let e = mat[(1, 1)];
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
        let len = plane.normal().norm();
        if len.abs() < EPSILON {
            // Degenerate plane? Just return clone (no transform)
            return self.clone();
        }

        // Unit normal:
        let n = plane.normal() / len;
        // Adjusted offset = w / ||n||
        let w = plane.offset() / len;

        // Step 1) Translate so the plane crosses the origin
        // The plane’s offset vector from origin is (w * n).
        let offset = n * w;
        let t1 = Translation3::from(-offset).to_homogeneous(); // push the plane to origin

        // Step 2) Build the reflection matrix about a plane normal n at the origin
        //   R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Step 3) Translate back
        let t2 = Translation3::from(offset).to_homogeneous(); // pull the plane back out

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
        let end_rad = end_angle_deg.to_radians();
        let sweep = end_rad - start_rad;

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
            let rot = nalgebra::Rotation3::from_axis_angle(
                &nalgebra::Vector3::z_axis(),
                angle,
            )
            .to_homogeneous();

            // translate out to radius in x
            let trans = nalgebra::Translation3::new(radius, 0.0, 0.0).to_homogeneous();
            let mat = rot * trans;

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
            let offset = step * (i as Real);
            let trans = nalgebra::Translation3::from(offset).to_homogeneous();

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
                let trans = nalgebra::Translation3::from(offset).to_homogeneous();

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
        let maybe_rect: Option<Rect<Real>> = self.geometry.bounding_rect();

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
    
    /// Convert a CSG into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> Mesh {
        let tessellated_csg = &self.tessellate();
        let polygons = &tessellated_csg.polygons;
    
        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32   = Vec::new();
        let mut indices      = Vec::new();
    
        let mut index_start = 0u32;
    
        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }
    
            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }
    
            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }
    
        // Create the mesh with the new 2-argument constructor
        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    
        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL,   normals_32);
    
        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));
    
        mesh
    }
}
