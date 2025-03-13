use crate::float_types::{EPSILON, PI, TAU, FRAC_PI_2, Real};
use crate::bsp::Node;
use crate::vertex::Vertex;
use crate::plane::Plane;
use crate::polygon::Polygon;
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Point3, Quaternion, Rotation3, Translation3, Unit, Vector3, partial_min, partial_max,
};
use geo::{
    Area, AffineTransform, AffineOps, BoundingRect, line_string, BooleanOps, coord, Coord, CoordsIter, Geometry, GeometryCollection, MultiPolygon, LineString, Orient, orient::Direction, Polygon as GeoPolygon, Rect, TriangulateEarcut,
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

#[cfg(any(feature = "metaballs", feature = "sdf"))]
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};

// For flattening curves, how many segments per quad/cubic
const CURVE_STEPS: usize = 8;

#[cfg(feature = "metaballs")]
#[derive(Debug, Clone)]
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

    // Return a new CSG representing space in either this CSG or in the
    // other CSG. Neither this CSG nor the other CSG are modified.
    // 
    //     let c = a.union(b);
    // 
    //     +-------+            +-------+
    //     |       |            |       |
    //     |   a   |            |       |
    //     |    +--+----+   =   |       +----+
    //     +----+--+    |       +----+       |
    //          |   b   |            |       |
    //          |       |            |       |
    //          +-------+            +-------+
    // 
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

    // Return a new CSG representing space in this CSG but not in the
    // other CSG. Neither this CSG nor the other CSG are modified.
    // 
    //     let c = a.difference(b);
    // 
    //     +-------+            +-------+
    //     |       |            |       |
    //     |   a   |            |       |
    //     |    +--+----+   =   |    +--+
    //     +----+--+    |       +----+
    //          |   b   |
    //          |       |
    //          +-------+
    // 
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

    // Return a new CSG representing space in both this CSG and the
    // other CSG. Neither this CSG nor the other CSG are modified.
    // 
    //     let c = a.intersect(b);
    // 
    //     +-------+
    //     |       |
    //     |   a   |
    //     |    +--+----+   =   +--+
    //     +----+--+    |       +--+
    //          |   b   |
    //          |       |
    //          +-------+
    // 
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
    
    // Return a new CSG representing space in this CSG excluding the space in the
    // other CSG plus the space in the other CSG excluding the space in this CSG.
    // Neither this CSG nor the other CSG are modified.
    // 
    //     let c = a.xor(b);
    // 
    //     +-------+            +-------+
    //     |       |            |       |
    //     |   a   |            |   a   |
    //     |    +--+----+   =   |    +--+----+
    //     +----+--+    |       +----+--+    |
    //          |   b   |            |       |
    //          |       |            |       |
    //          +-------+            +-------+
    // 
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

    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the square
    /// - `length`: the height of the square
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let sq2 = CSG::square(2.0, 3.0, None);
    pub fn square(width: Real, length: Real, metadata: Option<S>) -> Self {
        // In geo, a Polygon is basically (outer: LineString, Vec<LineString> for holes).
        let outer = line_string![
            (x: 0.0,     y: 0.0),
            (x: width,   y: 0.0),
            (x: width,   y: length),
            (x: 0.0,     y: length),
            (x: 0.0,     y: 0.0),  // close explicitly
        ];
        let polygon_2d = GeoPolygon::new(outer, vec![]);

        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Creates a 2D circle in the XY plane.
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = 2.0 * PI * (i as Real) / (segments as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            coords.push((x, y));
        }
        // close it
        coords.push((coords[0].0, coords[0].1));
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }
    
    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let line_string: LineString = vec![[0.0, 0.0], [width, 0.0], [0.0, height]].into();
        let polygon = GeoPolygon::new(line_string, vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon)]), metadata)
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
    /// let poly2d = CSG::polygon(&pts, metadata);
    pub fn polygon(points: &[[Real; 2]], metadata: Option<S>) -> Self {
        if points.len() < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(points.len() + 1);
        for p in points {
            coords.push((p[0], p[1]));
        }
        // close
        if coords[0] != *coords.last().unwrap() {
            coords.push(coords[0]);
        }
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let r = corner_radius.min(width * 0.5).min(height * 0.5);
        if r <= EPSILON {
            return Self::square(width, height, metadata);
        }
        let mut coords = Vec::new();
        // We'll approximate each 90° corner with `corner_segments` arcs
        let step = FRAC_PI_2 / corner_segments as Real;

        // Top-left corner arc, center (r, height-r), (π → 3π/2) angles 180 -> 270
        let cx_tl = r;
        let cy_tl = height - r;
        for i in 0..=corner_segments {
            let angle = FRAC_PI_2 + (i as Real) * step;
            let x = cx_tl + r * angle.cos();
            let y = cy_tl + r * angle.sin();
            coords.push((x, y));
        }
        
        // Bottom-left corner arc, center (r, r), (π/2 → π) angles 90 -> 180
        let cx_bl = r;
        let cy_bl = r;
        for i in 0..=corner_segments {
            let angle = PI + (i as Real) * step;
            let x = cx_bl + r * angle.cos();
            let y = cy_bl + r * angle.sin();
            coords.push((x, y));
        }
        
        // Bottom-right corner arc, center (width-r, r), (0 → π/2) angles 0 -> 90
        let cx_br = width - r;
        let cy_br = r;
        for i in 0..=corner_segments {
            let angle = 1.5 * PI + (i as Real) * step;
            let x = cx_br + r * angle.cos();
            let y = cy_br + r * angle.sin();
            coords.push((x, y));
        }

        // Top-right corner arc, center (width-r, height-r), (3π/2 → 2π) angles 270 -> 360
        let cx_tr = width - r;
        let cy_tr = height - r;
        for i in 0..=corner_segments {
            let angle = 0.0 + (i as Real) * step;
            let x = cx_tr + r * angle.cos();
            let y = cy_tr + r * angle.sin();
            coords.push((x, y));
        }

        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Ellipse in XY plane, centered at (0,0), with full width `width`, full height `height`.
    /// `segments` is the number of polygon edges approximating the ellipse.
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = TAU * (i as Real) / (segments as Real);
            let x = rx * theta.cos();
            let y = ry * theta.sin();
            coords.push((x, y));
        }
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Regular N-gon in XY plane, centered at (0,0), with circumscribed radius `radius`.
    /// `sides` is how many edges (>=3).
    pub fn regular_ngon(sides: usize, radius: Real, metadata: Option<S>) -> Self {
        if sides < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(sides + 1);
        for i in 0..sides {
            let theta = TAU * (i as Real) / (sides as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            coords.push((x, y));
        }
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Trapezoid from (0,0) -> (bottom_width,0) -> (top_width+top_offset,height) -> (top_offset,height)
    /// Note: this is a simple shape that can represent many trapezoids or parallelograms.
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
        metadata: Option<S>,
    ) -> Self {
        let coords = vec![
            (0.0,             0.0),
            (bottom_width,    0.0),
            (top_width + top_offset, height),
            (top_offset,      height),
            (0.0,             0.0), // close
        ];
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: Option<S>) -> Self {
        if num_points < 2 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(2 * num_points + 1);
        let step = TAU / (num_points as Real);
        for i in 0..num_points {
            // Outer point
            let theta_out = i as Real * step;
            let x_out = outer_radius * theta_out.cos();
            let y_out = outer_radius * theta_out.sin();
            coords.push((x_out, y_out));

            // Inner point
            let theta_in = theta_out + 0.5 * step;
            let x_in = inner_radius * theta_in.cos();
            let y_in = inner_radius * theta_in.sin();
            coords.push((x_in, y_in));
        }
        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Teardrop shape.  A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    /// This is just one of many possible "teardrop" definitions.
    // todo: center on focus of the arc
    pub fn teardrop_outline(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 2 || width < EPSILON || length < EPSILON {
            return CSG::new();
        }
        let r = 0.5 * width;
        let center_y = length - r;
        let half_seg = segments / 2;

        // We’ll store points, starting from the bottom tip at (0,0).
        let mut coords = Vec::with_capacity(segments + 2);
        coords.push((0.0, 0.0));

        // Arc around
        for i in 0..=half_seg {
            let t = PI * (i as Real / half_seg as Real);
            let x = -r * t.cos(); // left
            let y = center_y + r * t.sin();
            coords.push((x, y));
        }

        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    pub fn egg_outline(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * length;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = TAU * (i as Real) / (segments as Real);
            // toy distortion approach
            let distort = 1.0 + 0.2 * theta.cos();
            let x = rx * theta.sin();
            let y = ry * theta.cos() * distort * 0.8;
            coords.push((-x, y));  // mirrored
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }


    /// Squircle (superellipse) centered at (0,0) with bounding box width×height.
    /// We use an exponent = 4.0 for "classic" squircle shape. `segments` controls the resolution.
    pub fn squircle(
        width: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let m = 4.0;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let t = TAU * (i as Real) / (segments as Real);
            let ct = t.cos().abs().powf(2.0 / m) * t.cos().signum();
            let st = t.sin().abs().powf(2.0 / m) * t.sin().signum();
            let x = rx * ct;
            let y = ry * st;
            coords.push((x, y));
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Keyhole shape (simple version): a large circle + a rectangle "handle".
    /// This does *not* have a hole.  If you want a literal hole, you'd do difference ops.
    /// Here we do union of a circle and a rectangle.
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        // 1) Circle
        let mut circle_coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            circle_coords.push((circle_radius * th.cos(), circle_radius * th.sin()));
        }
        circle_coords.push(circle_coords[0]);
        let circle_poly = GeoPolygon::new(LineString::from(circle_coords), vec![]);

        // 2) Rectangle (handle), from -hw/2..+hw/2 in X and 0..handle_height in Y
        let rect_coords = vec![
            (-0.5 * handle_width, 0.0),
            ( 0.5 * handle_width, 0.0),
            ( 0.5 * handle_width, handle_height),
            (-0.5 * handle_width, handle_height),
            (-0.5 * handle_width, 0.0),
        ];
        let rect_poly = GeoPolygon::new(LineString::from(rect_coords), vec![]);

        // 3) Union them with geo’s BooleanOps
        let mp1 = MultiPolygon(vec![circle_poly]);
        let mp2 = MultiPolygon(vec![rect_poly]);
        let multipolygon_2d = mp1.union(&mp2);

        CSG::from_geo(GeometryCollection(vec![Geometry::MultiPolygon(multipolygon_2d)]), metadata)
    }

    /// Reuleaux polygon with `sides` and "radius".  Approximates constant-width shape.
    /// This is a simplified approximation that arcs from each vertex to the next.
    pub fn reuleaux_polygon(
        sides: usize,
        radius: Real,
        arc_segments_per_side: usize,
        metadata: Option<S>
    ) -> CSG<S> {
        if sides < 3 || arc_segments_per_side < 1 {
            return CSG::new();
        }
        // Corner positions (the "center" of each arc is the next corner).
        let mut corners = Vec::with_capacity(sides);
        for i in 0..sides {
            let theta = TAU * (i as Real) / (sides as Real);
            corners.push((radius * theta.cos(), radius * theta.sin()));
        }

        // Build one big ring of points by tracing arcs corner->corner.
        let mut coords = Vec::new();
        for i in 0..sides {
            let i_next = (i + 1) % sides;
            let center = corners[i_next];
            let start_pt = corners[i];
            let end_pt   = corners[(i + 2) % sides];

            let vx_s = start_pt.0 - center.0;
            let vy_s = start_pt.1 - center.1;
            let start_angle = vy_s.atan2(vx_s);

            let vx_e = end_pt.0 - center.0;
            let vy_e = end_pt.1 - center.1;
            let end_angle = vy_e.atan2(vx_e);

            let mut delta = end_angle - start_angle;
            while delta <= 0.0 {
                delta += TAU;
            }
            let step = delta / (arc_segments_per_side as Real);
            for seg_i in 0..arc_segments_per_side {
                let a = start_angle + (seg_i as Real) * step;
                let x = center.0 + radius * a.cos();
                let y = center.1 + radius * a.sin();
                coords.push((x, y));
            }
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }

    /// Ring with inner diameter = `id` and (radial) thickness = `thickness`.
    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    ///
    /// Internally, we do:
    ///   outer = circle(outer_radius)
    ///   inner = circle(inner_radius)
    ///   ring = outer.difference(inner)
    /// Then we call `flatten()` to unify into a single shape that has a hole.
    pub fn ring(
        id: Real,
        thickness: Real,
        segments: usize,
        metadata: Option<S>
    ) -> CSG<S> {
        if id <= 0.0 || thickness <= 0.0 || segments < 3 {
            return CSG::new();
        }
        let inner_radius = 0.5 * id;
        let outer_radius = inner_radius + thickness;

        // Outer ring (CCW)
        let mut outer = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            let x = outer_radius * th.cos();
            let y = outer_radius * th.sin();
            outer.push((x, y));
        }
        outer.push(outer[0]);

        // Inner ring (must be opposite orientation for a hole in geo)
        let mut inner = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            let x = inner_radius * th.cos();
            let y = inner_radius * th.sin();
            inner.push((x, y));
        }
        inner.push(inner[0]);
        inner.reverse();  // ensure hole is opposite winding from outer

        let polygon_2d = GeoPolygon::new(LineString::from(outer), vec![LineString::from(inner)]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }
    
    /// Create a 2D "pie slice" (wedge) in the XY plane.
    /// - `radius`: outer radius of the slice.
    /// - `start_angle_deg`: starting angle in degrees (measured from X-axis).
    /// - `end_angle_deg`: ending angle in degrees.
    /// - `segments`: how many segments to use to approximate the arc.
    /// - `metadata`: optional user metadata for this polygon.
    pub fn pie_slice(
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
        segments: usize,
        metadata: Option<S>
    ) -> CSG<S> {
        if segments < 1 {
            return CSG::new();
        }
    
        let start_rad = start_angle_deg.to_radians();
        let end_rad   = end_angle_deg.to_radians();
        let sweep     = end_rad - start_rad;
    
        // Build a ring of coordinates starting at (0,0), going around the arc, and closing at (0,0).
        let mut coords = Vec::with_capacity(segments + 2);
        coords.push((0.0, 0.0));
        for i in 0..=segments {
            let t = i as Real / (segments as Real);
            let angle = start_rad + t * sweep;
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            coords.push((x, y));
        }
        coords.push((0.0, 0.0)); // close explicitly
    
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }
    
    /// Create a 2D metaball iso-contour in XY plane from a set of 2D metaballs.
    /// - `balls`: array of (center, radius).
    /// - `resolution`: (nx, ny) grid resolution for marching squares.
    /// - `iso_value`: threshold for the iso-surface.
    /// - `padding`: extra boundary beyond each ball's radius.
    /// - `metadata`: optional user metadata.
    pub fn metaball_2d(
        balls: &[(nalgebra::Point2<Real>, Real)],
        resolution: (usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: Option<S>
    ) -> CSG<S> {
        let (nx, ny) = resolution;
        if balls.is_empty() || nx < 2 || ny < 2 {
            return CSG::new();
        }
    
        // 1) Compute bounding box around all metaballs
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;
        for (center, r) in balls {
            let rr = *r + padding;
            if center.x - rr < min_x { min_x = center.x - rr; }
            if center.x + rr > max_x { max_x = center.x + rr; }
            if center.y - rr < min_y { min_y = center.y - rr; }
            if center.y + rr > max_y { max_y = center.y + rr; }
        }
    
        let dx = (max_x - min_x) / (nx as Real - 1.0);
        let dy = (max_y - min_y) / (ny as Real - 1.0);
    
        // 2) Fill a grid with the summed “influence” minus iso_value
        fn scalar_field(balls: &[(nalgebra::Point2<Real>, Real)], x: Real, y: Real) -> Real {
            let mut v = 0.0;
            for (c, r) in balls {
                let dx = x - c.x;
                let dy = y - c.y;
                let dist_sq = dx*dx + dy*dy + EPSILON;
                v += (r*r) / dist_sq;
            }
            v
        }
    
        let mut grid = vec![0.0; nx * ny];
        let index = |ix: usize, iy: usize| -> usize { iy*nx + ix };
        for iy in 0..ny {
            let yv = min_y + (iy as Real)*dy;
            for ix in 0..nx {
                let xv = min_x + (ix as Real)*dx;
                let val = scalar_field(balls, xv, yv) - iso_value;
                grid[index(ix, iy)] = val;
            }
        }
    
        // 3) Marching squares -> line segments
        let mut contours = Vec::<LineString<Real>>::new();
    
        // Interpolator:
        let interpolate = |(x1, y1, v1): (Real,Real,Real),
                        (x2, y2, v2): (Real,Real,Real)| -> (Real,Real) {
            let denom = (v2 - v1).abs();
            if denom < EPSILON {
                (x1, y1)
            } else {
                let t = -v1 / (v2 - v1); // crossing at 0
                (x1 + t*(x2 - x1), y1 + t*(y2 - y1))
            }
        };
    
        for iy in 0..(ny - 1) {
            let y0 = min_y + (iy as Real)*dy;
            let y1 = min_y + ((iy+1) as Real)*dy;
    
            for ix in 0..(nx - 1) {
                let x0 = min_x + (ix as Real)*dx;
                let x1 = min_x + ((ix+1) as Real)*dx;
    
                let v0 = grid[index(ix,   iy  )];
                let v1 = grid[index(ix+1, iy  )];
                let v2 = grid[index(ix+1, iy+1)];
                let v3 = grid[index(ix,   iy+1)];
    
                // classification
                let mut c = 0u8;
                if v0 >= 0.0 { c |= 1; }
                if v1 >= 0.0 { c |= 2; }
                if v2 >= 0.0 { c |= 4; }
                if v3 >= 0.0 { c |= 8; }
                if c == 0 || c == 15 {
                    continue; // no crossing
                }
    
                let corners = [
                    (x0, y0, v0),
                    (x1, y0, v1),
                    (x1, y1, v2),
                    (x0, y1, v3),
                ];
    
                let mut pts = Vec::new();
                // function to check each edge
                let mut check_edge = |mask_a: u8, mask_b: u8, a: usize, b: usize| {
                    let inside_a = (c & mask_a) != 0;
                    let inside_b = (c & mask_b) != 0;
                    if inside_a != inside_b {
                        let (px, py) = interpolate(corners[a], corners[b]);
                        pts.push((px, py));
                    }
                };
    
                check_edge(1, 2, 0, 1);
                check_edge(2, 4, 1, 2);
                check_edge(4, 8, 2, 3);
                check_edge(8, 1, 3, 0);
    
                // we might get 2 intersection points => single line segment
                // or 4 => two line segments, etc.
                // For simplicity, we just store them in a small open polyline:
                if pts.len() >= 2 {
                    let mut pl = LineString::new(vec![]);
                    for &(px, py) in &pts {
                        pl.0.push(coord!{x: px, y: py});
                    }
                    // Do not close. These are just line segments from this cell.
                    contours.push(pl);
                }
            }
        }
    
        // 4) Convert these line segments into geo::LineStrings or geo::Polygons if closed.
        //    We store them in a GeometryCollection.
        let mut gc = GeometryCollection::default();
    
        // If you want to unify them into continuous lines, you can do so,
        // but for now let's just push each as a separate line or polygon if closed.
        for pl in contours {
            let n = pl.coords_count();
            if n < 2 { continue; }
    
            // gather coords
            let coords: Vec<_> = (0..n).map(|i| {
                let v = pl.0[i];
                (v.x, v.y)
            }).collect();
    
            // Check if first == last => closed
            let closed = {
                let first = coords[0];
                let last  = coords[n-1];
                let dx = first.0 - last.0;
                let dy = first.1 - last.1;
                (dx*dx + dy*dy).sqrt() < EPSILON
            };
    
            if closed {
                // Turn it into a Polygon
                let polygon_2d = GeoPolygon::new(LineString::from(coords.clone()), vec![]);
                gc.0.push(Geometry::Polygon(polygon_2d));
            } else {
                // It's an open line
                gc.0.push(Geometry::LineString(LineString::from(coords)));
            }
        }
    
        CSG::from_geo(gc, metadata)
    }

    /// Create a 2D supershape in the XY plane, approximated by `segments` edges.
    /// The superformula parameters are typically:
    ///   r(θ) = [ (|cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3) ^ (-1/n1) ]
    /// Adjust as needed for your use-case.
    pub fn supershape(
        a: Real,
        b: Real,
        m: Real,
        n1: Real,
        n2: Real,
        n3: Real,
        segments: usize,
        metadata: Option<S>
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
    
        // The typical superformula radius function
        fn supershape_r(
            theta: Real,
            a: Real, b: Real,
            m: Real, n1: Real, n2: Real, n3: Real
        ) -> Real {
            // r(θ) = [ |cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3 ]^(-1/n1)
            let t = m*theta*0.25;
            let cos_t = t.cos().abs();
            let sin_t = t.sin().abs();
            let term1 = (cos_t/a).powf(n2);
            let term2 = (sin_t/b).powf(n3);
            (term1 + term2).powf(-1.0/n1)
        }
    
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let frac = i as Real / (segments as Real);
            let theta = TAU * frac;
            let r = supershape_r(theta, a, b, m, n1, n2, n3);
    
            let x = r * theta.cos();
            let y = r * theta.sin();
            coords.push((x, y));
        }
        // close it
        coords.push(coords[0]);
    
        let polygon_2d = geo::Polygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon_2d)]), metadata)
    }
    
    /// Creates a 2D circle with a rectangular keyway slot cut out on the +X side.
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());
    
        // 2. Construct the keyway rectangle:
        //    - width along X = key_depth
        //    - height along Y = key_width
        //    - its right edge at x = +radius
        //    - so it spans from x = (radius - key_depth) to x = radius
        //    - and from y = -key_width/2 to y = +key_width/2
        let key_rect = CSG::square(key_depth, key_width, metadata.clone())
            .translate(radius - key_depth, -key_width * 0.5, 0.0);
    
        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    ///   - If flat_dist == 0.0 => chord passes through center => a half-circle
    ///   - If flat_dist < radius => chord is inside the circle => typical "D" shape
    ///
    /// Solve for distance from center using width of flat:
    /// let half_c = chord_len / 2.0;
    /// let flat_dist = (radius*radius - half_c*half_c).sqrt();
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());
    
        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        //    (i.e., we remove that portion to create a chord).
        //    Width = 2*radius is plenty to cover the circle’s X-range.
        //    Height = large enough, we just shift it so top edge is at y = -flat_dist.
        //    So that rectangle covers from y = -∞ up to y = -flat_dist.
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(2.0 * radius, cutter_height, metadata.clone())
            .translate(-radius, -cutter_height, 0.0) // put its bottom near "negative infinity"
            .translate(0.0, -flat_dist, 0.0);        // now top edge is at y = -flat_dist
    
        // 3. Subtract to produce the flat chord
        circle.difference(&rect_cutter)
    }

    /// Circle with two parallel flat chords on opposing sides (e.g., "double D" shape).
    /// `radius`   => circle radius
    /// `segments` => how many segments in the circle approximation
    /// `flat_dist` => half-distance between flats measured from the center.
    ///   - chord at y=+flat_dist  and  chord at y=-flat_dist
    pub fn circle_with_two_flats(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());
    
        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = 9999.0;
        let top_rect = CSG::square(2.0 * radius, cutter_height, metadata.clone())
            // place bottom at y=flat_dist
            .translate(-radius, flat_dist, 0.0);
    
        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = CSG::square(2.0 * radius, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(-radius, -cutter_height - flat_dist, 0.0);
    
        // 4. Subtract both
        let with_top_flat = circle.difference(&top_rect);
        let with_both_flats = with_top_flat.difference(&bottom_rect);
    
        with_both_flats
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

                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }
        CSG::from_polygons(&polygons)
    }
    
    /// Constructs a frustum between `start` and `end` with bottom radius = `radius1` and
    /// top radius = `radius2`. In the normal case, it creates side quads and cap triangles.
    /// However, if one of the radii is 0 (within EPSILON), then the degenerate face is treated
    /// as a single point and the side is stitched using triangles.
    /// 
    /// # Parameters
    /// - `start`: the center of the bottom face
    /// - `end`: the center of the top face
    /// - `radius1`: the radius at the bottom face
    /// - `radius2`: the radius at the top face
    /// - `segments`: number of segments around the circle (must be ≥ 3)
    /// - `metadata`: optional metadata
    /// 
    /// # Example
    /// ```
    /// let bottom = Point3::new(0.0, 0.0, 0.0);
    /// let top = Point3::new(0.0, 0.0, 5.0);
    /// // This will create a cone (bottom degenerate) because radius1 is 0:
    /// let cone = CSG::frustrum_ptp_special(bottom, top, 0.0, 2.0, 32, None);
    /// ```
    pub fn frustrum_ptp(
        start: Point3<Real>,
        end: Point3<Real>,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Compute the axis and check that start and end do not coincide.
        let s = start.coords;
        let e = end.coords;
        let ray = e - s;
        if ray.norm_squared() < EPSILON {
            return CSG::new();
        }
        let axis_z = ray.normalize();
        // Pick an axis not parallel to axis_z.
        let axis_x = if axis_z.y.abs() > 0.5 {
            Vector3::x()
        } else {
            Vector3::y()
        }
        .cross(&axis_z)
        .normalize();
        let axis_y = axis_x.cross(&axis_z).normalize();
    
        // The cap centers for the bottom and top.
        let start_v = Vertex::new(start, -axis_z);
        let end_v = Vertex::new(end, axis_z);
    
        // A closure that returns a vertex on the lateral surface.
        // For a given stack (0.0 for bottom, 1.0 for top), slice (fraction along the circle),
        // and a normal blend factor (used for cap smoothing), compute the vertex.
        let point = |stack: Real, slice: Real, normal_blend: Real| {
            // Linear interpolation of radius.
            let r = radius1 * (1.0 - stack) + radius2 * stack;
            let angle = slice * TAU;
            let radial_dir = axis_x * angle.cos() + axis_y * angle.sin();
            let pos = s + ray * stack + radial_dir * r;
            let normal = radial_dir * (1.0 - normal_blend.abs()) + axis_z * normal_blend;
            Vertex::new(Point3::from(pos), normal.normalize())
        };
    
        let mut polygons = Vec::new();
    
        // Special-case flags for degenerate faces.
        let bottom_degenerate = radius1.abs() < EPSILON;
        let top_degenerate = radius2.abs() < EPSILON;
    
        // If both faces are degenerate, we cannot build a meaningful volume.
        if bottom_degenerate && top_degenerate {
            return CSG::new();
        }
    
        // For each slice of the circle (0..segments)
        for i in 0..segments {
            let slice0 = i as Real / segments as Real;
            let slice1 = (i + 1) as Real / segments as Real;
    
            // In the normal frustrum_ptp, we always add a bottom cap triangle (fan) and a top cap triangle.
            // Here, we only add the cap triangle if the corresponding radius is not degenerate.
            if !bottom_degenerate {
                // Bottom cap: a triangle fan from the bottom center to two consecutive points on the bottom ring.
                polygons.push(Polygon::new(
                    vec![
                        start_v.clone(),
                        point(0.0, slice0, -1.0),
                        point(0.0, slice1, -1.0),
                    ],
                    metadata.clone(),
                ));
            }
            if !top_degenerate {
                // Top cap: a triangle fan from the top center to two consecutive points on the top ring.
                polygons.push(Polygon::new(
                    vec![
                        end_v.clone(),
                        point(1.0, slice1, 1.0),
                        point(1.0, slice0, 1.0),
                    ],
                    metadata.clone(),
                ));
            }
    
            // For the side wall, we normally build a quad spanning from the bottom ring (stack=0)
            // to the top ring (stack=1). If one of the rings is degenerate, that ring reduces to a single point.
            // In that case, we output a triangle.
            if bottom_degenerate {
                // Bottom is a point (start_v); create a triangle from start_v to two consecutive points on the top ring.
                polygons.push(Polygon::new(
                    vec![
                        start_v.clone(),
                        point(1.0, slice0, 0.0),
                        point(1.0, slice1, 0.0),
                    ],
                    metadata.clone(),
                ));
            } else if top_degenerate {
                // Top is a point (end_v); create a triangle from two consecutive points on the bottom ring to end_v.
                polygons.push(Polygon::new(
                    vec![
                        point(0.0, slice1, 0.0),
                        point(0.0, slice0, 0.0),
                        end_v.clone(),
                    ],
                    metadata.clone(),
                ));
            } else {
                // Normal case: both rings are non-degenerate. Use a quad for the side wall.
                polygons.push(Polygon::new(
                    vec![
                        point(0.0, slice1, 0.0),
                        point(0.0, slice0, 0.0),
                        point(1.0, slice0, 0.0),
                        point(1.0, slice1, 0.0),
                    ],
                    metadata.clone(),
                ));
            }
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
            let mut poly = Polygon::new(face_vertices, metadata.clone());

            // Set each vertex normal to match the polygon’s plane normal,
            let plane_normal = poly.plane.normal;
            for v in &mut poly.vertices {
                v.normal = plane_normal;
            }
            polygons.push(poly);
        }

        CSG::from_polygons(&polygons)
    }
    
    /// Creates a 3D "egg" shape by revolving the existing 2D `egg_outline` profile.
    ///
    /// # Parameters
    /// - `width`: The "width" of the 2D egg outline.
    /// - `length`: The "length" (height) of the 2D egg outline.
    /// - `revolve_segments`: Number of segments for the revolution.
    /// - `outline_segments`: Number of segments for the 2D egg outline itself.
    /// - `metadata`: Optional metadata.
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let egg_2d = Self::egg_outline(width, length, outline_segments, metadata.clone());
        
        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, cutter_height, metadata.clone())
            .translate(-cutter_height, -cutter_height/2.0, 0.0);
    
        let half_egg = egg_2d.difference(&rect_cutter);
        
        half_egg.rotate_extrude(360.0, revolve_segments).convex_hull()
    }
    
    /// Creates a 3D "teardrop" solid by revolving the existing 2D `teardrop` profile 360° around the Y-axis (via rotate_extrude).
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, cutter_height, metadata.clone())
            .translate(-cutter_height, -cutter_height/2.0, 0.0);
    
        let half_teardrop = td_2d.difference(&rect_cutter);

        // revolve 360 degrees
        half_teardrop.rotate_extrude(360.0, revolve_segments).convex_hull()
    }

    /// Creates a 3D "teardrop cylinder" by extruding the existing 2D `teardrop` in the Z+ axis.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());
        td_2d.extrude(height).convex_hull()
    }
    
    /// Creates an ellipsoid by taking a sphere of radius=1 and scaling it by (rx, ry, rz).
    ///
    /// # Parameters
    /// - `rx`: X-axis radius.
    /// - `ry`: Y-axis radius.
    /// - `rz`: Z-axis radius.
    /// - `segments`: Number of horizontal segments.
    /// - `stacks`: Number of vertical stacks.
    /// - `metadata`: Optional metadata.
    pub fn ellipsoid(
        rx: Real,
        ry: Real,
        rz: Real,
        segments: usize,
        stacks: usize,
        metadata: Option<S>,
    ) -> Self {
        let base_sphere = Self::sphere(1.0, segments, stacks, metadata.clone());
        base_sphere.scale(rx, ry, rz)
    }
    
    /// Creates an arrow CSG. The arrow is composed of:
    ///   - a cylindrical shaft, and
    ///   - a cone–like head (a frustrum from a larger base to a small tip)
    /// built along the canonical +Z axis. The arrow is then rotated so that +Z aligns with the given
    /// direction, and finally translated so that either its base (if `orientation` is false)
    /// or its tip (if `orientation` is true) is located at `start`.
    ///
    /// The arrow’s dimensions (shaft radius, head dimensions, etc.) are scaled proportionally to the
    /// total arrow length (the norm of the provided direction).
    ///
    /// # Parameters
    /// - `start`: the reference point (base or tip, depending on orientation)
    /// - `direction`: the vector defining arrow length and intended pointing direction
    /// - `segments`: number of segments for approximating the cylinder and frustrum
    /// - `orientation`: when false (default) the arrow points away from start (its base is at start);
    ///                        when true the arrow points toward start (its tip is at start).
    /// - `metadata`: optional metadata for the generated polygons.
    pub fn arrow(
        start: Point3<Real>,
        direction: Vector3<Real>,
        segments: usize,
        orientation: bool,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Compute the arrow's total length.
        let arrow_length = direction.norm();
        if arrow_length < EPSILON {
            return CSG::new();
        }
        // Compute the unit direction.
        let unit_dir = direction / arrow_length;
    
        // Define proportions:
        // - Arrow head occupies 20% of total length.
        // - Shaft occupies the remainder.
        let head_length = arrow_length * 0.2;
        let shaft_length = arrow_length - head_length;
    
        // Define thickness parameters proportional to the arrow length.
        let shaft_radius = arrow_length * 0.03;      // shaft radius
        let head_base_radius = arrow_length * 0.06;    // head base radius (wider than shaft)
        let tip_radius = arrow_length * 0.0;         // tip radius (nearly a point)
    
        // Build the shaft as a vertical cylinder along Z from 0 to shaft_length.
        let shaft = CSG::cylinder(shaft_radius, shaft_length, segments, metadata.clone());
    
        // Build the arrow head as a frustrum from z = shaft_length to z = shaft_length + head_length.
        let head = CSG::frustrum_ptp(
            Point3::new(0.0, 0.0, shaft_length),
            Point3::new(0.0, 0.0, shaft_length + head_length),
            head_base_radius,
            tip_radius,
            segments,
            metadata.clone(),
        );
    
        // Combine the shaft and head.
        let mut canonical_arrow = shaft.union(&head);
    
        // If the arrow should point toward start, mirror the geometry in canonical space.
        // The mirror transform about the plane z = arrow_length/2 maps any point (0,0,z) to (0,0, arrow_length - z).
        if orientation {
            let l = arrow_length;
            let mirror_mat: Matrix4<Real> =
                Translation3::new(0.0, 0.0, l / 2.0).to_homogeneous() *
                Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0)) *
                Translation3::new(0.0, 0.0, -l / 2.0).to_homogeneous();
            canonical_arrow = canonical_arrow.transform(&mirror_mat).inverse();
        }
        // In both cases, we now have a canonical arrow that extends from z=0 to z=arrow_length.
        // For orientation == false, z=0 is the base.
        // For orientation == true, after mirroring z=0 is now the tip.
    
        // Compute the rotation that maps the canonical +Z axis to the provided direction.
        let z_axis = Vector3::z();
        let rotation = Rotation3::rotation_between(&z_axis, &unit_dir)
            .unwrap_or_else(|| Rotation3::identity());
        let rot_mat: Matrix4<Real> = rotation.to_homogeneous();
    
        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);
    
        // Finally, translate the arrow so that the anchored vertex (canonical (0,0,0)) moves to 'start'.
        // In the false case, (0,0,0) is the base (arrow extends from start to start+direction).
        // In the true case, after mirroring, (0,0,0) is the tip (arrow extends from start to start+direction).
        let final_arrow = rotated_arrow.translate(start.x, start.y, start.z);
    
        final_arrow
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

    /// Linearly extrude this (2D) shape in the +Z direction by `height`.
    ///
    /// This is just a convenience wrapper around extrude_vector using Vector3::new(0.0, 0.0, height)
    pub fn extrude(&self, height: Real) -> CSG<S> {
        self.extrude_vector(Vector3::new(0.0, 0.0, height))
    }

    /// Linearly extrude any 2D geometry (Polygons, MultiPolygons, or sub-geometries
    /// in a GeometryCollection) in `self.geometry` along the given `direction`.
    ///
    /// Builds top, bottom, and side polygons in 3D, storing them in `csg.polygons`.
    /// Returns a new CSG containing these extruded polygons (plus any existing 3D polygons
    /// you already had in `self.polygons`).
    pub fn extrude_vector(&self, direction: Vector3<Real>) -> CSG<S> {
        // If the direction is near zero length, nothing to extrude:
        if direction.norm() < EPSILON {
            return self.clone(); // or return an empty CSG
        }

        // Collect the new 3D polygons generated by extrusion:
        let mut new_polygons = Vec::new();

        // A helper to handle any Geometry
        fn extrude_geometry<S: Clone + Send + Sync>(
            geom: &geo::Geometry<Real>,
            direction: Vector3<Real>,
            metadata: &Option<S>,
            out_polygons: &mut Vec<Polygon<S>>,
        )
        {
            match geom {
                geo::Geometry::Polygon(poly) => {
                    let exterior_coords: Vec<[Real; 2]> =
                        poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                    let interior_rings: Vec<Vec<[Real; 2]>> = poly
                        .interiors()
                        .into_iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();

                    // bottom
                    let bottom_tris =
                        CSG::<()>::tessellate_2d(&exterior_coords, &interior_rings.iter().map(|r| &r[..]).collect::<Vec<_>>());
                    for tri in bottom_tris {
                        let v0 = Vertex::new(tri[2], -Vector3::z());
                        let v1 = Vertex::new(tri[1], -Vector3::z());
                        let v2 = Vertex::new(tri[0], -Vector3::z());
                        out_polygons.push(Polygon::new(vec![v0, v1, v2], metadata.clone()));
                    }
                    // top
                    let top_tris =
                        CSG::<()>::tessellate_2d(&exterior_coords, &interior_rings.iter().map(|r| &r[..]).collect::<Vec<_>>());
                    for tri in top_tris {
                        let p0 = tri[0] + direction;
                        let p1 = tri[1] + direction;
                        let p2 = tri[2] + direction;
                        let v0 = Vertex::new(p0, Vector3::z());
                        let v1 = Vertex::new(p1, Vector3::z());
                        let v2 = Vertex::new(p2, Vector3::z());
                        out_polygons.push(Polygon::new(vec![v0, v1, v2], metadata.clone()));
                    }

                    // sides
                    let all_rings = std::iter::once(poly.exterior()).chain(poly.interiors());
                    for ring in all_rings {
                        let coords: Vec<_> = ring.coords_iter().collect();
                        let n = coords.len();
                        if n < 2 {
                            continue;
                        }
                        for i in 0..(n - 1) {
                            let j = i + 1;
                            let c_i = coords[i];
                            let c_j = coords[j];
                            let b_i = Point3::new(c_i.x, c_i.y, 0.0);
                            let b_j = Point3::new(c_j.x, c_j.y, 0.0);
                            let t_i = b_i + direction;
                            let t_j = b_j + direction;
                            out_polygons.push(Polygon::new(
                                vec![
                                    Vertex::new(b_i, Vector3::zeros()),
                                    Vertex::new(b_j, Vector3::zeros()),
                                    Vertex::new(t_j, Vector3::zeros()),
                                    Vertex::new(t_i, Vector3::zeros()),
                                ],
                                metadata.clone(),
                            ));
                        }
                    }
                }
                geo::Geometry::MultiPolygon(mp) => {
                    for poly in &mp.0 {
                        extrude_geometry(&geo::Geometry::Polygon(poly.clone()), direction, metadata, out_polygons);
                    }
                }
                geo::Geometry::GeometryCollection(gc) => {
                    for sub in &gc.0 {
                        extrude_geometry(sub, direction, metadata, out_polygons);
                    }
                }
                // Other geometry types (LineString, Point, etc.) are skipped or could be handled differently:
                _ => { /* skip */ }
            }
        }

        // -- 1) Extrude the polygons from self.polygons if you also want to re-extrude them.
        //    (Often, your `polygons` are already 3D, so you might skip extruding them again.)
        //    We'll skip it here for clarity.  If you do want to extrude existing 2D polygons
        //    in `self.polygons`, you'd add similar logic.

        // -- 2) Extrude from self.geometry (the `geo::GeometryCollection`).
        for geom in &self.geometry {
            extrude_geometry(geom, direction, &self.metadata, &mut new_polygons);
        }

        // Combine new extruded polygons with any existing 3D polygons:
        let mut final_polygons = self.polygons.clone();
        final_polygons.extend(new_polygons);

        // Return a new CSG
        CSG {
            polygons: final_polygons,
            geometry: self.geometry.clone(),
            metadata: self.metadata.clone(),
        }
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
                bottom.metadata.clone(), // carry over bottom polygon metadata
            );
            polygons.push(side_poly);
        }

        CSG::from_polygons(&polygons)
    }
    
    /*
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
    pub fn linear_extrude(
        shape: &CCShape<Real>,
        direction: Vector3<Real>,
        twist_degs: Real,
        segments: usize,
        scale_top: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let mut polygons_3d = Vec::new();
        if segments < 1 {
            return CSG::new();
        }
        let height = direction.norm();
        if height < EPSILON {
            // no real extrusion
            return CSG::new();
        }
    
        // Step 1) Build a series of “transforms” from bottom=0..top=height, subdivided into `segments`.
        //   For each i in [0..=segments], compute fraction f and:
        //   - scale in XY => s_i
        //   - twist about Z => rot_i
        //   - translate in Z => z_i
        //
        //   We'll store each “slice” in 3D form as a Vec<Vec<Point3<Real>>>,
        //   i.e. one 3D polyline for each boundary or hole in the shape.
        let mut slices: Vec<Vec<Vec<Point3<Real>>>> = Vec::with_capacity(segments + 1);
        // The axis to rotate around is the unit of `direction`. We'll do final alignment after constructing them along +Z.
        let axis_dir = direction.normalize();
    
        for i in 0..=segments {
            let f = i as Real / segments as Real;
            let s_i = 1.0 + (scale_top - 1.0) * f;  // lerp(1, scale_top, f)
            let twist_rad = twist_degs.to_radians() * f;
            let z_i = height * f;
    
            // Build transform T = Tz * Rz * Sxy
            //  - scale in XY
            //  - twist around Z
            //  - translate in Z
            let mat_scale = Matrix4::new_nonuniform_scaling(&Vector3::new(s_i, s_i, 1.0));
            let mat_rot = Rotation3::from_axis_angle(&Vector3::z_axis(), twist_rad).to_homogeneous();
            let mat_trans = Translation3::new(0.0, 0.0, z_i).to_homogeneous();
            let slice_mat = mat_trans * mat_rot * mat_scale;
    
            let slice_3d = project_shape_3d(shape, &slice_mat);
            slices.push(slice_3d);
        }
    
        // Step 2) “Stitch” consecutive slices to form side polygons.  
        // For each pair of slices[i], slices[i+1], for each boundary polyline j,
        // connect edges. We assume each polyline has the same vertex_count in both slices.
        // (If the shape is closed, we do wrap edges [n..0].)
        // Then we optionally build bottom & top caps if the polylines are closed.
    
        // a) bottom + top caps, similar to extrude_vector approach
        //    For slices[0], build a “bottom” by triangulating in XY, flipping normal.
        //    For slices[segments], build a “top” by normal up.
        //
        //    But we only do it if each boundary is closed. 
        //    We must group CCW with matching holes. This is the same logic as `extrude_vector`.
    
        // We'll do a small helper that triangulates shape in 2D, then lifts that triangulation to slice_3d.
        // You can re‐use the logic from `extrude_vector`.
    
        // Build the “bottom” from slices[0] if polylines are all or partially closed
        polygons_3d.extend(
            build_caps_from_slice(shape, &slices[0], true, metadata.clone())
        );
        // Build the “top” from slices[segments] 
        polygons_3d.extend(
            build_caps_from_slice(shape, &slices[segments], false, metadata.clone())
        );
    
        // b) side walls
        for i in 0..segments {
            let bottom_slice = &slices[i];
            let top_slice = &slices[i + 1];
    
            // We know bottom_slice has shape.ccw_plines.len() + shape.cw_plines.len() polylines
            // in the same order. Each polyline has the same vertex_count as in top_slice.
            // So we can do a direct 1:1 match: bottom_slice[j] <-> top_slice[j].
            for (pline_idx, bot3d) in bottom_slice.iter().enumerate() {
                let top3d = &top_slice[pline_idx];
                if bot3d.len() < 2 {
                    continue;
                }
                // is it closed? We can check shape’s corresponding polyline
                let is_closed = if pline_idx < shape.ccw_plines.len() {
                    shape.ccw_plines[pline_idx].polyline.is_closed()
                } else {
                    shape.cw_plines[pline_idx - shape.ccw_plines.len()].polyline.is_closed()
                };
                let n = bot3d.len();
                let edge_count = if is_closed { n } else { n - 1 };
    
                for k in 0..edge_count {
                    let k_next = (k + 1) % n;
                    let b_i = bot3d[k];
                    let b_j = bot3d[k_next];
                    let t_i = top3d[k];
                    let t_j = top3d[k_next];
    
                    let poly_side = Polygon::new(
                        vec![
                            Vertex::new(b_i, Vector3::zeros()),
                            Vertex::new(b_j, Vector3::zeros()),
                            Vertex::new(t_j, Vector3::zeros()),
                            Vertex::new(t_i, Vector3::zeros()),
                        ],
                        metadata.clone(),
                    );
                    polygons_3d.push(poly_side);
                }
            }
        }
    
        // Step 3) If direction is not along +Z, rotate final mesh so +Z aligns with your direction
        // (This is optional or can be done up front. Typical OpenSCAD style is to do everything 
        // along +Z, then rotate the final.)
        if (axis_dir - Vector3::z()).norm() > EPSILON {
            // rotate from +Z to axis_dir
            let rot_axis = Vector3::z().cross(&axis_dir);
            let sin_theta = rot_axis.norm();
            if sin_theta > EPSILON {
                let cos_theta = Vector3::z().dot(&axis_dir);
                let angle = cos_theta.acos();
                let rot = Rotation3::from_axis_angle(&Unit::new_normalize(rot_axis), angle);
                let mat = rot.to_homogeneous();
                // transform the polygons
                let mut final_polys = Vec::with_capacity(polygons_3d.len());
                for mut poly in polygons_3d {
                    for v in &mut poly.vertices {
                        let pos4 = mat * nalgebra::Vector4::new(v.pos.x, v.pos.y, v.pos.z, 1.0);
                        v.pos = Point3::new(pos4.x / pos4.w, pos4.y / pos4.w, pos4.z / pos4.w);
                    }
                    poly.set_new_normal();
                    final_polys.push(poly);
                }
                return CSG::from_polygons(&final_polys);
            }
        }
    
        // otherwise, just return as is
        CSG::from_polygons(&polygons_3d)
    }
    */

    /// Rotate-extrude (revolve) only the Polygon and MultiPolygon geometry in `self.geometry`
    /// around the Y-axis from 0..`angle_degs` in `segments` steps, producing side walls
    /// in an orientation consistent with the polygon's winding.
    ///
    /// - Ignores `self.polygons`.
    /// - Returns a new CSG containing **only** the newly extruded side polygons (no end caps).
    /// - `angle_degs`: how far to revolve, in degrees (e.g. 360 for a full revolve).
    /// - `segments`: number of subdivisions around the revolve.
    ///
    /// # Key Points
    /// - Only 2D geometry in `self.geometry` is used. Any `self.polygons` are ignored.
    /// - Axis of revolution: **Y-axis**. We treat each ring's (x,y) -> revolve_around_y(x,y,theta).
    /// - Exterior rings (CCW in Geo) produce outward-facing side polygons.
    /// - Interior rings (CW) produce inward-facing side polygons ("holes").
    /// - If `angle_degs < 360`, we add **two caps**: one at angle=0, one at angle=angle_degs.
    ///   - Cap orientation is set so that normals face outward, consistent with a solid.
    /// - Returns a new CSG with `.polygons` containing only the side walls + any caps.
    ///   The `.geometry` is empty, i.e. `GeometryCollection::default()`.
    pub fn rotate_extrude(&self, angle_degs: Real, segments: usize) -> CSG<S> {
        if segments < 2 {
            panic!("rotate_extrude requires at least 2 segments.");
        }

        let angle_radians = angle_degs.to_radians();
        let mut new_polygons = Vec::new();

        // A small helper to revolve a point (x,y) in the XY plane around the Y-axis by theta.
        // The output is a 3D point (X, Y, Z).
        fn revolve_around_y(x: Real, y: Real, theta: Real) -> Point3<Real> {
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            // Map (x, y, 0) => ( x*cos θ, y, x*sin θ )
            Point3::new(x * cos_t, y, x * sin_t)
        }

        // Another helper to determine if a ring (LineString) is CCW or CW in Geo.
        // In `geo`, ring.exterior() is CCW for an outer boundary, CW for holes.
        // If the signed area > 0 => CCW; < 0 => CW.
        fn is_ccw(ring: &LineString<Real>) -> bool {
            let poly = GeoPolygon::new(ring.clone(), vec![]);
            poly.signed_area() > 0.0
        }

        // A helper to extrude one ring of coordinates (including the last->first if needed),
        // pushing its side polygons into `out_polygons`.
        // - `ring_coords`: The ring’s sequence of points. Usually closed (last=first).
        // - `ring_is_ccw`: true if it's an exterior ring, false if interior/hole.
        // - `angle_radians`: total revolve sweep in radians.
        // - `segments`: how many discrete slices around the revolve.
        // - `metadata`: user metadata to attach to side polygons.
        fn revolve_ring<S: Clone + Send + Sync>(
            ring_coords: &[geo::Coord<Real>],
            ring_is_ccw: bool,
            angle_radians: Real,
            segments: usize,
            metadata: &Option<S>,
        ) -> Vec<Polygon<S>> {
            if ring_coords.len() < 2 {
                return vec![];
            }
            
            let mut out_polygons = Vec::new();
            // Typically the last point = first point for a closed ring.
            // We'll iterate over each edge i..i+1, and revolve them around by segments slices.

            // The revolve step size in radians:
            let step = angle_radians / (segments as Real);

            // For each edge in the ring:
            for i in 0..(ring_coords.len() - 1) {
                let c_i = ring_coords[i];
                let c_j = ring_coords[i + 1];

                // If these two points are the same, skip degenerate edge
                if (c_i.x - c_j.x).abs() < EPSILON && (c_i.y - c_j.y).abs() < EPSILON {
                    continue;
                }

                // For each revolve slice j..j+1
                for s in 0..segments {
                    let th0 = s as Real * step;
                    let th1 = (s as Real + 1.0) * step;

                    // revolve bottom edge endpoints at angle th0
                    let b_i = revolve_around_y(c_i.x, c_i.y, th0);
                    let b_j = revolve_around_y(c_j.x, c_j.y, th0);
                    // revolve top edge endpoints at angle th1
                    let t_i = revolve_around_y(c_i.x, c_i.y, th1);
                    let t_j = revolve_around_y(c_j.x, c_j.y, th1);

                    // Build a 4-vertex side polygon for the ring edge.  
                    // The orientation depends on ring_is_ccw:  
                    //    If CCW => outward walls -> [b_i, b_j, t_j, t_i]  
                    //    If CW  => reverse it -> [b_j, b_i, t_i, t_j]  
                    let quad_verts = if ring_is_ccw {
                        vec![b_i, b_j, t_j, t_i]
                    } else {
                        vec![b_j, b_i, t_i, t_j]
                    }
                    .into_iter()
                    .map(|pos| Vertex::new(pos, Vector3::zeros()))
                    .collect();

                    out_polygons.push(Polygon::new(quad_verts, metadata.clone()));
                }
            }
            out_polygons
        }
        
        // Build a single “cap” polygon from ring_coords at a given angle (0 or angle_radians).
        //  - revolve each 2D point by `angle`, produce a 3D ring
        //  - if `flip` is true, reverse the ring so the normal is inverted
        fn build_cap_polygon<S: Clone + Send + Sync>(
            ring_coords: &[geo::Coord<Real>],
            angle: Real,
            flip: bool,
            metadata: &Option<S>,
        ) -> Option<Polygon<S>> {
            if ring_coords.len() < 3 {
                return None;
            }
            // revolve each coordinate at the given angle
            let mut pts_3d: Vec<_> = ring_coords
                .iter()
                .map(|c| revolve_around_y(c.x, c.y, angle))
                .collect();

            // ensure closed if the ring wasn't strictly closed
            // (the last point in a Geo ring is typically the same as the first)
            let last = pts_3d.last().unwrap();
            let first = pts_3d.first().unwrap();
            if (last.x - first.x).abs() > EPSILON
                || (last.y - first.y).abs() > EPSILON
                || (last.z - first.z).abs() > EPSILON
            {
                pts_3d.push(*first);
            }

            // Turn into Vertex
            let mut verts: Vec<_> = pts_3d
                .into_iter()
                .map(|p3| Vertex::new(p3, Vector3::zeros()))
                .collect();

            // If flip == true, reverse them and flip each vertex
            if flip {
                verts.reverse();
                for v in &mut verts {
                    v.flip();
                }
            }

            // Build the polygon
            let poly = Polygon::new(verts, metadata.clone());
            Some(poly)
        }

        //----------------------------------------------------------------------
        // 2) Iterate over each geometry (Polygon or MultiPolygon),
        //    revolve the side walls, and possibly add caps if angle_degs < 360.
        //----------------------------------------------------------------------
        let full_revolve = (angle_degs - 360.0).abs() < EPSILON; // or angle_degs >= 359.999..., etc.
        let do_caps = !full_revolve && (angle_degs > 0.0);

        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Exterior ring
                    let ext_ring = poly2d.exterior();
                    let ext_ccw = is_ccw(ext_ring);

                    // (A) side walls
                    new_polygons.extend(revolve_ring(
                        &ext_ring.0,
                        ext_ccw,
                        angle_radians,
                        segments,
                        &self.metadata,
                    ));

                    // (B) cap(s) if partial revolve
                    if do_caps {
                        // start-cap at angle=0
                        //   flip if ext_ccw == true
                        if let Some(cap) = build_cap_polygon(
                            &ext_ring.0,
                            0.0,
                            ext_ccw, // exterior ring => flip the start cap
                            &self.metadata,
                        ) {
                            new_polygons.push(cap);
                        }

                        // end-cap at angle= angle_radians
                        //   flip if ext_ccw == false
                        if let Some(cap) = build_cap_polygon(
                            &ext_ring.0,
                            angle_radians,
                            !ext_ccw, // exterior ring => keep normal orientation for end
                            &self.metadata,
                        ) {
                            new_polygons.push(cap);
                        }
                    }

                    // Interior rings (holes)
                    for hole in poly2d.interiors() {
                        let hole_ccw = is_ccw(hole);
                        new_polygons.extend(revolve_ring(
                            &hole.0,
                            hole_ccw,
                            angle_radians,
                            segments,
                            &self.metadata,
                        ));
                    }
                }

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Each Polygon inside
                    for poly2d in &mpoly.0 {
                        let ext_ring = poly2d.exterior();
                        let ext_ccw = is_ccw(ext_ring);

                        new_polygons.extend(revolve_ring(
                            &ext_ring.0,
                            ext_ccw,
                            angle_radians,
                            segments,
                            &self.metadata,
                        ));
                        if do_caps {
                            if let Some(cap) = build_cap_polygon(
                                &ext_ring.0,
                                0.0,
                                ext_ccw,
                                &self.metadata,
                            ) {
                                new_polygons.push(cap);
                            }
                            if let Some(cap) = build_cap_polygon(
                                &ext_ring.0,
                                angle_radians,
                                !ext_ccw,
                                &self.metadata,
                            ) {
                                new_polygons.push(cap);
                            }
                        }

                        // holes
                        for hole in poly2d.interiors() {
                            let hole_ccw = is_ccw(hole);
                            new_polygons.extend(revolve_ring(
                                &hole.0,
                                hole_ccw,
                                angle_radians,
                                segments,
                                &self.metadata,
                            ));
                        }
                    }
                }

                // Ignore lines, points, etc.
                _ => {}
            }
        }

        //----------------------------------------------------------------------
        // 3) Return the new CSG:
        //----------------------------------------------------------------------
        CSG {
            polygons: new_polygons,
            geometry: GeometryCollection::default(),
            metadata: self.metadata.clone(),
        }
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
    /*
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
            let initial_rot = Rotation3::from_axis_angle(&Unit::new_unchecked(axis), angle);
            let rot = initial_rot.to_homogeneous()
    
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
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(side_poly);
            }
        }
    
        // Combine into a final CSG
        CSG::from_polygons(&all_polygons)
    }
    */

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
        metadata: Option<S>,
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
            let poly = Polygon::new(vec![v0, v2, v1], metadata.clone());
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
        metadata: Option<S>,
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
            let poly = Polygon::new(vec![v0, v1, v2], metadata.clone());
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

/// Helper to build a single Polygon from a “slice” of 3D points.
///
/// If `flip_winding` is true, we reverse the vertex order (so the polygon’s normal flips).
fn _polygon_from_slice<S: Clone + Send + Sync>(
    slice_pts: &[Point3<Real>],
    flip_winding: bool,
    metadata: Option<S>,
) -> Polygon<S> {
    if slice_pts.len() < 3 {
        // degenerate polygon
        return Polygon::new(vec![], metadata);
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

    let mut poly = Polygon::new(verts, metadata);
    poly.set_new_normal(); // Recompute its plane & normal for consistency
    poly
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