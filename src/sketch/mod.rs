//! `Sketch` struct and implementations of the `CSGOps` trait for `Sketch`

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{PI, Real};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

use crate::csg::CSG;
use crate::vertex::Vertex;
use geo::algorithm::winding_order::Winding;
use geo::{
    AffineOps, AffineTransform, BooleanOps as GeoBooleanOps, BoundingRect, Coord, CoordsIter,
    Geometry, GeometryCollection, Line, LineString, MultiPolygon, Orient,
    Polygon as GeoPolygon, Rect, orient::Direction,
};
use nalgebra::{Matrix4, Point3, UnitQuaternion, Vector3, partial_max, partial_min};
use std::fmt::Debug;
use std::sync::OnceLock;

pub mod extrudes;
pub mod shapes;

#[cfg(feature = "hershey-text")]
pub mod hershey;

#[cfg(feature = "image-io")]
pub mod image;

#[cfg(feature = "metaballs")]
pub mod metaballs;

#[cfg(feature = "offset")]
pub mod offset;

#[cfg(feature = "truetype-text")]
pub mod truetype;

pub mod triangulated;

/// Position transform plus rotation from the sketch's local XY plane into 3D.
pub(crate) type OriginTransform = (Vector3<Real>, UnitQuaternion<Real>);

#[derive(Debug, Clone)]
pub struct GraphicLineString {
    pub points: Vec<[f32; 3]>,
}

impl GraphicLineString {
    pub fn line_count(&self) -> usize {
        self.points.len().saturating_sub(1)
    }
}

#[derive(Debug, Clone)]
pub struct GraphicLineStrings {
    pub line_strings: Vec<GraphicLineString>,
}

impl GraphicLineStrings {
    pub fn total_line_count(&self) -> usize {
        self.line_strings
            .iter()
            .map(GraphicLineString::line_count)
            .sum()
    }
}

#[derive(Clone, Debug)]
pub struct Sketch<M> {
    /// 2D points, lines, polylines, polygons, and multipolygons
    pub geometry: GeometryCollection<Real>,

    /// Lazily calculated AABB that spans `geometry`.
    pub bounding_box: OnceLock<Aabb>,

    /// Whole-sketch metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub metadata: M,

    /// Origin of the sketch in 3D space.
    pub(crate) origin: Vertex,

    pub(crate) origin_transform: OriginTransform,
}

impl<M: Clone + Send + Sync + Debug> Sketch<M> {
    /// Return a new empty sketch with explicit metadata.
    pub fn empty(metadata: M) -> Self {
        Sketch {
            geometry: GeometryCollection::default(),
            bounding_box: OnceLock::new(),
            metadata,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// Take the [`geo::Polygon`]'s from the `CSG`'s geometry collection
    pub fn to_multipolygon(&self) -> MultiPolygon<Real> {
        let polygons = self
            .geometry
            .iter()
            .flat_map(|geom| match geom {
                Geometry::Polygon(poly) => vec![poly.clone()],
                Geometry::MultiPolygon(mp) => mp.0.clone(),
                _ => vec![],
            })
            .collect();

        MultiPolygon(polygons)
    }

    /// Create a Sketch from a `geo::GeometryCollection`.
    pub fn from_geo(geometry: GeometryCollection<Real>, metadata: M) -> Sketch<M> {
        Sketch {
            geometry,
            bounding_box: OnceLock::new(),
            metadata,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// Return this sketch with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Sketch<NewM> {
        Sketch {
            geometry: self.geometry,
            bounding_box: OnceLock::new(),
            metadata,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Map this sketch's metadata while preserving its geometry and origin.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, f: F) -> Sketch<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Sketch {
            geometry: self.geometry,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Set the origin used when rendering or lifting this sketch into 3D.
    pub fn set_origin(&mut self, origin: Vertex) {
        self.origin_transform = Self::prepare_origin_transform(origin);
        self.origin = origin;
    }

    /// Return this sketch with a new 3D origin.
    pub fn origin(mut self, origin: Vertex) -> Self {
        self.set_origin(origin);
        self
    }

    pub(crate) fn prepare_origin_transform(origin: Vertex) -> OriginTransform {
        let default_origin = Vertex::default();
        let pos_transform = origin.position - default_origin.position;
        let default_normal = default_origin
            .normal
            .try_normalize(crate::float_types::tolerance())
            .unwrap_or_else(Vector3::z);
        let origin_normal = origin
            .normal
            .try_normalize(crate::float_types::tolerance())
            .unwrap_or(default_normal);
        let rotation_quat = UnitQuaternion::rotation_between(&default_normal, &origin_normal)
            .unwrap_or_else(|| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI));

        (pos_transform, rotation_quat)
    }

    pub(crate) fn apply_origin_transform_vertex(
        vertex: Vertex,
        origin_transform: OriginTransform,
    ) -> Vertex {
        let (pos_transform, rotation_quat) = origin_transform;
        let position = Point3::from(rotation_quat * vertex.position.coords + pos_transform);
        let normal = rotation_quat * vertex.normal;
        Vertex::new(position, normal)
    }

    fn apply_origin_transform_point(
        point: Point3<Real>,
        origin_transform: OriginTransform,
    ) -> Point3<Real> {
        let (pos_transform, rotation_quat) = origin_transform;
        Point3::from(rotation_quat * point.coords + pos_transform)
    }

    /// Create render-ready line strings from this sketch's visible edges in 3D space.
    pub fn build_graphic_line_strings(&self) -> GraphicLineStrings {
        let mut graphic_line_strings = Vec::new();
        self.collect_graphic_line_strings(&self.geometry, &mut graphic_line_strings);
        GraphicLineStrings {
            line_strings: graphic_line_strings,
        }
    }

    fn collect_graphic_line_strings(
        &self,
        geometry: &GeometryCollection<Real>,
        out: &mut Vec<GraphicLineString>,
    ) {
        for geom in geometry {
            match geom {
                Geometry::Polygon(polygon) => {
                    out.push(self.line_string_to_graphic_line_string(polygon.exterior()));
                    for interior in polygon.interiors() {
                        out.push(self.line_string_to_graphic_line_string(interior));
                    }
                },
                Geometry::Line(line) => {
                    out.push(self.line_string_to_graphic_line_string(&LineString::from(
                        vec![line.start, line.end],
                    )));
                },
                Geometry::LineString(line_string) => {
                    out.push(self.line_string_to_graphic_line_string(line_string));
                },
                Geometry::MultiLineString(line_strings) => {
                    for line_string in line_strings {
                        out.push(self.line_string_to_graphic_line_string(line_string));
                    }
                },
                Geometry::MultiPolygon(polygons) => {
                    for polygon in polygons {
                        out.push(self.line_string_to_graphic_line_string(polygon.exterior()));
                        for interior in polygon.interiors() {
                            out.push(self.line_string_to_graphic_line_string(interior));
                        }
                    }
                },
                Geometry::Rect(rect) => {
                    let line_string = Self::lines_to_line_string(&rect.to_lines());
                    out.push(self.line_string_to_graphic_line_string(&line_string));
                },
                Geometry::Triangle(triangle) => {
                    let line_string = Self::lines_to_line_string(&triangle.to_lines());
                    out.push(self.line_string_to_graphic_line_string(&line_string));
                },
                Geometry::GeometryCollection(collection) => {
                    self.collect_graphic_line_strings(collection, out);
                },
                _ => {},
            }
        }
    }

    fn lines_to_line_string(lines: &[Line<Real>]) -> LineString<Real> {
        let Some(first_line) = lines.first() else {
            return LineString::empty();
        };

        let mut coord_vec = Vec::with_capacity(lines.len() + 1);
        coord_vec.push(first_line.start);
        coord_vec.extend(lines.iter().map(|line| line.end));
        LineString::new(coord_vec)
    }

    fn line_string_to_graphic_line_string(
        &self,
        line_string: &LineString<Real>,
    ) -> GraphicLineString {
        let points = line_string
            .coords()
            .map(|coord| {
                let point = Point3::new(coord.x, coord.y, 0.0);
                let point_transformed =
                    Self::apply_origin_transform_point(point, self.origin_transform);
                [
                    point_transformed.x as f32,
                    point_transformed.y as f32,
                    point_transformed.z as f32,
                ]
            })
            .collect();

        GraphicLineString { points }
    }

    /// Triangulate a polygon and holes into a list of triangles, each triangle is [v0, v1, v2].
    pub fn triangulate_with_holes(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> =
            outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> =
                    hole.iter().map(|&[x, y]| Coord { x, y }).collect();
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
            use geo::TriangulateDelaunay;
            // We want polygons with holes => constrained triangulation.
            // For safety, handle the Result the trait returns:
            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                // If a triangulation error is a possibility,
                // pick the error-handling you want here:
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                // Embed the 2D triangle vertices at z=0.
                let [a, b, c] = [triangle.v1(), triangle.v2(), triangle.v3()];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }

        #[cfg(feature = "delaunay-rs")]
        {
            use delaunay_triangulator::core::vertex::Vertex as DelaunayVertex;
            use delaunay_triangulator::triangulation::delaunay::DelaunayTriangulation;
            use geo::{Intersects, Point as GeoPoint};

            let mut all_points = Vec::<[Real; 2]>::new();
            let mut add_ring = |ring: &[[Real; 2]]| {
                for &point in ring {
                    if point[0].is_finite()
                        && point[1].is_finite()
                        && !all_points.contains(&point)
                    {
                        all_points.push(point);
                    }
                }
            };
            add_ring(outer);
            for hole in holes {
                add_ring(hole);
            }

            if all_points.len() < 3 {
                return Vec::new();
            }

            let delaunay_vertices: Vec<DelaunayVertex<f64, (), 2>> = all_points
                .iter()
                .map(|&[x, y]| delaunay_triangulator::vertex!([x as f64, y as f64]))
                .collect();
            let Ok(dt) = DelaunayTriangulation::<_, (), (), 2>::new(&delaunay_vertices) else {
                return Vec::new();
            };

            let mut result = Vec::with_capacity(dt.number_of_cells());
            for (cell_key, _) in dt.cells() {
                let Some(verts) = dt.cell_vertices(cell_key) else {
                    continue;
                };
                if verts.len() != 3 {
                    continue;
                }

                let Some(coords) = verts
                    .iter()
                    .map(|&key| dt.vertex_coords(key))
                    .collect::<Option<Vec<_>>>()
                else {
                    continue;
                };

                let sample_points = [
                    GeoPoint::new(
                        ((coords[0][0] + coords[1][0] + coords[2][0]) / 3.0) as Real,
                        ((coords[0][1] + coords[1][1] + coords[2][1]) / 3.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[0][0] + coords[1][0]) / 2.0) as Real,
                        ((coords[0][1] + coords[1][1]) / 2.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[1][0] + coords[2][0]) / 2.0) as Real,
                        ((coords[1][1] + coords[2][1]) / 2.0) as Real,
                    ),
                    GeoPoint::new(
                        ((coords[2][0] + coords[0][0]) / 2.0) as Real,
                        ((coords[2][1] + coords[0][1]) / 2.0) as Real,
                    ),
                ];
                if !sample_points.iter().all(|point| polygon.intersects(point)) {
                    continue;
                }

                result.push([
                    Point3::new(coords[0][0] as Real, coords[0][1] as Real, 0.0),
                    Point3::new(coords[1][0] as Real, coords[1][1] as Real, 0.0),
                    Point3::new(coords[2][0] as Real, coords[2][1] as Real, 0.0),
                ]);
            }

            result
        }
    }

    /// Triangulate all polygons in this Sketch.
    ///
    /// This function converts all polygons (including those from MultiPolygons) contained
    /// in the Sketch's geometry into a list of triangles. Each triangle is represented as
    /// a `[Point3<Real>; 3]`, where the Z-coordinate is 0.0.
    ///
    /// # Returns
    ///
    /// A `Vec<[Point3<Real>; 3]>` containing all the triangles resulting from the triangulation.
    pub fn triangulate(&self) -> Vec<[Point3<Real>; 3]> {
        let mut all_triangles = Vec::new();

        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly) => {
                    let outer: Vec<[Real; 2]> =
                        poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                    let holes: Vec<Vec<[Real; 2]>> = poly
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();
                    let hole_refs: Vec<&[[Real; 2]]> = holes.iter().map(|v| &v[..]).collect();
                    let tris = Self::triangulate_with_holes(&outer, &hole_refs);
                    all_triangles.extend(tris);
                },
                geo::Geometry::MultiPolygon(mp) => {
                    for poly in &mp.0 {
                        let outer: Vec<[Real; 2]> =
                            poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                        let holes: Vec<Vec<[Real; 2]>> = poly
                            .interiors()
                            .iter()
                            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                            .collect();
                        let hole_refs: Vec<&[[Real; 2]]> =
                            holes.iter().map(|v| &v[..]).collect();
                        let tris = Self::triangulate_with_holes(&outer, &hole_refs);
                        all_triangles.extend(tris);
                    }
                },
                // For other geometry types (LineString, Point, etc.), we might choose to ignore them
                // or handle them differently if needed. Currently, ignoring them.
                _ => {},
            }
        }

        all_triangles
    }

    /// Return a copy of this `Sketch` whose polygons are normalised so that
    /// exterior rings wind counter-clockwise and interior rings clockwise.
    pub fn renormalize(&self) -> Sketch<M> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
            .iter()
            .map(|geom| match geom {
                Geometry::Polygon(p) => {
                    Geometry::Polygon(p.clone().orient(Direction::Default))
                },
                Geometry::MultiPolygon(mp) => {
                    Geometry::MultiPolygon(mp.clone().orient(Direction::Default))
                },
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }
}

impl Sketch<()> {
    /// Return a new empty sketch with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Sketch<M> {
    /// Return a new Sketch representing union of the two Sketches.
    ///
    /// ```text
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
    fn union(&self, other: &Sketch<M>) -> Sketch<M> {
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
                },
                _ => final_gc.0.push(g.clone()),
            }
        }
        for g in &other.geometry.0 {
            match g {
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {
                    // skip [multi]polygons
                },
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Return a new Sketch representing diffarence of the two Sketches.
    ///
    /// ```text
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
    fn difference(&self, other: &Sketch<M>) -> Sketch<M> {
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
                Geometry::Polygon(_) | Geometry::MultiPolygon(_) => {}, // skip
                _ => final_gc.0.push(g.clone()),
            }
        }

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Return a new Sketch representing intersection of the two Sketches.
    ///
    /// ```text
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
    fn intersection(&self, other: &Sketch<M>) -> Sketch<M> {
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

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Return a new Sketch representing space in this Sketch excluding the space in the
    /// other Sketch plus the space in the other Sketch excluding the space in this Sketch.
    ///
    /// ```text
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
    fn xor(&self, other: &Sketch<M>) -> Sketch<M> {
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

        Sketch {
            geometry: final_gc,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to both polygons and polylines.
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D,
    /// and the 2D polylines are updated by ignoring the resulting z after transform.
    fn transform(&self, mat: &Matrix4<Real>) -> Sketch<M> {
        let mut sketch = self.clone();

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

        // Transform sketch.geometry (the GeometryCollection) in 2D
        sketch.geometry = sketch.geometry.affine_transform(&affine2);

        // invalidate the old cached bounding box
        sketch.bounding_box = OnceLock::new();

        sketch
    }

    /// Returns an axis-aligned bounding box containing:
    /// The 2D bounding rectangle of `self.geometry`, interpreted at z=0.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons and the 2D geometry’s bounding_rect.
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // Gather from the 2D geometry using `geo::BoundingRect`
            // This gives us (min_x, min_y) / (max_x, max_y)
            // Explicitly capture the result of `.bounding_rect()` as an Option<Rect<Real>>
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

            // If still uninitialized (e.g., no geometry), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Sketch (flip inside vs. outside)
    fn inverse(&self) -> Sketch<M> {
        // Re-build the collection, orienting only what’s supported.
        let oriented_geoms: Vec<Geometry<Real>> = self
            .geometry
            .iter()
            .map(|geom| match geom {
                Geometry::Polygon(p) => {
                    let flipped = if p.exterior().is_ccw() {
                        p.clone().orient(Direction::Reversed)
                    } else {
                        p.clone().orient(Direction::Default)
                    };
                    Geometry::Polygon(flipped)
                },
                Geometry::MultiPolygon(mp) => {
                    // Loop over every polygon inside and apply the same rule.
                    let flipped_polys: Vec<GeoPolygon<Real>> =
                        mp.0.iter()
                            .map(|p| {
                                if p.exterior().is_ccw() {
                                    p.clone().orient(Direction::Reversed)
                                } else {
                                    p.clone().orient(Direction::Default)
                                }
                            })
                            .collect();

                    Geometry::MultiPolygon(MultiPolygon(flipped_polys))
                },
                // Everything else keeps its original orientation.
                _ => geom.clone(),
            })
            .collect();

        Sketch {
            geometry: GeometryCollection(oriented_geoms),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }
}

#[cfg(feature = "mesh")]
impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for Sketch<M> {
    fn from(mesh: Mesh<M>) -> Self {
        // If mesh is empty, return empty Sketch
        if mesh.polygons.is_empty() {
            return Sketch::empty(mesh.metadata);
        }

        // Convert mesh into a collection of 2D polygons
        let mut flattened_3d = Vec::new(); // will store geo::Polygon<Real>

        for poly in &mesh.polygons {
            // Tessellate this polygon into triangles
            let triangles = poly.triangulate();
            // Each triangle has 3 vertices [v0, v1, v2].
            // Project them onto XY => build a 2D polygon (triangle).
            for tri in triangles {
                let ring = vec![
                    (tri[0].position.x, tri[0].position.y),
                    (tri[1].position.x, tri[1].position.y),
                    (tri[2].position.x, tri[2].position.y),
                    (tri[0].position.x, tri[0].position.y), // close ring explicitly
                ];
                let polygon_2d = geo::Polygon::new(LineString::from(ring), vec![]);
                flattened_3d.push(polygon_2d);
            }
        }

        // Union all these polygons together into one MultiPolygon
        // (We could chain them in a fold-based union.)
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

        // Ensure consistent orientation (CCW for exteriors):
        let oriented = unioned_from_3d.orient(Direction::Default);

        // Store final polygons as a MultiPolygon in a new GeometryCollection
        let mut new_gc = GeometryCollection::default();
        new_gc.0.push(Geometry::MultiPolygon(oriented));

        Sketch {
            geometry: new_gc,
            bounding_box: OnceLock::new(),
            metadata: mesh.metadata,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }
}
