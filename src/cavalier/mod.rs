//! Cavalier-contours backed 2D geometry backend.
//!
//! Unlike [`crate::sketch::Sketch`], this backend stores circular arc segments
//! directly using Cavalier's bulge polyline representation. Boolean operations,
//! offsets, signed area, path length, winding queries, and spatial indexes are
//! delegated to `cavalier_contours`.

use std::{fmt::Debug, sync::OnceLock};

use crate::{
    csg::CSG,
    float_types::{PI, Real, parry3d::bounding_volume::Aabb, tolerance},
    vertex::Vertex,
};
use cavalier_contours::core::math::Vector2;
pub use cavalier_contours::{
    polyline::{
        PlineCreation, PlineOrientation, PlineSource, PlineSourceMut, PlineVertex, Polyline,
    },
    shape_algorithms::{Shape, ShapeOffsetOptions},
};
use nalgebra::{Matrix4, Point3, UnitQuaternion, Vector3};

#[cfg(feature = "bmesh")]
use crate::bmesh::BMesh;
#[cfg(feature = "mesh")]
use crate::mesh::Mesh;
#[cfg(feature = "nurbs")]
use crate::nurbs::Nurbs;
#[cfg(feature = "sketch")]
use crate::sketch::Sketch;

/// Default maximum distance used when arcs must be approximated by line
/// segments for non-Cavalier backends.
pub const DEFAULT_ARC_ERROR_DISTANCE: Real = 1e-3;

/// Position transform plus rotation from the backend's local XY plane into 3D.
pub(crate) type OriginTransform = (Vector3<Real>, UnitQuaternion<Real>);

/// Render-ready polyline points in 3D space.
#[derive(Debug, Clone)]
pub struct GraphicLineString {
    pub points: Vec<[f32; 3]>,
}

impl GraphicLineString {
    pub fn line_count(&self) -> usize {
        self.points.len().saturating_sub(1)
    }
}

/// Render-ready collection of line strings.
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

/// A 2D CSG geometry backed by `cavalier_contours::Shape`.
#[derive(Clone, Debug)]
pub struct Cavalier<M: Clone + Send + Sync + Debug = ()> {
    /// Native Cavalier shape. CCW loops are filled material and CW loops are holes.
    pub shape: Shape<Real>,

    /// Lazily calculated AABB that spans `shape`.
    pub bounding_box: OnceLock<Aabb>,

    /// Whole-shape metadata. Use `M = ()` for no metadata and `M = Option<YourMetadata>`
    /// for optional metadata.
    pub metadata: M,

    /// Error distance used when arcs are approximated for non-Cavalier consumers.
    pub arc_approx_error: Real,

    /// Origin of the sketch in 3D space.
    pub(crate) origin: Vertex,

    pub(crate) origin_transform: OriginTransform,
}

impl<M: Clone + Send + Sync + Debug> Cavalier<M> {
    /// Return a new empty Cavalier backend with explicit metadata.
    pub fn empty(metadata: M) -> Self {
        Self::from_shape(Shape::empty(), metadata)
    }

    /// Create from an existing Cavalier shape.
    pub fn from_shape(shape: Shape<Real>, metadata: M) -> Self {
        Self {
            shape,
            bounding_box: OnceLock::new(),
            metadata,
            arc_approx_error: DEFAULT_ARC_ERROR_DISTANCE,
            origin: Vertex::default(),
            origin_transform: Self::prepare_origin_transform(Vertex::default()),
        }
    }

    /// Create from closed Cavalier polylines.
    ///
    /// Open and zero-area polylines are ignored because `Shape` represents filled
    /// 2D regions, not loose wires.
    pub fn from_plines<I>(plines: I, metadata: M) -> Self
    where
        I: IntoIterator<Item = Polyline<Real>>,
    {
        Self::from_shape(shape_from_closed_plines(plines), metadata)
    }

    /// Create from separately classified material and hole loops.
    pub fn from_signed_plines<I, J>(ccw_plines: I, cw_plines: J, metadata: M) -> Self
    where
        I: IntoIterator<Item = Polyline<Real>>,
        J: IntoIterator<Item = Polyline<Real>>,
    {
        let plines = ccw_plines
            .into_iter()
            .filter_map(|pline| {
                normalize_pline_orientation(pline, PlineOrientation::CounterClockwise)
            })
            .chain(cw_plines.into_iter().filter_map(|pline| {
                normalize_pline_orientation(pline, PlineOrientation::Clockwise)
            }));
        Self::from_shape(shape_from_closed_plines(plines), metadata)
    }

    /// Create a closed loop from `[x, y, bulge]` vertices.
    pub fn polyline_loop(vertices: &[[Real; 3]], metadata: M) -> Self {
        if vertices.len() < 2 {
            return Self::empty(metadata);
        }

        let end = if same_xy(
            (vertices[0][0], vertices[0][1]),
            (
                vertices[vertices.len() - 1][0],
                vertices[vertices.len() - 1][1],
            ),
        ) {
            vertices.len() - 1
        } else {
            vertices.len()
        };

        let mut pline = Polyline::new_closed();
        for [x, y, bulge] in vertices.iter().take(end).copied() {
            pline.add(x, y, bulge);
        }
        Self::from_plines([pline], metadata)
    }

    /// Creates an axis-aligned rectangle in the XY plane.
    pub fn rectangle(width: Real, height: Real, metadata: M) -> Self {
        Self::polygon(
            &[[0.0, 0.0], [width, 0.0], [width, height], [0.0, height]],
            metadata,
        )
    }

    /// Creates a square in the XY plane.
    pub fn square(width: Real, metadata: M) -> Self {
        Self::rectangle(width, width, metadata)
    }

    /// Creates a polygon from `[x, y]` points.
    pub fn polygon(points: &[[Real; 2]], metadata: M) -> Self {
        if points.len() < 3 {
            return Self::empty(metadata);
        }

        let end = if same_xy(
            (points[0][0], points[0][1]),
            (points[points.len() - 1][0], points[points.len() - 1][1]),
        ) {
            points.len() - 1
        } else {
            points.len()
        };

        let mut pline = Polyline::new_closed();
        for [x, y] in points.iter().take(end).copied() {
            pline.add(x, y, 0.0);
        }
        Self::from_plines([pline], metadata)
    }

    /// Creates an exact circular region using two Cavalier half-circle arc segments.
    pub fn circle(radius: Real, metadata: M) -> Self {
        Self::circle_at(0.0, 0.0, radius, metadata)
    }

    /// Creates an exact circular region centered at `(center_x, center_y)`.
    pub fn circle_at(center_x: Real, center_y: Real, radius: Real, metadata: M) -> Self {
        if radius <= tolerance() {
            return Self::empty(metadata);
        }

        let mut pline = Polyline::new_closed();
        pline.add(center_x - radius, center_y, 1.0);
        pline.add(center_x + radius, center_y, 1.0);
        Self::from_plines([pline], metadata)
    }

    /// Creates a rounded rectangle with exact circular corner arcs.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        metadata: M,
    ) -> Self {
        let radius = corner_radius.min(width.abs() * 0.5).min(height.abs() * 0.5);
        if radius <= tolerance() {
            return Self::rectangle(width, height, metadata);
        }

        let bulge = (PI / 8.0).tan();
        let mut pline = Polyline::new_closed();
        pline.add(radius, 0.0, 0.0);
        pline.add(width - radius, 0.0, bulge);
        pline.add(width, radius, 0.0);
        pline.add(width, height - radius, bulge);
        pline.add(width - radius, height, 0.0);
        pline.add(radius, height, bulge);
        pline.add(0.0, height - radius, 0.0);
        pline.add(0.0, radius, bulge);
        Self::from_plines([pline], metadata)
    }

    /// Borrow the native Cavalier shape.
    pub fn as_shape(&self) -> &Shape<Real> {
        &self.shape
    }

    /// Consume this backend and return its native Cavalier shape.
    pub fn into_shape(self) -> Shape<Real> {
        self.shape
    }

    /// Return this backend with a different arc approximation error.
    pub fn with_arc_approx_error(mut self, error_distance: Real) -> Self {
        self.arc_approx_error = error_distance.abs().max(tolerance());
        self
    }

    /// Return this backend with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Cavalier<NewM> {
        Cavalier {
            shape: self.shape,
            bounding_box: OnceLock::new(),
            metadata,
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Map this backend's metadata while preserving its geometry and origin.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, f: F) -> Cavalier<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Cavalier {
            shape: self.shape,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Iterate over filled/material CCW loops.
    pub fn ccw_polylines(&self) -> impl Iterator<Item = &Polyline<Real>> {
        self.shape.ccw_plines.iter().map(|ip| &ip.polyline)
    }

    /// Iterate over subtractive/hole CW loops.
    pub fn cw_polylines(&self) -> impl Iterator<Item = &Polyline<Real>> {
        self.shape.cw_plines.iter().map(|ip| &ip.polyline)
    }

    /// Iterate over all loops in shape storage order.
    pub fn polylines(&self) -> impl Iterator<Item = &Polyline<Real>> {
        self.ccw_polylines().chain(self.cw_polylines())
    }

    /// Number of material loops.
    pub fn ccw_loop_count(&self) -> usize {
        self.shape.ccw_plines.len()
    }

    /// Number of hole loops.
    pub fn cw_loop_count(&self) -> usize {
        self.shape.cw_plines.len()
    }

    /// Signed area: material contributes positive area, holes contribute negative area.
    pub fn signed_area(&self) -> Real {
        self.polylines().map(PlineSource::area).sum()
    }

    /// Total unsigned area across all loops.
    pub fn unsigned_loop_area(&self) -> Real {
        self.polylines().map(|pline| pline.area().abs()).sum()
    }

    /// Total loop path length, including circular arc length.
    pub fn path_length(&self) -> Real {
        self.polylines().map(PlineSource::path_length).sum()
    }

    /// Number of straight line segments in all loops.
    pub fn line_segment_count(&self) -> usize {
        self.polylines()
            .map(|pline| {
                pline
                    .iter_segments()
                    .filter(|(v1, _)| v1.bulge_is_zero())
                    .count()
            })
            .sum()
    }

    /// Number of circular arc segments in all loops.
    pub fn arc_segment_count(&self) -> usize {
        self.polylines()
            .map(|pline| {
                pline
                    .iter_segments()
                    .filter(|(v1, _)| !v1.bulge_is_zero())
                    .count()
            })
            .sum()
    }

    /// True when any loop contains at least one circular arc segment.
    pub fn has_arcs(&self) -> bool {
        self.arc_segment_count() > 0
    }

    /// Non-zero winding membership test against all signed loops.
    pub fn contains_point(&self, x: Real, y: Real) -> bool {
        self.winding_number(x, y) != 0
    }

    /// Sum of loop winding numbers at a point.
    pub fn winding_number(&self, x: Real, y: Real) -> i32 {
        let point = Vector2::new(x, y);
        self.ccw_polylines()
            .map(|pline| pline.winding_number(point))
            .chain(self.cw_polylines().map(|pline| pline.winding_number(point)))
            .sum()
    }

    /// Return all loops with circular arcs approximated by straight segments.
    pub fn to_approx_polylines(&self, error_distance: Real) -> Vec<Polyline<Real>> {
        self.polylines()
            .map(|pline| approximate_polyline(pline, error_distance))
            .collect()
    }

    /// Return a Cavalier backend with all circular arcs approximated by line segments.
    pub fn to_approx_lines(&self, error_distance: Real) -> Self {
        Self {
            shape: shape_from_closed_plines(self.to_approx_polylines(error_distance)),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: error_distance.abs().max(tolerance()),
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Offset the entire shape using Cavalier's shape offset algorithm.
    pub fn offset(&self, distance: Real) -> Self {
        self.offset_opt(distance, ShapeOffsetOptions::default())
    }

    /// Offset the entire shape using explicit Cavalier offset options.
    pub fn offset_opt(&self, distance: Real, options: ShapeOffsetOptions<Real>) -> Self {
        Self {
            shape: self.shape.parallel_offset(distance, options),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    /// Set the origin used when rendering or lifting this shape into 3D.
    pub fn set_origin(&mut self, origin: Vertex) {
        self.origin_transform = Self::prepare_origin_transform(origin);
        self.origin = origin;
    }

    /// Return this backend with a new 3D origin.
    pub fn origin(mut self, origin: Vertex) -> Self {
        self.set_origin(origin);
        self
    }

    pub(crate) fn prepare_origin_transform(origin: Vertex) -> OriginTransform {
        let default_origin = Vertex::default();
        let pos_transform = origin.position - default_origin.position;
        let default_normal = default_origin
            .normal
            .try_normalize(tolerance())
            .unwrap_or_else(Vector3::z);
        let origin_normal = origin
            .normal
            .try_normalize(tolerance())
            .unwrap_or(default_normal);
        let rotation_quat = UnitQuaternion::rotation_between(&default_normal, &origin_normal)
            .unwrap_or_else(|| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI));

        (pos_transform, rotation_quat)
    }

    fn apply_origin_transform_point(
        point: Point3<Real>,
        origin_transform: OriginTransform,
    ) -> Point3<Real> {
        let (pos_transform, rotation_quat) = origin_transform;
        Point3::from(rotation_quat * point.coords + pos_transform)
    }

    /// Create render-ready line strings from this shape's visible edges in 3D space.
    pub fn build_graphic_line_strings(&self) -> GraphicLineStrings {
        let line_strings = self
            .to_approx_polylines(self.arc_approx_error)
            .into_iter()
            .filter_map(|pline| self.pline_to_graphic_line_string(&pline))
            .collect();

        GraphicLineStrings { line_strings }
    }

    fn pline_to_graphic_line_string(
        &self,
        pline: &Polyline<Real>,
    ) -> Option<GraphicLineString> {
        if pline.vertex_count() < 2 {
            return None;
        }

        let mut points: Vec<[f32; 3]> = pline
            .iter_vertexes()
            .map(|vertex| {
                let point = Point3::new(vertex.x, vertex.y, 0.0);
                let transformed =
                    Self::apply_origin_transform_point(point, self.origin_transform);
                [
                    transformed.x as f32,
                    transformed.y as f32,
                    transformed.z as f32,
                ]
            })
            .collect();

        if pline.is_closed() {
            if let Some(first) = points.first().copied() {
                points.push(first);
            }
        }

        Some(GraphicLineString { points })
    }

    fn transformed_shape(&self, matrix: &Matrix4<Real>) -> Shape<Real> {
        let preserves_arcs = matrix_preserves_circular_arcs(matrix);
        let mut transformed = Vec::new();

        for pline in self.ccw_polylines() {
            if let Some(pline) = transform_pline(
                pline,
                matrix,
                preserves_arcs,
                PlineOrientation::CounterClockwise,
                self.arc_approx_error,
            ) {
                transformed.push(pline);
            }
        }

        for pline in self.cw_polylines() {
            if let Some(pline) = transform_pline(
                pline,
                matrix,
                preserves_arcs,
                PlineOrientation::Clockwise,
                self.arc_approx_error,
            ) {
                transformed.push(pline);
            }
        }

        shape_from_closed_plines(transformed)
    }

    #[cfg(any(feature = "sketch", feature = "nurbs"))]
    fn region_rings(
        &self,
        error_distance: Real,
    ) -> Vec<(Vec<(Real, Real)>, Vec<Vec<(Real, Real)>>)> {
        let ccw = self.ccw_polylines().collect::<Vec<_>>();
        let mut regions = Vec::<(usize, Vec<(Real, Real)>, Vec<Vec<(Real, Real)>>)>::new();

        for (index, pline) in ccw.iter().enumerate() {
            if let Some(ring) = point_ring_from_pline(pline, error_distance) {
                regions.push((index, ring, Vec::new()));
            }
        }

        for hole in self.cw_polylines() {
            let Some(hole_ring) = point_ring_from_pline(hole, error_distance) else {
                continue;
            };
            let Some(probe) = pline_probe_point(hole) else {
                continue;
            };

            let target = regions
                .iter()
                .position(|(ccw_index, _, _)| ccw[*ccw_index].winding_number(probe) != 0);
            if let Some(target) = target {
                regions[target].2.push(hole_ring);
            }
        }

        regions
            .into_iter()
            .map(|(_, exterior, holes)| (exterior, holes))
            .collect()
    }

    /// Convert into a geo-backed `Sketch`, approximating circular arcs.
    #[cfg(feature = "sketch")]
    pub fn to_sketch(&self) -> Sketch<M> {
        self.to_sketch_with_error(self.arc_approx_error)
    }

    /// Convert into a geo-backed `Sketch` with an explicit arc approximation error.
    #[cfg(feature = "sketch")]
    pub fn to_sketch_with_error(&self, error_distance: Real) -> Sketch<M> {
        use geo::{
            Geometry, GeometryCollection, LineString, MultiPolygon, Polygon as GeoPolygon,
        };

        let polygons = self
            .region_rings(error_distance)
            .into_iter()
            .filter_map(|(exterior, holes)| {
                let exterior = LineString::from(exterior);
                if exterior.0.len() < 4 {
                    return None;
                }

                let holes = holes
                    .into_iter()
                    .filter_map(|hole| {
                        let hole = LineString::from(hole);
                        (hole.0.len() >= 4).then_some(hole)
                    })
                    .collect();

                Some(GeoPolygon::new(exterior, holes))
            })
            .collect::<Vec<_>>();

        if polygons.is_empty() {
            return Sketch::empty(self.metadata.clone());
        }

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::MultiPolygon(MultiPolygon(polygons))]),
            self.metadata.clone(),
        )
    }
}

impl Cavalier<()> {
    /// Return a new empty Cavalier backend with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Cavalier<M> {
    fn union(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.union(&other.shape),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn difference(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.difference(&other.shape),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn intersection(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.intersection(&other.shape),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn xor(&self, other: &Self) -> Self {
        Self {
            shape: self.shape.xor(&other.shape),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn transform(&self, matrix: &Matrix4<Real>) -> Self {
        Self {
            shape: self.transformed_shape(matrix),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn inverse(&self) -> Self {
        let inverted = self
            .polylines()
            .map(|pline| {
                let mut pline = pline.clone();
                pline.invert_direction_mut();
                pline
            })
            .collect::<Vec<_>>();

        Self {
            shape: shape_from_closed_plines(inverted),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
            arc_approx_error: self.arc_approx_error,
            origin: self.origin,
            origin_transform: self.origin_transform,
        }
    }

    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            if let Some(bounds) = self.shape.plines_index.bounds() {
                Aabb::new(
                    Point3::new(bounds.min_x, bounds.min_y, 0.0),
                    Point3::new(bounds.max_x, bounds.max_y, 0.0),
                )
            } else {
                Aabb::new(Point3::origin(), Point3::origin())
            }
        })
    }

    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Send + Sync + Debug> From<Sketch<M>> for Cavalier<M> {
    fn from(sketch: Sketch<M>) -> Self {
        use geo::{Coord, CoordsIter, Geometry, LineString, Polygon as GeoPolygon};

        fn coord_same(a: Coord<Real>, b: Coord<Real>) -> bool {
            same_xy((a.x, a.y), (b.x, b.y))
        }

        fn line_string_to_pline(
            line_string: &LineString<Real>,
            desired_orientation: PlineOrientation,
        ) -> Option<Polyline<Real>> {
            let coords = line_string.coords_iter().collect::<Vec<_>>();
            if coords.len() < 3 {
                return None;
            }

            let end = if coord_same(coords[0], coords[coords.len() - 1]) {
                coords.len() - 1
            } else {
                coords.len()
            };

            if end < 3 {
                return None;
            }

            let mut pline = Polyline::new_closed();
            for coord in coords.into_iter().take(end) {
                pline.add(coord.x, coord.y, 0.0);
            }
            normalize_pline_orientation(pline, desired_orientation)
        }

        fn polygon_to_plines(poly: &GeoPolygon<Real>, out: &mut Vec<Polyline<Real>>) {
            if let Some(exterior) =
                line_string_to_pline(poly.exterior(), PlineOrientation::CounterClockwise)
            {
                out.push(exterior);
            }

            for interior in poly.interiors() {
                if let Some(hole) = line_string_to_pline(interior, PlineOrientation::Clockwise)
                {
                    out.push(hole);
                }
            }
        }

        fn push_rect(rect: &geo::Rect<Real>, out: &mut Vec<Polyline<Real>>) {
            let min = rect.min();
            let max = rect.max();
            let points = [
                [min.x, min.y],
                [max.x, min.y],
                [max.x, max.y],
                [min.x, max.y],
            ];
            let mut pline = Polyline::new_closed();
            for [x, y] in points {
                pline.add(x, y, 0.0);
            }
            if let Some(pline) =
                normalize_pline_orientation(pline, PlineOrientation::CounterClockwise)
            {
                out.push(pline);
            }
        }

        fn push_triangle(triangle: &geo::Triangle<Real>, out: &mut Vec<Polyline<Real>>) {
            let lines = triangle.to_lines();
            let mut pline = Polyline::new_closed();
            for line in lines {
                pline.add(line.start.x, line.start.y, 0.0);
            }
            if let Some(pline) =
                normalize_pline_orientation(pline, PlineOrientation::CounterClockwise)
            {
                out.push(pline);
            }
        }

        fn collect_geometry(geometry: &Geometry<Real>, out: &mut Vec<Polyline<Real>>) {
            match geometry {
                Geometry::Polygon(poly) => polygon_to_plines(poly, out),
                Geometry::MultiPolygon(multi) => {
                    for poly in multi {
                        polygon_to_plines(poly, out);
                    }
                },
                Geometry::LineString(line_string) => {
                    if let Some(pline) =
                        line_string_to_pline(line_string, PlineOrientation::CounterClockwise)
                    {
                        out.push(pline);
                    }
                },
                Geometry::MultiLineString(lines) => {
                    for line_string in lines {
                        if let Some(pline) = line_string_to_pline(
                            line_string,
                            PlineOrientation::CounterClockwise,
                        ) {
                            out.push(pline);
                        }
                    }
                },
                Geometry::Rect(rect) => push_rect(rect, out),
                Geometry::Triangle(triangle) => push_triangle(triangle, out),
                Geometry::GeometryCollection(collection) => {
                    for child in collection {
                        collect_geometry(child, out);
                    }
                },
                _ => {},
            }
        }

        let mut plines = Vec::new();
        for geometry in &sketch.geometry {
            collect_geometry(geometry, &mut plines);
        }

        Self::from_plines(plines, sketch.metadata)
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Send + Sync + Debug> From<Cavalier<M>> for Sketch<M> {
    fn from(cavalier: Cavalier<M>) -> Self {
        cavalier.to_sketch_with_error(cavalier.arc_approx_error)
    }
}

#[cfg(all(feature = "mesh", feature = "sketch"))]
impl<M: Clone + Send + Sync + Debug> From<Mesh<M>> for Cavalier<M> {
    fn from(mesh: Mesh<M>) -> Self {
        let sketch: Sketch<M> = mesh.into();
        sketch.into()
    }
}

#[cfg(all(feature = "mesh", feature = "sketch"))]
impl<M: Clone + Send + Sync + Debug> From<Cavalier<M>> for Mesh<M> {
    fn from(cavalier: Cavalier<M>) -> Self {
        let sketch = cavalier.to_sketch_with_error(cavalier.arc_approx_error);
        sketch.into()
    }
}

#[cfg(all(feature = "bmesh", feature = "mesh", feature = "sketch"))]
impl<M: Clone + Send + Sync + Debug> From<BMesh<M>> for Cavalier<M> {
    fn from(bmesh: BMesh<M>) -> Self {
        let mesh: Mesh<M> = bmesh.into();
        mesh.into()
    }
}

#[cfg(all(feature = "bmesh", feature = "mesh", feature = "sketch"))]
impl<M: Clone + Send + Sync + Debug> From<Cavalier<M>> for BMesh<M> {
    fn from(cavalier: Cavalier<M>) -> Self {
        let mesh: Mesh<M> = cavalier.into();
        mesh.into()
    }
}

#[cfg(all(feature = "nurbs", feature = "sketch"))]
impl<M: Clone + Send + Sync + Debug> From<Nurbs<M>> for Cavalier<M> {
    fn from(nurbs: Nurbs<M>) -> Self {
        nurbs.to_sketch_with_tolerance(Some(tolerance())).into()
    }
}

#[cfg(feature = "nurbs")]
impl<M: Clone + Send + Sync + Debug> From<Cavalier<M>> for Nurbs<M> {
    fn from(cavalier: Cavalier<M>) -> Self {
        use curvo::prelude::{CompoundCurve2D, NurbsCurve2D, Region};
        use nalgebra::Point2;

        let regions = cavalier
            .region_rings(cavalier.arc_approx_error)
            .into_iter()
            .filter_map(|(exterior, holes)| {
                let exterior_points = exterior
                    .into_iter()
                    .map(|(x, y)| Point2::new(x, y))
                    .collect::<Vec<_>>();
                if exterior_points.len() < 4 {
                    return None;
                }

                let exterior_curve: CompoundCurve2D<Real> =
                    NurbsCurve2D::polyline(&exterior_points, true).into();

                let hole_curves = holes
                    .into_iter()
                    .filter_map(|hole| {
                        let hole_points = hole
                            .into_iter()
                            .map(|(x, y)| Point2::new(x, y))
                            .collect::<Vec<_>>();
                        (hole_points.len() >= 4)
                            .then(|| NurbsCurve2D::polyline(&hole_points, true).into())
                    })
                    .collect::<Vec<CompoundCurve2D<Real>>>();

                Some(Region::new(exterior_curve, hole_curves))
            })
            .collect();

        Nurbs::from_regions(regions, cavalier.metadata)
    }
}

fn normalize_pline_orientation(
    mut pline: Polyline<Real>,
    desired_orientation: PlineOrientation,
) -> Option<Polyline<Real>> {
    if !valid_closed_pline(&pline) {
        return None;
    }

    if pline.orientation() != desired_orientation {
        pline.invert_direction_mut();
    }

    valid_closed_pline(&pline).then_some(pline)
}

fn shape_from_closed_plines<I>(plines: I) -> Shape<Real>
where
    I: IntoIterator<Item = Polyline<Real>>,
{
    Shape::from_plines(plines.into_iter().filter(valid_closed_pline))
}

fn valid_closed_pline(pline: &Polyline<Real>) -> bool {
    pline.is_closed() && pline.vertex_count() > 1 && pline.area().abs() > tolerance()
}

fn approximate_polyline(pline: &Polyline<Real>, error_distance: Real) -> Polyline<Real> {
    pline
        .arcs_to_approx_lines(error_distance.abs().max(tolerance()))
        .unwrap_or_else(|| pline.clone())
}

#[cfg(any(feature = "sketch", feature = "nurbs"))]
fn point_ring_from_pline(
    pline: &Polyline<Real>,
    error_distance: Real,
) -> Option<Vec<(Real, Real)>> {
    let approx = approximate_polyline(pline, error_distance);
    if approx.vertex_count() < 3 {
        return None;
    }

    let mut points = approx
        .iter_vertexes()
        .map(|vertex| (vertex.x, vertex.y))
        .collect::<Vec<_>>();

    if points.len() < 3 {
        return None;
    }

    let first = points[0];
    let last = points[points.len() - 1];
    if !same_xy(first, last) {
        points.push(first);
    }

    (points.len() >= 4).then_some(points)
}

#[cfg(any(feature = "sketch", feature = "nurbs"))]
fn pline_probe_point(pline: &Polyline<Real>) -> Option<Vector2<Real>> {
    pline.extents().map(|bounds| {
        Vector2::new(
            (bounds.min_x + bounds.max_x) * 0.5,
            (bounds.min_y + bounds.max_y) * 0.5,
        )
    })
}

fn transform_pline(
    pline: &Polyline<Real>,
    matrix: &Matrix4<Real>,
    preserve_arcs: bool,
    desired_orientation: PlineOrientation,
    arc_approx_error: Real,
) -> Option<Polyline<Real>> {
    let source = if preserve_arcs || !pline_has_arcs(pline) {
        pline.clone()
    } else {
        approximate_polyline(pline, arc_approx_error)
    };

    let a = matrix[(0, 0)];
    let b = matrix[(0, 1)];
    let tx = matrix[(0, 3)];
    let d = matrix[(1, 0)];
    let e = matrix[(1, 1)];
    let ty = matrix[(1, 3)];
    let reverses_orientation = (a * e - b * d) < 0.0;

    let mut transformed = Polyline::new();
    transformed.set_is_closed(source.is_closed());
    transformed.reserve(source.vertex_count());

    for vertex in source.iter_vertexes() {
        let x = a * vertex.x + b * vertex.y + tx;
        let y = d * vertex.x + e * vertex.y + ty;
        let bulge = if preserve_arcs && reverses_orientation {
            -vertex.bulge
        } else {
            vertex.bulge
        };
        transformed.add(x, y, bulge);
    }

    if !valid_closed_pline(&transformed) {
        return None;
    }

    if transformed.orientation() != desired_orientation {
        transformed.invert_direction_mut();
    }

    valid_closed_pline(&transformed).then_some(transformed)
}

fn pline_has_arcs(pline: &Polyline<Real>) -> bool {
    pline
        .iter_segments()
        .any(|(vertex, _)| !vertex.bulge_is_zero())
}

fn matrix_preserves_circular_arcs(matrix: &Matrix4<Real>) -> bool {
    let a = matrix[(0, 0)];
    let b = matrix[(0, 1)];
    let d = matrix[(1, 0)];
    let e = matrix[(1, 1)];

    let col0_len2 = a * a + d * d;
    let col1_len2 = b * b + e * e;
    let dot = a * b + d * e;
    let scale = col0_len2.max(col1_len2).max(1.0);
    let eps = tolerance().max(Real::EPSILON * 16.0) * scale;

    col0_len2 > eps
        && col1_len2 > eps
        && dot.abs() <= eps
        && (col0_len2 - col1_len2).abs() <= eps
}

fn same_xy(a: (Real, Real), b: (Real, Real)) -> bool {
    (a.0 - b.0).abs() <= tolerance() && (a.1 - b.1).abs() <= tolerance()
}
