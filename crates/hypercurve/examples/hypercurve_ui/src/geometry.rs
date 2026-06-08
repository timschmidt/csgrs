use std::f64::consts::PI;
use std::ops::{Index, IndexMut};

use geo::{BooleanOps, Buffer, Coord, LineString, MultiPolygon, Polygon};
use hypercurve::{
    BooleanOp as HBooleanOp, BulgeVertex2, Classification, Contour2, ContourFragmentSet,
    ContourIntersection, ContourIntersectionSet, ContourOperand, ContourSplitMarkers, CubicBezier2,
    CurvePolicy, CurveString2, FillRule, OffsetCap, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, Real, Region2, Segment2, Tolerance,
};
use serde::{Deserialize, Serialize};

type HPoint = Point2;
type HReal = Real;
type HSegment = Segment2;
type HContour = Contour2;
const DISPLAY_COORD_EPS: f64 = 2e-5;
const MIN_DISPLAY_LOOP_AREA: f64 = 1e-6;

/// A bulge polyline vertex. `bulge` describes the outgoing segment.
///
/// These `f64` fields are UI/editor records and Geo display data only.
/// Geometry operations lift them into hyperreal-backed `hypercurve` values at
/// the operation boundary before asking the exact curve kernel for topology.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
    pub bulge: f64,
}

impl Vertex {
    pub const fn new(x: f64, y: f64, bulge: f64) -> Self {
        Self { x, y, bulge }
    }

    fn validate_finite(self, index: usize) -> Result<(), String> {
        validate_finite(self.x, &format!("vertex {index} x"))?;
        validate_finite(self.y, &format!("vertex {index} y"))?;
        validate_finite(self.bulge, &format!("vertex {index} bulge"))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CurvePrimitive {
    Line {
        start: Vertex,
        end: Vertex,
    },
    CircularArc {
        start: Vertex,
        end: Vertex,
        bulge: f64,
    },
    QuadraticBezier {
        start: Vertex,
        control: Vertex,
        end: Vertex,
    },
    CubicBezier {
        start: Vertex,
        control1: Vertex,
        control2: Vertex,
        end: Vertex,
    },
    RationalQuadratic {
        start: Vertex,
        control: Vertex,
        end: Vertex,
        control_weight: f64,
    },
}

impl CurvePrimitive {
    fn handle_count(&self) -> usize {
        match self {
            Self::Line { .. } | Self::CircularArc { .. } => 2,
            Self::QuadraticBezier { .. } | Self::RationalQuadratic { .. } => 3,
            Self::CubicBezier { .. } => 4,
        }
    }

    fn handles(&self) -> Vec<Vertex> {
        match *self {
            Self::Line { start, end } | Self::CircularArc { start, end, .. } => vec![start, end],
            Self::QuadraticBezier {
                start,
                control,
                end,
            }
            | Self::RationalQuadratic {
                start,
                control,
                end,
                ..
            } => vec![start, control, end],
            Self::CubicBezier {
                start,
                control1,
                control2,
                end,
            } => vec![start, control1, control2, end],
        }
    }

    fn set_handle(&mut self, index: usize, x: f64, y: f64) {
        let target = match self {
            Self::Line { start, end } | Self::CircularArc { start, end, .. } => match index {
                0 => Some(start),
                1 => Some(end),
                _ => None,
            },
            Self::QuadraticBezier {
                start,
                control,
                end,
            }
            | Self::RationalQuadratic {
                start,
                control,
                end,
                ..
            } => match index {
                0 => Some(start),
                1 => Some(control),
                2 => Some(end),
                _ => None,
            },
            Self::CubicBezier {
                start,
                control1,
                control2,
                end,
            } => match index {
                0 => Some(start),
                1 => Some(control1),
                2 => Some(control2),
                3 => Some(end),
                _ => None,
            },
        };
        if let Some(vertex) = target {
            vertex.x = x;
            vertex.y = y;
        }
    }

    fn set_matching_handles(&mut self, old: Vertex, x: f64, y: f64) {
        for index in 0..self.handle_count() {
            if self
                .handles()
                .get(index)
                .is_some_and(|handle| same_vertex_position(*handle, old))
            {
                self.set_handle(index, x, y);
            }
        }
    }

    fn translate_mut(&mut self, dx: f64, dy: f64) {
        for index in 0..self.handle_count() {
            if let Some(vertex) = self.handles().get(index).copied() {
                self.set_handle(index, vertex.x + dx, vertex.y + dy);
            }
        }
    }

    fn validate_finite(&self, index: usize) -> Result<(), String> {
        for (handle_index, handle) in self.handles().into_iter().enumerate() {
            handle.validate_finite(handle_index)?;
        }
        if let Self::RationalQuadratic { control_weight, .. } = self {
            validate_finite(
                *control_weight,
                &format!("primitive {index} control weight"),
            )?;
        }
        Ok(())
    }

    fn append_samples(&self, points: &mut Vec<[f64; 2]>, max_angle_step: f64) {
        match *self {
            Self::Line { start, end } => {
                push_sample_point(points, [start.x, start.y]);
                push_sample_point(points, [end.x, end.y]);
            }
            Self::CircularArc { start, end, bulge } => {
                push_sample_point(points, [start.x, start.y]);
                append_segment_samples(
                    points,
                    Vertex::new(start.x, start.y, bulge),
                    end,
                    max_angle_step,
                );
            }
            Self::QuadraticBezier {
                start,
                control,
                end,
            } => {
                let curve = QuadraticBezier2::new(
                    point_from_vertex(start),
                    point_from_vertex(control),
                    point_from_vertex(end),
                );
                for vertex in sample_quadratic_vertices(&curve, 18) {
                    push_sample_point(points, [vertex.x, vertex.y]);
                }
            }
            Self::CubicBezier {
                start,
                control1,
                control2,
                end,
            } => {
                let curve = CubicBezier2::new(
                    point_from_vertex(start),
                    point_from_vertex(control1),
                    point_from_vertex(control2),
                    point_from_vertex(end),
                );
                for vertex in sample_cubic_vertices(&curve, 24) {
                    push_sample_point(points, [vertex.x, vertex.y]);
                }
            }
            Self::RationalQuadratic {
                start,
                control,
                end,
                control_weight,
            } => {
                let Ok(curve) = RationalQuadraticBezier2::try_unit_end_weights(
                    point_from_vertex(start),
                    point_from_vertex(control),
                    point_from_vertex(end),
                    Real::try_from(control_weight).unwrap_or_else(|_| Real::one()),
                ) else {
                    return;
                };
                for vertex in sample_rational_quadratic_vertices(&curve, 24) {
                    push_sample_point(points, [vertex.x, vertex.y]);
                }
            }
        }
    }

    fn reversed(self) -> Self {
        match self {
            Self::Line { start, end } => Self::Line {
                start: end,
                end: start,
            },
            Self::CircularArc { start, end, bulge } => Self::CircularArc {
                start: end,
                end: start,
                bulge: -bulge,
            },
            Self::QuadraticBezier {
                start,
                control,
                end,
            } => Self::QuadraticBezier {
                start: end,
                control,
                end: start,
            },
            Self::CubicBezier {
                start,
                control1,
                control2,
                end,
            } => Self::CubicBezier {
                start: end,
                control1: control2,
                control2: control1,
                end: start,
            },
            Self::RationalQuadratic {
                start,
                control,
                end,
                control_weight,
            } => Self::RationalQuadratic {
                start: end,
                control,
                end: start,
                control_weight,
            },
        }
    }
}

fn push_sample_point(points: &mut Vec<[f64; 2]>, point: [f64; 2]) {
    if points.last().is_none_or(|last| {
        (last[0] - point[0]).abs() > DISPLAY_COORD_EPS
            || (last[1] - point[1]).abs() > DISPLAY_COORD_EPS
    }) {
        points.push(point);
    }
}

fn same_vertex_position(first: Vertex, second: Vertex) -> bool {
    (first.x - second.x).abs() <= DISPLAY_COORD_EPS
        && (first.y - second.y).abs() <= DISPLAY_COORD_EPS
}

fn point_from_vertex(vertex: Vertex) -> Point2 {
    Point2::new(
        real_checked(vertex.x, "curve handle x").expect("finite curve handle x"),
        real_checked(vertex.y, "curve handle y").expect("finite curve handle y"),
    )
}

/// Editable bulge polyline used by the UI. Geometry operations convert this to
/// hypercurve curve strings or contours before doing any topology work.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Polyline {
    #[serde(default)]
    pub curve_data: Vec<CurvePrimitive>,
    #[serde(default)]
    pub vertex_data: Vec<Vertex>,
    pub is_closed: bool,
    #[serde(default)]
    pub is_hole: bool,
}

impl Polyline {
    pub const fn new() -> Self {
        Self {
            curve_data: Vec::new(),
            vertex_data: Vec::new(),
            is_closed: false,
            is_hole: false,
        }
    }

    pub fn from_curve_data(curve_data: Vec<CurvePrimitive>, is_closed: bool) -> Self {
        Self {
            curve_data,
            vertex_data: Vec::new(),
            is_closed,
            is_hole: false,
        }
    }

    pub fn marked_hole(mut self) -> Self {
        self.is_hole = true;
        self
    }

    #[cfg(test)]
    pub fn closed(vertices: &[(f64, f64, f64)]) -> Self {
        Self {
            curve_data: Vec::new(),
            vertex_data: vertices
                .iter()
                .map(|&(x, y, bulge)| Vertex::new(x, y, bulge))
                .collect(),
            is_closed: true,
            is_hole: false,
        }
    }

    pub fn add(&mut self, x: f64, y: f64, bulge: f64) {
        if self.curve_data.is_empty() {
            self.vertex_data.push(Vertex::new(x, y, bulge));
        }
    }

    pub fn remove(&mut self, index: usize) {
        if self.curve_data.is_empty() && index < self.vertex_data.len() {
            self.vertex_data.remove(index);
        }
    }

    pub fn handle(&self, index: usize) -> Option<Vertex> {
        if self.curve_data.is_empty() {
            self.vertex_data.get(index).copied()
        } else {
            self.handles().get(index).copied()
        }
    }

    pub fn set_handle(&mut self, index: usize, x: f64, y: f64) {
        if self.curve_data.is_empty() {
            if let Some(vertex) = self.vertex_data.get_mut(index) {
                vertex.x = x;
                vertex.y = y;
            }
            return;
        }

        let mut remaining = index;
        let mut old = None;
        for primitive in &mut self.curve_data {
            let count = primitive.handle_count();
            if remaining < count {
                old = primitive.handles().get(remaining).copied();
                primitive.set_handle(remaining, x, y);
                break;
            }
            remaining -= count;
        }

        if let Some(old) = old {
            for primitive in &mut self.curve_data {
                primitive.set_matching_handles(old, x, y);
            }
        }
    }

    pub const fn is_closed(&self) -> bool {
        self.is_closed
    }

    pub fn iter_vertexes(&self) -> impl DoubleEndedIterator<Item = &Vertex> {
        self.vertex_data.iter()
    }

    pub fn handles(&self) -> Vec<Vertex> {
        if self.curve_data.is_empty() {
            return self.vertex_data.clone();
        }
        self.curve_data
            .iter()
            .flat_map(CurvePrimitive::handles)
            .collect()
    }

    pub fn legacy_segments(&self) -> Vec<(Vertex, Vertex)> {
        let mut segments: Vec<_> = self
            .vertex_data
            .windows(2)
            .map(|pair| (pair[0], pair[1]))
            .collect();
        if self.is_closed && self.vertex_data.len() > 1 {
            segments.push((
                self.vertex_data[self.vertex_data.len() - 1],
                self.vertex_data[0],
            ));
        }
        segments
    }

    pub fn translate_mut(&mut self, dx: f64, dy: f64) {
        if !self.curve_data.is_empty() {
            for primitive in &mut self.curve_data {
                primitive.translate_mut(dx, dy);
            }
            return;
        }
        for vertex in &mut self.vertex_data {
            vertex.x += dx;
            vertex.y += dy;
        }
    }

    pub fn sample_points(&self, max_angle_step: f64) -> Vec<[f64; 2]> {
        if !self.curve_data.is_empty() {
            let mut points = Vec::new();
            for primitive in &self.curve_data {
                primitive.append_samples(&mut points, max_angle_step);
            }
            points.dedup_by(|a, b| {
                (a[0] - b[0]).abs() <= DISPLAY_COORD_EPS && (a[1] - b[1]).abs() <= DISPLAY_COORD_EPS
            });
            return points;
        }

        let mut points = Vec::new();
        let mut first = true;
        for (start, end) in self.legacy_segments() {
            if first {
                points.push([start.x, start.y]);
                first = false;
            }
            append_segment_samples(&mut points, start, end, max_angle_step);
        }
        points
    }

    pub fn signed_area_estimate(&self) -> f64 {
        if !self.is_closed || self.handles().len() < 2 {
            return 0.0;
        }

        let points = self.sample_points(0.04);
        signed_area_of_points(&points)
    }

    pub fn is_counter_clockwise(&self) -> bool {
        self.signed_area_estimate() >= 0.0
    }

    /// Validate that all editable UI coordinates are finite primitive floats.
    ///
    /// The UI stores `f64` values because egui, plotting, and Geo interop are
    /// primitive-float boundaries. Before any topology operation, those values
    /// must lift cleanly into hyperreal-backed Real values; non-finite values are
    /// reported as ordinary UI errors instead of reaching exact kernels.
    pub fn validate_finite(&self) -> Result<(), String> {
        for (index, vertex) in self.vertex_data.iter().copied().enumerate() {
            vertex.validate_finite(index)?;
        }
        for (index, primitive) in self.curve_data.iter().enumerate() {
            primitive
                .validate_finite(index)
                .map_err(|error| format!("curve primitive {index}: {error}"))?;
        }
        Ok(())
    }

    pub fn to_curve_string(&self) -> Result<CurveString2, String> {
        if !self.curve_data.is_empty() {
            return self.to_sampled_polyline(0.04).to_curve_string();
        }
        if self.vertex_data.len() < 2 {
            return Err("a curve string needs at least two vertices".into());
        }
        let vertices = self.hyper_vertices()?;
        CurveString2::from_bulge_vertices(&vertices[..]).map_err(|e| e.to_string())
    }

    pub fn to_contour(&self) -> Result<HContour, String> {
        if !self.curve_data.is_empty() {
            return self.to_sampled_polyline(0.04).to_contour();
        }
        if !self.is_closed {
            return Err("polyline must be closed".into());
        }
        if self.vertex_data.len() < 2 {
            return Err("a closed contour needs at least two vertices".into());
        }
        let vertices = self.hyper_vertices()?;
        Contour2::from_bulge_vertices_with_fill_rule(&vertices[..], FillRule::NonZero)
            .map_err(|e| e.to_string())
    }

    #[cfg(test)]
    pub fn offset_checked(&self, distance: f64) -> Result<Option<Self>, String> {
        let contour = self.to_contour()?;
        let distance = real_checked(distance, "offset distance")?;
        match contour
            .offset_left_checked(distance, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    #[cfg(test)]
    pub fn offset_for_display(&self, distance: f64) -> Result<Option<Self>, String> {
        Ok(self.offsets_for_display(distance)?.into_iter().next())
    }

    pub fn offsets_for_display(&self, distance: f64) -> Result<Vec<Self>, String> {
        self.validate_finite()?;
        validate_finite(distance, "offset distance")?;
        if self.is_closed
            && let Some(polygon) = polyline_to_geo_polygon(self)
        {
            let buffered = polygon.buffer(left_offset_buffer_distance(self, distance));
            return Ok(shape_from_geo(&buffered).into_polylines());
        }

        Ok(self.raw_offset(distance)?.into_iter().collect())
    }

    pub fn raw_offset(&self, distance: f64) -> Result<Option<Self>, String> {
        let distance = real_checked(distance, "offset distance")?;
        if self.is_closed {
            let contour = self.to_contour()?;
            match contour
                .offset_left_with_line_joins(distance, &policy())
                .map_err(|e| e.to_string())?
            {
                Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
                Classification::Uncertain(_) => Ok(None),
            }
        } else {
            let curve = self.to_curve_string()?;
            match curve
                .offset_left_with_line_joins(distance, &policy())
                .map_err(|e| e.to_string())?
            {
                Classification::Decided(curve) => {
                    Ok(Some(Self::from_segments(curve.segments(), false)))
                }
                Classification::Uncertain(_) => Ok(None),
            }
        }
    }

    pub fn outline(&self, distance: f64, cap: OffsetCap) -> Result<Option<Self>, String> {
        let curve = self.to_curve_string()?;
        let distance = real_checked(distance, "outline distance")?;
        match curve
            .offset_outline(distance, cap, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    pub fn raw_offset_segments(&self, distance: f64) -> Result<Vec<Self>, String> {
        let distance = real_checked(distance, "offset distance")?;
        let segments = if self.is_closed {
            self.to_contour()?.segments().to_vec()
        } else {
            self.to_curve_string()?.segments().to_vec()
        };
        let mut out = Vec::new();
        for segment in segments {
            match segment
                .offset_left(distance.clone(), &policy())
                .map_err(|e| e.to_string())?
            {
                Classification::Decided(offset) => out.push(Self::from_segments(&[offset], false)),
                Classification::Uncertain(_) => {}
            }
        }
        Ok(out)
    }

    pub fn from_contour(contour: &HContour) -> Self {
        Self::from_segments(contour.segments(), true)
    }

    pub fn from_segments(segments: &[HSegment], closed: bool) -> Self {
        let mut vertices = Vec::new();
        for segment in segments {
            vertices.push(vertex_for_segment_start(segment));
        }
        if !closed && let Some(last) = segments.last() {
            let (x, y) = hpoint_xy(last.end());
            vertices.push(Vertex::new(x, y, 0.0));
        }
        Self {
            curve_data: Vec::new(),
            vertex_data: vertices,
            is_closed: closed,
            is_hole: false,
        }
    }

    fn to_sampled_polyline(&self, max_angle_step: f64) -> Self {
        Self {
            curve_data: Vec::new(),
            vertex_data: self
                .sample_points(max_angle_step)
                .into_iter()
                .map(|point| Vertex::new(point[0], point[1], 0.0))
                .collect(),
            is_closed: self.is_closed,
            is_hole: self.is_hole,
        }
    }

    fn hyper_vertices(&self) -> Result<Vec<BulgeVertex2>, String> {
        self.vertex_data
            .iter()
            .enumerate()
            .map(|(index, vertex)| {
                Ok(BulgeVertex2::new(
                    Point2::new(
                        real_checked(vertex.x, &format!("vertex {index} x"))?,
                        real_checked(vertex.y, &format!("vertex {index} y"))?,
                    ),
                    real_checked(vertex.bulge, &format!("vertex {index} bulge"))?,
                ))
            })
            .collect()
    }
}

impl Index<usize> for Polyline {
    type Output = Vertex;

    fn index(&self, index: usize) -> &Self::Output {
        &self.vertex_data[index]
    }
}

impl IndexMut<usize> for Polyline {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.vertex_data[index]
    }
}

/// Multi-contour shape with explicit material and hole bins.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Shape {
    pub materials: Vec<Polyline>,
    pub holes: Vec<Polyline>,
}

pub fn curve_showcase_contour(origin_x: f64, origin_y: f64, scale: f64) -> Polyline {
    let a = demo_vertex(origin_x, origin_y, scale, -3.0, -1.0, 0.0);
    let b = demo_vertex(origin_x, origin_y, scale, -1.9, -1.25, 0.0);
    let c = demo_vertex(origin_x, origin_y, scale, -0.65, -0.7, 0.0);
    let d = demo_vertex(origin_x, origin_y, scale, 0.65, -0.65, 0.0);
    let e = demo_vertex(origin_x, origin_y, scale, 2.2, -0.8, 0.0);
    let f = demo_vertex(origin_x, origin_y, scale, 3.25, 1.1, 0.0);
    let g = demo_vertex(origin_x, origin_y, scale, 1.2, 1.75, 0.0);
    let h = demo_vertex(origin_x, origin_y, scale, -2.65, 1.15, 0.0);

    Polyline::from_curve_data(
        vec![
            CurvePrimitive::Line { start: a, end: b },
            CurvePrimitive::CircularArc {
                start: b,
                end: c,
                bulge: 0.46,
            },
            CurvePrimitive::QuadraticBezier {
                start: c,
                control: demo_vertex(origin_x, origin_y, scale, -0.05, 1.15, 0.0),
                end: d,
            },
            CurvePrimitive::CubicBezier {
                start: d,
                control1: demo_vertex(origin_x, origin_y, scale, 0.8, -2.15, 0.0),
                control2: demo_vertex(origin_x, origin_y, scale, 1.75, 0.95, 0.0),
                end: e,
            },
            CurvePrimitive::RationalQuadratic {
                start: e,
                control: demo_vertex(origin_x, origin_y, scale, 3.05, 1.0, 0.0),
                end: f,
                control_weight: 0.36,
            },
            CurvePrimitive::Line { start: f, end: g },
            CurvePrimitive::Line { start: g, end: h },
            CurvePrimitive::Line { start: h, end: a },
        ],
        true,
    )
}

pub fn curve_showcase_polylines(origin_x: f64, origin_y: f64, scale: f64) -> Vec<Polyline> {
    vec![
        curve_showcase_contour(origin_x, origin_y, scale),
        quadratic_lens(
            origin_x - 1.15 * scale,
            origin_y + 0.18 * scale,
            0.48 * scale,
            false,
        )
        .marked_hole(),
        cubic_lens(
            origin_x + 0.55 * scale,
            origin_y + 0.18 * scale,
            0.46 * scale,
            true,
        ),
        rational_lens(
            origin_x + 1.85 * scale,
            origin_y + 0.18 * scale,
            0.43 * scale,
            false,
        )
        .marked_hole(),
        circular_lens(
            origin_x + 0.15 * scale,
            origin_y - 0.56 * scale,
            0.38 * scale,
            false,
        )
        .marked_hole(),
    ]
}

impl Shape {
    pub fn from_materials(materials: Vec<Polyline>) -> Self {
        Self {
            materials,
            holes: Vec::new(),
        }
    }

    pub fn from_polylines(polylines: Vec<Polyline>) -> Self {
        let mut materials = Vec::new();
        let mut holes = Vec::new();
        for polyline in polylines {
            if polyline.handles().len() < 2 {
                continue;
            }
            if polyline.is_hole {
                holes.push(polyline);
            } else if polyline.is_counter_clockwise() {
                materials.push(polyline);
            } else {
                holes.push(polyline);
            }
        }
        Self { materials, holes }
    }

    pub fn from_region(region: &Region2) -> Self {
        Self {
            materials: region
                .material_contours()
                .iter()
                .map(Polyline::from_contour)
                .collect(),
            holes: region
                .hole_contours()
                .iter()
                .map(Polyline::from_contour)
                .collect(),
        }
    }

    pub fn translated(mut self, dx: f64, dy: f64) -> Self {
        for pline in self.materials.iter_mut().chain(self.holes.iter_mut()) {
            pline.translate_mut(dx, dy);
        }
        self
    }

    pub fn validate_finite(&self) -> Result<(), String> {
        for (index, material) in self.materials.iter().enumerate() {
            material
                .validate_finite()
                .map_err(|error| format!("material {index}: {error}"))?;
        }
        for (index, hole) in self.holes.iter().enumerate() {
            hole.validate_finite()
                .map_err(|error| format!("hole {index}: {error}"))?;
        }
        Ok(())
    }

    pub fn to_region(&self) -> Result<Region2, String> {
        self.validate_finite()?;
        let materials = self
            .materials
            .iter()
            .map(Polyline::to_contour)
            .collect::<Result<Vec<_>, _>>()?;
        let holes = self
            .holes
            .iter()
            .map(Polyline::to_contour)
            .collect::<Result<Vec<_>, _>>()?;
        let mut contours = Vec::with_capacity(materials.len() + holes.len());
        contours.extend(materials.iter().cloned());
        contours.extend(holes.iter().cloned());

        if let Some(region) = regularized_region(&materials, &holes)? {
            return Ok(region);
        }

        match Region2::from_boundary_contours(contours, &policy()).map_err(|e| e.to_string())? {
            Classification::Decided(region) => Ok(region),
            Classification::Uncertain(_) => Ok(Region2::new(materials, holes)),
        }
    }

    pub fn boolean(&self, other: &Self, op: BooleanMode) -> Result<Option<Self>, String> {
        self.validate_finite()?;
        other.validate_finite()?;
        let op = match op {
            BooleanMode::Union => HBooleanOp::Union,
            BooleanMode::Intersection => HBooleanOp::Intersection,
            BooleanMode::Difference => HBooleanOp::Difference,
            BooleanMode::Xor => HBooleanOp::Xor,
        };

        if let (Ok(first), Ok(second)) = (self.to_region(), other.to_region()) {
            match first.boolean_region(&second, op, FillRule::NonZero, &policy()) {
                Ok(Classification::Decided(region)) => return Ok(Some(Self::from_region(&region))),
                Ok(Classification::Uncertain(_)) | Err(_) => {}
            }
        }

        Ok(Some(geo_boolean_fallback(self, other, op)?))
    }

    pub fn offset_once(&self, distance: f64) -> Self {
        shape_from_geo(&shape_to_geo(self).buffer(-distance))
    }

    pub fn into_polylines(self) -> Vec<Polyline> {
        self.materials.into_iter().chain(self.holes).collect()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum BooleanMode {
    Union,
    Intersection,
    Difference,
    Xor,
}

pub fn policy() -> CurvePolicy {
    // The test article is an interactive rendering boundary, so it uses
    // `EdgePreview` for curve-local display tolerances. Hypercurve's
    // predicate policy inside this value remains strict, and the UI must not
    // treat sampled `f64`/Geo fallback output as exact topology provenance.
    // Hobby, "Practical Segment Intersection with Finite Precision Output"
    // (Computational Geometry 13(4), 199-214, 1999), is the relevant warning:
    // finite output is useful, but it needs explicit boundary handling.
    CurvePolicy::edge_preview(Tolerance::new(1e-7, 1e-7))
}

pub fn boolean_polylines(
    first: &Polyline,
    second: &Polyline,
    op: BooleanMode,
) -> Result<Option<Shape>, String> {
    Shape::from_materials(vec![first.clone()])
        .boolean(&Shape::from_materials(vec![second.clone()]), op)
}

pub fn contour_intersections(
    first: &Polyline,
    second: &Polyline,
) -> Result<(Vec<[f64; 2]>, Vec<Polyline>), String> {
    let first = first.to_contour()?;
    let second = second.to_contour()?;
    let events = first
        .intersect_contour(&second, &policy())
        .map_err(|e| e.to_string())?;
    let mut points = Vec::new();
    let mut overlaps = Vec::new();
    for event in events.events() {
        match event {
            ContourIntersection::Point(point) => points.push(hpoint_array(&point.point)),
            ContourIntersection::Overlap(overlap) => {
                overlaps.push(Polyline::from_segments(&[overlap.segment.clone()], false));
            }
            ContourIntersection::Uncertain(_) => {}
        }
    }
    Ok((points, overlaps))
}

pub fn contour_slices(
    first: &Polyline,
    second: &Polyline,
) -> Result<(Vec<Polyline>, Vec<Polyline>), String> {
    let first_contour = first.to_contour()?;
    let second_contour = second.to_contour()?;
    let events = first_contour
        .intersect_contour(&second_contour, &policy())
        .map_err(|e| e.to_string())?;
    let first_fragments = split_contour_for_slices(&first_contour, &events, ContourOperand::First)?;
    let second_fragments =
        split_contour_for_slices(&second_contour, &events, ContourOperand::Second)?;
    Ok((
        first_fragments
            .fragments()
            .iter()
            .map(|fragment| Polyline::from_segments(&[fragment.segment.clone()], false))
            .filter(display_slice_is_non_degenerate)
            .collect(),
        second_fragments
            .fragments()
            .iter()
            .map(|fragment| Polyline::from_segments(&[fragment.segment.clone()], false))
            .filter(display_slice_is_non_degenerate)
            .collect(),
    ))
}

fn display_slice_is_non_degenerate(slice: &Polyline) -> bool {
    let points = slice.sample_points(0.03);
    points.len() >= 2
        && points.windows(2).any(|pair| {
            (pair[0][0] - pair[1][0]).abs() > DISPLAY_COORD_EPS
                || (pair[0][1] - pair[1][1]).abs() > DISPLAY_COORD_EPS
        })
}

fn split_contour_for_slices(
    contour: &HContour,
    pair_events: &ContourIntersectionSet,
    operand: ContourOperand,
) -> Result<ContourFragmentSet, String> {
    // Slice mode is a visualization tool: it should expose every displayable
    // split but remain drawable when preview ordering cannot be certified. The
    // fallback to source fragments is intentionally local to the UI boundary;
    // exact library booleans still propagate uncertainty. This follows the
    // finite-output separation described by Hobby (1999) and avoids presenting
    // a broken branch graph as if it were exact topology.
    let policy = policy();
    let self_events = contour
        .intersect_self(&policy)
        .map_err(|error| error.to_string())?;
    let mut markers = ContourSplitMarkers::with_contour_endpoints(contour);

    match markers.merge_intersections(pair_events, operand, &policy) {
        Classification::Decided(()) => {}
        Classification::Uncertain(_) => return source_contour_fragments(contour),
    }
    match markers.merge_self_intersections(&self_events, &policy) {
        Classification::Decided(()) => {}
        Classification::Uncertain(_) => return source_contour_fragments(contour),
    }

    match ContourFragmentSet::from_split_markers(contour, &markers, &policy)
        .map_err(|error| error.to_string())?
    {
        Classification::Decided(fragments) => Ok(fragments),
        Classification::Uncertain(_) => source_contour_fragments(contour),
    }
}

fn source_contour_fragments(contour: &HContour) -> Result<ContourFragmentSet, String> {
    ContourFragmentSet::new(
        contour
            .segments()
            .iter()
            .cloned()
            .enumerate()
            .map(
                |(source_segment_index, segment)| hypercurve::ContourFragment {
                    source_segment_index,
                    source_range: hypercurve::ParamRange::new(Real::zero(), Real::one()),
                    segment,
                },
            )
            .collect(),
    )
    .map_err(|error| error.to_string())
}

fn signed_area_of_points(points: &[[f64; 2]]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    let mut twice_area = 0.0;
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        twice_area += current[0] * next[1] - next[0] * current[1];
    }
    0.5 * twice_area
}

fn signed_area_of_coords(coords: &[Coord<f64>]) -> f64 {
    if coords.len() < 3 {
        return 0.0;
    }

    let mut twice_area = 0.0;
    for index in 0..coords.len() {
        let current = coords[index];
        let next = coords[(index + 1) % coords.len()];
        twice_area += current.x * next.y - next.x * current.y;
    }
    0.5 * twice_area
}

fn regularized_region(
    materials: &[HContour],
    holes: &[HContour],
) -> Result<Option<Region2>, String> {
    let mut region = Region2::empty();

    for contour in materials {
        let next = Region2::from_material_contours(vec![contour.clone()]);
        region = match region
            .boolean_region(&next, HBooleanOp::Union, FillRule::NonZero, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(_) => return Ok(None),
        };
    }

    for contour in holes {
        let next = Region2::from_material_contours(vec![contour.clone()]);
        region = match region
            .boolean_region(&next, HBooleanOp::Difference, FillRule::NonZero, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(_) => return Ok(None),
        };
    }

    Ok(Some(region))
}

fn geo_boolean_fallback(first: &Shape, second: &Shape, op: HBooleanOp) -> Result<Shape, String> {
    // UI fallback only: this keeps the demo interactive for topology cases that
    // hypercurve reports as uncertain. The result is a lossy display artifact,
    // not a replacement for exact hypercurve boolean semantics.
    let first = shape_to_geo(first);
    let second = shape_to_geo(second);
    let result = match op {
        HBooleanOp::Union => first.union(&second),
        HBooleanOp::Intersection => first.intersection(&second),
        HBooleanOp::Difference => first.difference(&second),
        HBooleanOp::Xor => first.xor(&second),
    };
    Ok(shape_from_geo(&result))
}

fn shape_to_geo(shape: &Shape) -> MultiPolygon<f64> {
    let mut region = MultiPolygon(Vec::new());

    for material in &shape.materials {
        let Some(polygon) = polyline_to_geo_polygon(material) else {
            continue;
        };
        region = if region.0.is_empty() {
            MultiPolygon(vec![polygon])
        } else {
            region.union(&polygon)
        };
    }

    for hole in &shape.holes {
        let Some(polygon) = polyline_to_geo_polygon(hole) else {
            continue;
        };
        region = region.difference(&polygon);
    }

    region
}

fn left_offset_buffer_distance(polyline: &Polyline, distance: f64) -> f64 {
    if polyline.is_counter_clockwise() {
        -distance
    } else {
        distance
    }
}

fn polyline_to_geo_polygon(polyline: &Polyline) -> Option<Polygon<f64>> {
    let mut coords: Vec<_> = polyline
        .sample_points(SAMPLE_ANGLE_STEP_FOR_GEO)
        .into_iter()
        .map(|point| Coord {
            x: point[0],
            y: point[1],
        })
        .collect();
    close_geo_ring(&mut coords)?;
    Some(Polygon::new(LineString::new(coords), Vec::new()))
}

fn shape_from_geo(polygons: &MultiPolygon<f64>) -> Shape {
    let mut materials = Vec::new();
    let mut holes = Vec::new();
    for polygon in &polygons.0 {
        if let Some(material) = polyline_from_geo_ring(polygon.exterior()) {
            materials.push(material);
        }
        for interior in polygon.interiors() {
            if let Some(hole) = polyline_from_geo_ring(interior) {
                holes.push(hole);
            }
        }
    }
    Shape { materials, holes }
}

fn polyline_from_geo_ring(ring: &LineString<f64>) -> Option<Polyline> {
    let mut coords = ring.0.clone();
    if coords.len() > 1 && coords.first() == coords.last() {
        coords.pop();
    }
    sanitize_geo_ring_coords(&mut coords);
    if coords.len() < 3 {
        return None;
    }
    if signed_area_of_coords(&coords).abs() <= MIN_DISPLAY_LOOP_AREA {
        return None;
    }
    Some(Polyline {
        curve_data: Vec::new(),
        vertex_data: coords
            .into_iter()
            .map(|coord| Vertex::new(coord.x, coord.y, 0.0))
            .collect(),
        is_closed: true,
        is_hole: false,
    })
}

fn sanitize_geo_ring_coords(coords: &mut Vec<Coord<f64>>) {
    coords.dedup_by(|a, b| coords_nearly_same(*a, *b));
    if coords.len() > 1 && coords_nearly_same(coords[0], *coords.last().unwrap()) {
        coords.pop();
    }

    let mut changed = true;
    while changed && coords.len() >= 3 {
        changed = false;
        let mut index = 0;
        while index < coords.len() && coords.len() >= 3 {
            let previous = coords[(index + coords.len() - 1) % coords.len()];
            let current = coords[index];
            let next = coords[(index + 1) % coords.len()];
            if coords_nearly_same(previous, current)
                || coords_nearly_same(current, next)
                || coords_nearly_collinear(previous, current, next)
            {
                coords.remove(index);
                changed = true;
            } else {
                index += 1;
            }
        }
    }
}

fn coords_nearly_same(first: Coord<f64>, second: Coord<f64>) -> bool {
    (first.x - second.x).abs() <= DISPLAY_COORD_EPS
        && (first.y - second.y).abs() <= DISPLAY_COORD_EPS
}

fn coords_nearly_collinear(previous: Coord<f64>, current: Coord<f64>, next: Coord<f64>) -> bool {
    let abx = current.x - previous.x;
    let aby = current.y - previous.y;
    let bcx = next.x - current.x;
    let bcy = next.y - current.y;
    let cross = abx * bcy - aby * bcx;
    let scale = (abx.hypot(aby) + bcx.hypot(bcy)).max(1.0);
    cross.abs() <= DISPLAY_COORD_EPS * scale
}

fn close_geo_ring(coords: &mut Vec<Coord<f64>>) -> Option<()> {
    if coords.len() < 3 {
        return None;
    }
    if coords.first() != coords.last() {
        let first = *coords.first()?;
        coords.push(first);
    }
    if coords.len() < 4 { None } else { Some(()) }
}

const SAMPLE_ANGLE_STEP_FOR_GEO: f64 = 0.04;

fn real_checked(value: f64, label: &str) -> Result<HReal, String> {
    // UI/editor coordinates are accepted only as finite edge values and are
    // lifted to the exact binary rational represented by the `f64`.
    validate_finite(value, label)?;
    HReal::try_from(value).map_err(|_| format!("{label} could not be lifted exactly"))
}

fn validate_finite(value: f64, label: &str) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{label} must be finite"))
    }
}

fn hpoint_array(point: &HPoint) -> [f64; 2] {
    let (x, y) = hpoint_xy(point);
    [x, y]
}

fn hpoint_xy(point: &HPoint) -> (f64, f64) {
    (real_to_f64(point.x()), real_to_f64(point.y()))
}

fn real_to_f64(value: &HReal) -> f64 {
    value
        .to_f64_lossy()
        .unwrap_or_else(|| f64::from(value.clone()))
}

fn demo_vertex(origin_x: f64, origin_y: f64, scale: f64, x: f64, y: f64, bulge: f64) -> Vertex {
    Vertex::new(origin_x + x * scale, origin_y + y * scale, bulge)
}

fn quadratic_lens(origin_x: f64, origin_y: f64, scale: f64, ccw: bool) -> Polyline {
    let a = demo_vertex(origin_x, origin_y, scale, -1.0, 0.0, 0.0);
    let b = demo_vertex(origin_x, origin_y, scale, 1.0, 0.0, 0.0);
    let c = demo_vertex(origin_x, origin_y, scale, 1.0, -0.42, 0.0);
    let d = demo_vertex(origin_x, origin_y, scale, -1.0, -0.42, 0.0);
    oriented_curve_data(
        vec![
            CurvePrimitive::QuadraticBezier {
                start: a,
                control: demo_vertex(origin_x, origin_y, scale, 0.0, 0.85, 0.0),
                end: b,
            },
            CurvePrimitive::Line { start: b, end: c },
            CurvePrimitive::Line { start: c, end: d },
            CurvePrimitive::Line { start: d, end: a },
        ],
        ccw,
    )
}

fn cubic_lens(origin_x: f64, origin_y: f64, scale: f64, ccw: bool) -> Polyline {
    let a = demo_vertex(origin_x, origin_y, scale, -1.0, -0.1, 0.0);
    let b = demo_vertex(origin_x, origin_y, scale, 1.0, 0.12, 0.0);
    oriented_curve_data(
        vec![
            CurvePrimitive::CubicBezier {
                start: a,
                control1: demo_vertex(origin_x, origin_y, scale, -0.45, 0.95, 0.0),
                control2: demo_vertex(origin_x, origin_y, scale, 0.45, -0.75, 0.0),
                end: b,
            },
            CurvePrimitive::CubicBezier {
                start: b,
                control1: demo_vertex(origin_x, origin_y, scale, 0.5, -0.9, 0.0),
                control2: demo_vertex(origin_x, origin_y, scale, -0.55, 0.35, 0.0),
                end: a,
            },
        ],
        ccw,
    )
}

fn rational_lens(origin_x: f64, origin_y: f64, scale: f64, ccw: bool) -> Polyline {
    let a = demo_vertex(origin_x, origin_y, scale, -1.0, -0.05, 0.0);
    let b = demo_vertex(origin_x, origin_y, scale, 1.0, -0.05, 0.0);
    let c = demo_vertex(origin_x, origin_y, scale, 1.0, -0.48, 0.0);
    let d = demo_vertex(origin_x, origin_y, scale, -1.0, -0.48, 0.0);
    oriented_curve_data(
        vec![
            CurvePrimitive::RationalQuadratic {
                start: a,
                control: demo_vertex(origin_x, origin_y, scale, 0.0, 1.1, 0.0),
                end: b,
                control_weight: 0.34,
            },
            CurvePrimitive::Line { start: b, end: c },
            CurvePrimitive::Line { start: c, end: d },
            CurvePrimitive::Line { start: d, end: a },
        ],
        ccw,
    )
}

fn circular_lens(origin_x: f64, origin_y: f64, scale: f64, ccw: bool) -> Polyline {
    let a = demo_vertex(origin_x, origin_y, scale, -0.9, 0.0, 0.0);
    let b = demo_vertex(origin_x, origin_y, scale, 0.9, 0.0, 0.0);
    oriented_curve_data(
        vec![
            CurvePrimitive::CircularArc {
                start: a,
                end: b,
                bulge: 0.42,
            },
            CurvePrimitive::CircularArc {
                start: b,
                end: a,
                bulge: 0.42,
            },
        ],
        ccw,
    )
}

fn sample_quadratic_vertices(curve: &QuadraticBezier2, steps: usize) -> Vec<Vertex> {
    (0..=steps)
        .map(|index| curve.point_at(Real::try_from(index as f64 / steps as f64).unwrap()))
        .map(vertex_from_point)
        .collect()
}

fn sample_cubic_vertices(curve: &CubicBezier2, steps: usize) -> Vec<Vertex> {
    (0..=steps)
        .map(|index| curve.point_at(Real::try_from(index as f64 / steps as f64).unwrap()))
        .map(vertex_from_point)
        .collect()
}

fn sample_rational_quadratic_vertices(
    curve: &RationalQuadraticBezier2,
    steps: usize,
) -> Vec<Vertex> {
    (0..=steps)
        .filter_map(|index| {
            match curve.point_at(
                Real::try_from(index as f64 / steps as f64).unwrap(),
                &policy(),
            ) {
                Classification::Decided(point) => Some(vertex_from_point(point)),
                Classification::Uncertain(_) => None,
            }
        })
        .collect()
}

fn vertex_from_point(point: Point2) -> Vertex {
    let (x, y) = hpoint_xy(&point);
    Vertex::new(x, y, 0.0)
}

fn oriented_curve_data(mut curve_data: Vec<CurvePrimitive>, ccw: bool) -> Polyline {
    let polyline = Polyline::from_curve_data(curve_data.clone(), true);
    if polyline.is_counter_clockwise() != ccw {
        curve_data = reverse_curve_data(curve_data);
    }
    Polyline::from_curve_data(curve_data, true)
}

fn reverse_curve_data(curve_data: Vec<CurvePrimitive>) -> Vec<CurvePrimitive> {
    curve_data
        .into_iter()
        .rev()
        .map(CurvePrimitive::reversed)
        .collect()
}

fn vertex_for_segment_start(segment: &HSegment) -> Vertex {
    match segment {
        Segment2::Line(line) => {
            let (x, y) = hpoint_xy(line.start());
            Vertex::new(x, y, 0.0)
        }
        Segment2::Arc(arc) => {
            let (x, y) = hpoint_xy(arc.start());
            Vertex::new(x, y, bulge_for_arc(arc))
        }
    }
}

fn bulge_for_arc(arc: &hypercurve::CircularArc2) -> f64 {
    if let Some(bulge) = arc.bulge() {
        return real_to_f64(bulge);
    }

    let (sx, sy) = hpoint_xy(arc.start());
    let (ex, ey) = hpoint_xy(arc.end());
    let (cx, cy) = hpoint_xy(arc.center());
    let start_angle = (sy - cy).atan2(sx - cx);
    let end_angle = (ey - cy).atan2(ex - cx);
    let mut ccw = end_angle - start_angle;
    while ccw <= 0.0 {
        ccw += 2.0 * PI;
    }
    while ccw > 2.0 * PI {
        ccw -= 2.0 * PI;
    }
    let sweep = if arc.is_clockwise() {
        -(2.0 * PI - ccw)
    } else {
        ccw
    };
    (sweep / 4.0).tan()
}

fn append_segment_samples(
    points: &mut Vec<[f64; 2]>,
    start: Vertex,
    end: Vertex,
    max_angle_step: f64,
) {
    if start.bulge.abs() < 1e-12 {
        points.push([end.x, end.y]);
        return;
    }
    let Some((center_x, center_y)) = arc_center_from_bulge(start, end) else {
        points.push([end.x, end.y]);
        return;
    };
    let sweep = 4.0 * start.bulge.atan();
    let steps = ((sweep.abs() / max_angle_step.max(0.01)).ceil() as usize).clamp(4, 96);
    let radius = ((start.x - center_x).powi(2) + (start.y - center_y).powi(2)).sqrt();
    let start_angle = (start.y - center_y).atan2(start.x - center_x);
    for step in 1..=steps {
        let t = step as f64 / steps as f64;
        let angle = start_angle + sweep * t;
        points.push([
            center_x + radius * angle.cos(),
            center_y + radius * angle.sin(),
        ]);
    }
}

fn arc_center_from_bulge(start: Vertex, end: Vertex) -> Option<(f64, f64)> {
    let b = start.bulge;
    if b.abs() < 1e-12 {
        return None;
    }
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let factor = (1.0 - b * b) / (4.0 * b);
    Some((
        (start.x + end.x) * 0.5 - dy * factor,
        (start.y + end.y) * 0.5 + dx * factor,
    ))
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    const SAMPLE_STEP: f64 = 0.03;
    const GEOM_EPS: f64 = 1e-7;

    #[test]
    fn display_offset_clips_default_article_shape_instead_of_showing_raw_self_contacts() {
        let source = default_article_polyline();

        assert!(
            source.offset_checked(1.0).unwrap().is_none(),
            "the raw hypercurve offset should be recognized as needing clipping"
        );
        assert!(source.offset_for_display(1.0).unwrap().is_some());
        assert_valid_offset_set(&source.offsets_for_display(1.0).unwrap(), true);
    }

    #[test]
    fn contour_slices_include_nonadjacent_line_arc_self_intersections() {
        let first = Polyline::closed(&[
            (0.0, 0.0, 1.0),
            (2.0, 0.0, 0.0),
            (3.0, 2.0, 0.0),
            (1.0, 2.0, 0.0),
            (1.0, -2.0, 0.0),
            (3.0, -3.0, 0.0),
            (-1.0, -3.0, 0.0),
        ]);
        let second = Polyline::closed(&[
            (20.0, 20.0, 0.0),
            (22.0, 20.0, 0.0),
            (22.0, 22.0, 0.0),
            (20.0, 22.0, 0.0),
        ]);

        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_eq!(first_slices.len(), 9);
        assert_eq!(second_slices.len(), 4);
    }

    #[test]
    fn contour_slices_include_adjacent_line_arc_crossings_beyond_shared_endpoint() {
        let first = Polyline::closed(&[
            (0.0, 0.0, 1.0),
            (2.0, 0.0, 0.0),
            (0.0, -2.0, 0.0),
            (-1.0, 0.0, 0.0),
        ]);
        let second = Polyline::closed(&[
            (20.0, 20.0, 0.0),
            (22.0, 20.0, 0.0),
            (22.0, 22.0, 0.0),
            (20.0, 22.0, 0.0),
        ]);

        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_eq!(first_slices.len(), 6);
        assert_eq!(second_slices.len(), 4);
    }

    #[test]
    fn contour_slices_handle_dense_multipolygon_style_linework() {
        let first = alternating_band_polyline(9, 0.0, 0.0, 1.0);
        let second = alternating_band_polyline(9, 0.45, 0.25, -1.0);

        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_valid_slice_set(&first_slices, true);
        assert_valid_slice_set(&second_slices, true);
    }

    #[test]
    fn contour_slices_keep_display_fragments_for_many_line_arc_events() {
        let first = radial_polyline_with_transform(
            9,
            &[
                0.55,
                0.55,
                0.55,
                0.55,
                1.0102264538592962,
                0.753525233986273,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[0.0; 12],
            0.0,
            0.0,
            0.0,
        );
        let second = radial_polyline_with_transform(
            9,
            &[
                0.55,
                0.55,
                0.55,
                1.0777534861273332,
                1.2886771796815553,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[
                0.0,
                0.0,
                0.0,
                -0.5614702594038522,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            0.6637692991378273,
            -1.7664711101724753,
            0.6566402803495361,
        );

        assert!(contour_has_slice_events(&first, &second).unwrap());
        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_valid_slice_set(&first_slices, true);
        assert_valid_slice_set(&second_slices, true);
    }

    #[test]
    fn contour_slices_keep_display_fragments_for_self_arc_events() {
        let first = radial_polyline_with_transform(
            11,
            &[
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                1.3184567971532413,
                0.9584085075790264,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.8094809229883586,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            0.0,
            0.0,
            0.0,
        );
        let second = radial_polyline_with_transform(
            11,
            &[
                0.55,
                0.55,
                0.55,
                0.55,
                1.245577180132649,
                0.6548306493289698,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[0.0; 12],
            0.0,
            -2.408158343355632,
            0.7955786457885817,
        );

        assert!(contour_has_slice_events(&first, &second).unwrap());
        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_valid_slice_set(&first_slices, true);
        assert_valid_slice_set(&second_slices, true);
    }

    #[test]
    fn contour_slices_keep_display_fragments_for_small_arc_triangle() {
        let first = radial_polyline_with_transform(
            3,
            &[
                1.0006825808205817,
                1.0673754962372333,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[
                0.0,
                -0.7886604849578752,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            0.0,
            0.0,
            0.0,
        );
        let second = radial_polyline_with_transform(
            3,
            &[
                0.55,
                1.2160624638373176,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
                0.55,
            ],
            &[0.0; 12],
            3.0610844304447027,
            3.025470516022391,
            0.8196939276745006,
        );

        assert!(contour_has_slice_events(&first, &second).unwrap());
        let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

        assert_valid_slice_set(&first_slices, true);
        assert_valid_slice_set(&second_slices, true);
    }

    #[test]
    fn clipped_offsets_handle_convex_line_line_corners_across_angles() {
        for degrees in [8.0_f64, 15.0, 30.0, 60.0, 90.0, 120.0, 150.0, 172.0] {
            let theta = degrees.to_radians();
            let source = Polyline::closed(&[
                (0.0, 0.0, 0.0),
                (32.0, 0.0, 0.0),
                (32.0 * theta.cos(), 32.0 * theta.sin(), 0.0),
            ]);

            assert_valid_offset_set(&source.offsets_for_display(0.35).unwrap(), true);
        }
    }

    #[test]
    fn clipped_offsets_handle_reflex_line_line_corners_across_angles() {
        for width in [0.35_f64, 0.75, 1.5, 3.0, 5.0] {
            let source = Polyline::closed(&[
                (0.0, 0.0, 0.0),
                (20.0, 0.0, 0.0),
                (20.0, 12.0, 0.0),
                (10.0 + width, 12.0, 0.0),
                (10.0, 7.0, 0.0),
                (10.0 - width, 12.0, 0.0),
                (0.0, 12.0, 0.0),
            ]);

            assert_valid_offset_set(&source.offsets_for_display(0.8).unwrap(), false);
        }
    }

    #[test]
    fn clipped_offsets_handle_line_arc_corners() {
        let cases = [
            Polyline::closed(&[
                (0.0, 0.0, 0.0),
                (10.0, 0.0, 0.55),
                (10.0, 8.0, 0.0),
                (0.0, 8.0, 0.0),
            ]),
            Polyline::closed(&[
                (0.0, 0.0, 0.0),
                (14.0, 0.0, -0.45),
                (14.0, 8.0, 0.0),
                (6.5, 3.5, 0.0),
                (0.0, 8.0, 0.35),
            ]),
        ];

        for source in cases {
            assert_valid_offset_set(&source.offsets_for_display(0.75).unwrap(), true);
            assert_valid_offset_set(&source.offsets_for_display(-0.75).unwrap(), true);
        }
    }

    #[test]
    fn clipped_offsets_handle_arc_arc_corners() {
        let cases = [
            Polyline::closed(&[
                (0.0, 0.0, 0.25),
                (8.0, 0.0, 0.25),
                (8.0, 8.0, 0.25),
                (0.0, 8.0, 0.25),
            ]),
            Polyline::closed(&[
                (0.0, 0.0, -0.15),
                (9.0, 0.0, 0.35),
                (9.0, 6.0, -0.15),
                (0.0, 6.0, 0.35),
            ]),
        ];

        for source in cases {
            assert_valid_offset_set(&source.offsets_for_display(0.25).unwrap(), true);
            assert_valid_offset_set(&source.offsets_for_display(-0.25).unwrap(), true);
        }
    }

    #[test]
    fn shape_offset_clips_between_nearby_loops() {
        let shape = Shape::from_polylines(vec![
            Polyline::closed(&[
                (0.0, 0.0, 0.0),
                (18.0, 0.0, 0.0),
                (18.0, 10.0, 0.0),
                (0.0, 10.0, 0.0),
            ]),
            Polyline::closed(&[
                (6.0, 3.0, 0.0),
                (6.0, 7.0, 0.0),
                (12.0, 7.0, 0.0),
                (12.0, 3.0, 0.0),
            ]),
        ]);

        let offset = shape.offset_once(1.25);
        assert_valid_offset_set(&offset.materials, true);
        assert_valid_offset_set(&offset.holes, false);
    }

    #[test]
    fn non_finite_ui_values_are_reported_before_exact_lifting() {
        let invalid = Polyline::closed(&[(0.0, 0.0, 0.0), (f64::NAN, 0.0, 0.0), (1.0, 1.0, 0.0)]);
        let valid = Polyline::closed(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]);

        assert!(invalid.to_contour().unwrap_err().contains("must be finite"));
        assert!(
            invalid
                .offsets_for_display(1.0)
                .unwrap_err()
                .contains("must be finite")
        );
        assert!(
            Shape::from_materials(vec![invalid])
                .boolean(&Shape::from_materials(vec![valid]), BooleanMode::Union)
                .unwrap_err()
                .contains("must be finite")
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 512,
            max_shrink_iters: 128,
            ..ProptestConfig::default()
        })]

        #[test]
        fn clipped_offset_fuzzes_line_line_line_arc_and_arc_arc_corners(
            vertex_count in 3_usize..10,
            radius_scale in proptest::collection::vec(0.65_f64..1.35, 10),
            bulge_values in proptest::collection::vec(-0.65_f64..0.65, 10),
            distance in -1.25_f64..1.25,
        ) {
            let distance = if distance.abs() < 0.05 { 0.05 } else { distance };
            let source = radial_fuzz_polyline(vertex_count, &radius_scale, &bulge_values);
            let offsets = source.offsets_for_display(distance).unwrap();
            assert_valid_offset_set(&offsets, false);
        }

        #[test]
        fn contour_slices_fuzz_dense_intersection_sets(
            bands in 4_usize..14,
            dx in -1.25_f64..1.25,
            dy in -1.25_f64..1.25,
            first_skew in -0.85_f64..0.85,
            second_skew in -0.85_f64..0.85,
        ) {
            let first = alternating_band_polyline(bands, first_skew, 0.0, 1.0);
            let second = alternating_band_polyline(bands, second_skew + dx, dy, -1.0);

            let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

            assert_valid_slice_set(&first_slices, true);
            assert_valid_slice_set(&second_slices, true);
        }

        #[test]
        fn contour_slices_fuzz_arc_heavy_display_state(
            vertex_count in 3_usize..12,
            first_radii in proptest::collection::vec(0.55_f64..1.45, 12),
            second_radii in proptest::collection::vec(0.55_f64..1.45, 12),
            first_bulges in proptest::collection::vec(-0.95_f64..0.95, 12),
            second_bulges in proptest::collection::vec(-0.95_f64..0.95, 12),
            dx in -4.0_f64..4.0,
            dy in -4.0_f64..4.0,
            angle_shift in 0.0_f64..0.9,
        ) {
            let first = radial_polyline_with_transform(
                vertex_count,
                &first_radii,
                &first_bulges,
                0.0,
                0.0,
                0.0,
            );
            let second = radial_polyline_with_transform(
                vertex_count,
                &second_radii,
                &second_bulges,
                dx,
                dy,
                angle_shift,
            );

            let first_has_events = contour_has_slice_events(&first, &second).unwrap();
            let (first_slices, second_slices) = contour_slices(&first, &second).unwrap();

            assert_valid_slice_set(&first_slices, first_has_events);
            assert_valid_slice_set(&second_slices, first_has_events);
        }
    }

    fn default_article_polyline() -> Polyline {
        Polyline::closed(&[
            (10.0, 10.0, -0.5),
            (8.0, 9.0, 0.374794619217547),
            (21.0, 0.0, 0.0),
            (23.0, 0.0, 1.0),
            (32.0, 0.0, -0.5),
            (28.0, 0.0, 0.5),
            (39.0, 21.0, 0.0),
            (28.0, 12.0, 0.5),
        ])
    }

    fn radial_fuzz_polyline(
        vertex_count: usize,
        radius_scale: &[f64],
        bulge_values: &[f64],
    ) -> Polyline {
        radial_polyline_with_transform(vertex_count, radius_scale, bulge_values, 0.0, 0.0, 0.0)
    }

    fn radial_polyline_with_transform(
        vertex_count: usize,
        radius_scale: &[f64],
        bulge_values: &[f64],
        dx: f64,
        dy: f64,
        angle_shift: f64,
    ) -> Polyline {
        let vertices: Vec<_> = (0..vertex_count)
            .map(|index| {
                let angle =
                    angle_shift + index as f64 * std::f64::consts::TAU / vertex_count as f64;
                let radius = 12.0 * radius_scale[index];
                let bulge = if index % 4 == 0 {
                    0.0
                } else {
                    bulge_values[index]
                };
                (dx + radius * angle.cos(), dy + radius * angle.sin(), bulge)
            })
            .collect();
        Polyline::closed(&vertices)
    }

    fn contour_has_slice_events(first: &Polyline, second: &Polyline) -> Result<bool, String> {
        let first = first.to_contour()?;
        let second = second.to_contour()?;
        let policy = policy();
        Ok(!first
            .intersect_contour(&second, &policy)
            .map_err(|error| error.to_string())?
            .is_empty()
            || !first
                .intersect_self(&policy)
                .map_err(|error| error.to_string())?
                .is_empty()
            || !second
                .intersect_self(&policy)
                .map_err(|error| error.to_string())?
                .is_empty())
    }

    fn alternating_band_polyline(
        bands: usize,
        skew: f64,
        y_offset: f64,
        direction: f64,
    ) -> Polyline {
        let mut vertices = Vec::with_capacity(bands * 2 + 2);
        let height = 18.0;
        let step = 2.0;
        vertices.push((0.0, y_offset, 0.0));
        for index in 0..=bands {
            let x = index as f64 * step;
            let top_x = x + skew * (index as f64 / bands.max(1) as f64);
            if index % 2 == 0 {
                vertices.push((top_x, y_offset + direction * height, 0.0));
            } else {
                vertices.push((x - skew, y_offset - direction * height * 0.12, 0.0));
            }
        }
        vertices.push((bands as f64 * step + 1.5, y_offset, 0.0));
        Polyline::closed(&vertices)
    }

    fn assert_valid_slice_set(slices: &[Polyline], require_non_empty: bool) {
        if require_non_empty {
            assert!(!slices.is_empty(), "expected at least one slice");
        }

        for slice in slices {
            assert!(!slice.is_closed(), "slices should be open fragments");
            assert!(
                slice.vertex_data.len() >= 2,
                "slice fragments should have at least two vertices"
            );
            for vertex in &slice.vertex_data {
                assert!(vertex.x.is_finite(), "slice vertex x must be finite");
                assert!(vertex.y.is_finite(), "slice vertex y must be finite");
                assert!(
                    vertex.bulge.is_finite(),
                    "slice vertex bulge must be finite"
                );
            }
            let points = slice.sample_points(SAMPLE_STEP);
            assert!(
                points.len() >= 2,
                "slice sampling should retain at least two points"
            );
            assert!(
                points
                    .windows(2)
                    .any(|pair| !nearly_same_point(pair[0], pair[1])),
                "slice should not collapse to a zero-length display fragment"
            );
        }
    }

    fn assert_valid_offset_set(polylines: &[Polyline], require_non_empty: bool) {
        if require_non_empty {
            assert!(
                !polylines.is_empty(),
                "expected at least one clipped offset loop"
            );
        }

        for polyline in polylines {
            assert!(polyline.is_closed(), "offset loops must be closed");
            assert!(
                polyline.vertex_data.len() >= 3,
                "offset loops must have at least three vertices"
            );
            assert!(
                polyline.signed_area_estimate().abs() > MIN_DISPLAY_LOOP_AREA,
                "offset loops must enclose measurable area"
            );
            assert!(
                !sampled_polyline_has_self_intersections(polyline),
                "offset loop should be clipped to simple sampled linework: {polyline:?}"
            );
        }
    }

    fn sampled_polyline_has_self_intersections(polyline: &Polyline) -> bool {
        let mut points = polyline.sample_points(SAMPLE_STEP);
        points.dedup_by(|a, b| nearly_same_point(*a, *b));
        if points.len() < 4 {
            return false;
        }
        if !nearly_same_point(points[0], *points.last().unwrap()) {
            points.push(points[0]);
        }

        let segment_count = points.len() - 1;
        for first in 0..segment_count {
            for second in (first + 1)..segment_count {
                if sampled_segments_are_adjacent(first, second, segment_count) {
                    continue;
                }
                if sampled_segments_intersect(
                    points[first],
                    points[first + 1],
                    points[second],
                    points[second + 1],
                ) {
                    return true;
                }
            }
        }

        false
    }

    fn sampled_segments_are_adjacent(first: usize, second: usize, len: usize) -> bool {
        first.abs_diff(second) == 1 || (first == 0 && second + 1 == len)
    }

    fn sampled_segments_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
        if !sampled_boxes_overlap(a, b, c, d) {
            return false;
        }

        let ab_c = orient(a, b, c);
        let ab_d = orient(a, b, d);
        let cd_a = orient(c, d, a);
        let cd_b = orient(c, d, b);

        if ab_c.abs() <= GEOM_EPS && point_on_sampled_segment(c, a, b) {
            return true;
        }
        if ab_d.abs() <= GEOM_EPS && point_on_sampled_segment(d, a, b) {
            return true;
        }
        if cd_a.abs() <= GEOM_EPS && point_on_sampled_segment(a, c, d) {
            return true;
        }
        if cd_b.abs() <= GEOM_EPS && point_on_sampled_segment(b, c, d) {
            return true;
        }

        (ab_c > GEOM_EPS) != (ab_d > GEOM_EPS) && (cd_a > GEOM_EPS) != (cd_b > GEOM_EPS)
    }

    fn sampled_boxes_overlap(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
        a[0].min(b[0]) <= c[0].max(d[0]) + GEOM_EPS
            && c[0].min(d[0]) <= a[0].max(b[0]) + GEOM_EPS
            && a[1].min(b[1]) <= c[1].max(d[1]) + GEOM_EPS
            && c[1].min(d[1]) <= a[1].max(b[1]) + GEOM_EPS
    }

    fn point_on_sampled_segment(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> bool {
        point[0] >= start[0].min(end[0]) - GEOM_EPS
            && point[0] <= start[0].max(end[0]) + GEOM_EPS
            && point[1] >= start[1].min(end[1]) - GEOM_EPS
            && point[1] <= start[1].max(end[1]) + GEOM_EPS
    }

    fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    }

    fn nearly_same_point(first: [f64; 2], second: [f64; 2]) -> bool {
        (first[0] - second[0]).abs() <= GEOM_EPS && (first[1] - second[1]).abs() <= GEOM_EPS
    }
}
