use std::f64::consts::PI;
use std::ops::{Index, IndexMut};

use hypercurve::{
    ApproxBackend, BooleanOp as HBooleanOp, BulgeVertex2, Classification, Contour2,
    ContourIntersection, CurvePolicy, CurveString2, FillRule, OffsetCap, Point2, Region2, Scalar,
    Segment2, Tolerance,
};
use serde::{Deserialize, Serialize};

type Backend = ApproxBackend;
type HPoint = Point2<Backend>;
type HScalar = Scalar<Backend>;
type HSegment = Segment2<Backend>;
type HContour = Contour2<Backend>;

/// A Cavalier-style polyline vertex. `bulge` describes the outgoing segment.
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
}

/// Editable bulge polyline used by the UI. Geometry operations convert this to
/// hypercurve curve strings or contours before doing any topology work.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Polyline {
    pub vertex_data: Vec<Vertex>,
    pub is_closed: bool,
}

impl Polyline {
    pub const fn new() -> Self {
        Self {
            vertex_data: Vec::new(),
            is_closed: false,
        }
    }

    pub fn closed(vertices: &[(f64, f64, f64)]) -> Self {
        Self {
            vertex_data: vertices
                .iter()
                .map(|&(x, y, bulge)| Vertex::new(x, y, bulge))
                .collect(),
            is_closed: true,
        }
    }

    pub fn add(&mut self, x: f64, y: f64, bulge: f64) {
        self.vertex_data.push(Vertex::new(x, y, bulge));
    }

    pub fn remove(&mut self, index: usize) {
        if index < self.vertex_data.len() {
            self.vertex_data.remove(index);
        }
    }

    pub fn set(&mut self, index: usize, x: f64, y: f64, bulge: f64) {
        if let Some(vertex) = self.vertex_data.get_mut(index) {
            *vertex = Vertex::new(x, y, bulge);
        }
    }

    pub fn get(&self, index: usize) -> Option<&Vertex> {
        self.vertex_data.get(index)
    }

    pub const fn is_closed(&self) -> bool {
        self.is_closed
    }

    pub fn iter_vertexes(&self) -> impl DoubleEndedIterator<Item = &Vertex> {
        self.vertex_data.iter()
    }

    pub fn segments(&self) -> Vec<(Vertex, Vertex)> {
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
        for vertex in &mut self.vertex_data {
            vertex.x += dx;
            vertex.y += dy;
        }
    }

    pub fn scale_mut(&mut self, scale: f64) {
        for vertex in &mut self.vertex_data {
            vertex.x *= scale;
            vertex.y *= scale;
        }
    }

    pub fn sample_points(&self, max_angle_step: f64) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        let mut first = true;
        for (start, end) in self.segments() {
            if first {
                points.push([start.x, start.y]);
                first = false;
            }
            append_segment_samples(&mut points, start, end, max_angle_step);
        }
        points
    }

    pub fn to_curve_string(&self) -> Result<CurveString2<Backend>, String> {
        if self.vertex_data.len() < 2 {
            return Err("a curve string needs at least two vertices".into());
        }
        CurveString2::from_bulge_vertices(&self.hyper_vertices()[..]).map_err(|e| e.to_string())
    }

    pub fn to_contour(&self) -> Result<HContour, String> {
        if !self.is_closed {
            return Err("polyline must be closed".into());
        }
        if self.vertex_data.len() < 2 {
            return Err("a closed contour needs at least two vertices".into());
        }
        Contour2::from_bulge_vertices_with_fill_rule(&self.hyper_vertices()[..], FillRule::NonZero)
            .map_err(|e| e.to_string())
    }

    pub fn offset_checked(&self, distance: f64) -> Result<Option<Self>, String> {
        let contour = self.to_contour()?;
        match contour
            .offset_left_checked(scalar(distance), &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    pub fn raw_offset(&self, distance: f64) -> Result<Option<Self>, String> {
        if self.is_closed {
            let contour = self.to_contour()?;
            match contour
                .offset_left_with_line_joins(scalar(distance), &policy())
                .map_err(|e| e.to_string())?
            {
                Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
                Classification::Uncertain(_) => Ok(None),
            }
        } else {
            let curve = self.to_curve_string()?;
            match curve
                .offset_left_with_line_joins(scalar(distance), &policy())
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
        match curve
            .offset_outline(scalar(distance), cap, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(contour) => Ok(Some(Self::from_contour(&contour))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    pub fn raw_offset_segments(&self, distance: f64) -> Result<Vec<Self>, String> {
        let segments = if self.is_closed {
            self.to_contour()?.segments().to_vec()
        } else {
            self.to_curve_string()?.segments().to_vec()
        };
        let mut out = Vec::new();
        for segment in segments {
            match segment
                .offset_left(scalar(distance), &policy())
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
            vertex_data: vertices,
            is_closed: closed,
        }
    }

    fn hyper_vertices(&self) -> Vec<BulgeVertex2<Backend>> {
        self.vertex_data
            .iter()
            .map(|vertex| {
                BulgeVertex2::new(
                    Point2::new(scalar(vertex.x), scalar(vertex.y)),
                    scalar(vertex.bulge),
                )
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
#[derive(Clone, Debug, Default)]
pub struct Shape {
    pub materials: Vec<Polyline>,
    pub holes: Vec<Polyline>,
}

impl Shape {
    pub fn from_materials(materials: Vec<Polyline>) -> Self {
        Self {
            materials,
            holes: Vec::new(),
        }
    }

    pub fn from_region(region: &Region2<Backend>) -> Self {
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

    pub fn to_region(&self) -> Result<Region2<Backend>, String> {
        Ok(Region2::new(
            self.materials
                .iter()
                .map(Polyline::to_contour)
                .collect::<Result<Vec<_>, _>>()?,
            self.holes
                .iter()
                .map(Polyline::to_contour)
                .collect::<Result<Vec<_>, _>>()?,
        ))
    }

    pub fn boolean(&self, other: &Self, op: BooleanMode) -> Result<Option<Self>, String> {
        let first = self.to_region()?;
        let second = other.to_region()?;
        let op = match op {
            BooleanMode::Union => HBooleanOp::Union,
            BooleanMode::Intersection => HBooleanOp::Intersection,
            BooleanMode::Difference => HBooleanOp::Difference,
            BooleanMode::Xor => HBooleanOp::Xor,
        };
        match first
            .boolean_region(&second, op, FillRule::NonZero, &policy())
            .map_err(|e| e.to_string())?
        {
            Classification::Decided(region) => Ok(Some(Self::from_region(&region))),
            Classification::Uncertain(_) => Ok(None),
        }
    }

    pub fn offset_once(&self, distance: f64) -> Self {
        let mut materials = Vec::new();
        for pline in &self.materials {
            if let Ok(Some(offset)) = pline.offset_checked(distance) {
                materials.push(offset);
            }
        }
        let mut holes = Vec::new();
        for pline in &self.holes {
            if let Ok(Some(offset)) = pline.offset_checked(-distance) {
                holes.push(offset);
            }
        }
        Self { materials, holes }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BooleanMode {
    Union,
    Intersection,
    Difference,
    Xor,
}

pub fn policy() -> CurvePolicy {
    CurvePolicy::approximate(Tolerance::new(1e-7, 1e-7))
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
    let first_fragments = match first_contour
        .split_at_intersections(&events, hypercurve::ContourOperand::First, &policy())
        .map_err(|e| e.to_string())?
    {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(_) => return Ok((Vec::new(), Vec::new())),
    };
    let second_fragments = match second_contour
        .split_at_intersections(&events, hypercurve::ContourOperand::Second, &policy())
        .map_err(|e| e.to_string())?
    {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(_) => return Ok((Vec::new(), Vec::new())),
    };
    Ok((
        first_fragments
            .fragments()
            .iter()
            .map(|fragment| Polyline::from_segments(&[fragment.segment.clone()], false))
            .collect(),
        second_fragments
            .fragments()
            .iter()
            .map(|fragment| Polyline::from_segments(&[fragment.segment.clone()], false))
            .collect(),
    ))
}

pub fn scalar(value: f64) -> HScalar {
    HScalar::try_from(value).unwrap_or_else(|_| HScalar::from(0_i8))
}

fn hpoint_array(point: &HPoint) -> [f64; 2] {
    let (x, y) = hpoint_xy(point);
    [x, y]
}

fn hpoint_xy(point: &HPoint) -> (f64, f64) {
    (scalar_to_f64(point.x()), scalar_to_f64(point.y()))
}

fn scalar_to_f64(value: &HScalar) -> f64 {
    value
        .to_f64_approx()
        .unwrap_or_else(|| f64::from(value.clone()))
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

fn bulge_for_arc(arc: &hypercurve::CircularArc2<Backend>) -> f64 {
    if let Some(bulge) = arc.bulge() {
        return scalar_to_f64(bulge);
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
