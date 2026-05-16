use std::f64::consts::PI;
use std::ops::{Index, IndexMut};

use geo::{BooleanOps, Buffer, Coord, LineString, MultiPolygon, Polygon};
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
const DISPLAY_COORD_EPS: f64 = 2e-5;
const MIN_DISPLAY_LOOP_AREA: f64 = 1e-6;

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

    pub fn signed_area_estimate(&self) -> f64 {
        if !self.is_closed || self.vertex_data.len() < 2 {
            return 0.0;
        }

        let points = self.sample_points(0.04);
        signed_area_of_points(&points)
    }

    pub fn is_counter_clockwise(&self) -> bool {
        self.signed_area_estimate() >= 0.0
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

    #[cfg(test)]
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

    #[cfg(test)]
    pub fn offset_for_display(&self, distance: f64) -> Result<Option<Self>, String> {
        Ok(self.offsets_for_display(distance)?.into_iter().next())
    }

    pub fn offsets_for_display(&self, distance: f64) -> Result<Vec<Self>, String> {
        if self.is_closed
            && let Some(polygon) = polyline_to_geo_polygon(self)
        {
            let buffered = polygon.buffer(left_offset_buffer_distance(self, distance));
            return Ok(shape_from_geo(&buffered).into_polylines());
        }

        Ok(self.raw_offset(distance)?.into_iter().collect())
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

    pub fn from_polylines(polylines: Vec<Polyline>) -> Self {
        let mut materials = Vec::new();
        let mut holes = Vec::new();
        for polyline in polylines {
            if polyline.vertex_data.len() < 2 {
                continue;
            }
            if polyline.is_counter_clockwise() {
                materials.push(polyline);
            } else {
                holes.push(polyline);
            }
        }
        Self { materials, holes }
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
) -> Result<Option<Region2<Backend>>, String> {
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
        vertex_data: coords
            .into_iter()
            .map(|coord| Vertex::new(coord.x, coord.y, 0.0))
            .collect(),
        is_closed: true,
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
        let vertices: Vec<_> = (0..vertex_count)
            .map(|index| {
                let angle = index as f64 * std::f64::consts::TAU / vertex_count as f64;
                let radius = 12.0 * radius_scale[index];
                let bulge = if index % 3 == 0 {
                    0.0
                } else {
                    bulge_values[index]
                };
                (radius * angle.cos(), radius * angle.sin(), bulge)
            })
            .collect();
        Polyline::closed(&vertices)
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
