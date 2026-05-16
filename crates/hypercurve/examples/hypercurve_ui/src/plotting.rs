use std::ops::RangeInclusive;
use std::sync::Arc;

use egui::{Color32, Shape as EguiShape};
use egui_plot::{
    PlotBounds, PlotGeometry, PlotItem, PlotItemBase, PlotPoint, PlotPoints, PlotTransform, PlotUi,
    Points,
};
use lyon::path::Path;
use lyon::tessellation::{
    BuffersBuilder, FillOptions, FillTessellator, StrokeOptions, StrokeTessellator, VertexBuffers,
};

use crate::geometry::{Polyline, Shape};

pub const PLOT_VERTEX_RADIUS: f32 = 4.0;
const SAMPLE_ANGLE_STEP: f64 = 0.04;

pub fn draw_polyline(
    plot_ui: &mut PlotUi<'_>,
    name: impl Into<String>,
    polyline: &Polyline,
    stroke: Color32,
    fill: Option<Color32>,
    vertex_color: Option<Color32>,
) {
    plot_ui.add(
        PolylinePlotItem::new(name, polyline)
            .stroke_color(stroke)
            .fill_color(fill.unwrap_or(Color32::TRANSPARENT))
            .vertex_color(vertex_color.unwrap_or(Color32::TRANSPARENT)),
    );
}

pub fn draw_shape(
    plot_ui: &mut PlotUi<'_>,
    name: &str,
    shape: &Shape,
    stroke: Color32,
    fill: Option<Color32>,
    vertex_color: Option<Color32>,
) {
    plot_ui.add(
        ShapePlotItem::new(name, shape)
            .stroke_color(stroke)
            .fill_color(fill.unwrap_or(Color32::TRANSPARENT))
            .vertex_color(vertex_color.unwrap_or(Color32::TRANSPARENT)),
    );
}

pub fn draw_points(plot_ui: &mut PlotUi<'_>, name: &str, points: &[[f64; 2]], color: Color32) {
    if points.is_empty() {
        return;
    }
    plot_ui.points(
        Points::new(name, PlotPoints::from(points.to_vec()))
            .radius(PLOT_VERTEX_RADIUS * 1.5)
            .color(color),
    );
}

pub fn find_near_vertex(
    plot_ui: &PlotUi<'_>,
    coord: egui::Pos2,
    polylines: &[Polyline],
) -> Option<(usize, usize)> {
    for (pline_index, polyline) in polylines.iter().enumerate() {
        for (vertex_index, vertex) in polyline.iter_vertexes().enumerate() {
            let screen = plot_ui.screen_from_plot(PlotPoint::new(vertex.x, vertex.y));
            let hit_size = 2.0 * (plot_ui.ctx().input(|i| i.aim_radius()) + PLOT_VERTEX_RADIUS);
            let hit_box = egui::Rect::from_center_size(screen, egui::Vec2::splat(hit_size));
            if hit_box.contains(coord) {
                return Some((pline_index, vertex_index));
            }
        }
    }
    None
}

#[derive(Clone, Debug)]
struct DrawPolyline {
    points: Vec<[f64; 2]>,
    vertices: Vec<[f64; 2]>,
    closed: bool,
}

impl DrawPolyline {
    fn from_polyline(polyline: &Polyline) -> Self {
        let mut points = polyline.sample_points(SAMPLE_ANGLE_STEP);
        points.dedup_by(|a, b| points_equal(*a, *b));
        Self {
            points,
            vertices: polyline.iter_vertexes().map(|v| [v.x, v.y]).collect(),
            closed: polyline.is_closed(),
        }
    }
}

struct PolylinePlotItem {
    name: String,
    polylines: Vec<DrawPolyline>,
    stroke_color: Color32,
    fill_color: Color32,
    vertex_color: Color32,
    base: PlotItemBase,
}

impl PolylinePlotItem {
    fn new(name: impl Into<String>, polyline: &Polyline) -> Self {
        let name = name.into();
        Self {
            base: PlotItemBase::new(name.clone()),
            name,
            polylines: vec![DrawPolyline::from_polyline(polyline)],
            stroke_color: Color32::TRANSPARENT,
            fill_color: Color32::TRANSPARENT,
            vertex_color: Color32::TRANSPARENT,
        }
    }

    fn stroke_color(mut self, color: Color32) -> Self {
        self.stroke_color = color;
        self
    }

    fn fill_color(mut self, color: Color32) -> Self {
        self.fill_color = color;
        self
    }

    fn vertex_color(mut self, color: Color32) -> Self {
        self.vertex_color = color;
        self
    }
}

impl PlotItem for PolylinePlotItem {
    fn shapes(&self, _ui: &egui::Ui, transform: &PlotTransform, shapes: &mut Vec<EguiShape>) {
        let path = polylines_path(&self.polylines, transform);
        push_path_shapes(&path, self.stroke_color, self.fill_color, shapes);
        push_polyline_vertices(&self.polylines, self.vertex_color, transform, shapes);
    }

    fn initialize(&mut self, _x_range: RangeInclusive<f64>) {}

    fn name(&self) -> &str {
        &self.name
    }

    fn color(&self) -> Color32 {
        if self.fill_color == Color32::TRANSPARENT {
            self.stroke_color
        } else {
            self.fill_color
        }
    }

    fn highlight(&mut self) {}

    fn highlighted(&self) -> bool {
        false
    }

    fn allow_hover(&self) -> bool {
        false
    }

    fn geometry(&self) -> PlotGeometry<'_> {
        PlotGeometry::None
    }

    fn bounds(&self) -> PlotBounds {
        draw_polylines_bounds(&self.polylines)
    }

    fn base(&self) -> &PlotItemBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PlotItemBase {
        &mut self.base
    }
}

struct ShapePlotItem {
    name: String,
    polylines: Vec<DrawPolyline>,
    stroke_color: Color32,
    fill_color: Color32,
    vertex_color: Color32,
    base: PlotItemBase,
}

impl ShapePlotItem {
    fn new(name: impl Into<String>, shape: &Shape) -> Self {
        let name = name.into();
        Self {
            base: PlotItemBase::new(name.clone()),
            name,
            polylines: shape
                .materials
                .iter()
                .chain(shape.holes.iter())
                .map(DrawPolyline::from_polyline)
                .collect(),
            stroke_color: Color32::TRANSPARENT,
            fill_color: Color32::TRANSPARENT,
            vertex_color: Color32::TRANSPARENT,
        }
    }

    fn stroke_color(mut self, color: Color32) -> Self {
        self.stroke_color = color;
        self
    }

    fn fill_color(mut self, color: Color32) -> Self {
        self.fill_color = color;
        self
    }

    fn vertex_color(mut self, color: Color32) -> Self {
        self.vertex_color = color;
        self
    }
}

impl PlotItem for ShapePlotItem {
    fn shapes(&self, _ui: &egui::Ui, transform: &PlotTransform, shapes: &mut Vec<EguiShape>) {
        let path = polylines_path(&self.polylines, transform);
        push_path_shapes(&path, self.stroke_color, self.fill_color, shapes);
        push_polyline_vertices(&self.polylines, self.vertex_color, transform, shapes);
    }

    fn initialize(&mut self, _x_range: RangeInclusive<f64>) {}

    fn name(&self) -> &str {
        &self.name
    }

    fn color(&self) -> Color32 {
        if self.fill_color == Color32::TRANSPARENT {
            self.stroke_color
        } else {
            self.fill_color
        }
    }

    fn highlight(&mut self) {}

    fn highlighted(&self) -> bool {
        false
    }

    fn allow_hover(&self) -> bool {
        false
    }

    fn geometry(&self) -> PlotGeometry<'_> {
        PlotGeometry::None
    }

    fn bounds(&self) -> PlotBounds {
        draw_polylines_bounds(&self.polylines)
    }

    fn base(&self) -> &PlotItemBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut PlotItemBase {
        &mut self.base
    }
}

fn polylines_path(polylines: &[DrawPolyline], transform: &PlotTransform) -> Path {
    let mut builder = Path::builder();
    for polyline in polylines {
        append_polyline_path(polyline, transform, &mut builder);
    }
    builder.build()
}

fn append_polyline_path(
    polyline: &DrawPolyline,
    transform: &PlotTransform,
    builder: &mut lyon::path::Builder,
) {
    if polyline.points.len() < 2 {
        return;
    }

    let Some(first) = polyline.points.first() else {
        return;
    };

    builder.begin(lyon_point(*first, transform));
    for point in polyline.points.iter().skip(1) {
        builder.line_to(lyon_point(*point, transform));
    }
    if polyline.closed {
        builder.close();
    } else {
        builder.end(false);
    }
}

fn push_path_shapes(
    path: &Path,
    stroke_color: Color32,
    fill_color: Color32,
    shapes: &mut Vec<EguiShape>,
) {
    if fill_color != Color32::TRANSPARENT {
        let mut buffers: VertexBuffers<egui::epaint::Vertex, u32> = VertexBuffers::new();
        let mut tessellator = FillTessellator::new();
        if tessellator
            .tessellate_path(
                path.as_slice(),
                &FillOptions::DEFAULT,
                &mut BuffersBuilder::new(&mut buffers, VertexConstructor { color: fill_color }),
            )
            .is_ok()
        {
            shapes.push(EguiShape::mesh(Arc::new(egui::epaint::Mesh {
                vertices: buffers.vertices,
                indices: buffers.indices,
                texture_id: Default::default(),
            })));
        }
    }

    if stroke_color != Color32::TRANSPARENT {
        let mut buffers: VertexBuffers<egui::epaint::Vertex, u32> = VertexBuffers::new();
        let mut tessellator = StrokeTessellator::new();
        if tessellator
            .tessellate_path(
                path.as_slice(),
                &StrokeOptions::DEFAULT.with_line_width(1.4),
                &mut BuffersBuilder::new(
                    &mut buffers,
                    VertexConstructor {
                        color: stroke_color,
                    },
                ),
            )
            .is_ok()
        {
            shapes.push(EguiShape::mesh(Arc::new(egui::epaint::Mesh {
                vertices: buffers.vertices,
                indices: buffers.indices,
                texture_id: Default::default(),
            })));
        }
    }
}

fn push_polyline_vertices(
    polylines: &[DrawPolyline],
    vertex_color: Color32,
    transform: &PlotTransform,
    shapes: &mut Vec<EguiShape>,
) {
    if vertex_color == Color32::TRANSPARENT {
        return;
    }

    for polyline in polylines {
        for vertex in &polyline.vertices {
            shapes.push(EguiShape::circle_filled(
                transform.position_from_point(&PlotPoint::new(vertex[0], vertex[1])),
                PLOT_VERTEX_RADIUS,
                vertex_color,
            ));
        }
    }
}

fn lyon_point(point: [f64; 2], transform: &PlotTransform) -> lyon::math::Point {
    let position = transform.position_from_point(&PlotPoint::new(point[0], point[1]));
    lyon::math::point(position.x, position.y)
}

fn points_equal(first: [f64; 2], second: [f64; 2]) -> bool {
    (first[0] - second[0]).abs() <= 1e-9 && (first[1] - second[1]).abs() <= 1e-9
}

struct VertexConstructor {
    color: Color32,
}

impl lyon::tessellation::FillVertexConstructor<egui::epaint::Vertex> for VertexConstructor {
    fn new_vertex(&mut self, vertex: lyon::tessellation::FillVertex) -> egui::epaint::Vertex {
        egui::epaint::Vertex {
            pos: egui::pos2(vertex.position().x, vertex.position().y),
            uv: Default::default(),
            color: self.color,
        }
    }
}

impl lyon::tessellation::StrokeVertexConstructor<egui::epaint::Vertex> for VertexConstructor {
    fn new_vertex(&mut self, vertex: lyon::tessellation::StrokeVertex) -> egui::epaint::Vertex {
        egui::epaint::Vertex {
            pos: egui::pos2(vertex.position().x, vertex.position().y),
            uv: Default::default(),
            color: self.color,
        }
    }
}

fn draw_polylines_bounds(polylines: &[DrawPolyline]) -> PlotBounds {
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    for polyline in polylines {
        for point in &polyline.points {
            min[0] = min[0].min(point[0]);
            min[1] = min[1].min(point[1]);
            max[0] = max[0].max(point[0]);
            max[1] = max[1].max(point[1]);
        }
    }

    if min[0].is_finite() && min[1].is_finite() && max[0].is_finite() && max[1].is_finite() {
        PlotBounds::from_min_max(min, max)
    } else {
        PlotBounds::NOTHING
    }
}
