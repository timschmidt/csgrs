use egui::{Color32, Stroke};
use egui_plot::{Line, PlotPoint, PlotPoints, PlotUi, Points, Polygon};

use crate::geometry::{Polyline, Shape};

pub const PLOT_VERTEX_RADIUS: f32 = 4.0;

pub fn draw_polyline(
    plot_ui: &mut PlotUi<'_>,
    name: impl Into<String>,
    polyline: &Polyline,
    stroke: Color32,
    fill: Option<Color32>,
    vertex_color: Option<Color32>,
) {
    let sampled = polyline.sample_points(0.08);
    if sampled.len() < 2 {
        draw_vertices(plot_ui, polyline, vertex_color);
        return;
    }

    if polyline.is_closed()
        && let Some(fill) = fill
    {
        plot_ui.polygon(
            Polygon::new(
                format!("{} fill", name.into()),
                PlotPoints::from(sampled.clone()),
            )
            .fill_color(fill)
            .stroke(Stroke::new(1.0, stroke)),
        );
    } else {
        plot_ui.line(
            Line::new(name.into(), PlotPoints::from(sampled.clone()))
                .color(stroke)
                .width(1.4),
        );
    }

    if polyline.is_closed() && fill.is_some() {
        plot_ui.line(
            Line::new("outline", PlotPoints::from(sampled))
                .color(stroke)
                .width(1.4),
        );
    }

    draw_vertices(plot_ui, polyline, vertex_color);
}

pub fn draw_shape(
    plot_ui: &mut PlotUi<'_>,
    name: &str,
    shape: &Shape,
    stroke: Color32,
    fill: Option<Color32>,
    vertex_color: Option<Color32>,
) {
    for (index, polyline) in shape.materials.iter().enumerate() {
        draw_polyline(
            plot_ui,
            format!("{name} material {index}"),
            polyline,
            stroke,
            fill,
            vertex_color,
        );
    }
    for (index, polyline) in shape.holes.iter().enumerate() {
        draw_polyline(
            plot_ui,
            format!("{name} hole {index}"),
            polyline,
            stroke.gamma_multiply(0.85),
            None,
            vertex_color,
        );
    }
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

fn draw_vertices(plot_ui: &mut PlotUi<'_>, polyline: &Polyline, vertex_color: Option<Color32>) {
    let Some(vertex_color) = vertex_color else {
        return;
    };
    let points: Vec<_> = polyline.iter_vertexes().map(|v| [v.x, v.y]).collect();
    draw_points(plot_ui, "vertices", &points, vertex_color);
}
