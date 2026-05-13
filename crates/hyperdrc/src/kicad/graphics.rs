//! KiCad board-graphics parsing.
//!
//! Board graphics are used for physical outlines and manufacturing features,
//! not electrical copper. Keeping them separate makes the main loader easier to
//! scan and keeps arc/outline reconstruction in one place.

use geo::Polygon;

use crate::geometry::{arc_line_polygons, circle_polygon, line_polygon, polygon_from_points};
use crate::sexp::Sexp;

use super::xy_from_child;

pub(super) fn parse_graphics(
    root: &Sexp,
    edge_polygons: &mut Vec<Polygon<f64>>,
    panel_polygons: &mut Vec<Polygon<f64>>,
) {
    let mut edge_lines = Vec::new();

    for line in root.named_children("gr_line") {
        let Some(start) = xy_from_child(line, "start") else {
            continue;
        };
        let Some(end) = xy_from_child(line, "end") else {
            continue;
        };
        let width = line
            .named_child("width")
            .and_then(|width| width.f64_at(1))
            .unwrap_or(0.05);
        if is_edge_cuts(line) {
            edge_lines.push((start, end));
        } else if is_panel_layer(line)
            && let Some(polygon) = line_polygon(start, end, width.max(0.01))
        {
            panel_polygons.push(polygon);
        }
    }

    if let Some(outline) = closed_polygon_from_lines(&edge_lines) {
        edge_polygons.push(outline);
    } else {
        for (start, end) in edge_lines {
            if let Some(polygon) = line_polygon(start, end, 0.05) {
                edge_polygons.push(polygon);
            }
        }
    }

    for rect in root.named_children("gr_rect") {
        let Some(start) = xy_from_child(rect, "start") else {
            continue;
        };
        let Some(end) = xy_from_child(rect, "end") else {
            continue;
        };
        let polygon = polygon_from_points(vec![start, [end[0], start[1]], end, [start[0], end[1]]]);
        if is_edge_cuts(rect) {
            edge_polygons.push(polygon);
        } else if is_panel_layer(rect) {
            panel_polygons.push(polygon);
        }
    }

    for circle in root.named_children("gr_circle") {
        let Some(center) = xy_from_child(circle, "center") else {
            continue;
        };
        let Some(end) = xy_from_child(circle, "end") else {
            continue;
        };
        let radius = distance(center, end);
        let polygon = circle_polygon(center, radius, 64);
        if is_edge_cuts(circle) {
            edge_polygons.push(polygon);
        } else if is_panel_layer(circle) {
            panel_polygons.push(polygon);
        }
    }

    for arc in root.named_children("gr_arc") {
        let Some(center) = xy_from_child(arc, "start").or_else(|| xy_from_child(arc, "center"))
        else {
            continue;
        };
        let Some(mid) = xy_from_child(arc, "mid") else {
            continue;
        };
        let Some(end) = xy_from_child(arc, "end") else {
            continue;
        };
        let width = arc
            .named_child("width")
            .and_then(|width| width.f64_at(1))
            .unwrap_or(0.05)
            .max(0.01);
        let Some((arc_center, start, angle)) = arc_center_start_angle(center, mid, end) else {
            continue;
        };
        let polygons = arc_line_polygons(arc_center, start, angle, width, 24);
        if is_edge_cuts(arc) {
            edge_polygons.extend(polygons);
        } else if is_panel_layer(arc) {
            panel_polygons.extend(polygons);
        }
    }
}

fn arc_center_start_angle(
    start: [f64; 2],
    mid: [f64; 2],
    end: [f64; 2],
) -> Option<([f64; 2], [f64; 2], f64)> {
    // Circumcircle through start/mid/end. The midpoint determines whether the
    // represented arc follows the counter-clockwise or clockwise sweep.
    let d = 2.0
        * (start[0] * (mid[1] - end[1])
            + mid[0] * (end[1] - start[1])
            + end[0] * (start[1] - mid[1]));
    if d.abs() < 1.0e-9 {
        return None;
    }

    let start_sq = start[0] * start[0] + start[1] * start[1];
    let mid_sq = mid[0] * mid[0] + mid[1] * mid[1];
    let end_sq = end[0] * end[0] + end[1] * end[1];
    let center = [
        (start_sq * (mid[1] - end[1])
            + mid_sq * (end[1] - start[1])
            + end_sq * (start[1] - mid[1]))
            / d,
        (start_sq * (end[0] - mid[0])
            + mid_sq * (start[0] - end[0])
            + end_sq * (mid[0] - start[0]))
            / d,
    ];
    let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
    let mid_angle = (mid[1] - center[1]).atan2(mid[0] - center[0]);
    let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
    let ccw_delta = positive_angle_delta(start_angle, end_angle);
    let mid_delta = positive_angle_delta(start_angle, mid_angle);
    let angle = if mid_delta <= ccw_delta {
        ccw_delta.to_degrees()
    } else {
        -(std::f64::consts::TAU - ccw_delta).to_degrees()
    };

    Some((center, start, angle))
}

fn positive_angle_delta(start: f64, end: f64) -> f64 {
    let mut delta = end - start;
    while delta < 0.0 {
        delta += std::f64::consts::TAU;
    }
    while delta >= std::f64::consts::TAU {
        delta -= std::f64::consts::TAU;
    }
    delta
}

fn closed_polygon_from_lines(lines: &[([f64; 2], [f64; 2])]) -> Option<Polygon<f64>> {
    let (first_start, first_end) = *lines.first()?;
    let mut remaining = lines[1..].to_vec();
    let mut points = vec![first_start, first_end];

    while !remaining.is_empty() {
        let current = *points.last()?;
        // KiCad Edge.Cuts commonly arrives as unordered line segments. We stitch
        // exact endpoint matches before falling back to stroked line geometry.
        let (index, next) = remaining
            .iter()
            .enumerate()
            .find_map(|(index, (start, end))| {
                if same_point(current, *start) {
                    Some((index, *end))
                } else if same_point(current, *end) {
                    Some((index, *start))
                } else {
                    None
                }
            })?;

        points.push(next);
        remaining.remove(index);
    }

    if points.len() >= 4 && same_point(points[0], *points.last()?) {
        Some(polygon_from_points(points))
    } else {
        None
    }
}

fn same_point(left: [f64; 2], right: [f64; 2]) -> bool {
    (left[0] - right[0]).abs() < 1.0e-6 && (left[1] - right[1]).abs() < 1.0e-6
}

fn is_edge_cuts(item: &Sexp) -> bool {
    item.named_child("layer")
        .and_then(|layer| layer.atom_at(1))
        .is_some_and(|layer| layer == "Edge.Cuts")
}

fn is_panel_layer(item: &Sexp) -> bool {
    item.named_child("layer")
        .and_then(|layer| layer.atom_at(1))
        .is_some_and(|layer| {
            layer.contains("Panel")
                || layer.contains("VScore")
                || layer.contains("V-Score")
                || layer.contains("TabRoute")
                || layer.contains("Tab.Route")
                || layer.contains("Castellated")
                || layer.contains("Castellation")
                || layer.contains("EdgePlating")
                || layer.contains("Edge.Plating")
        })
}

fn distance(start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    (dx * dx + dy * dy).sqrt()
}
