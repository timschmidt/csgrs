//! KiCad board-graphics parsing.
//!
//! Board graphics are used for physical outlines and manufacturing features,
//! not electrical copper. Keeping them separate makes the main loader easier to
//! scan and keeps arc/outline reconstruction in one place.

use geo::Polygon;
use hyperlimit::{Point2, PredicatePolicy, point2_equal_with_policy};

use crate::geometry::{
    RuleGeometryProvenance, SourceGridFacts, SourceUnit, arc_line_polygons, bezier_line_polygons,
    circle_polygon, line_polygon, polygon_from_points,
};
use crate::sexp::Sexp;

use super::{
    arcs::arc_center_start_angle, points_from_pts, polygons_from_pts, stroke_width, xy_from_child,
};

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
        let width = stroke_width(line, 0.05);
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

    for poly in root.named_children("gr_poly") {
        let polygons = polygons_from_pts(poly);
        if polygons.is_empty() {
            continue;
        }
        if is_edge_cuts(poly) {
            log::trace!(
                "parsed KiCad Edge.Cuts polygon graphics: count={}",
                polygons.len()
            );
            edge_polygons.extend(polygons);
        } else if is_panel_layer(poly) {
            log::trace!(
                "parsed KiCad panel polygon graphics: count={}",
                polygons.len()
            );
            panel_polygons.extend(polygons);
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
        let width = stroke_width(arc, 0.05).max(0.01);
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

    for curve_name in ["bezier", "gr_curve"] {
        for curve in root.named_children(curve_name) {
            let points = points_from_pts(curve);
            let width = stroke_width(curve, 0.05).max(0.01);
            let polygons = bezier_line_polygons(&points, width, 24);
            if polygons.is_empty() {
                continue;
            }
            if is_edge_cuts(curve) {
                log::trace!(
                    "parsed KiCad Edge.Cuts Bezier graphics: control_points={} segments={}",
                    points.len(),
                    polygons.len()
                );
                edge_polygons.extend(polygons);
            } else if is_panel_layer(curve) {
                log::trace!(
                    "parsed KiCad panel Bezier graphics: control_points={} segments={}",
                    points.len(),
                    polygons.len()
                );
                panel_polygons.extend(polygons);
            }
        }
    }
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
    // Outline stitching changes imported topology before board-level checks
    // run, so endpoint equality must be exact after lifting finite parser
    // coordinates. KiCad coordinates arrive through an f64 parser boundary, but
    // every finite IEEE-754 value has an exact dyadic `Real` representation.
    // Route equality through `hyperlimit` rather than a local tolerance, in the
    // exact-geometric-computation style of Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7.1-2 (1997).
    //
    let provenance = RuleGeometryProvenance::new(
        "kicad-edge-stitching",
        SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter),
    );
    let Some(left) = lift_point(left, provenance) else {
        return false;
    };
    let Some(right) = lift_point(right, provenance) else {
        return false;
    };
    point2_equal_with_policy(&left, &right, PredicatePolicy::STRICT)
        .value()
        .unwrap_or(false)
}

fn lift_point(point: [f64; 2], provenance: RuleGeometryProvenance) -> Option<Point2> {
    Some(Point2::new(
        provenance.lift_f64(point[0])?,
        provenance.lift_f64(point[1])?,
    ))
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
