//! KiCad custom-pad primitive parsing.
//!
//! Custom pads can carry independent graphical primitives rather than one of
//! KiCad's simple built-in pad shapes. This module converts the additive
//! primitive geometry into conservative polygons for readiness checks; it does
//! not attempt lossless copper boolean reconstruction.

use geo::Polygon;

use crate::geometry::{arc_line_polygons, bezier_line_polygons, line_polygon, transform_polygon};
use crate::sexp::Sexp;

use super::graphic_primitives::{
    circle_polygons as circle_graphic_polygons, fill_enabled,
    polygon_polygons as polygon_graphic_polygons, rect_polygons as rect_graphic_polygons,
};
use super::{
    arcs::arc_center_start_angle_source, points_from_pts, rotate_translate, stroke_width,
    text::text_bbox_polygon, xy_from_child, xy_from_child_source,
};

pub(super) fn custom_pad_polygons(
    pad: &Sexp,
    location: [f64; 2],
    angle_degrees: f64,
) -> Vec<Polygon<f64>> {
    let mut polygons = Vec::new();
    let Some(primitives) = pad.named_child("primitives") else {
        return polygons;
    };

    for primitive in primitives.children().iter().skip(1) {
        match primitive.list_name() {
            Some("gr_rect") => {
                let Some(start) = xy_from_child(primitive, "start") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let filled = fill_enabled(primitive, true);
                let width = stroke_width(primitive, 0.01);
                let rect_polygons = rect_graphic_polygons(start, end, width, filled);
                if !filled {
                    log::trace!(
                        "parsed KiCad custom-pad unfilled rectangle primitive: location=({:.3},{:.3}) segments={}",
                        location[0],
                        location[1],
                        rect_polygons.len()
                    );
                }
                polygons.extend(
                    rect_polygons
                        .into_iter()
                        .map(|polygon| transform_polygon(&polygon, location, angle_degrees)),
                );
            }
            Some("gr_circle") => {
                let Some(center) = xy_from_child(primitive, "center") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let filled = fill_enabled(primitive, true);
                let width = stroke_width(primitive, 0.01);
                let circle_polygons = circle_graphic_polygons(center, end, width, filled, 48);
                if !filled {
                    log::trace!(
                        "parsed KiCad custom-pad unfilled circle primitive: location=({:.3},{:.3}) segments={}",
                        location[0],
                        location[1],
                        circle_polygons.len()
                    );
                }
                polygons.extend(
                    circle_polygons
                        .into_iter()
                        .map(|polygon| transform_polygon(&polygon, location, angle_degrees)),
                );
            }
            Some("gr_line") => {
                let Some(start) = xy_from_child(primitive, "start") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let width = stroke_width(primitive, 0.01);
                let start = rotate_translate(start, location, angle_degrees);
                let end = rotate_translate(end, location, angle_degrees);
                if let Some(polygon) = line_polygon(start, end, width) {
                    polygons.push(polygon);
                }
            }
            Some("gr_arc") => {
                let Some(start) = xy_from_child_source(primitive, "start") else {
                    continue;
                };
                let Some(mid) = xy_from_child_source(primitive, "mid") else {
                    continue;
                };
                let Some(end) = xy_from_child_source(primitive, "end") else {
                    continue;
                };
                let width = stroke_width(primitive, 0.01);
                // Custom-pad arc polygons remain a compatibility approximation
                // for current `geo`/`csgrs` consumers, but the arc degeneracy
                // predicate now consumes retained decimal-token `Real`
                // coordinates before sampling. This mirrors Yap's exact
                // geometric computation split between certified decisions and
                // approximation edges. See Yap, "Towards Exact Geometric
                // Computation," *Computational Geometry* 7.1-2 (1997).
                let Some((arc_center, arc_start, angle)) =
                    arc_center_start_angle_source(&start, &mid, &end)
                else {
                    continue;
                };
                let arc_polygons = arc_line_polygons(arc_center, arc_start, angle, width, 16);
                log::trace!(
                    "parsed KiCad custom-pad arc primitive: location=({:.3},{:.3}) segments={}",
                    location[0],
                    location[1],
                    arc_polygons.len()
                );
                polygons.extend(
                    arc_polygons
                        .into_iter()
                        .map(|polygon| transform_polygon(&polygon, location, angle_degrees)),
                );
            }
            Some("gr_poly") => {
                let points = points_from_pts(primitive);
                let filled = fill_enabled(primitive, true);
                let width = stroke_width(primitive, 0.01);
                let poly_polygons = polygon_graphic_polygons(&points, width, filled);
                if !filled {
                    log::trace!(
                        "parsed KiCad custom-pad unfilled polygon primitive: location=({:.3},{:.3}) points={} segments={}",
                        location[0],
                        location[1],
                        points.len(),
                        poly_polygons.len()
                    );
                }
                polygons.extend(
                    poly_polygons
                        .into_iter()
                        .map(|polygon| transform_polygon(&polygon, location, angle_degrees)),
                );
            }
            Some("bezier" | "fp_curve" | "gr_curve") => {
                let points = points_from_pts(primitive);
                let width = stroke_width(primitive, 0.01);
                let curve_polygons = bezier_line_polygons(&points, width, 16);
                log::trace!(
                    "parsed KiCad custom-pad Bezier primitive: location=({:.3},{:.3}) control_points={} segments={}",
                    location[0],
                    location[1],
                    points.len(),
                    curve_polygons.len()
                );
                polygons.extend(
                    curve_polygons
                        .into_iter()
                        .map(|polygon| transform_polygon(&polygon, location, angle_degrees)),
                );
            }
            Some("gr_text" | "fp_text") => {
                if let Some(polygon) = text_bbox_polygon(primitive, location, angle_degrees) {
                    polygons.push(polygon);
                }
            }
            _ => {}
        }
    }

    polygons
}
