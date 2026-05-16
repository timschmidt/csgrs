//! KiCad footprint copper-graphics parsing.
//!
//! Footprint graphics can live on copper layers and are commonly used for RF
//! structures, odd shields, decorative copper, and manufacturer footprints that
//! are not expressible as ordinary pads. These graphics are parsed as unnetted
//! copper so existing clearance and manufacturability checks can see them.

use geo::Polygon;

use crate::LayerMetadata;
use crate::geometry::{
    arc_line_polygons, bezier_line_polygons, line_polygon, polygons_to_sketch, transform_polygon,
};
use crate::sexp::Sexp;

use super::graphic_primitives::{
    circle_polygons as circle_graphic_polygons, fill_enabled,
    polygon_polygons as polygon_graphic_polygons, rect_polygons as rect_graphic_polygons,
};
use super::{
    CopperFeature, CopperKind, arcs::arc_center_start_angle, expand_copper_layers, midpoint,
    points_from_pts, rotate_translate, stroke_width, text::text_bbox_polygon, xy_from_child,
};

pub(super) fn parse_footprint_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    let before = copper.len();
    parse_line_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_rect_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_circle_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_arc_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_poly_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_curve_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    parse_text_graphics(
        footprint,
        footprint_at,
        footprint_angle,
        declared_copper_layers,
        copper,
    );
    let added = copper.len() - before;
    if added > 0 {
        log::trace!(
            "parsed KiCad footprint copper graphics: added={} footprint_at=({:.3},{:.3}) angle={:.3}",
            added,
            footprint_at[0],
            footprint_at[1],
            footprint_angle
        );
    }
}

fn parse_line_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for line in footprint.named_children("fp_line") {
        let Some(start) = xy_from_child(line, "start") else {
            continue;
        };
        let Some(end) = xy_from_child(line, "end") else {
            continue;
        };
        let width = stroke_width(line, 0.01);
        let start = rotate_translate(start, footprint_at, footprint_angle);
        let end = rotate_translate(end, footprint_at, footprint_angle);
        let Some(polygon) = line_polygon(start, end, width) else {
            continue;
        };
        push_graphic_features(
            line,
            vec![polygon],
            CopperKind::Segment,
            midpoint(start, end),
            "KiCad footprint line",
            declared_copper_layers,
            copper,
        );
    }
}

fn parse_rect_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for rect in footprint.named_children("fp_rect") {
        let Some(start) = xy_from_child(rect, "start") else {
            continue;
        };
        let Some(end) = xy_from_child(rect, "end") else {
            continue;
        };
        let filled = fill_enabled(rect, true);
        let width = stroke_width(rect, 0.01);
        let polygons = rect_graphic_polygons(start, end, width, filled)
            .into_iter()
            .map(|polygon| transform_polygon(&polygon, footprint_at, footprint_angle))
            .collect::<Vec<_>>();
        if !filled {
            log::trace!(
                "parsed KiCad footprint unfilled rectangle graphic: segments={}",
                polygons.len()
            );
        }
        push_graphic_features(
            rect,
            polygons,
            if filled {
                CopperKind::Zone
            } else {
                CopperKind::Segment
            },
            rotate_translate(midpoint(start, end), footprint_at, footprint_angle),
            "KiCad footprint rectangle",
            declared_copper_layers,
            copper,
        );
    }
}

fn parse_circle_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for circle in footprint.named_children("fp_circle") {
        let Some(center) = xy_from_child(circle, "center") else {
            continue;
        };
        let Some(end) = xy_from_child(circle, "end") else {
            continue;
        };
        let filled = fill_enabled(circle, true);
        let width = stroke_width(circle, 0.01);
        let polygons = circle_graphic_polygons(center, end, width, filled, 48)
            .into_iter()
            .map(|polygon| transform_polygon(&polygon, footprint_at, footprint_angle))
            .collect::<Vec<_>>();
        if !filled {
            log::trace!(
                "parsed KiCad footprint unfilled circle graphic: segments={}",
                polygons.len()
            );
        }
        push_graphic_features(
            circle,
            polygons,
            if filled {
                CopperKind::Zone
            } else {
                CopperKind::Segment
            },
            rotate_translate(center, footprint_at, footprint_angle),
            "KiCad footprint circle",
            declared_copper_layers,
            copper,
        );
    }
}

fn parse_arc_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for arc in footprint.named_children("fp_arc") {
        let Some(start) = xy_from_child(arc, "start") else {
            continue;
        };
        let Some(mid) = xy_from_child(arc, "mid") else {
            continue;
        };
        let Some(end) = xy_from_child(arc, "end") else {
            continue;
        };
        let width = stroke_width(arc, 0.01);
        let Some((arc_center, arc_start, angle)) = arc_center_start_angle(start, mid, end) else {
            continue;
        };
        let polygons = arc_line_polygons(arc_center, arc_start, angle, width, 16)
            .into_iter()
            .map(|polygon| transform_polygon(&polygon, footprint_at, footprint_angle))
            .collect::<Vec<_>>();
        push_graphic_features(
            arc,
            polygons,
            CopperKind::Segment,
            rotate_translate(midpoint(start, end), footprint_at, footprint_angle),
            "KiCad footprint arc",
            declared_copper_layers,
            copper,
        );
    }
}

fn parse_poly_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for poly in footprint.named_children("fp_poly") {
        let points = points_from_pts(poly);
        let filled = fill_enabled(poly, true);
        let width = stroke_width(poly, 0.01);
        let polygons = polygon_graphic_polygons(&points, width, filled)
            .into_iter()
            .map(|polygon| transform_polygon(&polygon, footprint_at, footprint_angle))
            .collect::<Vec<_>>();
        if !filled {
            log::trace!(
                "parsed KiCad footprint unfilled polygon graphic: points={} segments={}",
                points.len(),
                polygons.len()
            );
        }
        push_graphic_features(
            poly,
            polygons,
            if filled {
                CopperKind::Zone
            } else {
                CopperKind::Segment
            },
            footprint_at,
            "KiCad footprint polygon",
            declared_copper_layers,
            copper,
        );
    }
}

fn parse_curve_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for curve_name in ["fp_curve", "bezier", "gr_curve"] {
        for curve in footprint.named_children(curve_name) {
            let points = points_from_pts(curve);
            let width = stroke_width(curve, 0.01);
            let polygons = bezier_line_polygons(&points, width, 16)
                .into_iter()
                .map(|polygon| transform_polygon(&polygon, footprint_at, footprint_angle))
                .collect::<Vec<_>>();
            if !polygons.is_empty() {
                log::trace!(
                    "parsed KiCad footprint Bezier graphic: primitive={curve_name} control_points={} segments={}",
                    points.len(),
                    polygons.len()
                );
            }
            push_graphic_features(
                curve,
                polygons,
                CopperKind::Segment,
                points
                    .first()
                    .map(|point| rotate_translate(*point, footprint_at, footprint_angle))
                    .unwrap_or(footprint_at),
                "KiCad footprint Bezier",
                declared_copper_layers,
                copper,
            );
        }
    }
}

fn parse_text_graphics(
    footprint: &Sexp,
    footprint_at: [f64; 2],
    footprint_angle: f64,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    for text in footprint.named_children("fp_text") {
        let Some(polygon) = text_bbox_polygon(text, footprint_at, footprint_angle) else {
            continue;
        };
        let location = xy_from_child(text, "at")
            .map(|at| rotate_translate(at, footprint_at, footprint_angle))
            .unwrap_or(footprint_at);
        push_graphic_features(
            text,
            vec![polygon],
            CopperKind::Zone,
            location,
            "KiCad footprint text",
            declared_copper_layers,
            copper,
        );
    }
}

fn push_graphic_features(
    item: &Sexp,
    polygons: Vec<Polygon<f64>>,
    kind: CopperKind,
    location: [f64; 2],
    metadata_name: &str,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
) {
    let Some(layer_list) = item.named_child("layer") else {
        return;
    };
    let layers = layer_list
        .children()
        .iter()
        .skip(1)
        .filter_map(|item| item.as_atom().map(str::to_string))
        .collect::<Vec<_>>();
    let copper_layers = expand_copper_layers(&layers, declared_copper_layers);
    if copper_layers.is_empty() {
        return;
    }

    let valid_polygons = polygons
        .into_iter()
        .filter(|polygon| polygon.exterior().0.len() >= 4)
        .collect::<Vec<_>>();
    if valid_polygons.is_empty() {
        return;
    }

    for layer in copper_layers {
        for polygon in &valid_polygons {
            copper.push(CopperFeature {
                layer: layer.clone(),
                net: None,
                kind,
                sketch: polygons_to_sketch(
                    vec![polygon.clone()],
                    Some(LayerMetadata {
                        name: metadata_name.to_string(),
                    }),
                ),
                location,
            });
        }
    }
}
