//! KiCad PCB loader.
//!
//! This module keeps parser mechanics close to KiCad S-expression handling and
//! re-exports the board model used by checks.
//!
//! Reliability note: KiCad syntax evolves and custom footprint primitives can
//! encode geometry in ways this parser does not fully model. Suspect parsed
//! copper, drill, or panel results should be verified against KiCad's own DRC
//! and plotted manufacturing outputs.

mod arcs;
mod custom_pad;
mod footprint_graphics;
mod graphic_primitives;
mod graphics;
mod model;
mod text;

use custom_pad::custom_pad_polygons;
use footprint_graphics::parse_footprint_graphics;
use graphics::parse_graphics;
pub use model::{BoardModel, CopperFeature, CopperKind, DrillFeature};

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use geo::{Area, Polygon};

use crate::LayerMetadata;
use crate::geometry::{
    chamfered_rect_polygon, circle_polygon, line_polygon, polygon_from_points, polygons_to_sketch,
    rect_polygon, rounded_rect_polygon, trapezoid_polygon,
};
use crate::sexp::{self, Sexp};

/// Run or compute `load_kicad_pcb`.
pub fn load_kicad_pcb(path: &Path) -> Result<BoardModel> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let root = sexp::parse(&text).with_context(|| format!("failed to parse {}", path.display()))?;
    let nets = parse_nets(&root);
    let declared_copper_layers = parse_declared_copper_layers(&root);
    let mut copper = Vec::new();
    let mut drills = Vec::new();
    let mut edge_polygons = Vec::new();
    let mut panel_polygons = Vec::new();

    parse_footprints(
        &root,
        &nets,
        &declared_copper_layers,
        &mut copper,
        &mut drills,
    );
    parse_tracks_and_vias(
        &root,
        &nets,
        &declared_copper_layers,
        &mut copper,
        &mut drills,
    );
    parse_zones(&root, &nets, &mut copper);
    parse_graphics(&root, &mut edge_polygons, &mut panel_polygons);

    Ok(BoardModel {
        source: path.display().to_string(),
        copper,
        drills,
        board_outline: (!edge_polygons.is_empty()).then(|| {
            polygons_to_sketch(
                edge_polygons,
                Some(LayerMetadata {
                    name: "KiCad Edge.Cuts".to_string(),
                }),
            )
        }),
        panel_features: (!panel_polygons.is_empty()).then(|| {
            polygons_to_sketch(
                panel_polygons,
                Some(LayerMetadata {
                    name: "KiCad panel features".to_string(),
                }),
            )
        }),
    })
}

fn parse_nets(root: &Sexp) -> HashMap<i32, String> {
    root.named_children("net")
        .filter_map(|net| Some((net.i32_at(1)?, net.atom_at(2)?.to_string())))
        .collect()
}

fn parse_declared_copper_layers(root: &Sexp) -> Vec<String> {
    // KiCad's board-level `(layers ...)` table is the authoritative expansion
    // context for `"*.Cu"` pad/via/graphics layer wildcards. See the KiCad
    // S-expression layer documentation referenced in the project README.
    let layers = root
        .named_child("layers")
        .into_iter()
        .flat_map(|layers| layers.children().iter().skip(1))
        .filter_map(|layer| layer.atom_at(1))
        .filter(|name| name.ends_with(".Cu"))
        .map(str::to_string)
        .collect::<Vec<_>>();

    if !layers.is_empty() {
        log::trace!(
            "parsed KiCad declared copper layers: count={} layers={:?}",
            layers.len(),
            layers
        );
    }

    layers
}

fn parse_footprints(
    root: &Sexp,
    nets: &HashMap<i32, String>,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
    drills: &mut Vec<DrillFeature>,
) {
    for footprint in root.named_children("footprint") {
        let at = xy_from_child(footprint, "at").unwrap_or([0.0, 0.0]);
        let footprint_angle = footprint
            .named_child("at")
            .and_then(|at| at.f64_at(3))
            .unwrap_or(0.0);

        for pad in footprint.named_children("pad") {
            let pad_at = xy_from_child(pad, "at").unwrap_or([0.0, 0.0]);
            let pad_angle = pad
                .named_child("at")
                .and_then(|at| at.f64_at(3))
                .unwrap_or(0.0);
            let location = rotate_translate(pad_at, at, footprint_angle);
            let size = xy_from_child(pad, "size").unwrap_or([0.0, 0.0]);
            let Some(layers) = atom_values(pad.named_child("layers")) else {
                continue;
            };
            let shape = pad.atom_at(3).unwrap_or("circle");
            let net = net_name(pad, nets);

            let pad_angle_absolute = footprint_angle + pad_angle;
            let polygons = pad_polygons(pad, shape, location, size, pad_angle_absolute)
                .into_iter()
                .filter(|polygon| polygon.unsigned_area() > 0.0)
                .collect::<Vec<_>>();

            for layer in expand_copper_layers(&layers, declared_copper_layers) {
                for polygon in &polygons {
                    copper.push(CopperFeature {
                        layer: layer.clone(),
                        net: net.clone(),
                        kind: CopperKind::Pad,
                        sketch: polygons_to_sketch(
                            vec![polygon.clone()],
                            Some(LayerMetadata {
                                name: "KiCad pad".to_string(),
                            }),
                        ),
                        location,
                    });
                }
            }

            if let Some(drill_spec) = pad.named_child("drill")
                && let Some(drill) = drill_diameter(drill_spec)
            {
                let drill_location =
                    drill_location_from_pad(drill_spec, location, pad_angle_absolute);
                drills.push(DrillFeature {
                    location: drill_location,
                    diameter: drill,
                    net,
                    plated: pad.atom_at(2) != Some("np_thru_hole"),
                });
            }
        }

        parse_footprint_graphics(
            footprint,
            at,
            footprint_angle,
            declared_copper_layers,
            copper,
        );
    }
}

fn pad_polygons(
    pad: &Sexp,
    shape: &str,
    location: [f64; 2],
    size: [f64; 2],
    angle_degrees: f64,
) -> Vec<Polygon<f64>> {
    if shape == "custom" {
        let polygons = custom_pad_polygons(pad, location, angle_degrees);
        if !polygons.is_empty() {
            return polygons;
        }
    }

    match shape {
        "circle" => vec![circle_polygon(location, size[0].max(size[1]) / 2.0, 48)],
        "oval" => oval_polygons(location, size, angle_degrees),
        "trapezoid" => {
            let delta = xy_from_child(pad, "rect_delta").unwrap_or([0.0, 0.0]);
            log::trace!(
                "parsed KiCad trapezoid pad: location=({:.3},{:.3}) size=({:.3},{:.3}) delta=({:.3},{:.3})",
                location[0],
                location[1],
                size[0],
                size[1],
                delta[0],
                delta[1]
            );
            vec![trapezoid_polygon(location, size, delta, angle_degrees)]
        }
        "roundrect" => {
            if let Some(chamfer_ratio) = pad
                .named_child("chamfer_ratio")
                .and_then(|ratio| ratio.f64_at(1))
            {
                let corners = chamfered_corners(pad);
                if chamfer_ratio > 0.0 && corners.iter().any(|selected| *selected) {
                    let chamfer = size[0].abs().min(size[1].abs()) * chamfer_ratio.clamp(0.0, 0.5);
                    log::trace!(
                        "parsed KiCad chamfered pad: location=({:.3},{:.3}) size=({:.3},{:.3}) chamfer_ratio={:.3} chamfer={:.3} corners={:?}",
                        location[0],
                        location[1],
                        size[0],
                        size[1],
                        chamfer_ratio,
                        chamfer,
                        corners
                    );
                    return vec![chamfered_rect_polygon(
                        location,
                        size,
                        chamfer,
                        corners,
                        angle_degrees,
                    )];
                }
            }
            let ratio = pad
                .named_child("roundrect_rratio")
                .and_then(|ratio| ratio.f64_at(1))
                .unwrap_or(0.25)
                .clamp(0.0, 0.5);
            let radius = size[0].abs().min(size[1].abs()) * ratio;
            log::trace!(
                "parsed KiCad roundrect pad: location=({:.3},{:.3}) size=({:.3},{:.3}) rratio={:.3} radius={:.3}",
                location[0],
                location[1],
                size[0],
                size[1],
                ratio,
                radius
            );
            vec![rounded_rect_polygon(
                location,
                size,
                radius,
                angle_degrees,
                8,
            )]
        }
        _ => vec![rect_polygon(location, size, angle_degrees)],
    }
}

fn chamfered_corners(pad: &Sexp) -> [bool; 4] {
    let Some(chamfer) = pad.named_child("chamfer") else {
        return [false; 4];
    };
    [
        chamfer
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("top_left")),
        chamfer
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("top_right")),
        chamfer
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("bottom_right")),
        chamfer
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("bottom_left")),
    ]
}

fn oval_polygons(location: [f64; 2], size: [f64; 2], angle_degrees: f64) -> Vec<Polygon<f64>> {
    let length = size[0].max(size[1]);
    let width = size[0].min(size[1]);
    if length <= width {
        return vec![circle_polygon(location, width / 2.0, 48)];
    }

    let half_straight = (length - width) / 2.0;
    let local_a = if size[0] >= size[1] {
        [-half_straight, 0.0]
    } else {
        [0.0, -half_straight]
    };
    let local_b = if size[0] >= size[1] {
        [half_straight, 0.0]
    } else {
        [0.0, half_straight]
    };
    let a = rotate_translate(local_a, location, angle_degrees);
    let b = rotate_translate(local_b, location, angle_degrees);
    let mut polygons = Vec::new();
    if let Some(body) = line_polygon(a, b, width) {
        polygons.push(body);
    }
    polygons.push(circle_polygon(a, width / 2.0, 32));
    polygons.push(circle_polygon(b, width / 2.0, 32));
    polygons
}

fn parse_tracks_and_vias(
    root: &Sexp,
    nets: &HashMap<i32, String>,
    declared_copper_layers: &[String],
    copper: &mut Vec<CopperFeature>,
    drills: &mut Vec<DrillFeature>,
) {
    for segment in root.named_children("segment") {
        let Some(start) = xy_from_child(segment, "start") else {
            continue;
        };
        let Some(end) = xy_from_child(segment, "end") else {
            continue;
        };
        let width = segment
            .named_child("width")
            .and_then(|width| width.f64_at(1))
            .unwrap_or(0.0);
        let Some(polygon) = line_polygon(start, end, width) else {
            continue;
        };
        let layer = segment
            .named_child("layer")
            .and_then(|layer| layer.atom_at(1))
            .unwrap_or("unknown")
            .to_string();

        copper.push(CopperFeature {
            layer,
            net: net_name(segment, nets),
            kind: CopperKind::Segment,
            sketch: polygons_to_sketch(
                vec![polygon],
                Some(LayerMetadata {
                    name: "KiCad segment".to_string(),
                }),
            ),
            location: midpoint(start, end),
        });
    }

    for via in root.named_children("via") {
        let Some(location) = xy_from_child(via, "at") else {
            continue;
        };
        let size = via
            .named_child("size")
            .and_then(|size| size.f64_at(1))
            .unwrap_or(0.0);
        let drill = via
            .named_child("drill")
            .and_then(drill_diameter)
            .unwrap_or(size);
        let net = net_name(via, nets);
        let layers = atom_values(via.named_child("layers"))
            .unwrap_or_else(|| vec!["F.Cu".to_string(), "B.Cu".to_string()]);

        for layer in expand_copper_layers(&layers, declared_copper_layers) {
            copper.push(CopperFeature {
                layer,
                net: net.clone(),
                kind: CopperKind::Via,
                sketch: polygons_to_sketch(
                    vec![circle_polygon(location, size / 2.0, 48)],
                    Some(LayerMetadata {
                        name: "KiCad via".to_string(),
                    }),
                ),
                location,
            });
        }

        drills.push(DrillFeature {
            location,
            diameter: drill,
            net,
            plated: true,
        });
    }
}

fn drill_diameter(drill: &Sexp) -> Option<f64> {
    if drill.atom_at(1) == Some("oval") || drill.atom_at(1) == Some("rect") {
        return Some(drill.f64_at(2)?.max(drill.f64_at(3)?));
    }

    drill.f64_at(1)
}

/// Resolve the drilled center of a KiCad pad.
///
/// KiCad pad drill offsets are stored in the pad-local coordinate system, so
/// the parser applies the same affine rotation and translation used for pad
/// copper. This keeps annular-ring and drill-spacing readiness checks aligned
/// with the physical drill hit instead of the pad anchor. The affine transform
/// convention follows the planar geometry framing surveyed by Lee and
/// Preparata, "Computational Geometry - A Survey", IEEE Transactions on
/// Computers, 1984, <https://doi.org/10.1109/TC.1984.1676388>; the source
/// field is defined in the KiCad S-expression pad drill grammar.
fn drill_location_from_pad(
    drill: &Sexp,
    pad_location: [f64; 2],
    pad_angle_degrees: f64,
) -> [f64; 2] {
    let offset = xy_from_child(drill, "offset").unwrap_or([0.0, 0.0]);
    let location = rotate_translate(offset, pad_location, pad_angle_degrees);
    if offset != [0.0, 0.0] {
        log::trace!(
            "parsed KiCad pad drill offset: pad=({:.3},{:.3}) offset=({:.3},{:.3}) drill=({:.3},{:.3}) angle={:.3}",
            pad_location[0],
            pad_location[1],
            offset[0],
            offset[1],
            location[0],
            location[1],
            pad_angle_degrees
        );
    }
    location
}

fn parse_zones(root: &Sexp, nets: &HashMap<i32, String>, copper: &mut Vec<CopperFeature>) {
    for zone in root.named_children("zone") {
        let net = net_name(zone, nets);
        let layer = zone
            .named_child("layer")
            .and_then(|layer| layer.atom_at(1))
            .unwrap_or("unknown")
            .to_string();
        let polygons = zone
            .named_child("polygon")
            .into_iter()
            .flat_map(polygons_from_pts)
            .collect::<Vec<_>>();

        for polygon in polygons {
            copper.push(CopperFeature {
                layer: layer.clone(),
                net: net.clone(),
                kind: CopperKind::Zone,
                location: polygon
                    .exterior()
                    .0
                    .first()
                    .map(|coord| [coord.x, coord.y])
                    .unwrap_or([0.0, 0.0]),
                sketch: polygons_to_sketch(
                    vec![polygon],
                    Some(LayerMetadata {
                        name: "KiCad zone".to_string(),
                    }),
                ),
            });
        }
    }
}

pub(super) fn polygons_from_pts(parent: &Sexp) -> Vec<Polygon<f64>> {
    let points = points_from_pts(parent);

    if points.len() < 3 {
        return Vec::new();
    }

    vec![polygon_from_points(points)]
}

pub(super) fn points_from_pts(parent: &Sexp) -> Vec<[f64; 2]> {
    parent
        .named_children("pts")
        .flat_map(|pts| pts.named_children("xy"))
        .filter_map(|xy| Some([xy.f64_at(1)?, xy.f64_at(2)?]))
        .collect()
}

pub(super) fn xy_from_child(parent: &Sexp, child_name: &str) -> Option<[f64; 2]> {
    let child = parent.named_child(child_name)?;
    Some([child.f64_at(1)?, child.f64_at(2)?])
}

pub(super) fn stroke_width(parent: &Sexp, default: f64) -> f64 {
    parent
        .named_child("width")
        .and_then(|width| width.f64_at(1))
        .or_else(|| {
            parent
                .named_child("stroke")
                .and_then(|stroke| stroke.named_child("width"))
                .and_then(|width| width.f64_at(1))
        })
        .unwrap_or(default)
}

fn atom_values(list: Option<&Sexp>) -> Option<Vec<String>> {
    Some(
        list?
            .children()
            .iter()
            .skip(1)
            .filter_map(|item| item.as_atom().map(str::to_string))
            .collect(),
    )
}

fn net_name(item: &Sexp, nets: &HashMap<i32, String>) -> Option<String> {
    let net = item.named_child("net")?;
    if let Some(code) = net.i32_at(1) {
        return nets
            .get(&code)
            .cloned()
            .or_else(|| net.atom_at(2).map(str::to_string));
    }
    net.atom_at(1).map(str::to_string)
}

pub(super) fn expand_copper_layers(
    layers: &[String],
    declared_copper_layers: &[String],
) -> Vec<String> {
    let mut out = Vec::new();
    for layer in layers {
        match layer.as_str() {
            "*.Cu" => {
                if declared_copper_layers.is_empty() {
                    out.push("F.Cu".to_string());
                    out.push("B.Cu".to_string());
                } else {
                    out.extend(declared_copper_layers.iter().cloned());
                    log::trace!(
                        "expanded KiCad *.Cu wildcard over declared copper layers: count={}",
                        declared_copper_layers.len()
                    );
                }
            }
            layer if layer.ends_with(".Cu") => out.push(layer.to_string()),
            _ => {}
        }
    }
    out
}

pub(super) fn rotate_translate(point: [f64; 2], origin: [f64; 2], angle_degrees: f64) -> [f64; 2] {
    let theta = angle_degrees.to_radians();
    let cos = theta.cos();
    let sin = theta.sin();
    [
        origin[0] + point[0] * cos - point[1] * sin,
        origin[1] + point[0] * sin + point[1] * cos,
    ]
}

pub(super) fn midpoint(start: [f64; 2], end: [f64; 2]) -> [f64; 2] {
    [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0]
}

#[cfg(test)]
mod tests {
    use std::fs;

    use geo::Area;

    use super::{CopperKind, load_kicad_pcb};

    #[test]
    fn parses_basic_kicad_board_features() {
        let path = std::env::temp_dir().join("hyperdrc-basic.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "R"
                (at 10 20 0)
                (pad "1" smd rect (at 0 0) (size 1 2) (layers "F.Cu") (net 1 "GND")))
              (segment (start 0 0) (end 10 0) (width 0.25) (layer "F.Cu") (net 1))
              (via (at 5 5) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 1))
              (gr_line (start 0 0) (end 20 0) (layer "Edge.Cuts") (width 0.1))
              (gr_line (start 20 0) (end 20 10) (layer "Edge.Cuts") (width 0.1))
              (gr_line (start 20 10) (end 0 10) (layer "Edge.Cuts") (width 0.1))
              (gr_line (start 0 10) (end 0 0) (layer "Edge.Cuts") (width 0.1)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        assert!(!board.copper.is_empty());
        assert_eq!(board.drills.len(), 1);
        assert!(board.board_outline.is_some());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_custom_pad_primitives() {
        let path = std::env::temp_dir().join("hyperdrc-custom-pad.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "CUSTOM"
                (at 0 0 0)
                (pad "1" smd custom
                  (at 1 2 0)
                  (size 1 1)
                  (layers "F.Cu")
                  (net 1 "GND")
                  (primitives
                    (gr_poly
                      (pts (xy -0.5 -0.5) (xy 0.5 -0.5) (xy 0.0 0.5))
                      (width 0)
                      (fill yes))))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        assert_eq!(board.copper.len(), 1);
        assert_eq!(board.copper[0].location, [1.0, 2.0]);
        assert!(!board.copper[0].sketch.to_multipolygon().0.is_empty());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_custom_pad_unfilled_primitives_as_stroked_geometry() {
        let path = std::env::temp_dir().join("hyperdrc-custom-pad-unfilled.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "CUSTOM_STROKE"
                (at 0 0 0)
                (pad "1" smd custom
                  (at 1 2 0)
                  (size 1 1)
                  (layers "F.Cu")
                  (net 1 "GND")
                  (primitives
                    (gr_rect
                      (start -1 -0.5)
                      (end 1 0.5)
                      (width 0.1)
                      (fill no))
                    (gr_circle
                      (center 2 0)
                      (end 2.5 0)
                      (width 0.1)
                      (fill no))
                    (gr_poly
                      (pts (xy 3 0) (xy 4 0) (xy 3.5 0.8))
                      (width 0.1)
                      (fill no))))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let total_area = board
            .copper
            .iter()
            .map(|feature| feature.sketch.to_multipolygon().unsigned_area())
            .sum::<f64>();

        assert_eq!(board.copper.len(), 55);
        assert!(board.copper.iter().all(|feature| {
            feature.kind == CopperKind::Pad
                && feature.layer == "F.Cu"
                && feature.location == [1.0, 2.0]
        }));
        assert!(total_area > 1.0);
        assert!(total_area < 1.8);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_custom_pad_arc_primitives() {
        let path = std::env::temp_dir().join("hyperdrc-custom-pad-arc.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "CUSTOM_ARC"
                (at 10 20 90)
                (pad "1" smd custom
                  (at 1 0 0)
                  (size 1 1)
                  (layers "F.Cu")
                  (net 1 "GND")
                  (primitives
                    (gr_arc
                      (start 0.5 0)
                      (mid 0 0.5)
                      (end -0.5 0)
                      (width 0.1))))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let total_area = board
            .copper
            .iter()
            .map(|feature| feature.sketch.to_multipolygon().unsigned_area())
            .sum::<f64>();

        assert_eq!(board.copper.len(), 16);
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.kind == CopperKind::Pad
                    && feature.location == [10.0, 21.0]
                    && feature.layer == "F.Cu")
        );
        assert!(total_area > 0.15);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_custom_pad_bezier_and_text_primitives() {
        let path = std::env::temp_dir().join("hyperdrc-custom-pad-bezier-text.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "CUSTOM_CURVE_TEXT"
                (at 2 3 90)
                (pad "1" smd custom
                  (at 1 0 0)
                  (size 1 1)
                  (layers "F.Cu")
                  (net 1 "GND")
                  (primitives
                    (bezier
                      (pts (xy 0 0) (xy 0 1) (xy 1 1) (xy 1 0))
                      (stroke (width 0.1) (type default)))
                    (gr_text "A1"
                      (at 0.25 -0.25 45)
                      (effects
                        (font (size 0.5 0.4) (thickness 0.05))
                        (justify left top)))))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let total_area = board
            .copper
            .iter()
            .map(|feature| feature.sketch.to_multipolygon().unsigned_area())
            .sum::<f64>();

        assert_eq!(board.copper.len(), 17);
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.kind == CopperKind::Pad
                    && feature.layer == "F.Cu"
                    && feature.net.as_deref() == Some("GND")
                    && feature.location == [2.0, 4.0])
        );
        assert!(total_area > 0.45);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_modern_stroke_width_for_graphics_and_custom_pad_primitives() {
        let path = std::env::temp_dir().join("hyperdrc-stroke-widths.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "STROKE"
                (at 0 0 0)
                (pad "1" smd custom
                  (at 0 0 0)
                  (size 1 1)
                  (layers "F.Cu")
                  (primitives
                    (gr_line
                      (start -0.5 0)
                      (end 0.5 0)
                      (stroke (width 0.2) (type default))))))
              (gr_line
                (start 0 0)
                (end 10 0)
                (layer "User.Panel")
                (stroke (width 0.4) (type default))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let pad_area = board.copper[0].sketch.to_multipolygon().unsigned_area();
        let panel_area = board
            .panel_features
            .unwrap()
            .to_multipolygon()
            .unsigned_area();

        assert_eq!(board.copper.len(), 1);
        assert!(pad_area > 0.19);
        assert!(pad_area < 0.21);
        assert!(panel_area > 3.99);
        assert!(panel_area < 4.01);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_roundrect_pads_with_rratio() {
        let path = std::env::temp_dir().join("hyperdrc-roundrect-pad.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "RR"
                (at 0 0 0)
                (pad "1" smd roundrect
                  (at 0 0 0)
                  (size 2 1)
                  (layers "F.Cu")
                  (roundrect_rratio 0.25)
                  (net 1 "GND"))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 1);
        let area = board.copper[0].sketch.to_multipolygon().unsigned_area();
        assert_eq!(board.copper[0].kind, CopperKind::Pad);
        assert_eq!(board.copper[0].net.as_deref(), Some("GND"));
        assert!(area > 1.94);
        assert!(area < 1.95);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_trapezoid_pads_with_rect_delta() {
        let path = std::env::temp_dir().join("hyperdrc-trapezoid-pad.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "TRAP"
                (at 0 0 0)
                (pad "1" smd trapezoid
                  (at 0 0 0)
                  (size 2 1)
                  (rect_delta 0.2 0.1)
                  (layers "F.Cu")
                  (net 1 "GND"))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let multipolygon = board.copper[0].sketch.to_multipolygon();
        let area = multipolygon.unsigned_area();
        let coords = &multipolygon.0[0].exterior().0;

        assert_eq!(board.copper.len(), 1);
        assert_eq!(board.copper[0].kind, CopperKind::Pad);
        assert!((area - 2.0).abs() < 1.0e-9);
        assert!((coords[0].x + 1.1).abs() < 1.0e-9);
        assert!((coords[0].y - 0.7).abs() < 1.0e-9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_chamfered_roundrect_pads() {
        let path = std::env::temp_dir().join("hyperdrc-chamfered-pad.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "CHAMFER"
                (at 0 0 0)
                (pad "1" smd roundrect
                  (at 0 0 0)
                  (size 2 1)
                  (roundrect_rratio 0.25)
                  (chamfer_ratio 0.2)
                  (chamfer top_left bottom_right)
                  (layers "F.Cu")
                  (net 1 "GND"))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let multipolygon = board.copper[0].sketch.to_multipolygon();
        let area = multipolygon.unsigned_area();
        let coords = &multipolygon.0[0].exterior().0;

        assert_eq!(board.copper.len(), 1);
        assert_eq!(board.copper[0].kind, CopperKind::Pad);
        assert!((area - 1.96).abs() < 1.0e-9);
        assert_eq!(coords.len(), 7);
        assert!((coords[0].x + 0.8).abs() < 1.0e-9);
        assert!((coords[0].y + 0.5).abs() < 1.0e-9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_pad_drill_offsets_in_rotated_pad_coordinates() {
        let path = std::env::temp_dir().join("hyperdrc-pad-drill-offset.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "OFFSET_DRILL"
                (at 10 20 90)
                (pad "1" thru_hole circle
                  (at 2 0 90)
                  (size 1.4 1.4)
                  (drill 0.5 (offset 0.4 0.0))
                  (layers "*.Cu" "*.Mask")
                  (net 1 "GND"))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.drills.len(), 1);
        assert_eq!(board.drills[0].diameter, 0.5);
        assert!((board.drills[0].location[0] - 9.6).abs() < 1.0e-9);
        assert!((board.drills[0].location[1] - 22.0).abs() < 1.0e-9);
        assert_eq!(board.drills[0].net.as_deref(), Some("GND"));
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_rotated_footprint_and_pad_geometry_on_expanded_copper_layers() {
        let path = std::env::temp_dir().join("hyperdrc-rotated-pad.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "ROT"
                (at 10 20 90)
                (pad "1" smd rect
                  (at 2 0 45)
                  (size 2 4)
                  (layers "*.Cu" "F.Mask")
                  (net 1 "GND"))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 2);
        assert!(board.copper.iter().any(|feature| feature.layer == "F.Cu"));
        assert!(board.copper.iter().any(|feature| feature.layer == "B.Cu"));
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.kind == CopperKind::Pad
                    && feature.net.as_deref() == Some("GND")
                    && feature.location == [10.0, 22.0])
        );
        for feature in &board.copper {
            assert!((feature.sketch.to_multipolygon().unsigned_area() - 8.0).abs() < 1.0e-9);
        }
        let _ = fs::remove_file(path);
    }

    #[test]
    fn custom_pad_primitives_skip_degenerate_lines_and_rotate_shapes() {
        let path = std::env::temp_dir().join("hyperdrc-custom-pad-antagonistic.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "CUSTOM"
                (at 5 5 90)
                (pad "1" smd custom
                  (at 1 0 90)
                  (size 1 1)
                  (layers "F.Cu")
                  (primitives
                    (gr_rect (start -1 -0.5) (end 1 0.5) (width 0))
                    (gr_circle (center 2 0) (end 2.5 0) (width 0))
                    (gr_line (start 0 0) (end 0 0) (width 0.2))))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 2);
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.location == [5.0, 6.0])
        );
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.sketch.to_multipolygon().unsigned_area() > 0.0)
        );
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_footprint_copper_graphics() {
        let path = std::env::temp_dir().join("hyperdrc-footprint-copper-graphics.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "GRAPHICS"
                (at 10 20 90)
                (fp_line
                  (start 0 0)
                  (end 1 0)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))
                (fp_rect
                  (start 0 0)
                  (end 1 1)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default))
                  (fill yes))
                (fp_circle
                  (center 2 0)
                  (end 2.5 0)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))
                (fp_arc
                  (start 1 0)
                  (mid 0 1)
                  (end -1 0)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))
                (fp_poly
                  (pts (xy 0 0) (xy 1 0) (xy 0 1))
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default))
                  (fill yes))
                (fp_curve
                  (pts (xy 0 0) (xy 0 1) (xy 1 1) (xy 1 0))
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))
                (fp_text user "CU"
                  (at 0.5 0.5 45)
                  (layer "F.Cu")
                  (effects (font (size 0.5 0.4) (thickness 0.05))))
                (fp_line
                  (start 0 0)
                  (end 10 0)
                  (layer "F.SilkS")
                  (stroke (width 0.5) (type default)))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let total_area = board
            .copper
            .iter()
            .map(|feature| feature.sketch.to_multipolygon().unsigned_area())
            .sum::<f64>();

        assert_eq!(board.copper.len(), 37);
        assert!(board.copper.iter().all(|feature| {
            feature.layer == "F.Cu" && feature.net.is_none() && feature.location[0].is_finite()
        }));
        assert!(
            board
                .copper
                .iter()
                .any(|feature| feature.kind == CopperKind::Segment)
        );
        assert!(
            board
                .copper
                .iter()
                .any(|feature| feature.kind == CopperKind::Zone)
        );
        assert!(total_area > 3.0);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn expands_wildcard_copper_layers_from_declared_kicad_layers() {
        let path = std::env::temp_dir().join("hyperdrc-declared-copper-layers.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (layers
                (0 "F.Cu" signal)
                (1 "In1.Cu" signal)
                (2 "In2.Cu" signal)
                (31 "B.Cu" signal)
                (32 "B.Adhes" user))
              (net 1 "GND")
              (footprint "STACK"
                (at 0 0 0)
                (pad "1" smd rect
                  (at 0 0)
                  (size 1 1)
                  (layers "*.Cu")
                  (net 1 "GND"))
                (fp_line
                  (start 0 0)
                  (end 1 0)
                  (layer "*.Cu")
                  (stroke (width 0.1) (type default))))
              (via
                (at 2 0)
                (size 0.8)
                (drill 0.4)
                (layers "*.Cu")
                (net 1)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let layers = board
            .copper
            .iter()
            .map(|feature| feature.layer.as_str())
            .collect::<std::collections::BTreeSet<_>>();

        assert_eq!(board.copper.len(), 12);
        assert_eq!(layers.len(), 4);
        assert!(layers.contains("F.Cu"));
        assert!(layers.contains("In1.Cu"));
        assert!(layers.contains("In2.Cu"));
        assert!(layers.contains("B.Cu"));
        assert_eq!(
            board
                .copper
                .iter()
                .filter(|feature| feature.kind == CopperKind::Pad)
                .count(),
            4
        );
        assert_eq!(
            board
                .copper
                .iter()
                .filter(|feature| feature.kind == CopperKind::Via)
                .count(),
            4
        );
        assert_eq!(
            board
                .copper
                .iter()
                .filter(|feature| feature.kind == CopperKind::Segment)
                .count(),
            4
        );
        assert_eq!(board.drills.len(), 1);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_footprint_bezier_alias_graphics() {
        let path = std::env::temp_dir().join("hyperdrc-footprint-bezier-alias.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "GRAPHICS"
                (at 2 3 90)
                (bezier
                  (pts (xy 0 0) (xy 0 1) (xy 1 1) (xy 1 0))
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))
                (gr_curve
                  (pts (xy 2 0) (xy 2 1) (xy 3 1) (xy 3 0))
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default)))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 32);
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.kind == CopperKind::Segment && feature.layer == "F.Cu")
        );
        assert!(
            board
                .copper
                .iter()
                .all(|feature| feature.sketch.to_multipolygon().unsigned_area() > 0.0)
        );
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_footprint_unfilled_copper_graphics_as_strokes() {
        let path = std::env::temp_dir().join("hyperdrc-footprint-unfilled-graphics.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "GRAPHICS"
                (at 0 0 0)
                (fp_rect
                  (start -1 -0.5)
                  (end 1 0.5)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default))
                  (fill no))
                (fp_circle
                  (center 2 0)
                  (end 2.5 0)
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default))
                  (fill no))
                (fp_poly
                  (pts (xy 3 0) (xy 4 0) (xy 3.5 0.8))
                  (layer "F.Cu")
                  (stroke (width 0.1) (type default))
                  (fill no))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let total_area = board
            .copper
            .iter()
            .map(|feature| feature.sketch.to_multipolygon().unsigned_area())
            .sum::<f64>();

        assert_eq!(board.copper.len(), 55);
        assert!(board.copper.iter().all(|feature| {
            feature.kind == CopperKind::Segment
                && feature.layer == "F.Cu"
                && feature.net.is_none()
                && feature.location[0].is_finite()
        }));
        assert!(total_area > 1.0);
        assert!(total_area < 1.8);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_panel_arcs() {
        let path = std::env::temp_dir().join("hyperdrc-panel-arc.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (gr_arc
                (start 1 0)
                (mid 0 1)
                (end -1 0)
                (layer "User.Panel")
                (width 0.1)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        assert!(board.panel_features.is_some());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_panel_bezier_graphics() {
        let path = std::env::temp_dir().join("hyperdrc-panel-bezier.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (bezier
                (pts (xy 0 0) (xy 0 2) (xy 4 2) (xy 4 0))
                (layer "User.Panel")
                (stroke (width 0.2) (type default))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let panel_area = board
            .panel_features
            .unwrap()
            .to_multipolygon()
            .unsigned_area();

        assert!(panel_area > 0.8);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_graphic_polygons_for_edge_and_panel_layers() {
        let path = std::env::temp_dir().join("hyperdrc-graphic-polygons.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (gr_poly
                (pts (xy 0 0) (xy 4 0) (xy 4 3) (xy 0 3))
                (layer "Edge.Cuts")
                (width 0.1)
                (fill yes))
              (gr_poly
                (pts (xy 10 10) (xy 12 10) (xy 10 11))
                (layer "User.Panel")
                (width 0.1)
                (fill yes)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let outline_area = board
            .board_outline
            .unwrap()
            .to_multipolygon()
            .unsigned_area();
        let panel_area = board
            .panel_features
            .unwrap()
            .to_multipolygon()
            .unsigned_area();

        assert!((outline_area - 12.0).abs() < 1.0e-9);
        assert!((panel_area - 1.0).abs() < 1.0e-9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_oval_and_rect_drills_as_slot_keepouts() {
        let path = std::env::temp_dir().join("hyperdrc-slot-drills.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 1 "GND")
              (footprint "SLOT"
                (at 0 0 0)
                (pad "1" np_thru_hole oval
                  (at 1 2)
                  (size 1.2 2.0)
                  (drill oval 0.6 1.8)
                  (layers "*.Cu" "*.Mask")
                  (net 1 "GND"))
                (pad "2" np_thru_hole rect
                  (at 3 2)
                  (size 1.1 2.4)
                  (drill rect 0.5 2.1)
                  (layers "*.Cu" "*.Mask")
                  (net 1 "GND")))
              (via
                (at 5 5)
                (size 1.2)
                (drill oval 0.4 0.9)
                (layers "F.Cu" "B.Cu")
                (net 1)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.drills.len(), 3);
        assert_eq!(board.drills[0].diameter, 1.8);
        assert!(!board.drills[0].plated);
        assert_eq!(board.drills[1].diameter, 2.1);
        assert!(!board.drills[1].plated);
        assert_eq!(board.drills[2].diameter, 0.9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn parses_common_panelization_layer_names() {
        let path = std::env::temp_dir().join("hyperdrc-panel-layer-names.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (gr_line
                (start 0 0)
                (end 10 0)
                (layer "User.TabRoute")
                (width 0.2))
              (gr_line
                (start 0 1)
                (end 10 1)
                (layer "User.V-Score")
                (width 0.2))
              (gr_circle
                (center 1 1)
                (end 1.5 1)
                (layer "User.Castellated"))
              (gr_rect
                (start 2 2)
                (end 3 3)
                (layer "User.Edge.Plating")))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let panel_features = board.panel_features.unwrap();

        assert!(panel_features.to_multipolygon().0.len() >= 4);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn unordered_edge_lines_are_stitched_into_single_outline_polygon() {
        let path = std::env::temp_dir().join("hyperdrc-unordered-edge-lines.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (gr_line (start 10 0) (end 10 5) (layer "Edge.Cuts"))
              (gr_line (start 0 0) (end 10 0) (layer "Edge.Cuts"))
              (gr_line (start 0 5) (end 0 0) (layer "Edge.Cuts"))
              (gr_line (start 10 5) (end 0 5) (layer "Edge.Cuts")))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();
        let outline = board.board_outline.unwrap().to_multipolygon();

        assert_eq!(outline.0.len(), 1);
        assert!((outline.unsigned_area() - 50.0).abs() < 1.0e-9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn degenerate_and_non_copper_geometry_is_ignored_without_fallback_shapes() {
        let path = std::env::temp_dir().join("hyperdrc-degenerate-geometry.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (footprint "NOCU"
                (pad "1" smd rect
                  (at 0 0)
                  (size 1 1)
                  (layers "F.Mask" "B.SilkS")))
              (segment (start 0 0) (end 10 0) (width 0) (layer "F.Cu"))
              (segment (start 1 1) (end 1 1) (width 0.2) (layer "F.Cu"))
              (gr_arc
                (start 0 0)
                (mid 1 0)
                (end 2 0)
                (layer "User.Panel")
                (width 0.1)))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert!(board.copper.is_empty());
        assert!(board.panel_features.is_none());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn zones_skip_underdefined_polygons_and_keep_valid_area() {
        let path = std::env::temp_dir().join("hyperdrc-zone-antagonistic.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (net 7 "PWR")
              (zone
                (net 7)
                (layer "F.Cu")
                (polygon (pts (xy 0 0) (xy 1 0))))
              (zone
                (net 7)
                (layer "F.Cu")
                (polygon (pts (xy 0 0) (xy 2 0) (xy 2 1) (xy 0 1)))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert_eq!(board.copper.len(), 1);
        assert_eq!(board.copper[0].kind, CopperKind::Zone);
        assert_eq!(board.copper[0].net.as_deref(), Some("PWR"));
        assert!((board.copper[0].sketch.to_multipolygon().unsigned_area() - 2.0).abs() < 1.0e-9);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn malformed_kicad_file_returns_error() {
        let path = std::env::temp_dir().join("hyperdrc-malformed.kicad_pcb");
        fs::write(&path, "(kicad_pcb (net 1 GND)").unwrap();

        let result = load_kicad_pcb(&path);

        assert!(result.is_err());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn minimal_empty_board_is_valid_and_empty() {
        let path = std::env::temp_dir().join("hyperdrc-empty.kicad_pcb");
        fs::write(&path, "(kicad_pcb)").unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert!(board.copper.is_empty());
        assert!(board.drills.is_empty());
        assert!(board.board_outline.is_none());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn incomplete_objects_are_ignored_without_panicking() {
        let path = std::env::temp_dir().join("hyperdrc-incomplete.kicad_pcb");
        fs::write(
            &path,
            r#"
            (kicad_pcb
              (segment (start 0 0) (width 0.2) (layer "F.Cu"))
              (via (size 0.8))
              (footprint "BAD" (pad "1" smd rect (layers "F.Cu")))
              (zone (layer "F.Cu") (polygon (pts (xy 0 0) (xy 1 0)))))
            "#,
        )
        .unwrap();

        let board = load_kicad_pcb(&path).unwrap();

        assert!(board.copper.is_empty());
        assert!(board.drills.is_empty());
        let _ = fs::remove_file(path);
    }
}
