use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use geo::{Area, Polygon};

use crate::geometry::{
    arc_line_polygons, circle_polygon, empty_sketch, line_polygon, polygon_from_points,
    polygons_to_sketch, rect_polygon, transform_polygon,
};
use crate::sexp::{self, Sexp};
use crate::{LayerMetadata, PcbSketch};

#[derive(Clone, Debug)]
pub struct BoardModel {
    pub source: String,
    pub copper: Vec<CopperFeature>,
    pub drills: Vec<DrillFeature>,
    pub board_outline: Option<PcbSketch>,
    pub panel_features: Option<PcbSketch>,
}

#[derive(Clone, Debug)]
pub struct CopperFeature {
    pub layer: String,
    pub net: Option<String>,
    pub kind: CopperKind,
    pub sketch: PcbSketch,
    pub location: [f64; 2],
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum CopperKind {
    Pad,
    Via,
    Segment,
    Zone,
}

#[derive(Clone, Debug)]
pub struct DrillFeature {
    pub location: [f64; 2],
    pub diameter: f64,
    pub net: Option<String>,
    pub plated: bool,
}

pub fn load_kicad_pcb(path: &Path) -> Result<BoardModel> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let root = sexp::parse(&text).with_context(|| format!("failed to parse {}", path.display()))?;
    let nets = parse_nets(&root);
    let mut copper = Vec::new();
    let mut drills = Vec::new();
    let mut edge_polygons = Vec::new();
    let mut panel_polygons = Vec::new();

    parse_footprints(&root, &nets, &mut copper, &mut drills);
    parse_tracks_and_vias(&root, &nets, &mut copper, &mut drills);
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

impl BoardModel {
    pub fn copper_layers(&self, selected_layers: &[String]) -> Vec<(String, PcbSketch)> {
        let mut by_layer: HashMap<String, Vec<Polygon<f64>>> = HashMap::new();

        for feature in &self.copper {
            if !selected_layers.is_empty() && !selected_layers.contains(&feature.layer) {
                continue;
            }
            by_layer
                .entry(feature.layer.clone())
                .or_default()
                .extend(feature.sketch.to_multipolygon().0);
        }

        by_layer
            .into_iter()
            .map(|(layer, polygons)| {
                let sketch = polygons_to_sketch(
                    polygons,
                    Some(LayerMetadata {
                        name: format!("KiCad {layer}"),
                    }),
                );
                (layer, sketch)
            })
            .collect()
    }

    pub fn all_copper(&self) -> PcbSketch {
        let polygons = self
            .copper
            .iter()
            .flat_map(|feature| feature.sketch.to_multipolygon().0)
            .collect::<Vec<_>>();

        if polygons.is_empty() {
            return empty_sketch(Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }));
        }

        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: "KiCad copper".to_string(),
            }),
        )
    }
}

fn parse_nets(root: &Sexp) -> HashMap<i32, String> {
    root.named_children("net")
        .filter_map(|net| Some((net.i32_at(1)?, net.atom_at(2)?.to_string())))
        .collect()
}

fn parse_footprints(
    root: &Sexp,
    nets: &HashMap<i32, String>,
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

            for layer in expand_copper_layers(&layers) {
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

            if let Some(drill) = pad.named_child("drill").and_then(drill_diameter) {
                drills.push(DrillFeature {
                    location,
                    diameter: drill,
                    net,
                    plated: pad.atom_at(2) != Some("np_thru_hole"),
                });
            }
        }
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
        _ => vec![rect_polygon(location, size, angle_degrees)],
    }
}

fn custom_pad_polygons(pad: &Sexp, location: [f64; 2], angle_degrees: f64) -> Vec<Polygon<f64>> {
    let mut polygons = Vec::new();
    let Some(primitives) = pad.named_child("primitives") else {
        return polygons;
    };

    for primitive in primitives.children().iter().skip(1) {
        match primitive.list_name() {
            Some("gr_poly") => {
                for polygon in polygons_from_pts(primitive) {
                    polygons.push(transform_polygon(&polygon, location, angle_degrees));
                }
            }
            Some("gr_rect") => {
                let Some(start) = xy_from_child(primitive, "start") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let polygon =
                    polygon_from_points(vec![start, [end[0], start[1]], end, [start[0], end[1]]]);
                polygons.push(transform_polygon(&polygon, location, angle_degrees));
            }
            Some("gr_circle") => {
                let Some(center) = xy_from_child(primitive, "center") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let radius = distance(center, end);
                let transformed_center = rotate_translate(center, location, angle_degrees);
                polygons.push(circle_polygon(transformed_center, radius, 48));
            }
            Some("gr_line") => {
                let Some(start) = xy_from_child(primitive, "start") else {
                    continue;
                };
                let Some(end) = xy_from_child(primitive, "end") else {
                    continue;
                };
                let width = primitive
                    .named_child("width")
                    .and_then(|width| width.f64_at(1))
                    .unwrap_or(0.01);
                let start = rotate_translate(start, location, angle_degrees);
                let end = rotate_translate(end, location, angle_degrees);
                if let Some(polygon) = line_polygon(start, end, width) {
                    polygons.push(polygon);
                }
            }
            _ => {}
        }
    }

    polygons
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

        for layer in expand_copper_layers(&layers) {
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

fn parse_graphics(
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
    // represented arc follows the counter-clockwise or clockwise sweep. This is
    // the standard determinant form used for three-point circle reconstruction.
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
        // exact endpoint matches into a single outline before falling back to
        // stroked line geometry. Tolerance is intentionally tiny because KiCad
        // stores board coordinates in decimal millimeters.
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

fn polygons_from_pts(parent: &Sexp) -> Vec<Polygon<f64>> {
    let points = parent
        .named_children("pts")
        .flat_map(|pts| pts.named_children("xy"))
        .filter_map(|xy| Some([xy.f64_at(1)?, xy.f64_at(2)?]))
        .collect::<Vec<_>>();

    if points.len() < 3 {
        return Vec::new();
    }

    vec![polygon_from_points(points)]
}

fn xy_from_child(parent: &Sexp, child_name: &str) -> Option<[f64; 2]> {
    let child = parent.named_child(child_name)?;
    Some([child.f64_at(1)?, child.f64_at(2)?])
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

fn expand_copper_layers(layers: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    for layer in layers {
        match layer.as_str() {
            "*.Cu" => {
                out.push("F.Cu".to_string());
                out.push("B.Cu".to_string());
            }
            layer if layer.ends_with(".Cu") => out.push(layer.to_string()),
            _ => {}
        }
    }
    out
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

fn rotate_translate(point: [f64; 2], origin: [f64; 2], angle_degrees: f64) -> [f64; 2] {
    let theta = angle_degrees.to_radians();
    let cos = theta.cos();
    let sin = theta.sin();
    [
        origin[0] + point[0] * cos - point[1] * sin,
        origin[1] + point[0] * sin + point[1] * cos,
    ]
}

fn midpoint(start: [f64; 2], end: [f64; 2]) -> [f64; 2] {
    [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0]
}

fn distance(start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::load_kicad_pcb;

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
