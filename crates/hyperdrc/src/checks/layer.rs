use csgrs::csg::CSG;
use geo::{Area, BoundingRect, Coord, LineString};

use crate::geometry::{multipolygon_to_shapes, polygon_to_sketch, polygons_to_sketch};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

pub fn mask_island_keepout(
    layer_name: &str,
    sketch: &PcbSketch,
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let polygons = sketch.to_multipolygon().0;
    let mut violations = Vec::new();

    for island_index in 0..polygons.len() {
        let island = polygon_to_sketch(polygons[island_index].clone(), Some(metadata(layer_name)));
        let remaining_polygons = polygons
            .iter()
            .enumerate()
            .filter_map(|(index, polygon)| (index != island_index).then_some(polygon.clone()))
            .collect::<Vec<_>>();

        if remaining_polygons.is_empty() {
            continue;
        }

        let remaining = polygons_to_sketch(remaining_polygons, Some(metadata(layer_name)));
        let overlap = island
            .offset(keepout)
            .intersection(&remaining.offset(keepout));
        let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);

        if !shapes.is_empty() {
            violations.push(Violation::new(
                "mask-island-keepout",
                Severity::Error,
                vec![layer_name.to_string()],
                Some(island_index),
                shapes,
                Vec::new(),
                Some(format!(
                    "island keepout intersects neighboring mask geometry after {keepout} offset"
                )),
            ));
        }
    }

    violations
}

pub fn copper_overlap(
    left_name: &str,
    left: &PcbSketch,
    right_name: &str,
    right: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "copper-overlap",
            severity: Severity::Error,
            message: "copper regions overlap across layers",
        },
        left_name,
        left,
        right_name,
        right,
        min_area,
    )
}

pub fn board_edge_clearance(
    copper_name: &str,
    copper: &PcbSketch,
    board_name: &str,
    board: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = board.offset(-clearance);
    let intrusion = copper.difference(&allowed);
    let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "copper-to-board-edge-clearance",
        Severity::Error,
        vec![copper_name.to_string(), board_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(format!(
            "copper falls outside the board outline eroded by clearance {clearance}"
        )),
    )]
}

pub fn paste_overhang(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    tolerance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = copper.offset(tolerance);
    let overhang = paste.difference(&allowed);
    shapes_violation(
        "paste-aperture-overhang",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        overhang,
        min_area,
        format!("paste extends outside copper expanded by tolerance {tolerance}"),
    )
}

pub fn exposed_copper(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask_openings: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "exposed-copper",
            severity: Severity::Warning,
            message: "copper intersects solder mask openings",
        },
        copper_name,
        copper,
        mask_name,
        mask_openings,
        min_area,
    )
}

pub fn silkscreen_overlap(
    silk_name: &str,
    silk: &PcbSketch,
    blocker_name: &str,
    blocker: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "silkscreen-overlap",
            severity: Severity::Warning,
            message: "silkscreen overlaps copper or exposed-pad geometry",
        },
        silk_name,
        silk,
        blocker_name,
        blocker,
        min_area,
    )
}

pub fn min_copper_neck_width(
    copper_name: &str,
    copper: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    // Morphological opening: erode by r, then dilate by r. Features that cannot
    // contain a disk of radius r disappear, which makes this a useful fast
    // approximation for "minimum neck width" checks on copper. This follows the
    // dilation/erosion algebra formalized in Heijmans and Ronse,
    // "The algebraic basis of mathematical morphology I. Dilations and erosions",
    // Computer Vision, Graphics, and Image Processing, 1990.
    let reconstructed = copper.offset(-radius).offset(radius);
    let thin_features = copper.difference(&reconstructed);
    shapes_violation(
        "minimum-copper-neck-width",
        Severity::Warning,
        vec![copper_name.to_string()],
        thin_features,
        min_area,
        format!("copper features are removed by opening with width {min_width}"),
    )
}

pub fn solder_mask_sliver(
    mask_name: &str,
    mask: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    // Same opening operation as the copper neck-width check, applied to residual
    // mask geometry. The result is the geometry that is too thin to survive the
    // configured web width.
    let reconstructed = mask.offset(-radius).offset(radius);
    let slivers = mask.difference(&reconstructed);
    shapes_violation(
        "solder-mask-sliver",
        Severity::Warning,
        vec![mask_name.to_string()],
        slivers,
        min_area,
        format!("solder mask geometry is removed by opening with width {min_width}"),
    )
}

pub fn acid_trap_candidates(
    copper_name: &str,
    copper: &PcbSketch,
    max_angle_degrees: f64,
) -> Vec<Violation> {
    let mut locations = Vec::new();

    for polygon in copper.to_multipolygon().0 {
        collect_acute_vertices(polygon.exterior(), max_angle_degrees, &mut locations);
        for hole in polygon.interiors() {
            collect_acute_vertices(hole, max_angle_degrees, &mut locations);
        }
    }

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "acid-trap-candidate",
        Severity::Warning,
        vec![copper_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some(format!(
            "copper polygon vertices below {max_angle_degrees} degrees"
        )),
    )]
}

pub fn layer_sanity(
    layer_name: &str,
    sketch: &PcbSketch,
    max_layer_area: Option<f64>,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let multipolygon = sketch.to_multipolygon();
    let area = multipolygon.unsigned_area();

    if area <= 0.0 {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some("layer parsed to empty polygon geometry".to_string()),
        ));
    }

    if let Some(max_layer_area) = max_layer_area
        && area > max_layer_area
    {
        let shapes = multipolygon_to_shapes(&multipolygon, 0.0);
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "layer area {area:.9} exceeds maximum expected area {max_layer_area:.9}"
            )),
        ));
    }

    if sketch.geometry.bounding_rect().is_none() {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some("layer has no finite bounding rectangle".to_string()),
        ));
    }

    violations
}

fn intersection_violation(
    spec: PairCheck<'_>,
    left_name: &str,
    left: &PcbSketch,
    right_name: &str,
    right: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let overlap = left.intersection(right);
    shapes_violation(
        spec.check,
        spec.severity,
        vec![left_name.to_string(), right_name.to_string()],
        overlap,
        min_area,
        spec.message.to_string(),
    )
}

struct PairCheck<'a> {
    check: &'a str,
    severity: Severity,
    message: &'a str,
}

fn shapes_violation(
    check: &str,
    severity: Severity,
    layers: Vec<String>,
    sketch: PcbSketch,
    min_area: f64,
    message: String,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes(&sketch.to_multipolygon(), min_area);

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        check,
        severity,
        layers,
        None,
        shapes,
        Vec::new(),
        Some(message),
    )]
}

fn collect_acute_vertices(
    ring: &LineString<f64>,
    max_angle_degrees: f64,
    locations: &mut Vec<[f64; 2]>,
) {
    let coords = open_ring_coords(ring);
    if coords.len() < 3 {
        return;
    }

    for index in 0..coords.len() {
        let previous = coords[(index + coords.len() - 1) % coords.len()];
        let current = coords[index];
        let next = coords[(index + 1) % coords.len()];
        // This is a local vertex-angle heuristic, not a full manufacturability
        // proof. It intentionally reports candidates for review because acute
        // copper notches can be caused by polygon decomposition as well as by
        // intentional footprint geometry.
        let angle = angle_degrees(previous, current, next);

        if angle > 0.0 && angle < max_angle_degrees {
            locations.push([current.x, current.y]);
        }
    }
}

fn open_ring_coords(ring: &LineString<f64>) -> Vec<Coord<f64>> {
    let mut coords = ring.0.clone();
    if coords.len() > 1 && coords.first() == coords.last() {
        coords.pop();
    }
    coords
}

fn angle_degrees(previous: Coord<f64>, current: Coord<f64>, next: Coord<f64>) -> f64 {
    let ax = previous.x - current.x;
    let ay = previous.y - current.y;
    let bx = next.x - current.x;
    let by = next.y - current.y;
    let dot = ax * bx + ay * by;
    let a_len = (ax * ax + ay * ay).sqrt();
    let b_len = (bx * bx + by * by).sqrt();

    if a_len == 0.0 || b_len == 0.0 {
        return 0.0;
    }

    let cos = (dot / (a_len * b_len)).clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

fn metadata(layer_name: &str) -> LayerMetadata {
    LayerMetadata {
        name: layer_name.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use geo::{Coord, LineString, Polygon};

    use super::{
        acid_trap_candidates, board_edge_clearance, copper_overlap, mask_island_keepout,
        paste_overhang, solder_mask_sliver,
    };
    use crate::LayerMetadata;
    use crate::geometry::polygons_to_sketch;

    #[test]
    fn mask_island_keepout_reports_expanded_island_collision() {
        let layer = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.1, 0.0, 2.1, 1.0)],
        );

        let violations = mask_island_keepout("mask", &layer, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.total_area > 0.0)
        );
    }

    #[test]
    fn mask_island_keepout_allows_distant_islands() {
        let layer = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 6.0, 1.0)],
        );

        let violations = mask_island_keepout("mask", &layer, 0.1, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn copper_overlap_reports_intersection_coordinates() {
        let top = sketch("top", vec![square(0.0, 0.0, 2.0, 2.0)]);
        let bottom = sketch("bottom", vec![square(1.0, 1.0, 3.0, 3.0)]);

        let violations = copper_overlap("top", &top, "bottom", &bottom, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].polygons.len(), 1);
        assert!((violations[0].polygons[0].area - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn board_edge_clearance_reports_copper_outside_eroded_outline() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let copper = sketch("top", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations = board_edge_clearance("top", &copper, "edge", &board, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn paste_overhang_reports_paste_outside_copper() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = sketch("paste", vec![square(-0.1, 0.0, 1.0, 1.0)]);

        let violations = paste_overhang("paste", &paste, "top", &copper, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn solder_mask_sliver_reports_thin_mask_webs() {
        let mask = sketch("mask", vec![square(0.0, 0.0, 0.05, 2.0)]);

        let violations = solder_mask_sliver("mask", &mask, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn acid_trap_reports_acute_polygon_vertices() {
        let copper = sketch(
            "top",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 2.0, y: 0.0 },
                    Coord { x: 0.1, y: 0.2 },
                    Coord { x: 0.0, y: 2.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![],
            )],
        );

        let violations = acid_trap_candidates("top", &copper, 30.0);

        assert_eq!(violations.len(), 1);
        assert!(!violations[0].locations.is_empty());
    }

    fn sketch(name: &str, polygons: Vec<Polygon<f64>>) -> crate::PcbSketch {
        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: name.to_string(),
            }),
        )
    }

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Polygon<f64> {
        Polygon::new(
            LineString(vec![
                Coord { x: min_x, y: min_y },
                Coord { x: max_x, y: min_y },
                Coord { x: max_x, y: max_y },
                Coord { x: min_x, y: max_y },
                Coord { x: min_x, y: min_y },
            ]),
            vec![],
        )
    }
}
