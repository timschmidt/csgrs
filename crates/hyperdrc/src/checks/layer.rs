use csgrs::csg::CSG;
use geo::{Area, BoundingRect, Coord, LineString, MultiPolygon, Polygon};

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

pub fn paste_aperture_coverage(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let uncovered_copper = copper.difference(paste);
    shapes_violation(
        "paste-aperture-coverage",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        uncovered_copper,
        min_area,
        "copper is not covered by a paste aperture".to_string(),
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

pub fn solder_mask_opening_coverage(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask_openings: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let covered_copper = copper.difference(mask_openings);
    shapes_violation(
        "solder-mask-opening-coverage",
        Severity::Error,
        vec![copper_name.to_string(), mask_name.to_string()],
        covered_copper,
        min_area,
        "copper is not covered by a solder mask opening".to_string(),
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

pub fn silkscreen_min_width(
    silk_name: &str,
    silk: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    let reconstructed = silk.offset(-radius).offset(radius);
    let thin_features = silk.difference(&reconstructed);
    shapes_violation(
        "silkscreen-min-width",
        Severity::Warning,
        vec![silk_name.to_string()],
        thin_features,
        min_area,
        format!("silkscreen features are removed by opening with width {min_width}"),
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
    let source = copper.to_multipolygon();
    let thin = thin_features.to_multipolygon();
    let shapes = multipolygon_to_shapes(&thin, min_area);

    if shapes.is_empty() || whole_feature_removal_is_width_compliant(&source, &thin, min_width) {
        return Vec::new();
    }

    vec![Violation::new(
        "minimum-copper-neck-width",
        Severity::Warning,
        vec![copper_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(format!(
            "copper features are removed by opening with width {min_width}"
        )),
    )]
}

fn whole_feature_removal_is_width_compliant(
    source: &MultiPolygon<f64>,
    removed: &MultiPolygon<f64>,
    min_width: f64,
) -> bool {
    let source_area = source.unsigned_area();
    if source_area == 0.0 || (removed.unsigned_area() - source_area).abs() > source_area * 1.0e-6 {
        return false;
    }

    source
        .0
        .iter()
        .all(|polygon| shortest_exterior_segment(polygon) >= min_width)
}

fn shortest_exterior_segment(polygon: &Polygon<f64>) -> f64 {
    polygon
        .exterior()
        .0
        .windows(2)
        .map(|segment| {
            let dx = segment[1].x - segment[0].x;
            let dy = segment[1].y - segment[0].y;
            (dx * dx + dy * dy).sqrt()
        })
        .filter(|length| *length > 1.0e-12)
        .fold(f64::INFINITY, f64::min)
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

pub fn mechanical_layer_geometry(
    layer_name: &str,
    sketch: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    if !looks_like_mechanical_layer(layer_name) {
        return Vec::new();
    }

    let shapes = multipolygon_to_shapes(&sketch.to_multipolygon(), min_area);
    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "mechanical-layer-geometry",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some("geometry is present on a mechanical/user layer".to_string()),
    )]
}

pub fn board_outline_sanity(
    layer_name: &str,
    outline: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes(&outline.to_multipolygon(), min_area);
    if !shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some("board outline layer has no closed polygon area".to_string()),
    )]
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

fn looks_like_mechanical_layer(layer_name: &str) -> bool {
    let lower = layer_name.to_ascii_lowercase();
    lower.contains("mechanical")
        || lower.contains("mech")
        || lower.contains("user.")
        || lower.contains("dwgs.user")
        || lower.contains("cmts.user")
        || lower.contains("fab")
        || lower.contains("eco")
        || lower.contains("margin")
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
        acid_trap_candidates, board_edge_clearance, board_outline_sanity, copper_overlap,
        exposed_copper, layer_sanity, mask_island_keepout, mechanical_layer_geometry,
        min_copper_neck_width, paste_aperture_coverage, paste_overhang, silkscreen_min_width,
        silkscreen_overlap, solder_mask_opening_coverage, solder_mask_sliver,
    };
    use crate::LayerMetadata;
    use crate::geometry::{empty_sketch, line_polygon, polygons_to_sketch};

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
    fn paste_aperture_coverage_reports_undersized_or_missing_apertures() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let paste = sketch("paste", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations = paste_aperture_coverage("paste", &paste, "top", &copper, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-aperture-coverage");
    }

    #[test]
    fn paste_aperture_coverage_allows_full_apertures() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = sketch("paste", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations = paste_aperture_coverage("paste", &paste, "top", &copper, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn solder_mask_sliver_reports_thin_mask_webs() {
        let mask = sketch("mask", vec![square(0.0, 0.0, 0.05, 2.0)]);

        let violations = solder_mask_sliver("mask", &mask, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn minimum_line_width_flags_trace_below_three_mil_threshold() {
        let three_mil_mm = 0.0762;
        let narrow_trace = sketch(
            "top",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], three_mil_mm * 0.8).unwrap()],
        );

        assert_eq!(
            min_copper_neck_width("top", &narrow_trace, three_mil_mm, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn minimum_line_width_allows_six_mil_preferred_trace() {
        let three_mil_mm = 0.0762;
        let six_mil_mm = 0.1524;
        let preferred_trace = sketch(
            "top",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], six_mil_mm).unwrap()],
        );

        let violations = min_copper_neck_width("top", &preferred_trace, three_mil_mm, 1.0e-9);

        assert!(
            violations.is_empty(),
            "unexpected six mil trace violation area: {:?}",
            violations
                .iter()
                .map(|violation| violation.total_area)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn board_edge_clearance_covers_trace_below_point_two_mm() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let too_close_trace = sketch(
            "top",
            vec![line_polygon([0.10, 1.0], [0.10, 9.0], 0.05).unwrap()],
        );
        let compliant_trace = sketch(
            "top",
            vec![line_polygon([0.35, 1.0], [0.35, 9.0], 0.05).unwrap()],
        );

        assert_eq!(
            board_edge_clearance("top", &too_close_trace, "edge", &board, 0.20, 1.0e-9).len(),
            1
        );
        assert!(
            board_edge_clearance("top", &compliant_trace, "edge", &board, 0.20, 1.0e-9).is_empty()
        );
    }

    #[test]
    fn board_edge_clearance_reports_pad_crossing_outline() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let pad = sketch("top", vec![square(9.8, 4.0, 10.2, 4.4)]);

        let violations = board_edge_clearance("top", &pad, "edge", &board, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_reports_empty_outline_layers() {
        let outline = empty_sketch(Some(LayerMetadata {
            name: "edge".to_string(),
        }));

        let violations = board_outline_sanity("edge", &outline, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_accepts_closed_outline_area() {
        let outline = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(board_outline_sanity("edge", &outline, 1.0e-9).is_empty());
    }

    #[test]
    fn exposed_copper_reports_oversized_mask_opening_touching_neighbor() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );
        let mask_opening = sketch("mask", vec![square(-0.1, -0.1, 1.35, 1.1)]);

        let violations = exposed_copper("top", &copper, "mask", &mask_opening, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn solder_mask_opening_coverage_reports_undersized_or_missing_openings() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let mask_openings = sketch("mask", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations =
            solder_mask_opening_coverage("top", &copper, "mask", &mask_openings, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-opening-coverage");
    }

    #[test]
    fn solder_mask_opening_coverage_allows_full_openings() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = sketch("mask", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations =
            solder_mask_opening_coverage("top", &copper, "mask", &mask_openings, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_pad_or_slot() {
        let pad_opening = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = sketch(
            "silk",
            vec![line_polygon([-0.2, 0.5], [1.2, 0.5], 0.08).unwrap()],
        );

        let violations =
            silkscreen_overlap("silk", &silk_text_stroke, "mask", &pad_opening, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_v_score_or_slot_geometry() {
        let panel_feature = sketch(
            "V-Score",
            vec![line_polygon([0.5, -1.0], [0.5, 1.0], 0.12).unwrap()],
        );
        let silk_text_stroke = sketch(
            "B.SilkS",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_overlap(
            "B.SilkS",
            &silk_text_stroke,
            "V-Score",
            &panel_feature,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-overlap");
    }

    #[test]
    fn silkscreen_min_width_reports_thin_legend_strokes() {
        let silk = sketch(
            "silk",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_min_width("silk", &silk, 0.12, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_min_width_allows_wide_legend_strokes() {
        let silk = sketch("silk", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let violations = silkscreen_min_width("silk", &silk, 0.12, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn layer_sanity_reports_empty_or_unbounded_layers() {
        let empty = empty_sketch(Some(LayerMetadata {
            name: "empty mask".to_string(),
        }));

        let violations = layer_sanity("empty mask", &empty, None);

        assert_eq!(violations.len(), 2);
        assert!(
            violations.iter().any(|violation| violation
                .message
                .as_deref()
                .unwrap()
                .contains("empty"))
        );
        assert!(
            violations.iter().any(|violation| violation
                .message
                .as_deref()
                .unwrap()
                .contains("bounding"))
        );
    }

    #[test]
    fn layer_sanity_reports_area_excursions() {
        let flood = sketch("inner", vec![square(0.0, 0.0, 20.0, 20.0)]);

        let violations = layer_sanity("inner", &flood, Some(100.0));

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("exceeds maximum")
        );
    }

    #[test]
    fn mechanical_layer_geometry_reports_shapes_on_user_or_mechanical_layers() {
        let user = sketch("Dwgs.User", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mechanical = sketch("board-Mechanical.gbr", vec![square(2.0, 0.0, 3.0, 1.0)]);

        assert_eq!(
            mechanical_layer_geometry("Dwgs.User", &user, 1.0e-9).len(),
            1
        );
        assert_eq!(
            mechanical_layer_geometry("board-Mechanical.gbr", &mechanical, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn mechanical_layer_geometry_ignores_normal_copper_layers() {
        let copper = sketch("F.Cu", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(mechanical_layer_geometry("F.Cu", &copper, 1.0e-9).is_empty());
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
