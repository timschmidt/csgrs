//! KiCad review-marker board output.
//!
//! This sink writes a standalone `.kicad_pcb` companion with HyperDRC findings
//! on user layers. It is intentionally not an in-place board editor; preserving
//! a source KiCad file losslessly needs a richer writer than HyperDRC currently
//! has.

use std::hash::{Hash, Hasher};

use crate::report::{Report, Severity, Violation, ViolationPolygon};

/// Convert active report findings into a standalone KiCad marker board.
pub fn report_to_kicad_markers(report: &Report) -> String {
    let marker_radius = marker_radius(report);
    let mut out = String::new();
    out.push_str("(kicad_pcb\n");
    out.push_str("  (version 20240108)\n");
    out.push_str("  (generator \"hyperdrc\")\n");
    out.push_str("  (generator_version \"hyperdrc-marker-output\")\n");
    out.push_str("  (general\n    (thickness 1.6)\n  )\n");
    out.push_str("  (paper \"A4\")\n");
    out.push_str("  (layers\n");
    out.push_str("    (0 \"F.Cu\" signal)\n");
    out.push_str("    (31 \"B.Cu\" signal)\n");
    out.push_str("    (44 \"User.1\" user \"HYPERDRC_ERROR\")\n");
    out.push_str("    (45 \"User.2\" user \"HYPERDRC_WARNING\")\n");
    out.push_str("  )\n");
    out.push_str("  (setup\n");
    out.push_str(
        "    (pcbplotparams\n      (layerselection 0x00000000_00000000_00003000_00000000)\n    )\n",
    );
    out.push_str("  )\n");

    for (index, violation) in report.violations.iter().enumerate() {
        append_violation_markers(&mut out, violation, index, marker_radius);
    }

    out.push_str(")\n");
    out
}

/// Insert active report findings into a copy of an existing KiCad board.
pub fn merge_report_into_kicad_board(source: &str, report: &Report) -> String {
    let mut board = ensure_marker_layers(source);
    let markers = report_to_kicad_marker_items(report);
    if markers.trim().is_empty() {
        return board;
    }

    if let Some(index) = board.rfind("\n)") {
        board.insert_str(index, &markers);
        if !markers.ends_with('\n') {
            board.insert(index + markers.len(), '\n');
        }
        board
    } else {
        board.push('\n');
        board.push_str(&markers);
        board
    }
}

fn report_to_kicad_marker_items(report: &Report) -> String {
    let marker_radius = marker_radius(report);
    let mut out = String::new();
    for (index, violation) in report.violations.iter().enumerate() {
        append_violation_markers(&mut out, violation, index, marker_radius);
    }
    out
}

fn ensure_marker_layers(source: &str) -> String {
    let mut board = source.to_string();
    let Some(layers_start) = board.find("(layers") else {
        return board;
    };
    let Some(layers_end) = find_list_end(&board, layers_start) else {
        return board;
    };
    let layers = &board[layers_start..layers_end];
    let mut insert = String::new();
    if !layers.contains("\"User.1\"") {
        insert.push_str("    (44 \"User.1\" user \"HYPERDRC_ERROR\")\n");
    }
    if !layers.contains("\"User.2\"") {
        insert.push_str("    (45 \"User.2\" user \"HYPERDRC_WARNING\")\n");
    }
    if !insert.is_empty() {
        board.insert_str(layers_end.saturating_sub(1), &insert);
    }
    board
}

fn find_list_end(text: &str, start: usize) -> Option<usize> {
    let mut depth = 0usize;
    let mut in_string = false;
    let mut escaped = false;
    for (offset, ch) in text[start..].char_indices() {
        if in_string {
            if escaped {
                escaped = false;
            } else if ch == '\\' {
                escaped = true;
            } else if ch == '"' {
                in_string = false;
            }
            continue;
        }
        match ch {
            '"' => in_string = true,
            '(' => depth += 1,
            ')' => {
                depth = depth.checked_sub(1)?;
                if depth == 0 {
                    return Some(start + offset + ch.len_utf8());
                }
            }
            _ => {}
        }
    }
    None
}

fn append_violation_markers(out: &mut String, violation: &Violation, index: usize, radius: f64) {
    let layer = marker_layer(violation.severity);
    for (polygon_index, polygon) in violation.polygons.iter().enumerate() {
        append_polygon_marker(out, violation, index, polygon_index, polygon, layer);
    }
    for (point_index, point) in violation.locations.iter().enumerate() {
        append_circle_marker(out, violation, index, point_index, *point, radius, layer);
    }
    if let Some(anchor) = violation_anchor(violation) {
        append_text_marker(out, violation, index, anchor, layer);
    }
}

fn append_polygon_marker(
    out: &mut String,
    violation: &Violation,
    index: usize,
    polygon_index: usize,
    polygon: &ViolationPolygon,
    layer: &str,
) {
    if polygon.exterior.len() < 2 {
        return;
    }
    out.push_str("  (gr_poly\n");
    out.push_str("    (pts");
    for point in &polygon.exterior {
        out.push_str(&format!(" (xy {:.6} {:.6})", point[0], point[1]));
    }
    out.push_str(")\n");
    out.push_str(&format!(
        "    (stroke (width 0.100000) (type solid))\n    (fill none)\n    (layer \"{layer}\")\n    (uuid {})\n",
        marker_uuid(&violation.id, index, polygon_index, "polygon")
    ));
    out.push_str("  )\n");
    for (hole_index, hole) in polygon.holes.iter().enumerate() {
        if hole.len() < 2 {
            continue;
        }
        out.push_str("  (gr_poly\n");
        out.push_str("    (pts");
        for point in hole {
            out.push_str(&format!(" (xy {:.6} {:.6})", point[0], point[1]));
        }
        out.push_str(")\n");
        out.push_str(&format!(
            "    (stroke (width 0.050000) (type dash))\n    (fill none)\n    (layer \"{layer}\")\n    (uuid {})\n",
            marker_uuid(&violation.id, index, hole_index, "hole")
        ));
        out.push_str("  )\n");
    }
}

fn append_circle_marker(
    out: &mut String,
    violation: &Violation,
    index: usize,
    point_index: usize,
    point: [f64; 2],
    radius: f64,
    layer: &str,
) {
    out.push_str(&format!(
        "  (gr_circle\n    (center {:.6} {:.6})\n    (end {:.6} {:.6})\n    (stroke (width 0.100000) (type solid))\n    (fill none)\n    (layer \"{layer}\")\n    (uuid {})\n  )\n",
        point[0],
        point[1],
        point[0] + radius,
        point[1],
        marker_uuid(&violation.id, index, point_index, "point")
    ));
}

fn append_text_marker(
    out: &mut String,
    violation: &Violation,
    index: usize,
    point: [f64; 2],
    layer: &str,
) {
    let label = format!(
        "{} {}",
        match violation.severity {
            Severity::Error => "ERROR",
            Severity::Warning => "WARNING",
        },
        violation.check
    );
    out.push_str(&format!(
        "  (gr_text \"{}\"\n    (at {:.6} {:.6} 0)\n    (layer \"{layer}\")\n    (effects (font (size 1.000000 1.000000) (thickness 0.150000)) (justify left bottom))\n    (uuid {})\n  )\n",
        escape_kicad_string(&label),
        point[0],
        point[1],
        marker_uuid(&violation.id, index, 0, "text")
    ));
}

fn violation_anchor(violation: &Violation) -> Option<[f64; 2]> {
    violation.locations.first().copied().or_else(|| {
        violation
            .polygons
            .first()
            .and_then(|polygon| polygon.exterior.first().copied())
    })
}

fn marker_layer(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "User.1",
        Severity::Warning => "User.2",
    }
}

fn marker_radius(report: &Report) -> f64 {
    let mut bounds = Bounds::empty();
    for violation in &report.violations {
        for polygon in &violation.polygons {
            for point in &polygon.exterior {
                bounds.include(*point);
            }
            for hole in &polygon.holes {
                for point in hole {
                    bounds.include(*point);
                }
            }
        }
        for location in &violation.locations {
            bounds.include(*location);
        }
    }

    if bounds.is_valid() {
        ((bounds.max_x - bounds.min_x)
            .max(bounds.max_y - bounds.min_y)
            .max(1.0)
            * 0.01)
            .clamp(0.10, 1.00)
    } else {
        0.50
    }
}

fn marker_uuid(id: &str, index: usize, sub_index: usize, kind: &str) -> String {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    id.hash(&mut hasher);
    index.hash(&mut hasher);
    sub_index.hash(&mut hasher);
    kind.hash(&mut hasher);
    let left = hasher.finish();
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    kind.hash(&mut hasher);
    sub_index.hash(&mut hasher);
    index.hash(&mut hasher);
    id.hash(&mut hasher);
    let right = hasher.finish();
    let bytes = [
        ((left >> 56) & 0xff) as u8,
        ((left >> 48) & 0xff) as u8,
        ((left >> 40) & 0xff) as u8,
        ((left >> 32) & 0xff) as u8,
        ((left >> 24) & 0xff) as u8,
        ((left >> 16) & 0xff) as u8,
        ((left >> 8) & 0xff) as u8,
        (left & 0xff) as u8,
        ((right >> 56) & 0xff) as u8,
        ((right >> 48) & 0xff) as u8,
        ((right >> 40) & 0xff) as u8,
        ((right >> 32) & 0xff) as u8,
        ((right >> 24) & 0xff) as u8,
        ((right >> 16) & 0xff) as u8,
        ((right >> 8) & 0xff) as u8,
        (right & 0xff) as u8,
    ];
    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        bytes[8],
        bytes[9],
        bytes[10],
        bytes[11],
        bytes[12],
        bytes[13],
        bytes[14],
        bytes[15]
    )
}

fn escape_kicad_string(value: &str) -> String {
    value
        .chars()
        .flat_map(|ch| match ch {
            '\\' => "\\\\".chars().collect::<Vec<_>>(),
            '"' => "\\\"".chars().collect::<Vec<_>>(),
            '\n' | '\r' | '\t' => " ".chars().collect::<Vec<_>>(),
            ch if ch.is_ascii_graphic() || ch == ' ' => vec![ch],
            _ => "?".chars().collect::<Vec<_>>(),
        })
        .collect()
}

struct Bounds {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl Bounds {
    fn empty() -> Self {
        Self {
            min_x: f64::INFINITY,
            min_y: f64::INFINITY,
            max_x: f64::NEG_INFINITY,
            max_y: f64::NEG_INFINITY,
        }
    }

    fn include(&mut self, point: [f64; 2]) {
        self.min_x = self.min_x.min(point[0]);
        self.min_y = self.min_y.min(point[1]);
        self.max_x = self.max_x.max(point[0]);
        self.max_y = self.max_y.max(point[1]);
    }

    fn is_valid(&self) -> bool {
        self.min_x.is_finite()
            && self.min_y.is_finite()
            && self.max_x.is_finite()
            && self.max_y.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::{merge_report_into_kicad_board, report_to_kicad_markers};

    #[test]
    fn kicad_markers_write_user_layer_graphics() {
        let violations = vec![
            Violation::new(
                "poly",
                Severity::Error,
                vec!["F.Cu".to_string()],
                None,
                vec![ViolationPolygon {
                    area: 1.0,
                    exterior: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]],
                    holes: Vec::new(),
                }],
                Vec::new(),
                Some("bad \"poly\"".to_string()),
            ),
            Violation::new(
                "point",
                Severity::Warning,
                vec!["B.Cu".to_string()],
                None,
                Vec::new(),
                vec![[2.0, 3.0]],
                None,
            ),
        ];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: 2,
            waived_count: 0,
            waived_violations: Vec::new(),
            summary: report_summary(&violations, 0),
            violations,
        };

        let board = report_to_kicad_markers(&report);

        assert!(board.starts_with("(kicad_pcb"));
        assert!(board.contains("(44 \"User.1\" user \"HYPERDRC_ERROR\")"));
        assert!(board.contains("(45 \"User.2\" user \"HYPERDRC_WARNING\")"));
        assert!(board.contains("(gr_poly"));
        assert!(board.contains("(gr_circle"));
        assert!(board.contains("(gr_text \"ERROR poly\""));
        assert!(board.contains("(layer \"User.1\")"));
        assert!(board.contains("(layer \"User.2\")"));
    }

    #[test]
    fn kicad_markers_merge_into_existing_board_copy() {
        let violation = Violation::new(
            "point",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[2.0, 3.0]],
            None,
        );
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: 1,
            waived_count: 0,
            waived_violations: Vec::new(),
            summary: report_summary(std::slice::from_ref(&violation), 0),
            violations: vec![violation],
        };
        let source = "(kicad_pcb\n  (version 20240108)\n  (layers\n    (0 \"F.Cu\" signal)\n    (31 \"B.Cu\" signal)\n  )\n)\n";

        let board = merge_report_into_kicad_board(source, &report);

        assert!(board.starts_with("(kicad_pcb"));
        assert!(board.contains("(0 \"F.Cu\" signal)"));
        assert!(board.contains("(44 \"User.1\" user \"HYPERDRC_ERROR\")"));
        assert!(board.contains("(45 \"User.2\" user \"HYPERDRC_WARNING\")"));
        assert!(board.contains("(gr_circle"));
        assert!(board.ends_with(")\n"));
    }
}
