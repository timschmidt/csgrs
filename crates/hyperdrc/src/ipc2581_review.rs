//! IPC-2581-style manufacturing review output.
//!
//! This writer emits a small XML companion for manufacturing handoff review. It
//! is intentionally not a full IPC-2581 replacement package; it preserves
//! HyperDRC DRC/DFM annotations in an IPC-2581-like sectioned structure until a
//! complete IPC-2581 object model is available.

use crate::report::{Report, Violation};

/// Render an IPC-2581-style XML review companion from a HyperDRC report.
pub fn report_to_ipc2581_review(report: &Report) -> String {
    let mut text = String::new();
    text.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    text.push_str("<HyperDRCIPC2581Review version=\"1\">\n");
    text.push_str("  <Summary");
    push_attr(
        &mut text,
        "activeFindings",
        &report.violation_count.to_string(),
    );
    push_attr(
        &mut text,
        "waivedFindings",
        &report.waived_count.to_string(),
    );
    push_attr(
        &mut text,
        "diagnostics",
        &report.diagnostics.len().to_string(),
    );
    text.push_str("/>\n");
    text.push_str("  <DFMAnnotations>\n");
    for violation in &report.violations {
        push_violation(&mut text, violation);
    }
    text.push_str("  </DFMAnnotations>\n");
    if !report.waived_violations.is_empty() {
        text.push_str("  <WaivedAnnotations>\n");
        for violation in &report.waived_violations {
            push_violation(&mut text, violation);
        }
        text.push_str("  </WaivedAnnotations>\n");
    }
    text.push_str("</HyperDRCIPC2581Review>\n");
    text
}

fn push_violation(text: &mut String, violation: &Violation) {
    text.push_str("    <Annotation");
    push_attr(text, "id", &violation.id);
    push_attr(text, "geometryHash", &violation.id);
    push_attr(text, "check", &violation.check);
    push_attr(
        text,
        "severity",
        &format!("{:?}", violation.severity).to_ascii_lowercase(),
    );
    push_attr(text, "layers", &violation.layers.join(","));
    push_attr(text, "polygonCount", &violation.polygons.len().to_string());
    push_attr(text, "pointCount", &violation.locations.len().to_string());
    push_attr(text, "areaMm2", &format_float(violation.total_area));
    if let Some(message) = violation.message.as_deref() {
        push_attr(text, "message", message);
    }
    text.push_str(">\n");
    for location in &violation.locations {
        text.push_str("      <Location");
        push_attr(text, "x", &format_float(location[0]));
        push_attr(text, "y", &format_float(location[1]));
        text.push_str("/>\n");
    }
    for (index, polygon) in violation.polygons.iter().enumerate() {
        text.push_str("      <Region");
        push_attr(text, "index", &index.to_string());
        push_attr(text, "areaMm2", &format_float(polygon.area));
        push_attr(
            text,
            "exteriorVertices",
            &polygon.exterior.len().to_string(),
        );
        push_attr(text, "holeCount", &polygon.holes.len().to_string());
        let bounds = polygon_bounds(polygon);
        if let Some(bounds) = bounds {
            push_attr(text, "minX", &format_float(bounds.min_x));
            push_attr(text, "minY", &format_float(bounds.min_y));
            push_attr(text, "maxX", &format_float(bounds.max_x));
            push_attr(text, "maxY", &format_float(bounds.max_y));
        }
        text.push_str(">\n");
        text.push_str("        <Exterior>");
        for point in &polygon.exterior {
            text.push_str(&format!(
                "{},{} ",
                format_float(point[0]),
                format_float(point[1])
            ));
        }
        text.push_str("</Exterior>\n");
        text.push_str("      </Region>\n");
    }
    text.push_str("    </Annotation>\n");
}

struct RegionBounds {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

fn polygon_bounds(polygon: &crate::report::ViolationPolygon) -> Option<RegionBounds> {
    let mut bounds = None::<RegionBounds>;
    for point in polygon
        .exterior
        .iter()
        .chain(polygon.holes.iter().flat_map(|hole| hole.iter()))
    {
        let [x, y] = *point;
        if !(x.is_finite() && y.is_finite()) {
            continue;
        }
        if let Some(bounds) = &mut bounds {
            bounds.min_x = bounds.min_x.min(x);
            bounds.min_y = bounds.min_y.min(y);
            bounds.max_x = bounds.max_x.max(x);
            bounds.max_y = bounds.max_y.max(y);
        } else {
            bounds = Some(RegionBounds {
                min_x: x,
                min_y: y,
                max_x: x,
                max_y: y,
            });
        }
    }
    bounds
}

fn push_attr(text: &mut String, name: &str, value: &str) {
    text.push_str(&format!(" {name}=\"{}\"", escape_xml(value)));
}

fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

fn format_float(value: f64) -> String {
    let formatted = format!("{value:.6}");
    formatted
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::report_to_ipc2581_review;

    #[test]
    fn ipc2581_review_writes_xml_annotations_and_escapes_values() {
        let violations = vec![Violation::new(
            "drill-to-copper-clearance",
            Severity::Error,
            vec!["F.Cu".to_string(), "Drill".to_string()],
            None,
            vec![ViolationPolygon {
                area: 1.25,
                exterior: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]],
                holes: Vec::new(),
            }],
            vec![[0.5, 0.25]],
            Some("clearance < limit & review".to_string()),
        )];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 0,
            waived_violations: Vec::new(),
            summary: report_summary(&violations, 0),
            violations,
        };

        let xml = report_to_ipc2581_review(&report);

        assert!(xml.starts_with("<?xml version=\"1.0\""));
        assert!(xml.contains("<HyperDRCIPC2581Review version=\"1\">"));
        assert!(xml.contains("activeFindings=\"1\""));
        assert!(xml.contains("check=\"drill-to-copper-clearance\""));
        assert!(xml.contains("geometryHash=\""));
        assert!(xml.contains("severity=\"error\""));
        assert!(xml.contains("message=\"clearance &lt; limit &amp; review\""));
        assert!(xml.contains("<Location x=\"0.5\" y=\"0.25\"/>"));
        assert!(xml.contains("exteriorVertices=\"4\""));
        assert!(xml.contains("holeCount=\"0\""));
        assert!(xml.contains("minX=\"0\""));
        assert!(xml.contains("minY=\"0\""));
        assert!(xml.contains("maxX=\"1\""));
        assert!(xml.contains("maxY=\"1\""));
        assert!(xml.contains("<Exterior>0,0 1,0 1,1 0,0 </Exterior>"));
    }
}
