use crate::report::{Report, Severity, ViolationPolygon};

pub fn report_to_svg(report: &Report) -> String {
    let bounds = report_bounds(report).unwrap_or(Bounds {
        min_x: 0.0,
        min_y: 0.0,
        max_x: 100.0,
        max_y: 100.0,
    });
    let width = (bounds.max_x - bounds.min_x).max(1.0);
    let height = (bounds.max_y - bounds.min_y).max(1.0);
    let pad = width.max(height) * 0.05;
    let view_min_x = bounds.min_x - pad;
    let view_min_y = bounds.min_y - pad;
    let view_width = width + pad * 2.0;
    let view_height = height + pad * 2.0;
    let marker_radius = width.max(height) * 0.01;

    let mut out = String::new();
    out.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    out.push('\n');
    out.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{view_min_x:.6} {view_min_y:.6} {view_width:.6} {view_height:.6}">"#
    ));
    out.push('\n');
    out.push_str(r##"<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>"##);
    out.push('\n');
    out.push_str(&format!(
        r#"<g transform="translate(0 {:.6}) scale(1 -1)">"#,
        bounds.min_y + bounds.max_y
    ));
    out.push('\n');

    for violation in &report.violations {
        let style = style_for(violation.severity);
        for polygon in &violation.polygons {
            out.push_str(&format!(
                r#"<path d="{}" fill="{}" fill-opacity="0.35" stroke="{}" stroke-width="{}">"#,
                polygon_path(polygon),
                style.fill,
                style.stroke,
                marker_radius.max(0.01) / 2.0
            ));
            out.push_str(&title(&format!(
                "{} {} {}",
                violation.id,
                violation.check,
                violation.message.as_deref().unwrap_or("")
            )));
            out.push_str("</path>\n");
        }

        for location in &violation.locations {
            out.push_str(&format!(
                r#"<circle cx="{:.6}" cy="{:.6}" r="{:.6}" fill="{}" stroke="{}" stroke-width="{:.6}">"#,
                location[0],
                location[1],
                marker_radius.max(0.05),
                style.fill,
                style.stroke,
                marker_radius.max(0.01) / 2.0
            ));
            out.push_str(&title(&format!(
                "{} {} {}",
                violation.id,
                violation.check,
                violation.message.as_deref().unwrap_or("")
            )));
            out.push_str("</circle>\n");
        }
    }

    out.push_str("</g>\n");
    out.push_str("</svg>\n");
    out
}

fn report_bounds(report: &Report) -> Option<Bounds> {
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
    bounds.is_valid().then_some(bounds)
}

fn polygon_path(polygon: &ViolationPolygon) -> String {
    let mut parts = Vec::new();
    append_ring(&mut parts, &polygon.exterior);
    for hole in &polygon.holes {
        append_ring(&mut parts, hole);
    }
    parts.join(" ")
}

fn append_ring(parts: &mut Vec<String>, ring: &[[f64; 2]]) {
    let Some(first) = ring.first() else {
        return;
    };
    parts.push(format!("M {:.6} {:.6}", first[0], first[1]));
    for point in ring.iter().skip(1) {
        parts.push(format!("L {:.6} {:.6}", point[0], point[1]));
    }
    parts.push("Z".to_string());
}

fn title(value: &str) -> String {
    format!("<title>{}</title>", escape_xml(value))
}

fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

struct Style {
    fill: &'static str,
    stroke: &'static str,
}

fn style_for(severity: Severity) -> Style {
    match severity {
        Severity::Error => Style {
            fill: "#ff3b30",
            stroke: "#8f120c",
        },
        Severity::Warning => Style {
            fill: "#ffcc00",
            stroke: "#8a6d00",
        },
    }
}

#[derive(Copy, Clone)]
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

    fn is_valid(self) -> bool {
        self.min_x.is_finite()
            && self.min_y.is_finite()
            && self.max_x.is_finite()
            && self.max_y.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_svg;

    #[test]
    fn renders_polygon_and_point_overlay() {
        let violations = vec![
            Violation::new(
                "test-poly",
                Severity::Error,
                vec!["F.Cu".to_string()],
                None,
                vec![crate::report::ViolationPolygon {
                    area: 1.0,
                    exterior: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]],
                    holes: Vec::new(),
                }],
                Vec::new(),
                Some("poly".to_string()),
            ),
            Violation::new(
                "test-point",
                Severity::Warning,
                vec!["F.Cu".to_string()],
                None,
                Vec::new(),
                vec![[2.0, 2.0]],
                Some("point".to_string()),
            ),
        ];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 0,
            summary: report_summary(&violations, 0),
            violations,
        };

        let svg = report_to_svg(&report);

        assert!(svg.contains("<svg"));
        assert!(svg.contains("<path"));
        assert!(svg.contains("<circle"));
    }

    #[test]
    fn escapes_xml_in_titles() {
        let violations = vec![Violation::new(
            "bad<title>",
            Severity::Error,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[0.0, 0.0]],
            Some("contains & < > \" '".to_string()),
        )];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 0,
            summary: report_summary(&violations, 0),
            violations,
        };

        let svg = report_to_svg(&report);

        assert!(svg.contains("&lt;title&gt;"));
        assert!(svg.contains("&amp;"));
        assert!(svg.contains("&quot;"));
        assert!(svg.contains("&apos;"));
        assert!(!svg.contains("contains & < >"));
    }

    #[test]
    fn empty_report_still_renders_valid_svg_root() {
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: 0,
            waived_count: 0,
            summary: report_summary(&[], 0),
            violations: Vec::new(),
        };

        let svg = report_to_svg(&report);

        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
    }
}
