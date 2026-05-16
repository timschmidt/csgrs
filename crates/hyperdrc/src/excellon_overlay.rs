//! Excellon-style overlay rendering for report geometry.
//!
//! This is a drill-map review artifact: finding points are emitted as drill
//! hits, and polygon-only findings receive a marker at their approximate center.
//! It is not intended to be sent as manufacturing drill data.

use crate::report::{Report, Severity, Violation, ViolationPolygon};

const COORD_SCALE: f64 = 1000.0;

/// Convert active report findings into an Excellon-style marker drill file.
pub fn report_to_excellon(report: &Report) -> String {
    let diameter = marker_diameter(report);
    let mut out = String::new();
    out.push_str("; HyperDRC finding marker drill map - review artifact only\n");
    out.push_str("M48\n");
    out.push_str("METRIC,TZ\n");
    out.push_str(&format!("T01C{diameter:.3}\n"));
    out.push_str("%\n");
    out.push_str("T01\n");

    for violation in &report.violations {
        append_violation_comment(&mut out, violation);
        for location in &violation.locations {
            out.push_str(&format!("{}\n", coordinate(*location)));
        }
        if violation.locations.is_empty() {
            for polygon in &violation.polygons {
                if let Some(center) = polygon_center(polygon) {
                    out.push_str(&format!("{}\n", coordinate(center)));
                }
            }
        }
    }

    out.push_str("M30\n");
    out
}

fn append_violation_comment(out: &mut String, violation: &Violation) {
    out.push_str("; ");
    out.push_str(match violation.severity {
        Severity::Error => "ERROR",
        Severity::Warning => "WARNING",
    });
    out.push(' ');
    out.push_str(&comment_text(&violation.id));
    out.push(' ');
    out.push_str(&comment_text(&violation.check));
    if let Some(message) = &violation.message {
        out.push(' ');
        out.push_str(&comment_text(message));
    }
    out.push('\n');
}

fn polygon_center(polygon: &ViolationPolygon) -> Option<[f64; 2]> {
    let mut bounds = Bounds::empty();
    for point in &polygon.exterior {
        bounds.include(*point);
    }
    bounds.center()
}

fn marker_diameter(report: &Report) -> f64 {
    let mut bounds = Bounds::empty();
    for violation in &report.violations {
        for polygon in &violation.polygons {
            for point in &polygon.exterior {
                bounds.include(*point);
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
            .clamp(0.10, 0.80)
    } else {
        0.30
    }
}

fn coordinate(point: [f64; 2]) -> String {
    format!(
        "X{}Y{}",
        excellon_number(point[0]),
        excellon_number(point[1])
    )
}

fn excellon_number(value: f64) -> String {
    let scaled = (value * COORD_SCALE).round() as i64;
    scaled.to_string()
}

fn comment_text(value: &str) -> String {
    value
        .chars()
        .map(|ch| match ch {
            '\n' | '\r' => ' ',
            ch if ch.is_ascii_graphic() || ch == ' ' => ch,
            _ => '?',
        })
        .collect::<String>()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
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

    fn center(&self) -> Option<[f64; 2]> {
        self.is_valid().then_some([
            (self.min_x + self.max_x) * 0.5,
            (self.min_y + self.max_y) * 0.5,
        ])
    }
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::report_to_excellon;

    #[test]
    fn renders_drill_markers_for_points_and_polygons() {
        let violations = vec![
            Violation::new(
                "point",
                Severity::Warning,
                vec!["F.Cu".to_string()],
                None,
                Vec::new(),
                vec![[1.25, 2.5]],
                Some("point marker".to_string()),
            ),
            Violation::new(
                "poly",
                Severity::Error,
                vec!["B.Cu".to_string()],
                None,
                vec![ViolationPolygon {
                    area: 4.0,
                    exterior: vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 0.0]],
                    holes: Vec::new(),
                }],
                Vec::new(),
                Some("poly marker\ncomment".to_string()),
            ),
        ];
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

        let excellon = report_to_excellon(&report);

        assert!(excellon.contains("M48\n"));
        assert!(excellon.contains("METRIC,TZ\n"));
        assert!(excellon.contains("T01C"));
        assert!(excellon.contains("X1250Y2500"));
        assert!(excellon.contains("X1000Y1000"));
        assert!(excellon.ends_with("M30\n"));
        assert!(!excellon.contains('\r'));
    }
}
