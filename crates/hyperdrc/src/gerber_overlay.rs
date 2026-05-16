//! Gerber overlay rendering for report geometry.
//!
//! The overlay is intentionally simple RS-274X: violation polygons are emitted
//! as positive regions and point-only findings are emitted as circular flashes.
//! It is a review artifact for CAM or board viewers, not a manufacturing layer.

use crate::report::{Report, Severity, Violation, ViolationPolygon};

const COORD_SCALE: f64 = 1_000_000.0;

/// Convert active report findings into a Gerber review overlay.
pub fn report_to_gerber(report: &Report) -> String {
    report_to_gerber_with_options(
        report,
        "HyperDRC violation overlay - review artifact only",
        1.0,
    )
}

/// Convert active report findings into a Gerber keepout review layer.
pub fn report_to_gerber_keepout(report: &Report) -> String {
    report_to_gerber_with_options(
        report,
        "HyperDRC generated keepout overlay - review artifact only",
        2.0,
    )
}

fn report_to_gerber_with_options(report: &Report, title: &str, point_scale: f64) -> String {
    let point_diameter = point_diameter(report) * point_scale;
    let mut out = String::new();
    out.push_str("G04 ");
    out.push_str(title);
    out.push_str("*\n");
    out.push_str("%FSLAX46Y46*%\n");
    out.push_str("%MOMM*%\n");
    out.push_str("%TF.FileFunction,Other,Drawing*%\n");
    out.push_str("%TF.Part,Single*%\n");
    out.push_str(&format!("%ADD10C,{point_diameter:.6}*%\n"));
    out.push_str("G01*\n");
    out.push_str("%LPD*%\n");

    for violation in &report.violations {
        append_violation_comment(&mut out, violation);
        for polygon in &violation.polygons {
            append_polygon_region(&mut out, polygon);
        }
        for location in &violation.locations {
            out.push_str(&format!("{}D03*\n", coordinate(*location)));
        }
    }

    out.push_str("M02*\n");
    out
}

fn append_violation_comment(out: &mut String, violation: &Violation) {
    out.push_str("G04 ");
    out.push_str(match violation.severity {
        Severity::Error => "ERROR",
        Severity::Warning => "WARNING",
    });
    out.push(' ');
    out.push_str(&gerber_comment_text(&violation.id));
    out.push(' ');
    out.push_str(&gerber_comment_text(&violation.check));
    if let Some(message) = &violation.message {
        out.push(' ');
        out.push_str(&gerber_comment_text(message));
    }
    out.push_str("*\n");
}

fn append_polygon_region(out: &mut String, polygon: &ViolationPolygon) {
    append_ring_region(out, &polygon.exterior, true);
    for hole in &polygon.holes {
        out.push_str("%LPC*%\n");
        append_ring_region(out, hole, false);
        out.push_str("%LPD*%\n");
    }
}

fn append_ring_region(out: &mut String, ring: &[[f64; 2]], close: bool) {
    let Some(first) = ring.first().copied() else {
        return;
    };
    out.push_str("G36*\n");
    out.push_str(&format!("{}D02*\n", coordinate(first)));
    for point in ring.iter().skip(1) {
        out.push_str(&format!("{}D01*\n", coordinate(*point)));
    }
    if close && ring.last().copied() != Some(first) {
        out.push_str(&format!("{}D01*\n", coordinate(first)));
    }
    out.push_str("G37*\n");
}

fn point_diameter(report: &Report) -> f64 {
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
            .clamp(0.05, 0.50)
    } else {
        0.25
    }
}

fn coordinate(point: [f64; 2]) -> String {
    format!("X{}Y{}", gerber_number(point[0]), gerber_number(point[1]))
}

fn gerber_number(value: f64) -> String {
    let scaled = (value * COORD_SCALE).round() as i64;
    scaled.to_string()
}

fn gerber_comment_text(value: &str) -> String {
    value
        .chars()
        .map(|ch| match ch {
            '*' | '%' | '\n' | '\r' => ' ',
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
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::{report_to_gerber, report_to_gerber_keepout};

    #[test]
    fn renders_regions_and_flashes() {
        let violations = vec![
            Violation::new(
                "test-poly",
                Severity::Error,
                vec!["F.Cu".to_string()],
                None,
                vec![ViolationPolygon {
                    area: 1.0,
                    exterior: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
                    holes: vec![vec![[0.2, 0.2], [0.3, 0.2], [0.3, 0.3], [0.2, 0.2]]],
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
                Some("bad * comment % chars".to_string()),
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

        let gerber = report_to_gerber(&report);

        assert!(gerber.contains("%FSLAX46Y46*%"));
        assert!(gerber.contains("%MOMM*%"));
        assert!(gerber.contains("G36*"));
        assert!(gerber.contains("%LPC*%"));
        assert!(gerber.contains("D03*"));
        assert!(gerber.ends_with("M02*\n"));
        assert!(!gerber.contains("bad * comment"));
    }

    #[test]
    fn renders_keepout_overlay_with_larger_marker_aperture() {
        let violations = vec![Violation::new(
            "test-point",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[2.0, 2.0]],
            Some("point".to_string()),
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

        let overlay = report_to_gerber(&report);
        let keepout = report_to_gerber_keepout(&report);

        assert!(keepout.contains("generated keepout overlay"));
        assert!(keepout.contains("%ADD10C,0.100000*%"));
        assert!(overlay.contains("%ADD10C,0.050000*%"));
    }
}
