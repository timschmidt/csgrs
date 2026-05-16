//! DXF overlay rendering for report geometry.
//!
//! This emits a minimal ASCII DXF review artifact. Polygon rings become closed
//! lightweight polylines and point-only findings become circles.

use crate::report::{Report, Severity, ViolationPolygon};

/// Convert active report findings into a DXF review overlay.
pub fn report_to_dxf(report: &Report) -> String {
    let point_radius = point_radius(report);
    let mut out = String::new();
    out.push_str("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n");
    out.push_str("0\nSECTION\n2\nTABLES\n");
    out.push_str("0\nTABLE\n2\nLAYER\n70\n2\n");
    append_layer(&mut out, "HYPERDRC_ERROR", 1);
    append_layer(&mut out, "HYPERDRC_WARNING", 2);
    out.push_str("0\nENDTAB\n0\nENDSEC\n");
    out.push_str("0\nSECTION\n2\nENTITIES\n");

    for violation in &report.violations {
        let layer = match violation.severity {
            Severity::Error => "HYPERDRC_ERROR",
            Severity::Warning => "HYPERDRC_WARNING",
        };
        for polygon in &violation.polygons {
            append_polygon(&mut out, layer, polygon);
        }
        for point in &violation.locations {
            append_circle(&mut out, layer, *point, point_radius);
        }
    }

    out.push_str("0\nENDSEC\n0\nEOF\n");
    out
}

fn append_layer(out: &mut String, name: &str, color: i32) {
    out.push_str(&format!(
        "0\nLAYER\n2\n{name}\n70\n0\n62\n{color}\n6\nCONTINUOUS\n"
    ));
}

fn append_polygon(out: &mut String, layer: &str, polygon: &ViolationPolygon) {
    append_polyline(out, layer, &polygon.exterior);
    for hole in &polygon.holes {
        append_polyline(out, layer, hole);
    }
}

fn append_polyline(out: &mut String, layer: &str, ring: &[[f64; 2]]) {
    if ring.len() < 2 {
        return;
    }
    let closed_len = if ring.first() == ring.last() {
        ring.len() - 1
    } else {
        ring.len()
    };
    out.push_str(&format!(
        "0\nLWPOLYLINE\n8\n{layer}\n90\n{closed_len}\n70\n1\n"
    ));
    for point in ring.iter().take(closed_len) {
        out.push_str(&format!("10\n{:.6}\n20\n{:.6}\n", point[0], point[1]));
    }
}

fn append_circle(out: &mut String, layer: &str, point: [f64; 2], radius: f64) {
    out.push_str(&format!(
        "0\nCIRCLE\n8\n{layer}\n10\n{:.6}\n20\n{:.6}\n30\n0.000000\n40\n{radius:.6}\n",
        point[0], point[1]
    ));
}

fn point_radius(report: &Report) -> f64 {
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
            * 0.005)
            .clamp(0.025, 0.25)
    } else {
        0.125
    }
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

    use super::report_to_dxf;

    #[test]
    fn renders_polylines_and_circles() {
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
                None,
            ),
            Violation::new(
                "point",
                Severity::Warning,
                vec!["B.Cu".to_string()],
                None,
                Vec::new(),
                vec![[2.0, 2.0]],
                None,
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

        let dxf = report_to_dxf(&report);

        assert!(dxf.contains("SECTION\n2\nENTITIES"));
        assert!(dxf.contains("LWPOLYLINE"));
        assert!(dxf.contains("CIRCLE"));
        assert!(dxf.contains("HYPERDRC_ERROR"));
        assert!(dxf.ends_with("0\nEOF\n"));
    }
}
