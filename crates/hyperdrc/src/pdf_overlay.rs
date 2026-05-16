//! PDF overlay rendering for report geometry.
//!
//! The generated PDF is a compact review artifact: active finding polygons are
//! filled and stroked, and point-only findings are rendered as circular markers.

use crate::report::{Report, Severity, ViolationPolygon};

const PT_PER_MM: f64 = 72.0 / 25.4;
const PAGE_MARGIN_PT: f64 = 36.0;

/// Convert active report findings into a single-page PDF review overlay.
pub fn report_to_pdf(report: &Report) -> Vec<u8> {
    let bounds = report_bounds(report).unwrap_or(Bounds {
        min_x: 0.0,
        min_y: 0.0,
        max_x: 100.0,
        max_y: 100.0,
    });
    let geometry_width_mm = (bounds.max_x - bounds.min_x).max(1.0);
    let geometry_height_mm = (bounds.max_y - bounds.min_y).max(1.0);
    let page_width = geometry_width_mm * PT_PER_MM + PAGE_MARGIN_PT * 2.0;
    let page_height = geometry_height_mm * PT_PER_MM + PAGE_MARGIN_PT * 2.0;
    let marker_radius =
        (geometry_width_mm.max(geometry_height_mm) * PT_PER_MM * 0.005).clamp(2.0, 8.0);

    let mut content = String::new();
    content.push_str("q\n");
    content.push_str("0.8 w\n");
    for violation in &report.violations {
        let style = style_for(violation.severity);
        content.push_str(&format!(
            "{:.3} {:.3} {:.3} rg {:.3} {:.3} {:.3} RG\n",
            style.fill[0],
            style.fill[1],
            style.fill[2],
            style.stroke[0],
            style.stroke[1],
            style.stroke[2]
        ));
        for polygon in &violation.polygons {
            append_polygon(&mut content, polygon, bounds, page_height);
        }
        for location in &violation.locations {
            let [x, y] = map_point(*location, bounds, page_height);
            append_circle(&mut content, x, y, marker_radius);
        }
    }
    content.push_str("Q\n");

    build_pdf(page_width, page_height, content.into_bytes())
}

fn append_polygon(out: &mut String, polygon: &ViolationPolygon, bounds: Bounds, page_height: f64) {
    append_ring(out, &polygon.exterior, bounds, page_height);
    for hole in &polygon.holes {
        append_ring(out, hole, bounds, page_height);
    }
    out.push_str("f*\n");
}

fn append_ring(out: &mut String, ring: &[[f64; 2]], bounds: Bounds, page_height: f64) {
    let Some(first) = ring.first().copied() else {
        return;
    };
    let [x, y] = map_point(first, bounds, page_height);
    out.push_str(&format!("{x:.3} {y:.3} m\n"));
    for point in ring.iter().skip(1) {
        let [x, y] = map_point(*point, bounds, page_height);
        out.push_str(&format!("{x:.3} {y:.3} l\n"));
    }
    out.push_str("h\n");
}

fn append_circle(out: &mut String, x: f64, y: f64, radius: f64) {
    let c = radius * 0.552_284_749_831;
    out.push_str(&format!("{:.3} {:.3} m\n", x + radius, y));
    out.push_str(&format!(
        "{:.3} {:.3} {:.3} {:.3} {:.3} {:.3} c\n",
        x + radius,
        y + c,
        x + c,
        y + radius,
        x,
        y + radius
    ));
    out.push_str(&format!(
        "{:.3} {:.3} {:.3} {:.3} {:.3} {:.3} c\n",
        x - c,
        y + radius,
        x - radius,
        y + c,
        x - radius,
        y
    ));
    out.push_str(&format!(
        "{:.3} {:.3} {:.3} {:.3} {:.3} {:.3} c\n",
        x - radius,
        y - c,
        x - c,
        y - radius,
        x,
        y - radius
    ));
    out.push_str(&format!(
        "{:.3} {:.3} {:.3} {:.3} {:.3} {:.3} c\n",
        x + c,
        y - radius,
        x + radius,
        y - c,
        x + radius,
        y
    ));
    out.push_str("h\nB\n");
}

fn map_point(point: [f64; 2], bounds: Bounds, page_height: f64) -> [f64; 2] {
    [
        PAGE_MARGIN_PT + (point[0] - bounds.min_x) * PT_PER_MM,
        page_height - PAGE_MARGIN_PT - (point[1] - bounds.min_y) * PT_PER_MM,
    ]
}

fn build_pdf(page_width: f64, page_height: f64, content: Vec<u8>) -> Vec<u8> {
    let mut objects = Vec::<Vec<u8>>::new();
    objects.push(b"<< /Type /Catalog /Pages 2 0 R >>".to_vec());
    objects.push(b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>".to_vec());
    objects.push(
        format!(
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {:.3} {:.3}] /Contents 4 0 R >>",
            page_width, page_height
        )
        .into_bytes(),
    );
    let mut stream = format!("<< /Length {} >>\nstream\n", content.len()).into_bytes();
    stream.extend_from_slice(&content);
    stream.extend_from_slice(b"endstream");
    objects.push(stream);

    let mut pdf = Vec::new();
    pdf.extend_from_slice(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n");
    let mut offsets = Vec::with_capacity(objects.len() + 1);
    offsets.push(0usize);
    for (index, object) in objects.iter().enumerate() {
        offsets.push(pdf.len());
        pdf.extend_from_slice(format!("{} 0 obj\n", index + 1).as_bytes());
        pdf.extend_from_slice(object);
        pdf.extend_from_slice(b"\nendobj\n");
    }

    let xref_offset = pdf.len();
    pdf.extend_from_slice(format!("xref\n0 {}\n", objects.len() + 1).as_bytes());
    pdf.extend_from_slice(b"0000000000 65535 f \n");
    for offset in offsets.iter().skip(1) {
        pdf.extend_from_slice(format!("{offset:010} 00000 n \n").as_bytes());
    }
    pdf.extend_from_slice(
        format!(
            "trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{}\n%%EOF\n",
            objects.len() + 1,
            xref_offset
        )
        .as_bytes(),
    );
    pdf
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

struct Style {
    fill: [f64; 3],
    stroke: [f64; 3],
}

fn style_for(severity: Severity) -> Style {
    match severity {
        Severity::Error => Style {
            fill: [1.0, 0.23, 0.19],
            stroke: [0.56, 0.07, 0.05],
        },
        Severity::Warning => Style {
            fill: [1.0, 0.80, 0.0],
            stroke: [0.54, 0.43, 0.0],
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
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::report_to_pdf;

    #[test]
    fn renders_a_valid_pdf_with_geometry_stream() {
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

        let pdf = report_to_pdf(&report);
        let text = String::from_utf8_lossy(&pdf);

        assert!(pdf.starts_with(b"%PDF-1.4"));
        assert!(text.contains("/Type /Catalog"));
        assert!(text.contains("\nstream\n"));
        assert!(text.contains("\nf*\n"));
        assert!(text.contains("\nB\n"));
        assert!(text.ends_with("%%EOF\n"));
    }
}
