//! JUnit XML report sink for CI systems that do not ingest SARIF.
//!
//! JUnit XML is a de facto interchange format rather than a tightly maintained
//! standard. The adapter therefore emits the conservative subset most CI
//! systems accept: one testsuite, one testcase per active finding, and one
//! failure element per testcase carrying the geometric context in text.

use crate::report::{Report, Severity, Violation};

/// Run the `report_to_junit` design-readiness check or report helper.
pub fn report_to_junit(report: &Report) -> String {
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str(&format!(
        "<testsuite name=\"hyperdrc\" tests=\"{}\" failures=\"{}\" errors=\"0\" skipped=\"{}\">\n",
        report.violations.len(),
        report.violations.len(),
        report.waived_count
    ));
    out.push_str("  <properties>\n");
    out.push_str(&format!(
        "    <property name=\"errors\" value=\"{}\"/>\n",
        report.summary.errors
    ));
    out.push_str(&format!(
        "    <property name=\"warnings\" value=\"{}\"/>\n",
        report.summary.warnings
    ));
    out.push_str(&format!(
        "    <property name=\"waived\" value=\"{}\"/>\n",
        report.waived_count
    ));
    out.push_str(&format!(
        "    <property name=\"diagnostics\" value=\"{}\"/>\n",
        report.diagnostics.len()
    ));
    out.push_str("  </properties>\n");

    for violation in &report.violations {
        out.push_str(&testcase(violation));
    }

    out.push_str("</testsuite>\n");
    out
}

fn testcase(violation: &Violation) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "  <testcase classname=\"hyperdrc.{}\" name=\"{}\">\n",
        escape_attr(&violation.check),
        escape_attr(&violation.id)
    ));
    out.push_str(&format!(
        "    <failure type=\"{}\" message=\"{}\">{}</failure>\n",
        severity_name(violation.severity),
        escape_attr(&failure_message(violation)),
        escape_text(&failure_body(violation))
    ));
    out.push_str("  </testcase>\n");
    out
}

fn severity_name(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    }
}

fn failure_message(violation: &Violation) -> String {
    violation
        .message
        .clone()
        .unwrap_or_else(|| format!("{} on {}", violation.check, violation.layers.join(" + ")))
}

fn failure_body(violation: &Violation) -> String {
    let mut body = vec![
        format!("id: {}", violation.id),
        format!("geometry hash: {}", violation.id),
        format!("check: {}", violation.check),
        format!("severity: {}", severity_name(violation.severity)),
        format!("layers: {}", violation.layers.join(", ")),
    ];
    if !violation.locations.is_empty() {
        body.push(format!("locations: {}", coordinates(&violation.locations)));
    }
    if !violation.polygons.is_empty() {
        body.push(format!(
            "polygons: {}, total area: {:.6}",
            violation.polygons.len(),
            violation.total_area
        ));
    }
    body.join("\n")
}

fn coordinates(locations: &[[f64; 2]]) -> String {
    locations
        .iter()
        .take(8)
        .map(|point| format!("({:.6}, {:.6})", point[0], point[1]))
        .collect::<Vec<_>>()
        .join(", ")
}

fn escape_attr(value: &str) -> String {
    escape_text(value)
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

fn escape_text(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_junit;

    #[test]
    fn junit_report_contains_testcase_failures_and_escaped_geometry() {
        let violations = vec![Violation::new(
            "copper<overlap>",
            Severity::Warning,
            vec!["F.Cu".to_string(), "B.Cu".to_string()],
            None,
            Vec::new(),
            vec![[3.0, 4.0]],
            Some("overlap at U1 & U2".to_string()),
        )];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 1,
            waived_violations: Vec::new(),
            summary: report_summary(&violations, 1),
            violations,
        };

        let junit = report_to_junit(&report);

        assert!(junit.contains("<testsuite name=\"hyperdrc\" tests=\"1\" failures=\"1\""));
        assert!(junit.contains("hyperdrc.copper&lt;overlap&gt;"));
        assert!(junit.contains("overlap at U1 &amp; U2"));
        assert!(junit.contains("geometry hash:"));
        assert!(junit.contains("locations: (3.000000, 4.000000)"));
    }
}
