//! GitHub Actions annotation sink.
//!
//! GitHub workflow commands create log annotations using a compact stdout
//! protocol. PCB findings do not have source line numbers, so hyperdrc pins the
//! annotation to the URI-addressable artifact and carries coordinates in the
//! message instead of fabricating line/column data.

use crate::report::{Report, Severity, Violation};

pub fn report_to_github_annotations(report: &Report) -> String {
    let mut output = String::new();
    for violation in &report.violations {
        output.push_str(&annotation(violation));
        output.push('\n');
    }
    output
}

fn annotation(violation: &Violation) -> String {
    let command = match violation.severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    };
    let file = violation
        .layers
        .first()
        .map(String::as_str)
        .unwrap_or("hyperdrc");
    let title = format!("hyperdrc {}", violation.check);

    format!(
        "::{command} file={},title={}::{}",
        escape_property(file),
        escape_property(&title),
        escape_message(&message(violation))
    )
}

fn message(violation: &Violation) -> String {
    let mut parts = vec![
        format!("id {}", violation.id),
        violation
            .message
            .clone()
            .unwrap_or_else(|| format!("{} on {}", violation.check, violation.layers.join(" + "))),
    ];

    if !violation.locations.is_empty() {
        parts.push(format!(
            "locations {}",
            coordinate_list(&violation.locations)
        ));
    }
    if !violation.polygons.is_empty() {
        parts.push(format!(
            "{} polygon(s), total area {:.6}",
            violation.polygons.len(),
            violation.total_area
        ));
    }

    parts.join("; ")
}

fn coordinate_list(locations: &[[f64; 2]]) -> String {
    locations
        .iter()
        .take(4)
        .map(|point| format!("({:.6}, {:.6})", point[0], point[1]))
        .collect::<Vec<_>>()
        .join(", ")
}

fn escape_message(value: &str) -> String {
    value
        .replace('%', "%25")
        .replace('\r', "%0D")
        .replace('\n', "%0A")
}

fn escape_property(value: &str) -> String {
    escape_message(value)
        .replace(':', "%3A")
        .replace(',', "%2C")
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_github_annotations;

    #[test]
    fn github_annotations_escape_protocol_fields_and_include_geometry() {
        let violations = vec![Violation::new(
            "silkscreen-clearance",
            Severity::Warning,
            vec!["top,silk.gbr".to_string()],
            None,
            Vec::new(),
            vec![[1.25, 2.5]],
            Some("legend crosses pad: U1\nreview".to_string()),
        )];
        let report = Report {
            files: vec!["top,silk.gbr".to_string()],
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 0,
            summary: report_summary(&violations, 0),
            violations,
        };

        let annotations = report_to_github_annotations(&report);

        assert!(annotations.starts_with("::warning file=top%2Csilk.gbr"));
        assert!(annotations.contains("legend crosses pad: U1%0Areview"));
        assert!(annotations.contains("locations (1.250000, 2.500000)"));
    }
}
