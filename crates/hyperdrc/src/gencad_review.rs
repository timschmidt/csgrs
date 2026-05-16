//! GenCAD-style DFT and fixture review output.
//!
//! This is a companion artifact, not a replacement CAD export. It writes a
//! small sectioned text file with HyperDRC finding summaries and available
//! electrical-test coverage evidence so test engineering can review fixture
//! risks beside normal GenCAD/CAM deliverables.

use std::collections::BTreeSet;

use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Report, Ipc356Soldermask};
use crate::report::{Report, Violation};

/// Render a GenCAD-style review companion from the report and IPC-D-356 evidence.
pub fn report_to_gencad_review(report: &Report, ipc356_reports: &[Ipc356Report]) -> String {
    let mut text = String::new();
    text.push_str("$HEADER\n");
    text.push_str("GENCAD HyperDRC-Review\n");
    text.push_str("USER HyperDRC\n");
    text.push_str("COMMENT Companion review artifact, not a replacement CAD export\n");
    text.push_str("$ENDHEADER\n\n");

    text.push_str("$HYPERDRC_SUMMARY\n");
    text.push_str(&format!("ACTIVE_FINDINGS {}\n", report.violation_count));
    text.push_str(&format!("WAIVED_FINDINGS {}\n", report.waived_count));
    text.push_str(&format!("DIAGNOSTICS {}\n", report.diagnostics.len()));
    text.push_str("$ENDHYPERDRC_SUMMARY\n\n");

    push_ipc356_nets(&mut text, ipc356_reports);
    push_ipc356_components(&mut text, ipc356_reports);
    push_ipc356_testpoints(&mut text, ipc356_reports);
    push_findings(&mut text, report);

    text.push_str("$END\n");
    text
}

fn push_ipc356_nets(text: &mut String, ipc356_reports: &[Ipc356Report]) {
    text.push_str("$NETS\n");
    if ipc356_reports.is_empty() {
        text.push_str("COMMENT No IPC-D-356 sidecar evidence loaded\n");
    }
    let mut nets = BTreeSet::<String>::new();
    for report in ipc356_reports {
        for point in &report.points {
            let net = point.net.trim();
            if !net.is_empty() {
                nets.insert(net.to_string());
            }
        }
    }
    for net in nets {
        text.push_str(&format!("NET {}\n", quote(&net)));
    }
    text.push_str("$ENDNETS\n\n");
}

fn push_ipc356_components(text: &mut String, ipc356_reports: &[Ipc356Report]) {
    text.push_str("$COMPONENTS\n");
    let mut components = BTreeSet::<String>::new();
    for report in ipc356_reports {
        for point in &report.points {
            if let Some(reference) = point.reference.as_deref()
                && !reference.trim().is_empty()
            {
                components.insert(reference.trim().to_string());
            }
        }
    }
    if components.is_empty() {
        text.push_str("COMMENT No component references recovered from IPC-D-356 evidence\n");
    }
    for reference in components {
        text.push_str(&format!("COMPONENT {}\n", quote(&reference)));
    }
    text.push_str("$ENDCOMPONENTS\n\n");
}

fn push_ipc356_testpoints(text: &mut String, ipc356_reports: &[Ipc356Report]) {
    text.push_str("$TESTPINS\n");
    let mut count = 0usize;
    for report in ipc356_reports {
        for point in &report.points {
            count += 1;
            text.push_str(&format!(
                "TESTPIN {} NET {} REF {} PIN {} X {:.6} Y {:.6} DIAMETER {} ACCESS {} FEATURE {} MASK {}\n",
                quote(&format!("TP{count}")),
                quote(blank_as_unknown(&point.net)),
                quote(point.reference.as_deref().unwrap_or("")),
                quote(point.pin.as_deref().unwrap_or("")),
                point.location[0],
                point.location[1],
                point
                    .diameter
                    .map(|diameter| format!("{diameter:.6}"))
                    .unwrap_or_else(|| "\"\"".to_string()),
                quote(access_side_label(point.access_side)),
                quote(feature_type_label(point.feature_type)),
                quote(soldermask_label(point.soldermask))
            ));
        }
    }
    if count == 0 {
        text.push_str("COMMENT No IPC-D-356 testpoint geometry loaded\n");
    }
    text.push_str("$ENDTESTPINS\n\n");
}

fn push_findings(text: &mut String, report: &Report) {
    text.push_str("$HYPERDRC_FINDINGS\n");
    for violation in report.violations.iter().filter(is_gencad_relevant) {
        push_finding(text, "ACTIVE", violation);
    }
    for violation in report.waived_violations.iter().filter(is_gencad_relevant) {
        push_finding(text, "WAIVED", violation);
    }
    text.push_str("$ENDHYPERDRC_FINDINGS\n\n");
}

fn push_finding(text: &mut String, status: &str, violation: &Violation) {
    text.push_str(&format!(
        "FINDING {} GEOMETRY_HASH {} STATUS {} CHECK {} SEVERITY {:?} LAYERS {} LOCATIONS {} POLYGONS {} AREA {:.6} MESSAGE {}\n",
        quote(&violation.id),
        quote(&violation.id),
        quote(status),
        quote(&violation.check),
        violation.severity,
        quote(&violation.layers.join(",")),
        violation.locations.len(),
        violation.polygons.len(),
        normalized_area(violation.total_area),
        quote(violation.message.as_deref().unwrap_or(""))
    ));
    for (index, location) in violation.locations.iter().enumerate() {
        text.push_str(&format!(
            "FINDING_POINT {} INDEX {} X {:.6} Y {:.6}\n",
            quote(&violation.id),
            index,
            location[0],
            location[1]
        ));
    }
}

fn normalized_area(value: f64) -> f64 {
    if value.abs() < f64::EPSILON {
        0.0
    } else {
        value
    }
}

fn is_gencad_relevant(violation: &&Violation) -> bool {
    gencad_relevant_check(&violation.check)
}

fn gencad_relevant_check(check: &str) -> bool {
    check.contains("testpoint")
        || check.contains("fixture")
        || check.contains("drill")
        || check.contains("net")
        || check.contains("via")
        || check.contains("component")
        || check.contains("assembly")
        || check.contains("panel")
        || check.contains("rout")
}

fn blank_as_unknown(value: &str) -> &str {
    let value = value.trim();
    if value.is_empty() { "<blank>" } else { value }
}

fn access_side_label(access_side: Option<Ipc356AccessSide>) -> &'static str {
    match access_side {
        Some(Ipc356AccessSide::Top) => "top",
        Some(Ipc356AccessSide::Bottom) => "bottom",
        Some(Ipc356AccessSide::Both) => "both",
        None => "",
    }
}

fn feature_type_label(feature_type: Option<Ipc356FeatureType>) -> &'static str {
    match feature_type {
        Some(Ipc356FeatureType::ThroughHole) => "through-hole",
        Some(Ipc356FeatureType::Smd) => "smd",
        Some(Ipc356FeatureType::Via) => "via",
        Some(Ipc356FeatureType::Tooling) => "tooling",
        Some(Ipc356FeatureType::Connector) => "connector",
        Some(Ipc356FeatureType::Other) => "other",
        None => "",
    }
}

fn soldermask_label(soldermask: Option<Ipc356Soldermask>) -> &'static str {
    match soldermask {
        Some(Ipc356Soldermask::Open) => "open",
        Some(Ipc356Soldermask::Covered) => "covered",
        Some(Ipc356Soldermask::Unknown) => "unknown",
        None => "",
    }
}

fn quote(value: &str) -> String {
    let clean = value
        .chars()
        .map(|character| {
            if character.is_ascii_graphic() || character == ' ' {
                character
            } else {
                ' '
            }
        })
        .collect::<String>()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ");
    format!("\"{}\"", clean.replace('\\', "\\\\").replace('"', "\\\""))
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::ipc356::parse_ipc356_report;
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_gencad_review;

    #[test]
    fn gencad_review_writes_sections_nets_and_relevant_findings() {
        let ipc = parse_ipc356_report(
            "317 /GND U1 1 X010000Y020000D000600 ACCESS=TOP MASK=OPEN FEATURE=SMD\n327 /VCC U2 2 X030000Y040000D000600 ACCESS=BOTTOM FEATURE=VIA\n",
            Path::new("test.ipc"),
        );
        let waived = Violation::new(
            "fixture-access-readiness",
            Severity::Warning,
            vec!["fixture".to_string()],
            None,
            Vec::new(),
            vec![[3.0, 4.0]],
            Some("accepted fixture limit".to_string()),
        );
        let violations = vec![
            Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec!["ipc356".to_string()],
                None,
                Vec::new(),
                vec![[1.0, 2.0]],
                Some("probe spacing risk".to_string()),
            ),
            Violation::new(
                "silkscreen-clearance",
                Severity::Warning,
                vec!["silk".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some("not relevant".to_string()),
            ),
        ];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 1,
            waived_violations: vec![waived],
            summary: report_summary(&violations, 1),
            violations,
        };

        let text = report_to_gencad_review(&report, &[ipc]);

        assert!(text.contains("$HEADER"));
        assert!(text.contains("$NETS"));
        assert!(text.contains("$COMPONENTS"));
        assert!(text.contains("COMPONENT \"U1\""));
        assert!(text.contains("COMPONENT \"U2\""));
        assert!(text.contains("$TESTPINS"));
        assert!(text.contains("TESTPIN \"TP1\" NET \"GND\" REF \"U1\" PIN \"1\""));
        assert!(text.contains("ACCESS \"top\" FEATURE \"smd\" MASK \"open\""));
        assert!(text.contains("NET \"GND\""));
        assert!(text.contains("NET \"VCC\""));
        assert!(text.contains("$HYPERDRC_FINDINGS"));
        assert!(text.contains("GEOMETRY_HASH"));
        assert!(text.contains("STATUS \"ACTIVE\""));
        assert!(text.contains("STATUS \"WAIVED\""));
        assert!(text.contains("CHECK \"testpoint-accessibility-readiness\""));
        assert!(text.contains("POLYGONS 0 AREA 0.000000"));
        assert!(text.contains("FINDING_POINT"));
        assert!(text.contains("X 1.000000 Y 2.000000"));
        assert!(text.contains("X 3.000000 Y 4.000000"));
        assert!(!text.contains("silkscreen-clearance"));
    }
}
