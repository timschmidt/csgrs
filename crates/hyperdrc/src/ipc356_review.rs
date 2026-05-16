//! IPC-D-356 electrical-test review output.
//!
//! The writer emits an annotation-oriented IPC-D-356 companion file rather than
//! a replacement fabrication netlist. Comment records summarize parsed test
//! access coverage and active HyperDRC findings that may affect electrical-test
//! or drill/net review.

use std::collections::BTreeMap;

use crate::ipc356::{Ipc356AccessSide, Ipc356Point, Ipc356Report, Ipc356Soldermask};
use crate::report::Report;

/// Render an annotated IPC-D-356 review companion from parsed IPC-D-356 inputs.
pub fn report_to_ipc356_review(report: &Report, ipc356_reports: &[Ipc356Report]) -> String {
    let mut text = String::new();
    text.push_str("C HyperDRC IPC-D-356 electrical-test review annotations\n");
    text.push_str("C This companion file preserves review evidence; it is not a replacement fabrication netlist.\n");
    text.push_str(&format!(
        "C ACTIVE_FINDINGS TOTAL={} WAIVED={}\n",
        report.violation_count, report.waived_count
    ));
    if ipc356_reports.is_empty() {
        text.push_str("C IPC356_INPUTS COUNT=0 STATUS=missing\n");
    }
    for source_report in ipc356_reports {
        push_source_summary(&mut text, source_report);
        push_net_summaries(&mut text, source_report);
    }
    push_finding_summary(&mut text, report);
    push_machine_diff(&mut text, report, ipc356_reports);
    text.push_str("C END_HYPERDRC_IPC356_REVIEW\n");
    text
}

fn push_source_summary(text: &mut String, report: &Ipc356Report) {
    text.push_str(&format!(
        "C SOURCE {} POINTS={} UNIQUE_NETS={} BLANK_NET_RECORDS={} MALFORMED={} TOP_ACCESS={} BOTTOM_ACCESS={} BOTH_ACCESS={} MASK_OPEN={} MASK_COVERED={}\n",
        sanitize(&report.source),
        report.points.len(),
        report.net_stats.unique_nets,
        report.net_stats.blank_records,
        report.issue_stats.malformed_test_records,
        report.metadata_stats.top_access,
        report.metadata_stats.bottom_access,
        report.metadata_stats.both_access,
        report.metadata_stats.open_soldermask,
        report.metadata_stats.covered_soldermask
    ));
}

fn push_net_summaries(text: &mut String, report: &Ipc356Report) {
    let mut nets = BTreeMap::<String, NetAccessSummary>::new();
    for point in &report.points {
        let net = if point.net.trim().is_empty() {
            "<blank>".to_string()
        } else {
            point.net.trim().to_string()
        };
        nets.entry(net).or_default().count(point);
    }
    for (net, summary) in nets {
        text.push_str(&format!(
            "C NET {} POINTS={} TOP={} BOTTOM={} BOTH={} MASK_OPEN={} MASK_COVERED={} MISSING_ACCESS_SIDE={}\n",
            sanitize(&net),
            summary.points,
            summary.top,
            summary.bottom,
            summary.both,
            summary.mask_open,
            summary.mask_covered,
            summary.missing_access_side
        ));
    }
}

#[derive(Default)]
struct NetAccessSummary {
    points: usize,
    top: usize,
    bottom: usize,
    both: usize,
    mask_open: usize,
    mask_covered: usize,
    missing_access_side: usize,
}

impl NetAccessSummary {
    fn count(&mut self, point: &Ipc356Point) {
        self.points += 1;
        match point.access_side {
            Some(Ipc356AccessSide::Top) => self.top += 1,
            Some(Ipc356AccessSide::Bottom) => self.bottom += 1,
            Some(Ipc356AccessSide::Both) => self.both += 1,
            None => self.missing_access_side += 1,
        }
        match point.soldermask {
            Some(Ipc356Soldermask::Open) => self.mask_open += 1,
            Some(Ipc356Soldermask::Covered) => self.mask_covered += 1,
            Some(Ipc356Soldermask::Unknown) | None => {}
        }
    }
}

fn push_finding_summary(text: &mut String, report: &Report) {
    for violation in report
        .violations
        .iter()
        .filter(|violation| electrical_test_relevant_check(&violation.check))
    {
        text.push_str(&format!(
            "C FINDING ID={} GEOMETRY_HASH={} CHECK={} SEVERITY={:?} LAYERS={} LOCATIONS={} MESSAGE={}\n",
            sanitize(&violation.id),
            sanitize(&violation.id),
            sanitize(&violation.check),
            violation.severity,
            sanitize(&violation.layers.join(",")),
            violation.locations.len(),
            sanitize(violation.message.as_deref().unwrap_or(""))
        ));
    }
}

fn push_machine_diff(text: &mut String, report: &Report, ipc356_reports: &[Ipc356Report]) {
    text.push_str("C BEGIN_HYPERDRC_IPC356_DIFF FORMAT=key-value VERSION=1\n");
    text.push_str(&format!(
        "C DIFF_SUMMARY ACTIVE_RELEVANT={} WAIVED_RELEVANT={} SOURCES={} POINTS={}\n",
        report
            .violations
            .iter()
            .filter(|violation| electrical_test_relevant_check(&violation.check))
            .count(),
        report
            .waived_violations
            .iter()
            .filter(|violation| electrical_test_relevant_check(&violation.check))
            .count(),
        ipc356_reports.len(),
        ipc356_reports
            .iter()
            .map(|source_report| source_report.points.len())
            .sum::<usize>()
    ));
    for source_report in ipc356_reports {
        text.push_str(&format!(
            "C DIFF_SOURCE SOURCE={} POINTS={} NETS={} MALFORMED={} ISSUES={}\n",
            sanitize(&source_report.source),
            source_report.points.len(),
            source_report.net_stats.unique_nets,
            source_report.issue_stats.malformed_test_records,
            source_report.issues.len()
        ));
    }
    for (status, violations) in [
        ("ACTIVE", report.violations.as_slice()),
        ("WAIVED", report.waived_violations.as_slice()),
    ] {
        for violation in violations
            .iter()
            .filter(|violation| electrical_test_relevant_check(&violation.check))
        {
            let first_location = violation
                .locations
                .first()
                .map(|location| format!("{:.6},{:.6}", location[0], location[1]))
                .unwrap_or_else(|| "none".to_string());
            text.push_str(&format!(
                "C DIFF_FINDING STATUS={} ID={} GEOMETRY_HASH={} CHECK={} SEVERITY={:?} LAYERS={} POLYGONS={} LOCATIONS={} FIRST_LOCATION={} AREA_MM2={:.6}\n",
                status,
                sanitize(&violation.id),
                sanitize(&violation.id),
                sanitize(&violation.check),
                violation.severity,
                sanitize(&violation.layers.join(",")),
                violation.polygons.len(),
                violation.locations.len(),
                sanitize(&first_location),
                violation.total_area
            ));
        }
    }
    text.push_str("C END_HYPERDRC_IPC356_DIFF\n");
}

fn electrical_test_relevant_check(check: &str) -> bool {
    check.contains("ipc")
        || check.contains("drill")
        || check.contains("net")
        || check.contains("testpoint")
        || check.contains("plane-clearance")
        || check.contains("via")
}

fn sanitize(value: &str) -> String {
    value
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
        .join(" ")
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use crate::ipc356::parse_ipc356_report;
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_ipc356_review;

    #[test]
    fn ipc356_review_summarizes_sources_nets_and_relevant_findings() {
        let ipc = parse_ipc356_report(
            "317 /GND U1 1 X010000Y020000D000600 ACCESS=TOP MASK=OPEN\n327 /GND U2 2 X030000Y040000D000600 ACCESS=BOTTOM MASK=COVERED\n327 /VCC U3 3 X050000Y060000D000600\n",
            Path::new("fixture.ipc"),
        );
        let waived = Violation::new(
            "testpoint-accessibility-readiness",
            Severity::Warning,
            vec!["ipc356".to_string()],
            None,
            Vec::new(),
            vec![[3.0, 4.0]],
            Some("accepted fixture access limitation".to_string()),
        );
        let violations = vec![
            Violation::new(
                "drill-table-consistency",
                Severity::Warning,
                vec!["drill".to_string(), "ipc356".to_string()],
                None,
                Vec::new(),
                vec![[1.0, 2.0]],
                Some("drill diameter differs from IPC-D-356 evidence".to_string()),
            ),
            Violation::new(
                "silkscreen-clearance",
                Severity::Warning,
                vec!["silk".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some("not electrical-test relevant".to_string()),
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

        let text = report_to_ipc356_review(&report, &[ipc]);

        assert!(text.contains("C SOURCE fixture.ipc POINTS=3 UNIQUE_NETS=2"));
        assert!(text.contains("C NET GND POINTS=2 TOP=1 BOTTOM=1"));
        assert!(text.contains("C NET VCC POINTS=1"));
        assert!(text.contains("MISSING_ACCESS_SIDE=1"));
        assert!(text.contains("C FINDING"));
        assert!(text.contains("GEOMETRY_HASH="));
        assert!(text.contains("CHECK=drill-table-consistency"));
        assert!(!text.contains("silkscreen-clearance"));
        assert!(text.contains("C BEGIN_HYPERDRC_IPC356_DIFF FORMAT=key-value VERSION=1"));
        assert!(text.contains("C DIFF_SUMMARY ACTIVE_RELEVANT=1 WAIVED_RELEVANT=1"));
        assert!(text.contains("C DIFF_SOURCE SOURCE=fixture.ipc POINTS=3 NETS=2"));
        assert!(text.contains("C DIFF_FINDING STATUS=ACTIVE"));
        assert!(text.contains("C DIFF_FINDING STATUS=WAIVED"));
        assert!(text.contains("FIRST_LOCATION=1.000000,2.000000"));
    }
}
