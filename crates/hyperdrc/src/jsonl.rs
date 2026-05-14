//! JSON Lines report sink for append-friendly CI and manufacturing analytics.
//!
//! The normal JSON report is best for humans and one-shot integrations. JSON
//! Lines keeps one self-contained object per line, which makes it easier to
//! append many board runs into a log, stream through command-line tools, or load
//! into data warehouses without holding a full corpus in memory.

use anyhow::Result;
use serde_json::{Value, json};

use crate::report::{Report, Violation};

/// Run the `report_to_jsonl` design-readiness check or report helper.
pub fn report_to_jsonl(report: &Report) -> Result<String> {
    let mut lines = Vec::with_capacity(
        1 + report.inputs.len() + report.diagnostics.len() + report.violations.len(),
    );
    lines.push(serde_json::to_string(&run_record(report))?);
    for input in &report.inputs {
        lines.push(serde_json::to_string(&json!({
            "kind": "input",
            "input": input,
        }))?);
    }
    for diagnostic in &report.diagnostics {
        lines.push(serde_json::to_string(&json!({
            "kind": "diagnostic",
            "diagnostic": diagnostic,
        }))?);
    }
    for violation in &report.violations {
        lines.push(serde_json::to_string(&violation_record(violation))?);
    }

    Ok(format!("{}\n", lines.join("\n")))
}

fn run_record(report: &Report) -> Value {
    json!({
        "kind": "run",
        "files": report.files,
        "violation_count": report.violation_count,
        "waived_count": report.waived_count,
        "summary": report.summary,
    })
}

fn violation_record(violation: &Violation) -> Value {
    json!({
        "kind": "violation",
        "violation": violation,
    })
}

#[cfg(test)]
mod tests {
    use serde_json::Value;

    use crate::io::{IoAdapter, IoRole, SourceRecord};
    use crate::report::{Diagnostic, Report, Severity, Violation, report_summary};

    use super::report_to_jsonl;

    #[test]
    fn jsonl_report_emits_run_input_and_violation_records() {
        let violations = vec![Violation::new(
            "board-edge-clearance",
            Severity::Error,
            vec!["top.gbr".to_string()],
            None,
            Vec::new(),
            vec![[4.0, 5.0]],
            Some("copper is too close to outline".to_string()),
        )];
        let report = Report {
            files: vec!["top.gbr".to_string()],
            inputs: vec![SourceRecord::new(
                IoAdapter::DirectFile,
                IoRole::GerberLayer,
                "top.gbr",
                Option::<&str>::None,
            )],
            diagnostics: vec![Diagnostic {
                source: "top.gbr".to_string(),
                line: Some(7),
                severity: Severity::Warning,
                code: "parser::sample".to_string(),
                message: "sample parser warning".to_string(),
            }],
            violation_count: violations.len(),
            waived_count: 0,
            summary: report_summary(&violations, 0),
            violations,
        };

        let jsonl = report_to_jsonl(&report).unwrap();
        let records = jsonl
            .lines()
            .map(|line| serde_json::from_str::<Value>(line).unwrap())
            .collect::<Vec<_>>();

        assert_eq!(records.len(), 4);
        assert_eq!(records[0]["kind"], "run");
        assert_eq!(records[1]["kind"], "input");
        assert_eq!(records[2]["kind"], "diagnostic");
        assert_eq!(records[2]["diagnostic"]["line"], 7);
        assert_eq!(records[3]["kind"], "violation");
        assert_eq!(records[3]["violation"]["locations"][0][0], 4.0);
    }
}
