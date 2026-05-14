//! SARIF output adapter for CI and code review systems.
//!
//! SARIF v2.1.0 is an OASIS standard for static-analysis interchange. hyperdrc
//! findings are geometric rather than line-based, so the adapter keeps
//! URI-addressable artifacts and stores PCB coordinates in SARIF `properties`
//! instead of inventing source line numbers.

use std::collections::BTreeSet;

use serde_json::{Value, json};

use crate::report::{Report, Severity, Violation};

const SARIF_SCHEMA: &str = "https://json.schemastore.org/sarif-2.1.0.json";
const SARIF_VERSION: &str = "2.1.0";

/// Run the `report_to_sarif` design-readiness check or report helper.
pub fn report_to_sarif(report: &Report) -> Value {
    json!({
        "$schema": SARIF_SCHEMA,
        "version": SARIF_VERSION,
        "runs": [{
            "tool": {
                "driver": {
                    "name": "hyperdrc",
                    "informationUri": "https://github.com/timschmidt/hyperdrc",
                    "rules": rules(report),
                }
            },
            "invocations": [{
                "executionSuccessful": true,
                "properties": {
                    "violationCount": report.violation_count,
                    "waivedCount": report.waived_count,
                    "diagnosticCount": report.diagnostics.len(),
                    "errorCount": report.summary.errors,
                    "warningCount": report.summary.warnings,
                }
            }],
            "artifacts": artifacts(report),
            "results": results(report),
        }]
    })
}

fn rules(report: &Report) -> Vec<Value> {
    let mut checks = report
        .violations
        .iter()
        .map(|violation| violation.check.as_str())
        .collect::<BTreeSet<_>>();
    checks.extend(
        report
            .summary
            .checks
            .iter()
            .map(|summary| summary.check.as_str()),
    );

    checks
        .into_iter()
        .map(|check| {
            json!({
                "id": check,
                "name": check,
                "shortDescription": {
                    "text": format!("hyperdrc {check} finding")
                },
                "properties": {
                    "precision": "medium",
                    "tags": ["pcb", "drc", "dfm", "design-readiness"],
                }
            })
        })
        .collect()
}

fn artifacts(report: &Report) -> Vec<Value> {
    let mut uris = report
        .files
        .iter()
        .map(String::as_str)
        .collect::<BTreeSet<_>>();
    uris.extend(report.inputs.iter().map(|input| input.path.as_str()));
    for violation in &report.violations {
        uris.extend(violation.layers.iter().map(String::as_str));
    }

    uris.into_iter()
        .filter(|uri| !uri.is_empty())
        .map(|uri| json!({ "location": { "uri": uri } }))
        .collect()
}

fn results(report: &Report) -> Vec<Value> {
    report.violations.iter().map(result).collect()
}

fn result(violation: &Violation) -> Value {
    json!({
        "ruleId": violation.check,
        "level": sarif_level(violation.severity),
        "message": {
            "text": violation_message(violation),
        },
        "locations": [{
            "physicalLocation": {
                "artifactLocation": {
                    "uri": primary_uri(violation),
                }
            },
            "logicalLocations": violation.layers.iter().map(|layer| {
                json!({
                    "name": layer,
                    "kind": "layer",
                })
            }).collect::<Vec<_>>(),
        }],
        "partialFingerprints": {
            "hyperdrcStableId": violation.id,
        },
        "properties": {
            "hyperdrcId": violation.id,
            "layers": violation.layers,
            "islandIndex": violation.island_index,
            "totalArea": violation.total_area,
            "locations": violation.locations,
            "polygons": violation.polygons,
        }
    })
}

fn sarif_level(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    }
}

fn violation_message(violation: &Violation) -> String {
    violation
        .message
        .clone()
        .unwrap_or_else(|| format!("{} on {}", violation.check, violation.layers.join(" + ")))
}

fn primary_uri(violation: &Violation) -> &str {
    violation
        .layers
        .first()
        .map(String::as_str)
        .unwrap_or("hyperdrc")
}

#[cfg(test)]
mod tests {
    use crate::io::{IoAdapter, IoRole, SourceRecord};
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_sarif;

    #[test]
    fn sarif_report_contains_rules_results_and_geometry_properties() {
        let violations = vec![
            Violation::new(
                "minimum-copper-neck-width",
                Severity::Error,
                vec!["top.gbr".to_string()],
                Some(2),
                Vec::new(),
                vec![[1.2, 3.4]],
                Some("neck width is below rule".to_string()),
            ),
            Violation::new(
                "file-manifest-readiness",
                Severity::Warning,
                vec!["package:missing-bom".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                None,
            ),
        ];
        let report = Report {
            files: vec!["top.gbr".to_string()],
            inputs: vec![SourceRecord::new(
                IoAdapter::DirectFile,
                IoRole::GerberLayer,
                "top.gbr",
                Option::<&str>::None,
            )],
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 1,
            summary: report_summary(&violations, 1),
            violations,
        };

        let sarif = report_to_sarif(&report);

        assert_eq!(sarif["version"], "2.1.0");
        assert_eq!(sarif["runs"][0]["tool"]["driver"]["name"], "hyperdrc");
        assert_eq!(sarif["runs"][0]["results"][0]["level"], "error");
        assert_eq!(
            sarif["runs"][0]["results"][0]["partialFingerprints"]["hyperdrcStableId"],
            report.violations[0].id
        );
        assert_eq!(
            sarif["runs"][0]["results"][0]["properties"]["locations"][0][0],
            1.2
        );
        assert!(
            sarif["runs"][0]["tool"]["driver"]["rules"]
                .as_array()
                .unwrap()
                .iter()
                .any(|rule| rule["id"] == "file-manifest-readiness")
        );
    }
}
