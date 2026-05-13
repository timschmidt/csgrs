use crate::excellon::{ExcellonIssueKind, ExcellonReport};
use crate::report::{Severity, Violation};

pub fn excellon_readiness(report: &ExcellonReport) -> Vec<Violation> {
    let mut violations = Vec::new();
    let layer = format!("excellon:{}", report.source);

    for issue in &report.issues {
        violations.push(Violation::new(
            "excellon-readiness",
            excellon_issue_severity(&issue.kind),
            vec![layer.clone()],
            None,
            Vec::new(),
            Vec::new(),
            Some(issue.message()),
        ));
    }

    if report.drills.is_empty() {
        violations.push(Violation::new(
            "excellon-readiness",
            Severity::Warning,
            vec![layer],
            None,
            Vec::new(),
            Vec::new(),
            Some("no drill hits were parsed from this Excellon file".to_string()),
        ));
    }

    violations
}

pub fn excellon_batch_readiness(reports: &[ExcellonReport]) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut expected_unit: Option<(String, crate::excellon::ExcellonUnits)> = None;
    let mut has_missing_unit_declaration = false;

    for report in reports {
        violations.extend(excellon_readiness(report));

        if report
            .issues
            .iter()
            .any(|issue| matches!(issue.kind, ExcellonIssueKind::MissingUnitDeclaration))
        {
            has_missing_unit_declaration = true;
        }

        if let Some(unit) = report.declared_unit {
            if let Some((source, expected)) = &expected_unit {
                if expected != &unit {
                    violations.push(Violation::new(
                        "excellon-readiness",
                        Severity::Warning,
                        vec!["excellon:unit-consistency".to_string()],
                        None,
                        Vec::new(),
                        Vec::new(),
                        Some(format!(
                            "mixed Excellon unit declarations detected: {} used {:?}, {} used {:?}",
                            source, expected, report.source, unit
                        )),
                    ));
                }
            } else {
                expected_unit = Some((report.source.clone(), unit));
            }
        }
    }

    if expected_unit.is_some() && has_missing_unit_declaration {
        let (source, unit) = expected_unit.unwrap();
        violations.push(Violation::new(
            "excellon-readiness",
            Severity::Warning,
            vec!["excellon:unit-consistency".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Excellon files declare unit {unit:?} from {source} while others are missing unit declarations",
            )),
        ));
    }

    violations
}

pub fn excellon_issue_severity(kind: &ExcellonIssueKind) -> Severity {
    match kind {
        ExcellonIssueKind::MissingUnitDeclaration
        | ExcellonIssueKind::UnitConflict { .. }
        | ExcellonIssueKind::ToolRedefinition { .. }
        | ExcellonIssueKind::ToolDiameterNotPositive { .. }
        | ExcellonIssueKind::UnknownToolSelection { .. }
        | ExcellonIssueKind::DrillHitWithoutActiveTool
        | ExcellonIssueKind::DrillHitWithUnknownTool { .. }
        | ExcellonIssueKind::DrillHitWithoutDiameter { .. }
        | ExcellonIssueKind::InvalidToolDefinition { .. }
        | ExcellonIssueKind::InvalidCoordinate { .. } => Severity::Warning,
        ExcellonIssueKind::DuplicateToolDefinition { .. } => Severity::Warning,
    }
}

#[cfg(test)]
mod tests {
    use super::{excellon_batch_readiness, excellon_readiness};
    use crate::excellon::ExcellonUnits;
    use crate::excellon::{ExcellonIssue, ExcellonIssueKind, ExcellonReport};

    fn clean_report(source: &str, drills: usize, unit: ExcellonUnits) -> ExcellonReport {
        let drill_template = crate::kicad::DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.6,
            net: None,
            plated: false,
        };
        let drill_features = vec![drill_template; drills];

        ExcellonReport {
            source: source.to_string(),
            drills: drill_features,
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(unit),
        }
    }

    fn no_unit_report(source: &str, drills: usize, include_missing_issue: bool) -> ExcellonReport {
        let drill_template = crate::kicad::DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.6,
            net: None,
            plated: false,
        };
        let drill_features = vec![drill_template; drills];
        let issues = if include_missing_issue {
            vec![ExcellonIssue {
                line: 1,
                kind: ExcellonIssueKind::MissingUnitDeclaration,
                detail: source.to_string(),
            }]
        } else {
            Vec::new()
        };

        ExcellonReport {
            source: source.to_string(),
            drills: drill_features,
            issues,
            has_units: false,
            declared_unit: None,
        }
    }

    fn report_with(kind: ExcellonIssueKind) -> ExcellonReport {
        ExcellonReport {
            source: "panel.drl".to_string(),
            drills: Vec::new(),
            issues: vec![ExcellonIssue {
                line: 1,
                kind,
                detail: "line 1".to_string(),
            }],
            has_units: true,
            declared_unit: Some(crate::excellon::ExcellonUnits::Metric),
        }
    }

    #[test]
    fn missing_units_and_redefinition_issues_are_reported() {
        let report = report_with(ExcellonIssueKind::MissingUnitDeclaration);
        let violations = excellon_readiness(&report);

        assert_eq!(violations.len(), 2);
        assert_eq!(violations[0].check, "excellon-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("unit declaration"))
        );
    }

    #[test]
    fn no_drills_reports_an_empty_panel_warning() {
        let report = no_unit_report("empty.drl", 0, false);
        let mut report = report;
        report.has_units = true;
        report.declared_unit = Some(ExcellonUnits::Metric);
        let violations = excellon_readiness(&report);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("no drill hits")
        );
    }

    #[test]
    fn valid_hits_and_duplicate_tool_definition_skip_empty_hits_warning() {
        let report = ExcellonReport {
            source: "ok.drl".to_string(),
            drills: vec![crate::kicad::DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: None,
                plated: false,
            }],
            issues: vec![ExcellonIssue {
                line: 3,
                kind: ExcellonIssueKind::DuplicateToolDefinition {
                    tool: "T01".to_string(),
                    diameter: 0.6,
                },
                detail: "T01".to_string(),
            }],
            has_units: true,
            declared_unit: Some(ExcellonUnits::Metric),
        };

        let violations = excellon_readiness(&report);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn excellon_readiness_is_clean_for_declared_unit_reports_with_drills() {
        let report = clean_report("good.drl", 1, ExcellonUnits::Metric);
        let violations = excellon_readiness(&report);

        assert!(violations.is_empty());
    }

    #[test]
    fn excellon_batch_readiness_allows_consistent_units_and_no_warnings() {
        let first = clean_report("first.drl", 1, ExcellonUnits::Metric);
        let second = clean_report("second.drl", 2, ExcellonUnits::Metric);

        assert!(excellon_batch_readiness(&[first, second]).is_empty());
    }

    #[test]
    fn excellon_batch_readiness_has_no_unit_consistency_warning_when_all_units_missing() {
        let first = no_unit_report("first.drl", 1, true);
        let second = no_unit_report("second.drl", 1, true);
        let third = no_unit_report("third.drl", 1, true);

        let violations = excellon_batch_readiness(&[first, second, third]);

        assert_eq!(violations.len(), 3);
        assert!(!violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("mixed Excellon unit declarations"))
        }));
        assert!(violations.iter().all(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("unit declaration"))
        }));
    }

    #[test]
    fn batch_readiness_flags_mixed_unit_declarations() {
        let drill = crate::kicad::DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.6,
            net: None,
            plated: false,
        };
        let metric = ExcellonReport {
            source: "metric.drl".to_string(),
            drills: vec![drill.clone()],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Metric),
        };
        let inch = ExcellonReport {
            source: "inch.drl".to_string(),
            drills: vec![drill],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Inch),
        };

        let violations = excellon_batch_readiness(&[metric, inch]);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("mixed Excellon unit declarations"))
        );
    }

    #[test]
    fn batch_readiness_warns_when_unit_declarations_are_inconsistent_across_files() {
        let drill = crate::kicad::DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.6,
            net: None,
            plated: false,
        };
        let metric = ExcellonReport {
            source: "metric.drl".to_string(),
            drills: vec![drill.clone()],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Metric),
        };
        let missing = ExcellonReport {
            source: "missing.drl".to_string(),
            drills: vec![drill],
            issues: vec![ExcellonIssue {
                line: 1,
                kind: ExcellonIssueKind::MissingUnitDeclaration,
                detail: "missing.drl".to_string(),
            }],
            has_units: false,
            declared_unit: None,
        };

        let violations = excellon_batch_readiness(&[metric, missing]);

        assert!(
            violations
                .iter()
                .any(
                    |violation| violation.message.as_deref().is_some_and(|message| {
                        message.contains("while others are missing unit declarations")
                    })
                )
        );
    }

    #[test]
    fn batch_readiness_does_not_warn_about_mixed_missing_units_without_any_declared_reference() {
        let missing_a = ExcellonReport {
            source: "missing-a.drl".to_string(),
            drills: vec![crate::kicad::DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: None,
                plated: false,
            }],
            issues: vec![ExcellonIssue {
                line: 1,
                kind: ExcellonIssueKind::MissingUnitDeclaration,
                detail: "missing-a.drl".to_string(),
            }],
            has_units: false,
            declared_unit: None,
        };
        let missing_b = ExcellonReport {
            source: "missing-b.drl".to_string(),
            drills: vec![crate::kicad::DrillFeature {
                location: [1.0, 1.0],
                diameter: 0.8,
                net: None,
                plated: false,
            }],
            issues: vec![ExcellonIssue {
                line: 1,
                kind: ExcellonIssueKind::MissingUnitDeclaration,
                detail: "missing-b.drl".to_string(),
            }],
            has_units: false,
            declared_unit: None,
        };

        let violations = excellon_batch_readiness(&[missing_a, missing_b]);

        assert_eq!(violations.len(), 2);
        assert!(
            violations.iter().any(|violation| violation
                .message
                .as_deref()
                .is_some_and(|message| { message.contains("unit declaration") })),
            "expected missing-unit warnings for each file"
        );
        assert!(violations.iter().all(|violation| {
            !violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("missing unit declarations"))
        }));
    }
}
