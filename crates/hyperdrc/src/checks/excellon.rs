use crate::excellon::{
    ExcellonIssueKind, ExcellonPlatingIntent, ExcellonReport, infer_excellon_plating_intent,
};
use crate::report::{Severity, Violation};

/// Run the `excellon_readiness` design-readiness check or report helper.
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
    check_drill_diameter_outliers(report, &mut violations);

    log::trace!(
        "excellon readiness: source={} drills={} parser_issues={} violations={}",
        report.source,
        report.drills.len(),
        report.issues.len(),
        violations.len()
    );

    violations
}

/// Run the `excellon_batch_readiness` design-readiness check or report helper.
pub fn excellon_batch_readiness(reports: &[ExcellonReport]) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut expected_unit: Option<(String, crate::excellon::ExcellonUnits)> = None;
    let mut has_missing_unit_declaration = false;
    let mut drill_signatures =
        std::collections::BTreeMap::<(u8, Vec<(i64, i64, i64, bool)>), Vec<String>>::new();
    let mut plating_split_holes =
        std::collections::BTreeMap::<(i64, i64, i64), Vec<(DrillFilePlatingIntent, String)>>::new();

    for report in reports {
        violations.extend(excellon_readiness(report));

        if let Some(signature) = drill_geometry_signature(report) {
            drill_signatures
                .entry(signature)
                .or_default()
                .push(report.source.clone());
        }
        collect_plating_split_holes(report, &mut plating_split_holes);

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

    log::trace!(
        "excellon batch readiness: reports={} drill_signatures={} violations_before_duplicate_review={}",
        reports.len(),
        drill_signatures.len(),
        violations.len()
    );

    for sources in drill_signatures.values() {
        if sources.len() <= 1 {
            continue;
        }

        // IPC-NC-349 covers CNC formatting for drillers and routers. In a
        // release archive, two Excellon files with identical rounded geometry
        // usually indicate duplicate exported drill data, not two independent
        // manufacturing instructions. The check rounds to nanometer-scale
        // integer coordinates to keep harmless floating-point parser noise from
        // obscuring exact duplicates.
        violations.push(Violation::new(
            "excellon-readiness",
            Severity::Warning,
            vec!["excellon:duplicate-drill-geometry".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Excellon files appear to contain identical drill geometry; review duplicate drill exports: {}",
                sources.join(", ")
            )),
        ));
    }

    check_plating_split_conflicts(plating_split_holes, &mut violations);

    violations
}

fn drill_geometry_signature(report: &ExcellonReport) -> Option<(u8, Vec<(i64, i64, i64, bool)>)> {
    let unit_code = match report.declared_unit? {
        crate::excellon::ExcellonUnits::Metric => 1,
        crate::excellon::ExcellonUnits::Inch => 2,
    };
    if !report.has_units || report.drills.is_empty() {
        return None;
    }

    let mut signature = report
        .drills
        .iter()
        .map(|drill| {
            (
                rounded_microunits(drill.location[0]),
                rounded_microunits(drill.location[1]),
                rounded_microunits(drill.diameter),
                drill.plated,
            )
        })
        .collect::<Vec<_>>();
    signature.sort_unstable();
    Some((unit_code, signature))
}

fn rounded_microunits(value: f64) -> i64 {
    (value * 1_000_000.0).round() as i64
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum DrillFilePlatingIntent {
    Plated,
    NonPlated,
}

fn collect_plating_split_holes(
    report: &ExcellonReport,
    holes: &mut std::collections::BTreeMap<(i64, i64, i64), Vec<(DrillFilePlatingIntent, String)>>,
) {
    let Some(intent) = drill_file_plating_intent(&report.source) else {
        return;
    };

    for drill in &report.drills {
        if !(drill.location[0].is_finite()
            && drill.location[1].is_finite()
            && drill.diameter.is_finite()
            && drill.diameter > 0.0)
        {
            continue;
        }
        holes
            .entry((
                rounded_microunits(drill.location[0]),
                rounded_microunits(drill.location[1]),
                rounded_microunits(drill.diameter),
            ))
            .or_default()
            .push((intent, report.source.clone()));
    }
}

fn drill_file_plating_intent(source: &str) -> Option<DrillFilePlatingIntent> {
    match infer_excellon_plating_intent(std::path::Path::new(source))? {
        ExcellonPlatingIntent::Plated => Some(DrillFilePlatingIntent::Plated),
        ExcellonPlatingIntent::NonPlated => Some(DrillFilePlatingIntent::NonPlated),
    }
}

fn check_plating_split_conflicts(
    holes: std::collections::BTreeMap<(i64, i64, i64), Vec<(DrillFilePlatingIntent, String)>>,
    violations: &mut Vec<Violation>,
) {
    let conflicts = holes
        .into_iter()
        .filter_map(|((x, y, diameter), entries)| {
            let has_plated = entries
                .iter()
                .any(|(intent, _)| *intent == DrillFilePlatingIntent::Plated);
            let has_non_plated = entries
                .iter()
                .any(|(intent, _)| *intent == DrillFilePlatingIntent::NonPlated);
            if !(has_plated && has_non_plated) {
                return None;
            }
            let mut sources = entries
                .into_iter()
                .map(|(_, source)| source)
                .collect::<Vec<_>>();
            sources.sort();
            sources.dedup();
            Some(format!(
                "({:.6},{:.6}) diameter {:.6}: {}",
                x as f64 / 1_000_000.0,
                y as f64 / 1_000_000.0,
                diameter as f64 / 1_000_000.0,
                sources.join(", ")
            ))
        })
        .collect::<Vec<_>>();

    if conflicts.is_empty() {
        return;
    }

    // IPC-6012D distinguishes plated and unsupported holes in fabrication
    // requirements, while IPC-NC-349 carries the drill/rout transfer data.
    // If filename intent says the same rounded hole is both PTH and NPTH, the
    // manufacturing release needs review before downstream clearance checks
    // consume the sidecar holes as generic mechanical drills.
    violations.push(Violation::new(
        "excellon-readiness",
        Severity::Warning,
        vec!["excellon:plating-split-conflict".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Excellon plated/non-plated drill files contain overlapping hole definitions; review drill split: {}",
            conflicts.join("; ")
        )),
    ));
}

fn check_drill_diameter_outliers(report: &ExcellonReport, violations: &mut Vec<Violation>) {
    if report.drills.len() < 4 {
        return;
    }

    let mut diameters = report
        .drills
        .iter()
        .map(|drill| drill.diameter)
        .filter(|diameter| diameter.is_finite() && *diameter > 0.0)
        .collect::<Vec<_>>();
    if diameters.len() < 4 {
        return;
    }
    diameters.sort_by(f64::total_cmp);
    let median = diameters[diameters.len() / 2];
    if median <= 0.0 || !median.is_finite() {
        return;
    }

    let outliers = diameters
        .into_iter()
        .filter(|diameter| {
            let tiny_outlier = *diameter <= 0.075 && *diameter * 8.0 < median;
            let large_outlier = *diameter >= 6.0 && *diameter > median * 8.0;
            tiny_outlier || large_outlier
        })
        .map(|diameter| format!("{diameter:.6}"))
        .collect::<std::collections::BTreeSet<_>>();
    if outliers.is_empty() {
        return;
    }

    // IPC-2221B and IPC-NC-349 both put drill dimensions in the manufacturing
    // contract path: one as printed-board design guidance, the other as CNC
    // drill/rout transfer data. Diameter values that are both absolute and
    // median-relative outliers are therefore treated as CAM-review evidence for
    // unit mistakes, wrong drill tables, or accidental mixed drill exports.
    violations.push(Violation::new(
        "excellon-readiness",
        Severity::Warning,
        vec!["excellon:diameter-outlier".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Excellon file {} contains drill diameter outlier(s) relative to median {:.6}: {}",
            report.source,
            median,
            outliers.into_iter().collect::<Vec<_>>().join(", ")
        )),
    ));
}

/// Run the `excellon_issue_severity` design-readiness check or report helper.
pub fn excellon_issue_severity(kind: &ExcellonIssueKind) -> Severity {
    match kind {
        ExcellonIssueKind::MissingUnitDeclaration
        | ExcellonIssueKind::UnitConflict { .. }
        | ExcellonIssueKind::ZeroSuppressionDeclaration { .. }
        | ExcellonIssueKind::UnsupportedUnitDeclaration { .. }
        | ExcellonIssueKind::ToolRedefinition { .. }
        | ExcellonIssueKind::ToolDiameterNotPositive { .. }
        | ExcellonIssueKind::UnknownToolSelection { .. }
        | ExcellonIssueKind::DrillHitWithoutActiveTool
        | ExcellonIssueKind::DrillHitWithUnknownTool { .. }
        | ExcellonIssueKind::DrillHitWithoutDiameter { .. }
        | ExcellonIssueKind::InvalidToolDefinition { .. }
        | ExcellonIssueKind::InvalidCoordinate { .. }
        | ExcellonIssueKind::RoutedSlotCommand { .. } => Severity::Warning,
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
    fn zero_suppression_declarations_are_reported() {
        let report = report_with(ExcellonIssueKind::ZeroSuppressionDeclaration {
            mode: "TZ".to_string(),
        });
        let violations = excellon_readiness(&report);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("zero-suppression"))
        }));
    }

    #[test]
    fn unsupported_unit_declarations_are_reported() {
        let report = report_with(ExcellonIssueKind::UnsupportedUnitDeclaration {
            token: "MILS".to_string(),
        });
        let violations = excellon_readiness(&report);

        assert_eq!(violations[0].severity, crate::report::Severity::Warning);
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("unsupported Excellon unit"))
        }));
    }

    #[test]
    fn routed_slot_commands_are_reported() {
        let report = report_with(ExcellonIssueKind::RoutedSlotCommand {
            command: "G85".to_string(),
        });
        let violations = excellon_readiness(&report);

        assert_eq!(violations[0].severity, crate::report::Severity::Warning);
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("routing command"))
        }));
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
        };

        let violations = excellon_readiness(&report);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn drill_diameter_outliers_are_reported_without_flagging_tooling_holes() {
        let drill = |diameter| crate::kicad::DrillFeature {
            location: [diameter, 0.0],
            diameter,
            net: None,
            plated: false,
        };
        let outlier_report = ExcellonReport {
            source: "mixed-table.drl".to_string(),
            drills: vec![
                drill(0.30),
                drill(0.30),
                drill(0.35),
                drill(0.40),
                drill(12.70),
            ],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Metric),
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
        };
        let tooling_report = ExcellonReport {
            source: "tooling.drl".to_string(),
            drills: vec![
                drill(0.30),
                drill(0.30),
                drill(0.35),
                drill(0.40),
                drill(3.20),
            ],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Metric),
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
        };

        let outlier_violations = excellon_readiness(&outlier_report);
        let tooling_violations = excellon_readiness(&tooling_report);

        assert!(outlier_violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"excellon:diameter-outlier".to_string())
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("12.700000"))
        }));
        assert!(tooling_violations.is_empty(), "{tooling_violations:#?}");
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
    fn excellon_batch_readiness_reports_duplicate_drill_geometry() {
        let first = clean_report("pth.drl", 2, ExcellonUnits::Metric);
        let duplicate = clean_report("pth-copy.drl", 2, ExcellonUnits::Metric);
        let mut npth = clean_report("npth.drl", 2, ExcellonUnits::Metric);
        npth.drills[0].diameter = 0.9;

        let violations = excellon_batch_readiness(&[first, duplicate, npth]);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"excellon:duplicate-drill-geometry".to_string())
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("pth.drl, pth-copy.drl"))
        }));
    }

    #[test]
    fn excellon_batch_readiness_reports_plated_nonplated_split_conflicts() {
        let mut plated = clean_report("widget-PTH.drl", 2, ExcellonUnits::Metric);
        plated.drills[0].location = [1.0, 2.0];
        plated.drills[0].diameter = 0.6;
        plated.drills[1].location = [3.0, 4.0];
        plated.drills[1].diameter = 0.8;
        let mut non_plated = clean_report("widget-NPTH.drl", 2, ExcellonUnits::Metric);
        non_plated.drills[0].location = [1.0, 2.0];
        non_plated.drills[0].diameter = 0.6;
        non_plated.drills[1].location = [8.0, 9.0];
        non_plated.drills[1].diameter = 3.2;

        let violations = excellon_batch_readiness(&[plated, non_plated]);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"excellon:plating-split-conflict".to_string())
                && violation.message.as_deref().is_some_and(|message| {
                    message.contains("widget-PTH.drl")
                        && message.contains("widget-NPTH.drl")
                        && message.contains("diameter 0.600000")
                })
        }));
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
        };
        let inch = ExcellonReport {
            source: "inch.drl".to_string(),
            drills: vec![drill],
            issues: Vec::new(),
            has_units: true,
            declared_unit: Some(ExcellonUnits::Inch),
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
            unit_summary: Default::default(),
            program: Default::default(),
            tool_table: Default::default(),
            routing: Default::default(),
            hits: Default::default(),
            drill_summary: Default::default(),
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
