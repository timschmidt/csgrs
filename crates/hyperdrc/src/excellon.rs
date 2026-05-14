//! Excellon drill parser with non-fatal parser diagnostics.
//!
//! The parser accepts common Excellon unit declarations, tool definitions, and
//! hit records, returning drill geometry plus issues that can be surfaced as
//! readiness findings without aborting the rest of a package run.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::kicad::DrillFeature;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Public enumeration for `ExcellonUnits`.
pub enum ExcellonUnits {
    /// Variant `Metric`.
    Metric,
    /// Variant `Inch`.
    Inch,
}

#[derive(Clone, Debug)]
/// Public data model for `ExcellonIssue`.
pub struct ExcellonIssue {
    /// Field `line`.
    pub line: usize,
    /// Field `kind`.
    pub kind: ExcellonIssueKind,
    /// Field `detail`.
    pub detail: String,
}

#[derive(Clone, Debug)]
/// Public enumeration for `ExcellonIssueKind`.
pub enum ExcellonIssueKind {
    /// Variant `MissingUnitDeclaration`.
    MissingUnitDeclaration,
    /// Conflicting unit declarations appeared in one file.
    UnitConflict {
        /// Previously declared units.
        existing: ExcellonUnits,
        /// Later conflicting units.
        incoming: ExcellonUnits,
    },
    /// A tool definition could not be parsed.
    InvalidToolDefinition {
        /// Tool identifier.
        tool: String,
        /// Raw diameter token that failed parsing.
        raw_diameter: String,
    },
    /// A tool defined a zero or negative diameter.
    ToolDiameterNotPositive {
        /// Tool identifier.
        tool: String,
        /// Parsed diameter.
        diameter: f64,
    },
    /// A tool was defined more than once with the same diameter.
    DuplicateToolDefinition {
        /// Tool identifier.
        tool: String,
        /// Repeated diameter.
        diameter: f64,
    },
    /// A tool was redefined with a different diameter.
    ToolRedefinition {
        /// Tool identifier.
        tool: String,
        /// First diameter definition.
        previous: f64,
        /// Later diameter definition.
        replacement: f64,
    },
    /// A selected tool had no definition.
    UnknownToolSelection {
        /// Tool identifier.
        tool: String,
    },
    /// Variant `DrillHitWithoutActiveTool`.
    DrillHitWithoutActiveTool,
    /// A coordinate hit referenced an undefined tool.
    DrillHitWithUnknownTool {
        /// Tool identifier.
        tool: String,
    },
    /// A coordinate hit used a tool whose diameter was invalid.
    DrillHitWithoutDiameter {
        /// Tool identifier.
        tool: String,
    },
    /// A coordinate record could not be parsed.
    InvalidCoordinate {
        /// Raw line text.
        raw_line: String,
        /// Parse failure reason.
        reason: String,
    },
}

#[derive(Clone, Debug)]
/// Public data model for `ExcellonReport`.
pub struct ExcellonReport {
    /// Field `source`.
    pub source: String,
    /// Field `drills`.
    pub drills: Vec<DrillFeature>,
    /// Field `issues`.
    pub issues: Vec<ExcellonIssue>,
    /// Field `has_units`.
    pub has_units: bool,
    /// Field `declared_unit`.
    pub declared_unit: Option<ExcellonUnits>,
}

impl ExcellonIssueKind {
    fn message(&self, line: usize) -> String {
        match self {
            Self::MissingUnitDeclaration => {
                format!("line {line}: no unit declaration was provided before parsed geometry")
            }
            Self::UnitConflict { existing, incoming } => {
                format!(
                    "line {line}: unit declaration changed from {existing:?} to {incoming:?} mid-file"
                )
            }
            Self::InvalidToolDefinition { tool, raw_diameter } => {
                format!("line {line}: tool {tool} has invalid diameter token {raw_diameter:?}")
            }
            Self::ToolDiameterNotPositive { tool, diameter } => {
                format!("line {line}: tool {tool} defines non-positive diameter {diameter}")
            }
            Self::DuplicateToolDefinition { tool, diameter } => {
                format!("line {line}: tool {tool} is redefined with the same diameter {diameter}")
            }
            Self::ToolRedefinition {
                tool,
                previous,
                replacement,
            } => format!("line {line}: tool {tool} is redefined from {previous} to {replacement}"),
            Self::UnknownToolSelection { tool } => {
                format!("line {line}: selected tool {tool} was never defined")
            }
            Self::DrillHitWithoutActiveTool => {
                format!("line {line}: drill hit encountered before any tool selection")
            }
            Self::DrillHitWithUnknownTool { tool } => {
                format!("line {line}: drill hit uses tool {tool} which is undefined")
            }
            Self::DrillHitWithoutDiameter { tool } => {
                format!(
                    "line {line}: tool {tool} has a non-positive diameter and produced no drill"
                )
            }
            Self::InvalidCoordinate { raw_line, reason } => {
                format!("line {line}: cannot parse drill coordinate in {raw_line:?}: {reason}")
            }
        }
    }
}

impl ExcellonIssue {
    /// Run or compute `message`.
    pub fn message(&self) -> String {
        self.kind.message(self.line)
    }
}

/// Run or compute `load_excellon_report`.
pub fn load_excellon_report(path: &Path) -> Result<ExcellonReport> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(parse_excellon_report(&text, path))
}

/// Run or compute `load_excellon`.
pub fn load_excellon(path: &Path) -> Result<Vec<DrillFeature>> {
    Ok(load_excellon_report(path)?.drills)
}

/// Run or compute `parse_excellon`.
pub fn parse_excellon(input: &str) -> Vec<DrillFeature> {
    parse_excellon_report(input, &PathBuf::from("<inline-excellon>")).drills
}

/// Run or compute `parse_excellon_report`.
pub fn parse_excellon_report(input: &str, source: &Path) -> ExcellonReport {
    let mut tool_diameter = HashMap::<String, f64>::new();
    let mut current_tool: Option<String> = None;
    let mut drills = Vec::new();
    let mut issues = Vec::new();
    let mut has_units = false;
    let mut units_scale = 1.0;
    let mut declared_unit = None;

    for (index, raw_line) in input.lines().enumerate() {
        let line_number = index + 1;
        let stripped = raw_line.trim();
        if stripped.is_empty() || stripped.starts_with(';') {
            continue;
        }

        let normalized = stripped.split(';').next().unwrap_or_default().trim();
        if normalized.is_empty() {
            continue;
        }

        if matches!(normalized, "M30" | "M47" | "M95" | "%") {
            continue;
        }

        if let Some(units) = parse_units(normalized) {
            let incoming_scale = match units {
                ExcellonUnits::Metric => 1.0,
                ExcellonUnits::Inch => 25.4,
            };
            if let Some(existing) = declared_unit {
                if existing != units {
                    issues.push(ExcellonIssue {
                        line: line_number,
                        kind: ExcellonIssueKind::UnitConflict {
                            existing,
                            incoming: units,
                        },
                        detail: normalized.to_string(),
                    });
                }
            } else {
                has_units = true;
                declared_unit = Some(units);
            }
            units_scale = incoming_scale;
            continue;
        }

        if let Some((tool, diameter)) = parse_tool_definition(normalized, units_scale) {
            let raw_tool = tool.clone();
            match tool_diameter.get(&tool) {
                Some(previous) if *previous == diameter => {
                    issues.push(ExcellonIssue {
                        line: line_number,
                        kind: ExcellonIssueKind::DuplicateToolDefinition { tool, diameter },
                        detail: normalized.to_string(),
                    });
                }
                Some(previous) => {
                    issues.push(ExcellonIssue {
                        line: line_number,
                        kind: ExcellonIssueKind::ToolRedefinition {
                            tool: tool.clone(),
                            previous: *previous,
                            replacement: diameter,
                        },
                        detail: normalized.to_string(),
                    });
                }
                None => {
                    if diameter <= 0.0 {
                        issues.push(ExcellonIssue {
                            line: line_number,
                            kind: ExcellonIssueKind::ToolDiameterNotPositive {
                                tool: tool.clone(),
                                diameter,
                            },
                            detail: normalized.to_string(),
                        });
                    }
                }
            }

            // Keep the first definition to keep downstream geometry deterministic. Any
            // redefinition is surfaced as a readiness issue instead.
            tool_diameter.entry(raw_tool).or_insert(diameter);
            continue;
        }

        if let Some((line_tool, has_coordinate)) = parse_tool_with_optional_hit(normalized) {
            if has_coordinate {
                match parse_coordinate(normalized) {
                    Some((x, y)) => {
                        let Some(diameter) = tool_diameter.get(&line_tool) else {
                            issues.push(ExcellonIssue {
                                line: line_number,
                                kind: ExcellonIssueKind::DrillHitWithUnknownTool {
                                    tool: line_tool.clone(),
                                },
                                detail: normalized.to_string(),
                            });
                            current_tool = Some(line_tool);
                            continue;
                        };

                        if *diameter <= 0.0 {
                            issues.push(ExcellonIssue {
                                line: line_number,
                                kind: ExcellonIssueKind::DrillHitWithoutDiameter {
                                    tool: line_tool.clone(),
                                },
                                detail: normalized.to_string(),
                            });
                            current_tool = Some(line_tool);
                            continue;
                        }

                        current_tool = Some(line_tool.clone());
                        drills.push(DrillFeature {
                            location: [x * units_scale, y * units_scale],
                            diameter: *diameter,
                            net: None,
                            plated: false,
                        });
                        continue;
                    }
                    None => {
                        issues.push(ExcellonIssue {
                            line: line_number,
                            kind: ExcellonIssueKind::InvalidCoordinate {
                                raw_line: normalized.to_string(),
                                reason: "tool hit is missing valid X/Y coordinates".to_string(),
                            },
                            detail: normalized.to_string(),
                        });
                        current_tool = Some(line_tool);
                        continue;
                    }
                }
            }

            if let Some(previous_tool) = current_tool.clone() {
                if previous_tool == line_tool {
                    continue;
                }
            }

            if !tool_diameter.contains_key(&line_tool) {
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::UnknownToolSelection {
                        tool: line_tool.clone(),
                    },
                    detail: normalized.to_string(),
                });
            }
            current_tool = Some(line_tool);
            continue;
        }

        if let Some((x, y)) = parse_coordinate(normalized) {
            let Some(tool) = current_tool.clone() else {
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithoutActiveTool,
                    detail: normalized.to_string(),
                });
                continue;
            };
            let Some(diameter) = tool_diameter.get(&tool) else {
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithUnknownTool { tool },
                    detail: normalized.to_string(),
                });
                continue;
            };
            if *diameter <= 0.0 {
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithoutDiameter { tool },
                    detail: normalized.to_string(),
                });
                continue;
            }

            drills.push(DrillFeature {
                location: [x * units_scale, y * units_scale],
                diameter: *diameter,
                net: None,
                plated: false,
            });
            continue;
        } else if normalized.contains('X') && normalized.contains('Y') {
            issues.push(ExcellonIssue {
                line: line_number,
                kind: ExcellonIssueKind::InvalidCoordinate {
                    raw_line: normalized.to_string(),
                    reason: "tool hit is missing valid X/Y coordinates".to_string(),
                },
                detail: normalized.to_string(),
            });
        }

        if is_supported_axis_line(normalized) {
            issues.push(ExcellonIssue {
                line: line_number,
                kind: ExcellonIssueKind::InvalidToolDefinition {
                    tool: "Txx".to_string(),
                    raw_diameter: normalized.to_string(),
                },
                detail: normalized.to_string(),
            });
        }
    }

    if !has_units && (!tool_diameter.is_empty() || !drills.is_empty()) {
        issues.push(ExcellonIssue {
            line: 0,
            kind: ExcellonIssueKind::MissingUnitDeclaration,
            detail: source.display().to_string(),
        });
    }

    ExcellonReport {
        source: source.display().to_string(),
        drills,
        issues,
        has_units,
        declared_unit,
    }
}

fn parse_units(line: &str) -> Option<ExcellonUnits> {
    let token = line.split(',').next()?.trim();
    match token {
        "METRIC" => Some(ExcellonUnits::Metric),
        "INCH" => Some(ExcellonUnits::Inch),
        _ => None,
    }
}

fn parse_tool_definition(line: &str, units_scale: f64) -> Option<(String, f64)> {
    if !line.starts_with('T') || !line.contains('C') {
        return None;
    }

    let c_index = line.find('C')?;
    let tool = line[..c_index].trim().to_string();
    if tool.len() < 2 {
        return None;
    }
    if !tool[1..].chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }

    let diameter_text = line[c_index + 1..].trim();
    parse_number(diameter_text).map(|diameter| (tool, diameter * units_scale))
}

fn parse_tool_with_optional_hit(line: &str) -> Option<(String, bool)> {
    if !line.starts_with('T') {
        return None;
    }

    let first_non_digit = line
        .char_indices()
        .skip(1)
        .find_map(|(index, ch)| (!ch.is_ascii_digit()).then_some(index))
        .unwrap_or(line.len());
    if first_non_digit == 1 {
        return None;
    }

    let tool = line[..first_non_digit].to_string();
    let has_coordinate =
        line[first_non_digit..].contains('X') && line[first_non_digit..].contains('Y');
    Some((tool, has_coordinate))
}

fn parse_coordinate(line: &str) -> Option<(f64, f64)> {
    let x_index = line.find('X')?;
    let y_index = line.find('Y')?;
    let x_end = if x_index < y_index {
        y_index
    } else {
        line.len()
    };
    let y_end = if y_index < x_index {
        x_index
    } else {
        line.len()
    };

    let x = parse_number(&line[x_index + 1..x_end])?;
    let y = parse_number(&line[y_index + 1..y_end])?;
    Some((x, y))
}

fn parse_number(raw: &str) -> Option<f64> {
    let trimmed = raw.trim();
    if !trimmed
        .chars()
        .all(|ch| ch.is_ascii_digit() || ch == '+' || ch == '-' || ch == '.')
    {
        return None;
    }

    if trimmed.contains('.') {
        return trimmed.parse().ok();
    }

    let sign = trimmed.starts_with('-');
    let digits = trimmed.trim_start_matches(['+', '-']);
    if !digits.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }
    if digits.len() <= 3 {
        return trimmed.parse().ok();
    }

    let split = digits.len().saturating_sub(3);
    let mut normalized = String::new();
    if sign {
        normalized.push('-');
    }
    normalized.push_str(&digits[..split]);
    normalized.push('.');
    normalized.push_str(&digits[split..]);
    normalized.parse().ok()
}

fn is_supported_axis_line(line: &str) -> bool {
    if line.starts_with('G') {
        return true;
    }
    if line.starts_with('M') {
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::{ExcellonIssueKind, parse_excellon, parse_excellon_report};

    #[test]
    fn parses_metric_tool_hits() {
        let drills = parse_excellon(
            r#"
            M48
            METRIC,TZ
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#,
        );

        assert_eq!(drills.len(), 1);
        assert_eq!(drills[0].diameter, 0.6);
        assert_eq!(drills[0].location, [10.0, 20.0]);
    }

    #[test]
    fn ignores_hits_before_tool_selection_or_definition() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            X010000Y020000
            T01
            X010000Y020000
            "#,
            std::path::Path::new("test"),
        );

        assert_eq!(report.drills.len(), 0);
        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::DrillHitWithoutActiveTool))
        );
    }

    #[test]
    fn parses_inches_as_millimeters() {
        let drills = parse_excellon(
            r#"
            INCH,TZ
            T01C0.010
            T01
            X001000Y002000
            "#,
        );

        assert_eq!(drills.len(), 1);
        assert!((drills[0].diameter - 0.254).abs() < 1.0e-9);
        assert!((drills[0].location[0] - 25.4).abs() < 1.0e-9);
        assert!((drills[0].location[1] - 50.8).abs() < 1.0e-9);
    }

    #[test]
    fn reports_unit_conflict() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            T01C0.600
            INCH,TZ
            T01
            X001000Y002000
            "#,
            std::path::Path::new("test"),
        );

        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::UnitConflict { .. }))
        );
        assert_eq!(report.drills.len(), 1);
    }

    #[test]
    fn reports_tool_definition_conflicts() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            T01C0.600
            T01C0.600
            T01C0.900
            T01
            X010000Y020000
            "#,
            std::path::Path::new("test"),
        );

        let duplicate_count = report
            .issues
            .iter()
            .filter(|issue| {
                matches!(
                    issue.kind,
                    ExcellonIssueKind::DuplicateToolDefinition { .. }
                )
            })
            .count();
        let redefine_count = report
            .issues
            .iter()
            .filter(|issue| matches!(issue.kind, ExcellonIssueKind::ToolRedefinition { .. }))
            .count();

        assert_eq!(duplicate_count, 1);
        assert_eq!(redefine_count, 1);
        assert_eq!(report.drills.len(), 1);
    }

    #[test]
    fn parse_rejects_non_numeric_coordinates_and_reports_issues() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            T01C0.600
            T01
            X01A000Y020000
            "#,
            std::path::Path::new("test"),
        );

        assert_eq!(report.drills.len(), 0);
        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::InvalidCoordinate { .. }))
        );
    }

    #[test]
    fn unknown_tool_selection_is_reported() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            T02
            X010000Y020000
            "#,
            std::path::Path::new("test"),
        );

        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::UnknownToolSelection { .. }))
        );
    }

    #[test]
    fn report_without_units_warns_when_geometry_is_present() {
        let report = parse_excellon_report(
            r#"
            T01C0.600
            X010000Y020000
            "#,
            std::path::Path::new("test"),
        );

        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::MissingUnitDeclaration))
        );
    }

    proptest! {
        #[test]
        fn arbitrary_excellon_text_never_panics(input in "\\PC*") {
            let _ = parse_excellon_report(&input, std::path::Path::new("fuzz.drl"));
        }

        #[test]
        fn generated_metric_hits_are_finite(x in 0u32..999_999, y in 0u32..999_999, diameter in 1u32..5000) {
            let text = format!(
                "METRIC,TZ\nT01C{}.{:03}\nT01\nX{x:06}Y{y:06}\n",
                diameter / 1000,
                diameter % 1000
            );
            let report = parse_excellon_report(&text, std::path::Path::new("fuzz.drl"));
            prop_assert_eq!(report.drills.len(), 1);
            prop_assert!(report.drills[0].location[0].is_finite());
            prop_assert!(report.drills[0].location[1].is_finite());
            prop_assert!(report.drills[0].diameter.is_finite());
            prop_assert!(report.drills[0].diameter > 0.0);
        }
    }
}
