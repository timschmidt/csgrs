//! Excellon drill parser with non-fatal parser diagnostics.
//!
//! The parser accepts common Excellon unit declarations, tool definitions, and
//! hit records, returning drill geometry plus issues that can be surfaced as
//! readiness findings without aborting the rest of a package run.
//!
//! Reliability note: Excellon coordinate formats and headers vary by CAM
//! exporter. Unit inference or malformed-tool recovery is suspect and should be
//! checked against the fabrication drill report before release.

use std::collections::{BTreeSet, HashMap};
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

/// Plating intent inferred from common Excellon drill-file names.
///
/// CAM exporters commonly split plated through-hole and non-plated through-hole
/// drills into sidecars named with `PTH` and `NPTH`. Excellon itself is the CNC
/// transfer format, so this filename-derived intent is release-package evidence
/// rather than a replacement for the fabrication drawing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExcellonPlatingIntent {
    /// Filename evidence indicates plated through-hole drill data.
    Plated,
    /// Filename evidence indicates non-plated through-hole drill data.
    NonPlated,
}

/// Excellon program-structure evidence recovered from common CNC headers.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ExcellonProgramInfo {
    /// Line where the `M48` tool-table header starts, if present.
    pub header_start_line: Option<usize>,
    /// Line where the `%` header terminator appears, if present.
    pub header_end_line: Option<usize>,
    /// Line where the `M30` end-of-program marker appears, if present.
    pub end_of_program_line: Option<usize>,
}

/// Summary of Excellon unit-declaration evidence.
///
/// IPC-NC-349 drill/rout files rely on machine units and coordinate-format
/// context. HyperDRC normalizes parsed geometry to millimeters, but this
/// summary keeps the source-unit declarations visible so release checks can
/// distinguish explicit metric/inch data from ambiguous or dialect-specific
/// unit text.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ExcellonUnitSummary {
    /// Supported unit declarations seen by the parser.
    pub supported_declarations: usize,
    /// Supported metric declarations, including `METRIC` and `M71`.
    pub metric_declarations: usize,
    /// Supported inch declarations, including `INCH` and `M72`.
    pub inch_declarations: usize,
    /// Unsupported unit-like declarations such as `MILS` or `MM`.
    pub unsupported_declarations: usize,
    /// Conflicting supported unit declarations after the first unit.
    pub conflicts: usize,
    /// Explicit zero-suppression declarations such as `TZ` or `LZ`.
    pub zero_suppression_declarations: usize,
    /// Whether a missing-unit diagnostic was emitted for parsed drill data.
    pub missing_unit_diagnostic: bool,
}

/// Summary of Excellon tool-table evidence.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ExcellonToolTableSummary {
    /// Number of unique tool definitions kept by the parser.
    pub defined_tools: usize,
    /// Number of repeated definitions with the same diameter.
    pub duplicate_definitions: usize,
    /// Number of repeated definitions with a different diameter.
    pub redefinitions: usize,
    /// Number of unique tool definitions with non-positive diameter.
    pub non_positive_definitions: usize,
    /// Number of malformed tool-definition-like records emitted as diagnostics.
    pub invalid_definitions: usize,
}

/// Summary of Excellon routing-command evidence.
///
/// IPC-NC-349 covers numeric control data for drilling and routing. HyperDRC
/// currently keeps route-like records as diagnostics instead of synthesizing
/// slot geometry, so these counters make routed-slot evidence available to
/// package/readiness checks without requiring consumers to rescan issue text.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ExcellonRoutingSummary {
    /// Total recognized routing commands.
    pub commands: usize,
    /// Rapid-position route commands such as `G00` or `G0`.
    pub rapid_moves: usize,
    /// Linear-interpolation route commands such as `G01` or `G1`.
    pub linear_moves: usize,
    /// Routed slot commands such as `G85`.
    pub slot_commands: usize,
}

/// Summary of Excellon drill-hit parsing outcomes.
///
/// IPC-NC-349 drill data can express hits after an active tool selection or as
/// inline tool-coordinate records. HyperDRC keeps this summary so readiness
/// checks can distinguish successfully parsed drill geometry from skipped
/// records that lacked a selected/defined tool or valid coordinate syntax.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ExcellonHitSummary {
    /// Drill hits emitted as geometry.
    pub parsed_hits: usize,
    /// Parsed coordinate hits that used the current active tool.
    pub active_tool_hits: usize,
    /// Parsed coordinate hits that selected the tool inline.
    pub inline_tool_hits: usize,
    /// Coordinate-like records encountered before any active tool selection.
    pub hits_without_active_tool: usize,
    /// Coordinate-like records that referenced an undefined tool.
    pub hits_with_unknown_tool: usize,
    /// Coordinate-like records that referenced a non-positive-diameter tool.
    pub hits_without_diameter: usize,
    /// Coordinate-like records rejected as malformed.
    pub invalid_coordinate_records: usize,
}

/// Summary of parsed Excellon drill geometry.
///
/// HyperDRC normalizes parsed drill geometry to millimeters while retaining
/// filename-derived plating intent as release-package evidence. IPC-NC-349
/// carries the CNC drill/rout exchange data; IPC-6012D separates plated and
/// unsupported hole fabrication requirements, so this summary keeps diameter
/// and plating coverage visible for downstream drill-table checks.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExcellonDrillSummary {
    /// Parsed drill features emitted as geometry.
    pub parsed_drills: usize,
    /// Parsed drills marked plated from filename/package evidence.
    pub plated_drills: usize,
    /// Parsed drills marked non-plated from filename/package evidence.
    pub non_plated_drills: usize,
    /// Count of distinct parsed drill diameters after micron-level quantization.
    pub unique_diameters: usize,
    /// Smallest parsed drill diameter in millimeters.
    pub min_diameter: Option<f64>,
    /// Largest parsed drill diameter in millimeters.
    pub max_diameter: Option<f64>,
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
    /// The file declared a zero-suppression mode that the parser does not fully model.
    ZeroSuppressionDeclaration {
        /// Raw zero-suppression token such as `TZ` or `LZ`.
        mode: String,
    },
    /// A unit-like declaration was present, but it is not a supported Excellon unit command.
    UnsupportedUnitDeclaration {
        /// Raw unit token such as `MM` or `MILS`.
        token: String,
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
    /// A routing command was present in the drill file.
    RoutedSlotCommand {
        /// Routing command such as `G00`, `G01`, or `G85`.
        command: String,
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
    /// Excellon unit-declaration summary evidence.
    pub unit_summary: ExcellonUnitSummary,
    /// Excellon program header and end-marker evidence.
    pub program: ExcellonProgramInfo,
    /// Excellon tool-table summary evidence.
    pub tool_table: ExcellonToolTableSummary,
    /// Excellon routing-command summary evidence.
    pub routing: ExcellonRoutingSummary,
    /// Excellon drill-hit parsing summary evidence.
    pub hits: ExcellonHitSummary,
    /// Excellon parsed drill-geometry summary evidence.
    pub drill_summary: ExcellonDrillSummary,
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
            Self::ZeroSuppressionDeclaration { mode } => format!(
                "line {line}: zero-suppression declaration {mode:?} is present; parser uses conservative fixed-decimal coordinate inference"
            ),
            Self::UnsupportedUnitDeclaration { token } => format!(
                "line {line}: unsupported Excellon unit declaration {token:?}; parser supports METRIC/INCH and M71/M72 unit commands"
            ),
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
            Self::RoutedSlotCommand { command } => format!(
                "line {line}: Excellon routing command {command} is present; review exact routed-slot geometry against the fabrication drill/rout report"
            ),
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
    let mut unit_summary = ExcellonUnitSummary::default();
    let mut program = ExcellonProgramInfo::default();
    let mut tool_table = ExcellonToolTableSummary::default();
    let mut routing = ExcellonRoutingSummary::default();
    let mut hits = ExcellonHitSummary::default();
    let mut drill_summary = ExcellonDrillSummary::default();
    let mut unique_diameters = BTreeSet::<i64>::new();
    // IPC-NC-349 carries the CNC drill/rout data, while IPC-6012D separates
    // plated-through and unsupported hole fabrication requirements. Common CAM
    // packages encode that split in the drill sidecar filename, so HyperDRC
    // propagates conservative filename evidence into `DrillFeature::plated`.
    let plated = infer_excellon_plating_intent(source)
        .is_some_and(|intent| intent == ExcellonPlatingIntent::Plated);

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

        if normalized == "M48" {
            // IPC-NC-349 defines the CNC drill/rout transfer syntax that
            // Excellon-style files use. `M48` starts the tool-table header; the
            // geometry checks do not need the header block itself, but preserving
            // its presence helps package-readiness diagnostics distinguish a
            // structured drill file from loose coordinate fragments.
            program.header_start_line = program.header_start_line.or(Some(line_number));
            continue;
        }

        if normalized == "%" {
            program.header_end_line = program.header_end_line.or(Some(line_number));
            continue;
        }

        if normalized == "M30" {
            program.end_of_program_line = program.end_of_program_line.or(Some(line_number));
            continue;
        }

        if matches!(normalized, "M47" | "M95") {
            continue;
        }

        if let Some(command) = parse_routed_slot_command(normalized) {
            // IPC-NC-349 defines CNC data for both drilling and routing. When a
            // nominal Excellon sidecar contains route commands, HyperDRC keeps
            // later drill-hit parsing deterministic but emits a readiness
            // diagnostic because slot width/end-radius/slot-to-copper checks
            // need exact routed-path geometry rather than isolated hole hits.
            routing.count(&command);
            issues.push(ExcellonIssue {
                line: line_number,
                kind: ExcellonIssueKind::RoutedSlotCommand { command },
                detail: normalized.to_string(),
            });
            continue;
        }

        if let Some(unit_directive) = parse_unit_directive(normalized) {
            let units = match unit_directive {
                UnitDirective::Supported(units) => units,
                UnitDirective::Unsupported(token) => {
                    // IPC-NC-349 defines the machine transfer syntax, but CAM
                    // exporters sometimes use informal unit labels. Preserve
                    // the package run and emit explicit evidence instead of
                    // silently interpreting non-standard unit text.
                    unit_summary.unsupported_declarations += 1;
                    issues.push(ExcellonIssue {
                        line: line_number,
                        kind: ExcellonIssueKind::UnsupportedUnitDeclaration { token },
                        detail: normalized.to_string(),
                    });
                    continue;
                }
            };
            if let Some(mode) = parse_zero_suppression(normalized) {
                // Excellon zero suppression and coordinate-format declarations
                // affect how integer hit records are decoded. HyperDRC keeps a
                // deterministic fixed-decimal fallback for geometry continuity,
                // but emits a diagnostic so the fabrication drill report remains
                // the source of truth before release.
                unit_summary.zero_suppression_declarations += 1;
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::ZeroSuppressionDeclaration { mode },
                    detail: normalized.to_string(),
                });
            }
            let incoming_scale = match units {
                ExcellonUnits::Metric => {
                    unit_summary.metric_declarations += 1;
                    1.0
                }
                ExcellonUnits::Inch => {
                    unit_summary.inch_declarations += 1;
                    25.4
                }
            };
            unit_summary.supported_declarations += 1;
            if let Some(existing) = declared_unit {
                if existing != units {
                    unit_summary.conflicts += 1;
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
                    tool_table.duplicate_definitions += 1;
                    issues.push(ExcellonIssue {
                        line: line_number,
                        kind: ExcellonIssueKind::DuplicateToolDefinition { tool, diameter },
                        detail: normalized.to_string(),
                    });
                }
                Some(previous) => {
                    tool_table.redefinitions += 1;
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
                        tool_table.non_positive_definitions += 1;
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
            tool_table.defined_tools = tool_diameter.len();
            continue;
        }

        if let Some((line_tool, has_coordinate)) = parse_tool_with_optional_hit(normalized) {
            if has_coordinate {
                match parse_coordinate(normalized) {
                    Some((x, y)) => {
                        let Some(diameter) = tool_diameter.get(&line_tool) else {
                            hits.hits_with_unknown_tool += 1;
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
                            hits.hits_without_diameter += 1;
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
                        hits.inline_tool_hits += 1;
                        hits.parsed_hits += 1;
                        record_drill(
                            &mut drills,
                            &mut drill_summary,
                            &mut unique_diameters,
                            DrillFeature {
                                location: [x * units_scale, y * units_scale],
                                diameter: *diameter,
                                net: None,
                                plated,
                            },
                        );
                        continue;
                    }
                    None => {
                        hits.invalid_coordinate_records += 1;
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
                hits.hits_without_active_tool += 1;
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithoutActiveTool,
                    detail: normalized.to_string(),
                });
                continue;
            };
            let Some(diameter) = tool_diameter.get(&tool) else {
                hits.hits_with_unknown_tool += 1;
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithUnknownTool { tool },
                    detail: normalized.to_string(),
                });
                continue;
            };
            if *diameter <= 0.0 {
                hits.hits_without_diameter += 1;
                issues.push(ExcellonIssue {
                    line: line_number,
                    kind: ExcellonIssueKind::DrillHitWithoutDiameter { tool },
                    detail: normalized.to_string(),
                });
                continue;
            }

            hits.active_tool_hits += 1;
            hits.parsed_hits += 1;
            record_drill(
                &mut drills,
                &mut drill_summary,
                &mut unique_diameters,
                DrillFeature {
                    location: [x * units_scale, y * units_scale],
                    diameter: *diameter,
                    net: None,
                    plated,
                },
            );
            continue;
        } else if normalized.contains('X') && normalized.contains('Y') {
            hits.invalid_coordinate_records += 1;
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
            tool_table.invalid_definitions += 1;
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
        unit_summary.missing_unit_diagnostic = true;
        issues.push(ExcellonIssue {
            line: 0,
            kind: ExcellonIssueKind::MissingUnitDeclaration,
            detail: source.display().to_string(),
        });
    }
    drill_summary.unique_diameters = unique_diameters.len();

    log::trace!(
        "excellon parse: drills={} issues={} units={} supported_unit_declarations={} metric_unit_declarations={} inch_unit_declarations={} unsupported_unit_declarations={} unit_conflicts={} zero_suppression_declarations={} missing_unit_diagnostic={} drill_summary_drills={} plated_drills={} non_plated_drills={} unique_diameters={} min_diameter_mm={:?} max_diameter_mm={:?} tools={} duplicate_tools={} redefined_tools={} non_positive_tools={} invalid_tool_records={} parsed_hits={} active_tool_hits={} inline_tool_hits={} hits_without_active_tool={} hits_with_unknown_tool={} hits_without_diameter={} invalid_coordinate_records={} route_commands={} route_rapid_moves={} route_linear_moves={} route_slot_commands={} header_start={} header_end={} end_of_program={}",
        drills.len(),
        issues.len(),
        declared_unit.is_some(),
        unit_summary.supported_declarations,
        unit_summary.metric_declarations,
        unit_summary.inch_declarations,
        unit_summary.unsupported_declarations,
        unit_summary.conflicts,
        unit_summary.zero_suppression_declarations,
        unit_summary.missing_unit_diagnostic,
        drill_summary.parsed_drills,
        drill_summary.plated_drills,
        drill_summary.non_plated_drills,
        drill_summary.unique_diameters,
        drill_summary.min_diameter,
        drill_summary.max_diameter,
        tool_table.defined_tools,
        tool_table.duplicate_definitions,
        tool_table.redefinitions,
        tool_table.non_positive_definitions,
        tool_table.invalid_definitions,
        hits.parsed_hits,
        hits.active_tool_hits,
        hits.inline_tool_hits,
        hits.hits_without_active_tool,
        hits.hits_with_unknown_tool,
        hits.hits_without_diameter,
        hits.invalid_coordinate_records,
        routing.commands,
        routing.rapid_moves,
        routing.linear_moves,
        routing.slot_commands,
        program.header_start_line.is_some(),
        program.header_end_line.is_some(),
        program.end_of_program_line.is_some()
    );

    ExcellonReport {
        source: source.display().to_string(),
        drills,
        issues,
        has_units,
        declared_unit,
        unit_summary,
        program,
        tool_table,
        routing,
        hits,
        drill_summary,
    }
}

fn record_drill(
    drills: &mut Vec<DrillFeature>,
    summary: &mut ExcellonDrillSummary,
    unique_diameters: &mut BTreeSet<i64>,
    drill: DrillFeature,
) {
    summary.parsed_drills += 1;
    if drill.plated {
        summary.plated_drills += 1;
    } else {
        summary.non_plated_drills += 1;
    }
    summary.min_diameter = Some(
        summary
            .min_diameter
            .map_or(drill.diameter, |current| current.min(drill.diameter)),
    );
    summary.max_diameter = Some(
        summary
            .max_diameter
            .map_or(drill.diameter, |current| current.max(drill.diameter)),
    );
    if drill.diameter.is_finite() {
        unique_diameters.insert((drill.diameter * 1_000_000.0).round() as i64);
    }
    drills.push(drill);
}

impl ExcellonRoutingSummary {
    fn count(&mut self, command: &str) {
        self.commands += 1;
        match command {
            "G00" | "G0" => self.rapid_moves += 1,
            "G01" | "G1" => self.linear_moves += 1,
            "G85" => self.slot_commands += 1,
            _ => {}
        }
    }
}

/// Infer plated/non-plated intent from a drill sidecar path.
///
/// This intentionally recognizes conservative whole-token forms (`PTH`,
/// `NPTH`, `non-plated`, and close variants) so ordinary words containing
/// `pt` or `pth` do not silently affect parsed drill semantics.
pub fn infer_excellon_plating_intent(source: &Path) -> Option<ExcellonPlatingIntent> {
    let normalized = source.display().to_string().to_ascii_lowercase();
    let tokens = normalized
        .split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();

    if tokens.iter().any(|token| {
        matches!(
            *token,
            "npth" | "npt" | "nonplated" | "nonplatedthrough" | "unplated"
        )
    }) || normalized.contains("non-plated")
        || normalized.contains("non_plated")
    {
        return Some(ExcellonPlatingIntent::NonPlated);
    }
    if tokens.iter().any(|token| {
        matches!(
            *token,
            "pth" | "pt" | "plated" | "platedthrough" | "platedthru"
        )
    }) {
        return Some(ExcellonPlatingIntent::Plated);
    }
    None
}

enum UnitDirective {
    Supported(ExcellonUnits),
    Unsupported(String),
}

fn parse_unit_directive(line: &str) -> Option<UnitDirective> {
    let token = line.split(',').next()?.trim().to_ascii_uppercase();
    match token.as_str() {
        "METRIC" | "M71" => Some(UnitDirective::Supported(ExcellonUnits::Metric)),
        "INCH" | "M72" => Some(UnitDirective::Supported(ExcellonUnits::Inch)),
        "MM" | "MILLIMETER" | "MILLIMETERS" | "MIL" | "MILS" | "IMPERIAL" => {
            Some(UnitDirective::Unsupported(token))
        }
        _ => None,
    }
}

fn parse_zero_suppression(line: &str) -> Option<String> {
    line.split(',')
        .skip(1)
        .map(str::trim)
        .map(str::to_ascii_uppercase)
        .find(|token| matches!(token.as_str(), "TZ" | "LZ" | "TRAILING" | "LEADING"))
}

fn parse_routed_slot_command(line: &str) -> Option<String> {
    let mut chars = line.chars();
    if chars.next()? != 'G' {
        return None;
    }
    let digits = chars
        .take_while(|ch| ch.is_ascii_digit())
        .collect::<String>();
    let command = format!("G{digits}").to_ascii_uppercase();
    if matches!(command.as_str(), "G00" | "G0" | "G01" | "G1" | "G85") {
        Some(command)
    } else {
        None
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

    use super::{
        ExcellonIssueKind, ExcellonPlatingIntent, infer_excellon_plating_intent, parse_excellon,
        parse_excellon_report,
    };

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
    fn reports_zero_suppression_declarations_without_dropping_geometry() {
        let report = parse_excellon_report(
            r#"
            M48
            METRIC,LZ
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#,
            std::path::Path::new("test"),
        );

        assert_eq!(report.drills.len(), 1);
        assert!(report.issues.iter().any(|issue| {
            matches!(
                issue.kind,
                ExcellonIssueKind::ZeroSuppressionDeclaration { .. }
            )
        }));
    }

    #[test]
    fn preserves_program_structure_markers_without_diagnostics() {
        let report = parse_excellon_report(
            r#"
            M48
            METRIC
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#,
            std::path::Path::new("structured.drl"),
        );

        assert_eq!(report.program.header_start_line, Some(2));
        assert_eq!(report.program.header_end_line, Some(5));
        assert_eq!(report.program.end_of_program_line, Some(8));
        assert_eq!(report.drills.len(), 1);
        assert!(
            !report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::InvalidToolDefinition { .. })),
            "M48/%/M30 program markers should not be reported as malformed tools"
        );
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
        assert_eq!(report.hits.hits_without_active_tool, 1);
        assert_eq!(report.hits.hits_with_unknown_tool, 1);
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
    fn parses_m71_and_m72_unit_commands() {
        let metric_report = parse_excellon_report(
            r#"
            M48
            M71
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#,
            std::path::Path::new("metric-m71.drl"),
        );
        let inch_report = parse_excellon_report(
            r#"
            M48
            M72
            T01C0.010
            %
            T01
            X001000Y002000
            M30
            "#,
            std::path::Path::new("inch-m72.drl"),
        );

        assert_eq!(
            metric_report.declared_unit,
            Some(super::ExcellonUnits::Metric)
        );
        assert_eq!(metric_report.unit_summary.supported_declarations, 1);
        assert_eq!(metric_report.unit_summary.metric_declarations, 1);
        assert_eq!(metric_report.unit_summary.inch_declarations, 0);
        assert_eq!(metric_report.drills[0].location, [10.0, 20.0]);
        assert_eq!(inch_report.declared_unit, Some(super::ExcellonUnits::Inch));
        assert_eq!(inch_report.unit_summary.supported_declarations, 1);
        assert_eq!(inch_report.unit_summary.metric_declarations, 0);
        assert_eq!(inch_report.unit_summary.inch_declarations, 1);
        assert!((inch_report.drills[0].diameter - 0.254).abs() < 1.0e-9);
        assert!((inch_report.drills[0].location[0] - 25.4).abs() < 1.0e-9);
    }

    #[test]
    fn unsupported_unit_declarations_are_reported_without_guessing_units() {
        let report = parse_excellon_report(
            r#"
            M48
            MILS
            T01C0.010
            %
            T01
            X001000Y002000
            M30
            "#,
            std::path::Path::new("unsupported-units.drl"),
        );

        assert!(report.declared_unit.is_none());
        assert_eq!(report.unit_summary.unsupported_declarations, 1);
        assert!(report.unit_summary.missing_unit_diagnostic);
        assert!(report.issues.iter().any(|issue| {
            matches!(
                issue.kind,
                ExcellonIssueKind::UnsupportedUnitDeclaration { .. }
            )
        }));
        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::MissingUnitDeclaration))
        );
        assert!(report.unit_summary.missing_unit_diagnostic);
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
        assert_eq!(report.unit_summary.supported_declarations, 2);
        assert_eq!(report.unit_summary.metric_declarations, 1);
        assert_eq!(report.unit_summary.inch_declarations, 1);
        assert_eq!(report.unit_summary.conflicts, 1);
        assert_eq!(report.unit_summary.zero_suppression_declarations, 2);
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
            T02C0.000
            G99
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
        assert_eq!(report.tool_table.defined_tools, 2);
        assert_eq!(report.tool_table.duplicate_definitions, 1);
        assert_eq!(report.tool_table.redefinitions, 1);
        assert_eq!(report.tool_table.non_positive_definitions, 1);
        assert_eq!(report.tool_table.invalid_definitions, 1);
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
        assert_eq!(report.hits.invalid_coordinate_records, 1);
        assert!(
            report
                .issues
                .iter()
                .any(|issue| matches!(issue.kind, ExcellonIssueKind::InvalidCoordinate { .. }))
        );
    }

    #[test]
    fn routed_slot_commands_are_reported_without_creating_fake_drill_hits() {
        let report = parse_excellon_report(
            r#"
            M48
            METRIC,TZ
            T01C0.800
            %
            T01
            G00X010000Y010000
            G01X012000Y010000
            G85X014000Y010000X018000Y010000
            X020000Y020000
            M30
            "#,
            std::path::Path::new("slots.drl"),
        );

        assert_eq!(report.drills.len(), 1);
        assert_eq!(report.hits.parsed_hits, 1);
        assert_eq!(report.hits.active_tool_hits, 1);
        assert_eq!(report.hits.inline_tool_hits, 0);
        let route_commands = report
            .issues
            .iter()
            .filter_map(|issue| match &issue.kind {
                ExcellonIssueKind::RoutedSlotCommand { command } => Some(command.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>();

        assert_eq!(route_commands, vec!["G00", "G01", "G85"]);
        assert_eq!(report.routing.commands, 3);
        assert_eq!(report.routing.rapid_moves, 1);
        assert_eq!(report.routing.linear_moves, 1);
        assert_eq!(report.routing.slot_commands, 1);
    }

    #[test]
    fn summarizes_inline_active_and_rejected_drill_hits() {
        let report = parse_excellon_report(
            r#"
            METRIC,TZ
            X001000Y001000
            T02X002000Y002000
            T03C0.000
            T03X003000Y003000
            T01C0.600
            T01X004000Y004000
            X005000Y005000
            X00600AY006000
            "#,
            std::path::Path::new("hits.drl"),
        );

        assert_eq!(report.drills.len(), 2);
        assert_eq!(report.hits.parsed_hits, 2);
        assert_eq!(report.hits.inline_tool_hits, 1);
        assert_eq!(report.hits.active_tool_hits, 1);
        assert_eq!(report.hits.hits_without_active_tool, 1);
        assert_eq!(report.hits.hits_with_unknown_tool, 1);
        assert_eq!(report.hits.hits_without_diameter, 1);
        assert_eq!(report.hits.invalid_coordinate_records, 1);
        assert_eq!(report.drill_summary.parsed_drills, 2);
        assert_eq!(report.drill_summary.unique_diameters, 1);
        assert_eq!(report.drill_summary.min_diameter, Some(0.6));
        assert_eq!(report.drill_summary.max_diameter, Some(0.6));
    }

    #[test]
    fn filename_plating_intent_marks_sidecar_drills() {
        let text = r#"
            M48
            METRIC,TZ
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#;
        let pth_report = parse_excellon_report(text, std::path::Path::new("board-PTH.drl"));
        let npth_report = parse_excellon_report(text, std::path::Path::new("board-NPTH.drl"));
        let unknown_report = parse_excellon_report(text, std::path::Path::new("board-drill.drl"));

        assert_eq!(
            infer_excellon_plating_intent(std::path::Path::new("board-PTH.drl")),
            Some(ExcellonPlatingIntent::Plated)
        );
        assert_eq!(
            infer_excellon_plating_intent(std::path::Path::new("board-NPTH.drl")),
            Some(ExcellonPlatingIntent::NonPlated)
        );
        assert!(pth_report.drills[0].plated);
        assert!(!npth_report.drills[0].plated);
        assert!(!unknown_report.drills[0].plated);
        assert_eq!(pth_report.drill_summary.parsed_drills, 1);
        assert_eq!(pth_report.drill_summary.plated_drills, 1);
        assert_eq!(pth_report.drill_summary.non_plated_drills, 0);
        assert_eq!(npth_report.drill_summary.parsed_drills, 1);
        assert_eq!(npth_report.drill_summary.plated_drills, 0);
        assert_eq!(npth_report.drill_summary.non_plated_drills, 1);
        assert_eq!(unknown_report.drill_summary.parsed_drills, 1);
        assert_eq!(unknown_report.drill_summary.unique_diameters, 1);
    }

    #[test]
    fn summarizes_drill_diameter_range_and_unique_diameters() {
        let report = parse_excellon_report(
            r#"
            M48
            METRIC
            T01C0.300
            T02C0.600
            %
            T01
            X010000Y020000
            X011000Y020000
            T02
            X012000Y020000
            M30
            "#,
            std::path::Path::new("board-PTH.drl"),
        );

        assert_eq!(report.drill_summary.parsed_drills, 3);
        assert_eq!(report.drill_summary.plated_drills, 3);
        assert_eq!(report.drill_summary.non_plated_drills, 0);
        assert_eq!(report.drill_summary.unique_diameters, 2);
        assert_eq!(report.drill_summary.min_diameter, Some(0.3));
        assert_eq!(report.drill_summary.max_diameter, Some(0.6));
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
