//! IPC-D-356 electrical-test netlist parsing.
//!
//! IPC-D-356B describes a bare-substrate electrical test data format rather
//! than an artwork format. hyperdrc therefore treats records as net/test access
//! evidence and records malformed recognized test records as parser diagnostics
//! instead of guessing geometry. See IPC-D-356B, *Bare Substrate Electrical Test
//! Data Format* (IPC, 2002).

use std::collections::HashSet;
use std::path::Path;

use anyhow::{Context, Result};

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Point`.
pub struct Ipc356Point {
    /// Field `net`.
    pub net: String,
    /// Field `reference`.
    pub reference: Option<String>,
    /// Field `pin`.
    pub pin: Option<String>,
    /// Field `location`.
    pub location: [f64; 2],
    /// Field `diameter`.
    pub diameter: Option<f64>,
    /// Field `access_side`.
    pub access_side: Option<Ipc356AccessSide>,
    /// Field `feature_type`.
    pub feature_type: Option<Ipc356FeatureType>,
    /// Field `soldermask`.
    pub soldermask: Option<Ipc356Soldermask>,
}

/// Recognized IPC-D-356 test-record code.
///
/// IPC-D-356B standardizes several electrical-test record classes. HyperDRC
/// keeps the raw class as parser evidence so future checks can distinguish
/// dialect-specific record sources without changing geometry consumers.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Ipc356RecordCode {
    /// IPC-D-356 `317` test record.
    Code317,
    /// IPC-D-356 `327` test record.
    Code327,
    /// IPC-D-356 `367` test record.
    Code367,
}

/// Per-code IPC-D-356 parser summary.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356RecordStats {
    /// Recognized `317` records.
    pub code_317: usize,
    /// Recognized `327` records.
    pub code_327: usize,
    /// Recognized `367` records.
    pub code_367: usize,
    /// Recognized records that were malformed and emitted parser diagnostics.
    pub malformed: usize,
}

/// Report-level IPC-D-356 sidecar metadata summary.
///
/// IPC-D-356B is primarily an electrical-test netlist. Common CAD/CAM
/// exporters add sidecar tokens for probe side, feature class, and soldermask
/// exposure; this summary keeps those pragmatic DFT hints visible at the report
/// boundary. The underlying standard reference is IPC-D-356B, *Bare Substrate
/// Electrical Test Data Format* (IPC, 2002).
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356MetadataStats {
    /// Records with any parsed access-side hint.
    pub access_side_records: usize,
    /// Records explicitly marked top-side accessible.
    pub top_access: usize,
    /// Records explicitly marked bottom-side accessible.
    pub bottom_access: usize,
    /// Records explicitly marked accessible from either side.
    pub both_access: usize,
    /// Records with any parsed feature-type hint.
    pub feature_type_records: usize,
    /// Records marked as through-hole features.
    pub through_hole_features: usize,
    /// Records marked as surface-mount features.
    pub smd_features: usize,
    /// Records marked as vias.
    pub via_features: usize,
    /// Records marked as tooling or fiducial features.
    pub tooling_features: usize,
    /// Records marked as connector or edge-contact features.
    pub connector_features: usize,
    /// Records marked with a known but uncategorized feature class.
    pub other_features: usize,
    /// Records with any parsed soldermask hint.
    pub soldermask_records: usize,
    /// Records marked as exposed to the probe.
    pub open_soldermask: usize,
    /// Records marked as covered by soldermask.
    pub covered_soldermask: usize,
    /// Records with an explicit but unknown soldermask-access state.
    pub unknown_soldermask: usize,
}

/// Report-level IPC-D-356 net-name summary.
///
/// IPC-D-356B records carry electrical-test net identity. HyperDRC preserves a
/// compact net-name summary so DFT and release-package checks can distinguish a
/// useful netlist from coordinate-only probe data before cross-checking against
/// KiCad, Gerber attributes, or assembly fixtures.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356NetStats {
    /// Parsed point records with a non-empty net name.
    pub named_records: usize,
    /// Parsed point records whose net field is blank after normalization.
    pub blank_records: usize,
    /// Number of distinct non-empty net names.
    pub unique_nets: usize,
}

/// Report-level IPC-D-356 component/pin field summary.
///
/// IPC-D-356B test records can carry reference-designator and pin evidence in
/// addition to net identity. Keeping these counts at report scope helps future
/// BOM/centroid/netlist parity checks decide whether a sidecar can support
/// component-level DFT validation.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356ComponentStats {
    /// Parsed point records with a reference designator.
    pub reference_records: usize,
    /// Parsed point records with a pin designator.
    pub pin_records: usize,
    /// Parsed point records with both reference and pin fields.
    pub reference_pin_records: usize,
    /// Number of distinct parsed reference designators.
    pub unique_references: usize,
}

/// Report-level IPC-D-356 drill/probe diameter summary.
///
/// IPC-D-356B supports test-access diameter evidence that HyperDRC uses for
/// drill-table and fixture-access checks. These counters expose whether the
/// source file has enough diameter coverage for those checks before comparing
/// against KiCad or Excellon geometry.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356DiameterStats {
    /// Parsed point records with any diameter field.
    pub diameter_records: usize,
    /// Parsed point records without a diameter field.
    pub missing_diameter_records: usize,
    /// Parsed point records whose diameter is zero or negative.
    pub non_positive_diameter_records: usize,
}

/// Report-level IPC-D-356 geometry summary.
///
/// IPC-D-356B records carry test-access coordinates and optional probe/drill
/// diameters. HyperDRC keeps this compact geometry envelope so coverage,
/// fixture-access, and drill-diameter checks can inspect package scale and
/// diameter range without rescanning every parsed point.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ipc356GeometryStats {
    /// Parsed point records included in the geometry summary.
    pub point_records: usize,
    /// Minimum parsed X coordinate.
    pub min_x: Option<f64>,
    /// Maximum parsed X coordinate.
    pub max_x: Option<f64>,
    /// Minimum parsed Y coordinate.
    pub min_y: Option<f64>,
    /// Maximum parsed Y coordinate.
    pub max_y: Option<f64>,
    /// Smallest positive parsed diameter.
    pub min_positive_diameter: Option<f64>,
    /// Largest positive parsed diameter.
    pub max_positive_diameter: Option<f64>,
}

/// Report-level IPC-D-356 parser diagnostic summary.
///
/// IPC-D-356B files are often produced by CAM/test tooling that may preserve
/// usable records beside malformed or dialect-specific records. HyperDRC keeps
/// diagnostic counters at report scope so release checks can fail on parser
/// confidence without re-walking every diagnostic. See IPC-D-356B, *Bare
/// Substrate Electrical Test Data Format* (IPC, 2002).
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Ipc356IssueStats {
    /// Total parser diagnostics emitted for recognized IPC-D-356 records.
    pub total_issues: usize,
    /// Malformed recognized test records that could not produce a point.
    pub malformed_test_records: usize,
}

/// Probe-side hints from common IPC-D-356 sidecar exports.
///
/// IPC-D-356B standardizes electrical-test records, but many EDA/export flows
/// add pragmatic side tokens beside the record. hyperdrc keeps those tokens as
/// readiness evidence instead of treating them as authoritative geometry.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
/// Public enumeration for `Ipc356AccessSide`.
pub enum Ipc356AccessSide {
    /// Variant `Top`.
    Top,
    /// Variant `Bottom`.
    Bottom,
    /// Variant `Both`.
    Both,
}

/// Coarse test-feature class recovered from common sidecar annotations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Public enumeration for `Ipc356FeatureType`.
pub enum Ipc356FeatureType {
    /// Variant `ThroughHole`.
    ThroughHole,
    /// Variant `Smd`.
    Smd,
    /// Variant `Via`.
    Via,
    /// Variant `Tooling`.
    Tooling,
    /// Variant `Connector`.
    Connector,
    /// Variant `Other`.
    Other,
}

/// Soldermask access state recovered from common testpoint annotations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Public enumeration for `Ipc356Soldermask`.
pub enum Ipc356Soldermask {
    /// Variant `Open`.
    Open,
    /// Variant `Covered`.
    Covered,
    /// Variant `Unknown`.
    Unknown,
}

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Issue`.
pub struct Ipc356Issue {
    /// Field `line`.
    pub line: usize,
    /// Field `kind`.
    pub kind: Ipc356IssueKind,
    /// Field `detail`.
    pub detail: String,
}

#[derive(Clone, Debug)]
/// Public enumeration for `Ipc356IssueKind`.
pub enum Ipc356IssueKind {
    /// Variant `MalformedTestRecord`.
    MalformedTestRecord,
}

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Report`.
pub struct Ipc356Report {
    /// Field `source`.
    pub source: String,
    /// Field `points`.
    pub points: Vec<Ipc356Point>,
    /// Field `issues`.
    pub issues: Vec<Ipc356Issue>,
    /// Counts of recognized IPC-D-356 test-record classes.
    pub record_stats: Ipc356RecordStats,
    /// Counts of parsed access-side, feature-type, and soldermask sidecar hints.
    pub metadata_stats: Ipc356MetadataStats,
    /// Counts of parsed net-name coverage.
    pub net_stats: Ipc356NetStats,
    /// Counts of parsed reference-designator and pin coverage.
    pub component_stats: Ipc356ComponentStats,
    /// Counts of parsed diameter-field coverage.
    pub diameter_stats: Ipc356DiameterStats,
    /// Parsed coordinate and positive-diameter envelope.
    pub geometry_stats: Ipc356GeometryStats,
    /// Counts of parser diagnostics grouped by recognized issue category.
    pub issue_stats: Ipc356IssueStats,
}

impl Ipc356Issue {
    /// Run or compute `message`.
    pub fn message(&self) -> String {
        match self.kind {
            Ipc356IssueKind::MalformedTestRecord => format!(
                "line {}: recognized IPC-D-356 test record could not be parsed",
                self.line
            ),
        }
    }
}

impl Ipc356RecordStats {
    fn count(&mut self, code: Ipc356RecordCode) {
        match code {
            Ipc356RecordCode::Code317 => self.code_317 += 1,
            Ipc356RecordCode::Code327 => self.code_327 += 1,
            Ipc356RecordCode::Code367 => self.code_367 += 1,
        }
    }
}

impl Ipc356MetadataStats {
    fn count_point(&mut self, point: &Ipc356Point) {
        match point.access_side {
            Some(Ipc356AccessSide::Top) => {
                self.access_side_records += 1;
                self.top_access += 1;
            }
            Some(Ipc356AccessSide::Bottom) => {
                self.access_side_records += 1;
                self.bottom_access += 1;
            }
            Some(Ipc356AccessSide::Both) => {
                self.access_side_records += 1;
                self.both_access += 1;
            }
            None => {}
        }

        match point.feature_type {
            Some(Ipc356FeatureType::ThroughHole) => {
                self.feature_type_records += 1;
                self.through_hole_features += 1;
            }
            Some(Ipc356FeatureType::Smd) => {
                self.feature_type_records += 1;
                self.smd_features += 1;
            }
            Some(Ipc356FeatureType::Via) => {
                self.feature_type_records += 1;
                self.via_features += 1;
            }
            Some(Ipc356FeatureType::Tooling) => {
                self.feature_type_records += 1;
                self.tooling_features += 1;
            }
            Some(Ipc356FeatureType::Connector) => {
                self.feature_type_records += 1;
                self.connector_features += 1;
            }
            Some(Ipc356FeatureType::Other) => {
                self.feature_type_records += 1;
                self.other_features += 1;
            }
            None => {}
        }

        match point.soldermask {
            Some(Ipc356Soldermask::Open) => {
                self.soldermask_records += 1;
                self.open_soldermask += 1;
            }
            Some(Ipc356Soldermask::Covered) => {
                self.soldermask_records += 1;
                self.covered_soldermask += 1;
            }
            Some(Ipc356Soldermask::Unknown) => {
                self.soldermask_records += 1;
                self.unknown_soldermask += 1;
            }
            None => {}
        }
    }
}

impl Ipc356ComponentStats {
    fn count_point(&mut self, point: &Ipc356Point) {
        let has_reference = point
            .reference
            .as_deref()
            .is_some_and(|value| !value.trim().is_empty());
        let has_pin = point
            .pin
            .as_deref()
            .is_some_and(|value| !value.trim().is_empty());
        if has_reference {
            self.reference_records += 1;
        }
        if has_pin {
            self.pin_records += 1;
        }
        if has_reference && has_pin {
            self.reference_pin_records += 1;
        }
    }
}

impl Ipc356DiameterStats {
    fn count_point(&mut self, point: &Ipc356Point) {
        match point.diameter {
            Some(diameter) => {
                self.diameter_records += 1;
                if diameter <= 0.0 {
                    self.non_positive_diameter_records += 1;
                }
            }
            None => self.missing_diameter_records += 1,
        }
    }
}

impl Ipc356GeometryStats {
    fn count_point(&mut self, point: &Ipc356Point) {
        self.point_records += 1;
        self.min_x = Some(
            self.min_x
                .map_or(point.location[0], |value| value.min(point.location[0])),
        );
        self.max_x = Some(
            self.max_x
                .map_or(point.location[0], |value| value.max(point.location[0])),
        );
        self.min_y = Some(
            self.min_y
                .map_or(point.location[1], |value| value.min(point.location[1])),
        );
        self.max_y = Some(
            self.max_y
                .map_or(point.location[1], |value| value.max(point.location[1])),
        );
        if let Some(diameter) = point.diameter.filter(|diameter| *diameter > 0.0) {
            self.min_positive_diameter = Some(
                self.min_positive_diameter
                    .map_or(diameter, |value| value.min(diameter)),
            );
            self.max_positive_diameter = Some(
                self.max_positive_diameter
                    .map_or(diameter, |value| value.max(diameter)),
            );
        }
    }
}

impl Ipc356IssueStats {
    fn count(&mut self, kind: &Ipc356IssueKind) {
        self.total_issues += 1;
        match kind {
            Ipc356IssueKind::MalformedTestRecord => self.malformed_test_records += 1,
        }
    }
}

/// Run or compute `load_ipc356_report`.
pub fn load_ipc356_report(path: &Path) -> Result<Ipc356Report> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(parse_ipc356_report(&text, path))
}

/// Run or compute `load_ipc356`.
pub fn load_ipc356(path: &Path) -> Result<Vec<Ipc356Point>> {
    Ok(load_ipc356_report(path)?.points)
}

/// Run or compute `parse_ipc356`.
pub fn parse_ipc356(input: &str) -> Vec<Ipc356Point> {
    parse_ipc356_report(input, Path::new("<inline-ipc356>")).points
}

/// Run or compute `parse_ipc356_report`.
pub fn parse_ipc356_report(input: &str, source: &Path) -> Ipc356Report {
    let mut points = Vec::new();
    let mut issues = Vec::new();
    let mut record_stats = Ipc356RecordStats::default();
    let mut metadata_stats = Ipc356MetadataStats::default();
    let mut component_stats = Ipc356ComponentStats::default();
    let mut diameter_stats = Ipc356DiameterStats::default();
    let mut geometry_stats = Ipc356GeometryStats::default();
    let mut issue_stats = Ipc356IssueStats::default();
    let mut unique_nets = HashSet::<String>::new();
    let mut unique_references = HashSet::<String>::new();
    let mut blank_net_records = 0usize;
    for (index, raw_line) in input.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty()
            || line.starts_with('C')
            || line.starts_with('P')
            || line.starts_with('9')
        {
            continue;
        }
        let Some(record_code) = test_record_code(line) else {
            continue;
        };
        record_stats.count(record_code);
        match parse_record(line) {
            Some(point) => {
                metadata_stats.count_point(&point);
                component_stats.count_point(&point);
                diameter_stats.count_point(&point);
                geometry_stats.count_point(&point);
                if point.net.trim().is_empty() {
                    blank_net_records += 1;
                } else {
                    unique_nets.insert(point.net.clone());
                }
                if let Some(reference) = point.reference.as_deref() {
                    let reference = reference.trim();
                    if !reference.is_empty() {
                        unique_references.insert(reference.to_string());
                    }
                }
                points.push(point);
            }
            None => {
                record_stats.malformed += 1;
                let kind = Ipc356IssueKind::MalformedTestRecord;
                issue_stats.count(&kind);
                issues.push(Ipc356Issue {
                    line: index + 1,
                    kind,
                    detail: line.to_string(),
                });
            }
        }
    }
    let net_stats = Ipc356NetStats {
        named_records: points.len().saturating_sub(blank_net_records),
        blank_records: blank_net_records,
        unique_nets: unique_nets.len(),
    };
    component_stats.unique_references = unique_references.len();

    log::trace!(
        "ipc356 parse: points={} issues={} issue_stats_total={} malformed_test_record_issues={} code_317={} code_327={} code_367={} malformed={} named_net_records={} blank_net_records={} unique_nets={} reference_records={} pin_records={} reference_pin_records={} unique_references={} diameter_records={} missing_diameter_records={} non_positive_diameter_records={} geometry_points={} min_x={:?} max_x={:?} min_y={:?} max_y={:?} min_positive_diameter={:?} max_positive_diameter={:?} access_side_records={} feature_type_records={} soldermask_records={} top_access={} bottom_access={} both_access={} open_soldermask={} covered_soldermask={} unknown_soldermask={}",
        points.len(),
        issues.len(),
        issue_stats.total_issues,
        issue_stats.malformed_test_records,
        record_stats.code_317,
        record_stats.code_327,
        record_stats.code_367,
        record_stats.malformed,
        net_stats.named_records,
        net_stats.blank_records,
        net_stats.unique_nets,
        component_stats.reference_records,
        component_stats.pin_records,
        component_stats.reference_pin_records,
        component_stats.unique_references,
        diameter_stats.diameter_records,
        diameter_stats.missing_diameter_records,
        diameter_stats.non_positive_diameter_records,
        geometry_stats.point_records,
        geometry_stats.min_x,
        geometry_stats.max_x,
        geometry_stats.min_y,
        geometry_stats.max_y,
        geometry_stats.min_positive_diameter,
        geometry_stats.max_positive_diameter,
        metadata_stats.access_side_records,
        metadata_stats.feature_type_records,
        metadata_stats.soldermask_records,
        metadata_stats.top_access,
        metadata_stats.bottom_access,
        metadata_stats.both_access,
        metadata_stats.open_soldermask,
        metadata_stats.covered_soldermask,
        metadata_stats.unknown_soldermask
    );

    Ipc356Report {
        source: source.display().to_string(),
        points,
        issues,
        record_stats,
        metadata_stats,
        net_stats,
        component_stats,
        diameter_stats,
        geometry_stats,
        issue_stats,
    }
}

fn parse_record(raw_line: &str) -> Option<Ipc356Point> {
    let line = raw_line.trim();
    if line.contains(' ') {
        parse_loose_record(line).or_else(|| parse_fixed_record(line))
    } else {
        parse_fixed_record(line)
    }
}

fn test_record_code(line: &str) -> Option<Ipc356RecordCode> {
    if line.starts_with("317") {
        Some(Ipc356RecordCode::Code317)
    } else if line.starts_with("327") {
        Some(Ipc356RecordCode::Code327)
    } else if line.starts_with("367") {
        Some(Ipc356RecordCode::Code367)
    } else {
        None
    }
}

fn parse_fixed_record(line: &str) -> Option<Ipc356Point> {
    let net = slice(line, 3, 17)?
        .trim()
        .trim_start_matches('/')
        .to_string();
    let reference = nonempty(slice(line, 20, 26)?.trim());
    let pin = nonempty(slice(line, 27, 31)?.trim());
    let x_marker = line.find('X')?;
    let y_marker = line.find('Y')?;
    let x = parse_ipc_number(take_number(&line[x_marker + 1..])?)?;
    let y = parse_ipc_number(take_number(&line[y_marker + 1..])?)?;
    let diameter = line
        .find("D")
        .and_then(|index| take_number(&line[index + 1..]))
        .and_then(parse_ipc_number);
    let metadata = parse_metadata(
        line.split(|ch: char| ch.is_whitespace())
            .filter(|part| !part.is_empty()),
    );
    Some(Ipc356Point {
        net,
        reference,
        pin,
        location: [x, y],
        diameter,
        access_side: metadata.access_side,
        feature_type: metadata.feature_type,
        soldermask: metadata.soldermask,
    })
}

fn parse_loose_record(line: &str) -> Option<Ipc356Point> {
    let parts = line.split_whitespace().collect::<Vec<_>>();
    let net = parts.get(1)?.trim_start_matches('/').to_string();
    let coordinate_text = parts
        .iter()
        .find(|part| part.starts_with('X') && part.contains('Y'))?;
    let (x, y) = parse_xy_markers(coordinate_text)?;
    let diameter = parts
        .iter()
        .find(|part| part.starts_with('D'))
        .and_then(|part| parse_ipc_number(&part[1..]))
        .or_else(|| {
            coordinate_text
                .find('D')
                .and_then(|index| take_number(&coordinate_text[index + 1..]))
                .and_then(parse_ipc_number)
        });
    let metadata = parse_metadata(parts.iter().copied());
    Some(Ipc356Point {
        net,
        reference: parts.get(2).map(|value| (*value).to_string()),
        pin: parts.get(3).map(|value| (*value).to_string()),
        location: [x, y],
        diameter,
        access_side: metadata.access_side,
        feature_type: metadata.feature_type,
        soldermask: metadata.soldermask,
    })
}

#[derive(Copy, Clone, Debug, Default)]
struct Ipc356Metadata {
    access_side: Option<Ipc356AccessSide>,
    feature_type: Option<Ipc356FeatureType>,
    soldermask: Option<Ipc356Soldermask>,
}

fn parse_metadata<'a>(parts: impl IntoIterator<Item = &'a str>) -> Ipc356Metadata {
    let mut metadata = Ipc356Metadata::default();
    for part in parts {
        let normalized = part
            .trim_matches(|ch: char| ch == ',' || ch == ';')
            .to_ascii_uppercase();
        let value = normalized
            .strip_prefix("ACCESS=")
            .or_else(|| normalized.strip_prefix("SIDE="))
            .or_else(|| normalized.strip_prefix("A="));
        if let Some(value) = value {
            metadata.access_side = parse_access_side(value).or(metadata.access_side);
            continue;
        }

        let value = normalized
            .strip_prefix("FEATURE=")
            .or_else(|| normalized.strip_prefix("TYPE="))
            .or_else(|| normalized.strip_prefix("F="));
        if let Some(value) = value {
            metadata.feature_type = parse_feature_type(value).or(metadata.feature_type);
            continue;
        }

        let value = normalized
            .strip_prefix("MASK=")
            .or_else(|| normalized.strip_prefix("SOLDERMASK="))
            .or_else(|| normalized.strip_prefix("SM="));
        if let Some(value) = value {
            metadata.soldermask = parse_soldermask(value).or(metadata.soldermask);
            continue;
        }

        metadata.access_side = parse_access_side(&normalized).or(metadata.access_side);
        metadata.feature_type = parse_feature_type(&normalized).or(metadata.feature_type);
        metadata.soldermask = parse_soldermask(&normalized).or(metadata.soldermask);
    }
    metadata
}

fn parse_access_side(value: &str) -> Option<Ipc356AccessSide> {
    match value {
        "T" | "TOP" | "FRONT" | "COMPONENT" | "PRIMARY" => Some(Ipc356AccessSide::Top),
        "B" | "BOT" | "BOTTOM" | "BACK" | "SOLDER" | "SECONDARY" => Some(Ipc356AccessSide::Bottom),
        "BOTH" | "EITHER" | "ANY" => Some(Ipc356AccessSide::Both),
        _ => None,
    }
}

fn parse_feature_type(value: &str) -> Option<Ipc356FeatureType> {
    match value {
        "TH" | "THT" | "PTH" | "THROUGH" | "THROUGHHOLE" | "THROUGH-HOLE" => {
            Some(Ipc356FeatureType::ThroughHole)
        }
        "SMD" | "SMT" | "PAD" | "SURFACE" => Some(Ipc356FeatureType::Smd),
        "VIA" | "V" => Some(Ipc356FeatureType::Via),
        "TOOL" | "TOOLING" | "FID" | "FIDUCIAL" => Some(Ipc356FeatureType::Tooling),
        "CONN" | "CONNECTOR" | "EDGE" => Some(Ipc356FeatureType::Connector),
        "OTHER" => Some(Ipc356FeatureType::Other),
        _ => None,
    }
}

fn parse_soldermask(value: &str) -> Option<Ipc356Soldermask> {
    match value {
        "OPEN" | "UNMASKED" | "EXPOSED" | "CLEAR" | "WINDOW" => Some(Ipc356Soldermask::Open),
        "COVERED" | "MASKED" | "TENTED" | "CLOSED" => Some(Ipc356Soldermask::Covered),
        "UNKNOWN" | "NA" | "N/A" => Some(Ipc356Soldermask::Unknown),
        _ => None,
    }
}

fn parse_xy_markers(value: &str) -> Option<(f64, f64)> {
    let x_marker = value.find('X')?;
    let y_marker = value.find('Y')?;
    let x_end = if x_marker < y_marker {
        y_marker
    } else {
        value.len()
    };
    let y_end = value[y_marker + 1..]
        .find(|ch: char| ch == 'X' || ch == 'D' || ch.is_whitespace())
        .map(|offset| y_marker + 1 + offset)
        .unwrap_or(value.len());
    let x = parse_ipc_number(&value[x_marker + 1..x_end])?;
    let y = parse_ipc_number(&value[y_marker + 1..y_end])?;
    Some((x, y))
}

fn slice(value: &str, start: usize, end: usize) -> Option<&str> {
    value.get(start..end)
}

fn nonempty(value: &str) -> Option<String> {
    (!value.is_empty()).then(|| value.to_string())
}

fn take_number(value: &str) -> Option<&str> {
    let end = value
        .find(|ch: char| !(ch.is_ascii_digit() || ch == '+' || ch == '-' || ch == '.'))
        .unwrap_or(value.len());
    (end > 0).then_some(&value[..end])
}

fn parse_ipc_number(value: &str) -> Option<f64> {
    let value = value.trim();
    if value.is_empty() {
        return None;
    }
    if value.contains('.') {
        return value.parse().ok();
    }

    let sign = value.starts_with('-');
    let digits = value.trim_start_matches(['+', '-']);
    if !digits.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }
    if digits.len() <= 4 {
        let parsed = digits.parse::<f64>().ok()? / 1000.0;
        return Some(if sign { -parsed } else { parsed });
    }

    let split = digits.len().saturating_sub(4);
    let mut normalized = String::new();
    if sign {
        normalized.push('-');
    }
    normalized.push_str(&digits[..split]);
    normalized.push('.');
    normalized.push_str(&digits[split..]);
    normalized.parse().ok()
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::{
        Ipc356AccessSide, Ipc356FeatureType, Ipc356IssueKind, Ipc356Soldermask, parse_ipc356,
        parse_ipc356_report,
    };

    #[test]
    fn parses_loose_ipc356_test_record() {
        let points = parse_ipc356("327 /GND U1 1 X010000Y020000D000600\n");

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].net, "GND");
        assert_eq!(points[0].location, [1.0, 2.0]);
        assert_eq!(points[0].diameter, Some(0.06));
    }

    #[test]
    fn parses_optional_access_feature_and_soldermask_metadata() {
        let points = parse_ipc356(
            "327 /USB_D+ J1 2 X010000Y020000D000600 ACCESS=TOP FEATURE=SMD MASK=OPEN\n",
        );

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].access_side, Some(Ipc356AccessSide::Top));
        assert_eq!(points[0].feature_type, Some(Ipc356FeatureType::Smd));
        assert_eq!(points[0].soldermask, Some(Ipc356Soldermask::Open));
    }

    #[test]
    fn ignores_comments_and_unknown_records() {
        let points = parse_ipc356(
            r#"
            C comment
            P parameter
            999 /GND X010000Y020000
            327 missing-coordinates
            "#,
        );

        assert!(points.is_empty());
    }

    #[test]
    fn reports_malformed_recognized_records() {
        let report = parse_ipc356_report(
            r#"
            C comment
            999 /GND X010000Y020000
            327 missing-coordinates
            "#,
            std::path::Path::new("board.ipc"),
        );

        assert!(report.points.is_empty());
        assert_eq!(report.issues.len(), 1);
        assert!(matches!(
            report.issues[0].kind,
            Ipc356IssueKind::MalformedTestRecord
        ));
        assert!(report.issues[0].message().contains("line 4"));
    }

    #[test]
    fn records_parser_issue_summary_counts() {
        let report = parse_ipc356_report(
            "\
317 /GND U1 1 X010000Y020000D000600
327 missing-coordinates
367 malformed
999 ignored-unknown-record
",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.points.len(), 1);
        assert_eq!(report.issues.len(), 2);
        assert_eq!(report.issue_stats.total_issues, 2);
        assert_eq!(report.issue_stats.malformed_test_records, 2);
        assert_eq!(report.record_stats.malformed, 2);
    }

    #[test]
    fn records_recognized_test_record_code_counts() {
        let report = parse_ipc356_report(
            "317 /GND U1 1 X010000Y020000D000600\n327 /VCC U2 2 X030000Y040000D000700\n367 missing-coordinates\n999 /IGNORED X010000Y020000\n",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.points.len(), 2);
        assert_eq!(report.record_stats.code_317, 1);
        assert_eq!(report.record_stats.code_327, 1);
        assert_eq!(report.record_stats.code_367, 1);
        assert_eq!(report.record_stats.malformed, 1);
    }

    #[test]
    fn records_metadata_summary_counts() {
        let report = parse_ipc356_report(
            "\
317 /GND U1 1 X010000Y020000D000600 ACCESS=TOP FEATURE=SMD MASK=OPEN
327 /VCC U2 2 X030000Y040000D000700 ACCESS=BOTTOM FEATURE=VIA MASK=COVERED
327 /PGND TP1 1 X050000Y060000D000800 ACCESS=BOTH FEATURE=TH MASK=UNKNOWN
327 /FID FID1 1 X070000Y080000D000900 FEATURE=TOOLING
327 /EDGE J1 1 X090000Y100000D001000 FEATURE=CONNECTOR MASK=OPEN
327 /MISC U3 3 X110000Y120000D000500 FEATURE=OTHER
327 /NO_META U4 4 X130000Y140000D000500
",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.points.len(), 7);
        assert_eq!(report.metadata_stats.access_side_records, 3);
        assert_eq!(report.metadata_stats.top_access, 1);
        assert_eq!(report.metadata_stats.bottom_access, 1);
        assert_eq!(report.metadata_stats.both_access, 1);
        assert_eq!(report.metadata_stats.feature_type_records, 6);
        assert_eq!(report.metadata_stats.through_hole_features, 1);
        assert_eq!(report.metadata_stats.smd_features, 1);
        assert_eq!(report.metadata_stats.via_features, 1);
        assert_eq!(report.metadata_stats.tooling_features, 1);
        assert_eq!(report.metadata_stats.connector_features, 1);
        assert_eq!(report.metadata_stats.other_features, 1);
        assert_eq!(report.metadata_stats.soldermask_records, 4);
        assert_eq!(report.metadata_stats.open_soldermask, 2);
        assert_eq!(report.metadata_stats.covered_soldermask, 1);
        assert_eq!(report.metadata_stats.unknown_soldermask, 1);
    }

    #[test]
    fn records_net_name_summary_counts() {
        let report = parse_ipc356_report(
            "\
327 /GND U1 1 X010000Y020000D000600
327 /GND U2 2 X030000Y040000D000600
327 /VCC U3 3 X050000Y060000D000600
327 / U4 4 X070000Y080000D000600
",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.points.len(), 4);
        assert_eq!(report.net_stats.named_records, 3);
        assert_eq!(report.net_stats.blank_records, 1);
        assert_eq!(report.net_stats.unique_nets, 2);
    }

    #[test]
    fn records_component_pin_and_diameter_summary_counts() {
        let report = parse_ipc356_report(
            "\
327 /GND U1 1 X010000Y020000D000600
327 /VCC U1 2 X030000Y040000D000000
327 /SIG U2  X050000Y060000D000700
327 /NO_DIAM U3 3 X070000Y080000
",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.points.len(), 4);
        assert_eq!(report.component_stats.reference_records, 4);
        assert_eq!(report.component_stats.pin_records, 4);
        assert_eq!(report.component_stats.reference_pin_records, 4);
        assert_eq!(report.component_stats.unique_references, 3);
        assert_eq!(report.diameter_stats.diameter_records, 3);
        assert_eq!(report.diameter_stats.missing_diameter_records, 1);
        assert_eq!(report.diameter_stats.non_positive_diameter_records, 1);
    }

    #[test]
    fn records_geometry_summary_bounds_and_positive_diameter_range() {
        let report = parse_ipc356_report(
            "\
327 /GND U1 1 X010000Y020000D000600
327 /VCC U2 2 X030000Y010000D000000
327 /SIG U3 3 X005000Y040000D000900
327 /NO_DIAM U4 4 X070000Y080000
",
            std::path::Path::new("board.ipc"),
        );

        assert_eq!(report.geometry_stats.point_records, 4);
        assert_eq!(report.geometry_stats.min_x, Some(0.5));
        assert_eq!(report.geometry_stats.max_x, Some(7.0));
        assert_eq!(report.geometry_stats.min_y, Some(1.0));
        assert_eq!(report.geometry_stats.max_y, Some(8.0));
        assert_eq!(report.geometry_stats.min_positive_diameter, Some(0.06));
        assert_eq!(report.geometry_stats.max_positive_diameter, Some(0.09));
    }

    #[test]
    fn parses_fixed_record_without_panicking() {
        let points = parse_ipc356("327/GND          X010000Y020000D000600\n");

        assert_eq!(points.len(), 1);
        assert!(!points[0].net.is_empty());
        assert!(points[0].location[0].is_finite());
        assert!(points[0].location[1].is_finite());
    }

    proptest! {
        #[test]
        fn arbitrary_ipc356_text_never_panics(input in "\\PC*") {
            let _ = parse_ipc356(&input);
        }

        #[test]
        fn generated_loose_records_have_finite_coordinates(
            net in "[A-Z0-9_.+-]{1,24}",
            x in 0u32..999_999,
            y in 0u32..999_999,
            diameter in 0u32..999_999,
        ) {
            let text = format!("327 /{net} U1 1 X{x:06}Y{y:06}D{diameter:06}\n");
            let points = parse_ipc356(&text);
            prop_assert_eq!(points.len(), 1);
            prop_assert_eq!(points[0].net.as_str(), net.as_str());
            prop_assert!(points[0].location[0].is_finite());
            prop_assert!(points[0].location[1].is_finite());
        }
    }
}
