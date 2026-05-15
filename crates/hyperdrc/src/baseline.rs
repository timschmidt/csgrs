//! Waiver-stub and baseline report generation.
//!
//! These sinks are intentionally separate from waiver matching. Matching decides
//! whether an existing exception suppresses a finding; baseline generation
//! records the active finding set so reviewers can create controlled waivers or
//! compare release-to-release drift without changing the current run result.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

use crate::report::{Report, Severity, Violation};

const GEOMETRY_HASH_OFFSET: u64 = 0xcbf29ce484222325;
const GEOMETRY_HASH_PRIME: u64 = 0x00000100000001b3;
const GEOMETRY_HASH_SCALE: f64 = 1_000_000.0;

/// File containing generated waiver templates.
#[derive(Debug, Deserialize, Serialize)]
/// Public data model for `WaiverStubFile`.
pub struct WaiverStubFile {
    /// Generated waiver templates.
    pub waivers: Vec<WaiverStub>,
}

/// Waiver template generated from an active finding.
#[derive(Debug, Deserialize, Serialize)]
/// Public data model for `WaiverStub`.
pub struct WaiverStub {
    /// Finding id to target.
    pub id: String,
    /// Check identifier to target.
    pub check: String,
    /// Layers involved in the finding.
    pub layers: Vec<String>,
    /// Optional message excerpt for additional targeting.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `message_contains`.
    pub message_contains: Option<String>,
    /// Placeholder for the accepted-risk rationale.
    pub reason: String,
    /// Placeholder for the responsible reviewer.
    pub owner: String,
    /// Placeholder review date in `YYYY-MM-DD` format.
    pub review_date: String,
    /// Placeholder review source such as an ECO, ticket, or note.
    pub source: String,
    /// Stable geometry identity copied from the finding.
    pub geometry_hash: String,
}

/// Serialized baseline of active findings for release-to-release comparison.
#[derive(Debug, Clone, Deserialize, Serialize)]
/// Public data model for `BaselineFile`.
pub struct BaselineFile {
    /// Active findings captured in the baseline.
    pub findings: Vec<BaselineFinding>,
}

/// Stable baseline representation of one finding.
#[derive(Debug, Clone, Deserialize, Serialize)]
/// Public data model for `BaselineFinding`.
pub struct BaselineFinding {
    /// Finding id from the report.
    pub id: String,
    /// Check identifier.
    pub check: String,
    /// Finding severity.
    pub severity: Severity,
    /// Layers involved in the finding.
    pub layers: Vec<String>,
    /// Stable geometry identity for diffing.
    pub geometry_hash: String,
    /// Optional finding detail text.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `message`.
    pub message: Option<String>,
    /// Number of polygon geometries associated with the finding.
    pub polygon_count: usize,
    /// Number of point locations associated with the finding.
    pub point_count: usize,
    /// Total polygon area in square millimeters.
    pub total_area: f64,
}

/// Baseline comparison result.
#[derive(Debug, Serialize)]
/// Public data model for `BaselineDiffFile`.
pub struct BaselineDiffFile {
    /// Aggregate diff counts.
    pub summary: BaselineDiffSummary,
    /// Findings present in current but absent from reference.
    pub new_findings: Vec<BaselineFinding>,
    /// Findings present in reference but absent from current.
    pub resolved_findings: Vec<BaselineFinding>,
    /// Findings present in both baselines.
    pub unchanged_findings: Vec<BaselineFinding>,
}

/// Aggregate baseline diff counts.
#[derive(Debug, Serialize)]
/// Public data model for `BaselineDiffSummary`.
pub struct BaselineDiffSummary {
    /// Number of reference findings.
    pub reference_findings: usize,
    /// Number of current findings.
    pub current_findings: usize,
    /// Number of newly introduced findings.
    pub new_findings: usize,
    /// Number of resolved findings.
    pub resolved_findings: usize,
    /// Number of unchanged findings.
    pub unchanged_findings: usize,
}

/// Generate waiver templates for every active finding in a report.
pub fn report_to_waiver_stubs(report: &Report) -> WaiverStubFile {
    WaiverStubFile {
        waivers: report.violations.iter().map(waiver_stub).collect(),
    }
}

/// Generate a baseline file from a report's active findings.
pub fn report_to_baseline(report: &Report) -> BaselineFile {
    BaselineFile {
        findings: report.violations.iter().map(baseline_finding).collect(),
    }
}

/// Load a baseline file from JSON.
pub fn load_baseline(path: &Path) -> Result<BaselineFile> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    serde_json::from_str(&text)
        .with_context(|| format!("failed to parse baseline file {}", path.display()))
}

/// Compare reference and current baselines by check and geometry hash.
pub fn compare_baselines(reference: &BaselineFile, current: &BaselineFile) -> BaselineDiffFile {
    let reference_by_key = keyed_findings(&reference.findings);
    let current_by_key = keyed_findings(&current.findings);
    let reference_keys = reference_by_key.keys().cloned().collect::<BTreeSet<_>>();
    let current_keys = current_by_key.keys().cloned().collect::<BTreeSet<_>>();

    let new_findings = current_keys
        .difference(&reference_keys)
        .flat_map(|key| current_by_key.get(key))
        .cloned()
        .collect::<Vec<_>>();
    let resolved_findings = reference_keys
        .difference(&current_keys)
        .flat_map(|key| reference_by_key.get(key))
        .cloned()
        .collect::<Vec<_>>();
    let unchanged_findings = current_keys
        .intersection(&reference_keys)
        .flat_map(|key| current_by_key.get(key))
        .cloned()
        .collect::<Vec<_>>();

    BaselineDiffFile {
        summary: BaselineDiffSummary {
            reference_findings: reference.findings.len(),
            current_findings: current.findings.len(),
            new_findings: new_findings.len(),
            resolved_findings: resolved_findings.len(),
            unchanged_findings: unchanged_findings.len(),
        },
        new_findings,
        resolved_findings,
        unchanged_findings,
    }
}

fn waiver_stub(violation: &Violation) -> WaiverStub {
    WaiverStub {
        id: violation.id.clone(),
        check: violation.check.clone(),
        layers: violation.layers.clone(),
        message_contains: violation.message.as_ref().map(|message| {
            message
                .chars()
                .take(120)
                .collect::<String>()
                .trim()
                .to_string()
        }),
        reason: "TODO: document accepted risk or manufacturing disposition".to_string(),
        owner: "TODO".to_string(),
        review_date: "TODO: YYYY-MM-DD".to_string(),
        source: "TODO: review ticket, ECO, or fabrication note".to_string(),
        geometry_hash: geometry_hash(violation),
    }
}

fn baseline_finding(violation: &Violation) -> BaselineFinding {
    BaselineFinding {
        id: violation.id.clone(),
        check: violation.check.clone(),
        severity: violation.severity,
        layers: violation.layers.clone(),
        geometry_hash: geometry_hash(violation),
        message: violation.message.clone(),
        polygon_count: violation.polygons.len(),
        point_count: violation.locations.len(),
        total_area: violation.total_area,
    }
}

fn keyed_findings(findings: &[BaselineFinding]) -> BTreeMap<String, BaselineFinding> {
    findings
        .iter()
        .map(|finding| (finding_key(finding), finding.clone()))
        .collect()
}

fn finding_key(finding: &BaselineFinding) -> String {
    // Use the check name with the geometry hash rather than the human message:
    // review text can change as diagnostics improve, while the finding location
    // and rule identity are the stable release-management signal.
    format!("{}|{}", finding.check, finding.geometry_hash)
}

fn geometry_hash(violation: &Violation) -> String {
    let mut hash = GeometryHasher::new();
    hash.write_str(&violation.check);
    hash.write_usize(violation.layers.len());
    for layer in &violation.layers {
        hash.write_str(layer);
    }
    match violation.island_index {
        Some(island_index) => {
            hash.write_u8(1);
            hash.write_usize(island_index);
        }
        None => hash.write_u8(0),
    }
    hash.write_usize(violation.polygons.len());
    for polygon in &violation.polygons {
        hash.write_i64(quantize_geometry_value(polygon.area));
        hash.write_usize(polygon.exterior.len());
        for point in &polygon.exterior {
            hash.write_point(*point);
        }
        hash.write_usize(polygon.holes.len());
        for hole in &polygon.holes {
            hash.write_usize(hole.len());
            for point in hole {
                hash.write_point(*point);
            }
        }
    }
    hash.write_usize(violation.locations.len());
    for location in &violation.locations {
        hash.write_point(*location);
    }

    // IEEE 828-2012 treats baseline identity and status accounting as part of
    // configuration management. Use an explicit FNV-1a byte stream rather than
    // Rust's DefaultHasher so generated waiver fingerprints remain stable
    // across toolchain releases and can be reviewed as production artifacts.
    format!("hyperdrc-geometry-v1:{:016x}", hash.finish())
}

struct GeometryHasher {
    state: u64,
}

impl GeometryHasher {
    fn new() -> Self {
        Self {
            state: GEOMETRY_HASH_OFFSET,
        }
    }

    fn finish(self) -> u64 {
        self.state
    }

    fn write_u8(&mut self, value: u8) {
        self.write_bytes(&[value]);
    }

    fn write_usize(&mut self, value: usize) {
        self.write_bytes(&(value as u64).to_le_bytes());
    }

    fn write_i64(&mut self, value: i64) {
        self.write_bytes(&value.to_le_bytes());
    }

    fn write_point(&mut self, point: [f64; 2]) {
        self.write_i64(quantize_geometry_value(point[0]));
        self.write_i64(quantize_geometry_value(point[1]));
    }

    fn write_str(&mut self, value: &str) {
        self.write_usize(value.len());
        self.write_bytes(value.as_bytes());
    }

    fn write_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.state ^= u64::from(*byte);
            self.state = self.state.wrapping_mul(GEOMETRY_HASH_PRIME);
        }
    }
}

fn quantize_geometry_value(value: f64) -> i64 {
    if !value.is_finite() {
        return 0;
    }
    (value * GEOMETRY_HASH_SCALE).round() as i64
}

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, ViolationPolygon, report_summary};

    use super::{compare_baselines, geometry_hash, report_to_baseline, report_to_waiver_stubs};

    #[test]
    fn waiver_stubs_preserve_scope_and_governance_placeholders() {
        let report = sample_report();

        let stubs = report_to_waiver_stubs(&report);

        assert_eq!(stubs.waivers.len(), 1);
        assert_eq!(stubs.waivers[0].check, "acid-trap-candidate");
        assert_eq!(stubs.waivers[0].layers, vec!["F.Cu"]);
        assert!(
            stubs.waivers[0]
                .geometry_hash
                .starts_with("hyperdrc-geometry-v1:")
        );
        assert!(stubs.waivers[0].reason.contains("TODO"));
    }

    #[test]
    fn geometry_hash_ignores_message_but_tracks_geometry() {
        let base = polygon_violation("first diagnostic", [[0.0, 0.0], [1.0, 0.0]]);
        let same_geometry = polygon_violation("reworded diagnostic", [[0.0, 0.0], [1.0, 0.0]]);
        let moved_geometry = polygon_violation("first diagnostic", [[0.0, 0.0], [2.0, 0.0]]);

        assert_eq!(geometry_hash(&base), geometry_hash(&same_geometry));
        assert_ne!(geometry_hash(&base), geometry_hash(&moved_geometry));
    }

    #[test]
    fn baseline_records_active_finding_summary() {
        let report = sample_report();

        let baseline = report_to_baseline(&report);

        assert_eq!(baseline.findings.len(), 1);
        assert_eq!(baseline.findings[0].severity, Severity::Warning);
        assert_eq!(baseline.findings[0].point_count, 1);
        assert_eq!(baseline.findings[0].total_area, 0.0);
    }

    #[test]
    fn baseline_diff_classifies_new_resolved_and_unchanged_findings() {
        let first = baseline_finding("old-check", "old-id");
        let shared_reference = baseline_finding("shared-check", "shared-id");
        let shared_current = baseline_finding("shared-check", "shared-id");
        let new = baseline_finding("new-check", "new-id");
        let reference = super::BaselineFile {
            findings: vec![first, shared_reference],
        };
        let current = super::BaselineFile {
            findings: vec![shared_current, new],
        };

        let diff = compare_baselines(&reference, &current);

        assert_eq!(diff.summary.reference_findings, 2);
        assert_eq!(diff.summary.current_findings, 2);
        assert_eq!(diff.summary.new_findings, 1);
        assert_eq!(diff.summary.resolved_findings, 1);
        assert_eq!(diff.summary.unchanged_findings, 1);
        assert_eq!(diff.new_findings[0].check, "new-check");
        assert_eq!(diff.resolved_findings[0].check, "old-check");
        assert_eq!(diff.unchanged_findings[0].check, "shared-check");
    }

    fn sample_report() -> Report {
        let violations = vec![Violation::new(
            "acid-trap-candidate",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            Some("acute copper vertex below threshold".to_string()),
        )];
        Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: 0,
            summary: report_summary(&violations, 0),
            violations,
        }
    }

    fn baseline_finding(check: &str, id: &str) -> super::BaselineFinding {
        super::BaselineFinding {
            id: id.to_string(),
            check: check.to_string(),
            severity: Severity::Warning,
            layers: vec!["F.Cu".to_string()],
            geometry_hash: format!("hyperdrc-geometry-v1:{id}"),
            message: Some("sample".to_string()),
            polygon_count: 0,
            point_count: 1,
            total_area: 0.0,
        }
    }

    fn polygon_violation(message: &str, exterior_prefix: [[f64; 2]; 2]) -> Violation {
        Violation::new(
            "copper-overlap",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            Some(0),
            vec![ViolationPolygon {
                area: 1.0,
                exterior: vec![
                    exterior_prefix[0],
                    exterior_prefix[1],
                    [1.0, 1.0],
                    [0.0, 1.0],
                    exterior_prefix[0],
                ],
                holes: Vec::new(),
            }],
            vec![[0.5, 0.5]],
            Some(message.to_string()),
        )
    }
}
