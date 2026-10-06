//! Serializable report model and report format helpers.
//!
//! Checks return [`Violation`] values, and the application layer collects those
//! findings with source provenance and parser diagnostics into a [`Report`].
//! The same model is used for JSON output and as the source for SARIF, GeoJSON,
//! JUnit, HTML, SVG, and streaming formats.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use crate::io::SourceRecord;
use crate::readiness::CheckCoverage;

/// Report model emitted by a completed run.
#[derive(Debug, Serialize)]
/// Public data model for `Report`.
pub struct Report {
    /// Display paths for Gerber-like layer files that participated in the run.
    pub files: Vec<String>,
    /// Structured provenance records for every source file or generated input.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    /// Field `inputs`.
    pub inputs: Vec<SourceRecord>,
    /// Non-finding parser, loader, or readiness diagnostics.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    /// Field `diagnostics`.
    pub diagnostics: Vec<Diagnostic>,
    /// Explicit outcome for every check selected in this run.
    pub coverage: CheckCoverage,
    /// Number of active, non-waived violations.
    pub violation_count: usize,
    /// Number of violations suppressed by waiver policy.
    pub waived_count: usize,
    /// Findings suppressed by waiver policy, retained for review sinks.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub waived_violations: Vec<Violation>,
    /// Aggregated counts by severity and check.
    pub summary: ReportSummary,
    /// Active findings emitted by checks after waiver filtering.
    pub violations: Vec<Violation>,
}

/// Parser or package diagnostic that is not tied to one geometric finding.
#[derive(Debug, Serialize)]
/// Public data model for `Diagnostic`.
pub struct Diagnostic {
    /// Source file, artifact, or subsystem that produced the diagnostic.
    pub source: String,
    /// Optional one-based source line number.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `line`.
    pub line: Option<usize>,
    /// Severity used by report sinks and CI integrations.
    pub severity: Severity,
    /// Stable machine-readable diagnostic code.
    pub code: String,
    /// Human-readable explanation.
    pub message: String,
}

/// Summary counts for the active report.
#[derive(Debug, Serialize)]
/// Public data model for `ReportSummary`.
pub struct ReportSummary {
    /// Number of active error findings.
    pub errors: usize,
    /// Number of active warning findings.
    pub warnings: usize,
    /// Number of waived findings.
    pub waived: usize,
    /// Active finding count grouped by check id.
    pub checks: Vec<CheckSummary>,
}

/// Per-check count in a report summary.
#[derive(Debug, Serialize)]
/// Public data model for `CheckSummary`.
pub struct CheckSummary {
    /// Stable check identifier.
    pub check: String,
    /// Number of active findings for this check.
    pub count: usize,
}

/// A single design-readiness finding.
#[derive(Debug, Serialize)]
/// Public data model for `Violation`.
pub struct Violation {
    /// Stable hash derived from check, layers, geometry, and locations.
    pub id: String,
    /// Durable SHA-256 identity suitable for release evidence and comparison.
    pub evidence_id: String,
    /// Stable check identifier.
    pub check: String,
    /// Finding severity.
    pub severity: Severity,
    /// Source layers or package roles involved in the finding.
    pub layers: Vec<String>,
    /// Optional geometry island index from checks that split multi-polygons.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `island_index`.
    pub island_index: Option<usize>,
    /// Sum of all polygon areas in square millimeters.
    pub total_area: f64,
    /// Polygon geometry associated with the finding.
    pub polygons: Vec<ViolationPolygon>,
    /// Point geometry associated with the finding.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    /// Field `locations`.
    pub locations: Vec<[f64; 2]>,
    /// Structured semantic objects and source ranges involved in the finding.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub subjects: Vec<FindingSubject>,
    /// Optional human-readable finding details.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `message`.
    pub message: Option<String>,
}

/// Release context bound into a durable finding identity.
#[derive(Clone, Debug, Default, Eq, PartialEq, Serialize)]
pub struct EvidenceContext {
    /// Digest of the unsigned canonical release core, when available.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub release_digest: Option<String>,
    /// Digest of the resolved capability and rule policy, when available.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub policy_digest: Option<String>,
}

/// One exact source position supplied by a native authoring system.
#[derive(Clone, Debug, Eq, Hash, PartialEq, Serialize)]
pub struct FindingSourcePosition {
    /// Zero-based byte offset.
    pub byte: u64,
    /// One-based source line.
    pub line: u32,
    /// One-based source column.
    pub column: u32,
}

/// Half-open source range supplied by a native authoring system.
#[derive(Clone, Debug, Eq, Hash, PartialEq, Serialize)]
pub struct FindingSourceSpan {
    /// File path, URI, or registry locator.
    pub uri: String,
    /// Inclusive start.
    pub start: FindingSourcePosition,
    /// Exclusive end.
    pub end: FindingSourcePosition,
}

/// Structured semantic subject attached to one finding.
#[derive(Clone, Debug, Eq, Hash, PartialEq, Serialize)]
pub struct FindingSubject {
    /// Stable subject family, such as `net`, `instance`, `pin`, or `role`.
    pub kind: String,
    /// Stable identity within that family.
    pub id: String,
    /// Optional source-language range.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<FindingSourceSpan>,
}

/// Polygon geometry serialized with a violation.
#[derive(Debug, Serialize)]
/// Public data model for `ViolationPolygon`.
pub struct ViolationPolygon {
    /// Polygon area in square millimeters.
    pub area: f64,
    /// Exterior ring coordinates.
    pub exterior: Vec<[f64; 2]>,
    /// Interior rings, when present.
    pub holes: Vec<Vec<[f64; 2]>>,
}

/// Severity level used by checks and report sinks.
#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `Severity`.
pub enum Severity {
    /// A release-blocking readiness issue.
    Error,
    /// A non-blocking readiness concern.
    Warning,
}

/// Build aggregate counts for the active report.
pub fn report_summary(violations: &[Violation], waived_count: usize) -> ReportSummary {
    let mut checks = std::collections::BTreeMap::<String, usize>::new();
    let mut errors = 0;
    let mut warnings = 0;

    for violation in violations {
        match violation.severity {
            Severity::Error => errors += 1,
            Severity::Warning => warnings += 1,
        }
        *checks.entry(violation.check.clone()).or_default() += 1;
    }

    ReportSummary {
        errors,
        warnings,
        waived: waived_count,
        checks: checks
            .into_iter()
            .map(|(check, count)| CheckSummary { check, count })
            .collect(),
    }
}

impl Violation {
    /// Create a violation and compute its stable id and total polygon area.
    pub fn new(
        check: impl Into<String>,
        severity: Severity,
        layers: Vec<String>,
        island_index: Option<usize>,
        polygons: Vec<ViolationPolygon>,
        locations: Vec<[f64; 2]>,
        message: Option<String>,
    ) -> Self {
        let check = check.into();
        let total_area = polygons.iter().map(|polygon| polygon.area).sum();
        let id = violation_id(&check, &layers, island_index, &polygons, &locations, &[]);
        let evidence_id = evidence_id(
            &check,
            &layers,
            island_index,
            &polygons,
            &locations,
            &[],
            &EvidenceContext::default(),
        );

        Self {
            id,
            evidence_id,
            check,
            severity,
            layers,
            island_index,
            total_area,
            polygons,
            locations,
            subjects: Vec::new(),
            message,
        }
    }

    /// Attaches structured semantic subjects and includes them in the stable id.
    pub fn with_subjects(mut self, subjects: Vec<FindingSubject>) -> Self {
        self.subjects = subjects;
        self.id = violation_id(
            &self.check,
            &self.layers,
            self.island_index,
            &self.polygons,
            &self.locations,
            &self.subjects,
        );
        self.evidence_id = evidence_id(
            &self.check,
            &self.layers,
            self.island_index,
            &self.polygons,
            &self.locations,
            &self.subjects,
            &EvidenceContext::default(),
        );
        self
    }

    /// Binds this finding to a canonical release and resolved policy context.
    ///
    /// The display/waiver id remains unchanged; detached signatures and release
    /// manifests should use `evidence_id`.
    pub fn with_evidence_context(mut self, context: &EvidenceContext) -> Self {
        self.bind_evidence_context(context);
        self
    }

    /// Rebinds durable identity in place while retaining display geometry/id.
    pub fn bind_evidence_context(&mut self, context: &EvidenceContext) {
        self.evidence_id = evidence_id(
            &self.check,
            &self.layers,
            self.island_index,
            &self.polygons,
            &self.locations,
            &self.subjects,
            context,
        );
    }
}

/// Convert report geometry into a GeoJSON feature collection.
pub fn report_to_geojson(report: &Report) -> Value {
    let features = report
        .violations
        .iter()
        .flat_map(violation_to_features)
        .collect::<Vec<_>>();

    json!({
        "type": "FeatureCollection",
        "features": features,
    })
}

fn violation_to_features(violation: &Violation) -> Vec<Value> {
    let mut features = Vec::new();

    for polygon in &violation.polygons {
        features.push(json!({
            "type": "Feature",
            "properties": feature_properties(violation),
            "geometry": {
                "type": "Polygon",
                "coordinates": polygon_coordinates(polygon),
            },
        }));
    }

    for location in &violation.locations {
        features.push(json!({
            "type": "Feature",
            "properties": feature_properties(violation),
            "geometry": {
                "type": "Point",
                "coordinates": location,
            },
        }));
    }

    features
}

fn feature_properties(violation: &Violation) -> Value {
    json!({
        "id": violation.id,
        "evidence_id": violation.evidence_id,
        "check": violation.check,
        "severity": violation.severity,
        "layers": violation.layers,
        "island_index": violation.island_index,
        "total_area": violation.total_area,
        "message": violation.message,
        "subjects": violation.subjects,
    })
}

fn polygon_coordinates(polygon: &ViolationPolygon) -> Vec<Vec<[f64; 2]>> {
    let mut rings = Vec::with_capacity(polygon.holes.len() + 1);
    rings.push(polygon.exterior.clone());
    rings.extend(polygon.holes.clone());
    rings
}

fn violation_id(
    check: &str,
    layers: &[String],
    island_index: Option<usize>,
    polygons: &[ViolationPolygon],
    locations: &[[f64; 2]],
    subjects: &[FindingSubject],
) -> String {
    let mut hasher = DefaultHasher::new();
    check.hash(&mut hasher);
    layers.hash(&mut hasher);
    island_index.hash(&mut hasher);

    for polygon in polygons {
        quantize(polygon.area).hash(&mut hasher);
        for point in &polygon.exterior {
            quantize_point(*point).hash(&mut hasher);
        }
        for hole in &polygon.holes {
            for point in hole {
                quantize_point(*point).hash(&mut hasher);
            }
        }
    }

    for location in locations {
        quantize_point(*location).hash(&mut hasher);
    }
    subjects.hash(&mut hasher);

    format!("{:016x}", hasher.finish())
}

fn evidence_id(
    check: &str,
    layers: &[String],
    island_index: Option<usize>,
    polygons: &[ViolationPolygon],
    locations: &[[f64; 2]],
    subjects: &[FindingSubject],
    context: &EvidenceContext,
) -> String {
    let mut hasher = Sha256::new();
    digest_field(&mut hasher, b"namespace", b"hyperdrc-evidence-v1");
    digest_field(&mut hasher, b"check", check.as_bytes());
    digest_field(&mut hasher, b"check-version", b"1");
    digest_optional_usize(&mut hasher, b"island-index", island_index);
    digest_optional_string(
        &mut hasher,
        b"release-digest",
        context.release_digest.as_deref(),
    );
    digest_optional_string(
        &mut hasher,
        b"policy-digest",
        context.policy_digest.as_deref(),
    );

    let mut canonical_layers = layers.iter().map(String::as_str).collect::<Vec<_>>();
    canonical_layers.sort_unstable();
    for layer in canonical_layers {
        digest_field(&mut hasher, b"layer", layer.as_bytes());
    }

    let mut canonical_polygons = polygons
        .iter()
        .map(canonical_polygon_bytes)
        .collect::<Vec<_>>();
    canonical_polygons.sort_unstable();
    for polygon in canonical_polygons {
        digest_field(&mut hasher, b"polygon", &polygon);
    }

    let mut canonical_locations = locations
        .iter()
        .map(|location| quantize_point(*location))
        .collect::<Vec<_>>();
    canonical_locations.sort_unstable();
    for [x, y] in canonical_locations {
        let mut bytes = Vec::with_capacity(16);
        bytes.extend_from_slice(&x.to_be_bytes());
        bytes.extend_from_slice(&y.to_be_bytes());
        digest_field(&mut hasher, b"location", &bytes);
    }

    let mut canonical_subjects = subjects.iter().collect::<Vec<_>>();
    canonical_subjects.sort_unstable_by(|left, right| {
        (
            left.kind.as_str(),
            left.id.as_str(),
            source_sort_key(left.source.as_ref()),
        )
            .cmp(&(
                right.kind.as_str(),
                right.id.as_str(),
                source_sort_key(right.source.as_ref()),
            ))
    });
    for subject in canonical_subjects {
        let bytes = serde_json::to_vec(subject)
            .expect("finding subjects are infallibly serializable evidence carriers");
        digest_field(&mut hasher, b"subject", &bytes);
    }

    format!("sha256:{:x}", hasher.finalize())
}

fn canonical_polygon_bytes(polygon: &ViolationPolygon) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&quantize(polygon.area).to_be_bytes());
    digest_ring_bytes(&mut bytes, &polygon.exterior);
    let mut holes = polygon
        .holes
        .iter()
        .map(|hole| {
            let mut encoded = Vec::new();
            digest_ring_bytes(&mut encoded, hole);
            encoded
        })
        .collect::<Vec<_>>();
    holes.sort_unstable();
    for hole in holes {
        bytes.extend_from_slice(&(hole.len() as u64).to_be_bytes());
        bytes.extend_from_slice(&hole);
    }
    bytes
}

fn digest_ring_bytes(bytes: &mut Vec<u8>, ring: &[[f64; 2]]) {
    bytes.extend_from_slice(&(ring.len() as u64).to_be_bytes());
    for point in ring {
        for coordinate in quantize_point(*point) {
            bytes.extend_from_slice(&coordinate.to_be_bytes());
        }
    }
}

fn source_sort_key(source: Option<&FindingSourceSpan>) -> (&str, u64, u32, u32, u64, u32, u32) {
    source.map_or(("", 0, 0, 0, 0, 0, 0), |source| {
        (
            source.uri.as_str(),
            source.start.byte,
            source.start.line,
            source.start.column,
            source.end.byte,
            source.end.line,
            source.end.column,
        )
    })
}

fn digest_field(hasher: &mut Sha256, name: &[u8], value: &[u8]) {
    hasher.update((name.len() as u64).to_be_bytes());
    hasher.update(name);
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value);
}

fn digest_optional_string(hasher: &mut Sha256, name: &[u8], value: Option<&str>) {
    match value {
        Some(value) => digest_field(hasher, name, value.as_bytes()),
        None => digest_field(hasher, name, b"<unbound>"),
    }
}

fn digest_optional_usize(hasher: &mut Sha256, name: &[u8], value: Option<usize>) {
    match value {
        Some(value) => digest_field(hasher, name, &(value as u64).to_be_bytes()),
        None => digest_field(hasher, name, b"<none>"),
    }
}

fn quantize_point(point: [f64; 2]) -> [i64; 2] {
    [quantize(point[0]), quantize(point[1])]
}

fn quantize(value: f64) -> i64 {
    (value * 1_000_000.0).round() as i64
}

#[cfg(test)]
mod tests {
    use super::{
        EvidenceContext, FindingSubject, Report, Severity, Violation, ViolationPolygon,
        report_summary, report_to_geojson,
    };

    #[test]
    fn violation_ids_are_stable_for_identical_input() {
        let left = sample_violation();
        let right = sample_violation();

        assert_eq!(left.id, right.id);
        assert_eq!(left.evidence_id, right.evidence_id);
        assert!(left.evidence_id.starts_with("sha256:"));
    }

    #[test]
    fn durable_evidence_identity_binds_release_and_policy_context() {
        let base = sample_violation();
        let release_a = base.with_evidence_context(&EvidenceContext {
            release_digest: Some("sha256:release-a".into()),
            policy_digest: Some("sha256:prototype".into()),
        });
        let release_b = sample_violation().with_evidence_context(&EvidenceContext {
            release_digest: Some("sha256:release-b".into()),
            policy_digest: Some("sha256:prototype".into()),
        });

        assert_eq!(release_a.id, release_b.id);
        assert_ne!(release_a.evidence_id, release_b.evidence_id);
    }

    #[test]
    fn summary_counts_errors_warnings_waivers_and_checks() {
        let violations = vec![
            sample_violation(),
            Violation::new(
                "point-check",
                Severity::Warning,
                vec!["B.Cu".to_string()],
                None,
                Vec::new(),
                vec![[1.0, 2.0]],
                None,
            ),
        ];

        let summary = report_summary(&violations, 3);

        assert_eq!(summary.errors, 1);
        assert_eq!(summary.warnings, 1);
        assert_eq!(summary.waived, 3);
        assert_eq!(summary.checks.len(), 2);
    }

    #[test]
    fn geojson_contains_polygon_and_point_features() {
        let violations = vec![
            sample_violation(),
            Violation::new(
                "point-check",
                Severity::Warning,
                vec!["B.Cu".to_string()],
                None,
                Vec::new(),
                vec![[1.0, 2.0]],
                None,
            ),
        ];
        let report = Report {
            files: Vec::new(),
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            coverage: Default::default(),
            violation_count: violations.len(),
            waived_count: 0,
            waived_violations: Vec::new(),
            summary: report_summary(&violations, 0),
            violations,
        };

        let geojson = report_to_geojson(&report);
        assert_eq!(geojson["type"], "FeatureCollection");
        assert_eq!(geojson["features"].as_array().unwrap().len(), 2);
    }

    fn sample_violation() -> Violation {
        Violation::new(
            "sample-check",
            Severity::Error,
            vec!["F.Cu".to_string()],
            None,
            vec![ViolationPolygon {
                area: 1.0,
                exterior: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]],
                holes: Vec::new(),
            }],
            Vec::new(),
            Some("sample".to_string()),
        )
    }

    #[test]
    fn structured_subjects_participate_in_stable_finding_identity() {
        let base = Violation::new(
            "semantic-check",
            Severity::Error,
            vec!["F.Cu".into()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            None,
        );
        let left = Violation::new(
            "semantic-check",
            Severity::Error,
            vec!["F.Cu".into()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            None,
        )
        .with_subjects(vec![FindingSubject {
            kind: "instance".into(),
            id: "C1".into(),
            source: None,
        }]);
        let right = Violation::new(
            "semantic-check",
            Severity::Error,
            vec!["F.Cu".into()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            None,
        )
        .with_subjects(vec![FindingSubject {
            kind: "instance".into(),
            id: "C2".into(),
            source: None,
        }]);

        assert_ne!(base.id, left.id);
        assert_ne!(left.id, right.id);
    }
}
