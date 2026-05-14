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

use crate::io::SourceRecord;

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
    /// Number of active, non-waived violations.
    pub violation_count: usize,
    /// Number of violations suppressed by waiver policy.
    pub waived_count: usize,
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
    /// Optional human-readable finding details.
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Field `message`.
    pub message: Option<String>,
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
        let id = violation_id(&check, &layers, island_index, &polygons, &locations);

        Self {
            id,
            check,
            severity,
            layers,
            island_index,
            total_area,
            polygons,
            locations,
            message,
        }
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
        "check": violation.check,
        "severity": violation.severity,
        "layers": violation.layers,
        "island_index": violation.island_index,
        "total_area": violation.total_area,
        "message": violation.message,
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

    format!("{:016x}", hasher.finish())
}

fn quantize_point(point: [f64; 2]) -> [i64; 2] {
    [quantize(point[0]), quantize(point[1])]
}

fn quantize(value: f64) -> i64 {
    (value * 1_000_000.0).round() as i64
}

#[cfg(test)]
mod tests {
    use super::{Report, Severity, Violation, ViolationPolygon, report_summary, report_to_geojson};

    #[test]
    fn violation_ids_are_stable_for_identical_input() {
        let left = sample_violation();
        let right = sample_violation();

        assert_eq!(left.id, right.id);
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
            violation_count: violations.len(),
            waived_count: 0,
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
}
