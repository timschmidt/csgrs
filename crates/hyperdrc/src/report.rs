use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use serde::Serialize;
use serde_json::{Value, json};

#[derive(Debug, Serialize)]
pub struct Report {
    pub files: Vec<String>,
    pub violation_count: usize,
    pub waived_count: usize,
    pub summary: ReportSummary,
    pub violations: Vec<Violation>,
}

#[derive(Debug, Serialize)]
pub struct ReportSummary {
    pub errors: usize,
    pub warnings: usize,
    pub waived: usize,
    pub checks: Vec<CheckSummary>,
}

#[derive(Debug, Serialize)]
pub struct CheckSummary {
    pub check: String,
    pub count: usize,
}

#[derive(Debug, Serialize)]
pub struct Violation {
    pub id: String,
    pub check: String,
    pub severity: Severity,
    pub layers: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub island_index: Option<usize>,
    pub total_area: f64,
    pub polygons: Vec<ViolationPolygon>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub locations: Vec<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct ViolationPolygon {
    pub area: f64,
    pub exterior: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum Severity {
    Error,
    Warning,
}

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
