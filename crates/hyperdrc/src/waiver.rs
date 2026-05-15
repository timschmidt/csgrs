//! Waiver loading, matching, and governance checks.
//!
//! Waivers suppress known findings only when their scope matches. Separate
//! governance checks keep those suppressions reviewable by requiring ownership,
//! reason, review date, source, and geometry hash metadata.

use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::date::{current_day_number, parse_iso_day};
use crate::report::{Severity, Violation};

/// Top-level JSON waiver file.
#[derive(Debug, Deserialize)]
/// Public data model for `WaiverFile`.
pub struct WaiverFile {
    /// Waiver entries loaded from JSON.
    #[serde(default)]
    /// Field `waivers`.
    pub waivers: Vec<Waiver>,
}

/// One waiver entry from a waiver policy file.
#[derive(Debug, Deserialize)]
/// Public data model for `Waiver`.
pub struct Waiver {
    /// Optional exact violation id.
    pub id: Option<String>,
    /// Optional check identifier.
    pub check: Option<String>,
    /// Optional layer names that must all be present on the violation.
    #[serde(default)]
    /// Field `layers`.
    pub layers: Vec<String>,
    /// Optional substring that must appear in the violation message.
    pub message_contains: Option<String>,
    /// Reviewable reason for accepting the finding.
    #[serde(default)]
    /// Field `reason`.
    pub reason: Option<String>,
    /// Person or team responsible for re-review.
    #[serde(default)]
    /// Field `owner`.
    pub owner: Option<String>,
    /// Review date in `YYYY-MM-DD` format.
    #[serde(default)]
    /// Field `review_date`.
    pub review_date: Option<String>,
    /// Ticket, ECO, drawing note, or other source reference.
    #[serde(default)]
    /// Field `source`.
    pub source: Option<String>,
    /// Stable geometry hash expected for the waived finding.
    #[serde(default)]
    /// Field `geometry_hash`.
    pub geometry_hash: Option<String>,
}

/// Load waiver entries from a JSON file.
pub fn load_waivers(path: &Path) -> Result<Vec<Waiver>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let file: WaiverFile = serde_json::from_str(&text)
        .with_context(|| format!("failed to parse waiver file {}", path.display()))?;
    Ok(file.waivers)
}

/// Split violations into active and waived sets.
pub fn apply_waivers(
    violations: Vec<Violation>,
    waivers: &[Waiver],
) -> (Vec<Violation>, Vec<Violation>) {
    let mut active = Vec::new();
    let mut waived = Vec::new();

    for violation in violations {
        if waivers.iter().any(|waiver| waiver.matches(&violation)) {
            waived.push(violation);
        } else {
            active.push(violation);
        }
    }

    (active, waived)
}

/// Validates waiver governance metadata before review acceptance.
///
/// Waiver entries are operational exceptions and should remain auditable in CI:
/// IEEE 828-2012, "IEEE Standard for Configuration Management in Systems and
/// Software Engineering," frames baselines, change control, and status
/// accounting as configuration-management activities. This check applies that
/// model to DRC suppressions by requiring scope, reason, ownership, review date,
/// source, and geometry-hash evidence before a waiver becomes a durable release
/// artifact.
pub fn governance_violations(waivers: &[Waiver]) -> Vec<Violation> {
    governance_violations_on(waivers, current_day_number())
}

fn governance_violations_on(waivers: &[Waiver], today: Option<i64>) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (index, waiver) in waivers.iter().enumerate() {
        let label = waiver_scope_label(index, waiver);

        if !waiver.has_scope() {
            violations.push(Violation::new(
                "waiver-governance-scope",
                Severity::Warning,
                vec!["waiver".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!(
                    "{label} has no targeting scope; set one of id, check, layers, or message_contains"
                )),
            ));
        }

        if !waiver.has_text(&waiver.reason) {
            violations.push(Violation::new(
                "waiver-governance-reason",
                Severity::Warning,
                vec!["waiver".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!("{label} is missing a reason")),
            ));
        }

        if !waiver.has_text(&waiver.owner) {
            violations.push(Violation::new(
                "waiver-governance-owner",
                Severity::Warning,
                vec!["waiver".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!("{label} is missing an owner")),
            ));
        }

        match waiver.review_date_status(today) {
            ReviewDateStatus::Missing => {
                violations.push(Violation::new(
                    "waiver-governance-review-date",
                    Severity::Warning,
                    vec!["waiver".to_string()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!("{label} is missing a review_date")),
                ));
            }
            ReviewDateStatus::Malformed(value) => {
                violations.push(Violation::new(
                    "waiver-governance-review-date-format",
                    Severity::Warning,
                    vec!["waiver".to_string()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!(
                        "{label} has invalid review_date {value:?}; use YYYY-MM-DD"
                    )),
                ));
            }
            ReviewDateStatus::Expired(value) => {
                violations.push(Violation::new(
                    "waiver-governance-review-date-expired",
                    Severity::Warning,
                    vec!["waiver".to_string()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!("{label} review_date {value} has expired")),
                ));
            }
            ReviewDateStatus::Current => {}
        }

        if !waiver.has_text(&waiver.source) {
            violations.push(Violation::new(
                "waiver-governance-source",
                Severity::Warning,
                vec!["waiver".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!("{label} is missing a source link")),
            ));
        }

        if !waiver.has_text(&waiver.geometry_hash) {
            violations.push(Violation::new(
                "waiver-governance-geometry-hash",
                Severity::Warning,
                vec!["waiver".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!("{label} is missing a geometry_hash")),
            ));
        }
    }

    violations
}

enum ReviewDateStatus {
    Current,
    Expired(String),
    Malformed(String),
    Missing,
}

impl Waiver {
    /// Matching supports progressively selective targeting:
    /// id > check > layers > message_contains. Unset fields are ignored.
    fn matches(&self, violation: &Violation) -> bool {
        if let Some(id) = &self.id
            && id != &violation.id
        {
            return false;
        }

        if let Some(check) = &self.check
            && check != &violation.check
        {
            return false;
        }

        if !self.layers.is_empty()
            && !self
                .layers
                .iter()
                .all(|layer| violation.layers.iter().any(|value| value == layer))
        {
            return false;
        }

        if let Some(message_contains) = &self.message_contains
            && !violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains(message_contains))
        {
            return false;
        }

        true
    }

    fn has_scope(&self) -> bool {
        self.id.is_some()
            || self.check.is_some()
            || !self.layers.is_empty()
            || self.message_contains.is_some()
    }

    fn has_text(&self, text: &Option<String>) -> bool {
        text.as_ref().is_some_and(|value| !value.trim().is_empty())
    }

    fn review_date_status(&self, today: Option<i64>) -> ReviewDateStatus {
        let Some(review_date) = self.review_date.as_deref().map(str::trim) else {
            return ReviewDateStatus::Missing;
        };
        if review_date.is_empty() {
            return ReviewDateStatus::Missing;
        }
        let Some(review_day) = parse_iso_day(review_date) else {
            return ReviewDateStatus::Malformed(review_date.to_string());
        };
        if today.is_some_and(|today| review_day < today) {
            return ReviewDateStatus::Expired(review_date.to_string());
        }

        ReviewDateStatus::Current
    }
}

fn waiver_scope_label(index: usize, waiver: &Waiver) -> String {
    if let Some(id) = &waiver.id {
        return format!("Waiver #{}, id={id}", index + 1);
    }
    if let Some(check) = &waiver.check {
        return format!("Waiver #{}, check={check}", index + 1);
    }
    if !waiver.layers.is_empty() {
        return format!("Waiver #{}, layers={}", index + 1, waiver.layers.join(", "));
    }
    if let Some(message_contains) = &waiver.message_contains {
        return format!("Waiver #{}, message_contains={message_contains}", index + 1);
    }

    format!("Waiver #{}", index + 1)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use crate::report::{Severity, Violation};

    use crate::date::parse_iso_day;

    use super::{
        Waiver, apply_waivers, governance_violations, governance_violations_on, load_waivers,
    };

    #[test]
    fn applies_check_and_layer_waivers() {
        let violation = Violation::new(
            "acid-trap-candidate",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            Some("acute vertex".to_string()),
        );
        let waiver = Waiver {
            id: None,
            check: Some("acid-trap-candidate".to_string()),
            layers: vec!["F.Cu".to_string()],
            message_contains: Some("acute".to_string()),
            reason: Some("accepted".to_string()),
            owner: None,
            review_date: None,
            source: None,
            geometry_hash: None,
        };

        let (active, waived) = apply_waivers(vec![violation], &[waiver]);

        assert!(active.is_empty());
        assert_eq!(waived.len(), 1);
    }

    #[test]
    fn non_matching_waiver_leaves_violation_active() {
        let violation = Violation::new(
            "acid-trap-candidate",
            Severity::Warning,
            vec!["F.Cu".to_string()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            Some("acute vertex".to_string()),
        );
        let waiver = Waiver {
            id: None,
            check: Some("different-check".to_string()),
            layers: Vec::new(),
            message_contains: None,
            reason: None,
            owner: None,
            review_date: None,
            source: None,
            geometry_hash: None,
        };

        let (active, waived) = apply_waivers(vec![violation], &[waiver]);

        assert_eq!(active.len(), 1);
        assert!(waived.is_empty());
    }

    #[test]
    fn rejects_malformed_waiver_json() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-bad-waiver-{}.json", std::process::id()));
        fs::write(&path, "{bad").unwrap();

        let result = load_waivers(&path);

        assert!(result.is_err());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn governance_violations_require_scope_and_metadata() {
        let waivers = vec![Waiver {
            id: None,
            check: None,
            layers: Vec::new(),
            message_contains: None,
            reason: None,
            owner: None,
            review_date: None,
            source: None,
            geometry_hash: None,
        }];

        let violations = governance_violations(&waivers);

        assert_eq!(violations.len(), 6);
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-scope")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-reason")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-owner")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-review-date")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-source")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-geometry-hash")
        );
    }

    #[test]
    fn governance_violations_allow_complete_waiver_metadata() {
        let waivers = vec![Waiver {
            id: Some("acid-trap-candidate".to_string()),
            check: None,
            layers: Vec::new(),
            message_contains: None,
            reason: Some("accept known DNP footprint".to_string()),
            owner: Some("PCB platform team".to_string()),
            review_date: Some("2027-05-01".to_string()),
            source: Some("https://jira.example/issues/123".to_string()),
            geometry_hash: Some("sha256:0000".to_string()),
        }];

        assert!(governance_violations(&waivers).is_empty());
    }

    #[test]
    fn governance_violations_report_malformed_and_expired_review_dates() {
        let waivers = vec![
            complete_waiver_with_review_date("2026-05-12"),
            complete_waiver_with_review_date("2026-02-30"),
            complete_waiver_with_review_date("2026/05/14"),
            complete_waiver_with_review_date("2026-05-13"),
            complete_waiver_with_review_date("2026-05-14"),
        ];

        let today = parse_iso_day("2026-05-13");
        let violations = governance_violations_on(&waivers, today);

        assert_eq!(violations.len(), 3);
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-review-date-expired")
        );
        assert_eq!(
            violations
                .iter()
                .filter(|violation| violation.check == "waiver-governance-review-date-format")
                .count(),
            2
        );
    }

    #[test]
    fn governance_violations_treat_blank_fields_as_missing() {
        let waivers = vec![Waiver {
            id: Some("acid-trap-candidate".to_string()),
            check: None,
            layers: Vec::new(),
            message_contains: None,
            reason: Some("   ".to_string()),
            owner: Some(String::new()),
            review_date: None,
            source: Some("".to_string()),
            geometry_hash: Some("   ".to_string()),
        }];

        let violations = governance_violations(&waivers);

        assert_eq!(violations.len(), 5);
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-owner")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-review-date")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-source")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-reason")
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-geometry-hash")
        );
        assert!(
            !violations
                .iter()
                .any(|violation| violation.check == "waiver-governance-scope")
        );
    }

    fn complete_waiver_with_review_date(review_date: &str) -> Waiver {
        Waiver {
            id: Some("acid-trap-candidate".to_string()),
            check: None,
            layers: Vec::new(),
            message_contains: None,
            reason: Some("accept known footprint".to_string()),
            owner: Some("PCB platform team".to_string()),
            review_date: Some(review_date.to_string()),
            source: Some("https://jira.example/issues/123".to_string()),
            geometry_hash: Some("sha256:0000".to_string()),
        }
    }
}
