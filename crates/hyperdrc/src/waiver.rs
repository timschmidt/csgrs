use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::report::{Severity, Violation};

#[derive(Debug, Deserialize)]
pub struct WaiverFile {
    #[serde(default)]
    pub waivers: Vec<Waiver>,
}

#[derive(Debug, Deserialize)]
pub struct Waiver {
    pub id: Option<String>,
    pub check: Option<String>,
    #[serde(default)]
    pub layers: Vec<String>,
    pub message_contains: Option<String>,
    #[serde(default)]
    pub reason: Option<String>,
    #[serde(default)]
    pub owner: Option<String>,
    #[serde(default)]
    pub review_date: Option<String>,
    #[serde(default)]
    pub source: Option<String>,
    #[serde(default)]
    pub geometry_hash: Option<String>,
}

pub fn load_waivers(path: &Path) -> Result<Vec<Waiver>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let file: WaiverFile = serde_json::from_str(&text)
        .with_context(|| format!("failed to parse waiver file {}", path.display()))?;
    Ok(file.waivers)
}

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
/// Waiver entries are operational exceptions and should remain auditable in CI.
/// These checks generate non-fatal warnings that are always run when waivers are
/// provided, even if the waivers are not used.
pub fn governance_violations(waivers: &[Waiver]) -> Vec<Violation> {
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

        if !waiver.has_text(&waiver.review_date) {
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

    use super::{Waiver, apply_waivers, governance_violations, load_waivers};

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
            review_date: Some("2026-05-01".to_string()),
            source: Some("https://jira.example/issues/123".to_string()),
            geometry_hash: Some("sha256:0000".to_string()),
        }];

        assert!(governance_violations(&waivers).is_empty());
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
}
