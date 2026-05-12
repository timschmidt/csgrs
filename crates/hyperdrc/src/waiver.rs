use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::report::Violation;

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
    pub reason: Option<String>,
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

impl Waiver {
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

        let _ = &self.reason;
        true
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use crate::report::{Severity, Violation};

    use super::{Waiver, apply_waivers, load_waivers};

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
}
