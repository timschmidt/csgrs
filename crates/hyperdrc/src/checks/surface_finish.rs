//! Surface-finish compatibility heuristics for release-package notes.
//!
//! These checks operate on normalized README/order text rather than copper
//! geometry. They intentionally report review warnings: the authoritative finish
//! callout should still live in the fabrication drawing or purchase order.

use crate::report::{Severity, Violation};

pub fn readme_surface_finish_compatibility(path: &str, normalized: &str) -> Vec<Violation> {
    let mut violations = Vec::new();

    // IPC-4552B and IPC-4556 describe ENIG/ENEPIG as multifunctional finishes
    // for soldering plus contact, press-fit, and wire-bond use cases. README
    // notes are not a substitute for a fab drawing, but they are often the only
    // machine-readable source of order intent in early release packages.
    if mentions_edge_contact_use(normalized)
        && !has_any(
            normalized,
            &[
                "hard gold",
                "electrolytic gold",
                "edge connector gold",
                "contact gold",
                "enepig",
            ],
        )
    {
        violations.push(surface_finish_violation(
            path,
            "README mentions edge contacts, gold fingers, or card-edge use but does not specify hard/electrolytic contact gold or ENEPIG finish intent",
        ));
    }

    if mentions_fine_pitch_or_array_package(normalized)
        && has_any(normalized, &["hasl", "hot air solder"])
        && !has_any(normalized, &["lf-hasl", "lead-free hasl"])
    {
        violations.push(surface_finish_violation(
            path,
            "README combines HASL finish with fine-pitch, BGA, CSP, LGA, QFN, or DFN assembly language; review finish planarity before release",
        ));
    }

    if has_any(normalized, &["press-fit", "press fit", "pressfit"])
        && has_any(normalized, &["osp", "hasl", "hot air solder"])
        && !has_any(normalized, &["enig", "enepig", "immersion silver", "iag"])
    {
        violations.push(surface_finish_violation(
            path,
            "README mentions press-fit hardware with OSP/HASL-style finish notes but no ENIG, ENEPIG, or immersion-silver contact finish context",
        ));
    }

    if has_any(normalized, &["wire bond", "wirebond", "wire-bond"])
        && !has_any(normalized, &["enig", "enepig", "immersion silver", "iag"])
    {
        violations.push(surface_finish_violation(
            path,
            "README mentions wire bonding but does not specify ENIG, ENEPIG, or immersion-silver finish compatibility",
        ));
    }

    violations
}

fn mentions_edge_contact_use(text: &str) -> bool {
    has_any(
        text,
        &[
            "gold finger",
            "gold fingers",
            "card edge",
            "card-edge",
            "edge connector",
            "edge contacts",
            "contact fingers",
            "keypad contact",
            "membrane switch",
            "zif contact",
        ],
    )
}

fn mentions_fine_pitch_or_array_package(text: &str) -> bool {
    has_any(
        text,
        &[
            "fine pitch",
            "fine-pitch",
            "bga",
            "csp",
            "lga",
            "qfn",
            "dfn",
            "0.5mm pitch",
            "0.4mm pitch",
            "0.3mm pitch",
        ],
    )
}

fn has_any(haystack: &str, needles: &[&str]) -> bool {
    needles.iter().any(|needle| haystack.contains(needle))
}

fn surface_finish_violation(layer: &str, message: &str) -> Violation {
    Violation::new(
        "production-artifact-readiness",
        Severity::Warning,
        vec![layer.to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(message.to_string()),
    )
}
