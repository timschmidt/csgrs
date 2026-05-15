//! Package-level handoff language detectors for sidecar artifact checks.
//!
//! `artifacts.rs` owns table parsing and cross-file parity. This module keeps
//! release-note vocabulary that describes assembly package handoffs separate
//! from CSV/TSV parsing mechanics.

/// Height above which a populated BOM row needs explicit mechanical-height
/// handoff language rather than only row-local metadata.
pub(super) const TALL_COMPONENT_HEIGHT_MM: f64 = 5.0;

pub(super) fn mentions_reflow_profile_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "reflow profile",
            "thermal profile",
            "oven profile",
            "oven recipe",
            "reflow recipe",
            "peak temperature",
            "peak temp",
            "soak",
            "ramp rate",
            "time above liquidus",
            "tal",
            "profile validation",
            "thermal profiling",
        ],
    )
}

pub(super) fn mentions_height_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "component height",
            "max height",
            "maximum height",
            "height limit",
            "z-height",
            "z height",
            "tall component",
            "enclosure clearance",
            "mechanical clearance",
            "mechanical keepout",
            "keepout height",
            "mating height",
            "connector height",
            "battery clearance",
        ],
    )
}

pub(super) fn mentions_thermal_validation_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "thermal validation",
            "thermal review",
            "thermal analysis",
            "thermal simulation",
            "thermal test",
            "temperature rise",
            "temp rise",
            "derating",
            "power dissipation",
            "junction temperature",
            "theta ja",
            "theta-ja",
            "heatsink",
            "heat sink",
            "thermal interface",
            "thermal interface material",
            "tim pad",
            "tim grease",
            "tim sheet",
            "airflow",
            "thermal pad",
            "thermal via",
            "copper pour heat spreader",
            "heat spreader",
        ],
    )
}

pub(super) fn mentions_low_standoff_cleanliness_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "no-clean",
            "no clean",
            "cleanliness",
            "flux residue",
            "flux residues",
            "wash process",
            "aqueous wash",
            "ionic contamination",
            "ionic cleanliness",
            "conformal coating cleanliness",
            "under-component cleaning",
            "low standoff",
            "low-standoff",
        ],
    )
}

pub(super) fn mentions_press_fit_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "press-fit",
            "press fit",
            "pressfit",
            "compliant pin",
            "compliant-pin",
            "insertion force",
            "push-in force",
            "push out force",
            "push-out force",
            "finished hole",
            "hole tolerance",
            "plated through hole tolerance",
            "pth tolerance",
            "press tooling",
            "support fixture",
            "connector press",
        ],
    )
}

pub(super) fn mentions_wire_bond_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "wire bond",
            "wire-bond",
            "wirebond",
            "bond pad",
            "bondable",
            "chip on board",
            "chip-on-board",
            "cob assembly",
            "bare die",
            "die attach",
            "glob top",
            "dam and fill",
            "bond pull",
            "pull test",
            "bond diagram",
            "bond map",
            "loop height",
            "soft gold",
            "enepig",
        ],
    )
}

pub(super) fn mentions_fabrication_marking_zone_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "marking zone",
            "marking zones",
            "allowed marking",
            "allowed zone",
            "label location",
            "label area",
            "date-code location",
            "date code location",
            "ul mark location",
            "ul logo location",
            "barcode location",
            "qr code location",
            "revision text location",
            "fab drawing label",
            "fab drawing marking",
            "fabrication drawing label",
            "fabrication drawing marking",
            "silkscreen location",
            "legend location",
        ],
    )
}

fn has_any(text: &str, needles: &[&str]) -> bool {
    needles.iter().any(|needle| text.contains(needle))
}
