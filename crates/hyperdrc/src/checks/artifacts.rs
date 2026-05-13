//! Pre-production sidecar artifact checks.
//!
//! These checks validate lightweight BOM and centroid structure before a package
//! reaches assembly quoting or programming/test fixture review. The parser is
//! intentionally conservative: it understands common CSV/TSV headers and emits
//! review warnings rather than trying to become a full spreadsheet engine.

use std::collections::{BTreeMap, BTreeSet};

use crate::report::{Severity, Violation};

#[derive(Clone, Debug)]
pub struct TextArtifact {
    pub path: String,
    pub text: String,
}

#[derive(Clone, Debug)]
pub struct FileArtifact {
    pub path: String,
    pub byte_len: u64,
}

pub fn production_artifact_readiness(
    bom_files: &[TextArtifact],
    centroid_files: &[TextArtifact],
    netlist_files: &[TextArtifact],
    readme_files: &[TextArtifact],
    fab_drawing_files: &[FileArtifact],
    assembly_drawing_files: &[FileArtifact],
    rout_drawing_files: &[FileArtifact],
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut bom_refs = BTreeSet::new();
    let mut bom_not_populated_refs = BTreeSet::new();
    let mut centroid_refs = BTreeSet::new();
    let mut netlist_refs = BTreeSet::new();

    for artifact in bom_files {
        violations.extend(analyze_text_artifact_path(artifact, TextArtifactKind::Bom));
        let report = analyze_bom(artifact);
        bom_refs.extend(report.refs);
        bom_not_populated_refs.extend(report.not_populated_refs);
        violations.extend(report.violations);
    }

    for artifact in centroid_files {
        violations.extend(analyze_text_artifact_path(
            artifact,
            TextArtifactKind::Centroid,
        ));
        let report = analyze_centroid(artifact);
        centroid_refs.extend(report.refs);
        violations.extend(report.violations);
    }

    for artifact in netlist_files {
        violations.extend(analyze_text_artifact_path(
            artifact,
            TextArtifactKind::Netlist,
        ));
        let report = analyze_netlist(artifact);
        netlist_refs.extend(report.refs);
        violations.extend(report.violations);
    }

    for artifact in readme_files {
        violations.extend(analyze_text_artifact_path(
            artifact,
            TextArtifactKind::Readme,
        ));
        violations.extend(analyze_readme(artifact));
    }
    violations.extend(analyze_file_artifacts(
        fab_drawing_files,
        DrawingKind::Fabrication,
    ));
    violations.extend(analyze_file_artifacts(
        assembly_drawing_files,
        DrawingKind::Assembly,
    ));
    violations.extend(analyze_file_artifacts(
        rout_drawing_files,
        DrawingKind::Rout,
    ));

    if !bom_refs.is_empty() && !centroid_refs.is_empty() {
        for reference in bom_refs.difference(&centroid_refs) {
            violations.push(artifact_violation(
                "bom-centroid-parity",
                Some(format!(
                    "BOM reference {reference} has no matching centroid placement"
                )),
            ));
        }
        let bom_known_refs = bom_refs
            .union(&bom_not_populated_refs)
            .cloned()
            .collect::<BTreeSet<_>>();
        for reference in centroid_refs.difference(&bom_known_refs) {
            violations.push(artifact_violation(
                "centroid-bom-parity",
                Some(format!(
                    "centroid reference {reference} has no matching BOM row"
                )),
            ));
        }
    }

    if !bom_not_populated_refs.is_empty() && !centroid_refs.is_empty() {
        for reference in bom_not_populated_refs.intersection(&centroid_refs) {
            violations.push(artifact_violation(
                "bom-centroid-population-parity",
                Some(format!(
                    "BOM marks reference {reference} as DNP/DNI, but the centroid file still places it"
                )),
            ));
        }
    }

    if !bom_refs.is_empty() && !netlist_refs.is_empty() {
        for reference in netlist_refs.difference(&bom_refs) {
            violations.push(artifact_violation(
                "netlist-bom-parity",
                Some(format!(
                    "netlist reference {reference} has no matching BOM row"
                )),
            ));
        }
        for reference in bom_refs.difference(&netlist_refs) {
            violations.push(artifact_violation(
                "bom-netlist-parity",
                Some(format!(
                    "BOM reference {reference} has no matching netlist pin record"
                )),
            ));
        }
    }

    if !centroid_refs.is_empty() && !netlist_refs.is_empty() {
        for reference in centroid_refs.difference(&netlist_refs) {
            violations.push(artifact_violation(
                "centroid-netlist-parity",
                Some(format!(
                    "centroid reference {reference} has no matching netlist pin record"
                )),
            ));
        }
    }

    violations
}

#[derive(Copy, Clone)]
enum DrawingKind {
    Fabrication,
    Assembly,
    Rout,
}

#[derive(Copy, Clone)]
enum TextArtifactKind {
    Bom,
    Centroid,
    Netlist,
    Readme,
}

#[derive(Default)]
struct ArtifactAnalysis {
    refs: BTreeSet<String>,
    not_populated_refs: BTreeSet<String>,
    violations: Vec<Violation>,
}

struct ReadmeRequirement {
    label: &'static str,
    needles: &'static [&'static str],
}

const README_ORDER_REQUIREMENTS: &[ReadmeRequirement] = &[
    ReadmeRequirement {
        label: "board thickness",
        needles: &[
            "board thickness",
            "thickness",
            "1.6mm",
            "1.6 mm",
            "0.8mm",
            "0.8 mm",
        ],
    },
    ReadmeRequirement {
        label: "copper weight",
        needles: &[
            "copper weight",
            "copper thickness",
            "1 oz",
            "1oz",
            "2 oz",
            "2oz",
        ],
    },
    ReadmeRequirement {
        label: "surface finish",
        needles: &["finish", "enig", "enepig", "hasl", "hard gold", "osp"],
    },
    ReadmeRequirement {
        label: "soldermask color/process",
        needles: &[
            "soldermask",
            "solder mask",
            "mask color",
            "green mask",
            "black mask",
        ],
    },
    ReadmeRequirement {
        label: "controlled impedance",
        needles: &["controlled impedance", "impedance", "no impedance"],
    },
    ReadmeRequirement {
        label: "panelization or depanelization",
        needles: &[
            "panel",
            "panelization",
            "depanel",
            "v-score",
            "vscore",
            "tab route",
        ],
    },
    ReadmeRequirement {
        label: "via treatment",
        needles: &[
            "via tent",
            "tented via",
            "via fill",
            "filled via",
            "plugged via",
            "capped via",
        ],
    },
    ReadmeRequirement {
        label: "edge plating/castellation",
        needles: &[
            "edge plating",
            "edge plated",
            "no edge plating",
            "castellation",
            "castellated",
            "no castellation",
        ],
    },
];

const README_PREFLIGHT_REQUIREMENTS: &[ReadmeRequirement] = &[
    ReadmeRequirement {
        label: "EDA DRC/ERC",
        needles: &["drc", "erc", "design rule check", "electrical rule check"],
    },
    ReadmeRequirement {
        label: "zone refill",
        needles: &[
            "zone refill",
            "zones refilled",
            "refilled zones",
            "pour refill",
        ],
    },
    ReadmeRequirement {
        label: "fresh output generation",
        needles: &["generated", "plotted", "exported", "fabrication output"],
    },
    ReadmeRequirement {
        label: "independent viewer review",
        needles: &[
            "viewer",
            "gerber viewer",
            "independent review",
            "reloaded outputs",
        ],
    },
    ReadmeRequirement {
        label: "HyperDRC review",
        needles: &["hyperdrc"],
    },
    ReadmeRequirement {
        label: "waiver review",
        needles: &["waiver", "waivers", "no waivers"],
    },
    ReadmeRequirement {
        label: "submitted package archive",
        needles: &[
            "archive",
            "archived",
            "release package",
            "submitted package",
        ],
    },
];

const README_ASSEMBLY_REQUIREMENTS: &[ReadmeRequirement] = &[
    ReadmeRequirement {
        label: "pin-1 or polarity review",
        needles: &["pin 1", "pin-1", "polarity", "polarized"],
    },
    ReadmeRequirement {
        label: "test or programming handoff",
        needles: &["test", "programming", "fixture", "ict", "fct"],
    },
];

fn analyze_bom(artifact: &TextArtifact) -> ArtifactAnalysis {
    let mut analysis = ArtifactAnalysis::default();
    let Some(table) = parse_table(&artifact.text) else {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM file has no parseable rows".to_string()),
        ));
        return analysis;
    };

    let ref_col = find_column(&table.headers, &["ref", "reference", "designator"]);
    let part_col = find_column(
        &table.headers,
        &["mpn", "manufacturer", "part", "partnumber", "value"],
    );
    let manufacturer_col = find_column(&table.headers, &["manufacturer", "mfr", "maker"]);
    let supplier_col = find_column(
        &table.headers,
        &[
            "supplier",
            "vendor",
            "distributor",
            "sku",
            "ordercode",
            "orderable",
        ],
    );
    let value_col = find_column(&table.headers, &["value", "description"]);
    let package_col = find_column(&table.headers, &["package", "footprint", "case"]);
    let qty_col = find_column(&table.headers, &["qty", "quantity"]);

    if ref_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no reference/designator column".to_string()),
        ));
    }
    if part_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no part number, manufacturer part, or value column".to_string()),
        ));
    }
    if qty_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no quantity column".to_string()),
        ));
    }
    if manufacturer_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no manufacturer column".to_string()),
        ));
    }
    if supplier_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some(
                "BOM header has no supplier, distributor, SKU, or orderable part column"
                    .to_string(),
            ),
        ));
    }
    if value_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no value or description column".to_string()),
        ));
    }
    if package_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no footprint, package, or case column".to_string()),
        ));
    }

    let mut occurrences = BTreeMap::<String, usize>::new();
    let mut mpn_values = BTreeMap::<String, BTreeSet<String>>::new();
    let mut mpn_packages = BTreeMap::<String, BTreeSet<String>>::new();
    for (row_index, row) in table.rows.iter().enumerate() {
        let not_populated = is_not_populated_row(row);
        let references = ref_col
            .map(|column| split_references(cell(row, column)))
            .unwrap_or_default();
        if ref_col.is_some() {
            for reference in &references {
                *occurrences.entry(reference.clone()).or_default() += 1;
                if !not_populated {
                    analysis.refs.insert(reference.clone());
                } else {
                    analysis.not_populated_refs.insert(reference.clone());
                }
                if !is_reference_designator(reference) {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} has unusual reference designator {:?}",
                            row_index + 2,
                            reference
                        )),
                    ));
                }
            }
        }
        if let Some(column) = part_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no populated part identifier",
                    row_index + 2
                )),
            ));
        }
        if let Some(column) = manufacturer_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no populated manufacturer",
                    row_index + 2
                )),
            ));
        }
        if let Some(column) = supplier_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no populated supplier/distributor/SKU",
                    row_index + 2
                )),
            ));
        }
        if let Some(column) = value_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no populated value/description",
                    row_index + 2
                )),
            ));
        }
        if let Some(column) = package_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no populated footprint/package",
                    row_index + 2
                )),
            ));
        }
        if !not_populated
            && let (Some(part_column), Some(value_column), Some(package_column)) =
                (part_col, value_col, package_col)
        {
            let part = normalize_bom_key(cell(row, part_column));
            let value = normalize_bom_key(cell(row, value_column));
            let package = normalize_bom_key(cell(row, package_column));
            if !part.is_empty() && !value.is_empty() {
                mpn_values.entry(part.clone()).or_default().insert(value);
            }
            if !part.is_empty() && !package.is_empty() {
                mpn_packages.entry(part).or_default().insert(package);
            }
        }
        if let Some(column) = qty_col {
            let quantity_text = cell(row, column).trim();
            let quantity = parse_quantity(quantity_text);
            if quantity.is_none() && !quantity_text.is_empty() {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} has invalid quantity {:?}",
                        row_index + 2,
                        quantity_text
                    )),
                ));
            }
            if let Some(quantity) = quantity
                && quantity != references.len()
                && !not_populated
            {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} quantity {} does not match {} reference designator(s)",
                        row_index + 2,
                        quantity,
                        references.len()
                    )),
                ));
            }
            if let Some(quantity) = quantity
                && quantity > 0
                && not_populated
            {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} is marked DNP/DNI but has nonzero quantity {}",
                        row_index + 2,
                        quantity
                    )),
                ));
            }
        }
    }

    for (reference, count) in occurrences {
        if count > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("BOM reference {reference} appears in {count} rows")),
            ));
        }
    }
    for (part, values) in mpn_values {
        if values.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM part {part} is used with multiple values/descriptions: {}",
                    values.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }
    for (part, packages) in mpn_packages {
        if packages.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM part {part} is used with multiple footprints/packages: {}",
                    packages.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }

    analysis
}

fn analyze_centroid(artifact: &TextArtifact) -> ArtifactAnalysis {
    let mut analysis = ArtifactAnalysis::default();
    let Some(table) = parse_table(&artifact.text) else {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("centroid file has no parseable rows".to_string()),
        ));
        return analysis;
    };

    let ref_col = find_column(&table.headers, &["ref", "reference", "designator"]);
    let x_col = find_column(&table.headers, &["x", "posx", "mid x", "centerx"]);
    let y_col = find_column(&table.headers, &["y", "posy", "mid y", "centery"]);
    let rotation_col = find_column(&table.headers, &["rotation", "rot", "angle"]);
    let side_col = find_column(&table.headers, &["side", "layer", "mountside"]);

    for (name, column) in [
        ("reference/designator", ref_col),
        ("x coordinate", x_col),
        ("y coordinate", y_col),
        ("rotation", rotation_col),
        ("side/layer", side_col),
    ] {
        if column.is_none() {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("centroid header has no {name} column")),
            ));
        }
    }

    let mut occurrences = BTreeMap::<String, usize>::new();
    let mut placements = BTreeMap::<(String, String, String), Vec<String>>::new();
    for (row_index, row) in table.rows.iter().enumerate() {
        let mut row_reference = None;
        if let Some(column) = ref_col {
            let reference = cell(row, column).trim();
            if reference.is_empty() {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!("centroid row {} has no reference", row_index + 2)),
                ));
            } else {
                let reference = normalize_reference(reference);
                *occurrences.entry(reference.clone()).or_default() += 1;
                analysis.refs.insert(reference.clone());
                row_reference = Some(reference.clone());
                if !is_reference_designator(&reference) {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "centroid row {} has unusual reference designator {:?}",
                            row_index + 2,
                            reference
                        )),
                    ));
                }
            }
        }

        let mut row_x = None;
        let mut row_y = None;
        for (label, column) in [("x", x_col), ("y", y_col), ("rotation", rotation_col)] {
            if let Some(column) = column {
                let value = normalize_numeric_cell(cell(row, column));
                let Ok(numeric) = value.parse::<f64>() else {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "centroid row {} has invalid {label} value {:?}",
                            row_index + 2,
                            cell(row, column)
                        )),
                    ));
                    continue;
                };
                if label == "rotation" && !(-360.0..=360.0).contains(&numeric) {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "centroid row {} rotation {:?} is outside the expected -360..360 degree range",
                            row_index + 2,
                            cell(row, column)
                        )),
                    ));
                }
                if label == "x" {
                    row_x = Some(format!("{numeric:.4}"));
                } else if label == "y" {
                    row_y = Some(format!("{numeric:.4}"));
                }
            }
        }

        let mut row_side = None;
        if let Some(column) = side_col {
            let side = cell(row, column).trim().to_ascii_lowercase();
            if !matches!(
                side.as_str(),
                "top" | "bottom" | "front" | "back" | "f" | "b" | "f.cu" | "b.cu"
            ) {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "centroid row {} has invalid side value {:?}",
                        row_index + 2,
                        cell(row, column)
                    )),
                ));
            } else {
                row_side = Some(normalize_side(&side).to_string());
            }
        }

        if let (Some(reference), Some(x), Some(y), Some(side)) =
            (row_reference, row_x, row_y, row_side)
        {
            placements.entry((x, y, side)).or_default().push(reference);
        }
    }

    for (reference, count) in occurrences {
        if count > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "centroid reference {reference} appears in {count} rows"
                )),
            ));
        }
    }

    for ((x, y, side), references) in placements {
        if references.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "centroid places multiple references at {x},{y} on {side}: {}",
                    references.join(", ")
                )),
            ));
        }
    }

    analysis
}

fn analyze_netlist(artifact: &TextArtifact) -> ArtifactAnalysis {
    let mut analysis = ArtifactAnalysis::default();
    let Some(table) = parse_table(&artifact.text) else {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("netlist file has no parseable rows".to_string()),
        ));
        return analysis;
    };

    let net_col = find_column(&table.headers, &["net", "netname", "signal"]);
    let ref_col = find_column(
        &table.headers,
        &["ref", "reference", "designator", "component"],
    );
    let pin_col = find_column(&table.headers, &["pin", "pad", "terminal"]);

    for (name, column) in [
        ("net", net_col),
        ("reference/designator", ref_col),
        ("pin/pad", pin_col),
    ] {
        if column.is_none() {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist header has no {name} column")),
            ));
        }
    }

    let mut pin_nets = BTreeMap::<(String, String), BTreeSet<String>>::new();
    let mut pin_rows = BTreeMap::<(String, String, String), usize>::new();
    let mut net_pins = BTreeMap::<String, BTreeSet<(String, String)>>::new();
    for (row_index, row) in table.rows.iter().enumerate() {
        let net = net_col.map(|column| cell(row, column).trim()).unwrap_or("");
        let reference = ref_col.map(|column| cell(row, column).trim()).unwrap_or("");
        let pin = pin_col.map(|column| cell(row, column).trim()).unwrap_or("");
        let normalized_reference = (!reference.is_empty()).then(|| normalize_reference(reference));

        if net.is_empty() {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist row {} has no net name", row_index + 2)),
            ));
        }
        if reference.is_empty() {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist row {} has no reference", row_index + 2)),
            ));
        } else if let Some(reference) = &normalized_reference {
            analysis.refs.insert(reference.clone());
            if !is_reference_designator(reference) {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "netlist row {} has unusual reference designator {:?}",
                        row_index + 2,
                        reference
                    )),
                ));
            }
        }
        if pin.is_empty() {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist row {} has no pin/pad", row_index + 2)),
            ));
        }

        if let Some(reference) = normalized_reference
            && !pin.is_empty()
            && !net.is_empty()
        {
            let net = net.to_string();
            let pin = pin.to_string();
            pin_nets
                .entry((reference.clone(), pin.clone()))
                .or_default()
                .insert(net.clone());
            *pin_rows
                .entry((reference.clone(), pin.clone(), net.clone()))
                .or_default() += 1;
            net_pins.entry(net).or_default().insert((reference, pin));
        }
    }

    for ((reference, pin), nets) in pin_nets {
        if nets.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "netlist assigns {reference}.{pin} to multiple nets: {}",
                    nets.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }

    for ((reference, pin, net), count) in pin_rows {
        if count > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "netlist repeats {reference}.{pin} on net {net} in {count} rows"
                )),
            ));
        }
    }

    for (net, pins) in net_pins {
        if pins.len() < 2 && !is_intentional_single_pin_net(&net) {
            let pin = pins
                .into_iter()
                .next()
                .map(|(reference, pin)| format!("{reference}.{pin}"))
                .unwrap_or_else(|| "unknown pin".to_string());
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist net {net} has only one pin record ({pin})")),
            ));
        }
    }

    analysis
}

fn analyze_readme(artifact: &TextArtifact) -> Vec<Violation> {
    let mut violations = Vec::new();
    let normalized = artifact.text.to_ascii_lowercase();

    if artifact.text.trim().is_empty() {
        violations.push(artifact_violation(
            &artifact.path,
            Some("README artifact is empty".to_string()),
        ));
        return violations;
    }

    if !has_revision_marker(&normalized) {
        violations.push(artifact_violation(
            &artifact.path,
            Some("README artifact does not mention a revision or version".to_string()),
        ));
    }

    if !has_any(
        &normalized,
        &[
            "stackup",
            "layer",
            "thickness",
            "finish",
            "assembly",
            "impedance",
            "fabrication",
            "fab",
        ],
    ) {
        violations.push(artifact_violation(
            &artifact.path,
            Some(
                "README artifact does not mention stackup, fabrication, finish, assembly, or impedance notes"
                    .to_string(),
            ),
        ));
    }

    for requirement in README_ORDER_REQUIREMENTS {
        if !has_any(&normalized, requirement.needles) {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "README artifact does not mention {} order intent",
                    requirement.label
                )),
            ));
        }
    }

    for requirement in README_PREFLIGHT_REQUIREMENTS {
        if !has_any(&normalized, requirement.needles) {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "README artifact does not mention {} preflight evidence",
                    requirement.label
                )),
            ));
        }
    }

    for requirement in README_ASSEMBLY_REQUIREMENTS {
        if !has_any(&normalized, requirement.needles) {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "README artifact does not mention {} assembly evidence",
                    requirement.label
                )),
            ));
        }
    }

    if has_any(&normalized, &["impedance", "controlled impedance"])
        && !has_any(
            &normalized,
            &["coupon", "stackup", "trace width", "trace/space"],
        )
    {
        violations.push(artifact_violation(
            &artifact.path,
            Some(
                "README artifact mentions impedance but not coupon, stackup, or trace geometry intent"
                    .to_string(),
            ),
        ));
    }

    violations.extend(readme_contradictions(&artifact.path, &normalized));

    violations
}

fn analyze_file_artifacts(artifacts: &[FileArtifact], kind: DrawingKind) -> Vec<Violation> {
    let mut violations = Vec::new();
    for artifact in artifacts {
        if artifact.byte_len == 0 {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!("{} artifact is empty", kind.name())),
            ));
        } else if artifact.byte_len < 64 {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "{} artifact is only {} bytes; this looks like a placeholder rather than a usable drawing",
                    kind.name(),
                    artifact.byte_len
                )),
            ));
        }

        let extension = path_extension(&artifact.path);
        if !kind.allowed_extensions().contains(&extension.as_str()) {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "{} artifact extension {:?} is not a common supported drawing/package format",
                    kind.name(),
                    extension
                )),
            ));
        }

        if !filename_has_any(&artifact.path, kind.filename_tokens()) {
            violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "{} artifact filename does not include an expected role token such as {}",
                    kind.name(),
                    kind.filename_tokens().join(", ")
                )),
            ));
        }
    }

    violations
}

fn readme_contradictions(path: &str, normalized: &str) -> Vec<Violation> {
    let mut violations = Vec::new();
    for (negative, positive, label) in [
        (
            &["no impedance", "impedance not required"][..],
            &[
                "controlled impedance required",
                "impedance required",
                "impedance coupon required",
            ][..],
            "controlled impedance",
        ),
        (
            &["no panel", "single board only", "no panelization"][..],
            &["panelized", "tab route", "v-score", "vscore"][..],
            "panelization",
        ),
        (
            &["no edge plating", "edge plating not required"][..],
            &["edge plating required", "edge plated"][..],
            "edge plating",
        ),
        (
            &["no castellation", "no castellations"][..],
            &["castellation required", "castellated"][..],
            "castellation",
        ),
    ] {
        if has_any(normalized, negative) && has_any(normalized, positive) {
            violations.push(artifact_violation(
                path,
                Some(format!(
                    "README artifact contains contradictory {label} order intent"
                )),
            ));
        }
    }
    violations
}

fn analyze_text_artifact_path(artifact: &TextArtifact, kind: TextArtifactKind) -> Vec<Violation> {
    let mut violations = Vec::new();
    let extension = path_extension(&artifact.path);
    if !kind.allowed_extensions().contains(&extension.as_str()) {
        violations.push(artifact_violation(
            &artifact.path,
            Some(format!(
                "{} artifact extension {:?} is not a common supported text/package format",
                kind.name(),
                extension
            )),
        ));
    }

    if !filename_has_any(&artifact.path, kind.filename_tokens()) {
        violations.push(artifact_violation(
            &artifact.path,
            Some(format!(
                "{} artifact filename does not include an expected role token such as {}",
                kind.name(),
                kind.filename_tokens().join(", ")
            )),
        ));
    }

    violations
}

struct Table {
    headers: Vec<String>,
    rows: Vec<Vec<String>>,
}

fn parse_table(text: &str) -> Option<Table> {
    let lines = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();
    let header = lines.first()?;
    let delimiter = if header.contains('\t') { '\t' } else { ',' };
    let headers = split_row(header, delimiter)
        .into_iter()
        .map(|header| normalize_header(&header))
        .collect::<Vec<_>>();
    let rows = lines
        .iter()
        .skip(1)
        .map(|line| split_row(line, delimiter))
        .filter(|row| row.iter().any(|cell| !cell.trim().is_empty()))
        .collect::<Vec<_>>();

    Some(Table { headers, rows }).filter(|table| !table.headers.is_empty())
}

fn split_row(line: &str, delimiter: char) -> Vec<String> {
    let mut cells = Vec::new();
    let mut current = String::new();
    let mut in_quotes = false;
    let mut chars = line.chars().peekable();

    while let Some(ch) = chars.next() {
        if ch == '"' {
            if in_quotes && chars.peek() == Some(&'"') {
                current.push('"');
                chars.next();
            } else {
                in_quotes = !in_quotes;
            }
        } else if ch == delimiter && !in_quotes {
            cells.push(current.trim().to_string());
            current.clear();
        } else {
            current.push(ch);
        }
    }
    cells.push(current.trim().to_string());

    cells
}

fn normalize_header(header: &str) -> String {
    header
        .trim_matches('"')
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}

fn find_column(headers: &[String], candidates: &[&str]) -> Option<usize> {
    let candidates = candidates
        .iter()
        .map(|candidate| normalize_header(candidate))
        .collect::<Vec<_>>();
    headers.iter().position(|header| {
        candidates
            .iter()
            .any(|candidate| header == candidate || header.contains(candidate))
    })
}

fn cell(row: &[String], column: usize) -> &str {
    row.get(column).map(String::as_str).unwrap_or("")
}

fn split_references(value: &str) -> Vec<String> {
    value
        .split(|ch: char| ch.is_whitespace() || matches!(ch, ',' | ';' | '|'))
        .map(str::trim)
        .filter(|reference| !reference.is_empty())
        .flat_map(expand_reference_token)
        .collect()
}

fn expand_reference_token(token: &str) -> Vec<String> {
    let normalized = normalize_reference(token);
    let Some((start, end)) = normalized.split_once('-') else {
        return vec![normalized];
    };
    let Some((start_prefix, start_number)) = split_reference_designator(start) else {
        return vec![normalized];
    };
    let Some((end_prefix, end_number)) = split_reference_designator(end) else {
        return vec![normalized];
    };
    if start_prefix != end_prefix || start_number > end_number || end_number - start_number > 500 {
        return vec![normalized];
    }

    (start_number..=end_number)
        .map(|number| format!("{start_prefix}{number}"))
        .collect()
}

fn normalize_reference(reference: &str) -> String {
    reference
        .trim()
        .trim_matches('"')
        .trim_matches('\'')
        .to_ascii_uppercase()
}

fn normalize_bom_key(value: &str) -> String {
    value
        .trim()
        .trim_matches('"')
        .trim_matches('\'')
        .to_ascii_uppercase()
}

fn is_reference_designator(reference: &str) -> bool {
    split_reference_designator(reference).is_some()
}

fn split_reference_designator(reference: &str) -> Option<(&str, usize)> {
    let split_at = reference
        .char_indices()
        .find_map(|(index, ch)| ch.is_ascii_digit().then_some(index))?;
    let (prefix, suffix) = reference.split_at(split_at);
    if prefix.is_empty()
        || !prefix.chars().all(|ch| ch.is_ascii_alphabetic())
        || suffix.is_empty()
        || !suffix.chars().all(|ch| ch.is_ascii_digit())
    {
        return None;
    }
    Some((prefix, suffix.parse().ok()?))
}

fn normalize_numeric_cell(value: &str) -> String {
    value
        .trim()
        .trim_end_matches("mm")
        .trim_end_matches("MM")
        .trim_end_matches("deg")
        .trim_end_matches("DEG")
        .to_string()
}

fn normalize_side(side: &str) -> &'static str {
    match side {
        "top" | "front" | "f" | "f.cu" => "top",
        "bottom" | "back" | "b" | "b.cu" => "bottom",
        _ => "unknown",
    }
}

fn parse_quantity(value: &str) -> Option<usize> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return None;
    }
    if let Ok(integer) = trimmed.parse::<usize>() {
        return Some(integer);
    }
    let numeric = trimmed.parse::<f64>().ok()?;
    if numeric.fract().abs() <= f64::EPSILON && numeric >= 0.0 {
        Some(numeric as usize)
    } else {
        None
    }
}

fn is_not_populated_row(row: &[String]) -> bool {
    row.iter().any(|cell| {
        let normalized = cell.to_ascii_lowercase();
        has_any(
            &normalized,
            &["dnp", "dni", "dnf", "do not populate", "not fitted"],
        )
    })
}

fn is_intentional_single_pin_net(net: &str) -> bool {
    let normalized = net.to_ascii_lowercase();
    let compact = normalized
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>();
    matches!(compact.as_str(), "nc" | "noconnect" | "dnp" | "dni")
        || compact.starts_with("nc")
        || compact.starts_with("noconnect")
        || normalized.contains("no connect")
        || normalized.contains("not connected")
}

fn has_revision_marker(text: &str) -> bool {
    text.split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .any(|token| {
            token == "rev"
                || token == "revision"
                || token == "version"
                || token
                    .strip_prefix("rev")
                    .is_some_and(|suffix| !suffix.is_empty())
                || token.strip_prefix('v').is_some_and(|suffix| {
                    suffix.chars().next().is_some_and(|ch| ch.is_ascii_digit())
                })
        })
}

fn has_any(haystack: &str, needles: &[&str]) -> bool {
    needles.iter().any(|needle| haystack.contains(needle))
}

fn path_extension(path: &str) -> String {
    std::path::Path::new(path)
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or("")
        .to_ascii_lowercase()
}

fn filename_has_any(path: &str, tokens: &[&str]) -> bool {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path)
        .to_ascii_lowercase();
    tokens.iter().any(|token| stem.contains(token))
}

impl DrawingKind {
    fn name(self) -> &'static str {
        match self {
            DrawingKind::Fabrication => "fabrication drawing",
            DrawingKind::Assembly => "assembly drawing",
            DrawingKind::Rout => "rout drawing",
        }
    }

    fn allowed_extensions(self) -> &'static [&'static str] {
        match self {
            DrawingKind::Fabrication => &["pdf", "dxf", "svg", "dwg"],
            DrawingKind::Assembly => &["pdf", "dxf", "svg", "png", "jpg", "jpeg"],
            DrawingKind::Rout => &["pdf", "dxf", "svg", "dwg", "gko", "gm1", "gm2", "gml"],
        }
    }

    fn filename_tokens(self) -> &'static [&'static str] {
        match self {
            DrawingKind::Fabrication => &["fab", "fabrication", "fabricator", "drawing"],
            DrawingKind::Assembly => &["assy", "assembly", "placement", "drawing"],
            DrawingKind::Rout => &[
                "rout", "route", "routing", "vscore", "v-score", "panel", "tool",
            ],
        }
    }
}

impl TextArtifactKind {
    fn name(self) -> &'static str {
        match self {
            TextArtifactKind::Bom => "BOM",
            TextArtifactKind::Centroid => "centroid",
            TextArtifactKind::Netlist => "netlist",
            TextArtifactKind::Readme => "README",
        }
    }

    fn allowed_extensions(self) -> &'static [&'static str] {
        match self {
            TextArtifactKind::Bom => &["csv", "tsv", "txt", "xlsx", "xls"],
            TextArtifactKind::Centroid => &["csv", "tsv", "txt", "pos"],
            TextArtifactKind::Netlist => &["csv", "tsv", "txt", "net", "ipc", "356"],
            TextArtifactKind::Readme => &["md", "markdown", "txt"],
        }
    }

    fn filename_tokens(self) -> &'static [&'static str] {
        match self {
            TextArtifactKind::Bom => &["bom", "bill-of-materials", "bill_of_materials"],
            TextArtifactKind::Centroid => &[
                "centroid",
                "placement",
                "positions",
                "pick-place",
                "pick_place",
                "pnp",
            ],
            TextArtifactKind::Netlist => &["netlist", "ipc356", "ipc-356", "net"],
            TextArtifactKind::Readme => &["readme", "release", "notes", "fabrication"],
        }
    }
}

fn artifact_violation(layer: &str, message: Option<String>) -> Violation {
    Violation::new(
        "production-artifact-readiness",
        Severity::Warning,
        vec![layer.to_string()],
        None,
        Vec::new(),
        Vec::new(),
        message,
    )
}

#[cfg(test)]
mod tests {
    use super::{FileArtifact, TextArtifact, production_artifact_readiness};

    #[test]
    fn complete_bom_and_centroid_with_matching_refs_is_clean() {
        let bom = artifact(
            "bom.csv",
            "Ref,Qty,MPN,Value,Footprint,Manufacturer,Supplier SKU\nR1,1,RC0603,10k,0603,Yageo,SKU-R\nC1,1,CC0603,100nF,0603,Murata,SKU-C\n",
        );
        let centroid = artifact(
            "centroid.csv",
            "Designator,X,Y,Rotation,Side\nR1,1.0,2.0,90,Top\nC1,3.0,4.0,0,Bottom\n",
        );

        assert!(
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]).is_empty()
        );
    }

    #[test]
    fn bom_readiness_reports_missing_columns_duplicate_refs_and_blank_parts() {
        let bom = artifact("bom.csv", "Reference,Quantity\nR1,1\nR1,2\nC1,1\n");

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| message.contains("no part")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("value or description"))
        );
        assert!(messages.iter().any(|message| message.contains("footprint")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("manufacturer column"))
        );
        assert!(messages.iter().any(|message| message.contains("supplier")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("R1 appears"))
        );
    }

    #[test]
    fn bom_readiness_reports_empty_value_and_package_cells() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier\nR1,1,RC0603,,0603,Yageo,DistA\nC1,1,CC0603,100nF,,Murata,DistB\nD1,0,DNP,,,,\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("no populated value"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no populated footprint"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("BOM row 4 has no populated"))
        );
    }

    #[test]
    fn bom_readiness_reports_missing_procurement_cells_and_conflicting_part_metadata() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier SKU\nR1,1,RC0603,10k,0603,Yageo,SKU1\nR2,1,RC0603,1k,0402,,\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("no populated manufacturer"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no populated supplier"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple values") && message.contains("RC0603"))
        );
        assert!(
            messages.iter().any(
                |message| message.contains("multiple footprints") && message.contains("RC0603")
            )
        );
    }

    #[test]
    fn bom_readiness_checks_quantity_against_grouped_references() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN\n\"R1 R2\",1,RC0603\nC1,bad,CC0603\nD1,0,DNP\nD2,1,DNI\n\"U1-U3\",3,MCU\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("quantity 1 does not match 2"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("invalid quantity"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("quantity 0"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("nonzero quantity 1"))
        );
        assert!(!messages.iter().any(|message| message.contains("U1-U3")));
    }

    #[test]
    fn bom_readiness_expands_common_reference_groups_and_skips_dnp_parity() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier\n\"r1,r2 r3;R4|R5\",5,RC0603,10k,0603,Yageo,DistA\nD1,0,DNP,,,,\n",
        );
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nR1,0,0,0,Top\nR2,1,0,0,Top\nR3,2,0,0,Top\nR4,3,0,0,Top\nR5,4,0,0,Top\n",
        );

        assert!(
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]).is_empty()
        );
    }

    #[test]
    fn bom_dnp_references_are_reported_when_still_placed() {
        let bom = artifact("bom.csv", "Reference,Quantity,MPN\nR1,1,RC0603\nD1,0,DNP\n");
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nR1,0,0,0,Top\nD1,2,0,0,Top\n",
        );

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("DNP/DNI") && message.contains("D1"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("D1 has no matching BOM"))
        );
    }

    #[test]
    fn artifact_reference_designators_are_validated() {
        let bom = artifact("bom.csv", "Reference,Quantity,MPN\nMOUNT,1,hardware\n");
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\nBAD,0,0,0,Top\n");
        let netlist = artifact("netlist.csv", "Net,Ref,Pin\nGND,fixture,1\n");

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[netlist], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("unusual reference designator"))
                .count(),
            3
        );
    }

    #[test]
    fn centroid_readiness_reports_bad_columns_values_sides_and_duplicates() {
        let centroid = artifact(
            "centroid.csv",
            "Ref,Mid X,Mid Y,Rot,Layer\nU1,not-a-number,2.0,90,Top\nU1,1.0,2.0,bad,Inner\nU2,1.0,2.0,720,Top\nR1,5.0,6.0,0,Top\nR2,5.00001,6.00001,90,F.Cu\n",
        );

        let violations = production_artifact_readiness(&[], &[centroid], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| message.contains("invalid x")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("invalid rotation"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("invalid side"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("U1 appears"))
        );
        assert!(messages.iter().any(|message| message.contains("-360..360")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple references") && message.contains("R1"))
        );
    }

    #[test]
    fn bom_centroid_reference_parity_is_checked_bidirectionally() {
        let bom = artifact("bom.csv", "Ref,Qty,MPN\nR1,1,RC0603\nC1,1,CC0603\n");
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nR1,1.0,2.0,90,Top\nU1,3.0,4.0,0,Bottom\n",
        );

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| message.contains("C1")));
        assert!(messages.iter().any(|message| message.contains("U1")));
    }

    #[test]
    fn netlist_readiness_reports_missing_columns_empty_rows_and_pin_conflicts() {
        let netlist = artifact(
            "netlist.csv",
            "Net,Reference,Pin\nGND,U1,1\n3V3,U1,1\nGND,U4,1\nGND,U4,1\n,U2,2\nSIG,,3\nGPIO,U3,\nONEPIN,U5,1\nNC,U6,1\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[netlist], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple nets"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("repeats U4.1"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("ONEPIN") && message.contains("only one pin"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("NC") && message.contains("only one pin"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no net name"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no reference"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no pin/pad"))
        );
    }

    #[test]
    fn netlist_bom_and_centroid_parity_are_checked() {
        let bom = artifact("bom.csv", "Ref,Qty,MPN\nU1,1,MCU\nR1,1,RES\n");
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nU1,1.0,2.0,0,Top\nJ1,3.0,4.0,0,Top\n",
        );
        let netlist = artifact("netlist.csv", "Net,Ref,Pin\nGND,U1,1\nVBUS,J1,1\n");

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[netlist], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("J1 has no matching BOM"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("U1 has no matching netlist"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("R1 has no matching netlist"))
        );
    }

    #[test]
    fn text_artifact_paths_report_unknown_extensions_and_role_ambiguous_names() {
        let bom = artifact("parts.data", "Ref,Qty,MPN\nR1,1,RC0603\n");
        let centroid = artifact("widget.csv", "Ref,X,Y,Rotation,Side\nR1,0,0,0,Top\n");
        let netlist = artifact("connectivity.bin", "Net,Ref,Pin\nGND,R1,1\n");
        let readme = artifact(
            "handoff.json",
            "Revision A. Fabrication stackup. Thickness 1.6mm, copper weight 1 oz, ENIG finish, \
             soldermask green, no impedance, panelization none, tented vias, no edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer \
             checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. \
             Test fixture/programming handoff complete.\n",
        );

        let violations = production_artifact_readiness(
            &[bom],
            &[centroid],
            &[netlist],
            &[readme],
            &[],
            &[],
            &[],
        );
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("BOM artifact extension"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("BOM artifact filename"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("centroid artifact filename"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("netlist artifact extension"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("README artifact extension"))
        );
    }

    #[test]
    fn text_artifact_paths_allow_common_role_names_and_extensions() {
        let bom = artifact(
            "widget_bom.tsv",
            "Ref\tQty\tMPN\tValue\tFootprint\tManufacturer\tSupplier SKU\nR1\t1\tRC0603\t10k\t0603\tYageo\tSKU-R\n",
        );
        let centroid = artifact(
            "widget_pick_place.pos",
            "Ref X Y Rotation Side\nR1 0 0 0 Top\n",
        );
        let netlist = artifact("widget_netlist.net", "Net,Ref,Pin\nGND,R1,1\n");
        let readme = artifact(
            "release_notes.md",
            "Revision A. Fabrication stackup. Thickness 1.6mm, copper weight 1 oz, ENIG finish, \
             soldermask green, no impedance, panelization none, tented vias, no edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer \
             checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. \
             Test fixture/programming handoff complete.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[centroid],
            &[netlist],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("artifact extension"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("artifact filename"))
        );
    }

    #[test]
    fn readme_readiness_requires_revision_and_manufacturing_notes() {
        let readme = artifact("README.md", "Release package for board.\n");

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| message.contains("revision")));
        assert!(messages.iter().any(|message| message.contains("stackup")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("board thickness order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("copper weight order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("via treatment order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("EDA DRC/ERC preflight evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("HyperDRC review preflight evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("pin-1 or polarity review assembly evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("test or programming handoff assembly evidence"))
        );
    }

    #[test]
    fn readme_readiness_allows_revision_and_fabrication_notes() {
        let readme = artifact(
            "README.md",
            "Revision B production package.\n\
             Stackup: 4 layer, 1.6mm board thickness, 1 oz copper weight.\n\
             Finish: ENIG. Soldermask: green.\n\
             Controlled impedance: no controlled-impedance coupon.\n\
             Panelization: no panel, single board only.\n\
             Via treatment: tented vias. Edge plating: no edge plating. Castellations: no castellation.\n\
             Preflight: DRC/ERC passed, zones refilled, outputs generated and reloaded in Gerber viewer.\n\
             HyperDRC reviewed with no waivers. Submitted package archived.\n\
             Assembly: pin-1 and polarity reviewed. Test fixture and programming handoff complete.\n",
        );

        assert!(production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]).is_empty());
    }

    #[test]
    fn readme_readiness_requires_order_parameter_intent() {
        let readme = artifact(
            "release_notes.md",
            "Revision C. Fabrication notes: 4 layer stackup, 1.2mm thickness, ENIG finish.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("copper weight order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("soldermask color/process order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("controlled impedance order intent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("edge plating/castellation order intent"))
        );
    }

    #[test]
    fn readme_readiness_expands_impedance_context_warning() {
        let readme = artifact(
            "README.md",
            "Revision C. Fabrication package. Thickness 1.6mm, copper weight 1 oz, ENIG finish, \
             soldermask green, controlled impedance required, panelized tab route, tented vias, \
             no edge plating, no castellation. DRC/ERC passed, zones refilled, outputs generated, \
             Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity \
             reviewed. Test fixture/programming handoff complete.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("mentions impedance"))
        );
    }

    #[test]
    fn readme_readiness_reports_contradictory_order_intent() {
        let readme = artifact(
            "README.md",
            "Revision E. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green. No impedance, but controlled impedance required. \
             No panel, but panelized tab route requested. No edge plating, but edge plating required. \
             No castellation, but castellated edge required. Tented vias. DRC/ERC passed, zones \
             refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, \
             archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        for label in [
            "controlled impedance",
            "panelization",
            "edge plating",
            "castellation",
        ] {
            assert!(
                messages
                    .iter()
                    .any(|message| message.contains("contradictory") && message.contains(label)),
                "missing contradiction for {label}"
            );
        }
    }

    #[test]
    fn readme_readiness_requires_preflight_evidence() {
        let readme = artifact(
            "README.md",
            "Revision D. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, panelization none, tented vias, \
             no edge plating, no castellation.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("zone refill preflight evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("independent viewer review preflight evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("waiver review preflight evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("submitted package archive preflight evidence"))
        );
    }

    #[test]
    fn drawing_artifact_readiness_reports_empty_unknown_or_role_ambiguous_files() {
        let fab = file("board_notes.txt", 0);
        let assembly = file("generic.pdf", 128);
        let rout = file("panel_route.dxf", 12);

        let violations =
            production_artifact_readiness(&[], &[], &[], &[], &[fab], &[assembly], &[rout]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| message.contains("empty")));
        assert!(messages.iter().any(|message| message.contains("extension")));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("assembly drawing artifact filename"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("rout drawing artifact filename"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("only 12 bytes"))
        );
    }

    #[test]
    fn drawing_artifact_readiness_allows_common_named_formats() {
        assert!(
            production_artifact_readiness(
                &[],
                &[],
                &[],
                &[],
                &[file("widget_fab.pdf", 128)],
                &[file("widget_assembly.svg", 128)],
                &[file("widget_vscore.dxf", 128)],
            )
            .is_empty()
        );
    }

    fn artifact(path: &str, text: &str) -> TextArtifact {
        TextArtifact {
            path: path.to_string(),
            text: text.to_string(),
        }
    }

    fn file(path: &str, byte_len: u64) -> FileArtifact {
        FileArtifact {
            path: path.to_string(),
            byte_len,
        }
    }

    fn messages(violations: &[crate::report::Violation]) -> Vec<String> {
        violations
            .iter()
            .filter_map(|violation| violation.message.clone())
            .collect()
    }
}
