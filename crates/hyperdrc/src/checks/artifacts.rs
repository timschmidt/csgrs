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
    let mut bom_sides = BTreeMap::<String, BTreeSet<String>>::new();
    let mut bom_values = BTreeMap::<String, BTreeSet<String>>::new();
    let mut bom_packages = BTreeMap::<String, BTreeSet<String>>::new();
    let mut bom_rotations = BTreeMap::<String, BTreeSet<String>>::new();
    let mut centroid_refs = BTreeSet::new();
    let mut centroid_sides = BTreeMap::<String, BTreeSet<String>>::new();
    let mut centroid_values = BTreeMap::<String, BTreeSet<String>>::new();
    let mut centroid_packages = BTreeMap::<String, BTreeSet<String>>::new();
    let mut centroid_rotations = BTreeMap::<String, BTreeSet<String>>::new();
    let mut netlist_refs = BTreeSet::new();
    let mut release_notes = String::new();

    for artifact in bom_files {
        violations.extend(analyze_text_artifact_path(artifact, TextArtifactKind::Bom));
        let report = analyze_bom(artifact);
        bom_refs.extend(report.refs);
        bom_not_populated_refs.extend(report.not_populated_refs);
        merge_side_maps(&mut bom_sides, report.sides);
        merge_side_maps(&mut bom_values, report.values);
        merge_side_maps(&mut bom_packages, report.packages);
        merge_side_maps(&mut bom_rotations, report.rotations);
        violations.extend(report.violations);
    }

    for artifact in centroid_files {
        violations.extend(analyze_text_artifact_path(
            artifact,
            TextArtifactKind::Centroid,
        ));
        let report = analyze_centroid(artifact);
        centroid_refs.extend(report.refs);
        merge_side_maps(&mut centroid_sides, report.sides);
        merge_side_maps(&mut centroid_values, report.values);
        merge_side_maps(&mut centroid_packages, report.packages);
        merge_side_maps(&mut centroid_rotations, report.rotations);
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
        release_notes.push_str(&artifact.text.to_ascii_lowercase());
        release_notes.push('\n');
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

    if !bom_sides.is_empty() && !centroid_sides.is_empty() {
        for (reference, bom_side_values) in &bom_sides {
            let Some(centroid_side_values) = centroid_sides.get(reference) else {
                continue;
            };
            if bom_side_values.is_disjoint(centroid_side_values) {
                violations.push(artifact_violation(
                    "bom-centroid-side-parity",
                    Some(format!(
                        "BOM places {reference} on {}, but centroid places it on {}",
                        join_set(bom_side_values),
                        join_set(centroid_side_values)
                    )),
                ));
            }
        }
    }
    compare_reference_metadata(
        &bom_values,
        &centroid_values,
        "bom-centroid-value-parity",
        "value/description",
        &mut violations,
    );
    compare_reference_metadata(
        &bom_packages,
        &centroid_packages,
        "bom-centroid-package-parity",
        "footprint/package",
        &mut violations,
    );
    compare_reference_metadata(
        &bom_rotations,
        &centroid_rotations,
        "bom-centroid-rotation-parity",
        "rotation",
        &mut violations,
    );

    if centroid_has_bottom_placements(&centroid_sides)
        && !readme_mentions_double_sided_assembly(&release_notes)
    {
        violations.push(artifact_violation(
            "double-sided-assembly-handoff",
            Some(
                "centroid includes bottom-side placements, but README does not mention double-sided or bottom-side assembly handoff"
                    .to_string(),
            ),
        ));
    }

    if !release_notes.trim().is_empty() {
        if readme_denies_panelization(&release_notes) && !rout_drawing_files.is_empty() {
            violations.push(artifact_violation(
                "readme-rout-drawing-parity",
                Some(
                    "README says the job is not panelized, but rout/panel drawing artifacts were provided"
                        .to_string(),
                ),
            ));
        }
        if readme_requests_panelization(&release_notes) && rout_drawing_files.is_empty() {
            violations.push(artifact_violation(
                "readme-rout-drawing-parity",
                Some(
                    "README requests panelization, tab route, or V-score handling, but no rout/panel drawing artifact was provided"
                        .to_string(),
                ),
            ));
        }
        if readme_requests_controlled_impedance(&release_notes) && fab_drawing_files.is_empty() {
            violations.push(artifact_violation(
                "readme-fab-drawing-parity",
                Some(
                    "README requests controlled impedance, but no fabrication drawing artifact was provided for stackup/coupon notes"
                        .to_string(),
                ),
            ));
        }
        if readme_requests_edge_or_castellation_process(&release_notes)
            && fab_drawing_files.is_empty()
        {
            violations.push(artifact_violation(
                "readme-fab-drawing-parity",
                Some(
                    "README requests edge plating or castellations, but no fabrication drawing artifact was provided for edge-finish notes"
                        .to_string(),
                ),
            ));
        }
        if readme_mentions_double_sided_assembly(&release_notes)
            && !centroid_has_bottom_placements(&centroid_sides)
        {
            violations.push(artifact_violation(
                "double-sided-assembly-handoff",
                Some(
                    "README mentions double-sided or bottom-side assembly, but centroid data has no bottom-side placements"
                        .to_string(),
                ),
            ));
        }
        if readme_mentions_double_sided_assembly(&release_notes)
            && assembly_drawing_files.is_empty()
        {
            violations.push(artifact_violation(
                "readme-assembly-drawing-parity",
                Some(
                    "README mentions double-sided or bottom-side assembly, but no assembly drawing artifact was provided"
                        .to_string(),
                ),
            ));
        }
        if (readme_mentions_selective_or_wave_solder(&release_notes)
            || readme_mentions_conformal_coating(&release_notes))
            && assembly_drawing_files.is_empty()
        {
            violations.push(artifact_violation(
                "readme-assembly-drawing-parity",
                Some(
                    "README mentions special assembly process notes, but no assembly drawing artifact was provided"
                        .to_string(),
                ),
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
    sides: BTreeMap<String, BTreeSet<String>>,
    values: BTreeMap<String, BTreeSet<String>>,
    packages: BTreeMap<String, BTreeSet<String>>,
    rotations: BTreeMap<String, BTreeSet<String>>,
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

const README_CONDITIONAL_ASSEMBLY_REQUIREMENTS: &[ReadmeRequirement] = &[
    ReadmeRequirement {
        label: "selective/wave solder process",
        needles: &[
            "selective solder",
            "wave solder",
            "solder pallet",
            "solder fixture",
            "solder thieves",
            "wave keepout",
            "pallet clearance",
            "thieving",
        ],
    },
    ReadmeRequirement {
        label: "conformal coating process",
        needles: &[
            "coating keepout",
            "mask coating",
            "coating fixture",
            "coating mask",
            "coat keepout",
            "no-clean",
            "cleanliness",
        ],
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
    let lifecycle_col = find_column(
        &table.headers,
        &[
            "lifecycle",
            "life cycle",
            "status",
            "partstatus",
            "part status",
        ],
    );
    let alternate_col = find_column(
        &table.headers,
        &[
            "alternate",
            "alternates",
            "approvedalternate",
            "approved alternate",
            "substitute",
            "substitution",
            "secondsource",
            "second source",
        ],
    );
    let value_col = find_column(&table.headers, &["value", "description"]);
    let package_col = find_column(&table.headers, &["package", "footprint", "case"]);
    let qty_col = find_column(&table.headers, &["qty", "quantity"]);
    let rotation_col = find_column(&table.headers, &["rotation", "rot", "angle", "orientation"]);
    let polarity_col = find_column(
        &table.headers,
        &[
            "polarity",
            "pin1",
            "pin 1",
            "orientationmark",
            "orientation mark",
            "cathode",
            "anode",
            "marking",
        ],
    );
    let moisture_col = find_column(
        &table.headers,
        &[
            "msl",
            "moisture",
            "moisturesensitivity",
            "moisture sensitivity",
            "bake",
            "drypack",
            "dry pack",
        ],
    );
    let height_col = find_column(
        &table.headers,
        &[
            "height",
            "maxheight",
            "max height",
            "componentheight",
            "component height",
            "zheight",
            "z height",
        ],
    );
    let side_col = find_column(
        &table.headers,
        &[
            "side",
            "mountside",
            "mount side",
            "assemblyside",
            "assembly side",
            "layer",
        ],
    );

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
    if lifecycle_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no lifecycle/status column".to_string()),
        ));
    }
    if alternate_col.is_none() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM header has no approved alternate/substitute column".to_string()),
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
    let mut row_sides = BTreeMap::<String, BTreeSet<String>>::new();
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
        if let Some(column) = side_col
            && !not_populated
        {
            let side = cell(row, column).trim().to_ascii_lowercase();
            if !side.is_empty() {
                if matches!(
                    side.as_str(),
                    "top" | "bottom" | "front" | "back" | "f" | "b" | "f.cu" | "b.cu"
                ) {
                    let side = normalize_side(&side).to_string();
                    for reference in &references {
                        row_sides
                            .entry(reference.clone())
                            .or_default()
                            .insert(side.clone());
                        analysis
                            .sides
                            .entry(reference.clone())
                            .or_default()
                            .insert(side.clone());
                    }
                } else {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} has invalid assembly side value {:?}",
                            row_index + 2,
                            cell(row, column)
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
        if let Some(column) = lifecycle_col
            && !not_populated
        {
            let lifecycle = cell(row, column).trim();
            if lifecycle.is_empty() {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} has no populated lifecycle/status",
                        row_index + 2
                    )),
                ));
            } else if is_risky_lifecycle(lifecycle) {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} lifecycle/status {:?} requires procurement review",
                        row_index + 2,
                        lifecycle
                    )),
                ));
            }
        }
        if let Some(column) = alternate_col
            && cell(row, column).trim().is_empty()
            && !not_populated
        {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no approved alternate/substitute",
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
        if !not_populated {
            if likely_polarized_bom_row(&references, row, part_col, value_col, package_col) {
                match polarity_col {
                    Some(column) if !cell(row, column).trim().is_empty() => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely contains polarized or pin-1-sensitive parts but has no populated polarity/orientation note",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely contains polarized or pin-1-sensitive parts but BOM has no polarity/orientation column",
                            row_index + 2
                        )),
                    )),
                }
            }
            if likely_moisture_sensitive_bom_row(row, part_col, value_col, package_col) {
                match moisture_col {
                    Some(column) if !cell(row, column).trim().is_empty() => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely uses an MSL-sensitive package but has no populated moisture/MSL handling note",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely uses an MSL-sensitive package but BOM has no moisture/MSL column",
                            row_index + 2
                        )),
                    )),
                }
            }
            if likely_height_sensitive_bom_row(&references, row, part_col, value_col, package_col) {
                match height_col {
                    Some(column) if parse_positive_dimension(cell(row, column)).is_some() => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs component-height review but has no valid populated height",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs component-height review but BOM has no component-height column",
                            row_index + 2
                        )),
                    )),
                }
            }
            if let Some(column) = rotation_col {
                let rotation_text = cell(row, column).trim();
                if !rotation_text.is_empty() {
                    match normalized_rotation(rotation_text) {
                        Some(rotation) => {
                            for reference in &references {
                                analysis
                                    .rotations
                                    .entry(reference.clone())
                                    .or_default()
                                    .insert(rotation.clone());
                            }
                        }
                        None => analysis.violations.push(artifact_violation(
                            &artifact.path,
                            Some(format!(
                                "BOM row {} has invalid rotation/orientation value {:?}",
                                row_index + 2,
                                rotation_text
                            )),
                        )),
                    }
                }
            }
            if let Some(column) = value_col {
                let value = normalize_bom_key(cell(row, column));
                if !value.is_empty() {
                    for reference in &references {
                        analysis
                            .values
                            .entry(reference.clone())
                            .or_default()
                            .insert(value.clone());
                    }
                }
            }
            if let Some(column) = package_col {
                let package = normalize_bom_key(cell(row, column));
                if !package.is_empty() {
                    for reference in &references {
                        analysis
                            .packages
                            .entry(reference.clone())
                            .or_default()
                            .insert(package.clone());
                    }
                }
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
    for (reference, sides) in row_sides {
        if sides.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM reference {reference} appears with multiple assembly sides: {}",
                    join_set(&sides)
                )),
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
    let value_col = find_column(&table.headers, &["value", "description", "comment"]);
    let package_col = find_column(&table.headers, &["package", "footprint", "case"]);

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
                } else if label == "rotation"
                    && let Some(reference) = &row_reference
                    && let Some(rotation) = normalized_rotation(cell(row, column))
                {
                    analysis
                        .rotations
                        .entry(reference.clone())
                        .or_default()
                        .insert(rotation);
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
                let side = normalize_side(&side).to_string();
                row_side = Some(side.clone());
                if let Some(reference) = &row_reference {
                    analysis
                        .sides
                        .entry(reference.clone())
                        .or_default()
                        .insert(side);
                }
            }
        }

        if let (Some(reference), Some(x), Some(y), Some(side)) =
            (row_reference.clone(), row_x, row_y, row_side)
        {
            placements.entry((x, y, side)).or_default().push(reference);
        }

        if let Some(reference) = &row_reference {
            if let Some(column) = value_col {
                let value = normalize_bom_key(cell(row, column));
                if !value.is_empty() {
                    analysis
                        .values
                        .entry(reference.clone())
                        .or_default()
                        .insert(value);
                }
            }
            if let Some(column) = package_col {
                let package = normalize_bom_key(cell(row, column));
                if !package.is_empty() {
                    analysis
                        .packages
                        .entry(reference.clone())
                        .or_default()
                        .insert(package);
                }
            }
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

    for (reference, values) in &analysis.values {
        if values.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "centroid reference {reference} appears with multiple values/descriptions: {}",
                    join_set(values)
                )),
            ));
        }
    }
    for (reference, packages) in &analysis.packages {
        if packages.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "centroid reference {reference} appears with multiple footprints/packages: {}",
                    join_set(packages)
                )),
            ));
        }
    }
    for (reference, rotations) in &analysis.rotations {
        if rotations.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "centroid reference {reference} appears with multiple rotations: {}",
                    join_set(rotations)
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

    if readme_mentions_selective_or_wave_solder(&normalized)
        && !has_any(
            &normalized,
            README_CONDITIONAL_ASSEMBLY_REQUIREMENTS[0].needles,
        )
    {
        violations.push(artifact_violation(
            &artifact.path,
            Some(
                "README artifact mentions through-hole/wave/selective assembly but not selective/wave solder process notes"
                    .to_string(),
            ),
        ));
    }

    if readme_mentions_conformal_coating(&normalized)
        && !has_any(
            &normalized,
            README_CONDITIONAL_ASSEMBLY_REQUIREMENTS[1].needles,
        )
    {
        violations.push(artifact_violation(
            &artifact.path,
            Some(
                "README artifact mentions conformal coating but not coating keepout, fixture, or cleanliness notes"
                    .to_string(),
            ),
        ));
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
    for (label, tokens) in [
        (
            "surface finish",
            &["enig", "hasl", "osp", "enepig", "hard gold"][..],
        ),
        (
            "soldermask color",
            &[
                "green mask",
                "black mask",
                "blue mask",
                "red mask",
                "white mask",
                "yellow mask",
                "purple mask",
            ][..],
        ),
        (
            "via treatment",
            &[
                "tented vias",
                "open vias",
                "untented vias",
                "filled vias",
                "plugged vias",
                "capped vias",
            ][..],
        ),
        (
            "board thickness",
            &[
                "0.6mm", "0.8mm", "1.0mm", "1.2mm", "1.6mm", "2.0mm", "2.4mm",
            ][..],
        ),
        ("copper weight", &["0.5 oz", "1 oz", "2 oz", "3 oz"][..]),
    ] {
        let matches = distinct_present_tokens(normalized, tokens);
        if matches.len() > 1 {
            violations.push(artifact_violation(
                path,
                Some(format!(
                    "README artifact contains contradictory {label} order intent: {}",
                    matches.join(", ")
                )),
            ));
        }
    }
    violations
}

fn distinct_present_tokens(text: &str, tokens: &[&str]) -> Vec<String> {
    tokens
        .iter()
        .filter(|token| text.contains(**token))
        .map(|token| (*token).to_string())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
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

fn normalized_rotation(value: &str) -> Option<String> {
    let numeric = normalize_numeric_cell(value).parse::<f64>().ok()?;
    if !numeric.is_finite() {
        return None;
    }
    let normalized = numeric.rem_euclid(360.0);
    Some(format!("{normalized:.3}"))
}

fn parse_positive_dimension(value: &str) -> Option<f64> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return None;
    }
    let numeric = trimmed
        .trim_end_matches("mm")
        .trim_end_matches("MM")
        .parse::<f64>()
        .ok()?;
    (numeric.is_finite() && numeric > 0.0).then_some(numeric)
}

fn normalize_side(side: &str) -> &'static str {
    match side {
        "top" | "front" | "f" | "f.cu" => "top",
        "bottom" | "back" | "b" | "b.cu" => "bottom",
        _ => "unknown",
    }
}

fn merge_side_maps(
    target: &mut BTreeMap<String, BTreeSet<String>>,
    source: BTreeMap<String, BTreeSet<String>>,
) {
    for (reference, sides) in source {
        target.entry(reference).or_default().extend(sides);
    }
}

fn compare_reference_metadata(
    bom_metadata: &BTreeMap<String, BTreeSet<String>>,
    centroid_metadata: &BTreeMap<String, BTreeSet<String>>,
    slug: &str,
    label: &str,
    violations: &mut Vec<Violation>,
) {
    if bom_metadata.is_empty() || centroid_metadata.is_empty() {
        return;
    }

    for (reference, bom_values) in bom_metadata {
        let Some(centroid_values) = centroid_metadata.get(reference) else {
            continue;
        };
        if bom_values.is_disjoint(centroid_values) {
            violations.push(artifact_violation(
                slug,
                Some(format!(
                    "BOM {label} for {reference} is {}, but centroid {label} is {}",
                    join_set(bom_values),
                    join_set(centroid_values)
                )),
            ));
        }
    }
}

fn join_set(values: &BTreeSet<String>) -> String {
    values.iter().cloned().collect::<Vec<_>>().join(", ")
}

fn centroid_has_bottom_placements(sides: &BTreeMap<String, BTreeSet<String>>) -> bool {
    sides
        .values()
        .any(|reference_sides| reference_sides.contains("bottom"))
}

fn readme_mentions_double_sided_assembly(text: &str) -> bool {
    has_any(
        text,
        &[
            "double-sided",
            "double sided",
            "two-sided",
            "two sided",
            "both sides",
            "bottom-side",
            "bottom side",
            "second side",
            "b-side",
            "b side",
        ],
    )
}

fn readme_mentions_selective_or_wave_solder(text: &str) -> bool {
    has_any(
        text,
        &[
            "through-hole",
            "through hole",
            "tht",
            "wave",
            "selective",
            "hand solder",
            "pin header",
        ],
    )
}

fn readme_mentions_conformal_coating(text: &str) -> bool {
    has_any(
        text,
        &[
            "conformal coating",
            "coated assembly",
            "coat board",
            "coating required",
        ],
    )
}

fn readme_requests_controlled_impedance(text: &str) -> bool {
    has_any(
        text,
        &[
            "controlled impedance required",
            "impedance required",
            "impedance coupon required",
            "controlled impedance: yes",
        ],
    )
}

fn readme_requests_edge_or_castellation_process(text: &str) -> bool {
    has_any(
        text,
        &[
            "edge plating required",
            "edge plated",
            "plated edge",
            "castellation required",
            "castellated",
            "castellations required",
        ],
    )
}

fn readme_denies_panelization(text: &str) -> bool {
    has_any(
        text,
        &[
            "no panel",
            "no panelization",
            "single board",
            "single-board",
            "individual board",
        ],
    )
}

fn readme_requests_panelization(text: &str) -> bool {
    has_any(
        text,
        &[
            "panelized",
            "panelised",
            "tab route",
            "mouse bite",
            "mouse-bite",
            "v-score",
            "vscore",
            "breakaway",
            "rails",
        ],
    )
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

fn likely_polarized_bom_row(
    references: &[String],
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    references.iter().any(|reference| {
        split_reference_designator(reference).is_some_and(|(prefix, _)| {
            matches!(
                prefix,
                "D" | "LED" | "U" | "Q" | "J" | "P" | "SW" | "K" | "BT"
            )
        })
    }) || row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "diode",
            "led",
            "schottky",
            "tvs",
            "zener",
            "mosfet",
            "transistor",
            "ic",
            "mcu",
            "processor",
            "connector",
            "header",
            "switch",
            "electrolytic",
            "polarized",
            "pin 1",
            "pin-1",
        ],
    )
}

fn likely_moisture_sensitive_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "bga", "lga", "qfn", "dfn", "wlcsp", "csp", "qfp", "tqfp", "module",
        ],
    )
}

fn likely_height_sensitive_bom_row(
    references: &[String],
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    references.iter().any(|reference| {
        split_reference_designator(reference).is_some_and(|(prefix, _)| {
            matches!(
                prefix,
                "J" | "P" | "CN" | "CON" | "SW" | "BT" | "K" | "L" | "T"
            )
        })
    }) || row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "connector",
            "header",
            "usb",
            "jack",
            "switch",
            "button",
            "battery",
            "terminal",
            "inductor",
            "transformer",
            "relay",
            "heatsink",
            "module",
            "display",
        ],
    )
}

fn row_text_matches(row: &[String], columns: &[Option<usize>], needles: &[&str]) -> bool {
    columns.iter().flatten().any(|column| {
        let text = cell(row, *column).to_ascii_lowercase();
        has_any(&text, needles)
    })
}

fn is_risky_lifecycle(value: &str) -> bool {
    let normalized = value.to_ascii_lowercase();
    has_any(
        &normalized,
        &[
            "obsolete",
            "eol",
            "end of life",
            "not recommended",
            "nrnd",
            "last time buy",
            "ltb",
        ],
    )
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
            "Ref,Qty,MPN,Value,Footprint,Manufacturer,Supplier SKU,Lifecycle,Approved Alternate,Side\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,RC0603_ALT,Top\nC1,1,CC0603,100nF,0603,Murata,SKU-C,Active,CC0603_ALT,Bottom\n",
        );
        let centroid = artifact(
            "centroid.csv",
            "Designator,X,Y,Rotation,Side\nR1,1.0,2.0,90,Top\nC1,3.0,4.0,0,Bottom\n",
        );
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
             Assembly: double-sided assembly, pin-1 and polarity reviewed. Test fixture and programming handoff complete.\n",
        );
        let assembly = file("widget_assembly.pdf", 256);

        assert!(
            production_artifact_readiness(
                &[bom],
                &[centroid],
                &[],
                &[readme],
                &[],
                &[assembly],
                &[],
            )
            .is_empty()
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
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier SKU,Lifecycle,Approved Alternate\nR1,1,RC0603,10k,0603,Yageo,SKU1,Active,ALT1\nR2,1,RC0603,1k,0402,,,NRND,\n",
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
                .any(|message| message.contains("requires procurement review"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no approved alternate"))
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
    fn bom_readiness_reports_missing_polarity_msl_and_height_handoff_metadata() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nD1,1,LED0603,LED,0603 LED,LiteOn,DistA,Active,ALT1\nU1,1,MCU123,MCU,QFN32,Vendor,DistB,Active,ALT2\nJ1,1,USB-C,USB connector,USB-C,Vendor,DistC,Active,ALT3\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("polarity/orientation column"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("moisture/MSL column"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("component-height column"))
        );
    }

    #[test]
    fn bom_readiness_allows_explicit_polarity_msl_and_height_handoff_metadata() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Polarity,MSL,Height\nD1,1,LED0603,LED,0603 LED,LiteOn,DistA,Active,ALT1,Cathode mark reviewed,1,0.8mm\nU1,1,MCU123,MCU,QFN32,Vendor,DistB,Active,ALT2,Pin 1 dot,3,0.9\nJ1,1,USB-C,USB connector,USB-C,Vendor,DistC,Active,ALT3,Pin 1 on shell,1,3.2mm\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("polarity/orientation"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("moisture/MSL"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("component-height"))
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
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\n\"r1,r2 r3;R4|R5\",5,RC0603,10k,0603,Yageo,DistA,Active,RC0603_ALT\nD1,0,DNP,,,,,,\n",
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
    fn bom_centroid_side_parity_and_bottom_side_handoff_are_checked() {
        let bom = artifact(
            "bom.csv",
            "Ref,Qty,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Side\nR1,1,RC0603,10k,0603,Yageo,DistA,Active,ALT1,Top\nC1,1,CC0603,100nF,0603,Murata,DistB,Active,ALT2,Top\nU1,1,MCU,QFN,QFN32,Vendor,DistC,Active,ALT3,Middle\n",
        );
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nR1,0,0,0,Top\nC1,3,0,0,Bottom\nU1,6,0,0,Top\n",
        );
        let readme = artifact(
            "README.md",
            "Revision G. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, panelization none, tented vias, \
             no edge plating, no castellation. DRC/ERC passed, zones refilled, outputs generated, \
             Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity \
             reviewed. Test fixture handoff complete.\n",
        );

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("invalid assembly side"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("BOM places C1 on top")
                    && message.contains("centroid places it on bottom"))
        );
        assert!(messages.iter().any(
            |message| message.contains("bottom-side placements") && message.contains("README")
        ));
    }

    #[test]
    fn bom_centroid_value_package_and_rotation_parity_are_checked_when_available() {
        let bom = artifact(
            "bom.csv",
            "Ref,Qty,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Side,Rotation\nR1,1,RC0603,10k,0603,Yageo,DistA,Active,ALT1,Top,90\nC1,1,CC0603,100nF,0603,Murata,DistB,Active,ALT2,Top,180\n",
        );
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side,Value,Footprint\nR1,0,0,0,Top,1k,0402\nC1,3,0,-180,Top,100nF,0603\n",
        );

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(messages.iter().any(|message| {
            message.contains("BOM value/description for R1 is 10K")
                && message.contains("centroid value/description is 1K")
        }));
        assert!(messages.iter().any(|message| {
            message.contains("BOM footprint/package for R1 is 0603")
                && message.contains("centroid footprint/package is 0402")
        }));
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("BOM value/description for C1"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("BOM rotation for R1 is 90.000")
                    && message.contains("centroid rotation is 0.000"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("BOM rotation for C1"))
        );
    }

    #[test]
    fn centroid_readiness_reports_conflicting_value_package_and_rotation_metadata() {
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side,Value,Footprint\nR1,0,0,0,Top,10k,0603\nR1,1,0,90,Top,1k,0402\n",
        );

        let violations = production_artifact_readiness(&[], &[centroid], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple values") && message.contains("R1"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple footprints") && message.contains("R1"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple rotations") && message.contains("R1"))
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
    fn readme_readiness_reports_conflicting_order_parameters() {
        let readme = artifact(
            "README.md",
            "Revision F. Fabrication package. Stackup 4 layer, 0.8mm and 1.6mm board thickness, \
             1 oz and 2 oz copper weight. Finish: ENIG and HASL. Green mask and black mask. \
             No impedance. Panelization none. Tented vias and filled vias. No edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer \
             checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. \
             Test fixture/programming handoff complete.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        for label in [
            "surface finish",
            "soldermask color",
            "via treatment",
            "board thickness",
            "copper weight",
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
    fn readme_readiness_checks_panel_and_double_sided_handoff_parity() {
        let readme = artifact(
            "README.md",
            "Revision H. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance. Panelization: no panel, single board only. \
             Tented vias, no edge plating, no castellation. DRC/ERC passed, zones refilled, outputs \
             generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. \
             Assembly: double-sided assembly, pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\nR1,0,0,0,Top\n");
        let rout = file("widget_panel_route.dxf", 256);

        let violations =
            production_artifact_readiness(&[], &[centroid], &[], &[readme], &[], &[], &[rout]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("not panelized")
                    && message.contains("rout/panel drawing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("double-sided")
                    && message.contains("no bottom-side placements"))
        );
    }

    #[test]
    fn readme_readiness_requires_rout_artifact_when_panelization_is_requested() {
        let readme = artifact(
            "README.md",
            "Revision I. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance. Panelized tab route with mouse-bite rails. \
             Tented vias, no edge plating, no castellation. DRC/ERC passed, zones refilled, outputs \
             generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. \
             Assembly: pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("requests panelization")
                    && message.contains("no rout/panel drawing"))
        );
    }

    #[test]
    fn readme_readiness_requires_conditional_assembly_process_notes() {
        let readme = artifact(
            "README.md",
            "Revision J. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance. Panelization none. Tented vias, no edge plating, \
             no castellation. Assembly includes through-hole pin headers and conformal coating. DRC/ERC passed, \
             zones refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive \
             created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("selective/wave solder process notes"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("coating keepout")
                    && message.contains("cleanliness"))
        );
    }

    #[test]
    fn readme_readiness_allows_conditional_assembly_process_notes() {
        let readme = artifact(
            "README.md",
            "Revision K. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance. Panelization none. Tented vias, no edge plating, \
             no castellation. Assembly includes through-hole pin headers; selective solder process uses solder \
             pallet clearance and solder thieves. Conformal coating required with coating keepout, coating fixture, \
             and cleanliness/no-clean flux review. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer \
             checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. Test fixture \
             handoff complete.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("selective/wave solder process notes"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("coating keepout")
                    && message.contains("cleanliness"))
        );
    }

    #[test]
    fn readme_readiness_requires_drawings_for_special_fabrication_and_assembly_handoffs() {
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\nU1,0,0,0,Bottom\n");
        let readme = artifact(
            "README.md",
            "Revision L. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green. Controlled impedance required with coupon. Panelization none. \
             Tented vias, edge plating required, castellated edge required. Assembly: double-sided assembly \
             with through-hole pin header; selective solder process uses solder pallet clearance. Conformal \
             coating required with coating keepout and cleanliness review. DRC/ERC passed, zones refilled, \
             outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. \
             Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let violations =
            production_artifact_readiness(&[], &[centroid], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("controlled impedance")
                    && message.contains("no fabrication drawing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("edge plating or castellations")
                    && message.contains("no fabrication drawing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("double-sided")
                    && message.contains("no assembly drawing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("special assembly process")
                    && message.contains("no assembly drawing"))
        );
    }

    #[test]
    fn readme_readiness_accepts_drawings_for_special_fabrication_and_assembly_handoffs() {
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\nU1,0,0,0,Bottom\n");
        let readme = artifact(
            "README.md",
            "Revision M. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green. Controlled impedance required with coupon. Panelization none. \
             Tented vias, edge plating required. Assembly: double-sided assembly with through-hole pin header; \
             selective solder process uses solder pallet clearance. DRC/ERC passed, zones refilled, outputs \
             generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity \
             reviewed. Test fixture handoff complete.\n",
        );
        let fab = file("widget_fab.pdf", 256);
        let assembly = file("widget_assembly.pdf", 256);

        let messages = messages(&production_artifact_readiness(
            &[],
            &[centroid],
            &[],
            &[readme],
            &[fab],
            &[assembly],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("no fabrication drawing"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("no assembly drawing"))
        );
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
