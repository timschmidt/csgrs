//! Pre-production sidecar artifact checks.
//!
//! These checks validate lightweight BOM and centroid structure before a package
//! reaches assembly quoting or programming/test fixture review. The parser is
//! intentionally conservative: it understands common CSV/TSV headers and emits
//! review warnings rather than trying to become a full spreadsheet engine.

use std::collections::{BTreeMap, BTreeSet};

use super::artifact_table::{cell, find_column, parse_table};
use super::surface_finish::readme_surface_finish_compatibility;
use crate::report::{Severity, Violation};

#[derive(Clone, Debug)]
/// Public data model for `TextArtifact`.
pub struct TextArtifact {
    /// Field `path`.
    pub path: String,
    /// Field `text`.
    pub text: String,
}

#[derive(Clone, Debug)]
/// Public data model for `FileArtifact`.
pub struct FileArtifact {
    /// Field `path`.
    pub path: String,
    /// Field `byte_len`.
    pub byte_len: u64,
}

/// Run the `production_artifact_readiness` design-readiness check or report helper.
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
    let mut release_markers = Vec::<(String, String)>::new();
    let mut through_hole_refs = BTreeSet::new();
    let mut inspection_sensitive_refs = BTreeSet::new();
    let mut programmable_refs = BTreeSet::new();
    let mut assembly_variants = BTreeMap::<String, BTreeSet<String>>::new();

    for artifact in bom_files {
        release_markers.push(("BOM".to_string(), artifact.path.clone()));
        violations.extend(analyze_text_artifact_path(artifact, TextArtifactKind::Bom));
        let report = analyze_bom(artifact);
        bom_refs.extend(report.refs);
        bom_not_populated_refs.extend(report.not_populated_refs);
        through_hole_refs.extend(report.through_hole_refs);
        inspection_sensitive_refs.extend(report.inspection_sensitive_refs);
        programmable_refs.extend(report.programmable_refs);
        merge_side_maps(&mut assembly_variants, report.assembly_variants);
        merge_side_maps(&mut bom_sides, report.sides);
        merge_side_maps(&mut bom_values, report.values);
        merge_side_maps(&mut bom_packages, report.packages);
        merge_side_maps(&mut bom_rotations, report.rotations);
        violations.extend(report.violations);
    }

    for artifact in centroid_files {
        release_markers.push(("centroid".to_string(), artifact.path.clone()));
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
        release_markers.push(("netlist".to_string(), artifact.path.clone()));
        violations.extend(analyze_text_artifact_path(
            artifact,
            TextArtifactKind::Netlist,
        ));
        let report = analyze_netlist(artifact);
        netlist_refs.extend(report.refs);
        violations.extend(report.violations);
    }

    for artifact in readme_files {
        release_markers.push(("README path".to_string(), artifact.path.clone()));
        release_markers.push(("README content".to_string(), artifact.text.clone()));
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
    for artifact in fab_drawing_files {
        release_markers.push(("fabrication drawing".to_string(), artifact.path.clone()));
    }
    violations.extend(analyze_file_artifacts(
        assembly_drawing_files,
        DrawingKind::Assembly,
    ));
    for artifact in assembly_drawing_files {
        release_markers.push(("assembly drawing".to_string(), artifact.path.clone()));
    }
    violations.extend(analyze_file_artifacts(
        rout_drawing_files,
        DrawingKind::Rout,
    ));
    for artifact in rout_drawing_files {
        release_markers.push(("rout drawing".to_string(), artifact.path.clone()));
    }
    violations.extend(analyze_release_marker_consistency(&release_markers));

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

    if (centroid_has_bottom_placements(&centroid_sides)
        || centroid_has_bottom_placements(&bom_sides))
        && !readme_mentions_double_sided_assembly(&release_notes)
    {
        violations.push(artifact_violation(
            "double-sided-assembly-handoff",
            Some(
                "BOM or centroid data includes bottom-side placements, but README does not mention double-sided or bottom-side assembly handoff"
                    .to_string(),
            ),
        ));
    }

    if !through_hole_refs.is_empty() && !readme_mentions_selective_or_wave_solder(&release_notes) {
        violations.push(artifact_violation(
            "bom-readme-assembly-process-parity",
            Some(format!(
                "BOM includes likely through-hole or hand-soldered references ({}) but README does not mention selective, wave, or hand-solder process notes",
                join_limited_set(&through_hole_refs, 8)
            )),
        ));
    }

    if !inspection_sensitive_refs.is_empty() && !readme_mentions_inspection_process(&release_notes)
    {
        violations.push(artifact_violation(
            "bom-readme-assembly-process-parity",
            Some(format!(
                "BOM includes likely BGA/CSP/LGA inspection-sensitive references ({}) but README does not mention X-ray, AOI, or inspection handoff",
                join_limited_set(&inspection_sensitive_refs, 8)
            )),
        ));
    }

    if !programmable_refs.is_empty() && !readme_mentions_programming_handoff(&release_notes) {
        violations.push(artifact_violation(
            "bom-readme-assembly-process-parity",
            Some(format!(
                "BOM includes likely programmable references ({}) but README does not mention firmware, programming, or test handoff",
                join_limited_set(&programmable_refs, 8)
            )),
        ));
    }
    if !programmable_refs.is_empty() && readme_mentions_programming_handoff(&release_notes) {
        if !readme_mentions_firmware_revision(&release_notes) {
            violations.push(artifact_violation(
                "bom-readme-programming-traceability",
                Some(format!(
                    "BOM includes likely programmable references ({}) but README programming handoff does not mention firmware revision, image, or checksum traceability",
                    join_limited_set(&programmable_refs, 8)
                )),
            ));
        }
        if !readme_mentions_programming_method(&release_notes) {
            violations.push(artifact_violation(
                "bom-readme-programming-method",
                Some(format!(
                    "BOM includes likely programmable references ({}) but README programming handoff does not mention SWD/JTAG/bootloader/fixture method",
                    join_limited_set(&programmable_refs, 8)
                )),
            ));
        }
        if !readme_mentions_test_acceptance(&release_notes) {
            violations.push(artifact_violation(
                "bom-readme-test-acceptance",
                Some(format!(
                    "BOM includes likely programmable references ({}) but README does not mention functional-test acceptance, pass/fail, or test record criteria",
                    join_limited_set(&programmable_refs, 8)
                )),
            ));
        }
    }

    if assembly_variants.len() > 1 && !readme_mentions_assembly_variants(&release_notes) {
        violations.push(artifact_violation(
            "bom-readme-variant-parity",
            Some(format!(
                "BOM includes multiple assembly/build variants ({}) but README does not describe variant handling",
                describe_marker_map(&assembly_variants)
            )),
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
        if readme_mentions_fabrication_marking(&release_notes) && fab_drawing_files.is_empty() {
            violations.push(artifact_violation(
                "readme-fab-drawing-parity",
                Some(
                    "README mentions fabrication markings, labels, date codes, UL marks, or serialization, but no fabrication drawing artifact was provided for allowed marking zones"
                        .to_string(),
                ),
            ));
        }
        if readme_mentions_serialization(&release_notes)
            && !readme_mentions_serialization_handoff(&release_notes)
        {
            violations.push(artifact_violation(
                "readme-serialization-handoff",
                Some(
                    "README mentions serialization or barcodes but not serial format, label location, range, or traceability handoff"
                        .to_string(),
                ),
            ));
        }
        if readme_mentions_packaging(&release_notes)
            && !readme_mentions_packaging_handoff(&release_notes)
        {
            violations.push(artifact_violation(
                "readme-packaging-handoff",
                Some(
                    "README mentions packaging or shipping but not ESD, moisture, labeling, lot, or tray/reel handling notes"
                        .to_string(),
                ),
            ));
        }
        if readme_mentions_double_sided_assembly(&release_notes)
            && !centroid_has_bottom_placements(&centroid_sides)
            && !centroid_has_bottom_placements(&bom_sides)
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
    through_hole_refs: BTreeSet<String>,
    inspection_sensitive_refs: BTreeSet<String>,
    programmable_refs: BTreeSet<String>,
    assembly_variants: BTreeMap<String, BTreeSet<String>>,
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
    ReadmeRequirement {
        label: "first-article or sample approval",
        needles: &[
            "first article",
            "fai",
            "sample approval",
            "golden sample",
            "pilot build",
        ],
    },
    ReadmeRequirement {
        label: "production acceptance criteria",
        needles: &[
            "acceptance criteria",
            "acceptance",
            "pass/fail",
            "pass fail",
            "aql",
            "inspection criteria",
        ],
    },
    ReadmeRequirement {
        label: "lot traceability",
        needles: &[
            "traceability",
            "lot",
            "date code",
            "coc",
            "certificate of conformance",
            "traveler",
        ],
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
    if table.rows.is_empty() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("BOM file has no component rows".to_string()),
        ));
    }

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
    let unit_cost_col = find_column(
        &table.headers,
        &[
            "unitcost",
            "unit cost",
            "price",
            "cost",
            "extendedcost",
            "extended cost",
        ],
    );
    let population_col = find_column(
        &table.headers,
        &[
            "population",
            "populate",
            "fitted",
            "fit",
            "mount",
            "mounted",
        ],
    );
    let variant_col = find_column(
        &table.headers,
        &[
            "assemblyoption",
            "assembly option",
            "buildvariant",
            "build variant",
            "variant",
            "variantname",
            "variant name",
            "bomvariant",
            "bom variant",
        ],
    );
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
    let compliance_col = find_column(
        &table.headers,
        &[
            "compliance",
            "rohs",
            "reach",
            "leadfree",
            "lead free",
            "materialdeclaration",
            "material declaration",
        ],
    );
    let traceability_col = find_column(
        &table.headers,
        &[
            "traceability",
            "lot",
            "lotcode",
            "lot code",
            "datecode",
            "date code",
            "coc",
            "certificate",
        ],
    );
    let source_control_col = find_column(
        &table.headers,
        &[
            "sourcecontrol",
            "source control",
            "approvedvendor",
            "approved vendor",
            "AVL",
            "authorized",
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
    let mut mpn_manufacturers = BTreeMap::<String, BTreeSet<String>>::new();
    let mut mpn_suppliers = BTreeMap::<String, BTreeSet<String>>::new();
    let mut mpn_lifecycle = BTreeMap::<String, BTreeSet<String>>::new();
    let mut supplier_parts = BTreeMap::<String, BTreeSet<String>>::new();
    let mut row_sides = BTreeMap::<String, BTreeSet<String>>::new();
    for (row_index, row) in table.rows.iter().enumerate() {
        let not_populated = is_not_populated_row(row)
            || population_col
                .is_some_and(|column| is_not_populated_population_cell(cell(row, column)));
        let references = ref_col
            .map(|column| split_references(cell(row, column)))
            .unwrap_or_default();
        if ref_col.is_some() && references.is_empty() && !not_populated {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM row {} has no reference designator",
                    row_index + 2
                )),
            ));
        }
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
        if let Some(column) = variant_col
            && !not_populated
            && let Some(variant) = released_bom_key(cell(row, column))
        {
            analysis
                .assembly_variants
                .entry(variant)
                .or_default()
                .extend(references.iter().cloned());
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
            && is_unreleased_cell(cell(row, column))
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
            && is_unreleased_cell(cell(row, column))
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
            && is_unreleased_cell(cell(row, column))
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
            if is_unreleased_cell(lifecycle) {
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
            && is_unreleased_cell(cell(row, column))
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
        if !not_populated
            && let (Some(part_column), Some(alternate_column)) = (part_col, alternate_col)
        {
            let part = released_bom_key(cell(row, part_column));
            let alternate = released_bom_key(cell(row, alternate_column));
            if let (Some(part), Some(alternate)) = (part, alternate)
                && part == alternate
            {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} approved alternate/substitute is the same as the primary part",
                        row_index + 2
                    )),
                ));
            }
        }
        if let Some(column) = value_col
            && is_unreleased_cell(cell(row, column))
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
            && is_unreleased_cell(cell(row, column))
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
            let part = released_bom_key(cell(row, part_column));
            let value = released_bom_key(cell(row, value_column));
            let package = released_bom_key(cell(row, package_column));
            if let (Some(part), Some(value)) = (&part, value) {
                mpn_values.entry(part.clone()).or_default().insert(value);
            }
            if let (Some(part), Some(package)) = (part, package) {
                mpn_packages.entry(part).or_default().insert(package);
            }
        }
        if !not_populated && let Some(part_column) = part_col {
            if let Some(part) = released_bom_key(cell(row, part_column)) {
                if let Some(column) = manufacturer_col {
                    if let Some(manufacturer) = released_bom_key(cell(row, column)) {
                        mpn_manufacturers
                            .entry(part.clone())
                            .or_default()
                            .insert(manufacturer);
                    }
                }
                if let Some(column) = supplier_col {
                    if let Some(supplier) = released_bom_key(cell(row, column)) {
                        mpn_suppliers
                            .entry(part.clone())
                            .or_default()
                            .insert(supplier.clone());
                        supplier_parts
                            .entry(supplier)
                            .or_default()
                            .insert(part.clone());
                    }
                }
                if let Some(column) = lifecycle_col {
                    if let Some(lifecycle) = released_bom_key(cell(row, column)) {
                        mpn_lifecycle
                            .entry(part.clone())
                            .or_default()
                            .insert(lifecycle);
                    }
                }
            }
        }
        if !not_populated {
            if likely_through_hole_bom_row(&references, row, part_col, value_col, package_col) {
                analysis
                    .through_hole_refs
                    .extend(references.iter().cloned());
            }
            if likely_inspection_sensitive_bom_row(row, part_col, value_col, package_col) {
                analysis
                    .inspection_sensitive_refs
                    .extend(references.iter().cloned());
            }
            if likely_programmable_bom_row(row, part_col, value_col, package_col) {
                analysis
                    .programmable_refs
                    .extend(references.iter().cloned());
            }
            if likely_polarized_bom_row(&references, row, part_col, value_col, package_col) {
                match polarity_col {
                    Some(column) if !is_unreleased_cell(cell(row, column)) => {}
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
                    Some(column) if !is_unreleased_cell(cell(row, column)) => {}
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
            if likely_traceability_sensitive_bom_row(
                row,
                part_col,
                value_col,
                package_col,
                lifecycle_col,
            ) {
                // IPC J-STD-001H frames assembly acceptance as process and
                // material-control evidence, while IPC-7351B makes package
                // class central to land-pattern/assembly interpretation. These
                // BOM heuristics therefore ask for explicit lot/date-code or
                // certificate traceability on packages and lifecycle states
                // that often drive production risk.
                match traceability_col {
                    Some(column) if !is_unreleased_cell(cell(row, column)) => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs lot/date-code or certificate traceability but has no populated traceability note",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs lot/date-code or certificate traceability but BOM has no traceability column",
                            row_index + 2
                        )),
                    )),
                }
            }
            if likely_regulatory_sensitive_bom_row(row, part_col, value_col, package_col) {
                match compliance_col {
                    Some(column) if !is_unreleased_cell(cell(row, column)) => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs RoHS/REACH/lead-free compliance evidence but has no populated compliance note",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs RoHS/REACH/lead-free compliance evidence but BOM has no compliance column",
                            row_index + 2
                        )),
                    )),
                }
            }
            if likely_source_control_sensitive_bom_row(row, part_col, value_col, package_col) {
                match source_control_col {
                    Some(column) if !is_unreleased_cell(cell(row, column)) => {}
                    Some(_) => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs approved-vendor/source-control evidence but has no populated source-control note",
                            row_index + 2
                        )),
                    )),
                    None => analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "BOM row {} likely needs approved-vendor/source-control evidence but BOM has no source-control column",
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
            if let Some(0) = quantity
                && !not_populated
            {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} has zero quantity but is not marked DNP/DNI or not fitted",
                        row_index + 2
                    )),
                ));
            }
        }
        if let Some(column) = unit_cost_col
            && !not_populated
        {
            let cost_text = cell(row, column).trim();
            if is_unreleased_cell(cost_text) {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} has no populated unit cost or price",
                        row_index + 2
                    )),
                ));
            } else if parse_non_negative_money(cost_text).is_none() {
                analysis.violations.push(artifact_violation(
                    &artifact.path,
                    Some(format!(
                        "BOM row {} has invalid unit cost/price {:?}",
                        row_index + 2,
                        cost_text
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
    for (part, manufacturers) in mpn_manufacturers {
        if manufacturers.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM part {part} is used with multiple manufacturers: {}",
                    manufacturers.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }
    for (part, suppliers) in mpn_suppliers {
        if suppliers.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM part {part} is used with multiple supplier/distributor/SKU values: {}",
                    suppliers.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }
    for (part, lifecycle_values) in mpn_lifecycle {
        if lifecycle_values.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM part {part} is used with multiple lifecycle/status values: {}",
                    lifecycle_values.into_iter().collect::<Vec<_>>().join(", ")
                )),
            ));
        }
    }
    for (supplier, parts) in supplier_parts {
        if parts.len() > 1 {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!(
                    "BOM supplier/distributor/SKU value {supplier} is assigned to multiple parts: {}",
                    parts.into_iter().collect::<Vec<_>>().join(", ")
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
    if table.rows.is_empty() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("centroid file has no placement rows".to_string()),
        ));
    }

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
    // IPC-7351B land-pattern and assembly data is only meaningful when the
    // placement origin, side, units, and rotation convention are unambiguous.
    // Many EDA centroid exports omit that context, so hyperdrc treats missing
    // package-level metadata as a production-handoff warning rather than trying
    // to infer the assembly house convention.
    let normalized_text = artifact.text.to_ascii_lowercase();
    if !has_any(
        &normalized_text,
        &[
            "unit",
            "units",
            "mm",
            "millimeter",
            "millimetre",
            "inch",
            "inches",
        ],
    ) {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some(
                "centroid file does not state placement units; review mm/inch export settings"
                    .to_string(),
            ),
        ));
    }
    if !has_any(
        &normalized_text,
        &[
            "origin",
            "aux origin",
            "grid origin",
            "board origin",
            "absolute",
            "relative",
        ],
    ) {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some(
                "centroid file does not state placement origin; review board/aux-origin convention"
                    .to_string(),
            ),
        ));
    }
    if rotation_col.is_some()
        && !has_any(
            &normalized_text,
            &[
                "rotation convention",
                "degrees",
                "clockwise",
                "counterclockwise",
                "ccw",
            ],
        )
    {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("centroid file has rotations but does not state rotation convention".to_string()),
        ));
    }

    let mut occurrences = BTreeMap::<String, usize>::new();
    let mut placements = BTreeMap::<(String, String, String), Vec<String>>::new();
    for (row_index, row) in table.rows.iter().enumerate() {
        let mut row_reference = None;
        if let Some(column) = ref_col {
            let reference = cell(row, column).trim();
            if is_unreleased_cell(reference) {
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
                if matches!(label, "x" | "y") && numeric.abs() > 5_000.0 {
                    analysis.violations.push(artifact_violation(
                        &artifact.path,
                        Some(format!(
                            "centroid row {} {label} coordinate {:?} is unusually large; review placement units and origin",
                            row_index + 2,
                            cell(row, column)
                        )),
                    ));
                }
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
                if let Some(value) = released_bom_key(cell(row, column)) {
                    analysis
                        .values
                        .entry(reference.clone())
                        .or_default()
                        .insert(value);
                }
            }
            if let Some(column) = package_col {
                if let Some(package) = released_bom_key(cell(row, column)) {
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
    if table.rows.is_empty() {
        analysis.violations.push(artifact_violation(
            &artifact.path,
            Some("netlist file has no pin rows".to_string()),
        ));
    }

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
        let normalized_reference =
            (!is_unreleased_cell(reference)).then(|| normalize_reference(reference));

        if is_unreleased_cell(net) {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist row {} has no net name", row_index + 2)),
            ));
        }
        if is_unreleased_cell(reference) {
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
        if is_unreleased_cell(pin) {
            analysis.violations.push(artifact_violation(
                &artifact.path,
                Some(format!("netlist row {} has no pin/pad", row_index + 2)),
            ));
        }

        if let Some(reference) = normalized_reference
            && !is_unreleased_cell(pin)
            && !is_unreleased_cell(net)
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

    violations.extend(readme_surface_finish_compatibility(
        &artifact.path,
        &normalized,
    ));
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
        (
            &["no conformal coating", "coating not required"][..],
            &["conformal coating required", "coating required"][..],
            "conformal coating",
        ),
        (
            &["no programming", "programming not required", "no firmware"][..],
            &[
                "programming required",
                "firmware required",
                "flash firmware",
            ][..],
            "programming",
        ),
        (
            &[
                "no test fixture",
                "fixture not required",
                "no ict",
                "no fct",
            ][..],
            &["test fixture required", "ict required", "fct required"][..],
            "test fixture",
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
        (
            "layer count",
            &[
                "1 layer", "1-layer", "2 layer", "2-layer", "4 layer", "4-layer", "6 layer",
                "6-layer", "8 layer", "8-layer", "10 layer", "10-layer", "12 layer", "12-layer",
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

fn analyze_release_marker_consistency(markers: &[(String, String)]) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut revisions = BTreeMap::<String, BTreeSet<String>>::new();
    let mut dates = BTreeMap::<String, BTreeSet<String>>::new();

    for (label, text) in markers {
        for revision in extract_revision_markers(text) {
            revisions.entry(revision).or_default().insert(label.clone());
        }
        for date in extract_date_markers(text) {
            dates.entry(date).or_default().insert(label.clone());
        }
    }

    if revisions.len() > 1 {
        violations.push(artifact_violation(
            "release-marker-consistency",
            Some(format!(
                "release artifacts mention multiple revision markers: {}",
                describe_marker_map(&revisions)
            )),
        ));
    }
    if dates.len() > 1 {
        violations.push(artifact_violation(
            "release-marker-consistency",
            Some(format!(
                "release artifacts mention multiple generated/release dates: {}",
                describe_marker_map(&dates)
            )),
        ));
    }

    violations
}

fn describe_marker_map(markers: &BTreeMap<String, BTreeSet<String>>) -> String {
    markers
        .iter()
        .map(|(marker, labels)| format!("{marker} in {}", join_set(labels)))
        .collect::<Vec<_>>()
        .join("; ")
}

fn extract_revision_markers(text: &str) -> BTreeSet<String> {
    let tokens = release_tokens(text);
    let mut revisions = BTreeSet::new();
    for (index, token) in tokens.iter().enumerate() {
        if matches!(token.as_str(), "rev" | "revision" | "version") {
            if let Some(next) = tokens.get(index + 1)
                && looks_like_revision_value(next)
            {
                revisions.insert(format!("rev{}", next.to_ascii_uppercase()));
            }
        } else if let Some(suffix) = token.strip_prefix("rev")
            && looks_like_compact_revision_suffix(suffix)
        {
            revisions.insert(format!("rev{}", suffix.to_ascii_uppercase()));
        } else if let Some(suffix) = token.strip_prefix('v')
            && suffix.chars().next().is_some_and(|ch| ch.is_ascii_digit())
            && looks_like_revision_value(suffix)
        {
            revisions.insert(format!("rev{}", suffix.to_ascii_uppercase()));
        }
    }
    revisions
}

fn looks_like_revision_value(value: &str) -> bool {
    let len = value.len();
    (1..=8).contains(&len) && value.chars().all(|ch| ch.is_ascii_alphanumeric())
}

fn looks_like_compact_revision_suffix(value: &str) -> bool {
    let len = value.len();
    (1..=4).contains(&len) && value.chars().all(|ch| ch.is_ascii_alphanumeric())
}

fn extract_date_markers(text: &str) -> BTreeSet<String> {
    let tokens = release_tokens(text);
    let mut dates = BTreeSet::new();
    for (index, token) in tokens.iter().enumerate() {
        if token.len() == 8 && token.chars().all(|ch| ch.is_ascii_digit()) {
            let year = &token[0..4];
            let month = &token[4..6];
            let day = &token[6..8];
            if is_plausible_date_parts(year, month, day) {
                dates.insert(format!("{year}-{month}-{day}"));
            }
        }
        if token.len() == 4
            && token.chars().all(|ch| ch.is_ascii_digit())
            && let (Some(month), Some(day)) = (tokens.get(index + 1), tokens.get(index + 2))
            && is_plausible_date_parts(token, month, day)
        {
            dates.insert(format!("{token}-{month:0>2}-{day:0>2}"));
        }
    }
    dates
}

fn is_plausible_date_parts(year: &str, month: &str, day: &str) -> bool {
    let Ok(year) = year.parse::<u16>() else {
        return false;
    };
    let Ok(month) = month.parse::<u8>() else {
        return false;
    };
    let Ok(day) = day.parse::<u8>() else {
        return false;
    };
    (2000..=2099).contains(&year) && (1..=12).contains(&month) && (1..=31).contains(&day)
}

fn release_tokens(text: &str) -> Vec<String> {
    text.split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .map(|token| token.to_ascii_lowercase())
        .collect()
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

fn released_bom_key(value: &str) -> Option<String> {
    (!is_unreleased_cell(value)).then(|| normalize_bom_key(value))
}

fn is_unreleased_cell(value: &str) -> bool {
    let normalized = value
        .trim()
        .trim_matches('"')
        .trim_matches('\'')
        .to_ascii_lowercase();
    normalized.is_empty()
        || matches!(
            normalized.as_str(),
            "tbd"
                | "todo"
                | "unknown"
                | "unk"
                | "n/a"
                | "na"
                | "none"
                | "-"
                | "--"
                | "?"
                | "pending"
                | "placeholder"
                | "select"
                | "choose"
        )
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

fn parse_non_negative_money(value: &str) -> Option<f64> {
    let compact = value
        .trim()
        .trim_start_matches('$')
        .replace([',', '€', '£'], "");
    let numeric = compact.parse::<f64>().ok()?;
    (numeric.is_finite() && numeric >= 0.0).then_some(numeric)
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

fn join_limited_set(values: &BTreeSet<String>, limit: usize) -> String {
    let mut joined = values.iter().take(limit).cloned().collect::<Vec<_>>();
    if values.len() > limit {
        joined.push(format!("and {} more", values.len() - limit));
    }
    joined.join(", ")
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

fn readme_mentions_inspection_process(text: &str) -> bool {
    has_any(
        text,
        &[
            "x-ray",
            "xray",
            "x ray",
            "aoi",
            "automated optical inspection",
            "inspection",
            "inspect",
            "bga review",
            "voiding review",
        ],
    )
}

fn readme_mentions_programming_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "programming",
            "program",
            "firmware",
            "flashing",
            "flash",
            "bootloader",
            "test fixture",
            "ict",
            "fct",
            "functional test",
        ],
    )
}

fn readme_mentions_firmware_revision(text: &str) -> bool {
    has_any(
        text,
        &[
            "firmware revision",
            "firmware version",
            "fw revision",
            "fw version",
            "firmware image",
            "image version",
            "hex file",
            "bin file",
            "checksum",
            "sha256",
            "crc",
        ],
    )
}

fn readme_mentions_programming_method(text: &str) -> bool {
    has_any(
        text,
        &[
            "swd",
            "jtag",
            "bootloader",
            "tag-connect",
            "tag connect",
            "pogo",
            "programming fixture",
            "test fixture",
            "ict",
            "uart boot",
            "usb dfu",
            "fixture method",
        ],
    )
}

fn readme_mentions_test_acceptance(text: &str) -> bool {
    has_any(
        text,
        &[
            "acceptance",
            "pass/fail",
            "pass fail",
            "test record",
            "test report",
            "functional test",
            "fct",
            "ict",
            "production test",
            "calibration record",
        ],
    )
}

fn readme_mentions_assembly_variants(text: &str) -> bool {
    has_any(
        text,
        &[
            "assembly variant",
            "build variant",
            "bom variant",
            "variant handling",
            "variant:",
            "variants:",
            "prototype variant",
            "production variant",
            "do-not-populate variant",
        ],
    )
}

fn readme_mentions_fabrication_marking(text: &str) -> bool {
    has_any(
        text,
        &[
            "ul mark",
            "ul logo",
            "fab mark",
            "fabrication marking",
            "board label",
            "serial",
            "serialization",
            "barcode",
            "qr code",
            "revision text",
        ],
    )
}

fn readme_mentions_serialization(text: &str) -> bool {
    has_any(
        text,
        &[
            "serial",
            "serialization",
            "serialized",
            "barcode",
            "qr code",
            "uid label",
            "unit id",
        ],
    )
}

fn readme_mentions_serialization_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "serial format",
            "serial range",
            "barcode format",
            "label location",
        ],
    ) && has_any(
        text,
        &[
            "traceability",
            "traveler",
            "lot record",
            "unit record",
            "label file",
            "label drawing",
        ],
    )
}

fn readme_mentions_packaging(text: &str) -> bool {
    has_any(
        text,
        &[
            "packaging",
            "shipping",
            "ship in",
            "esd bag",
            "moisture barrier",
            "tray",
            "tape and reel",
            "reel",
            "vacuum pack",
        ],
    )
}

fn readme_mentions_packaging_handoff(text: &str) -> bool {
    has_any(
        text,
        &[
            "esd",
            "moisture",
            "msl",
            "humidity card",
            "desiccant",
            "tray",
            "tape and reel",
            "lot label",
            "box label",
            "labeling",
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
    row.iter().any(|cell| is_not_populated_text(cell))
}

fn is_not_populated_text(value: &str) -> bool {
    let normalized = value.trim().to_ascii_lowercase();
    matches!(
        normalized.as_str(),
        "dnp"
            | "dni"
            | "dnf"
            | "do not populate"
            | "do not install"
            | "do not fit"
            | "not populated"
            | "not fitted"
            | "not installed"
            | "no fit"
            | "nofit"
            | "unfitted"
            | "unplaced"
            | "exclude"
            | "excluded"
    ) || has_any(
        &normalized,
        &[
            "do not populate",
            "do not install",
            "do not fit",
            "not fitted",
            "not populated",
            "no stuff",
        ],
    )
}

fn is_not_populated_population_cell(value: &str) -> bool {
    is_not_populated_text(value)
        || matches!(
            value.trim().to_ascii_lowercase().as_str(),
            "false" | "no" | "n" | "0" | "off"
        )
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

fn likely_through_hole_bom_row(
    references: &[String],
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    references.iter().any(|reference| {
        split_reference_designator(reference).is_some_and(|(prefix, _)| {
            matches!(prefix, "J" | "P" | "CN" | "CON" | "K" | "SW" | "T")
        })
    }) || row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "through-hole",
            "through hole",
            "tht",
            "pht",
            "pin header",
            "header",
            "terminal block",
            "barrel jack",
            "switch",
            "relay",
            "connector",
            "press-fit",
            "press fit",
        ],
    )
}

fn likely_inspection_sensitive_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "bga",
            "lga",
            "wlcsp",
            "csp",
            "ucsp",
            "fbga",
            "ubga",
            "flip chip",
            "flip-chip",
        ],
    )
}

fn likely_programmable_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "mcu",
            "microcontroller",
            "processor",
            "fpga",
            "cpld",
            "soc",
            "flash",
            "eeprom",
            "bootloader",
            "firmware",
            "esp32",
            "stm32",
            "nrf52",
            "nrf53",
            "module",
            "wifi",
            "bluetooth",
            "radio module",
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

fn likely_traceability_sensitive_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
    lifecycle_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col, lifecycle_col],
        &[
            "bga",
            "csp",
            "lga",
            "qfn",
            "dfn",
            "mcu",
            "processor",
            "fpga",
            "flash",
            "memory",
            "radio",
            "wireless",
            "crystal",
            "oscillator",
            "tvs",
            "esd",
            "connector",
            "battery",
            "engineering sample",
            "sample",
            "allocation",
            "last time buy",
            "ltb",
        ],
    )
}

fn likely_regulatory_sensitive_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "connector",
            "cable",
            "battery",
            "fuse",
            "relay",
            "display",
            "led",
            "module",
            "wireless",
            "radio",
            "antenna",
            "power supply",
            "adapter",
        ],
    )
}

fn likely_source_control_sensitive_bom_row(
    row: &[String],
    part_col: Option<usize>,
    value_col: Option<usize>,
    package_col: Option<usize>,
) -> bool {
    row_text_matches(
        row,
        &[part_col, value_col, package_col],
        &[
            "mcu",
            "processor",
            "fpga",
            "asic",
            "module",
            "wireless",
            "radio",
            "connector",
            "battery",
            "crystal",
            "oscillator",
            "regulator",
            "pmic",
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
            "discontinued",
            "deprecated",
            "inactive",
            "eol",
            "end of life",
            "not recommended",
            "not for new design",
            "nrnd",
            "last time buy",
            "ltb",
            "preliminary",
            "preview",
            "sample",
            "engineering sample",
            "prototype only",
            "allocation",
            "shortage",
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
            "# units mm; origin aux origin; rotation convention clockwise degrees\nDesignator,X,Y,Rotation,Side\nR1,1.0,2.0,90,Top\nC1,3.0,4.0,0,Bottom\n",
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
             Assembly: double-sided assembly, pin-1 and polarity reviewed. Test fixture and programming handoff complete.\n\
             First article approval required before production. Acceptance criteria: pass/fail functional test and AQL inspection.\n\
             Lot traceability: supplier lot, date code, COC, and traveler records retained.\n",
        );
        let assembly = file("widget_assembly.pdf", 256);

        let violations = production_artifact_readiness(
            &[bom],
            &[centroid],
            &[],
            &[readme],
            &[],
            &[assembly],
            &[],
        );
        assert!(
            violations.is_empty(),
            "unexpected clean-package artifact warnings: {:?}",
            messages(&violations)
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
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier SKU,Lifecycle,Approved Alternate\nR1,1,RC0603,10k,0603,Yageo,SKU1,Active,ALT1\nR2,1,RC0603,1k,0402,,,NRND,\nR3,1,RC0603,10k,0603,Vishay,SKU2,Active,ALT2\nC1,1,CC0603,100nF,0603,Murata,SKU2,Active,ALT3\n",
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
        assert!(messages.iter().any(
            |message| message.contains("multiple manufacturers") && message.contains("RC0603")
        ));
        assert!(messages.iter().any(
            |message| message.contains("multiple supplier/distributor/SKU")
                && message.contains("RC0603")
        ));
        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple lifecycle/status")
                    && message.contains("RC0603"))
        );
        assert!(messages.iter().any(|message| message.contains("SKU2")
            && message.contains("multiple parts")
            && message.contains("CC0603")
            && message.contains("RC0603")));
    }

    #[test]
    fn bom_readiness_reports_primary_part_reused_as_approved_alternate() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,RC0603\nC1,1,CC0603,100nF,0603,Murata,SKU-C,Active,CC0603-ALT\n",
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
            messages
                .iter()
                .any(|message| message.contains("approved alternate/substitute")
                    && message.contains("same as the primary part"))
        );
    }

    #[test]
    fn bom_readiness_flags_broader_lifecycle_risks() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nU1,1,MCU123,MCU,QFN32,Vendor,SKU-U,not for new design,ALT-U\nU2,1,ASIC123,ASIC,BGA100,Vendor,SKU-A,engineering sample,ALT-A\n",
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

        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("requires procurement review"))
                .count(),
            2
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
    fn bom_readiness_treats_placeholder_cells_as_missing_release_metadata() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Polarity,MSL,Height\nD1,1,TBD,N/A,?,unknown,pending,select,none,TBD,N/A,TBD\nU1,1,MCU123,MCU,QFN32,Vendor,SKU-U,Active,ALT-U,Pin 1 reviewed,N/A,0.9\nJ1,1,USB-C,USB connector,USB-C,Vendor,SKU-J,Active,ALT-J,Pin 1 reviewed,1,TBD\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        for expected in [
            "no populated part identifier",
            "no populated manufacturer",
            "no populated supplier/distributor/SKU",
            "no populated lifecycle/status",
            "no approved alternate/substitute",
            "no populated value/description",
            "no populated footprint/package",
            "no populated polarity/orientation note",
            "no populated moisture/MSL handling note",
            "no valid populated height",
        ] {
            assert!(
                messages.iter().any(|message| message.contains(expected)),
                "missing warning containing {expected:?}"
            );
        }
    }

    #[test]
    fn placeholder_procurement_cells_do_not_create_conflict_noise() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R\nR2,1,RC0603,TBD,unknown,unknown,pending,pending,TBD\n",
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
                .any(|message| message.contains("multiple values")
                    || message.contains("multiple footprints")
                    || message.contains("multiple manufacturers")
                    || message.contains("multiple supplier/distributor/SKU")
                    || message.contains("multiple lifecycle/status")),
            "placeholder cells should only produce missing-field warnings, got {messages:?}"
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no populated value/description"))
        );
    }

    #[test]
    fn artifact_tables_accept_semicolon_and_whitespace_delimited_sidecars() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference;Quantity;MPN;Value;Footprint;Manufacturer;Supplier;Lifecycle;Approved Alternate\nR1;1;RC0603;10k;0603;Yageo;SKU-R;Active;ALT-R\n",
        );
        let centroid = artifact(
            "widget_pick_place.pos",
            "Ref X Y Rotation Side Value Footprint\nR1 10.0 20.0 90 Top 10k 0603\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[centroid],
            &[],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("header has no")),
            "unexpected header parse warning: {messages:?}"
        );
        assert!(!messages.iter().any(|message| message.contains("invalid x")
            || message.contains("invalid y")
            || message.contains("invalid rotation")));
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("artifact extension")
                    || message.contains("artifact filename"))
        );
    }

    #[test]
    fn artifact_release_markers_report_revision_and_date_mismatches() {
        let bom = artifact(
            "widget_revA_20260501_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R\n",
        );
        let centroid = artifact(
            "widget_revB_20260502_centroid.csv",
            "Ref,X,Y,Rotation,Side\nR1,10.0,20.0,90,Top\n",
        );
        let readme = artifact(
            "README.md",
            "Revision A release package generated 2026-05-01. Fabrication stackup 2 layer, \
             thickness 1.6mm, copper weight 1 oz, ENIG finish, soldermask green, no impedance, \
             no panelization, tented vias, no edge plating, no castellation. DRC/ERC passed, \
             zones refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, \
             no waivers, archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[readme], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("multiple revision markers")
                    && message.contains("revA")
                    && message.contains("revB"))
        );
        assert!(messages.iter().any(|message| {
            message.contains("multiple generated/release dates")
                && message.contains("2026-05-01")
                && message.contains("2026-05-02")
        }));
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
            "# units mm; origin aux origin; rotation convention clockwise degrees\nRef,X,Y,Rotation,Side\nR1,0,0,0,Top\nR2,1,0,0,Top\nR3,2,0,0,Top\nR4,3,0,0,Top\nR5,4,0,0,Top\n",
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
    fn bom_population_columns_and_zero_quantity_rows_are_checked() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Fitted\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Yes\nC1,0,CC0603,100nF,0603,Murata,SKU-C,Active,ALT-C,Yes\nD1,2,LED0603,LED,0603 LED,LiteOn,SKU-D,Active,ALT-D,No\n,1,MCU,QFN,QFN32,Vendor,SKU-U,Active,ALT-U,Yes\n",
        );

        let violations = production_artifact_readiness(&[bom], &[], &[], &[], &[], &[], &[]);
        let messages = messages(&violations);

        assert!(
            messages
                .iter()
                .any(|message| message.contains("zero quantity")
                    && message.contains("not marked DNP/DNI"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("marked DNP/DNI")
                    && message.contains("nonzero quantity 2"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no reference designator"))
        );
    }

    #[test]
    fn bom_readiness_validates_optional_unit_cost_cells_when_present() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Unit Cost\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,$0.01\nC1,1,CC0603,100nF,0603,Murata,SKU-C,Active,ALT-C,TBD\nU1,1,MCU,QFN,QFN32,Vendor,SKU-U,Active,ALT-U,not-a-price\nD1,0,DNP,,,,,,,\n",
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
            messages
                .iter()
                .any(|message| message.contains("no populated unit cost"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("invalid unit cost/price"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("BOM row 5") && message.contains("unit cost")),
            "DNP rows should not require pricing: {messages:?}"
        );
    }

    #[test]
    fn bom_readiness_reports_traceability_compliance_and_source_control_gaps() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Traceability,Compliance,Source Control\nU1,1,MCU123,MCU,QFN32,Vendor,SKU-U,Active,ALT-U,,,\nJ1,1,CONN123,USB connector,USB-C,Vendor,SKU-J,Active,ALT-J,,,\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,,,\n",
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
            messages
                .iter()
                .any(|message| message.contains("traceability note"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("compliance evidence"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("source-control evidence"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("BOM row 4 likely needs")),
            "passive row should not trigger advanced procurement handoff warnings: {messages:?}"
        );
    }

    #[test]
    fn bom_variant_columns_require_readme_variant_handoff_when_multiple_variants_exist() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Build Variant\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Prototype\nR2,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Production\n",
        );
        let readme = artifact(
            "README.md",
            "Revision R. Fabrication package. Stackup 2 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, \
             HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages.iter().any(|message| {
                message.contains("multiple assembly/build variants")
                    && message.contains("PROTOTYPE")
                    && message.contains("PRODUCTION")
            }),
            "missing variant warning in {messages:?}"
        );
    }

    #[test]
    fn bom_variant_columns_accept_explicit_readme_variant_handoff() {
        let bom = artifact(
            "widget_bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Build Variant\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Prototype\nR2,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Production\n",
        );
        let readme = artifact(
            "README.md",
            "Revision S. Fabrication package. Stackup 2 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. Assembly variant handling: build Prototype or Production as selected by PO. \
             DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, \
             no waivers, archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
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
                .any(|message| message.contains("multiple assembly/build variants")),
            "explicit variant handoff should suppress warning: {messages:?}"
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
    fn centroid_readiness_reports_unusually_large_coordinates() {
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side\nU1,10000,25,0,Top\nU2,25,-6000,90,Bottom\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[],
            &[centroid],
            &[],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(messages.iter().any(|message| {
            message.contains("x coordinate")
                && message.contains("unusually large")
                && message.contains("units")
        }));
        assert!(messages.iter().any(|message| {
            message.contains("y coordinate")
                && message.contains("unusually large")
                && message.contains("origin")
        }));
    }

    #[test]
    fn centroid_readiness_requires_units_origin_and_rotation_convention() {
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\nU1,10,20,90,Top\n");

        let messages = messages(&production_artifact_readiness(
            &[],
            &[centroid],
            &[],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages
                .iter()
                .any(|message| message.contains("does not state placement units"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("does not state placement origin"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("does not state rotation convention"))
        );
    }

    #[test]
    fn centroid_readiness_treats_placeholder_reference_and_metadata_as_missing() {
        let centroid = artifact(
            "centroid.csv",
            "Ref,X,Y,Rotation,Side,Value,Footprint\nTBD,0,0,0,Top,TBD,unknown\nR1,1,1,0,Top,10k,0603\nR1,2,2,90,Top,N/A,?\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[],
            &[centroid],
            &[],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages
                .iter()
                .any(|message| message.contains("centroid row 2 has no reference"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("multiple values")
                    || message.contains("multiple footprints")),
            "placeholder centroid metadata should not create conflict warnings: {messages:?}"
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
    fn bom_bottom_side_data_triggers_double_sided_handoff_review() {
        let bom = artifact(
            "bom.csv",
            "Ref,Qty,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate,Side\nR1,1,RC0603,10k,0603,Yageo,SKU-R,Active,ALT-R,Bottom\n",
        );
        let readme = artifact(
            "README.md",
            "Revision N. Fabrication package. Stackup 2 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, \
             HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(messages.iter().any(|message| {
            message.contains("bottom-side placements")
                && message.contains("README")
                && message.contains("double-sided")
        }));
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
    fn netlist_readiness_treats_placeholder_cells_as_missing() {
        let netlist = artifact(
            "netlist.csv",
            "Net,Reference,Pin\nTBD,U1,1\nGND,?,2\n3V3,U2,N/A\nSIG,U3,1\nSIG,U4,1\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[],
            &[],
            &[netlist],
            &[],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages
                .iter()
                .any(|message| message.contains("row 2 has no net name"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("row 3 has no reference"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("row 4 has no pin/pad"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("multiple nets")),
            "placeholder netlist cells should not create pin conflict noise: {messages:?}"
        );
    }

    #[test]
    fn sidecar_tables_with_only_headers_are_reported_as_empty() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\n",
        );
        let centroid = artifact("centroid.csv", "Ref,X,Y,Rotation,Side\n");
        let netlist = artifact("netlist.csv", "Net,Ref,Pin\n");

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[centroid],
            &[netlist],
            &[],
            &[],
            &[],
            &[],
        ));

        for expected in [
            "BOM file has no component rows",
            "centroid file has no placement rows",
            "netlist file has no pin rows",
        ] {
            assert!(
                messages.iter().any(|message| message.contains(expected)),
                "missing empty-sidecar warning containing {expected:?}"
            );
        }
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
             Assembly: pin-1 and polarity reviewed. Test fixture and programming handoff complete.\n\
             First article approval required before production. Acceptance criteria: pass/fail functional test and AQL inspection.\n\
             Lot traceability: supplier lot, date code, COC, and traveler records retained.\n",
        );

        let violations = production_artifact_readiness(&[], &[], &[], &[readme], &[], &[], &[]);
        assert!(
            violations.is_empty(),
            "unexpected README warnings: {:?}",
            messages(&violations)
        );
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
    fn readme_readiness_reports_contradictory_assembly_and_test_intent() {
        let readme = artifact(
            "README.md",
            "Revision AA. Fabrication package. Stackup 2 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green. No impedance. No panelization. Tented vias. No edge plating, \
             no castellation. No conformal coating, but conformal coating required. No programming, but \
             firmware required and flash firmware at test. No test fixture, but ICT required. DRC/ERC passed, \
             zones refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive \
             created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
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

        for label in ["conformal coating", "programming", "test fixture"] {
            assert!(
                messages
                    .iter()
                    .any(|message| message.contains("contradictory") && message.contains(label)),
                "missing contradiction for {label}: {messages:?}"
            );
        }
    }

    #[test]
    fn readme_readiness_reports_conflicting_order_parameters() {
        let readme = artifact(
            "README.md",
            "Revision F. Fabrication package. Stackup 2 layer and 4 layer, 0.8mm and 1.6mm board thickness, \
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
            "layer count",
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
    fn readme_readiness_flags_surface_finish_compatibility_risks() {
        let readme = artifact(
            "README.md",
            "Revision SF. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             HASL finish, soldermask green. No impedance. Panelization none. Tented vias. \
             No edge plating, no castellation. Assembly includes BGA and QFN packages plus card edge \
             gold fingers, press-fit pins, and wire bond pads. DRC/ERC passed, zones refilled, outputs \
             generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. Pin-1 and \
             polarity reviewed. Test fixture/programming handoff complete.\n",
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

        for expected in [
            "edge contacts",
            "fine-pitch, BGA",
            "press-fit hardware",
            "wire bonding",
        ] {
            assert!(
                messages.iter().any(|message| message.contains(expected)),
                "missing surface-finish warning containing {expected}: {messages:?}"
            );
        }
    }

    #[test]
    fn readme_readiness_accepts_explicit_surface_finish_handoffs() {
        let readme = artifact(
            "README.md",
            "Revision SG. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENEPIG finish with hard gold contact fingers, soldermask green. No impedance. Panelization none. \
             Tented vias. No edge plating, no castellation. Assembly includes BGA and QFN packages, press-fit pins, \
             and wire bond pads. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, \
             HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed. \
             Test fixture/programming handoff complete.\n",
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
                .any(|message| message.contains("finish compatibility")
                    || message.contains("finish with"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("edge contacts")
                    || message.contains("fine-pitch")
                    || message.contains("press-fit hardware")
                    || message.contains("wire bonding"))
        );
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
    fn bom_driven_assembly_risks_require_readme_handoff_notes() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nJ1,1,CONN-TH,Through-hole connector,Pin Header,Vendor,SKU-J,Active,ALT-J\nU1,1,MCU-BGA,MCU with firmware,BGA100,Vendor,SKU-U,Active,ALT-U\n",
        );
        let readme = artifact(
            "README.md",
            "Revision P. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, \
             HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages
                .iter()
                .any(|message| message.contains("through-hole")
                    && message.contains("J1")
                    && message.contains("README"))
        );
        assert!(messages.iter().any(|message| {
            message.contains("inspection-sensitive")
                && message.contains("U1")
                && message.contains("X-ray")
        }));
        assert!(messages.iter().any(|message| {
            message.contains("programmable")
                && message.contains("U1")
                && message.contains("firmware")
        }));
    }

    #[test]
    fn bom_driven_assembly_risks_accept_explicit_readme_handoff_notes() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nJ1,1,CONN-TH,Through-hole connector,Pin Header,Vendor,SKU-J,Active,ALT-J\nU1,1,MCU-BGA,MCU with firmware,BGA100,Vendor,SKU-U,Active,ALT-U\n",
        );
        let readme = artifact(
            "README.md",
            "Revision Q. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. Assembly includes selective solder process notes for pin headers. \
             BGA X-ray and AOI inspection handoff complete. Firmware programming via SWD test fixture uses \
             firmware revision 1.2.3 with SHA256 checksum and functional test acceptance records. \
             DRC/ERC passed, zones refilled, outputs generated, Gerber viewer checked, \
             HyperDRC reviewed, no waivers, archive created. Pin-1 and polarity reviewed.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            !messages.iter().any(
                |message| message.contains("bom-readme-assembly-process-parity")
                    || message.contains("BOM includes likely through-hole")
                    || message.contains("inspection-sensitive")
                    || message.contains("BOM includes likely programmable")
            ),
            "explicit process notes should suppress BOM-driven handoff warnings: {messages:?}"
        );
    }

    #[test]
    fn bom_driven_programming_requires_traceability_method_and_acceptance_notes() {
        let bom = artifact(
            "bom.csv",
            "Reference,Quantity,MPN,Value,Footprint,Manufacturer,Supplier,Lifecycle,Approved Alternate\nU1,1,STM32F4,MCU firmware,QFP64,Vendor,SKU-U,Active,ALT-U\n",
        );
        let readme = artifact(
            "README.md",
            "Revision PRG. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. Firmware programming handoff complete. DRC/ERC passed, zones refilled, \
             outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. \
             Pin-1 and polarity reviewed.\n",
        );

        let messages = messages(&production_artifact_readiness(
            &[bom],
            &[],
            &[],
            &[readme],
            &[],
            &[],
            &[],
        ));

        assert!(
            messages
                .iter()
                .any(|message| message.contains("firmware revision"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("SWD/JTAG/bootloader"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("functional-test acceptance"))
        );
    }

    #[test]
    fn readme_readiness_checks_marking_serialization_and_packaging_handoffs() {
        let readme = artifact(
            "README.md",
            "Revision SER. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. Add date code, barcode serialization, and production packaging. DRC/ERC passed, \
             zones refilled, outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, \
             archive created. Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
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
            messages
                .iter()
                .any(|message| message.contains("allowed marking zones"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("serial format"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("ESD") && message.contains("packaging"))
        );
    }

    #[test]
    fn readme_readiness_accepts_marking_serialization_and_packaging_handoffs() {
        let readme = artifact(
            "README.md",
            "Revision SER2. Fabrication package. Stackup 4 layer, thickness 1.6mm, copper weight 1 oz, \
             ENIG finish, soldermask green, no impedance, no panelization, tented vias, no edge plating, \
             no castellation. Date code and barcode serialization use serial format WID-####, serial range \
             WID-0001..WID-0100, label location in fab drawing, and lot record traceability handoff. \
             Packaging uses ESD bag, moisture barrier, desiccant, and lot label. DRC/ERC passed, zones refilled, \
             outputs generated, Gerber viewer checked, HyperDRC reviewed, no waivers, archive created. \
             Pin-1 and polarity reviewed. Test fixture handoff complete.\n",
        );
        let fab = file("widget_fab.pdf", 256);

        let messages = messages(&production_artifact_readiness(
            &[],
            &[],
            &[],
            &[readme],
            &[fab],
            &[],
            &[],
        ));

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("allowed marking zones")
                    || message.contains("serial format")
                    || message.contains("packaging or shipping"))
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
