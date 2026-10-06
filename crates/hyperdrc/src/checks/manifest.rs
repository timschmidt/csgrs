//! Package-level readiness checks over the discovered input manifest.
//!
//! Geometry checks can prove local design-rule problems, but pre-production
//! review also needs the uploaded file set to be coherent. This module catches
//! missing or duplicated manufacturing deliverables using conservative filename
//! role inference and Gerber X2 metadata. The goal is to catch file-set
//! mismatches before geometry and electrical checks begin.

use crate::date::{current_day_number, parse_compact_day, parse_iso_day};
use crate::report::{Severity, Violation};

const DEFAULT_GENERATED_DATE_STALE_DAYS: i64 = 90;

#[derive(Clone, Debug)]
/// Public data model for `ManifestGerberLayer`.
pub struct ManifestGerberLayer {
    /// Human-readable layer name from the geometry parser.
    pub name: String,
    /// Source filename or path for manifest diagnostics.
    pub source_path: String,
    /// Optional Gerber X2 `.Part` attribute, for example `Single`, `Array`,
    /// `FabricationPanel`, `Coupon`, or `Other,<field>`.
    pub part: Option<String>,
    /// Optional Gerber X2 `.FileFunction` attribute, for example
    /// `Copper,L1,Top`.
    pub file_function: Option<String>,
    /// Optional Gerber X2 `.FilePolarity` attribute, for example `Positive` or
    /// `Negative`.
    pub file_polarity: Option<String>,
    /// Optional Gerber X2 `.SameCoordinates` identifier. An empty string means
    /// the attribute was present without an identifier.
    pub same_coordinates: Option<String>,
    /// Optional Gerber X2 `.CreationDate` value.
    pub creation_date: Option<String>,
    /// Optional Gerber X2 `.GenerationSoftware` value.
    pub generation_software: Option<String>,
    /// Optional Gerber X2 `.ProjectId` value.
    pub project_id: Option<String>,
    /// Optional Gerber X2 `.MD5` file signature/checksum value.
    pub md5: Option<String>,
    /// Optional Gerber `%MO...*%` image units, normalized for report output.
    pub units: Option<String>,
    /// Optional Gerber `%FS...*%` coordinate format, normalized for report output.
    pub coordinate_format: Option<String>,
}

#[derive(Clone, Debug, Default)]
/// Public data model for `ManifestInput`.
pub struct ManifestInput {
    /// Field `gerber_layers`.
    pub gerber_layers: Vec<ManifestGerberLayer>,
    /// Field `artifact_paths`.
    pub artifact_paths: Vec<String>,
    /// Field `bom_file_count`.
    pub bom_file_count: usize,
    /// Field `centroid_file_count`.
    pub centroid_file_count: usize,
    /// Field `netlist_file_count`.
    pub netlist_file_count: usize,
    /// Field `fab_drawing_file_count`.
    pub fab_drawing_file_count: usize,
    /// Field `assembly_drawing_file_count`.
    pub assembly_drawing_file_count: usize,
    /// Field `readme_file_count`.
    pub readme_file_count: usize,
    /// Field `rout_drawing_file_count`.
    pub rout_drawing_file_count: usize,
    /// Field `required_artifacts`.
    pub required_artifacts: ManifestRequirements,
    /// Field `required_layers`.
    pub required_layers: ManifestLayerRequirements,
    /// Field `declared_copper_layer_count`.
    pub declared_copper_layer_count: Option<usize>,
    /// Field `generated_date_stale_days`.
    pub generated_date_stale_days: Option<usize>,
    /// Field `kicad_copper_layer_count`.
    pub kicad_copper_layer_count: Option<usize>,
    /// Field `has_board_outline`.
    pub has_board_outline: bool,
    /// Field `has_drill_data`.
    pub has_drill_data: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
/// Public data model for `ManifestRequirements`.
pub struct ManifestRequirements {
    /// Field `bom`.
    pub bom: bool,
    /// Field `centroid`.
    pub centroid: bool,
    /// Field `netlist`.
    pub netlist: bool,
    /// Field `fab_drawing`.
    pub fab_drawing: bool,
    /// Field `assembly_drawing`.
    pub assembly_drawing: bool,
    /// Field `readme`.
    pub readme: bool,
    /// Field `rout_drawing`.
    pub rout_drawing: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
/// Public data model for `ManifestLayerRequirements`.
pub struct ManifestLayerRequirements {
    /// Field `board_outline`.
    pub board_outline: bool,
    /// Field `drill_data`.
    pub drill_data: bool,
    /// Field `top_mask`.
    pub top_mask: bool,
    /// Field `bottom_mask`.
    pub bottom_mask: bool,
    /// Field `top_paste`.
    pub top_paste: bool,
    /// Field `bottom_paste`.
    pub bottom_paste: bool,
    /// Field `top_silkscreen`.
    pub top_silkscreen: bool,
    /// Field `bottom_silkscreen`.
    pub bottom_silkscreen: bool,
}

impl Default for ManifestLayerRequirements {
    fn default() -> Self {
        Self {
            board_outline: true,
            drill_data: true,
            top_mask: true,
            bottom_mask: true,
            top_paste: true,
            bottom_paste: true,
            top_silkscreen: true,
            bottom_silkscreen: true,
        }
    }
}

impl Default for ManifestRequirements {
    fn default() -> Self {
        Self {
            bom: true,
            centroid: true,
            netlist: true,
            fab_drawing: true,
            assembly_drawing: true,
            readme: true,
            rout_drawing: true,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum GerberRole {
    TopCopper,
    BottomCopper,
    InnerCopper,
    TopMask,
    BottomMask,
    TopPaste,
    BottomPaste,
    TopSilk,
    BottomSilk,
    Outline,
    Other,
}

/// Run the `file_manifest_readiness` design-readiness check or report helper.
pub fn file_manifest_readiness(input: &ManifestInput) -> Vec<Violation> {
    let classified = input
        .gerber_layers
        .iter()
        .map(|layer| (layer, classify_gerber_role(layer)))
        .collect::<Vec<_>>();

    let top_copper = role_count(&classified, GerberRole::TopCopper);
    let bottom_copper = role_count(&classified, GerberRole::BottomCopper);
    let inner_copper = role_count(&classified, GerberRole::InnerCopper);
    let copper_count = top_copper + bottom_copper + inner_copper;
    let outline_count = role_count(&classified, GerberRole::Outline);
    let top_mask_count = role_count(&classified, GerberRole::TopMask);
    let bottom_mask_count = role_count(&classified, GerberRole::BottomMask);
    let top_paste_count = role_count(&classified, GerberRole::TopPaste);
    let bottom_paste_count = role_count(&classified, GerberRole::BottomPaste);
    let top_silk_count = role_count(&classified, GerberRole::TopSilk);
    let bottom_silk_count = role_count(&classified, GerberRole::BottomSilk);
    let x2_file_function_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.file_function.is_some())
        .count();
    let x2_part_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.part.is_some())
        .count();
    let x2_negative_polarity_count = input
        .gerber_layers
        .iter()
        .filter(|layer| {
            layer
                .file_polarity
                .as_deref()
                .is_some_and(is_negative_file_polarity)
        })
        .count();
    let x2_same_coordinates_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.same_coordinates.is_some())
        .count();
    let x2_creation_date_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.creation_date.is_some())
        .count();
    let x2_generation_software_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.generation_software.is_some())
        .count();
    let x2_project_id_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.project_id.is_some())
        .count();
    let x2_md5_count = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.md5.is_some())
        .count();
    let mut violations = Vec::new();

    log::trace!(
        "file-manifest-readiness roles: gerber_layers={} x2_parts={} x2_file_functions={} x2_negative_file_polarities={} x2_same_coordinates={} x2_creation_dates={} x2_generation_software={} x2_project_ids={} x2_md5={} copper={} top_copper={} bottom_copper={} inner_copper={} outline={} mask={}/{} paste={}/{} silk={}/{}",
        input.gerber_layers.len(),
        x2_part_count,
        x2_file_function_count,
        x2_negative_polarity_count,
        x2_same_coordinates_count,
        x2_creation_date_count,
        x2_generation_software_count,
        x2_project_id_count,
        x2_md5_count,
        copper_count,
        top_copper,
        bottom_copper,
        inner_copper,
        outline_count,
        top_mask_count,
        bottom_mask_count,
        top_paste_count,
        bottom_paste_count,
        top_silk_count,
        bottom_silk_count
    );

    if copper_count == 0 {
        violations.push(package_violation(
            "missing-copper",
            Severity::Error,
            "Gerber package does not contain any recognizable copper layer",
        ));
    }

    if input.required_layers.board_outline && outline_count == 0 && !input.has_board_outline {
        violations.push(package_violation(
            "missing-board-outline",
            Severity::Warning,
            "Gerber package has no recognizable board outline/profile layer",
        ));
    }

    if input.required_layers.drill_data && !input.has_drill_data {
        violations.push(package_violation(
            "missing-drill-data",
            Severity::Warning,
            "input package has no Excellon or KiCad drill data",
        ));
    }

    if input.required_layers.top_mask && top_copper > 0 && top_mask_count == 0 {
        violations.push(package_violation(
            "missing-top-mask",
            Severity::Warning,
            "top copper is present but no top solder mask opening layer was recognized",
        ));
    }

    if input.required_layers.bottom_mask && bottom_copper > 0 && bottom_mask_count == 0 {
        violations.push(package_violation(
            "missing-bottom-mask",
            Severity::Warning,
            "bottom copper is present but no bottom solder mask opening layer was recognized",
        ));
    }

    if input.required_layers.top_silkscreen && top_copper > 0 && top_silk_count == 0 {
        violations.push(package_violation(
            "missing-top-silkscreen",
            Severity::Warning,
            "top copper is present but no top silkscreen layer was recognized",
        ));
    }

    if input.required_layers.bottom_silkscreen && bottom_copper > 0 && bottom_silk_count == 0 {
        violations.push(package_violation(
            "missing-bottom-silkscreen",
            Severity::Warning,
            "bottom copper is present but no bottom silkscreen layer was recognized",
        ));
    }

    if input.required_layers.top_paste && top_copper > 0 && top_paste_count == 0 {
        violations.push(package_violation(
            "missing-top-paste",
            Severity::Warning,
            "top copper is present but no top paste layer was recognized",
        ));
    }

    if input.required_layers.bottom_paste && bottom_copper > 0 && bottom_paste_count == 0 {
        violations.push(package_violation(
            "missing-bottom-paste",
            Severity::Warning,
            "bottom copper is present but no bottom paste layer was recognized",
        ));
    }

    duplicate_role_warning(
        &classified,
        GerberRole::TopCopper,
        "duplicate-top-copper",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::BottomCopper,
        "duplicate-bottom-copper",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::TopMask,
        "duplicate-top-mask",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::BottomMask,
        "duplicate-bottom-mask",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::TopPaste,
        "duplicate-top-paste",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::BottomPaste,
        "duplicate-bottom-paste",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::TopSilk,
        "duplicate-top-silkscreen",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::BottomSilk,
        "duplicate-bottom-silkscreen",
        &mut violations,
    );
    duplicate_role_warning(
        &classified,
        GerberRole::Outline,
        "duplicate-outline",
        &mut violations,
    );
    check_layer_name_side_conflicts(&classified, &mut violations);
    check_negative_copper_file_polarity(&classified, &mut violations);
    check_file_polarity_evidence(input, &classified, &mut violations);
    check_file_function_evidence(input, &mut violations);
    check_part_evidence(input, &mut violations);
    check_same_coordinates_evidence(input, &mut violations);
    check_generation_software_evidence(input, &mut violations);
    check_project_id_evidence(input, &mut violations);
    check_md5_evidence(input, &mut violations);
    check_image_setup_consistency(input, &mut violations);

    check_layer_role_coherence(
        top_copper,
        bottom_copper,
        inner_copper,
        top_mask_count,
        bottom_mask_count,
        top_paste_count,
        bottom_paste_count,
        top_silk_count,
        bottom_silk_count,
        &mut violations,
    );

    verify_kicad_copper_layer_count(
        input.kicad_copper_layer_count,
        copper_count,
        &mut violations,
    );

    check_single_file(
        input.bom_file_count,
        input.required_artifacts.bom,
        "bom",
        "bill of materials",
        &mut violations,
    );

    check_single_file(
        input.centroid_file_count,
        input.required_artifacts.centroid,
        "centroid",
        "centroid",
        &mut violations,
    );

    check_single_file(
        input.netlist_file_count,
        input.required_artifacts.netlist,
        "netlist",
        "netlist",
        &mut violations,
    );

    check_single_file(
        input.fab_drawing_file_count,
        input.required_artifacts.fab_drawing,
        "fab-drawing",
        "fabrication drawing",
        &mut violations,
    );

    check_single_file(
        input.assembly_drawing_file_count,
        input.required_artifacts.assembly_drawing,
        "assembly-drawing",
        "assembly drawing",
        &mut violations,
    );

    check_single_file(
        input.readme_file_count,
        input.required_artifacts.readme,
        "readme",
        "readme",
        &mut violations,
    );
    check_single_file(
        input.rout_drawing_file_count,
        input.required_artifacts.rout_drawing,
        "rout-drawing",
        "rout drawing",
        &mut violations,
    );
    verify_declared_copper_layer_count(
        input.declared_copper_layer_count,
        copper_count,
        &mut violations,
    );
    check_filename_layer_count_consistency(input, copper_count, &mut violations);
    check_revision_consistency(input, &mut violations);
    check_generated_date_consistency(input, &mut violations);
    check_x2_creation_date_consistency(input, &mut violations);
    check_generated_date_age(
        input,
        current_day_number(),
        generated_date_stale_days(input),
        &mut violations,
    );
    check_x2_creation_date_age(
        input,
        current_day_number(),
        generated_date_stale_days(input),
        &mut violations,
    );
    check_project_name_consistency(input, &mut violations);
    check_stale_artifact_names(input, &mut violations);

    violations
}

fn check_generated_date_age(
    input: &ManifestInput,
    today: Option<i64>,
    stale_days: i64,
    violations: &mut Vec<Violation>,
) {
    let Some(today) = today else {
        return;
    };

    let dated_paths = manifest_paths(input)
        .into_iter()
        .filter_map(|path| generated_date_tag(&path).map(|date| (path, date)))
        .collect::<Vec<_>>();
    let stale = dated_paths
        .iter()
        .filter(|(_, date)| parse_compact_day(date).is_some_and(|day| today - day > stale_days))
        .map(|(path, date)| format!("{path} ({date})"))
        .collect::<Vec<_>>();
    let future = dated_paths
        .iter()
        .filter(|(_, date)| parse_compact_day(date).is_some_and(|day| day > today))
        .map(|(path, date)| format!("{path} ({date})"))
        .collect::<Vec<_>>();

    if !stale.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:stale-generated-date".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "input package contains generated-date tags older than {stale_days} days: {}",
                stale.join(", ")
            )),
        ));
    }

    if !future.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:future-generated-date".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "input package contains generated-date tags later than the current run date: {}",
                future.join(", ")
            )),
        ));
    }
}

fn check_x2_creation_date_age(
    input: &ManifestInput,
    today: Option<i64>,
    stale_days: i64,
    violations: &mut Vec<Violation>,
) {
    let Some(today) = today else {
        return;
    };

    let dated_layers = x2_creation_date_tags(input);
    let stale = dated_layers
        .iter()
        .filter(|(_, date)| parse_compact_day(date).is_some_and(|day| today - day > stale_days))
        .map(|(path, date)| format!("{path} ({date})"))
        .collect::<Vec<_>>();
    let future = dated_layers
        .iter()
        .filter(|(_, date)| parse_compact_day(date).is_some_and(|day| day > today))
        .map(|(path, date)| format!("{path} ({date})"))
        .collect::<Vec<_>>();

    if !stale.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:stale-x2-creation-date".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Gerber X2 CreationDate values are older than {stale_days} days: {}",
                stale.join(", ")
            )),
        ));
    }

    if !future.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:future-x2-creation-date".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Gerber X2 CreationDate values are later than the current run date: {}",
                future.join(", ")
            )),
        ));
    }
}

fn generated_date_stale_days(input: &ManifestInput) -> i64 {
    input
        .generated_date_stale_days
        .and_then(|days| i64::try_from(days).ok())
        .filter(|days| *days > 0)
        .unwrap_or(DEFAULT_GENERATED_DATE_STALE_DAYS)
}

fn verify_kicad_copper_layer_count(
    expected: Option<usize>,
    observed: usize,
    violations: &mut Vec<Violation>,
) {
    let Some(expected) = expected else {
        return;
    };

    if expected != observed {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:copper-layer-parity".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Gerber package exposes {observed} recognized copper layer role(s), while KiCad data contains {expected} copper layer(s)"
            )),
        ));
    }
}

fn verify_declared_copper_layer_count(
    expected: Option<usize>,
    observed: usize,
    violations: &mut Vec<Violation>,
) {
    let Some(expected) = expected else {
        return;
    };

    if expected != observed {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:declared-copper-layer-parity".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Declared manifest copper layer count ({expected}) does not match observed stack ({observed})"
            )),
        ));
    }
}

fn check_filename_layer_count_consistency(
    input: &ManifestInput,
    observed_copper_count: usize,
    violations: &mut Vec<Violation>,
) {
    let filename_counts = filename_layer_count_tags(input);
    if filename_counts.is_empty() {
        return;
    }

    if filename_counts.len() > 1 {
        let summary = filename_counts
            .into_iter()
            .map(|(count, paths)| format!("{count}: {}", paths.join(", ")))
            .collect::<Vec<_>>()
            .join("; ");
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:filename-layer-count-conflict".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "input package appears to mix filename layer-count tags across files: {summary}"
            )),
        ));
        return;
    }

    let (filename_count, paths) = filename_counts
        .into_iter()
        .next()
        .expect("non-empty filename layer-count map has one entry");
    let path_summary = paths.join(", ");

    if observed_copper_count > 0 && filename_count != observed_copper_count {
        // IEEE Std 828-2012 treats controlled release files as configuration
        // items. Conservative filename-count parsing catches package baselines
        // where the Gerber set and the advertised layer-count convention cannot
        // both describe the submitted manufacturing stack.
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:filename-copper-layer-parity".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "filename layer-count tag ({filename_count}) does not match observed recognized copper layer role count ({observed_copper_count}): {path_summary}"
            )),
        ));
    }

    if let Some(declared_count) = input.declared_copper_layer_count
        && declared_count != filename_count
    {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:filename-declared-copper-layer-parity".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "filename layer-count tag ({filename_count}) does not match declared manifest copper layer count ({declared_count}): {path_summary}"
            )),
        ));
    }

    if let Some(kicad_count) = input.kicad_copper_layer_count
        && kicad_count != filename_count
    {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:filename-kicad-copper-layer-parity".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "filename layer-count tag ({filename_count}) does not match KiCad copper layer count ({kicad_count}): {path_summary}"
            )),
        ));
    }
}

fn filename_layer_count_tags(
    input: &ManifestInput,
) -> std::collections::BTreeMap<usize, Vec<String>> {
    let mut counts = std::collections::BTreeMap::<usize, Vec<String>>::new();
    for path in manifest_paths(input) {
        if let Some(count) = filename_layer_count_tag(&path) {
            counts.entry(count).or_default().push(path);
        }
    }
    counts
}

fn filename_layer_count_tag(path: &str) -> Option<usize> {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    let tokens = stem
        .split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .map(str::to_ascii_lowercase)
        .collect::<Vec<_>>();

    for token in &tokens {
        if let Some(number) = token
            .strip_suffix("layers")
            .or_else(|| token.strip_suffix("layer"))
            .or_else(|| token.strip_suffix("lyr"))
            .or_else(|| token.strip_suffix('l'))
            .and_then(parse_filename_layer_count_number)
        {
            return Some(number);
        }
    }

    for window in tokens.windows(2) {
        let [count, label] = window else {
            continue;
        };
        if matches!(label.as_str(), "layer" | "layers" | "lyr")
            && let Some(number) = parse_filename_layer_count_number(count)
        {
            return Some(number);
        }
    }

    None
}

fn parse_filename_layer_count_number(value: &str) -> Option<usize> {
    if value.is_empty() || !value.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }
    let count = value.parse::<usize>().ok()?;
    (1..=64).contains(&count).then_some(count)
}

fn check_single_file(
    count: usize,
    required: bool,
    slug: &str,
    name: &str,
    violations: &mut Vec<Violation>,
) {
    if count == 0 && required {
        violations.push(package_violation(
            &format!("missing-{slug}"),
            Severity::Warning,
            &format!("no {name} file was provided"),
        ));
    } else if count > 1 {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec![format!("package:duplicate-{slug}")],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!("expected one {name} file, received {count}")),
        ));
    }
}

#[allow(clippy::too_many_arguments)]
fn check_layer_role_coherence(
    top_copper: usize,
    bottom_copper: usize,
    inner_copper: usize,
    top_mask: usize,
    bottom_mask: usize,
    top_paste: usize,
    bottom_paste: usize,
    top_silk: usize,
    bottom_silk: usize,
    violations: &mut Vec<Violation>,
) {
    if inner_copper > 0 && (top_copper == 0 || bottom_copper == 0) {
        violations.push(package_violation(
            "inner-copper-without-outer-stack",
            Severity::Warning,
            "inner copper was recognized without both top and bottom copper layers",
        ));
    }

    let copper_count = top_copper + bottom_copper + inner_copper;
    if copper_count > 2 && copper_count % 2 == 1 {
        violations.push(package_violation(
            "odd-copper-layer-stack",
            Severity::Warning,
            "recognized copper layer count is odd; production stackups are normally even-layer constructions",
        ));
    }

    check_orphan_side_layer(
        top_copper,
        top_mask,
        "orphan-top-mask",
        "top solder mask was recognized without top copper",
        violations,
    );
    check_orphan_side_layer(
        bottom_copper,
        bottom_mask,
        "orphan-bottom-mask",
        "bottom solder mask was recognized without bottom copper",
        violations,
    );
    check_orphan_side_layer(
        top_copper,
        top_paste,
        "orphan-top-paste",
        "top paste was recognized without top copper",
        violations,
    );
    check_orphan_side_layer(
        bottom_copper,
        bottom_paste,
        "orphan-bottom-paste",
        "bottom paste was recognized without bottom copper",
        violations,
    );
    check_orphan_side_layer(
        top_copper,
        top_silk,
        "orphan-top-silkscreen",
        "top silkscreen was recognized without top copper",
        violations,
    );
    check_orphan_side_layer(
        bottom_copper,
        bottom_silk,
        "orphan-bottom-silkscreen",
        "bottom silkscreen was recognized without bottom copper",
        violations,
    );

    let bottom_outputs = bottom_mask + bottom_paste + bottom_silk;
    if copper_count == 1 && top_copper > 0 && bottom_outputs > 0 {
        violations.push(package_violation(
            "ambiguous-single-copper-with-bottom-outputs",
            Severity::Warning,
            "package has one recognized copper layer but also contains bottom-side manufacturing outputs",
        ));
    }
    let top_outputs = top_mask + top_paste + top_silk;
    if copper_count == 1 && bottom_copper > 0 && top_outputs > 0 {
        violations.push(package_violation(
            "ambiguous-single-copper-with-top-outputs",
            Severity::Warning,
            "package has one recognized copper layer but also contains top-side manufacturing outputs",
        ));
    }
    if top_paste > 0 && top_mask == 0 {
        violations.push(package_violation(
            "top-paste-without-mask",
            Severity::Warning,
            "top paste was recognized without a top solder mask layer; review paste/mask export completeness",
        ));
    }
    if bottom_paste > 0 && bottom_mask == 0 {
        violations.push(package_violation(
            "bottom-paste-without-mask",
            Severity::Warning,
            "bottom paste was recognized without a bottom solder mask layer; review paste/mask export completeness",
        ));
    }
}

fn check_orphan_side_layer(
    copper_count: usize,
    side_output_count: usize,
    slug: &str,
    message: &str,
    violations: &mut Vec<Violation>,
) {
    if copper_count == 0 && side_output_count > 0 {
        violations.push(package_violation(slug, Severity::Warning, message));
    }
}

fn check_revision_consistency(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let mut revisions = std::collections::BTreeMap::<String, Vec<String>>::new();
    for path in manifest_paths(input) {
        if let Some(revision) = revision_tag(&path) {
            revisions.entry(revision).or_default().push(path);
        }
    }

    if revisions.len() <= 1 {
        return;
    }

    let summary = revisions
        .into_iter()
        .map(|(revision, paths)| format!("{revision}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-revisions".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "input package appears to mix revision tags across files: {summary}"
        )),
    ));
}

fn check_generated_date_consistency(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let mut dates = std::collections::BTreeMap::<String, Vec<String>>::new();
    for path in manifest_paths(input) {
        if let Some(date) = generated_date_tag(&path) {
            dates.entry(date).or_default().push(path);
        }
    }

    if dates.len() <= 1 {
        return;
    }

    let summary = dates
        .into_iter()
        .map(|(date, paths)| format!("{date}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-generated-dates".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "input package appears to mix generated date tags across files: {summary}"
        )),
    ));
}

fn check_x2_creation_date_consistency(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let mut dates = std::collections::BTreeMap::<String, Vec<String>>::new();
    for (path, date) in x2_creation_date_tags(input) {
        dates.entry(date).or_default().push(path);
    }

    if dates.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.6:
    // `.CreationDate` defines each Gerber file creation timestamp. Mixed dates
    // across parseable Gerber headers can indicate a stale layer was left in
    // the release package even when filenames are opaque.
    let summary = dates
        .into_iter()
        .map(|(date, paths)| format!("{date}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-x2-creation-dates".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber X2 CreationDate values differ across layers: {summary}"
        )),
    ));
}

fn check_project_name_consistency(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let mut names = std::collections::BTreeMap::<String, Vec<String>>::new();
    for path in manifest_paths(input) {
        if let Some(name) = project_name_tag(&path) {
            names.entry(name).or_default().push(path);
        }
    }

    if names.len() <= 1 {
        return;
    }

    let summary = names
        .into_iter()
        .map(|(name, paths)| format!("{name}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-project-names".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "input package appears to mix project or job name tags across files: {summary}"
        )),
    ));
}

fn check_stale_artifact_names(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let stale_paths = manifest_paths(input)
        .into_iter()
        .filter(|path| has_stale_name_token(path))
        .collect::<Vec<_>>();

    if stale_paths.is_empty() {
        return;
    }

    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:stale-artifact-name".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "input package contains filenames that look stale or archival: {}",
            stale_paths.join(", ")
        )),
    ));
}

fn manifest_paths(input: &ManifestInput) -> Vec<String> {
    input
        .gerber_layers
        .iter()
        .map(|layer| layer.source_path.clone())
        .chain(input.artifact_paths.iter().cloned())
        .collect()
}

fn x2_creation_date_tags(input: &ManifestInput) -> Vec<(String, String)> {
    input
        .gerber_layers
        .iter()
        .filter_map(|layer| {
            layer
                .creation_date
                .as_deref()
                .and_then(normalize_x2_creation_date)
                .map(|date| (layer.source_path.clone(), date))
        })
        .collect()
}

fn normalize_x2_creation_date(value: &str) -> Option<String> {
    let trimmed = value.trim();
    if let Some(date) = trimmed.get(0..10)
        && parse_iso_day(date).is_some()
    {
        return Some(format!("{}{}{}", &date[0..4], &date[5..7], &date[8..10]));
    }

    let digits = trimmed
        .chars()
        .filter(|ch| ch.is_ascii_digit())
        .take(8)
        .collect::<String>();
    normalize_date_token(&digits)
}

fn has_stale_name_token(path: &str) -> bool {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    stem.split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .any(|token| {
            matches!(
                token.to_ascii_lowercase().as_str(),
                "backup" | "bak" | "old" | "obsolete" | "previous" | "archive" | "archived"
            )
        })
}

fn revision_tag(path: &str) -> Option<String> {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    let tokens = stem
        .split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();

    for (index, token) in tokens.iter().enumerate() {
        let normalized = token.to_ascii_uppercase();
        if normalized == "REV" {
            if let Some(next) = tokens.get(index + 1)
                && looks_revision_value(next)
            {
                return Some(format!("REV{}", next.to_ascii_uppercase()));
            }
        } else if let Some(suffix) = normalized.strip_prefix("REV") {
            if looks_revision_value(suffix) {
                return Some(normalized);
            }
        } else if let Some(suffix) = normalized.strip_prefix('V')
            && suffix.chars().next().is_some_and(|ch| ch.is_ascii_digit())
        {
            return Some(normalized);
        }
    }

    None
}

fn generated_date_tag(path: &str) -> Option<String> {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    let tokens = stem
        .split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .collect::<Vec<_>>();

    for token in &tokens {
        if let Some(date) = normalize_date_token(token) {
            return Some(date);
        }
    }

    for window in tokens.windows(3) {
        let [year, month, day] = window else {
            continue;
        };
        if year.len() == 4
            && year.chars().all(|ch| ch.is_ascii_digit())
            && month.chars().all(|ch| ch.is_ascii_digit())
            && day.chars().all(|ch| ch.is_ascii_digit())
        {
            let candidate = format!("{year}{month:0>2}{day:0>2}");
            if let Some(date) = normalize_date_token(&candidate) {
                return Some(date);
            }
        }
    }

    None
}

fn project_name_tag(path: &str) -> Option<String> {
    let stem = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    stem.split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .find_map(|token| {
            let normalized = token.to_ascii_lowercase();
            (!is_manifest_noise_token(&normalized) && !normalize_date_token(&normalized).is_some())
                .then_some(normalized)
        })
}

fn is_manifest_noise_token(token: &str) -> bool {
    token.len() <= 2
        || token.contains("copper")
        || token.contains("mask")
        || token.contains("paste")
        || token.contains("silk")
        || token.contains("soldermask")
        || token.contains("silkscreen")
        || token.contains("outline")
        || token.contains("profile")
        || token.starts_with("inner")
        || matches!(
            token,
            "gerber"
                | "top"
                | "bottom"
                | "front"
                | "back"
                | "copper"
                | "cu"
                | "mask"
                | "soldermask"
                | "paste"
                | "silk"
                | "silkscreen"
                | "outline"
                | "profile"
                | "edge"
                | "cuts"
                | "board"
                | "layer"
                | "inner"
                | "fab"
                | "fabrication"
                | "assembly"
                | "assy"
                | "drawing"
                | "bom"
                | "centroid"
                | "placement"
                | "positions"
                | "pnp"
                | "netlist"
                | "readme"
                | "release"
                | "notes"
                | "rout"
                | "route"
                | "routing"
                | "panel"
                | "tooling"
                | "rev"
                | "revision"
                | "version"
        )
        || token.starts_with("rev")
}

fn normalize_date_token(token: &str) -> Option<String> {
    if token.len() != 8 || !token.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }
    let year = token[0..4].parse::<u16>().ok()?;
    if !(2000..=2099).contains(&year) || parse_compact_day(token).is_none() {
        return None;
    }
    Some(token.to_string())
}

fn looks_revision_value(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 8
        && value
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || ch == '.')
}

fn classify_gerber_role(layer: &ManifestGerberLayer) -> GerberRole {
    if let Some(role) = layer
        .file_function
        .as_deref()
        .and_then(classify_file_function)
    {
        return role;
    }

    let path = layer.source_path.to_ascii_lowercase();
    let name = layer.name.to_ascii_lowercase();
    let combined = format!("{path} {name}");

    if has_any(
        &combined,
        &["edge.cuts", "outline", "profile", "boardoutline"],
    ) || path.ends_with(".gko")
        || path.ends_with(".gm1")
        || path.ends_with(".gm2")
        || path.ends_with(".gml")
    {
        GerberRole::Outline
    } else if has_any(
        &combined,
        &[
            "topcopper",
            "top_copper",
            "top-copper",
            "f.cu",
            "front copper",
        ],
    ) || path.ends_with(".gtl")
    {
        GerberRole::TopCopper
    } else if has_any(
        &combined,
        &[
            "bottomcopper",
            "bottom_copper",
            "bottom-copper",
            "b.cu",
            "back copper",
        ],
    ) || path.ends_with(".gbl")
    {
        GerberRole::BottomCopper
    } else if has_any(&combined, &["inner", "in1", "in2", "g1", "g2"]) {
        GerberRole::InnerCopper
    } else if has_any(
        &combined,
        &[
            "topsoldermask",
            "top_soldermask",
            "top-soldermask",
            "top-mask",
            "f.mask",
        ],
    ) || path.ends_with(".gts")
    {
        GerberRole::TopMask
    } else if has_any(
        &combined,
        &[
            "bottomsoldermask",
            "bottom_soldermask",
            "bottom-soldermask",
            "bottom-mask",
            "b.mask",
        ],
    ) || path.ends_with(".gbs")
    {
        GerberRole::BottomMask
    } else if has_any(
        &combined,
        &[
            "topsolderpaste",
            "top_solderpaste",
            "top-solderpaste",
            "top-paste",
            "f.paste",
        ],
    ) || path.ends_with(".gtp")
        || path.ends_with(".gpt")
    {
        GerberRole::TopPaste
    } else if has_any(
        &combined,
        &[
            "bottomsolderpaste",
            "bottom_solderpaste",
            "bottom-solderpaste",
            "bottom-paste",
            "b.paste",
        ],
    ) || path.ends_with(".gbp")
        || path.ends_with(".gpb")
    {
        GerberRole::BottomPaste
    } else if has_any(
        &combined,
        &[
            "topsilkscreen",
            "top_silkscreen",
            "top-silkscreen",
            "top-silk",
            "f.silk",
        ],
    ) || path.ends_with(".gto")
    {
        GerberRole::TopSilk
    } else if has_any(
        &combined,
        &[
            "bottomsilkscreen",
            "bottom_silkscreen",
            "bottom-silkscreen",
            "bottom-silk",
            "b.silk",
        ],
    ) || path.ends_with(".gbo")
    {
        GerberRole::BottomSilk
    } else {
        GerberRole::Other
    }
}

fn classify_file_function(file_function: &str) -> Option<GerberRole> {
    let tokens = file_function_tokens(file_function);
    let function = tokens.first()?;

    match function.as_str() {
        "copper" => match file_function_side(&tokens) {
            Some(GerberSide::Top) => Some(GerberRole::TopCopper),
            Some(GerberSide::Bottom) => Some(GerberRole::BottomCopper),
            None => Some(GerberRole::InnerCopper),
        },
        "soldermask" | "mask" => match file_function_side(&tokens) {
            Some(GerberSide::Top) => Some(GerberRole::TopMask),
            Some(GerberSide::Bottom) => Some(GerberRole::BottomMask),
            None => None,
        },
        "paste" | "solderpaste" => match file_function_side(&tokens) {
            Some(GerberSide::Top) => Some(GerberRole::TopPaste),
            Some(GerberSide::Bottom) => Some(GerberRole::BottomPaste),
            None => None,
        },
        "legend" | "silk" | "silkscreen" => match file_function_side(&tokens) {
            Some(GerberSide::Top) => Some(GerberRole::TopSilk),
            Some(GerberSide::Bottom) => Some(GerberRole::BottomSilk),
            None => None,
        },
        "profile" | "outline" => Some(GerberRole::Outline),
        _ => None,
    }
}

fn file_function_tokens(file_function: &str) -> Vec<String> {
    file_function
        .split(',')
        .map(|field| {
            field
                .trim()
                .chars()
                .filter(|ch| !ch.is_ascii_whitespace() && *ch != '-' && *ch != '_')
                .collect::<String>()
                .to_ascii_lowercase()
        })
        .filter(|field| !field.is_empty())
        .collect()
}

fn file_function_side(tokens: &[String]) -> Option<GerberSide> {
    if tokens
        .iter()
        .any(|token| matches!(token.as_str(), "top" | "front" | "l1"))
    {
        Some(GerberSide::Top)
    } else if tokens
        .iter()
        .any(|token| matches!(token.as_str(), "bot" | "bottom" | "back"))
    {
        Some(GerberSide::Bottom)
    } else {
        None
    }
}

fn has_any(haystack: &str, needles: &[&str]) -> bool {
    needles.iter().any(|needle| haystack.contains(needle))
}

fn role_count(classified: &[(&ManifestGerberLayer, GerberRole)], role: GerberRole) -> usize {
    classified
        .iter()
        .filter(|(_, classified_role)| *classified_role == role)
        .count()
}

fn duplicate_role_warning(
    classified: &[(&ManifestGerberLayer, GerberRole)],
    role: GerberRole,
    slug: &str,
    violations: &mut Vec<Violation>,
) {
    let paths = classified
        .iter()
        .filter_map(|(layer, classified_role)| {
            (*classified_role == role).then_some(layer.source_path.clone())
        })
        .collect::<Vec<_>>();

    if paths.len() > 1 {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec![format!("package:{slug}")],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "multiple files were recognized for one manufacturing role: {}",
                paths.join(", ")
            )),
        ));
    }
}

fn check_layer_name_side_conflicts(
    classified: &[(&ManifestGerberLayer, GerberRole)],
    violations: &mut Vec<Violation>,
) {
    for (layer, role) in classified {
        let Some(expected_side) = gerber_role_side(*role) else {
            continue;
        };
        let text = format!("{} {}", layer.source_path, layer.name).to_ascii_lowercase();
        let has_wrong_side = match expected_side {
            GerberSide::Top => has_bottom_side_marker(&text),
            GerberSide::Bottom => has_top_side_marker(&text),
        };
        if !has_wrong_side {
            continue;
        }

        let side = match expected_side {
            GerberSide::Top => "top",
            GerberSide::Bottom => "bottom",
        };
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:layer-side-name-conflict".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "file {:?} was classified as a {side}-side manufacturing layer, but its name also contains opposite-side tokens",
                layer.source_path
            )),
        ));
    }
}

fn check_negative_copper_file_polarity(
    classified: &[(&ManifestGerberLayer, GerberRole)],
    violations: &mut Vec<Violation>,
) {
    let negative_copper_paths = classified
        .iter()
        .filter(|(layer, role)| {
            is_copper_role(*role)
                && layer
                    .file_polarity
                    .as_deref()
                    .is_some_and(is_negative_file_polarity)
        })
        .map(|(layer, _)| layer.source_path.clone())
        .collect::<Vec<_>>();

    if negative_copper_paths.is_empty() {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.4:
    // `.FilePolarity` changes interpretation, not image geometry. Negative
    // copper means image objects represent absence of copper, so the release
    // package needs explicit CAM review before fabrication.
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:negative-copper-file-polarity".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "copper Gerber layer(s) declare X2 FilePolarity=Negative; review negative-plane CAM polarity before fabrication: {}",
            negative_copper_paths.join(", ")
        )),
    ));
}

fn is_copper_role(role: GerberRole) -> bool {
    matches!(
        role,
        GerberRole::TopCopper | GerberRole::BottomCopper | GerberRole::InnerCopper
    )
}

fn is_negative_file_polarity(file_polarity: &str) -> bool {
    file_polarity.trim().eq_ignore_ascii_case("negative")
}

fn check_file_polarity_evidence(
    input: &ManifestInput,
    classified: &[(&ManifestGerberLayer, GerberRole)],
    violations: &mut Vec<Violation>,
) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.file_polarity.is_some())
        .count();
    if attested_layers == 0 {
        return;
    }

    if attested_layers < input.gerber_layers.len() {
        let missing = input
            .gerber_layers
            .iter()
            .filter(|layer| layer.file_polarity.is_none())
            .map(|layer| layer.source_path.clone())
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-x2-file-polarity-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare X2 FilePolarity but others do not; review CAM polarity evidence: {}",
                missing.join(", ")
            )),
        ));
    }

    let mut polarities = std::collections::BTreeMap::<String, Vec<String>>::new();
    for (layer, role) in classified {
        if !is_copper_role(*role) {
            continue;
        }
        let Some(value) = layer.file_polarity.as_deref() else {
            continue;
        };
        let normalized = normalize_file_polarity(value);
        if normalized.is_empty() {
            continue;
        }
        polarities
            .entry(normalized)
            .or_default()
            .push(layer.source_path.clone());
    }

    if polarities.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.4:
    // `.FilePolarity` declares whether the file image is positive or negative.
    // Mixed polarity is expected between positive artwork and negative
    // soldermask openings, so this review is intentionally scoped to copper
    // roles where it can change the CAM interpretation of conductive material.
    let summary = polarities
        .into_iter()
        .map(|(value, paths)| format!("{value}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-x2-file-polarities".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber X2 FilePolarity values differ across layers: {summary}"
        )),
    ));
}

fn normalize_file_polarity(value: &str) -> String {
    match value.trim().to_ascii_lowercase().as_str() {
        "positive" => "Positive".to_string(),
        "negative" => "Negative".to_string(),
        _ => value.trim().to_string(),
    }
}

fn check_file_function_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.file_function.is_some())
        .count();
    if attested_layers == 0 || attested_layers == input.gerber_layers.len() {
        return;
    }

    let missing = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.file_function.is_none())
        .map(|layer| layer.source_path.clone())
        .collect::<Vec<_>>();

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.3:
    // `.FileFunction` is the machine-readable layer role. Partial coverage
    // means some layers are explicitly attested while others still depend on
    // filename conventions, which weakens release-package traceability.
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:partial-x2-file-function-evidence".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "some Gerber layers declare X2 FileFunction but others do not; review layer-role provenance: {}",
            missing.join(", ")
        )),
    ));
}

fn check_image_setup_consistency(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let mut units = std::collections::BTreeMap::<String, Vec<String>>::new();
    let mut formats = std::collections::BTreeMap::<String, Vec<String>>::new();
    let mut missing_units = Vec::new();
    let mut missing_formats = Vec::new();

    for layer in &input.gerber_layers {
        match layer.units.as_deref() {
            Some(value) => units
                .entry(value.to_string())
                .or_default()
                .push(layer.source_path.clone()),
            None => missing_units.push(layer.source_path.clone()),
        }
        match layer.coordinate_format.as_deref() {
            Some(value) => formats
                .entry(value.to_string())
                .or_default()
                .push(layer.source_path.clone()),
            None => missing_formats.push(layer.source_path.clone()),
        }
    }

    if units.len() > 1 {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:mixed-gerber-units".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Gerber package mixes image units across layers; review parser normalization: {}",
                format_evidence_groups(&units)
            )),
        ));
    }

    if !units.is_empty() && !missing_units.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-gerber-unit-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare image units while others do not; review source-unit provenance: {}",
                missing_units.join(", ")
            )),
        ));
    }

    if formats.len() > 1 {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:mixed-gerber-coordinate-format".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "Gerber package mixes coordinate formats across layers; review fixed-coordinate interpretation: {}",
                format_evidence_groups(&formats)
            )),
        ));
    }

    if !formats.is_empty() && !missing_formats.is_empty() {
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-gerber-coordinate-format-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare coordinate format while others do not; review fixed-coordinate provenance: {}",
                missing_formats.join(", ")
            )),
        ));
    }
}

fn format_evidence_groups(groups: &std::collections::BTreeMap<String, Vec<String>>) -> String {
    groups
        .iter()
        .map(|(value, paths)| format!("{value} in {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ")
}

fn check_part_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.part.is_some())
        .count();
    if attested_layers == 0 {
        return;
    }

    if attested_layers < input.gerber_layers.len() {
        let missing = input
            .gerber_layers
            .iter()
            .filter(|layer| layer.part.is_none())
            .map(|layer| layer.source_path.clone())
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-x2-part-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare X2 Part but others do not; review single/panel/coupon package intent: {}",
                missing.join(", ")
            )),
        ));
    }

    let mut parts = std::collections::BTreeMap::<String, Vec<String>>::new();
    for layer in &input.gerber_layers {
        let Some(value) = layer.part.as_deref() else {
            continue;
        };
        let normalized = normalize_part(value);
        if normalized.is_empty() {
            continue;
        }
        parts
            .entry(normalized)
            .or_default()
            .push(layer.source_path.clone());
    }

    if parts.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.2:
    // `.Part` distinguishes Single, Array, FabricationPanel, Coupon, and
    // Other-part files. Mixing those values in one ordinary layer set often
    // means a panel, coupon, or stale single-board layer was packaged with the
    // wrong release artifact.
    let summary = parts
        .into_iter()
        .map(|(value, paths)| format!("{value}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-x2-parts".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber X2 Part values differ across layers: {summary}"
        )),
    ));
}

fn normalize_part(value: &str) -> String {
    let fields = value
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect::<Vec<_>>();
    let Some(kind) = fields.first() else {
        return String::new();
    };
    let kind = kind.to_ascii_lowercase();
    match kind.as_str() {
        "single" => "Single".to_string(),
        "array" => "Array".to_string(),
        "fabricationpanel" | "fabrication-panel" | "fabrication_panel" => {
            "FabricationPanel".to_string()
        }
        "coupon" => "Coupon".to_string(),
        "other" => {
            if fields.len() > 1 {
                format!("Other,{}", fields[1..].join(","))
            } else {
                "Other".to_string()
            }
        }
        _ => fields.join(","),
    }
}

fn check_same_coordinates_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.same_coordinates.is_some())
        .count();
    if attested_layers == 0 {
        return;
    }

    if attested_layers < input.gerber_layers.len() {
        let missing = input
            .gerber_layers
            .iter()
            .filter(|layer| layer.same_coordinates.is_none())
            .map(|layer| layer.source_path.clone())
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-same-coordinates-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare X2 SameCoordinates but others do not; review package alignment evidence: {}",
                missing.join(", ")
            )),
        ));
    }

    let mut identifiers = std::collections::BTreeMap::<String, Vec<String>>::new();
    for layer in &input.gerber_layers {
        let Some(identifier) = layer.same_coordinates.as_deref() else {
            continue;
        };
        let identifier = identifier.trim();
        if !identifier.is_empty() {
            identifiers
                .entry(identifier.to_string())
                .or_default()
                .push(layer.source_path.clone());
        }
    }

    if identifiers.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.5:
    // `.SameCoordinates` can carry an identifier so files generated after a CAD
    // origin change do not falsely claim alignment with older output. Mixed
    // identifiers therefore deserve release-package review.
    let summary = identifiers
        .into_iter()
        .map(|(identifier, paths)| format!("{identifier}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-same-coordinates-identifiers".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber layers declare different X2 SameCoordinates identifiers: {summary}"
        )),
    ));
}

fn check_generation_software_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.generation_software.is_some())
        .count();
    if attested_layers == 0 {
        return;
    }

    if attested_layers < input.gerber_layers.len() {
        let missing = input
            .gerber_layers
            .iter()
            .filter(|layer| layer.generation_software.is_none())
            .map(|layer| layer.source_path.clone())
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-x2-generation-software-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare X2 GenerationSoftware but others do not; review output provenance: {}",
                missing.join(", ")
            )),
        ));
    }

    let mut software = std::collections::BTreeMap::<String, Vec<String>>::new();
    for layer in &input.gerber_layers {
        let Some(value) = layer.generation_software.as_deref() else {
            continue;
        };
        let normalized = normalize_generation_software(value);
        if normalized.is_empty() {
            continue;
        }
        software
            .entry(normalized)
            .or_default()
            .push(layer.source_path.clone());
    }

    if software.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.7:
    // `.GenerationSoftware` identifies the program that created the layer.
    // Mixed tools or versions in one release can be legitimate, but they are a
    // strong signal for stale CAM output or post-processed layers.
    let summary = software
        .into_iter()
        .map(|(value, paths)| format!("{value}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-x2-generation-software".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber X2 GenerationSoftware values differ across layers: {summary}"
        )),
    ));
}

fn normalize_generation_software(value: &str) -> String {
    value
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect::<Vec<_>>()
        .join(",")
}

fn check_project_id_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.project_id.is_some())
        .count();
    if attested_layers == 0 {
        return;
    }

    if attested_layers < input.gerber_layers.len() {
        let missing = input
            .gerber_layers
            .iter()
            .filter(|layer| layer.project_id.is_none())
            .map(|layer| layer.source_path.clone())
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "file-manifest-readiness",
            Severity::Warning,
            vec!["package:partial-x2-project-id-evidence".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(format!(
                "some Gerber layers declare X2 ProjectId but others do not; review project/revision provenance: {}",
                missing.join(", ")
            )),
        ));
    }

    let mut project_ids = std::collections::BTreeMap::<String, Vec<String>>::new();
    for layer in &input.gerber_layers {
        let Some(value) = layer.project_id.as_deref() else {
            continue;
        };
        let normalized = normalize_project_id(value);
        if normalized.is_empty() {
            continue;
        }
        project_ids
            .entry(normalized)
            .or_default()
            .push(layer.source_path.clone());
    }

    if project_ids.len() <= 1 {
        return;
    }

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.8:
    // `.ProjectId` is intended to identify the project and revision across a
    // fabrication set. Mixed values therefore map directly to release-baseline
    // consistency review, just like filename revision tags but without relying
    // on naming conventions.
    let summary = project_ids
        .into_iter()
        .map(|(value, paths)| format!("{value}: {}", paths.join(", ")))
        .collect::<Vec<_>>()
        .join("; ");
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:mixed-x2-project-ids".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "Gerber X2 ProjectId values differ across layers: {summary}"
        )),
    ));
}

fn normalize_project_id(value: &str) -> String {
    value
        .split(',')
        .map(str::trim)
        .filter(|field| !field.is_empty())
        .collect::<Vec<_>>()
        .join(",")
}

fn check_md5_evidence(input: &ManifestInput, violations: &mut Vec<Violation>) {
    let attested_layers = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.md5.is_some())
        .count();
    if attested_layers == 0 || attested_layers == input.gerber_layers.len() {
        return;
    }

    let missing = input
        .gerber_layers
        .iter()
        .filter(|layer| layer.md5.is_none())
        .map(|layer| layer.source_path.clone())
        .collect::<Vec<_>>();

    // Ucamco Gerber Layer Format Specification, rev. 2024.05, section 5.6.9:
    // `.MD5` is per-file checksum evidence. HyperDRC does not recompute the
    // digest yet, but partial coverage still matters: it tells review that only
    // part of the fabrication set carries integrity evidence.
    violations.push(Violation::new(
        "file-manifest-readiness",
        Severity::Warning,
        vec!["package:partial-x2-md5-evidence".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "some Gerber layers declare X2 MD5 checksums but others do not; review package integrity evidence: {}",
            missing.join(", ")
        )),
    ));
}

#[derive(Copy, Clone)]
enum GerberSide {
    Top,
    Bottom,
}

fn gerber_role_side(role: GerberRole) -> Option<GerberSide> {
    match role {
        GerberRole::TopCopper
        | GerberRole::TopMask
        | GerberRole::TopPaste
        | GerberRole::TopSilk => Some(GerberSide::Top),
        GerberRole::BottomCopper
        | GerberRole::BottomMask
        | GerberRole::BottomPaste
        | GerberRole::BottomSilk => Some(GerberSide::Bottom),
        GerberRole::InnerCopper | GerberRole::Outline | GerberRole::Other => None,
    }
}

fn has_top_side_marker(text: &str) -> bool {
    has_side_token(text, &["top", "front", "f"]) || text.contains("f.cu")
}

fn has_bottom_side_marker(text: &str) -> bool {
    has_side_token(text, &["bottom", "bot", "back", "b"]) || text.contains("b.cu")
}

fn has_side_token(text: &str, tokens: &[&str]) -> bool {
    text.split(|ch: char| !ch.is_ascii_alphanumeric())
        .filter(|token| !token.is_empty())
        .any(|token| tokens.contains(&token))
}

fn package_violation(slug: &str, severity: Severity, message: &str) -> Violation {
    Violation::new(
        "file-manifest-readiness",
        severity,
        vec![format!("package:{slug}")],
        None,
        Vec::new(),
        Vec::new(),
        Some(message.to_string()),
    )
}

#[cfg(test)]
mod tests {
    use crate::date::parse_iso_day;

    use super::{
        GerberRole, ManifestGerberLayer, ManifestInput, ManifestLayerRequirements,
        ManifestRequirements, check_generated_date_age, check_x2_creation_date_age,
        check_x2_creation_date_consistency, classify_file_function, classify_gerber_role,
        file_manifest_readiness, normalize_date_token,
    };

    #[test]
    fn complete_gerber_package_with_full_manufacturing_roles_has_no_manifest_gaps() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Gerber_TopCopperLayer.GTL"),
                layer("Gerber_BottomCopperLayer.GBL"),
                layer("Gerber_TopSolderMaskLayer.GTS"),
                layer("Gerber_BottomSolderMaskLayer.GBS"),
                layer("Gerber_TopSolderPasteLayer.GTP"),
                layer("Gerber_BottomSolderPasteLayer.GBP"),
                layer("Gerber_TopSilkscreenLayer.GTO"),
                layer("Gerber_BottomSilkscreenLayer.GBO"),
                layer("Gerber_BoardOutlineLayer.GKO"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };

        let violations = file_manifest_readiness(&input);
        assert!(violations.is_empty(), "{violations:#?}");
    }

    #[test]
    fn missing_production_deliverables_are_reported_independently() {
        let input = ManifestInput {
            gerber_layers: vec![layer("board-top.gtl")],
            has_board_outline: false,
            has_drill_data: false,
            ..Default::default()
        };

        let violations = file_manifest_readiness(&input);
        let layers = violations
            .iter()
            .flat_map(|violation| violation.layers.clone())
            .collect::<Vec<_>>();

        assert!(layers.contains(&"package:missing-board-outline".to_string()));
        assert!(layers.contains(&"package:missing-drill-data".to_string()));
        assert!(layers.contains(&"package:missing-top-mask".to_string()));
        assert!(layers.contains(&"package:missing-top-silkscreen".to_string()));
        assert!(layers.contains(&"package:missing-top-paste".to_string()));
    }

    #[test]
    fn duplicate_role_detection_lists_conflicting_files() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("first.gtl"),
                layer("second-top-copper.gbr"),
                layer("outline.gko"),
                layer("top-mask.gts"),
                layer("top-silk.gto"),
                layer("top-solderpaste.gtp"),
                layer("top-solderpaste-backup.gtp"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };

        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:duplicate-top-copper".to_string())
        }));

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:duplicate-top-paste".to_string())
        }));
    }

    #[test]
    fn complete_multi_role_package_with_mixed_layers_has_no_delivery_gaps() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("TopCopper.gtl"),
                layer("BottomCopper.gbl"),
                layer("Inner1.g2"),
                layer("Inner2.g3"),
                layer("TopMask.gts"),
                layer("BottomMask.gbs"),
                layer("TopPaste.gtp"),
                layer("BottomPaste.gbp"),
                layer("TopSilk.gto"),
                layer("BottomSilk.gbo"),
                layer("Profile-outline.gko"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };

        let violations = file_manifest_readiness(&input);
        assert!(violations.is_empty(), "{violations:#?}");
    }

    #[test]
    fn production_readiness_files_require_exactly_one_input_each() {
        let input = ManifestInput {
            bom_file_count: 0,
            centroid_file_count: 2,
            netlist_file_count: 2,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 0,
            readme_file_count: 1,
            rout_drawing_file_count: 2,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violations
            .iter()
            .flat_map(|violation| violation.layers.iter())
            .cloned()
            .collect::<Vec<_>>();

        assert!(slugs.contains(&"package:missing-bom".to_string()));
        assert!(slugs.contains(&"package:duplicate-centroid".to_string()));
        assert!(slugs.contains(&"package:missing-assembly-drawing".to_string()));
        assert!(slugs.contains(&"package:duplicate-netlist".to_string()));
        assert!(slugs.contains(&"package:duplicate-rout-drawing".to_string()));
    }

    #[test]
    fn optional_production_artifacts_do_not_report_missing_inputs() {
        let input = ManifestInput {
            gerber_layers: vec![layer("TopCopper.GTL")],
            has_board_outline: true,
            has_drill_data: true,
            required_artifacts: ManifestRequirements {
                bom: true,
                centroid: false,
                netlist: false,
                fab_drawing: true,
                assembly_drawing: false,
                readme: true,
                rout_drawing: false,
            },
            bom_file_count: 1,
            fab_drawing_file_count: 1,
            readme_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:missing-centroid".to_string()));
        assert!(!slugs.contains(&"package:missing-netlist".to_string()));
        assert!(!slugs.contains(&"package:missing-assembly-drawing".to_string()));
        assert!(!slugs.contains(&"package:missing-rout-drawing".to_string()));
    }

    #[test]
    fn optional_production_artifacts_still_report_duplicate_inputs() {
        let input = ManifestInput {
            gerber_layers: vec![layer("TopCopper.GTL")],
            has_board_outline: true,
            has_drill_data: true,
            required_artifacts: ManifestRequirements {
                centroid: false,
                ..ManifestRequirements::default()
            },
            bom_file_count: 1,
            centroid_file_count: 2,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:duplicate-centroid".to_string()));
    }

    #[test]
    fn optional_layer_roles_do_not_report_missing_layers() {
        let input = ManifestInput {
            gerber_layers: vec![layer("TopCopper.GTL")],
            required_layers: ManifestLayerRequirements {
                board_outline: false,
                drill_data: false,
                top_mask: false,
                bottom_mask: true,
                top_paste: false,
                bottom_paste: true,
                top_silkscreen: false,
                bottom_silkscreen: true,
            },
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:missing-board-outline".to_string()));
        assert!(!slugs.contains(&"package:missing-drill-data".to_string()));
        assert!(!slugs.contains(&"package:missing-top-mask".to_string()));
        assert!(!slugs.contains(&"package:missing-top-paste".to_string()));
        assert!(!slugs.contains(&"package:missing-top-silkscreen".to_string()));
    }

    #[test]
    fn optional_layer_roles_still_report_duplicate_layers() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("TopCopper.GTL"),
                layer("TopPaste.GTP"),
                layer("TopPaste-backup.GTP"),
            ],
            required_layers: ManifestLayerRequirements {
                top_paste: false,
                ..ManifestLayerRequirements::default()
            },
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:duplicate-top-paste".to_string()));
    }

    #[test]
    fn declared_copper_layer_count_is_checked_against_manifest() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("TopCopper.GTL"),
                layer("BottomCopper.GBL"),
                layer("Inner1.G2"),
            ],
            declared_copper_layer_count: Some(4),
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violations
            .iter()
            .flat_map(|violation| violation.layers.iter())
            .cloned()
            .collect::<Vec<_>>();

        assert!(slugs.contains(&"package:declared-copper-layer-parity".to_string()));
    }

    #[test]
    fn kicad_and_gerber_copper_layer_counts_are_compared() {
        let input = ManifestInput {
            gerber_layers: vec![layer("TopCopper.GTL"), layer("BottomCopper.GBL")],
            kicad_copper_layer_count: Some(4),
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violations
            .iter()
            .flat_map(|violation| violation.layers.iter())
            .cloned()
            .collect::<Vec<_>>();

        assert!(slugs.contains(&"package:copper-layer-parity".to_string()));
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("exposes 2"))
        }));
    }

    #[test]
    fn filename_layer_count_tags_are_checked_against_manifest_stack() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_4layer_TopCopper.GTL"),
                layer("Widget_4layer_BottomCopper.GBL"),
                layer("Widget_4layer_TopMask.GTS"),
                layer("Widget_4layer_BottomMask.GBS"),
                layer("Widget_4layer_TopPaste.GTP"),
                layer("Widget_4layer_BottomPaste.GBP"),
                layer("Widget_4layer_TopSilk.GTO"),
                layer("Widget_4layer_BottomSilk.GBO"),
                layer("Widget_4layer_Outline.GKO"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:filename-copper-layer-parity".to_string()));
        assert!(violations.iter().any(|violation| {
            violation.message.as_deref().is_some_and(|message| {
                message.contains("filename layer-count tag (4)") && message.contains("observed")
            })
        }));
    }

    #[test]
    fn conflicting_filename_layer_count_tags_are_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_4layer_TopCopper.GTL"),
                layer("Widget_6layer_BottomCopper.GBL"),
                layer("Widget_4layer_TopMask.GTS"),
                layer("Widget_6layer_BottomMask.GBS"),
                layer("Widget_4layer_TopPaste.GTP"),
                layer("Widget_6layer_BottomPaste.GBP"),
                layer("Widget_4layer_TopSilk.GTO"),
                layer("Widget_6layer_BottomSilk.GBO"),
                layer("Widget_4layer_Outline.GKO"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:filename-layer-count-conflict".to_string()));
        assert!(!slugs.contains(&"package:filename-copper-layer-parity".to_string()));
    }

    #[test]
    fn orphan_side_outputs_are_reported_without_matching_copper() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("TopCopper.GTL"),
                layer("BottomSolderMask.GBS"),
                layer("BottomSilkscreen.GBO"),
                layer("BottomPaste.GBP"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:orphan-bottom-mask".to_string()));
        assert!(slugs.contains(&"package:orphan-bottom-silkscreen".to_string()));
        assert!(slugs.contains(&"package:orphan-bottom-paste".to_string()));
        assert!(slugs.contains(&"package:ambiguous-single-copper-with-bottom-outputs".to_string()));
    }

    #[test]
    fn bottom_single_copper_with_top_outputs_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("BottomCopper.GBL"),
                layer("TopSolderMask.GTS"),
                layer("TopSilkscreen.GTO"),
                layer("TopPaste.GTP"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:orphan-top-mask".to_string()));
        assert!(slugs.contains(&"package:orphan-top-silkscreen".to_string()));
        assert!(slugs.contains(&"package:orphan-top-paste".to_string()));
        assert!(slugs.contains(&"package:ambiguous-single-copper-with-top-outputs".to_string()));
    }

    #[test]
    fn paste_without_matching_mask_is_reported_even_when_mask_is_optional() {
        let input = ManifestInput {
            gerber_layers: vec![layer("TopCopper.GTL"), layer("TopPaste.GTP")],
            required_layers: ManifestLayerRequirements {
                top_mask: false,
                top_silkscreen: false,
                board_outline: false,
                drill_data: false,
                ..ManifestLayerRequirements::default()
            },
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:top-paste-without-mask".to_string()));
        assert!(!slugs.contains(&"package:missing-top-mask".to_string()));
    }

    #[test]
    fn layer_name_side_conflicts_are_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_B_Cu.gtl"),
                layer("Widget_top_layer.gbs"),
                layer("Widget_EdgeCuts.gko"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(
            violation_slugs(&violations)
                .iter()
                .filter(|slug| *slug == "package:layer-side-name-conflict")
                .count()
                >= 2
        );
    }

    #[test]
    fn inner_copper_without_outer_stack_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Inner1.G2"),
                layer("TopSolderMask.GTS"),
                layer("BottomSolderMask.GBS"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:inner-copper-without-outer-stack".to_string()));
        assert!(slugs.contains(&"package:orphan-top-mask".to_string()));
        assert!(slugs.contains(&"package:orphan-bottom-mask".to_string()));
    }

    #[test]
    fn odd_copper_layer_stack_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("TopCopper.GTL"),
                layer("Inner1.G2"),
                layer("BottomCopper.GBL"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:odd-copper-layer-stack".to_string()));
    }

    #[test]
    fn role_classification_handles_multiple_naming_patterns_across_keywords_and_extensions() {
        let samples = [
            (layer("TopCopperLayer.gtl"), GerberRole::TopCopper),
            (layer("front copper"), GerberRole::TopCopper),
            (layer("B.Cu"), GerberRole::BottomCopper),
            (layer("back copper"), GerberRole::BottomCopper),
            (layer("in1.g2"), GerberRole::InnerCopper),
            (layer("inner1.g3"), GerberRole::InnerCopper),
            (layer("top-solder-mask.gts"), GerberRole::TopMask),
            (layer("bottom-soldermask"), GerberRole::BottomMask),
            (layer("top paste .GTP"), GerberRole::TopPaste),
            (layer("bottom-paste.gpb"), GerberRole::BottomPaste),
            (layer("TopSilkScreen.gto"), GerberRole::TopSilk),
            (layer("B.Silk screen"), GerberRole::BottomSilk),
            (layer("Edge.Cuts"), GerberRole::Outline),
            (layer("profile.GKO"), GerberRole::Outline),
            (layer("random-layer"), GerberRole::Other),
        ];

        for (layer, expected) in samples {
            assert_eq!(
                classify_gerber_role(&layer),
                expected,
                "mismatch for {}",
                layer.source_path
            );
        }
    }

    #[test]
    fn x2_file_function_attributes_classify_opaque_layer_names() {
        let input = ManifestInput {
            gerber_layers: vec![
                x2_layer("Widget_opaque01.gbr", "Copper,L1,Top", "Positive"),
                x2_layer("Widget_opaque02.gbr", "Copper,L2,Inr,Plane", "Positive"),
                x2_layer("Widget_opaque03.gbr", "Copper,L3,Inr,Signal", "Positive"),
                x2_layer("Widget_opaque04.gbr", "Copper,L4,Bot", "Positive"),
                x2_layer("Widget_opaque05.gbr", "Soldermask,Top", "Negative"),
                x2_layer("Widget_opaque06.gbr", "Soldermask,Bot", "Negative"),
                x2_layer("Widget_opaque07.gbr", "Paste,Top", "Positive"),
                x2_layer("Widget_opaque08.gbr", "Paste,Bot", "Positive"),
                x2_layer("Widget_opaque09.gbr", "Legend,Top", "Positive"),
                x2_layer("Widget_opaque10.gbr", "Legend,Bot", "Positive"),
                x2_layer("Widget_opaque11.gbr", "Profile,NP", "Positive"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };

        let violations = file_manifest_readiness(&input);
        assert!(violations.is_empty(), "{violations:#?}");
    }

    #[test]
    fn partial_x2_file_function_role_evidence_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                x2_layer("Widget_opaque01.gbr", "Copper,L1,Top", "Positive"),
                layer("Widget_B_Cu.gbl"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-x2-file-function-evidence".to_string()));
    }

    #[test]
    fn negative_x2_copper_file_polarity_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                x2_layer("plane.gbr", "Copper,L2,Inr,Plane", "Negative"),
                x2_layer("top.gbr", "Copper,L1,Top", "Positive"),
                x2_layer("bottom.gbr", "Copper,L4,Bot", "Positive"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            required_layers: ManifestLayerRequirements {
                top_mask: false,
                bottom_mask: false,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
                ..ManifestLayerRequirements::default()
            },
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(
            violation_slugs(&violations)
                .contains(&"package:negative-copper-file-polarity".to_string())
        );
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("plane.gbr"))
        }));
    }

    #[test]
    fn partial_and_mixed_x2_file_polarity_evidence_is_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                x2_layer("top.gbr", "Copper,L1,Top", "Positive"),
                x2_layer("plane.gbr", "Copper,L2,Inr,Plane", "Negative"),
                layer("bottom.gbr"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            required_layers: ManifestLayerRequirements {
                top_mask: false,
                bottom_mask: false,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
                ..ManifestLayerRequirements::default()
            },
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:partial-x2-file-polarity-evidence".to_string()));
        assert!(slugs.contains(&"package:mixed-x2-file-polarities".to_string()));
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("bottom.gbr"))
        }));
    }

    #[test]
    fn partial_x2_same_coordinates_evidence_is_reported() {
        let mut top = x2_layer("Widget_top.gbr", "Copper,L1,Top", "Positive");
        top.same_coordinates = Some("PX1".to_string());
        let bottom = x2_layer("Widget_bottom.gbr", "Copper,L2,Bot", "Positive");
        let input = ManifestInput {
            gerber_layers: vec![top, bottom],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            required_layers: ManifestLayerRequirements {
                top_mask: false,
                bottom_mask: false,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
                ..ManifestLayerRequirements::default()
            },
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(
            violation_slugs(&violations)
                .contains(&"package:partial-same-coordinates-evidence".to_string())
        );
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("Widget_bottom.gbr"))
        }));
    }

    #[test]
    fn mixed_x2_same_coordinates_identifiers_are_reported() {
        let mut top = x2_layer("Widget_top.gbr", "Copper,L1,Top", "Positive");
        top.same_coordinates = Some("PX1".to_string());
        let mut bottom = x2_layer("Widget_bottom.gbr", "Copper,L2,Bot", "Positive");
        bottom.same_coordinates = Some("PX2".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            required_layers: ManifestLayerRequirements {
                top_mask: false,
                bottom_mask: false,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
                ..ManifestLayerRequirements::default()
            },
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:mixed-same-coordinates-identifiers".to_string()));
        assert!(!slugs.contains(&"package:partial-same-coordinates-evidence".to_string()));
    }

    #[test]
    fn file_function_parser_maps_standard_fabrication_roles() {
        let cases = [
            ("Copper,L1,Top", Some(GerberRole::TopCopper)),
            ("Copper,L4,Bot", Some(GerberRole::BottomCopper)),
            ("Copper,L2,Inr,Plane", Some(GerberRole::InnerCopper)),
            ("Soldermask,Top", Some(GerberRole::TopMask)),
            ("Paste,Bot", Some(GerberRole::BottomPaste)),
            ("Legend,Top", Some(GerberRole::TopSilk)),
            ("Profile,NP", Some(GerberRole::Outline)),
            ("Drillmap", None),
        ];

        for (file_function, expected) in cases {
            assert_eq!(classify_file_function(file_function), expected);
        }
    }

    #[test]
    fn duplicate_role_message_lists_all_conflicting_paths() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("primary-top.gtl"),
                layer("secondary-top-copper.gbr"),
                layer("alt-top-copper.gbr"),
                layer("top_mask.gts"),
                layer("top_mask_backup.gts"),
            ],
            has_board_outline: false,
            has_drill_data: true,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);
        let duplicate = violations
            .iter()
            .find(|violation| {
                violation
                    .layers
                    .iter()
                    .any(|layer| layer == "package:duplicate-top-copper")
            })
            .expect("top copper duplicate warning should be emitted");
        let message = duplicate
            .message
            .as_ref()
            .expect("duplicate-top-copper should include a message");
        let source_paths = input
            .gerber_layers
            .iter()
            .filter_map(|layer| {
                (classify_gerber_role(layer) == GerberRole::TopCopper)
                    .then_some(layer.source_path.as_str())
                    .map(str::to_owned)
            })
            .collect::<Vec<_>>()
            .join(", ");

        assert!(message.contains(&source_paths));
    }

    #[test]
    fn manifest_allows_other_layers_without_manifest_gap_when_non_required_roles_are_unavailable() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("random_a.ly1"),
                layer("random_b.ly2"),
                layer("readme.txt"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        let warnings = violations
            .iter()
            .map(|violation| violation.layers.join(","))
            .collect::<Vec<_>>();

        assert!(
            warnings
                .iter()
                .any(|value| value.contains("package:missing-copper"))
        );
        assert!(
            !warnings
                .iter()
                .any(|value| value.contains("package:missing-board-outline"))
        );
        assert!(
            !warnings
                .iter()
                .any(|value| value.contains("package:missing-drill-data"))
        );
        assert!(
            !warnings
                .iter()
                .any(|value| value.contains("package:missing-top-mask"))
        );
    }

    #[test]
    fn mixed_revision_tags_are_reported_across_layers_and_artifacts() {
        let input = ManifestInput {
            gerber_layers: vec![layer("Widget_revA_F_Cu.gtl"), layer("Widget_revA_B_Cu.gbl")],
            artifact_paths: vec![
                "Widget_revB_bom.csv".to_string(),
                "Widget_revA_centroid.csv".to_string(),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:mixed-revisions".to_string())
        }));
    }

    #[test]
    fn mixed_generated_date_tags_are_reported_across_layers_and_artifacts() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_20260501_F_Cu.gtl"),
                layer("Widget_20260501_B_Cu.gbl"),
            ],
            artifact_paths: vec![
                "Widget_2026-05-01_bom.csv".to_string(),
                "Widget_2026-05-03_centroid.csv".to_string(),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:mixed-generated-dates".to_string())
        }));
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("20260501") && message.contains("20260503"))
        }));
    }

    #[test]
    fn matching_generated_date_tags_do_not_warn() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_20260501_F_Cu.gtl"),
                layer("Widget_2026-05-01_B_Cu.gbl"),
            ],
            artifact_paths: vec!["Widget_2026_05_01_bom.csv".to_string()],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:mixed-generated-dates".to_string()));
    }

    #[test]
    fn impossible_generated_date_tags_are_ignored_as_dates() {
        assert_eq!(normalize_date_token("20260229"), None);
        assert_eq!(
            normalize_date_token("20240229"),
            Some("20240229".to_string())
        );
    }

    #[test]
    fn stale_and_future_generated_dates_are_reported() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_20260101_F_Cu.gtl"),
                layer("Widget_20260514_B_Cu.gbl"),
                layer("Widget_20260513_GKO.gko"),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let mut violations = Vec::new();

        check_generated_date_age(&input, parse_iso_day("2026-05-13"), 90, &mut violations);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:stale-generated-date".to_string()));
        assert!(slugs.contains(&"package:future-generated-date".to_string()));
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("20260101"))
        }));
    }

    #[test]
    fn configured_generated_date_freshness_window_is_applied() {
        let input = ManifestInput {
            gerber_layers: vec![layer("Widget_20260501_F_Cu.gtl")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let mut violations = Vec::new();

        check_generated_date_age(&input, parse_iso_day("2026-05-13"), 7, &mut violations);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:stale-generated-date".to_string()));
    }

    #[test]
    fn x2_creation_dates_are_checked_for_mixed_stale_and_future_outputs() {
        let mut old_top = layer("opaque-top.gbr");
        old_top.creation_date = Some("2026-01-01T08:00:00Z".to_string());
        let mut fresh_bottom = layer("opaque-bottom.gbr");
        fresh_bottom.creation_date = Some("2026-05-14T08:00:00Z".to_string());
        let mut future_outline = layer("opaque-outline.gbr");
        future_outline.creation_date = Some("2026-05-18T08:00:00Z".to_string());
        let input = ManifestInput {
            gerber_layers: vec![old_top, fresh_bottom, future_outline],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let mut violations = Vec::new();

        check_x2_creation_date_consistency(&input, &mut violations);
        check_x2_creation_date_age(&input, parse_iso_day("2026-05-13"), 90, &mut violations);
        let slugs = violation_slugs(&violations);

        assert!(slugs.contains(&"package:mixed-x2-creation-dates".to_string()));
        assert!(slugs.contains(&"package:stale-x2-creation-date".to_string()));
        assert!(slugs.contains(&"package:future-x2-creation-date".to_string()));
    }

    #[test]
    fn x2_part_intent_gaps_are_reported() {
        let mut top = layer("top.gbr");
        top.part = Some("Single".to_string());
        let mut bottom = layer("bottom.gbr");
        bottom.part = Some("Array".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom, layer("coupon.gbr")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-x2-part-evidence".to_string()));
        assert!(slugs.contains(&"package:mixed-x2-parts".to_string()));
    }

    #[test]
    fn x2_part_normalization_allows_consistent_other_values() {
        let mut top = layer("top.gbr");
        top.part = Some("Other, impedance coupon carrier".to_string());
        let mut bottom = layer("bottom.gbr");
        bottom.part = Some("Other,impedance coupon carrier".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:mixed-x2-parts".to_string()));
    }

    #[test]
    fn x2_generation_software_provenance_gaps_are_reported() {
        let mut top = layer("top.gbr");
        top.generation_software = Some("KiCad, KiCad, 9.0".to_string());
        let mut bottom = layer("bottom.gbr");
        bottom.generation_software = Some("KiCad,KiCad,8.0".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom, layer("outline.gbr")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-x2-generation-software-evidence".to_string()));
        assert!(slugs.contains(&"package:mixed-x2-generation-software".to_string()));
    }

    #[test]
    fn x2_project_id_revision_gaps_are_reported() {
        let mut top = layer("top.gbr");
        top.project_id = Some("Widget,550e8400-e29b-41d4-a716-446655440000,A".to_string());
        let mut bottom = layer("bottom.gbr");
        bottom.project_id = Some("Widget,550e8400-e29b-41d4-a716-446655440000,B".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom, layer("outline.gbr")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-x2-project-id-evidence".to_string()));
        assert!(slugs.contains(&"package:mixed-x2-project-ids".to_string()));
    }

    #[test]
    fn x2_md5_integrity_evidence_gaps_are_reported() {
        let mut top = layer("top.gbr");
        top.md5 = Some("d41d8cd98f00b204e9800998ecf8427e".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, layer("bottom.gbr")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-x2-md5-evidence".to_string()));
    }

    #[test]
    fn x2_md5_integrity_evidence_is_clean_when_all_layers_have_checksums() {
        let mut top = layer("top.gbr");
        top.md5 = Some("d41d8cd98f00b204e9800998ecf8427e".to_string());
        let mut bottom = layer("bottom.gbr");
        bottom.md5 = Some("0cc175b9c0f1b6a831c399e269772661".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:partial-x2-md5-evidence".to_string()));
    }

    #[test]
    fn mixed_project_name_tags_are_reported_across_layers_and_artifacts() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_revA_20260501_F_Cu.gtl"),
                layer("Widget_revA_20260501_B_Cu.gbl"),
            ],
            artifact_paths: vec![
                "Gizmo_revA_20260501_bom.csv".to_string(),
                "Widget_revA_20260501_centroid.csv".to_string(),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:mixed-project-names".to_string())
        }));
        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("widget") && message.contains("gizmo"))
        }));
    }

    #[test]
    fn matching_project_name_tags_do_not_warn() {
        let input = ManifestInput {
            gerber_layers: vec![layer("Widget_revA_F_Cu.gtl"), layer("Widget_revA_B_Cu.gbl")],
            artifact_paths: vec![
                "Widget_revA_bom.csv".to_string(),
                "Widget_revA_centroid.csv".to_string(),
            ],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(!slugs.contains(&"package:mixed-project-names".to_string()));
    }

    #[test]
    fn stale_artifact_name_tokens_are_reported_without_matching_solder_substrings() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Widget_revA_TopSolderMask.gts"),
                layer("Widget_revA_TopCopper.gtl"),
            ],
            artifact_paths: vec!["Widget_revA_bom_backup.csv".to_string()],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:stale-artifact-name".to_string())
        }));
        assert!(violations.iter().all(|violation| {
            !violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("TopSolderMask"))
        }));
    }

    #[test]
    fn mixed_gerber_image_setup_is_reported() {
        let mut top = layer("Widget_F_Cu.gtl");
        top.units = Some("millimeters".to_string());
        top.coordinate_format = Some("4:6".to_string());
        let mut bottom = layer("Widget_B_Cu.gbl");
        bottom.units = Some("inches".to_string());
        bottom.coordinate_format = Some("2:6".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, bottom],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:mixed-gerber-units".to_string()));
        assert!(slugs.contains(&"package:mixed-gerber-coordinate-format".to_string()));
    }

    #[test]
    fn partial_gerber_image_setup_evidence_is_reported() {
        let mut top = layer("Widget_F_Cu.gtl");
        top.units = Some("millimeters".to_string());
        top.coordinate_format = Some("4:6".to_string());
        let input = ManifestInput {
            gerber_layers: vec![top, layer("Widget_B_Cu.gbl")],
            has_board_outline: true,
            has_drill_data: true,
            bom_file_count: 1,
            centroid_file_count: 1,
            netlist_file_count: 1,
            fab_drawing_file_count: 1,
            assembly_drawing_file_count: 1,
            readme_file_count: 1,
            rout_drawing_file_count: 1,
            ..Default::default()
        };
        let slugs = violation_slugs(&file_manifest_readiness(&input));

        assert!(slugs.contains(&"package:partial-gerber-unit-evidence".to_string()));
        assert!(slugs.contains(&"package:partial-gerber-coordinate-format-evidence".to_string()));
    }

    fn layer(path: &str) -> ManifestGerberLayer {
        ManifestGerberLayer {
            name: path.to_string(),
            source_path: path.to_string(),
            part: None,
            file_function: None,
            file_polarity: None,
            same_coordinates: None,
            creation_date: None,
            generation_software: None,
            project_id: None,
            md5: None,
            units: None,
            coordinate_format: None,
        }
    }

    fn x2_layer(path: &str, file_function: &str, file_polarity: &str) -> ManifestGerberLayer {
        ManifestGerberLayer {
            name: path.to_string(),
            source_path: path.to_string(),
            part: None,
            file_function: Some(file_function.to_string()),
            file_polarity: Some(file_polarity.to_string()),
            same_coordinates: None,
            creation_date: None,
            generation_software: None,
            project_id: None,
            md5: None,
            units: None,
            coordinate_format: None,
        }
    }

    fn violation_slugs(violations: &[crate::report::Violation]) -> Vec<String> {
        violations
            .iter()
            .flat_map(|violation| violation.layers.iter().cloned())
            .collect()
    }
}
