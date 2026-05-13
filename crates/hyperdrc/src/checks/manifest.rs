//! Package-level readiness checks over the discovered input manifest.
//!
//! Geometry checks can prove local design-rule problems, but pre-production
//! review also needs the uploaded file set to be coherent. This module catches
//! missing or duplicated manufacturing deliverables using conservative filename
//! role inference. The goal is to catch file-set mismatches before geometry and
//! electrical checks begin.

use crate::date::{current_day_number, parse_compact_day};
use crate::report::{Severity, Violation};

const DEFAULT_GENERATED_DATE_STALE_DAYS: i64 = 90;

#[derive(Clone, Debug)]
pub struct ManifestGerberLayer {
    pub name: String,
    pub source_path: String,
}

#[derive(Clone, Debug, Default)]
pub struct ManifestInput {
    pub gerber_layers: Vec<ManifestGerberLayer>,
    pub artifact_paths: Vec<String>,
    pub bom_file_count: usize,
    pub centroid_file_count: usize,
    pub netlist_file_count: usize,
    pub fab_drawing_file_count: usize,
    pub assembly_drawing_file_count: usize,
    pub readme_file_count: usize,
    pub rout_drawing_file_count: usize,
    pub required_artifacts: ManifestRequirements,
    pub required_layers: ManifestLayerRequirements,
    pub declared_copper_layer_count: Option<usize>,
    pub generated_date_stale_days: Option<usize>,
    pub kicad_copper_layer_count: Option<usize>,
    pub has_board_outline: bool,
    pub has_drill_data: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ManifestRequirements {
    pub bom: bool,
    pub centroid: bool,
    pub netlist: bool,
    pub fab_drawing: bool,
    pub assembly_drawing: bool,
    pub readme: bool,
    pub rout_drawing: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ManifestLayerRequirements {
    pub board_outline: bool,
    pub drill_data: bool,
    pub top_mask: bool,
    pub bottom_mask: bool,
    pub top_paste: bool,
    pub bottom_paste: bool,
    pub top_silkscreen: bool,
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
    let mut violations = Vec::new();

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
    check_revision_consistency(input, &mut violations);
    check_generated_date_consistency(input, &mut violations);
    check_generated_date_age(
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
        ManifestRequirements, check_generated_date_age, classify_gerber_role,
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

        assert!(file_manifest_readiness(&input).is_empty());
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

        assert!(file_manifest_readiness(&input).is_empty());
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

    fn layer(path: &str) -> ManifestGerberLayer {
        ManifestGerberLayer {
            name: path.to_string(),
            source_path: path.to_string(),
        }
    }

    fn violation_slugs(violations: &[crate::report::Violation]) -> Vec<String> {
        violations
            .iter()
            .flat_map(|violation| violation.layers.iter().cloned())
            .collect()
    }
}
