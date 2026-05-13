//! Package-level readiness checks over the discovered input manifest.
//!
//! Geometry checks can prove local design-rule problems, but pre-production
//! review also needs the uploaded file set to be coherent. This module catches
//! missing or duplicated manufacturing deliverables using conservative filename
//! role inference.

use crate::report::{Severity, Violation};

#[derive(Clone, Debug)]
pub struct ManifestGerberLayer {
    pub name: String,
    pub source_path: String,
}

#[derive(Clone, Debug, Default)]
pub struct ManifestInput {
    pub gerber_layers: Vec<ManifestGerberLayer>,
    pub has_board_outline: bool,
    pub has_drill_data: bool,
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
    if input.gerber_layers.is_empty() {
        return Vec::new();
    }

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
    let mut violations = Vec::new();

    if copper_count == 0 {
        violations.push(package_violation(
            "missing-copper",
            Severity::Error,
            "Gerber package does not contain any recognizable copper layer",
        ));
    }

    if outline_count == 0 && !input.has_board_outline {
        violations.push(package_violation(
            "missing-board-outline",
            Severity::Warning,
            "Gerber package has no recognizable board outline/profile layer",
        ));
    }

    if !input.has_drill_data {
        violations.push(package_violation(
            "missing-drill-data",
            Severity::Warning,
            "input package has no Excellon or KiCad drill data",
        ));
    }

    if top_copper > 0 && role_count(&classified, GerberRole::TopMask) == 0 {
        violations.push(package_violation(
            "missing-top-mask",
            Severity::Warning,
            "top copper is present but no top solder mask opening layer was recognized",
        ));
    }

    if bottom_copper > 0 && role_count(&classified, GerberRole::BottomMask) == 0 {
        violations.push(package_violation(
            "missing-bottom-mask",
            Severity::Warning,
            "bottom copper is present but no bottom solder mask opening layer was recognized",
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
        GerberRole::Outline,
        "duplicate-outline",
        &mut violations,
    );

    violations
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
    use super::{ManifestGerberLayer, ManifestInput, file_manifest_readiness};

    #[test]
    fn complete_two_layer_gerber_package_has_no_manifest_warnings() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("Gerber_TopCopperLayer.GTL"),
                layer("Gerber_BottomCopperLayer.GBL"),
                layer("Gerber_TopSolderMaskLayer.GTS"),
                layer("Gerber_BottomSolderMaskLayer.GBS"),
                layer("Gerber_BoardOutlineLayer.GKO"),
            ],
            has_board_outline: false,
            has_drill_data: true,
        };

        assert!(file_manifest_readiness(&input).is_empty());
    }

    #[test]
    fn missing_production_deliverables_are_reported_independently() {
        let input = ManifestInput {
            gerber_layers: vec![layer("board-top.gtl")],
            has_board_outline: false,
            has_drill_data: false,
        };

        let violations = file_manifest_readiness(&input);
        let layers = violations
            .iter()
            .flat_map(|violation| violation.layers.clone())
            .collect::<Vec<_>>();

        assert!(layers.contains(&"package:missing-board-outline".to_string()));
        assert!(layers.contains(&"package:missing-drill-data".to_string()));
        assert!(layers.contains(&"package:missing-top-mask".to_string()));
    }

    #[test]
    fn duplicate_role_detection_lists_conflicting_files() {
        let input = ManifestInput {
            gerber_layers: vec![
                layer("first.gtl"),
                layer("second-top-copper.gbr"),
                layer("outline.gko"),
                layer("top-mask.gts"),
            ],
            has_board_outline: false,
            has_drill_data: true,
        };

        let violations = file_manifest_readiness(&input);

        assert!(violations.iter().any(|violation| {
            violation
                .layers
                .contains(&"package:duplicate-top-copper".to_string())
        }));
    }

    fn layer(path: &str) -> ManifestGerberLayer {
        ManifestGerberLayer {
            name: path.to_string(),
            source_path: path.to_string(),
        }
    }
}
