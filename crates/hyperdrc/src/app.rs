use std::path::PathBuf;

use anyhow::{Context, Result, anyhow};
use csgrs::io::gerber::FromGerber;

use crate::cli::{Check, Cli, DEFAULT_CHECKS, OutputFormat};
use crate::config::{self, EffectiveRules};
use crate::report::{Report, Violation, report_summary, report_to_geojson};
use crate::{LayerMetadata, PcbSketch};
use crate::{checks, conversion, excellon, ipc356, kicad, svg_overlay, waiver};

#[derive(Clone, Debug)]
struct Layer {
    path: PathBuf,
    sketch: PcbSketch,
}

pub fn run(cli: Cli) -> Result<()> {
    let config = match &cli.config {
        Some(path) => config::RuleConfig::load(path)?,
        None => config::RuleConfig::default(),
    };
    let rules = config::effective_rules(
        &config,
        config::RuleOverrides {
            keepout: cli.keepout,
            clearance: cli.clearance,
            paste_tolerance: cli.paste_tolerance,
            min_width: cli.min_width,
            min_mask_width: cli.min_mask_width,
            acid_trap_angle: cli.acid_trap_angle,
            annular_ring: cli.annular_ring,
            drill_clearance: cli.drill_clearance,
            net_clearance: cli.net_clearance,
            registration_tolerance: cli.registration_tolerance,
            panel_clearance: cli.panel_clearance,
            ipc356_tolerance: cli.ipc356_tolerance,
            min_area: cli.min_area,
            max_layer_area: cli.max_layer_area,
        },
    );
    let kicad_copper_layers = if cli.kicad_copper_layers.is_empty() {
        config.kicad_copper_layers.clone()
    } else {
        cli.kicad_copper_layers.clone()
    };

    if cli.files.is_empty()
        && cli.gerber_dirs.is_empty()
        && cli.conversion_inputs.is_empty()
        && cli.kicad_pcbs.is_empty()
        && cli.excellon_files.is_empty()
        && cli.ipc356_files.is_empty()
    {
        return Err(anyhow!(
            "provide at least one Gerber file, --gerber-dir, --convert-input, --kicad-pcb, --excellon, or --ipc356 input"
        ));
    }

    let conversion_outputs = run_conversions(&cli)?;
    let layers = load_all_layers(&cli.files, &cli.gerber_dirs, &conversion_outputs)?;
    let mut boards = load_boards(&cli.kicad_pcbs)?;
    let excellon_drills = load_excellon_drills(&cli.excellon_files)?;
    let ipc356_points = load_ipc356_points(&cli.ipc356_files)?;
    let waivers = load_waivers(&cli.waiver_files)?;

    for board in &mut boards {
        checks::apply_ipc356_nets(board, &ipc356_points, rules.ipc356_tolerance);
    }

    if cli.list_kicad_layers {
        for board in &boards {
            eprintln!("{}: {}", board.source, checks::layer_names_csv(board));
        }
    }

    validate_layer_index(layers.len(), cli.board_outline, "--board-outline")?;
    validate_layer_indexes(layers.len(), &cli.copper_layers, "--copper-layer")?;
    validate_layer_indexes(layers.len(), &cli.mask_layers, "--mask-layer")?;
    validate_layer_indexes(layers.len(), &cli.silk_layers, "--silk-layer")?;
    validate_board_outline_role(
        cli.board_outline,
        &cli.copper_layers,
        &cli.mask_layers,
        &cli.silk_layers,
    )?;
    validate_silk_layer_roles(layers.len(), &cli.silk_layers)?;

    let checks = if cli.checks.is_empty() {
        DEFAULT_CHECKS.to_vec()
    } else {
        cli.checks.clone()
    };
    let violations = run_checks(
        &checks,
        &rules,
        &kicad_copper_layers,
        &cli,
        &layers,
        &boards,
        &excellon_drills,
        &ipc356_points,
    )?;

    let (violations, waived) = waiver::apply_waivers(violations, &waivers);
    let summary = report_summary(&violations, waived.len());

    if let Some(summary_file) = &cli.summary_file {
        std::fs::write(summary_file, serde_json::to_vec_pretty(&summary)?)
            .with_context(|| format!("failed to write {}", summary_file.display()))?;
    }

    let report = Report {
        files: cli
            .files
            .iter()
            .chain(cli.gerber_dirs.iter())
            .chain(cli.conversion_inputs.iter())
            .chain(cli.kicad_pcbs.iter())
            .chain(cli.excellon_files.iter())
            .chain(cli.ipc356_files.iter())
            .chain(cli.waiver_files.iter())
            .map(|path| path.display().to_string())
            .collect(),
        violation_count: violations.len(),
        waived_count: waived.len(),
        summary,
        violations,
    };

    if let Some(svg_overlay) = &cli.svg_overlay {
        std::fs::write(svg_overlay, svg_overlay::report_to_svg(&report))
            .with_context(|| format!("failed to write {}", svg_overlay.display()))?;
    }

    match cli.format {
        OutputFormat::Text => print_text_report(&report),
        OutputFormat::Json => println!("{}", serde_json::to_string_pretty(&report)?),
        OutputFormat::Geojson => println!(
            "{}",
            serde_json::to_string_pretty(&report_to_geojson(&report))?
        ),
    }

    if report.violation_count > 0 {
        std::process::exit(1);
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn run_checks(
    selected_checks: &[Check],
    rules: &EffectiveRules,
    kicad_copper_layers: &[String],
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
    excellon_drills: &[kicad::DrillFeature],
    ipc356_points: &[ipc356::Ipc356Point],
) -> Result<Vec<Violation>> {
    let mut violations = Vec::new();

    for check in selected_checks {
        match check {
            Check::MaskIslandKeepout => {
                for layer in layers {
                    violations.extend(checks::mask_island_keepout(
                        &layer_name(layer),
                        &layer.sketch,
                        rules.keepout,
                        rules.min_area,
                    ));
                }
            }
            Check::CopperOverlap => {
                let pairs = layer_pairs(layers.len(), &cli.pairs)?;
                for (left, right) in pairs {
                    let left_layer = &layers[left];
                    let right_layer = &layers[right];
                    violations.extend(checks::copper_overlap(
                        &layer_name(left_layer),
                        &left_layer.sketch,
                        &layer_name(right_layer),
                        &right_layer.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::BoardEdgeClearance => run_board_edge_clearance(
                &mut violations,
                rules,
                kicad_copper_layers,
                cli,
                layers,
                boards,
            ),
            Check::BoardOutlineSanity => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_sanity(
                        &layer_name(board),
                        &board.sketch,
                        rules.min_area,
                    ));
                }
                for board in boards {
                    match &board.board_outline {
                        Some(outline) => violations.extend(checks::board_outline_sanity(
                            "KiCad Edge.Cuts",
                            outline,
                            rules.min_area,
                        )),
                        None => violations.push(Violation::new(
                            "board-outline-sanity",
                            crate::report::Severity::Warning,
                            vec![board.source.clone()],
                            None,
                            Vec::new(),
                            Vec::new(),
                            Some("KiCad board has no parsed Edge.Cuts outline".to_string()),
                        )),
                    }
                }
            }
            Check::PasteOverhang => {
                for (paste_index, copper_index) in
                    explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                {
                    let paste = &layers[paste_index];
                    let copper = &layers[copper_index];
                    violations.extend(checks::paste_overhang(
                        &layer_name(paste),
                        &paste.sketch,
                        &layer_name(copper),
                        &copper.sketch,
                        rules.paste_tolerance,
                        rules.min_area,
                    ));
                }
            }
            Check::PasteApertureCoverage => {
                for (paste_index, copper_index) in
                    explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                {
                    let paste = &layers[paste_index];
                    let copper = &layers[copper_index];
                    violations.extend(checks::paste_aperture_coverage(
                        &layer_name(paste),
                        &paste.sketch,
                        &layer_name(copper),
                        &copper.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::ExposedCopper => {
                for (copper_index, mask_index) in
                    explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                {
                    let copper = &layers[copper_index];
                    let mask = &layers[mask_index];
                    violations.extend(checks::exposed_copper(
                        &layer_name(copper),
                        &copper.sketch,
                        &layer_name(mask),
                        &mask.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::SolderMaskOpeningCoverage => {
                for (copper_index, mask_index) in
                    explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                {
                    let copper = &layers[copper_index];
                    let mask = &layers[mask_index];
                    violations.extend(checks::solder_mask_opening_coverage(
                        &layer_name(copper),
                        &copper.sketch,
                        &layer_name(mask),
                        &mask.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::SilkscreenOverlap => {
                for (silk_index, blocker_index) in
                    explicit_layer_pairs(layers.len(), &cli.silk_pairs)?
                {
                    let silk = &layers[silk_index];
                    let blocker = &layers[blocker_index];
                    violations.extend(checks::silkscreen_overlap(
                        &layer_name(silk),
                        &silk.sketch,
                        &layer_name(blocker),
                        &blocker.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::SilkscreenMinWidth => {
                for silk_index in selected_layers(layers.len(), &cli.silk_layers) {
                    let silk = &layers[silk_index];
                    violations.extend(checks::silkscreen_min_width(
                        &layer_name(silk),
                        &silk.sketch,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
            }
            Check::MinCopperNeck => {
                for copper_index in selected_layers(layers.len(), &cli.copper_layers) {
                    let copper = &layers[copper_index];
                    violations.extend(checks::min_copper_neck_width(
                        &layer_name(copper),
                        &copper.sketch,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
                for board in boards {
                    for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                        violations.extend(checks::min_copper_neck_width(
                            &format!("{}:{layer_name}", board.source),
                            &copper,
                            rules.min_width,
                            rules.min_area,
                        ));
                    }
                }
            }
            Check::AcidTrap => {
                for copper_index in selected_layers(layers.len(), &cli.copper_layers) {
                    let copper = &layers[copper_index];
                    violations.extend(checks::acid_trap_candidates(
                        &layer_name(copper),
                        &copper.sketch,
                        rules.acid_trap_angle,
                    ));
                }
                for board in boards {
                    for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                        violations.extend(checks::acid_trap_candidates(
                            &format!("{}:{layer_name}", board.source),
                            &copper,
                            rules.acid_trap_angle,
                        ));
                    }
                }
            }
            Check::LayerSanity => {
                for layer in layers {
                    violations.extend(checks::layer_sanity(
                        &layer_name(layer),
                        &layer.sketch,
                        rules.max_layer_area,
                    ));
                }
                for board in boards {
                    for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                        violations.extend(checks::layer_sanity(
                            &format!("{}:{layer_name}", board.source),
                            &copper,
                            rules.max_layer_area,
                        ));
                    }
                }
            }
            Check::MechanicalLayerGeometry => {
                for layer in layers {
                    violations.extend(checks::mechanical_layer_geometry(
                        &layer_name(layer),
                        &layer.sketch,
                        rules.min_area,
                    ));
                }
            }
            Check::SolderMaskSliver => {
                for mask_index in selected_layers(layers.len(), &cli.mask_layers) {
                    let mask = &layers[mask_index];
                    violations.extend(checks::solder_mask_sliver(
                        &layer_name(mask),
                        &mask.sketch,
                        rules.min_mask_width,
                        rules.min_area,
                    ));
                }
            }
            Check::AnnularRing => {
                for board in boards {
                    violations.extend(checks::annular_ring(
                        board,
                        rules.annular_ring,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::DrillCopperClearance => {
                for board in boards {
                    violations.extend(checks::drill_to_copper_clearance(
                        board,
                        excellon_drills,
                        rules.drill_clearance,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::DrillSpacing => {
                if boards.is_empty() {
                    violations.extend(checks::drill_spacing(
                        &[],
                        excellon_drills,
                        rules.drill_clearance,
                    ));
                } else {
                    for board in boards {
                        violations.extend(checks::drill_spacing(
                            &board.drills,
                            excellon_drills,
                            rules.drill_clearance,
                        ));
                    }
                }
            }
            Check::NetSpacing => {
                for board in boards {
                    violations.extend(checks::net_spacing(
                        board,
                        rules.net_clearance,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::RegistrationTolerance => {
                for board in boards {
                    violations.extend(checks::registration_tolerance(
                        board,
                        rules.registration_tolerance,
                        rules.min_area,
                    ));
                }
            }
            Check::PanelizationClearance => {
                for board in boards {
                    violations.extend(checks::panelization_clearance(
                        board,
                        excellon_drills,
                        rules.panel_clearance,
                        rules.min_area,
                    ));
                }
            }
            Check::Ipc356Coverage => {
                for board in boards {
                    violations.extend(checks::ipc356_coverage(
                        board,
                        ipc356_points,
                        rules.ipc356_tolerance,
                    ));
                }
            }
            Check::Ipc356DrillDiameter => {
                for board in boards {
                    violations.extend(checks::ipc356_drill_diameter(
                        board,
                        ipc356_points,
                        rules.ipc356_tolerance,
                    ));
                }
            }
        }
    }

    Ok(violations)
}

fn run_board_edge_clearance(
    violations: &mut Vec<Violation>,
    rules: &EffectiveRules,
    kicad_copper_layers: &[String],
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
) {
    if let Some(board_index) = cli.board_outline {
        let board = &layers[board_index];
        for copper_index in selected_layers(layers.len(), &cli.copper_layers) {
            if copper_index == board_index {
                continue;
            }
            let copper = &layers[copper_index];
            violations.extend(checks::board_edge_clearance(
                &layer_name(copper),
                &copper.sketch,
                &layer_name(board),
                &board.sketch,
                rules.clearance,
                rules.min_area,
            ));
        }
    }

    for board in boards {
        if let Some(outline) = &board.board_outline {
            for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                violations.extend(checks::board_edge_clearance(
                    &format!("{}:{layer_name}", board.source),
                    &copper,
                    "KiCad Edge.Cuts",
                    outline,
                    rules.clearance,
                    rules.min_area,
                ));
            }
        }
    }
}

fn selected_layers(layer_count: usize, explicit_layers: &[usize]) -> Vec<usize> {
    if explicit_layers.is_empty() {
        return (0..layer_count).collect();
    }

    explicit_layers.to_vec()
}

fn load_layers(files: &[PathBuf]) -> Result<Vec<Layer>> {
    files
        .iter()
        .map(|path| {
            let bytes = std::fs::read(path)
                .with_context(|| format!("failed to read {}", path.display()))?;
            let name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("layer")
                .to_string();
            let sketch = PcbSketch::from_gerber(&bytes, Some(LayerMetadata { name }))
                .with_context(|| format!("failed to parse Gerber {}", path.display()))?;
            Ok(Layer {
                path: path.clone(),
                sketch,
            })
        })
        .collect()
}

fn load_all_layers(
    files: &[PathBuf],
    gerber_dirs: &[PathBuf],
    conversion_outputs: &[conversion::ConversionOutput],
) -> Result<Vec<Layer>> {
    let mut layer_paths = files.to_vec();
    for directory in gerber_dirs {
        layer_paths.extend(gerber_files_in_dir(directory)?);
    }
    for output in conversion_outputs {
        layer_paths.extend(gerber_files_in_dir(&output.gerber_dir)?);
    }

    load_layers(&layer_paths)
}

fn run_conversions(cli: &Cli) -> Result<Vec<conversion::ConversionOutput>> {
    cli.conversion_inputs
        .iter()
        .enumerate()
        .map(|(index, input_dir)| {
            let request = conversion::ConversionRequest {
                backend: cli.converter,
                input_dir: input_dir.clone(),
                output_dir: conversion::default_conversion_output_dir(
                    &cli.conversion_output_dir,
                    index,
                ),
                source_eda: cli.source_eda,
                zip: cli.conversion_zip,
                zip_name: cli.conversion_zip_name.clone(),
                top_color_image: cli.top_color_image.clone(),
                bottom_color_image: cli.bottom_color_image.clone(),
                transjlc_bin: cli.transjlc_bin.clone(),
            };
            conversion::convert(&request)
        })
        .collect()
}

fn gerber_files_in_dir(directory: &PathBuf) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(directory)
        .with_context(|| format!("failed to read Gerber directory {}", directory.display()))?
    {
        let entry =
            entry.with_context(|| format!("failed to read entry in {}", directory.display()))?;
        let path = entry.path();
        if path.is_file() && is_gerber_path(&path) {
            files.push(path);
        }
    }
    files.sort();
    Ok(files)
}

fn is_gerber_path(path: &std::path::Path) -> bool {
    let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
        return false;
    };
    let lower = name.to_ascii_lowercase();
    matches!(
        path.extension()
            .and_then(|extension| extension.to_str())
            .map(str::to_ascii_lowercase)
            .as_deref(),
        Some(
            "gbr"
                | "ger"
                | "gtl"
                | "gbl"
                | "gts"
                | "gbs"
                | "gto"
                | "gbo"
                | "gko"
                | "gm1"
                | "gm2"
                | "gml"
                | "gpb"
                | "gpt"
        )
    ) || lower.starts_with("gerber_")
        || lower.contains("copper")
        || lower.contains("silkscreen")
        || lower.contains("soldermask")
        || lower.contains("solderpaste")
        || lower.contains("outline")
}

fn load_boards(files: &[PathBuf]) -> Result<Vec<kicad::BoardModel>> {
    files
        .iter()
        .map(|path| kicad::load_kicad_pcb(path))
        .collect()
}

fn load_excellon_drills(files: &[PathBuf]) -> Result<Vec<kicad::DrillFeature>> {
    let mut drills = Vec::new();
    for path in files {
        drills.extend(excellon::load_excellon(path)?);
    }
    Ok(drills)
}

fn load_ipc356_points(files: &[PathBuf]) -> Result<Vec<ipc356::Ipc356Point>> {
    let mut points = Vec::new();
    for path in files {
        points.extend(ipc356::load_ipc356(path)?);
    }
    Ok(points)
}

fn load_waivers(files: &[PathBuf]) -> Result<Vec<waiver::Waiver>> {
    let mut waivers = Vec::new();
    for path in files {
        waivers.extend(waiver::load_waivers(path)?);
    }
    Ok(waivers)
}

fn validate_layer_indexes(layer_count: usize, indexes: &[usize], flag: &str) -> Result<()> {
    let mut seen = std::collections::HashSet::new();
    for index in indexes {
        validate_layer_index(layer_count, Some(*index), flag)?;
        if !seen.insert(*index) {
            return Err(anyhow!("{flag} index {index} is listed more than once"));
        }
    }
    Ok(())
}

fn validate_layer_index(layer_count: usize, index: Option<usize>, flag: &str) -> Result<()> {
    if let Some(index) = index
        && index >= layer_count
    {
        return Err(anyhow!(
            "{flag} index {index} is out of range for {layer_count} input file(s)"
        ));
    }
    Ok(())
}

fn validate_silk_layer_roles(layer_count: usize, silk_layers: &[usize]) -> Result<()> {
    if layer_count > 1 && silk_layers.len() == layer_count {
        return Err(anyhow!(
            "--silk-layer marks every Gerber input as silkscreen; check layer role mapping"
        ));
    }
    Ok(())
}

fn validate_board_outline_role(
    board_outline: Option<usize>,
    copper_layers: &[usize],
    mask_layers: &[usize],
    silk_layers: &[usize],
) -> Result<()> {
    let Some(board_outline) = board_outline else {
        return Ok(());
    };

    if copper_layers.contains(&board_outline) {
        return Err(anyhow!(
            "--board-outline index {board_outline} is also listed as --copper-layer"
        ));
    }
    if mask_layers.contains(&board_outline) {
        return Err(anyhow!(
            "--board-outline index {board_outline} is also listed as --mask-layer"
        ));
    }
    if silk_layers.contains(&board_outline) {
        return Err(anyhow!(
            "--board-outline index {board_outline} is also listed as --silk-layer"
        ));
    }

    Ok(())
}

fn explicit_layer_pairs(layer_count: usize, raw_pairs: &[String]) -> Result<Vec<(usize, usize)>> {
    if raw_pairs.is_empty() {
        return Ok(Vec::new());
    }

    layer_pairs(layer_count, raw_pairs)
}

fn layer_pairs(layer_count: usize, raw_pairs: &[String]) -> Result<Vec<(usize, usize)>> {
    if raw_pairs.is_empty() {
        let mut pairs = Vec::new();
        for left in 0..layer_count {
            for right in (left + 1)..layer_count {
                pairs.push((left, right));
            }
        }
        return Ok(pairs);
    }

    let mut seen = std::collections::HashSet::new();
    raw_pairs
        .iter()
        .map(|raw| {
            let (left, right) = raw
                .split_once(':')
                .ok_or_else(|| anyhow!("invalid layer pair '{raw}', expected INDEX:INDEX"))?;
            let left = left.parse::<usize>()?;
            let right = right.parse::<usize>()?;
            if left >= layer_count || right >= layer_count {
                return Err(anyhow!(
                    "layer pair '{raw}' is out of range for {layer_count} input file(s)"
                ));
            }
            if left == right {
                return Err(anyhow!(
                    "layer pair '{raw}' references the same layer twice"
                ));
            }
            if !seen.insert((left, right)) {
                return Err(anyhow!("layer pair '{raw}' is listed more than once"));
            }
            Ok((left, right))
        })
        .collect()
}

fn layer_name(layer: &Layer) -> String {
    if let Some(metadata) = &layer.sketch.metadata {
        return metadata.name.clone();
    }

    layer.path.display().to_string()
}

fn print_text_report(report: &Report) {
    if report.violations.is_empty() {
        if report.waived_count == 0 {
            println!("No violations found.");
        } else {
            println!(
                "No active violations found. {} waived.",
                report.waived_count
            );
        }
        return;
    }

    println!(
        "{} violation(s) found, {} waived:",
        report.violation_count, report.waived_count
    );
    for (index, violation) in report.violations.iter().enumerate() {
        print_violation(index + 1, violation);
    }
}

fn print_violation(index: usize, violation: &Violation) {
    println!(
        "{index}. {} [{:?}] on {}: {} polygon(s), {} point(s), total area {:.9}",
        violation.check,
        violation.severity,
        violation.layers.join(" + "),
        violation.polygons.len(),
        violation.locations.len(),
        violation.total_area
    );
    println!("   id: {}", violation.id);

    if let Some(message) = &violation.message {
        println!("   {message}");
    }

    if let Some(island) = violation.island_index {
        println!("   island: {island}");
    }

    if !violation.locations.is_empty() {
        println!("   locations: {:?}", violation.locations);
    }

    for (poly_index, polygon) in violation.polygons.iter().enumerate() {
        println!(
            "   polygon {poly_index}: exterior {} point(s), {} hole(s), area {:.9}",
            polygon.exterior.len(),
            polygon.holes.len(),
            polygon.area
        );
        println!("      exterior: {:?}", polygon.exterior);
        if !polygon.holes.is_empty() {
            println!("      holes: {:?}", polygon.holes);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use clap::Parser;

    use crate::cli::Cli;

    use super::{
        explicit_layer_pairs, gerber_files_in_dir, is_gerber_path, layer_pairs, load_boards,
        load_excellon_drills, load_ipc356_points, load_layers, run, validate_board_outline_role,
        validate_layer_index, validate_layer_indexes, validate_silk_layer_roles,
    };

    #[test]
    fn layer_pairs_defaults_to_unique_pairs() {
        assert_eq!(layer_pairs(3, &[]).unwrap(), vec![(0, 1), (0, 2), (1, 2)]);
    }

    #[test]
    fn explicit_pairs_do_not_default_to_all_pairs() {
        assert!(explicit_layer_pairs(3, &[]).unwrap().is_empty());
    }

    #[test]
    fn layer_pairs_reject_malformed_or_duplicate_pairs() {
        assert!(layer_pairs(2, &["bad".to_string()]).is_err());
        assert!(layer_pairs(2, &["0:0".to_string()]).is_err());
        assert!(layer_pairs(2, &["0:2".to_string()]).is_err());
        assert!(layer_pairs(2, &["0:1".to_string(), "0:1".to_string()]).is_err());
    }

    #[test]
    fn layer_index_validation_rejects_out_of_range_indexes() {
        assert!(validate_layer_index(2, Some(1), "--layer").is_ok());
        assert!(validate_layer_index(2, Some(2), "--layer").is_err());
        assert!(validate_layer_indexes(2, &[0, 1], "--layer").is_ok());
        assert!(validate_layer_indexes(2, &[0, 2], "--layer").is_err());
        assert!(validate_layer_indexes(2, &[1, 1], "--layer").is_err());
    }

    #[test]
    fn silk_layer_validation_rejects_every_gerber_layer_as_silkscreen() {
        assert!(validate_silk_layer_roles(3, &[0, 1]).is_ok());
        assert!(validate_silk_layer_roles(1, &[0]).is_ok());
        assert!(validate_silk_layer_roles(3, &[0, 1, 2]).is_err());
    }

    #[test]
    fn board_outline_validation_rejects_conflicting_explicit_roles() {
        assert!(validate_board_outline_role(Some(0), &[1], &[2], &[3]).is_ok());
        assert!(validate_board_outline_role(Some(0), &[0], &[], &[]).is_err());
        assert!(validate_board_outline_role(Some(0), &[], &[0], &[]).is_err());
        assert!(validate_board_outline_role(Some(0), &[], &[], &[0]).is_err());
    }

    #[test]
    fn run_rejects_empty_input_set() {
        let cli = Cli::parse_from(["hyperdrc"]);

        let error = run(cli).unwrap_err().to_string();

        assert!(error.contains("provide at least one"));
    }

    #[test]
    fn gerber_directory_loader_discovers_supported_extensions_in_stable_order() {
        let dir = std::env::temp_dir().join(format!("hyperdrc-gerber-dir-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("z-bottom.gbl"), "%").unwrap();
        std::fs::write(dir.join("a-top.gtl"), "%").unwrap();
        std::fs::write(dir.join("notes.txt"), "not gerber").unwrap();

        let files = gerber_files_in_dir(&dir).unwrap();

        assert_eq!(files, vec![dir.join("a-top.gtl"), dir.join("z-bottom.gbl")]);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn gerber_path_detection_covers_jlc_style_names() {
        assert!(is_gerber_path(&PathBuf::from("board.gbr")));
        assert!(is_gerber_path(&PathBuf::from("Gerber_TopCopperLayer.GTL")));
        assert!(is_gerber_path(&PathBuf::from("Fabrication_Outline.GKO")));
        assert!(!is_gerber_path(&PathBuf::from("board.drl")));
        assert!(!is_gerber_path(&PathBuf::from("readme.txt")));
    }

    #[test]
    fn loaders_report_missing_sidecar_files() {
        let missing = PathBuf::from("/tmp/hyperdrc-definitely-missing-input-file");

        assert!(
            load_layers(std::slice::from_ref(&missing))
                .unwrap_err()
                .to_string()
                .contains("failed to read")
        );
        assert!(
            load_boards(std::slice::from_ref(&missing))
                .unwrap_err()
                .to_string()
                .contains("failed to read")
        );
        assert!(
            load_excellon_drills(std::slice::from_ref(&missing))
                .unwrap_err()
                .to_string()
                .contains("failed to read")
        );
        assert!(
            load_ipc356_points(&[missing])
                .unwrap_err()
                .to_string()
                .contains("failed to read")
        );
    }
}
