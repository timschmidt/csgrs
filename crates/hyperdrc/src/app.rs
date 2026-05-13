use std::path::PathBuf;

use anyhow::{Context, Result, anyhow};
use csgrs::io::gerber::FromGerber;

use crate::cli::{Check, Cli, DEFAULT_CHECKS, OutputFormat};
use crate::config::{self, EffectiveRules};
use crate::io::{self, SourceRecord};
use crate::report::{Report, Violation, report_summary, report_to_geojson};
use crate::{LayerMetadata, PcbSketch};
use crate::{checks, conversion, excellon, ipc356, kicad, svg_overlay, waiver};

#[derive(Clone, Debug)]
struct Layer {
    path: PathBuf,
    source: SourceRecord,
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
            min_paste_area_ratio: cli.min_paste_area_ratio,
            max_paste_area_ratio: cli.max_paste_area_ratio,
            min_width: cli.min_width,
            min_mask_width: cli.min_mask_width,
            acid_trap_angle: cli.acid_trap_angle,
            max_copper_imbalance_ratio: cli.max_copper_imbalance_ratio,
            annular_ring: cli.annular_ring,
            drill_clearance: cli.drill_clearance,
            board_thickness: cli.board_thickness,
            max_drill_aspect_ratio: cli.max_drill_aspect_ratio,
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
        && cli.bom_files.is_empty()
        && cli.centroid_files.is_empty()
        && cli.netlist_files.is_empty()
        && cli.fab_drawing_files.is_empty()
        && cli.assembly_drawing_files.is_empty()
        && cli.readme_files.is_empty()
        && cli.rout_drawing_files.is_empty()
    {
        return Err(anyhow!(
            "provide at least one Gerber file, --gerber-dir, --convert-input, --kicad-pcb, --excellon, --ipc356, --bom, --centroid, --netlist, --fab-drawing, --assembly-drawing, --readme, or --rout-drawing input"
        ));
    }

    let conversion_outputs = run_conversions(&cli)?;
    let layers = load_all_layers(&cli.files, &cli.gerber_dirs, &conversion_outputs)?;
    let mut boards = load_boards(&cli.kicad_pcbs)?;
    let excellon_reports = load_excellon_reports(&cli.excellon_files)?;
    let excellon_drills = excellon_reports
        .iter()
        .flat_map(|report| report.drills.iter())
        .cloned()
        .collect::<Vec<_>>();
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
        &excellon_reports,
        &excellon_drills,
        &ipc356_points,
    )?;

    let (active_violations, waived) = waiver::apply_waivers(violations, &waivers);
    let mut violations = active_violations;
    violations.extend(waiver::governance_violations(&waivers));
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
            .chain(cli.bom_files.iter())
            .chain(cli.centroid_files.iter())
            .chain(cli.netlist_files.iter())
            .chain(cli.fab_drawing_files.iter())
            .chain(cli.assembly_drawing_files.iter())
            .chain(cli.readme_files.iter())
            .chain(cli.rout_drawing_files.iter())
            .chain(cli.waiver_files.iter())
            .map(|path| path.display().to_string())
            .collect(),
        inputs: input_manifest(&cli, &layers),
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
    excellon_reports: &[excellon::ExcellonReport],
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
            Check::BoardOutlineCutoutClearance => run_board_outline_cutout_clearance(
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
            Check::BoardOutlineFragments => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_fragments(
                        &layer_name(board),
                        &board.sketch,
                        rules.min_area,
                    ));
                }
                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_fragments(
                            "KiCad Edge.Cuts",
                            outline,
                            rules.min_area,
                        ));
                    }
                }
            }
            Check::BoardOutlineSelfIntersectionReadiness => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_self_intersection_readiness(
                        &layer_name(board),
                        &board.sketch,
                    ));
                }
                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_self_intersection_readiness(
                            "KiCad Edge.Cuts",
                            outline,
                        ));
                    }
                }
            }
            Check::BoardOutlineNotchReadiness => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_notch_readiness(
                        &layer_name(board),
                        &board.sketch,
                    ));
                }
                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_notch_readiness(
                            "KiCad Edge.Cuts",
                            outline,
                        ));
                    }
                }
            }
            Check::BoardOutlineDuplicateReadiness => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_duplicate_readiness(
                        &layer_name(board),
                        &board.sketch,
                    ));
                }
                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_duplicate_readiness(
                            "KiCad Edge.Cuts",
                            outline,
                        ));
                    }
                }
            }
            Check::BoardOutlineNestingReadiness => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    violations.extend(checks::board_outline_nesting_readiness(
                        &layer_name(board),
                        &board.sketch,
                    ));
                }
                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_nesting_readiness(
                            "KiCad Edge.Cuts",
                            outline,
                        ));
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
            Check::PasteApertureRatio => {
                for (paste_index, copper_index) in
                    explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                {
                    let paste = &layers[paste_index];
                    let copper = &layers[copper_index];
                    violations.extend(checks::paste_aperture_ratio(
                        &layer_name(paste),
                        &paste.sketch,
                        &layer_name(copper),
                        &copper.sketch,
                        rules.min_paste_area_ratio,
                        rules.max_paste_area_ratio,
                        rules.min_area,
                    ));
                }
            }
            Check::MinimumPasteAperture => {
                let paste_layers = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                    .into_iter()
                    .map(|(paste_index, _)| paste_index)
                    .collect::<std::collections::BTreeSet<_>>();
                for paste_index in paste_layers {
                    let paste = &layers[paste_index];
                    violations.extend(checks::minimum_paste_aperture(
                        &layer_name(paste),
                        &paste.sketch,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
            }
            Check::PasteApertureSpacing => {
                let paste_layers = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                    .into_iter()
                    .map(|(paste_index, _)| paste_index)
                    .collect::<std::collections::BTreeSet<_>>();
                for paste_index in paste_layers {
                    let paste = &layers[paste_index];
                    violations.extend(checks::paste_aperture_spacing(
                        &layer_name(paste),
                        &paste.sketch,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
            }
            Check::PasteMaskAlignment => {
                let paste_pairs = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?;
                let mask_pairs = explicit_layer_pairs(layers.len(), &cli.mask_pairs)?;
                for (paste_index, paste_copper_index) in paste_pairs {
                    for (mask_copper_index, mask_index) in &mask_pairs {
                        if paste_copper_index != *mask_copper_index {
                            continue;
                        }
                        let paste = &layers[paste_index];
                        let mask = &layers[*mask_index];
                        violations.extend(checks::paste_mask_alignment(
                            &layer_name(paste),
                            &paste.sketch,
                            &layer_name(mask),
                            &mask.sketch,
                            rules.min_area,
                        ));
                    }
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
            Check::SolderMaskExpansion => {
                for (copper_index, mask_index) in
                    explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                {
                    let copper = &layers[copper_index];
                    let mask = &layers[mask_index];
                    violations.extend(checks::solder_mask_expansion(
                        &layer_name(copper),
                        &copper.sketch,
                        &layer_name(mask),
                        &mask.sketch,
                        rules.clearance,
                        rules.min_area,
                    ));
                }
            }
            Check::SolderMaskOverlapClearance => {
                for (copper_index, mask_index) in
                    explicit_layer_pairs(layers.len(), &cli.mask_pairs)?
                {
                    let copper = &layers[copper_index];
                    let mask = &layers[mask_index];
                    violations.extend(checks::solder_mask_overlap_clearance(
                        &layer_name(copper),
                        &copper.sketch,
                        &layer_name(mask),
                        &mask.sketch,
                        rules.clearance,
                        rules.min_area,
                    ));
                }
            }
            Check::SolderMaskBoardEdgeClearance => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    for mask_index in &cli.mask_layers {
                        let mask = &layers[*mask_index];
                        violations.extend(checks::solder_mask_board_edge_clearance(
                            &layer_name(mask),
                            &mask.sketch,
                            &layer_name(board),
                            &board.sketch,
                            rules.clearance,
                            rules.min_area,
                        ));
                    }
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
            Check::SilkscreenClearance => {
                for (silk_index, blocker_index) in
                    explicit_layer_pairs(layers.len(), &cli.silk_pairs)?
                {
                    let silk = &layers[silk_index];
                    let blocker = &layers[blocker_index];
                    violations.extend(checks::silkscreen_clearance(
                        &layer_name(silk),
                        &silk.sketch,
                        &layer_name(blocker),
                        &blocker.sketch,
                        rules.clearance,
                        rules.min_area,
                    ));
                }
            }
            Check::SilkscreenBoardEdgeClearance => {
                if let Some(board_index) = cli.board_outline {
                    let board = &layers[board_index];
                    for silk_index in &cli.silk_layers {
                        let silk = &layers[*silk_index];
                        violations.extend(checks::silkscreen_board_edge_clearance(
                            &layer_name(silk),
                            &silk.sketch,
                            &layer_name(board),
                            &board.sketch,
                            rules.clearance,
                            rules.min_area,
                        ));
                    }
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
            Check::CopperBalance => {
                if !cli.copper_layers.is_empty() {
                    let gerber_copper = cli
                        .copper_layers
                        .iter()
                        .map(|index| {
                            let layer = &layers[*index];
                            (layer_name(layer), layer.sketch.clone())
                        })
                        .collect::<Vec<_>>();
                    violations.extend(checks::copper_balance(
                        &gerber_copper,
                        rules.max_copper_imbalance_ratio,
                        rules.min_area,
                    ));
                }
                for board in boards {
                    let kicad_copper = board
                        .copper_layers(kicad_copper_layers)
                        .into_iter()
                        .map(|(layer_name, copper)| {
                            (format!("{}:{layer_name}", board.source), copper)
                        })
                        .collect::<Vec<_>>();
                    violations.extend(checks::copper_balance(
                        &kicad_copper,
                        rules.max_copper_imbalance_ratio,
                        rules.min_area,
                    ));
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
            Check::MinimumMaskOpening => {
                for mask_index in &cli.mask_layers {
                    let mask = &layers[*mask_index];
                    violations.extend(checks::minimum_mask_opening(
                        &layer_name(mask),
                        &mask.sketch,
                        rules.min_mask_width,
                        rules.min_area,
                    ));
                }
            }
            Check::SolderMaskOpeningSpacing => {
                for mask_index in &cli.mask_layers {
                    let mask = &layers[*mask_index];
                    violations.extend(checks::solder_mask_opening_spacing(
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
            Check::AnnularRingTolerance => {
                for board in boards {
                    violations.extend(checks::annular_ring_tolerance(
                        board,
                        rules.annular_ring,
                        rules.registration_tolerance,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::PlatingIntent => {
                for board in boards {
                    violations.extend(checks::plating_intent(
                        board,
                        kicad_copper_layers,
                        rules.ipc356_tolerance,
                    ));
                }
            }
            Check::RoutedSlotReadiness => {
                for board in boards {
                    violations.extend(checks::routed_slot_readiness(board, rules.min_width));
                }
            }
            Check::CastellationIntent => {
                for board in boards {
                    violations.extend(checks::castellation_intent(board, rules.min_area));
                }
            }
            Check::CastellationHoleReadiness => {
                for board in boards {
                    violations.extend(checks::castellation_hole_readiness(
                        board,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
            }
            Check::ViaInPadReadiness => {
                for board in boards {
                    violations.extend(checks::via_in_pad_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_area,
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
            Check::BoardOutlineDrillClearance => {
                let board_drills: Vec<_> = boards
                    .iter()
                    .flat_map(|board| board.drills.iter().cloned())
                    .collect();
                let has_board_drills = !board_drills.is_empty();
                let has_excellon_drills = !excellon_drills.is_empty();

                if let Some(board_index) = cli.board_outline {
                    let outline = &layers[board_index];
                    if has_board_drills || has_excellon_drills {
                        let drill_source = if has_board_drills {
                            if has_excellon_drills {
                                "KiCad + Excellon drills"
                            } else {
                                "KiCad"
                            }
                        } else {
                            "Excellon drills"
                        };

                        violations.extend(checks::board_outline_drill_clearance(
                            drill_source,
                            &layer_name(outline),
                            &outline.sketch,
                            &board_drills,
                            excellon_drills,
                            rules.drill_clearance,
                            rules.min_area,
                        ));
                    }
                }

                for board in boards {
                    if let Some(outline) = &board.board_outline {
                        violations.extend(checks::board_outline_drill_clearance(
                            &format!("{} drills", board.source),
                            "KiCad Edge.Cuts",
                            outline,
                            &board.drills,
                            excellon_drills,
                            rules.drill_clearance,
                            rules.min_area,
                        ));
                    }
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
            Check::DrillAspectRatio => {
                for board in boards {
                    violations.extend(checks::drill_aspect_ratio(
                        &format!("{} drills", board.source),
                        &board.drills,
                        rules.board_thickness,
                        rules.max_drill_aspect_ratio,
                    ));
                }
                if !excellon_drills.is_empty() {
                    violations.extend(checks::drill_aspect_ratio(
                        "Excellon drills",
                        excellon_drills,
                        rules.board_thickness,
                        rules.max_drill_aspect_ratio,
                    ));
                }
            }
            Check::DrillTableConsistency => {
                if boards.is_empty() {
                    violations.extend(checks::drill_table_consistency(
                        &[],
                        excellon_drills,
                        ipc356_points,
                        rules.ipc356_tolerance,
                    ));
                } else {
                    for board in boards {
                        violations.extend(checks::drill_table_consistency(
                            &board.drills,
                            excellon_drills,
                            ipc356_points,
                            rules.ipc356_tolerance,
                        ));
                    }
                }
            }
            Check::CopperWidthReadiness => {
                for board in boards {
                    violations.extend(checks::copper_width_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_width,
                    ));
                }
            }
            Check::CopperNetIntent => {
                for board in boards {
                    violations.extend(checks::copper_net_intent(board, kicad_copper_layers));
                }
            }
            Check::TeardropReadiness => {
                for board in boards {
                    violations.extend(checks::teardrop_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_width,
                        rules.min_area,
                    ));
                }
            }
            Check::ThermalReliefReadiness => {
                for board in boards {
                    violations.extend(checks::thermal_relief_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::PlaneClearanceReadiness => {
                for board in boards {
                    violations.extend(checks::plane_clearance_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::BoardEdgeExposure => {
                for board in boards {
                    violations.extend(checks::board_edge_exposure(
                        board,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::HighSpeedEdgeReadiness => {
                for board in boards {
                    violations.extend(checks::high_speed_edge_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 2.0,
                        rules.min_area,
                    ));
                }
            }
            Check::EdgeCopperPullbackReadiness => {
                for board in boards {
                    violations.extend(checks::edge_copper_pullback_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance,
                        rules.min_area,
                    ));
                }
            }
            Check::HighVoltageEdgeReadiness => {
                for board in boards {
                    violations.extend(checks::high_voltage_edge_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 4.0,
                        rules.min_area,
                    ));
                }
            }
            Check::ControlledImpedanceReadiness => {
                for board in boards {
                    violations.extend(checks::controlled_impedance_readiness(
                        board,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::DifferentialPairReadiness => {
                for board in boards {
                    violations.extend(checks::differential_pair_readiness(
                        board,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::DifferentialPairSpacingReadiness => {
                for board in boards {
                    violations.extend(checks::differential_pair_spacing_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 4.0,
                    ));
                }
            }
            Check::DifferentialPairViaSymmetryReadiness => {
                for board in boards {
                    violations.extend(checks::differential_pair_via_symmetry_readiness(
                        board,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::ReferencePlaneReadiness => {
                for board in boards {
                    violations.extend(checks::reference_plane_readiness(
                        board,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::ReferencePlaneVoidReadiness => {
                for board in boards {
                    violations.extend(checks::reference_plane_void_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::OrphanedZoneReadiness => {
                for board in boards {
                    violations.extend(checks::orphaned_zone_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance,
                    ));
                }
            }
            Check::SameNetIslandReadiness => {
                for board in boards {
                    violations.extend(checks::same_net_island_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance,
                    ));
                }
            }
            Check::ReturnPathReadiness => {
                for board in boards {
                    violations.extend(checks::return_path_readiness(
                        board,
                        rules.net_clearance,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::HighCurrentReadiness => {
                for board in boards {
                    violations.extend(checks::high_current_readiness(board, kicad_copper_layers));
                }
            }
            Check::PowerViaArrayReadiness => {
                for board in boards {
                    violations.extend(checks::power_via_array_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 4.0,
                    ));
                }
            }
            Check::ThermalViaReadiness => {
                for board in boards {
                    violations.extend(checks::thermal_via_readiness(
                        board,
                        kicad_copper_layers,
                        2,
                        rules.net_clearance,
                    ));
                }
            }
            Check::PowerPlaneReadiness => {
                for board in boards {
                    violations.extend(checks::power_plane_readiness(board, kicad_copper_layers));
                }
            }
            Check::HighCurrentNeckReadiness => {
                for board in boards {
                    violations.extend(checks::high_current_neck_readiness(
                        board,
                        kicad_copper_layers,
                        rules.min_width * 2.0,
                    ));
                }
            }
            Check::VoltageClearanceReadiness => {
                for board in boards {
                    violations.extend(checks::voltage_clearance_readiness(
                        board,
                        rules.net_clearance * 2.0,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::SensitiveNetSpacingReadiness => {
                for board in boards {
                    violations.extend(checks::sensitive_net_spacing_readiness(
                        board,
                        rules.net_clearance * 2.0,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::SensitiveReturnReadiness => {
                for board in boards {
                    violations.extend(checks::sensitive_return_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 2.0,
                    ));
                }
            }
            Check::RfKeepoutReadiness => {
                for board in boards {
                    violations.extend(checks::rf_keepout_readiness(
                        board,
                        rules.net_clearance * 4.0,
                        kicad_copper_layers,
                        rules.min_area,
                    ));
                }
            }
            Check::ChassisStitchingReadiness => {
                for board in boards {
                    violations.extend(checks::chassis_stitching_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance * 4.0,
                    ));
                }
            }
            Check::EdgeStitchingReadiness => {
                for board in boards {
                    violations.extend(checks::edge_stitching_readiness(
                        board,
                        kicad_copper_layers,
                        rules.net_clearance,
                        rules.net_clearance * 6.0,
                        rules.min_area,
                    ));
                }
            }
            Check::GoldFingerReadiness => {
                for board in boards {
                    violations.extend(checks::gold_finger_readiness(board, kicad_copper_layers));
                }
            }
            Check::TestpointCoverageReadiness => {
                for board in boards {
                    violations.extend(checks::testpoint_coverage_readiness(
                        board,
                        ipc356_points,
                        kicad_copper_layers,
                    ));
                }
            }
            Check::FiducialReadiness => {
                for board in boards {
                    violations.extend(checks::fiducial_readiness(
                        board,
                        kicad_copper_layers,
                        rules.clearance * 2.0,
                    ));
                }
            }
            Check::DensePadEscapeReadiness => {
                for board in boards {
                    violations.extend(checks::dense_pad_escape_readiness(
                        board,
                        kicad_copper_layers,
                        0.8,
                        rules.net_clearance * 10.0,
                    ));
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
            Check::FileManifestReadiness => {
                violations.extend(checks::file_manifest_readiness(&manifest_input(
                    cli, layers, boards,
                )));
            }
            Check::ExcellonReadiness => {
                violations.extend(checks::excellon_batch_readiness(excellon_reports));
            }
            Check::ProductionArtifactReadiness => {
                let bom_files = load_text_artifacts(&cli.bom_files)?;
                let centroid_files = load_text_artifacts(&cli.centroid_files)?;
                let netlist_files = load_text_artifacts(&cli.netlist_files)?;
                let readme_files = load_text_artifacts(&cli.readme_files)?;
                let fab_drawing_files = load_file_artifacts(&cli.fab_drawing_files)?;
                let assembly_drawing_files = load_file_artifacts(&cli.assembly_drawing_files)?;
                let rout_drawing_files = load_file_artifacts(&cli.rout_drawing_files)?;
                violations.extend(checks::production_artifact_readiness(
                    &bom_files,
                    &centroid_files,
                    &netlist_files,
                    &readme_files,
                    &fab_drawing_files,
                    &assembly_drawing_files,
                    &rout_drawing_files,
                ));
            }
        }
    }

    Ok(violations)
}

fn load_text_artifacts(files: &[PathBuf]) -> Result<Vec<checks::TextArtifact>> {
    files
        .iter()
        .map(|path| {
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("failed to read {}", path.display()))?;
            Ok(checks::TextArtifact {
                path: path.display().to_string(),
                text,
            })
        })
        .collect()
}

fn load_file_artifacts(files: &[PathBuf]) -> Result<Vec<checks::FileArtifact>> {
    files
        .iter()
        .map(|path| {
            let metadata = std::fs::metadata(path)
                .with_context(|| format!("failed to stat {}", path.display()))?;
            Ok(checks::FileArtifact {
                path: path.display().to_string(),
                byte_len: metadata.len(),
            })
        })
        .collect()
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

fn run_board_outline_cutout_clearance(
    violations: &mut Vec<Violation>,
    rules: &EffectiveRules,
    kicad_copper_layers: &[String],
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
) {
    if let Some(board_index) = cli.board_outline {
        let outline = &layers[board_index];
        for copper_index in selected_layers(layers.len(), &cli.copper_layers) {
            if copper_index == board_index {
                continue;
            }
            let subject = &layers[copper_index];
            violations.extend(checks::board_outline_cutout_clearance(
                &layer_name(subject),
                &subject.sketch,
                &layer_name(outline),
                &outline.sketch,
                rules.clearance,
                rules.min_area,
            ));
        }
    }

    for board in boards {
        if let Some(outline) = &board.board_outline {
            for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                violations.extend(checks::board_outline_cutout_clearance(
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

#[cfg(test)]
fn load_layers(files: &[PathBuf]) -> Result<Vec<Layer>> {
    let discovered = files
        .iter()
        .cloned()
        .map(io::direct_gerber_file)
        .collect::<Vec<_>>();
    load_discovered_layers(&discovered)
}

fn load_discovered_layers(files: &[io::DiscoveredFile]) -> Result<Vec<Layer>> {
    files
        .iter()
        .map(|file| {
            let bytes = std::fs::read(&file.path)
                .with_context(|| format!("failed to read {}", file.path.display()))?;
            let name = file
                .path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("layer")
                .to_string();
            let sketch = PcbSketch::from_gerber(&bytes, Some(LayerMetadata { name }))
                .with_context(|| format!("failed to parse Gerber {}", file.path.display()))?;
            Ok(Layer {
                path: file.path.clone(),
                source: file.source.clone(),
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
    let mut layer_files = files
        .iter()
        .cloned()
        .map(io::direct_gerber_file)
        .collect::<Vec<_>>();
    for directory in gerber_dirs {
        layer_files.extend(io::discover_gerber_dir(directory)?);
    }
    for output in conversion_outputs {
        for mut file in io::discover_gerber_dir(&output.gerber_dir)? {
            file.source = io::converted_gerber_file(file.path.clone(), &output.source_dir).source;
            layer_files.push(file);
        }
    }

    load_discovered_layers(&layer_files)
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
                extra_args: cli.conversion_args.clone(),
            };
            conversion::convert(&request)
        })
        .collect()
}

fn load_boards(files: &[PathBuf]) -> Result<Vec<kicad::BoardModel>> {
    files
        .iter()
        .map(|path| kicad::load_kicad_pcb(path))
        .collect()
}

fn input_manifest(cli: &Cli, layers: &[Layer]) -> Vec<SourceRecord> {
    let mut inputs = layers
        .iter()
        .map(|layer| layer.source.clone())
        .collect::<Vec<_>>();
    inputs.extend(cli.kicad_pcbs.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::KiCad,
            io::IoRole::KiCadBoard,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.excellon_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::Excellon,
            io::IoRole::DrillSidecar,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.ipc356_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::Ipc356,
            io::IoRole::NetlistSidecar,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.waiver_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::Waiver,
            io::IoRole::Waiver,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.bom_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::BomFile,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.centroid_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::CentroidFile,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.netlist_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::NetlistFile,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.fab_drawing_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::FabDrawing,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.assembly_drawing_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::AssemblyDrawing,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.readme_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::ReadmeFile,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(cli.rout_drawing_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::DirectFile,
            io::IoRole::RoutDrawingFile,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs
}

fn manifest_input(
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
) -> checks::ManifestInput {
    let mut kicad_copper_layers = std::collections::HashSet::new();
    for board in boards {
        for feature in &board.copper {
            kicad_copper_layers.insert(feature.layer.clone());
        }
    }

    checks::ManifestInput {
        gerber_layers: layers
            .iter()
            .map(|layer| checks::ManifestGerberLayer {
                name: layer_name(layer),
                source_path: layer.source.path.clone(),
            })
            .collect(),
        artifact_paths: cli
            .bom_files
            .iter()
            .chain(cli.centroid_files.iter())
            .chain(cli.netlist_files.iter())
            .chain(cli.fab_drawing_files.iter())
            .chain(cli.assembly_drawing_files.iter())
            .chain(cli.readme_files.iter())
            .chain(cli.rout_drawing_files.iter())
            .chain(cli.kicad_pcbs.iter())
            .chain(cli.excellon_files.iter())
            .chain(cli.ipc356_files.iter())
            .map(|path| path.display().to_string())
            .collect(),
        bom_file_count: cli.bom_files.len(),
        centroid_file_count: cli.centroid_files.len(),
        netlist_file_count: cli.netlist_files.len(),
        fab_drawing_file_count: cli.fab_drawing_files.len(),
        assembly_drawing_file_count: cli.assembly_drawing_files.len(),
        readme_file_count: cli.readme_files.len(),
        rout_drawing_file_count: cli.rout_drawing_files.len(),
        declared_copper_layer_count: cli.declared_copper_layer_count.filter(|count| *count > 0),
        kicad_copper_layer_count: Some(kicad_copper_layers.len()).filter(|count| *count > 0),
        has_board_outline: boards.iter().any(|board| board.board_outline.is_some()),
        has_drill_data: !cli.excellon_files.is_empty()
            || boards.iter().any(|board| !board.drills.is_empty()),
    }
}

#[allow(dead_code)]
fn load_excellon_drills(files: &[PathBuf]) -> Result<Vec<kicad::DrillFeature>> {
    Ok(load_excellon_reports(files)?
        .into_iter()
        .flat_map(|report| report.drills.into_iter())
        .collect::<Vec<_>>())
}

fn load_excellon_reports(files: &[PathBuf]) -> Result<Vec<excellon::ExcellonReport>> {
    let mut reports = Vec::new();
    for path in files {
        reports.push(excellon::load_excellon_report(path)?);
    }
    Ok(reports)
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
    use std::path::{Path, PathBuf};
    use std::{fs, process};

    use clap::Parser;

    use crate::cli::Cli;
    use crate::geometry::empty_sketch;
    use crate::io::{IoAdapter, IoRole, SourceRecord, discover_gerber_dir, is_gerber_path};
    use crate::kicad;

    use super::{
        Layer, explicit_layer_pairs, input_manifest, layer_pairs, load_all_layers, load_boards,
        load_excellon_drills, load_ipc356_points, load_layers, manifest_input, run,
        validate_board_outline_role, validate_layer_index, validate_layer_indexes,
        validate_silk_layer_roles,
    };

    const VALID_GERBER: &str =
        "G04 trace*\n%MOMM*%\n%FSLAX46Y46*%\n%ADD10C,1*%\nD10*\nX0Y0D02*\nX4000000Y0D01*\nM02*\n";

    fn make_layer(path: &str) -> Layer {
        Layer {
            path: PathBuf::from(path),
            source: SourceRecord::new(
                IoAdapter::DirectFile,
                IoRole::GerberLayer,
                path,
                Option::<&Path>::None,
            ),
            sketch: empty_sketch(None),
        }
    }

    fn make_board_model(
        copper_layers: &[&str],
        has_outline: bool,
        has_drill: bool,
    ) -> kicad::BoardModel {
        let mut copper = Vec::new();
        for layer in copper_layers {
            copper.push(kicad::CopperFeature {
                layer: (*layer).to_string(),
                net: None,
                kind: kicad::CopperKind::Pad,
                sketch: empty_sketch(Some(crate::LayerMetadata {
                    name: (*layer).to_string(),
                })),
                location: [0.0, 0.0],
            });
        }

        kicad::BoardModel {
            source: "board.kicad_pcb".to_string(),
            copper,
            drills: if has_drill {
                vec![kicad::DrillFeature {
                    location: [1.0, 1.0],
                    diameter: 0.6,
                    net: None,
                    plated: true,
                }]
            } else {
                Vec::new()
            },
            board_outline: if has_outline {
                Some(empty_sketch(Some(crate::LayerMetadata {
                    name: "Edge.Cuts".to_string(),
                })))
            } else {
                None
            },
            panel_features: None,
        }
    }

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

        let files = discover_gerber_dir(&dir).unwrap();

        assert_eq!(
            files.into_iter().map(|file| file.path).collect::<Vec<_>>(),
            vec![dir.join("a-top.gtl"), dir.join("z-bottom.gbl")]
        );
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

    #[test]
    fn input_manifest_records_readiness_artifacts() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--bom",
            "bom.csv",
            "--centroid",
            "placement.txt",
            "--netlist",
            "netlist.csv",
            "--fab-drawing",
            "fab.pdf",
            "--assembly-drawing",
            "assembly.dxf",
            "--readme",
            "README.md",
            "--rout-drawing",
            "rout.dxf",
            "top.gbr",
        ]);
        let manifest = input_manifest(&cli, &[]);
        let roles = manifest
            .iter()
            .map(|source| source.role.clone())
            .collect::<Vec<_>>();

        assert!(roles.contains(&IoRole::BomFile));
        assert!(roles.contains(&IoRole::CentroidFile));
        assert!(roles.contains(&IoRole::NetlistFile));
        assert!(roles.contains(&IoRole::FabDrawing));
        assert!(roles.contains(&IoRole::AssemblyDrawing));
        assert!(roles.contains(&IoRole::ReadmeFile));
        assert!(roles.contains(&IoRole::RoutDrawingFile));
        assert_eq!(manifest.len(), 7);
    }

    #[test]
    fn manifest_input_collects_declared_counts_and_kicad_metadata() {
        let cli = Cli::parse_from([
            "hyperdrc",
            "--declared-copper-layer-count",
            "4",
            "--bom",
            "bom.csv",
            "--centroid",
            "centroid.csv",
            "--netlist",
            "netlist.csv",
            "--fab-drawing",
            "fab.dxf",
            "--assembly-drawing",
            "assembly.dxf",
            "--readme",
            "README.md",
            "--rout-drawing",
            "rout.dxf",
            "top.gbr",
            "bottom.gbr",
        ]);

        let layers = vec![make_layer("top.gbr"), make_layer("bottom.gbr")];
        let boards = vec![make_board_model(&["F.Cu", "B.Cu", "F.Cu"], true, true)];
        let manifest = manifest_input(&cli, &layers, &boards);

        assert_eq!(manifest.gerber_layers.len(), 2);
        assert!(manifest.artifact_paths.contains(&"bom.csv".to_string()));
        assert!(manifest.artifact_paths.contains(&"rout.dxf".to_string()));
        assert_eq!(manifest.bom_file_count, 1);
        assert_eq!(manifest.centroid_file_count, 1);
        assert_eq!(manifest.netlist_file_count, 1);
        assert_eq!(manifest.fab_drawing_file_count, 1);
        assert_eq!(manifest.assembly_drawing_file_count, 1);
        assert_eq!(manifest.readme_file_count, 1);
        assert_eq!(manifest.rout_drawing_file_count, 1);
        assert_eq!(manifest.declared_copper_layer_count, Some(4));
        assert_eq!(manifest.kicad_copper_layer_count, Some(2));
        assert!(manifest.has_board_outline);
        assert!(manifest.has_drill_data);
    }

    #[test]
    fn manifest_input_discards_non_positive_optional_counts_when_no_readiness_context() {
        let cli = Cli::parse_from(["hyperdrc", "--declared-copper-layer-count", "0", "top.gbr"]);
        let layers = vec![make_layer("top.gbr")];
        let boards = vec![make_board_model(&[], false, false)];
        let manifest = manifest_input(&cli, &layers, &boards);

        assert_eq!(manifest.gerber_layers.len(), 1);
        assert_eq!(manifest.declared_copper_layer_count, None);
        assert_eq!(manifest.kicad_copper_layer_count, None);
        assert!(!manifest.has_board_outline);
        assert!(!manifest.has_drill_data);
    }

    #[test]
    fn load_all_layers_discovers_all_input_channels_and_marks_conversion_origin() {
        let process_id = process::id();
        let workspace = PathBuf::from(format!("/tmp/hyperdrc-load-all-layers-{process_id}"));
        let direct_path = workspace.join("direct.gbr");
        let gerber_dir = workspace.join("in");
        let conversion_source_dir = workspace.join("conversion-source");
        let conversion_gerber_dir = workspace.join("conversion-output");
        let _ = fs::remove_dir_all(&workspace);
        fs::create_dir_all(&workspace).unwrap();
        fs::create_dir_all(&gerber_dir).unwrap();
        fs::create_dir_all(&conversion_source_dir).unwrap();
        fs::create_dir_all(&conversion_gerber_dir).unwrap();

        fs::write(&direct_path, VALID_GERBER).unwrap();
        fs::write(gerber_dir.join("dir-layer.gbr"), VALID_GERBER).unwrap();
        fs::write(
            conversion_source_dir.join("source.txt"),
            "conversion input placeholder",
        )
        .unwrap();
        fs::write(conversion_gerber_dir.join("conv.gbr"), VALID_GERBER).unwrap();

        let conversion_output = crate::conversion::ConversionOutput {
            source_dir: conversion_source_dir.clone(),
            gerber_dir: conversion_gerber_dir.clone(),
        };

        let layers = load_all_layers(
            std::slice::from_ref(&direct_path),
            std::slice::from_ref(&gerber_dir),
            &[conversion_output],
        )
        .unwrap();

        assert_eq!(layers.len(), 3);
        assert_eq!(layers[0].path, direct_path);
        assert_eq!(layers[1].path, gerber_dir.join("dir-layer.gbr"));
        assert_eq!(layers[2].path, conversion_gerber_dir.join("conv.gbr"));
        assert_eq!(layers[0].source.adapter, IoAdapter::DirectFile);
        assert_eq!(layers[1].source.adapter, IoAdapter::GerberDirectory);
        assert_eq!(layers[2].source.adapter, IoAdapter::Conversion);
        assert_eq!(
            layers[2].source.origin.as_deref(),
            Some(conversion_source_dir.to_str().unwrap())
        );

        let _ = fs::remove_dir_all(workspace);
    }

    #[test]
    fn load_all_layers_wraps_parse_errors_with_file_path_context() {
        let process_id = process::id();
        let workspace = PathBuf::from(format!("/tmp/hyperdrc-load-all-layers-parse-{process_id}"));
        let _ = fs::remove_dir_all(&workspace);
        fs::create_dir_all(&workspace).unwrap();
        let invalid_file = workspace.join("invalid.gbr");
        fs::write(&invalid_file, "this is not a gerber file\n").unwrap();

        let error = load_all_layers(std::slice::from_ref(&invalid_file), &[], &[]).unwrap_err();

        let message = format!("{error}");
        assert!(message.contains("failed to parse Gerber"));
        assert!(message.contains("invalid.gbr"));

        let _ = fs::remove_dir_all(workspace);
    }
}
