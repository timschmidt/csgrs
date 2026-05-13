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
            Check::GoldFingerReadiness => {
                for board in boards {
                    violations.extend(checks::gold_finger_readiness(board, kicad_copper_layers));
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
    inputs
}

fn manifest_input(
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
) -> checks::ManifestInput {
    checks::ManifestInput {
        gerber_layers: layers
            .iter()
            .map(|layer| checks::ManifestGerberLayer {
                name: layer_name(layer),
                source_path: layer.source.path.clone(),
            })
            .collect(),
        has_board_outline: boards.iter().any(|board| board.board_outline.is_some()),
        has_drill_data: !cli.excellon_files.is_empty()
            || boards.iter().any(|board| !board.drills.is_empty()),
    }
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
    use crate::io::{discover_gerber_dir, is_gerber_path};

    use super::{
        explicit_layer_pairs, layer_pairs, load_boards, load_excellon_drills, load_ipc356_points,
        load_layers, run, validate_board_outline_role, validate_layer_index,
        validate_layer_indexes, validate_silk_layer_roles,
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
}
