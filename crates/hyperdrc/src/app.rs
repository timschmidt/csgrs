use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow};
use csgrs::io::gerber::FromGerber;

use crate::cli::{Check, Cli, DEFAULT_CHECKS, OutputFormat};
use crate::config::{self, EffectiveRules};
use crate::io::{self, SourceRecord};
use crate::report::{Diagnostic, Report, Severity, Violation, report_summary, report_to_geojson};
use crate::{LayerMetadata, PcbSketch};
use crate::{
    baseline, checks, conversion, excellon, github_annotations, html_report, ipc356, jsonl, junit,
    kicad, sarif, svg_overlay, waiver,
};

#[derive(Clone, Debug)]
struct Layer {
    path: PathBuf,
    source: SourceRecord,
    sketch: PcbSketch,
}

#[derive(Clone, Debug, Default)]
struct PackageInputs {
    excellon_files: Vec<io::DiscoveredFile>,
    ipc356_files: Vec<io::DiscoveredFile>,
    bom_files: Vec<io::DiscoveredFile>,
    centroid_files: Vec<io::DiscoveredFile>,
    netlist_files: Vec<io::DiscoveredFile>,
    fab_drawing_files: Vec<io::DiscoveredFile>,
    assembly_drawing_files: Vec<io::DiscoveredFile>,
    readme_files: Vec<io::DiscoveredFile>,
    rout_drawing_files: Vec<io::DiscoveredFile>,
}

/// Result of a completed HyperDRC run.
///
/// The reusable library entry point returns this value instead of terminating
/// the process. Command-line callers should use [`run_cli`], which preserves the
/// traditional non-zero exit status when active findings remain.
#[derive(Debug)]
/// Public data model for `RunOutcome`.
pub struct RunOutcome {
    /// Fully rendered report model for downstream tooling.
    pub report: Report,
    /// Wall-clock duration of the run.
    pub elapsed: Duration,
}

/// Run HyperDRC using parsed command-line options and return the report model.
///
/// This function may write requested side artifacts and emit the selected report
/// format to standard output because those behaviors are part of [`Cli`]. It
/// does not call `std::process::exit`, making it suitable for integration tests
/// and embedding.
pub fn run(cli: Cli) -> Result<RunOutcome> {
    let run_started = Instant::now();
    eprintln!("hyperdrc: starting run");

    let run_result: Result<Report> = (|| {
        let config = status_activity("load configuration", || {
            Ok(match &cli.config {
                Some(path) => config::RuleConfig::load(path)?,
                None => config::RuleConfig::default(),
            })
        })?;
        let rules = status_activity("resolve rule configuration", || {
            Ok(config::effective_rules(
                &config,
                config::RuleOverrides {
                    keepout: cli.keepout,
                    clearance: cli.clearance,
                    paste_tolerance: cli.paste_tolerance,
                    min_paste_area_ratio: cli.min_paste_area_ratio,
                    max_paste_area_ratio: cli.max_paste_area_ratio,
                    stencil_thickness: cli.stencil_thickness,
                    min_stencil_area_ratio: cli.min_stencil_area_ratio,
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
                    generated_date_stale_days: cli.generated_date_stale_days,
                },
            ))
        })?;
        let kicad_copper_layers = status_activity("resolve KiCad copper layers", || {
            Ok(if cli.kicad_copper_layers.is_empty() {
                config.kicad_copper_layers.clone()
            } else {
                cli.kicad_copper_layers.clone()
            })
        })?;

        status_activity("validate input selection", || {
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
            Ok(())
        })?;

        let conversion_outputs =
            status_activity("run input conversions", || run_conversions(&cli))?;
        let layers = status_activity("load Gerber layers", || {
            load_all_layers(&cli.files, &cli.gerber_dirs, &conversion_outputs)
        })?;
        let mut boards = status_activity("load KiCad boards", || load_boards(&cli.kicad_pcbs))?;
        let discovered_sidecars = status_activity("discover package sidecars", || {
            io::discover_package_sidecars(&cli.gerber_dirs)
        })?;
        let package_inputs = status_activity("resolve package inputs", || {
            Ok(package_inputs(&cli, discovered_sidecars))
        })?;
        let excellon_reports = status_activity("load Excellon reports", || {
            load_excellon_reports(&package_input_paths(&package_inputs.excellon_files))
        })?;
        let excellon_drills = status_activity("collect Excellon drills", || {
            Ok(excellon_reports
                .iter()
                .flat_map(|report| report.drills.iter())
                .cloned()
                .collect::<Vec<_>>())
        })?;
        let ipc356_reports = status_activity("load IPC-D-356 reports", || {
            load_ipc356_reports(&package_input_paths(&package_inputs.ipc356_files))
        })?;
        let ipc356_points = status_activity("collect IPC-D-356 points", || {
            Ok(ipc356_reports
                .iter()
                .flat_map(|report| report.points.iter())
                .cloned()
                .collect::<Vec<_>>())
        })?;
        let waivers = status_activity("load waivers", || load_waivers(&cli.waiver_files))?;

        status_activity("apply IPC-D-356 net annotations", || {
            for board in &mut boards {
                checks::apply_ipc356_nets(board, &ipc356_points, rules.ipc356_tolerance);
            }
            Ok(())
        })?;

        if cli.list_kicad_layers {
            status_activity("list KiCad layers", || {
                for board in &boards {
                    eprintln!("{}: {}", board.source, checks::layer_names_csv(board));
                }
                Ok(())
            })?;
        }

        status_activity("validate layer selections", || {
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
            Ok(())
        })?;

        let checks = status_activity("resolve selected checks", || {
            Ok(if cli.checks.is_empty() {
                DEFAULT_CHECKS.to_vec()
            } else {
                cli.checks.clone()
            })
        })?;
        let violations = run_checks(
            &checks,
            &config,
            &rules,
            &kicad_copper_layers,
            &cli,
            &layers,
            &boards,
            &excellon_reports,
            &excellon_drills,
            &ipc356_points,
            &package_inputs,
        )?;

        let (violations, waived) = status_activity("apply waivers", || {
            let (active_violations, waived) = waiver::apply_waivers(violations, &waivers);
            let mut violations = active_violations;
            violations.extend(waiver::governance_violations(&waivers));
            Ok((violations, waived))
        })?;
        let summary = status_activity("summarize findings", || {
            Ok(report_summary(&violations, waived.len()))
        })?;

        if let Some(summary_file) = &cli.summary_file {
            status_activity("write summary file", || {
                std::fs::write(summary_file, serde_json::to_vec_pretty(&summary)?)
                    .with_context(|| format!("failed to write {}", summary_file.display()))
            })?;
        }

        let report = status_activity("build report model", || {
            Ok(Report {
                files: cli
                    .files
                    .iter()
                    .chain(cli.gerber_dirs.iter())
                    .chain(cli.conversion_inputs.iter())
                    .chain(cli.kicad_pcbs.iter())
                    .chain(cli.waiver_files.iter())
                    .map(|path| path.display().to_string())
                    .chain(package_input_paths_flat(&package_inputs))
                    .collect(),
                inputs: input_manifest(&cli, &layers, &package_inputs),
                diagnostics: parser_diagnostics(&excellon_reports, &ipc356_reports),
                violation_count: violations.len(),
                waived_count: waived.len(),
                summary,
                violations,
            })
        })?;

        if let Some(svg_overlay) = &cli.svg_overlay {
            status_activity("write SVG overlay", || {
                std::fs::write(svg_overlay, svg_overlay::report_to_svg(&report))
                    .with_context(|| format!("failed to write {}", svg_overlay.display()))
            })?;
        }
        let current_baseline = status_activity("build baseline model", || {
            Ok(
                if cli.baseline_file.is_some()
                    || cli.baseline_reference.is_some()
                    || cli.baseline_diff_file.is_some()
                {
                    Some(baseline::report_to_baseline(&report))
                } else {
                    None
                },
            )
        })?;
        if let Some(waiver_stubs) = &cli.waiver_stubs {
            status_activity("write waiver stubs", || {
                std::fs::write(
                    waiver_stubs,
                    serde_json::to_vec_pretty(&baseline::report_to_waiver_stubs(&report))?,
                )
                .with_context(|| format!("failed to write {}", waiver_stubs.display()))
            })?;
        }
        if let Some(baseline_file) = &cli.baseline_file {
            status_activity("write baseline file", || {
                std::fs::write(
                    baseline_file,
                    serde_json::to_vec_pretty(
                        current_baseline.as_ref().expect(
                            "current baseline is created when baseline output is requested",
                        ),
                    )?,
                )
                .with_context(|| format!("failed to write {}", baseline_file.display()))
            })?;
        }
        status_activity("write baseline diff", || {
            match (&cli.baseline_reference, &cli.baseline_diff_file) {
                (Some(reference_path), Some(diff_path)) => {
                    let reference = baseline::load_baseline(reference_path)?;
                    let diff = baseline::compare_baselines(
                        &reference,
                        current_baseline
                            .as_ref()
                            .expect("current baseline is created when baseline diff is requested"),
                    );
                    std::fs::write(diff_path, serde_json::to_vec_pretty(&diff)?)
                        .with_context(|| format!("failed to write {}", diff_path.display()))?;
                }
                (None, Some(_)) => {
                    return Err(anyhow!(
                        "--baseline-diff-file requires --baseline-reference so hyperdrc has a previous finding set to compare"
                    ));
                }
                (Some(_), None) | (None, None) => {}
            }
            Ok(())
        })?;

        status_activity("emit report", || {
            match cli.format {
                OutputFormat::Text => print_text_report(&report),
                OutputFormat::Json => println!("{}", serde_json::to_string_pretty(&report)?),
                OutputFormat::Jsonl => print!("{}", jsonl::report_to_jsonl(&report)?),
                OutputFormat::Geojson => println!(
                    "{}",
                    serde_json::to_string_pretty(&report_to_geojson(&report))?
                ),
                OutputFormat::Sarif => println!(
                    "{}",
                    serde_json::to_string_pretty(&sarif::report_to_sarif(&report))?
                ),
                OutputFormat::GithubAnnotations => {
                    print!(
                        "{}",
                        github_annotations::report_to_github_annotations(&report)
                    );
                }
                OutputFormat::Html => print!("{}", html_report::report_to_html(&report)),
                OutputFormat::Junit => print!("{}", junit::report_to_junit(&report)),
            }
            Ok(())
        })?;

        Ok(report)
    })();

    match run_result {
        Ok(report) => {
            let elapsed = run_started.elapsed();
            eprintln!(
                "hyperdrc: finished run ({} active finding(s), {} waived, {:.3}s)",
                report.violation_count,
                report.waived_count,
                elapsed.as_secs_f64()
            );
            Ok(RunOutcome { report, elapsed })
        }
        Err(error) => {
            eprintln!(
                "hyperdrc: failed run after {:.3}s",
                run_started.elapsed().as_secs_f64()
            );
            Err(error)
        }
    }
}

/// Run HyperDRC as a command-line program.
///
/// This wrapper is intentionally thin: it delegates all work to [`run`] and then
/// converts active findings into the process exit status expected by CI.
pub fn run_cli(cli: Cli) -> Result<()> {
    let outcome = run(cli)?;
    if outcome.report.violation_count > 0 {
        std::process::exit(1);
    }
    Ok(())
}

fn status_activity<T>(activity: &str, work: impl FnOnce() -> Result<T>) -> Result<T> {
    let started = Instant::now();
    eprintln!("hyperdrc: starting {activity}");
    match work() {
        Ok(value) => {
            eprintln!(
                "hyperdrc: finished {activity} ({:.3}s)",
                started.elapsed().as_secs_f64()
            );
            Ok(value)
        }
        Err(error) => {
            eprintln!(
                "hyperdrc: failed {activity} after {:.3}s",
                started.elapsed().as_secs_f64()
            );
            Err(error)
        }
    }
}

fn progress_check_item(
    check_name: &str,
    item_index: usize,
    item_count: usize,
    item: &str,
    check_started: Instant,
) {
    eprintln!(
        "hyperdrc: check {check_name} progress {}/{} after {:.3}s: {item}",
        item_index + 1,
        item_count,
        check_started.elapsed().as_secs_f64()
    );
}

#[allow(clippy::too_many_arguments)]
fn run_checks(
    selected_checks: &[Check],
    config: &config::RuleConfig,
    rules: &EffectiveRules,
    kicad_copper_layers: &[String],
    cli: &Cli,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
    excellon_reports: &[excellon::ExcellonReport],
    excellon_drills: &[kicad::DrillFeature],
    ipc356_points: &[ipc356::Ipc356Point],
    package_inputs: &PackageInputs,
) -> Result<Vec<Violation>> {
    let mut violations = Vec::new();

    for check in selected_checks {
        let check_name = check_slug(*check);
        let check_started = std::time::Instant::now();
        let before_count = violations.len();
        eprintln!("hyperdrc: starting check {check_name}");

        let check_result: Result<()> = (|| {
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
                Check::ThermalPadPasteWindowpaneReadiness => {
                    for (paste_index, copper_index) in
                        explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                    {
                        let paste = &layers[paste_index];
                        let copper = &layers[copper_index];
                        violations.extend(checks::thermal_pad_paste_windowpane_readiness(
                            &layer_name(paste),
                            &paste.sketch,
                            &layer_name(copper),
                            &copper.sketch,
                            rules.min_area * 50.0,
                            rules.max_paste_area_ratio,
                            rules.min_area,
                        ));
                    }
                }
                Check::StencilAreaRatioReadiness => {
                    let paste_layers = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                        .into_iter()
                        .map(|(paste_index, _)| paste_index)
                        .collect::<std::collections::BTreeSet<_>>();
                    for paste_index in paste_layers {
                        let paste = &layers[paste_index];
                        violations.extend(checks::stencil_area_ratio_readiness(
                            &layer_name(paste),
                            &paste.sketch,
                            rules.stencil_thickness,
                            rules.min_stencil_area_ratio,
                            rules.min_area,
                        ));
                    }
                }
                Check::PasteApertureAspectRatioReadiness => {
                    let paste_layers = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                        .into_iter()
                        .map(|(paste_index, _)| paste_index)
                        .collect::<std::collections::BTreeSet<_>>();
                    for paste_index in paste_layers {
                        let paste = &layers[paste_index];
                        violations.extend(checks::paste_aperture_aspect_ratio_readiness(
                            &layer_name(paste),
                            &paste.sketch,
                            4.0,
                            rules.min_area,
                        ));
                    }
                }
                Check::TombstonePasteImbalanceReadiness => {
                    for (paste_index, copper_index) in
                        explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                    {
                        let paste = &layers[paste_index];
                        let copper = &layers[copper_index];
                        violations.extend(checks::tombstone_paste_imbalance_readiness(
                            &layer_name(paste),
                            &paste.sketch,
                            &layer_name(copper),
                            &copper.sketch,
                            rules.min_width * 8.0,
                            0.35,
                            rules.min_area,
                        ));
                    }
                }
                Check::PasteViaExposureReadiness => {
                    let paste_layers = explicit_layer_pairs(layers.len(), &cli.paste_pairs)?
                        .into_iter()
                        .map(|(paste_index, _)| paste_index)
                        .collect::<std::collections::BTreeSet<_>>();
                    for paste_index in paste_layers {
                        let paste = &layers[paste_index];
                        for board in boards {
                            violations.extend(checks::paste_via_exposure_readiness(
                                &layer_name(paste),
                                &paste.sketch,
                                board,
                                kicad_copper_layers,
                                rules.min_area,
                            ));
                        }
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
                    let item_count = paste_layers.len();
                    for (item_index, paste_index) in paste_layers.into_iter().enumerate() {
                        let paste = &layers[paste_index];
                        let name = layer_name(paste);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("checking aperture spacing on {name}"),
                            check_started,
                        );
                        violations.extend(checks::paste_aperture_spacing(
                            &name,
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
                    let silk_layers = selected_layers(layers.len(), &cli.silk_layers);
                    let item_count = silk_layers.len();
                    for (item_index, silk_index) in silk_layers.into_iter().enumerate() {
                        let silk = &layers[silk_index];
                        let name = layer_name(silk);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("opening silk layer {name}"),
                            check_started,
                        );
                        violations.extend(checks::silkscreen_min_width(
                            &name,
                            &silk.sketch,
                            rules.min_width,
                            rules.min_area,
                        ));
                    }
                }
                Check::MinCopperNeck => {
                    let gerber_copper_layers = selected_layers(layers.len(), &cli.copper_layers);
                    let gerber_copper_layer_count = gerber_copper_layers.len();
                    let item_count = gerber_copper_layer_count
                        + selected_kicad_copper_layer_count(boards, kicad_copper_layers);

                    for (item_index, copper_index) in gerber_copper_layers.into_iter().enumerate() {
                        let copper = &layers[copper_index];
                        let name = layer_name(copper);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("opening copper layer {name}"),
                            check_started,
                        );
                        violations.extend(checks::min_copper_neck_width(
                            &name,
                            &copper.sketch,
                            rules.min_width,
                            rules.min_area,
                        ));
                    }
                    let mut item_index = gerber_copper_layer_count;
                    for board in boards {
                        for (layer_name, copper) in board.copper_layers(kicad_copper_layers) {
                            let name = format!("{}:{layer_name}", board.source);
                            progress_check_item(
                                check_name,
                                item_index,
                                item_count,
                                &format!("opening copper layer {name}"),
                                check_started,
                            );
                            item_index += 1;
                            violations.extend(checks::min_copper_neck_width(
                                &name,
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
                    let mask_layers = selected_layers(layers.len(), &cli.mask_layers);
                    let item_count = mask_layers.len();
                    for (item_index, mask_index) in mask_layers.into_iter().enumerate() {
                        let mask = &layers[mask_index];
                        let name = layer_name(mask);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("opening mask layer {name}"),
                            check_started,
                        );
                        violations.extend(checks::solder_mask_sliver(
                            &name,
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
                    let item_count = cli.mask_layers.len();
                    for (item_index, mask_index) in cli.mask_layers.iter().enumerate() {
                        let mask = &layers[*mask_index];
                        let name = layer_name(mask);
                        progress_check_item(
                            check_name,
                            item_index,
                            item_count,
                            &format!("checking opening spacing on {name}"),
                            check_started,
                        );
                        violations.extend(checks::solder_mask_opening_spacing(
                            &name,
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
                    for (item_index, board) in boards.iter().enumerate() {
                        progress_check_item(
                            check_name,
                            item_index,
                            boards.len(),
                            &format!("checking {}", board.source),
                            check_started,
                        );
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
                Check::DifferentialPairReturnReadiness => {
                    for board in boards {
                        violations.extend(checks::differential_pair_return_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 2.0,
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
                        violations
                            .extend(checks::high_current_readiness(board, kicad_copper_layers));
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
                        violations
                            .extend(checks::power_plane_readiness(board, kicad_copper_layers));
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
                Check::RfViaFenceReadiness => {
                    for board in boards {
                        violations.extend(checks::rf_via_fence_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
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
                        violations
                            .extend(checks::gold_finger_readiness(board, kicad_copper_layers));
                    }
                }
                Check::GoldFingerEdgeReadiness => {
                    for board in boards {
                        violations.extend(checks::gold_finger_edge_readiness(
                            board,
                            kicad_copper_layers,
                            rules.clearance * 2.0,
                        ));
                    }
                }
                Check::GoldFingerSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::gold_finger_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::GoldFingerDrillKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::gold_finger_drill_keepout_readiness(
                            board,
                            excellon_drills,
                            kicad_copper_layers,
                            rules.clearance * 2.0,
                            rules.min_area,
                        ));
                    }
                }
                Check::ComponentEdgeClearanceReadiness => {
                    for board in boards {
                        violations.extend(checks::component_edge_clearance_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.component_edge_clearance,
                        ));
                    }
                }
                Check::ComponentHoleClearanceReadiness => {
                    for board in boards {
                        violations.extend(checks::component_hole_clearance_readiness(
                            board,
                            excellon_drills,
                            kicad_copper_layers,
                            rules.assembly.component_hole_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::ComponentSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::component_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.component_edge_clearance,
                            rules.assembly.connector_min_pad_dimension,
                        ));
                    }
                }
                Check::ConnectorReworkClearanceReadiness => {
                    for board in boards {
                        violations.extend(checks::connector_rework_clearance_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.connector_rework_clearance,
                            rules.assembly.connector_min_pad_dimension,
                        ));
                    }
                }
                Check::PadPairAsymmetryReadiness => {
                    for board in boards {
                        violations.extend(checks::pad_pair_asymmetry_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.pad_pair_max_gap,
                            rules.assembly.pad_pair_max_area_ratio,
                            rules.assembly.pad_pair_max_pad_dimension,
                        ));
                    }
                }
                Check::ConnectorReturnPathReadiness => {
                    for board in boards {
                        violations.extend(checks::connector_return_path_readiness(
                            board,
                            kicad_copper_layers,
                            rules.clearance * 2.0,
                            rules.net_clearance * 6.0,
                        ));
                    }
                }
                Check::DecouplingProximityReadiness => {
                    for board in boards {
                        violations.extend(checks::decoupling_proximity_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
                        ));
                    }
                }
                Check::EsdProtectionReadiness => {
                    for board in boards {
                        violations.extend(checks::esd_protection_readiness(
                            board,
                            kicad_copper_layers,
                            rules.clearance * 2.0,
                            rules.net_clearance * 8.0,
                        ));
                    }
                }
                Check::SwitchNodeKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::switch_node_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 4.0,
                            rules.min_area,
                        ));
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
                Check::TestpointAccessibilityReadiness => {
                    for board in boards {
                        violations.extend(checks::testpoint_accessibility_readiness(
                            board,
                            ipc356_points,
                            rules.assembly.testpoint_min_diameter,
                            rules.assembly.testpoint_min_spacing,
                            rules.assembly.testpoint_edge_clearance,
                        ));
                    }
                }
                Check::TestpointCopperClearanceReadiness => {
                    for board in boards {
                        violations.extend(checks::testpoint_copper_clearance_readiness(
                            board,
                            ipc356_points,
                            kicad_copper_layers,
                            rules.assembly.testpoint_min_diameter,
                            rules.assembly.testpoint_min_spacing,
                            rules.min_area,
                        ));
                    }
                }
                Check::ToolingHoleReadiness => {
                    for board in boards {
                        violations.extend(checks::tooling_hole_readiness(
                            board,
                            excellon_drills,
                            rules.assembly.tooling_min_diameter,
                            rules.assembly.tooling_max_diameter,
                            rules.assembly.tooling_edge_clearance,
                        ));
                    }
                }
                Check::MouseBiteReadiness => {
                    for board in boards {
                        violations.extend(checks::mouse_bite_readiness(
                            board,
                            excellon_drills,
                            rules.assembly.mouse_bite_min_diameter,
                            rules.assembly.mouse_bite_max_diameter,
                            rules.assembly.mouse_bite_min_spacing,
                            rules.assembly.mouse_bite_max_spacing,
                        ));
                    }
                }
                Check::FiducialReadiness => {
                    for board in boards {
                        violations.extend(checks::fiducial_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.fiducial_edge_clearance,
                        ));
                    }
                }
                Check::LocalFiducialReadiness => {
                    for board in boards {
                        violations.extend(checks::local_fiducial_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.local_fiducial_pitch,
                            rules.assembly.local_fiducial_search_radius,
                        ));
                    }
                }
                Check::FiducialKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::fiducial_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.fiducial_edge_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::DensePadEscapeReadiness => {
                    for board in boards {
                        violations.extend(checks::dense_pad_escape_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.dense_pad_pitch,
                            rules.assembly.dense_pad_via_search_radius,
                        ));
                    }
                }
                Check::SelectiveWaveSolderKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::selective_wave_solder_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules
                                .assembly
                                .selective_solder_keepout
                                .max(rules.assembly.wave_solder_keepout),
                            rules.min_area,
                        ));
                    }
                }
                Check::PressFitKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::press_fit_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.press_fit_keepout,
                            rules.min_area,
                        ));
                    }
                }
                Check::ConformalCoatingKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::conformal_coating_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.assembly.conformal_coating_keepout,
                            rules.min_area,
                        ));
                    }
                }
                Check::ThermalPadViaReadiness => {
                    for board in boards {
                        violations.extend(checks::thermal_pad_via_readiness(
                            board,
                            kicad_copper_layers,
                            rules.min_width * 12.0,
                        ));
                    }
                }
                Check::ThermalCopperAreaReadiness => {
                    for board in boards {
                        violations.extend(checks::thermal_copper_area_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 8.0,
                        ));
                    }
                }
                Check::HotComponentSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::hot_component_spacing_readiness(
                            board,
                            kicad_copper_layers,
                            rules.net_clearance * 6.0,
                            rules.min_area,
                        ));
                    }
                }
                Check::ThermalMechanicalKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::thermal_mechanical_keepout_readiness(
                            board,
                            excellon_drills,
                            kicad_copper_layers,
                            rules.clearance * 3.0,
                            rules.min_area,
                        ));
                    }
                }
                Check::MountingHoleGroundingReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_grounding_readiness(
                            board,
                            kicad_copper_layers,
                            rules.panel_clearance * 4.0,
                        ));
                    }
                }
                Check::MountingHoleCopperKeepoutReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_copper_keepout_readiness(
                            board,
                            kicad_copper_layers,
                            rules.panel_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::MountingHoleEdgeClearanceReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_edge_clearance_readiness(
                            board,
                            rules.panel_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::MountingHolePlatingIntentReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_plating_intent_readiness(
                            board,
                            kicad_copper_layers,
                            rules.panel_clearance * 4.0,
                        ));
                    }
                }
                Check::MountingHoleDistributionReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_distribution_readiness(
                            board,
                            rules.panel_clearance * 8.0,
                        ));
                    }
                }
                Check::MountingHoleSpacingReadiness => {
                    for board in boards {
                        violations.extend(checks::mounting_hole_spacing_readiness(
                            board,
                            rules.panel_clearance,
                        ));
                    }
                }
                Check::PanelFeatureOutlineReadiness => {
                    for board in boards {
                        violations.extend(checks::panel_feature_outline_readiness(
                            board,
                            rules.panel_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::EdgePlatingIntentReadiness => {
                    for board in boards {
                        violations.extend(checks::edge_plating_intent_readiness(
                            board,
                            kicad_copper_layers,
                            rules.panel_clearance,
                            rules.min_area,
                        ));
                    }
                }
                Check::CastellationPitchReadiness => {
                    for board in boards {
                        violations.extend(checks::castellation_pitch_readiness(
                            board,
                            rules.drill_clearance,
                        ));
                    }
                }
                Check::NetSpacing => {
                    for (item_index, board) in boards.iter().enumerate() {
                        progress_check_item(
                            check_name,
                            item_index,
                            boards.len(),
                            &format!("checking {}", board.source),
                            check_started,
                        );
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
                        cli,
                        rules,
                        layers,
                        boards,
                        package_inputs,
                    )));
                }
                Check::ExcellonReadiness => {
                    violations.extend(checks::excellon_batch_readiness(excellon_reports));
                }
                Check::ProductionArtifactReadiness => {
                    let bom_files =
                        load_text_artifacts(&package_input_paths(&package_inputs.bom_files))?;
                    let centroid_files =
                        load_text_artifacts(&package_input_paths(&package_inputs.centroid_files))?;
                    let netlist_files =
                        load_text_artifacts(&package_input_paths(&package_inputs.netlist_files))?;
                    let readme_files =
                        load_text_artifacts(&package_input_paths(&package_inputs.readme_files))?;
                    let fab_drawing_files = load_file_artifacts(&package_input_paths(
                        &package_inputs.fab_drawing_files,
                    ))?;
                    let assembly_drawing_files = load_file_artifacts(&package_input_paths(
                        &package_inputs.assembly_drawing_files,
                    ))?;
                    let rout_drawing_files = load_file_artifacts(&package_input_paths(
                        &package_inputs.rout_drawing_files,
                    ))?;
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
                Check::StackupReadiness => {
                    violations.extend(checks::stackup_readiness(config.stackup.as_ref(), boards));
                }
                Check::NetConstraintReadiness => {
                    violations.extend(checks::net_constraint_readiness(
                        &config.net_classes,
                        config.stackup.as_ref(),
                        boards,
                        kicad_copper_layers,
                    ));
                }
            }
            Ok(())
        })();

        let elapsed = check_started.elapsed();
        let added_count = violations.len().saturating_sub(before_count);
        match check_result {
            Ok(()) => {
                eprintln!(
                    "hyperdrc: finished check {check_name} ({added_count} finding(s), {:.3}s)",
                    elapsed.as_secs_f64()
                );
            }
            Err(error) => {
                eprintln!(
                    "hyperdrc: failed check {check_name} after {:.3}s ({added_count} finding(s) before failure)",
                    elapsed.as_secs_f64()
                );
                return Err(error);
            }
        }
    }

    Ok(violations)
}

fn check_slug(check: Check) -> &'static str {
    match check {
        Check::MaskIslandKeepout => "mask-island-keepout",
        Check::CopperOverlap => "copper-overlap",
        Check::BoardEdgeClearance => "board-edge-clearance",
        Check::BoardOutlineCutoutClearance => "board-outline-cutout-clearance",
        Check::BoardOutlineSanity => "board-outline-sanity",
        Check::BoardOutlineFragments => "board-outline-fragments",
        Check::BoardOutlineSelfIntersectionReadiness => "board-outline-self-intersection-readiness",
        Check::BoardOutlineNotchReadiness => "board-outline-notch-readiness",
        Check::BoardOutlineDuplicateReadiness => "board-outline-duplicate-readiness",
        Check::BoardOutlineNestingReadiness => "board-outline-nesting-readiness",
        Check::PasteOverhang => "paste-overhang",
        Check::PasteApertureCoverage => "paste-aperture-coverage",
        Check::PasteApertureRatio => "paste-aperture-ratio",
        Check::ThermalPadPasteWindowpaneReadiness => "thermal-pad-paste-windowpane-readiness",
        Check::StencilAreaRatioReadiness => "stencil-area-ratio-readiness",
        Check::PasteApertureAspectRatioReadiness => "paste-aperture-aspect-ratio-readiness",
        Check::TombstonePasteImbalanceReadiness => "tombstone-paste-imbalance-readiness",
        Check::PasteViaExposureReadiness => "paste-via-exposure-readiness",
        Check::MinimumPasteAperture => "minimum-paste-aperture",
        Check::PasteApertureSpacing => "paste-aperture-spacing",
        Check::PasteMaskAlignment => "paste-mask-alignment",
        Check::ExposedCopper => "exposed-copper",
        Check::SolderMaskOpeningCoverage => "solder-mask-opening-coverage",
        Check::SolderMaskExpansion => "solder-mask-expansion",
        Check::SolderMaskOverlapClearance => "solder-mask-overlap-clearance",
        Check::SolderMaskBoardEdgeClearance => "solder-mask-board-edge-clearance",
        Check::SilkscreenOverlap => "silkscreen-overlap",
        Check::SilkscreenClearance => "silkscreen-clearance",
        Check::SilkscreenBoardEdgeClearance => "silkscreen-board-edge-clearance",
        Check::SilkscreenMinWidth => "silkscreen-min-width",
        Check::MinCopperNeck => "min-copper-neck",
        Check::AcidTrap => "acid-trap",
        Check::LayerSanity => "layer-sanity",
        Check::CopperBalance => "copper-balance",
        Check::MechanicalLayerGeometry => "mechanical-layer-geometry",
        Check::SolderMaskSliver => "solder-mask-sliver",
        Check::MinimumMaskOpening => "minimum-mask-opening",
        Check::SolderMaskOpeningSpacing => "solder-mask-opening-spacing",
        Check::AnnularRing => "annular-ring",
        Check::AnnularRingTolerance => "annular-ring-tolerance",
        Check::PlatingIntent => "plating-intent",
        Check::RoutedSlotReadiness => "routed-slot-readiness",
        Check::CastellationIntent => "castellation-intent",
        Check::CastellationHoleReadiness => "castellation-hole-readiness",
        Check::ViaInPadReadiness => "via-in-pad-readiness",
        Check::DrillCopperClearance => "drill-copper-clearance",
        Check::BoardOutlineDrillClearance => "board-outline-drill-clearance",
        Check::DrillSpacing => "drill-spacing",
        Check::DrillAspectRatio => "drill-aspect-ratio",
        Check::DrillTableConsistency => "drill-table-consistency",
        Check::CopperWidthReadiness => "copper-width-readiness",
        Check::CopperNetIntent => "copper-net-intent",
        Check::TeardropReadiness => "teardrop-readiness",
        Check::ThermalReliefReadiness => "thermal-relief-readiness",
        Check::PlaneClearanceReadiness => "plane-clearance-readiness",
        Check::BoardEdgeExposure => "board-edge-exposure",
        Check::HighSpeedEdgeReadiness => "high-speed-edge-readiness",
        Check::EdgeCopperPullbackReadiness => "edge-copper-pullback-readiness",
        Check::HighVoltageEdgeReadiness => "high-voltage-edge-readiness",
        Check::ControlledImpedanceReadiness => "controlled-impedance-readiness",
        Check::DifferentialPairReadiness => "differential-pair-readiness",
        Check::DifferentialPairSpacingReadiness => "differential-pair-spacing-readiness",
        Check::DifferentialPairViaSymmetryReadiness => "differential-pair-via-symmetry-readiness",
        Check::DifferentialPairReturnReadiness => "differential-pair-return-readiness",
        Check::ReferencePlaneReadiness => "reference-plane-readiness",
        Check::ReferencePlaneVoidReadiness => "reference-plane-void-readiness",
        Check::OrphanedZoneReadiness => "orphaned-zone-readiness",
        Check::SameNetIslandReadiness => "same-net-island-readiness",
        Check::ReturnPathReadiness => "return-path-readiness",
        Check::HighCurrentReadiness => "high-current-readiness",
        Check::PowerViaArrayReadiness => "power-via-array-readiness",
        Check::ThermalViaReadiness => "thermal-via-readiness",
        Check::PowerPlaneReadiness => "power-plane-readiness",
        Check::HighCurrentNeckReadiness => "high-current-neck-readiness",
        Check::VoltageClearanceReadiness => "voltage-clearance-readiness",
        Check::SensitiveNetSpacingReadiness => "sensitive-net-spacing-readiness",
        Check::SensitiveReturnReadiness => "sensitive-return-readiness",
        Check::RfKeepoutReadiness => "rf-keepout-readiness",
        Check::RfViaFenceReadiness => "rf-via-fence-readiness",
        Check::ChassisStitchingReadiness => "chassis-stitching-readiness",
        Check::EdgeStitchingReadiness => "edge-stitching-readiness",
        Check::GoldFingerReadiness => "gold-finger-readiness",
        Check::GoldFingerEdgeReadiness => "gold-finger-edge-readiness",
        Check::GoldFingerSpacingReadiness => "gold-finger-spacing-readiness",
        Check::GoldFingerDrillKeepoutReadiness => "gold-finger-drill-keepout-readiness",
        Check::ComponentEdgeClearanceReadiness => "component-edge-clearance-readiness",
        Check::ComponentHoleClearanceReadiness => "component-hole-clearance-readiness",
        Check::ComponentSpacingReadiness => "component-spacing-readiness",
        Check::ConnectorReworkClearanceReadiness => "connector-rework-clearance-readiness",
        Check::PadPairAsymmetryReadiness => "pad-pair-asymmetry-readiness",
        Check::ConnectorReturnPathReadiness => "connector-return-path-readiness",
        Check::DecouplingProximityReadiness => "decoupling-proximity-readiness",
        Check::EsdProtectionReadiness => "esd-protection-readiness",
        Check::SwitchNodeKeepoutReadiness => "switch-node-keepout-readiness",
        Check::TestpointCoverageReadiness => "testpoint-coverage-readiness",
        Check::TestpointAccessibilityReadiness => "testpoint-accessibility-readiness",
        Check::TestpointCopperClearanceReadiness => "testpoint-copper-clearance-readiness",
        Check::ToolingHoleReadiness => "tooling-hole-readiness",
        Check::MouseBiteReadiness => "mouse-bite-readiness",
        Check::FiducialReadiness => "fiducial-readiness",
        Check::LocalFiducialReadiness => "local-fiducial-readiness",
        Check::FiducialKeepoutReadiness => "fiducial-keepout-readiness",
        Check::DensePadEscapeReadiness => "dense-pad-escape-readiness",
        Check::SelectiveWaveSolderKeepoutReadiness => "selective-wave-solder-keepout-readiness",
        Check::PressFitKeepoutReadiness => "press-fit-keepout-readiness",
        Check::ConformalCoatingKeepoutReadiness => "conformal-coating-keepout-readiness",
        Check::ThermalPadViaReadiness => "thermal-pad-via-readiness",
        Check::ThermalCopperAreaReadiness => "thermal-copper-area-readiness",
        Check::HotComponentSpacingReadiness => "hot-component-spacing-readiness",
        Check::ThermalMechanicalKeepoutReadiness => "thermal-mechanical-keepout-readiness",
        Check::MountingHoleGroundingReadiness => "mounting-hole-grounding-readiness",
        Check::MountingHoleCopperKeepoutReadiness => "mounting-hole-copper-keepout-readiness",
        Check::MountingHoleEdgeClearanceReadiness => "mounting-hole-edge-clearance-readiness",
        Check::MountingHolePlatingIntentReadiness => "mounting-hole-plating-intent-readiness",
        Check::MountingHoleDistributionReadiness => "mounting-hole-distribution-readiness",
        Check::MountingHoleSpacingReadiness => "mounting-hole-spacing-readiness",
        Check::PanelFeatureOutlineReadiness => "panel-feature-outline-readiness",
        Check::EdgePlatingIntentReadiness => "edge-plating-intent-readiness",
        Check::CastellationPitchReadiness => "castellation-pitch-readiness",
        Check::NetSpacing => "net-spacing",
        Check::RegistrationTolerance => "registration-tolerance",
        Check::PanelizationClearance => "panelization-clearance",
        Check::Ipc356Coverage => "ipc356-coverage",
        Check::Ipc356DrillDiameter => "ipc356-drill-diameter",
        Check::ExcellonReadiness => "excellon-readiness",
        Check::FileManifestReadiness => "file-manifest-readiness",
        Check::ProductionArtifactReadiness => "production-artifact-readiness",
        Check::StackupReadiness => "stackup-readiness",
        Check::NetConstraintReadiness => "net-constraint-readiness",
    }
}

fn package_inputs(cli: &Cli, discovered: io::PackageSidecars) -> PackageInputs {
    let mut inputs = PackageInputs::default();
    extend_package_inputs(
        &mut inputs.excellon_files,
        explicit_package_inputs(
            &cli.excellon_files,
            io::IoAdapter::Excellon,
            io::IoRole::DrillSidecar,
        ),
    );
    extend_package_inputs(
        &mut inputs.ipc356_files,
        explicit_package_inputs(
            &cli.ipc356_files,
            io::IoAdapter::Ipc356,
            io::IoRole::NetlistSidecar,
        ),
    );
    extend_package_inputs(
        &mut inputs.bom_files,
        explicit_package_inputs(
            &cli.bom_files,
            io::IoAdapter::DirectFile,
            io::IoRole::BomFile,
        ),
    );
    extend_package_inputs(
        &mut inputs.centroid_files,
        explicit_package_inputs(
            &cli.centroid_files,
            io::IoAdapter::DirectFile,
            io::IoRole::CentroidFile,
        ),
    );
    extend_package_inputs(
        &mut inputs.netlist_files,
        explicit_package_inputs(
            &cli.netlist_files,
            io::IoAdapter::DirectFile,
            io::IoRole::NetlistFile,
        ),
    );
    extend_package_inputs(
        &mut inputs.fab_drawing_files,
        explicit_package_inputs(
            &cli.fab_drawing_files,
            io::IoAdapter::DirectFile,
            io::IoRole::FabDrawing,
        ),
    );
    extend_package_inputs(
        &mut inputs.assembly_drawing_files,
        explicit_package_inputs(
            &cli.assembly_drawing_files,
            io::IoAdapter::DirectFile,
            io::IoRole::AssemblyDrawing,
        ),
    );
    extend_package_inputs(
        &mut inputs.readme_files,
        explicit_package_inputs(
            &cli.readme_files,
            io::IoAdapter::DirectFile,
            io::IoRole::ReadmeFile,
        ),
    );
    extend_package_inputs(
        &mut inputs.rout_drawing_files,
        explicit_package_inputs(
            &cli.rout_drawing_files,
            io::IoAdapter::DirectFile,
            io::IoRole::RoutDrawingFile,
        ),
    );

    extend_package_inputs(&mut inputs.excellon_files, discovered.excellon_files);
    extend_package_inputs(&mut inputs.ipc356_files, discovered.ipc356_files);
    extend_package_inputs(&mut inputs.bom_files, discovered.bom_files);
    extend_package_inputs(&mut inputs.centroid_files, discovered.centroid_files);
    extend_package_inputs(&mut inputs.netlist_files, discovered.netlist_files);
    extend_package_inputs(&mut inputs.fab_drawing_files, discovered.fab_drawing_files);
    extend_package_inputs(
        &mut inputs.assembly_drawing_files,
        discovered.assembly_drawing_files,
    );
    extend_package_inputs(&mut inputs.readme_files, discovered.readme_files);
    extend_package_inputs(
        &mut inputs.rout_drawing_files,
        discovered.rout_drawing_files,
    );
    inputs
}

fn explicit_package_inputs(
    paths: &[PathBuf],
    adapter: io::IoAdapter,
    role: io::IoRole,
) -> Vec<io::DiscoveredFile> {
    paths
        .iter()
        .cloned()
        .map(|path| io::DiscoveredFile {
            source: SourceRecord::new(
                adapter.clone(),
                role.clone(),
                &path,
                Option::<&std::path::Path>::None,
            ),
            path,
        })
        .collect()
}

fn extend_package_inputs(target: &mut Vec<io::DiscoveredFile>, inputs: Vec<io::DiscoveredFile>) {
    for input in inputs {
        if !target.iter().any(|existing| existing.path == input.path) {
            target.push(input);
        }
    }
}

fn package_input_paths(inputs: &[io::DiscoveredFile]) -> Vec<PathBuf> {
    inputs.iter().map(|input| input.path.clone()).collect()
}

fn package_input_paths_flat(inputs: &PackageInputs) -> impl Iterator<Item = String> + '_ {
    inputs
        .excellon_files
        .iter()
        .chain(inputs.ipc356_files.iter())
        .chain(inputs.bom_files.iter())
        .chain(inputs.centroid_files.iter())
        .chain(inputs.netlist_files.iter())
        .chain(inputs.fab_drawing_files.iter())
        .chain(inputs.assembly_drawing_files.iter())
        .chain(inputs.readme_files.iter())
        .chain(inputs.rout_drawing_files.iter())
        .map(|input| input.path.display().to_string())
}

fn package_sources(inputs: &PackageInputs) -> impl Iterator<Item = SourceRecord> + '_ {
    inputs
        .excellon_files
        .iter()
        .chain(inputs.ipc356_files.iter())
        .chain(inputs.bom_files.iter())
        .chain(inputs.centroid_files.iter())
        .chain(inputs.netlist_files.iter())
        .chain(inputs.fab_drawing_files.iter())
        .chain(inputs.assembly_drawing_files.iter())
        .chain(inputs.readme_files.iter())
        .chain(inputs.rout_drawing_files.iter())
        .map(|input| input.source.clone())
}

fn load_text_artifacts(files: &[PathBuf]) -> Result<Vec<checks::TextArtifact>> {
    files
        .iter()
        .map(|path| {
            let bytes = std::fs::read(path)
                .with_context(|| format!("failed to read {}", path.display()))?;
            // BOM and placement spreadsheets are often binary formats. Until a
            // typed spreadsheet adapter is added, keep the run non-fatal and let
            // production-artifact-readiness report missing table structure.
            let text = match String::from_utf8(bytes) {
                Ok(text) => text,
                Err(error) => String::from_utf8_lossy(error.as_bytes()).into_owned(),
            };
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

fn selected_kicad_copper_layer_count(
    boards: &[kicad::BoardModel],
    selected_layers: &[String],
) -> usize {
    boards
        .iter()
        .map(|board| {
            board
                .copper
                .iter()
                .filter(|feature| {
                    selected_layers.is_empty() || selected_layers.contains(&feature.layer)
                })
                .map(|feature| feature.layer.as_str())
                .collect::<std::collections::BTreeSet<_>>()
                .len()
        })
        .sum()
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

fn input_manifest(
    cli: &Cli,
    layers: &[Layer],
    package_inputs: &PackageInputs,
) -> Vec<SourceRecord> {
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
    inputs.extend(cli.waiver_files.iter().map(|path| {
        SourceRecord::new(
            io::IoAdapter::Waiver,
            io::IoRole::Waiver,
            path,
            Option::<&std::path::Path>::None,
        )
    }));
    inputs.extend(package_sources(package_inputs));
    inputs
}

fn manifest_input(
    cli: &Cli,
    rules: &EffectiveRules,
    layers: &[Layer],
    boards: &[kicad::BoardModel],
    package_inputs: &PackageInputs,
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
            .kicad_pcbs
            .iter()
            .map(|path| path.display().to_string())
            .chain(package_input_paths_flat(package_inputs))
            .collect(),
        bom_file_count: package_inputs.bom_files.len(),
        centroid_file_count: package_inputs.centroid_files.len(),
        netlist_file_count: package_inputs.netlist_files.len(),
        fab_drawing_file_count: package_inputs.fab_drawing_files.len(),
        assembly_drawing_file_count: package_inputs.assembly_drawing_files.len(),
        readme_file_count: package_inputs.readme_files.len(),
        rout_drawing_file_count: package_inputs.rout_drawing_files.len(),
        required_artifacts: checks::ManifestRequirements {
            bom: rules.required_artifacts.bom,
            centroid: rules.required_artifacts.centroid,
            netlist: rules.required_artifacts.netlist,
            fab_drawing: rules.required_artifacts.fab_drawing,
            assembly_drawing: rules.required_artifacts.assembly_drawing,
            readme: rules.required_artifacts.readme,
            rout_drawing: rules.required_artifacts.rout_drawing,
        },
        required_layers: checks::ManifestLayerRequirements {
            board_outline: rules.required_layers.board_outline,
            drill_data: rules.required_layers.drill_data,
            top_mask: rules.required_layers.top_mask,
            bottom_mask: rules.required_layers.bottom_mask,
            top_paste: rules.required_layers.top_paste,
            bottom_paste: rules.required_layers.bottom_paste,
            top_silkscreen: rules.required_layers.top_silkscreen,
            bottom_silkscreen: rules.required_layers.bottom_silkscreen,
        },
        declared_copper_layer_count: cli.declared_copper_layer_count.filter(|count| *count > 0),
        generated_date_stale_days: Some(rules.generated_date_stale_days).filter(|days| *days > 0),
        kicad_copper_layer_count: Some(kicad_copper_layers.len()).filter(|count| *count > 0),
        has_board_outline: boards.iter().any(|board| board.board_outline.is_some()),
        has_drill_data: !package_inputs.excellon_files.is_empty()
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

fn load_ipc356_reports(files: &[PathBuf]) -> Result<Vec<ipc356::Ipc356Report>> {
    let mut reports = Vec::new();
    for path in files {
        reports.push(ipc356::load_ipc356_report(path)?);
    }
    Ok(reports)
}

#[cfg(test)]
fn load_ipc356_points(files: &[PathBuf]) -> Result<Vec<ipc356::Ipc356Point>> {
    Ok(load_ipc356_reports(files)?
        .into_iter()
        .flat_map(|report| report.points.into_iter())
        .collect())
}

fn parser_diagnostics(
    excellon_reports: &[excellon::ExcellonReport],
    ipc356_reports: &[ipc356::Ipc356Report],
) -> Vec<Diagnostic> {
    let mut diagnostics = Vec::new();
    for report in excellon_reports {
        diagnostics.extend(report.issues.iter().map(|issue| Diagnostic {
            source: report.source.clone(),
            line: Some(issue.line),
            severity: checks::excellon_issue_severity(&issue.kind),
            code: excellon_issue_code(&issue.kind).to_string(),
            message: issue.message(),
        }));
    }
    for report in ipc356_reports {
        diagnostics.extend(report.issues.iter().map(|issue| Diagnostic {
            source: report.source.clone(),
            line: Some(issue.line),
            severity: Severity::Warning,
            code: ipc356_issue_code(&issue.kind).to_string(),
            message: issue.message(),
        }));
    }
    diagnostics
}

fn excellon_issue_code(kind: &excellon::ExcellonIssueKind) -> &'static str {
    match kind {
        excellon::ExcellonIssueKind::MissingUnitDeclaration => "excellon::missing-unit",
        excellon::ExcellonIssueKind::UnitConflict { .. } => "excellon::unit-conflict",
        excellon::ExcellonIssueKind::InvalidToolDefinition { .. } => {
            "excellon::invalid-tool-definition"
        }
        excellon::ExcellonIssueKind::ToolDiameterNotPositive { .. } => {
            "excellon::non-positive-tool-diameter"
        }
        excellon::ExcellonIssueKind::DuplicateToolDefinition { .. } => {
            "excellon::duplicate-tool-definition"
        }
        excellon::ExcellonIssueKind::ToolRedefinition { .. } => "excellon::tool-redefinition",
        excellon::ExcellonIssueKind::UnknownToolSelection { .. } => {
            "excellon::unknown-tool-selection"
        }
        excellon::ExcellonIssueKind::DrillHitWithoutActiveTool => {
            "excellon::hit-without-active-tool"
        }
        excellon::ExcellonIssueKind::DrillHitWithUnknownTool { .. } => {
            "excellon::hit-with-unknown-tool"
        }
        excellon::ExcellonIssueKind::DrillHitWithoutDiameter { .. } => {
            "excellon::hit-without-diameter"
        }
        excellon::ExcellonIssueKind::InvalidCoordinate { .. } => "excellon::invalid-coordinate",
    }
}

fn ipc356_issue_code(kind: &ipc356::Ipc356IssueKind) -> &'static str {
    match kind {
        ipc356::Ipc356IssueKind::MalformedTestRecord => "ipc356::malformed-test-record",
    }
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
        print_diagnostics(report);
        return;
    }

    println!(
        "{} violation(s) found, {} waived:",
        report.violation_count, report.waived_count
    );
    for (index, violation) in report.violations.iter().enumerate() {
        print_violation(index + 1, violation);
    }
    print_diagnostics(report);
}

fn print_diagnostics(report: &Report) {
    if report.diagnostics.is_empty() {
        return;
    }
    println!("{} parser diagnostic(s):", report.diagnostics.len());
    for diagnostic in &report.diagnostics {
        let line = diagnostic
            .line
            .map(|line| format!(":{line}"))
            .unwrap_or_default();
        println!(
            "   [{:?}] {}{} {}: {}",
            diagnostic.severity, diagnostic.source, line, diagnostic.code, diagnostic.message
        );
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
    use crate::config::{self, RuleOverrides};
    use crate::geometry::empty_sketch;
    use crate::io::{
        IoAdapter, IoRole, SourceRecord, discover_gerber_dir, discover_package_sidecars,
        is_gerber_path,
    };
    use crate::kicad;

    use super::{
        Layer, explicit_layer_pairs, input_manifest, layer_pairs, load_all_layers, load_boards,
        load_excellon_drills, load_ipc356_points, load_layers, load_text_artifacts, manifest_input,
        package_inputs, parser_diagnostics, run, validate_board_outline_role, validate_layer_index,
        validate_layer_indexes, validate_silk_layer_roles,
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

    fn default_rules() -> config::EffectiveRules {
        config::effective_rules(
            &config::RuleConfig::default(),
            RuleOverrides {
                keepout: None,
                clearance: None,
                paste_tolerance: None,
                min_paste_area_ratio: None,
                max_paste_area_ratio: None,
                stencil_thickness: None,
                min_stencil_area_ratio: None,
                min_width: None,
                min_mask_width: None,
                acid_trap_angle: None,
                max_copper_imbalance_ratio: None,
                annular_ring: None,
                drill_clearance: None,
                board_thickness: None,
                max_drill_aspect_ratio: None,
                net_clearance: None,
                registration_tolerance: None,
                panel_clearance: None,
                ipc356_tolerance: None,
                min_area: None,
                max_layer_area: None,
                generated_date_stale_days: None,
            },
        )
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
    fn parser_diagnostics_collect_excellon_and_ipc356_issues() {
        let excellon_report =
            crate::excellon::parse_excellon_report("T01\nXbadY0200\n", Path::new("panel.drl"));
        let ipc356_report =
            crate::ipc356::parse_ipc356_report("327 missing-coordinates\n", Path::new("board.ipc"));

        let diagnostics = parser_diagnostics(&[excellon_report], &[ipc356_report]);

        assert!(diagnostics.iter().any(|diagnostic| {
            diagnostic.code == "excellon::unknown-tool-selection"
                || diagnostic.code == "excellon::invalid-coordinate"
        }));
        assert!(
            diagnostics
                .iter()
                .any(|diagnostic| diagnostic.code == "ipc356::malformed-test-record")
        );
    }

    #[test]
    fn text_artifact_loader_keeps_binary_spreadsheets_non_fatal() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-binary-bom-{}.xlsx", std::process::id()));
        std::fs::write(&path, [0xff, 0xfe, b'B', b'O', b'M']).unwrap();

        let artifacts = load_text_artifacts(std::slice::from_ref(&path)).unwrap();

        assert_eq!(artifacts[0].path, path.display().to_string());
        assert!(artifacts[0].text.contains("BOM"));
        let _ = std::fs::remove_file(path);
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
        let package_inputs = package_inputs(&cli, Default::default());
        let manifest = input_manifest(&cli, &[], &package_inputs);
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
        let package_inputs = package_inputs(&cli, Default::default());
        let rules = default_rules();
        let manifest = manifest_input(&cli, &rules, &layers, &boards, &package_inputs);

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
        assert!(manifest.required_artifacts.bom);
        assert!(manifest.required_artifacts.centroid);
        assert!(manifest.required_artifacts.netlist);
        assert!(manifest.required_layers.board_outline);
        assert!(manifest.required_layers.drill_data);
        assert!(manifest.required_layers.top_mask);
        assert_eq!(manifest.declared_copper_layer_count, Some(4));
        assert_eq!(manifest.generated_date_stale_days, Some(90));
        assert_eq!(manifest.kicad_copper_layer_count, Some(2));
        assert!(manifest.has_board_outline);
        assert!(manifest.has_drill_data);
    }

    #[test]
    fn manifest_input_discards_non_positive_optional_counts_when_no_readiness_context() {
        let cli = Cli::parse_from(["hyperdrc", "--declared-copper-layer-count", "0", "top.gbr"]);
        let layers = vec![make_layer("top.gbr")];
        let boards = vec![make_board_model(&[], false, false)];
        let package_inputs = package_inputs(&cli, Default::default());
        let rules = default_rules();
        let manifest = manifest_input(&cli, &rules, &layers, &boards, &package_inputs);

        assert_eq!(manifest.gerber_layers.len(), 1);
        assert_eq!(manifest.declared_copper_layer_count, None);
        assert_eq!(manifest.kicad_copper_layer_count, None);
        assert!(!manifest.has_board_outline);
        assert!(!manifest.has_drill_data);
    }

    #[test]
    fn gerber_directory_sidecars_feed_manifest_and_provenance() {
        let dir =
            std::env::temp_dir().join(format!("hyperdrc-package-sidecars-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("board.drl"),
            "M48\nMETRIC\nT01C0.6\n%\nT01\nX0Y0\nM30\n",
        )
        .unwrap();
        std::fs::write(dir.join("release_bom.csv"), "Ref,MPN\nR1,RC0603\n").unwrap();
        std::fs::write(dir.join("README.md"), "Revision A fabrication notes\n").unwrap();

        let cli = Cli::parse_from(["hyperdrc", "--gerber-dir", dir.to_str().unwrap()]);
        let discovered = discover_package_sidecars(std::slice::from_ref(&dir)).unwrap();
        let package_inputs = package_inputs(&cli, discovered);
        let sources = input_manifest(&cli, &[], &package_inputs);
        let rules = default_rules();
        let manifest = manifest_input(&cli, &rules, &[], &[], &package_inputs);

        assert_eq!(package_inputs.excellon_files.len(), 1);
        assert_eq!(manifest.bom_file_count, 1);
        assert_eq!(manifest.readme_file_count, 1);
        assert!(manifest.has_drill_data);
        assert!(sources.iter().any(|source| {
            source.role == IoRole::BomFile
                && source.adapter == IoAdapter::GerberDirectory
                && source.origin.as_deref() == Some(dir.to_str().unwrap())
        }));

        let _ = std::fs::remove_dir_all(dir);
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
